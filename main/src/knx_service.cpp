// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "board.h"
#include "control_core.hpp"
#include "control_service.hpp"
#include "control_state.h"
#include "device_identity.hpp"
#include "device_secret.hpp"
#include "hvac_control.hpp"
#include "knx_dpt9.hpp"
#include "knx_product.hpp"
#include "protocol_adapter.h"
#include "sensor_fusion_service.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include "knx/platform/esp32_platform.hpp"
#include "knx/product/commissioned_product.hpp"
#include "knx/util/log.hpp"

#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
#include "device_config.h"
#include "device_default_name.h"
#include "knx_ip_addr.h"
#include "esp_netif.h"
#include "knx/netip/netip_parameter_object.hpp"
#include "wifi_service.h"
#include <string>
#else
#include "knx/physical/bitbang_driver_timer_isr_espidf.hpp"
#include "knx/physical/physical_factory.hpp"
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <span>
#include <tuple>
#include <type_traits>

using namespace knx;
using namespace knx::application;
using namespace knx::product;
using namespace habinari_knx;

namespace {

static constexpr const char *TAG = "knx_service";
// Startup, not the service loop, sets this task's stack peak. The commissioned
// runtime is ~53 KB for this product and is built directly on the heap by
// startCommissionedProduct(), so it never crosses this stack; what remains here
// is the bindings builder (~8.9 KB) and the BAU init call tree with
// NVS/crypto/logging under it.
//
// Everything on that path that is sized by the port and parameter counts —
// the builder, the endpoint bindings, the parameter callbacks — is now handed
// down by rvalue reference rather than by value, so it exists once instead of
// once per frame it passes through. Measured frames before that change, at 61
// ports / 73 parameters: knxServiceTask 19,040 B + startCommissionedProduct
// 96 B + make_unique 9,008 B + the runtime constructor 5,920 B = 34,064 B,
// which overflowed a 32 KB stack during boot (the C6 hardware stack guard
// trapped it inside the constructor prologue).
//
// The high-water mark is logged once below; retune this from that number rather
// than from guesswork if the product grows.
static constexpr uint32_t kKnxServiceTaskStackSize = 32 * 1024;

// How long knxAdapterStart() waits for the service task to report whether the
// product started. Start-up itself (NVS load, BAU init) takes well under a
// second. The KNXnet/IP wait for Wi-Fi is deliberately not part of the
// allowance: app_main starts the BLE service channel after the adapters, and
// that channel is how a device with wrong Wi-Fi credentials gets corrected.
// Holding it back for the whole Wi-Fi wait would delay the repair by exactly
// the outage it is meant to repair.
static constexpr uint32_t kKnxStartTimeoutMs = 5000u;

// --- Bus-citizenship: boot-storm mitigation -------------------------------
// On a bus-wide power-up (line powered on) every device would otherwise dump
// its full object set at t=0, swamping the shared ~50 telegram/s TP1 medium —
// worse the more of these boards share a line. Two independent guards:
//   1. A per-device random startup delay before the first full publish, so
//      devices on the same line desynchronise instead of all firing at once.
//   2. A global outbound telegram rate limit. The stack defers over-budget
//      sends (coalescing each object to its latest value) and drains them from
//      loop(), so even the initial burst and later change storms are spread
//      over time with nothing dropped. Requires a monotonic time source, which
//      also enables the cyclic status heartbeats (previously never armed).
static constexpr uint32_t kStartupPublishMaxJitterMs = 8000;
static constexpr uint32_t kMaxUnsolicitedTelegramsPerWindow = 5;
static constexpr uint32_t kTelegramRateWindowMs = 1000;

// --- KNX Data Secure ------------------------------------------------------
// The device's FDSK / initial tool key is derived from a 256-bit root secret
// held in a read-protected eFuse key block (see device_secret.hpp), so it
// survives a factory reset and the same firmware image still gives every board
// a different key. It is logged every boot, together with the serial number
// and the ETS device certificate, so an installer can capture it locally for
// secure commissioning.
//
// Until that eFuse block is provisioned — which is deliberately off by default
// while the dry run is being checked on hardware — the device falls back to
// the legacy behaviour: a random 16-byte key generated on first boot and kept
// in NVS. That key does NOT survive knx_service_reset_nvm(), which erases the
// whole default NVS partition, so a factory reset regenerates it and the
// previously printed certificate stops matching the device.
//
// Data Secure is always on in this firmware; the ETS catalogue entry says so
// too (SecurityPolicy in knx_product.hpp), and the two must agree.
static constexpr const char *kSecureNvsNamespace = "knx_secure";
static constexpr const char *kSecureNvsToolKeyBlob = "tool_key";

// Hex, base32, CRC-4 and ETS device-certificate rendering now live in
// device_identity.hpp so the root-secret dry run can print the same
// certificate and the encoding is covered by host tests.
using habinari::identity::formatDeviceCertificate;
using habinari::identity::toHex;

// Load the persisted KNX Data Secure tool key, generating and storing a fresh
// random one on first boot. Returns true on success; `created` reports whether
// a new key was generated this call.
bool loadOrCreateToolKey(std::array<uint8_t, 16> &key, bool &created)
{
    created = false;
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kSecureNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Data Secure: nvs_open failed: %s", esp_err_to_name(err));
        return false;
    }

    size_t len = key.size();
    err = nvs_get_blob(handle, kSecureNvsToolKeyBlob, key.data(), &len);
    if (err == ESP_ERR_NVS_NOT_FOUND || (err == ESP_OK && len != key.size())) {
        esp_fill_random(key.data(), key.size());
        err = nvs_set_blob(handle, kSecureNvsToolKeyBlob, key.data(), key.size());
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        created = true;
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Data Secure: tool key %s failed: %s",
                 created ? "generation" : "load", esp_err_to_name(err));
        return false;
    }
    return true;
}

// Where the factory tool key came from. The two differ in shelf life: the
// eFuse-derived FDSK is fixed for the life of the chip, while the NVS key is
// erased by a factory reset.
enum class ToolKeySource { EfuseDerived, NvsFallback };

// Resolve the device's factory tool key, preferring the FDSK derived from the
// eFuse root secret and falling back to the legacy NVS key while that block is
// still unprovisioned. `serial` is null when the MAC read failed, which rules
// out the derived path (the serial is part of the KDF message).
bool resolveFactoryToolKey(const uint8_t *serial,
                           std::array<uint8_t, 16> &key,
                           ToolKeySource &source,
                           bool &created)
{
    created = false;
    if (serial != nullptr) {
        const auto resolved =
            habinari::secret::resolveFdsk(std::span<const uint8_t, 6>(serial, 6));
        if (resolved.fdskValid) {
            key = resolved.fdsk;
            source = ToolKeySource::EfuseDerived;
            return true;
        }
    }
    source = ToolKeySource::NvsFallback;
    return loadOrCreateToolKey(key, created);
}

namespace hvac_ns = habinari::hvac;

// The device's own state — settings, installation inputs, control outputs —
// lives in control_service.cpp now, not here. This adapter reads and writes it
// through the same handle every other adapter uses; the only thing it still
// owns privately is its task.
//
// Binding a reference is safe before control_service_start(): state() returns a
// constant-initialised namespace-scope object, and `mutex` being null is the
// documented "service not running yet" case that LockGuard tolerates.
namespace control = habinari::control;
using control::LockGuard;
using Settings = control::Settings;
using Outputs = control::Outputs;

auto &g_state = control::state();

// What only the KNX personality has.
struct KnxAdapterState {
    // Written by the service task as its first act, so that anything able to
    // notify it — the control tick, the stack's work callback, the Wi-Fi event
    // — finds the handle already there. Read from those other tasks, hence
    // atomic.
    std::atomic<TaskHandle_t> taskHandle{nullptr};
    bool started{false};
    // The task waiting in knxAdapterStart() for the start-up result, or null
    // once nobody is. Taken with an exchange by whichever side gets there
    // first: the service task to report, or knxAdapterStart() giving up. A
    // report can then never be sent to an app_main that has already returned
    // and been deleted.
    std::atomic<TaskHandle_t> startWaiter{nullptr};
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
    // True while startProduct() waits for Wi-Fi. knxAdapterStart() stops
    // waiting for a start-up that is only waiting for the network.
    std::atomic<bool> waitingForNetwork{false};
#endif
    // Raised by the control task through the adapter's on_control_tick hook,
    // consumed by the service loop with an exchange, so a tick raised between
    // the read and the clear is never lost.
    std::atomic<bool> controlTickPending{true};
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
    // Raised from the Wi-Fi event loop when the interface gains an address, so
    // the service task republishes what ETS reads back. Consumed the same way
    // as controlTickPending above.
    std::atomic<bool> ipIdentityDirty{false};
    // True when the routing socket opened with no interface to join the group
    // on. Set before the socket exists and read once the event callback is
    // installed, to close the gap between the two: an interface that arrived in
    // between leaves no event to fire and would strand the device off the group.
    bool startedWithoutNetwork{false};
#endif
};

KnxAdapterState g_knx;

// The per-port "has this changed?" bookkeeping this snapshot used to carry is
// gone: the stack's GroupObjectTransmitPolicy already decides Send / Defer /
// Suppress per object from the COV delta, the minimum repetition time and the
// heartbeat (see applyTransmitPolicies), which is the same model the KNX sensor
// FBs specify. So the loop publishes every port every tick and lets the policy
// layer drop what has not moved — one place deciding, instead of two that could
// disagree.
struct PublishSnapshot {
    sensor_data_t sensorData{};
    Settings settings{};
    Outputs out{};
    hvac_ns::OperatingPreset hvacOperatingMode{hvac_ns::OperatingPreset::Comfort};
    hvac_ns::ControllerMode controllerMode{hvac_ns::ControllerMode::Auto};
    hvac_ns::VentilationMode ventilationMode{hvac_ns::VentilationMode::Auto};
    bool controllerOnOff{true};
};

// A port the ETS project left unlinked publishes as a successful no-op, so
// nothing here fires for unused datapoints; see reportUnlinkedPorts for the
// one-shot inventory of those. Anything that does reach this log is a genuine
// send failure, printed by name because the bare enum value was unreadable.
template <typename AppT>
void publishPort(AppT &app, HabinariPort port, float value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %s", label, util::errorCodeToString(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, HabinariPort port, bool value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %s", label, util::errorCodeToString(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, HabinariPort port, uint8_t value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %s", label, util::errorCodeToString(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, HabinariPort port, uint16_t value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %s", label, util::errorCodeToString(result.error()));
    }
}

// One-shot inventory of transmitting communication objects that the ETS project
// linked no group address to. Publishing on them is a silent no-op, so without
// this the integrator has no way to tell "configured off" from "broken". Logged
// once per transition into Operational (i.e. after every download).
template <typename AppT>
void reportUnlinkedPorts(AppT &app)
{
    const auto &compiled = app.compiledEndpoint();
    const auto &runtimeObjects = compiled.runtime.communicationObjects;
    const auto &exportObjects = compiled.exportDescriptor.communicationObjects;

    unsigned unlinked = 0;
    for (size_t slot = 0; slot < runtimeObjects.size(); ++slot) {
        const auto &descriptor = runtimeObjects[slot];
        if (!descriptor.transmit || app.isPortLinked(descriptor.logicalId)) {
            continue;
        }
        ++unlinked;
        KNX_LOGI(TAG,
                 "CO #%u \"%.*s\" has no group address - not transmitted",
                 static_cast<unsigned>(exportObjects[slot].exportNumber),
                 static_cast<int>(exportObjects[slot].displayName.size()),
                 exportObjects[slot].displayName.data());
    }

    if (unlinked == 0) {
        KNX_LOGI(TAG, "All transmitting communication objects are linked");
    } else {
        KNX_LOGI(TAG, "%u transmitting communication object(s) unlinked in this project", unlinked);
    }
}

// Push the send-on-change / minimum-repetition / cyclic-heartbeat transmit
// policy (see knx/application/group_object.hpp) to every transmitting port.
//
// This is the device's implementation of the KNX sensor Functional Block
// sending model (Vol 7/19/20 clause 3.x: Heartbeat, MinRepTime, COVCondition):
// the ETS parameters name those three quantities directly, and the stack turns
// them into Send / Defer / Suppress decisions. The publish path therefore does
// not filter anything itself.
//
// Cheap and idempotent, so it is simply re-applied every control tick (~1 Hz)
// rather than only on onParameterChanged, which keeps it in step with the
// "reconfigure-from-settings-every-tick" pattern used for the controllers.
// Write-only CommandPorts never transmit and are intentionally absent.
template <typename AppT>
void applyTransmitPolicies(AppT &app, const Settings &settings)
{
    const uint32_t heartbeatMs = settings.heartbeatSeconds * 1000u;
    const uint32_t minRepMs = settings.minRepTimeSeconds * 1000u;

    const auto covPolicy = [&](float threshold) {
        GroupObjectTransmitPolicy policy{};
        policy.onChangeEnabled = true;
        policy.changeThreshold = static_cast<double>(threshold);
        policy.cyclicIntervalMs = heartbeatMs;
        policy.minIntervalMs = minRepMs;
        return policy;
    };

    // Analogue measurements and derived values: a per-measurand COV delta, since
    // 0.2 is a sensible step in K and a meaningless one in ppm.
    struct CovPort {
        HabinariPort port;
        float threshold;
    };
    const CovPort covPorts[] = {
        {HabinariPort::RoomTemperature, settings.roomTemperatureCovK},
        {HabinariPort::RoomHumidity, settings.roomHumidityCovPct},
        {HabinariPort::RoomCo2, settings.co2CovPpm},
        {HabinariPort::RoomAirPressure, settings.pressureCovPa},
        {HabinariPort::RoomAirPressureSeaLevel, settings.pressureCovPa},
        {HabinariPort::RoomAirQualityIndex, settings.airQualityCovIndex},
        {HabinariPort::RoomCo2Equivalent, settings.co2CovPpm},
        {HabinariPort::RoomVocEquivalent, settings.co2CovPpm},
        {HabinariPort::RoomDewPoint, settings.derivedCov},
        {HabinariPort::RoomAbsoluteHumidity, settings.derivedCov},
        {HabinariPort::FloorTemperature, settings.floorTemperatureCovK},
        {HabinariPort::FloorHumidity, settings.floorHumidityCovPct},
        {HabinariPort::FloorAbsoluteHumidity, settings.derivedCov},
        {HabinariPort::DewPointMargin, settings.derivedCov},
        {HabinariPort::SetpointBase, settings.roomTemperatureCovK},
        {HabinariPort::SetpointStatus, settings.roomTemperatureCovK},
        {HabinariPort::SetpointHeatingStatus, settings.roomTemperatureCovK},
        {HabinariPort::SetpointCoolingStatus, settings.roomTemperatureCovK},
        {HabinariPort::SetpointShiftStatus, settings.roomTemperatureCovK},
        {HabinariPort::Co2Setpoint, settings.co2CovPpm},
        // The trend is published in K/h, so its COV threshold is the room
        // temperature COV per hour rather than per degree.
        {HabinariPort::TemperatureTrend, settings.roomTemperatureCovK},
    };
    for (const auto &entry : covPorts) {
        (void)app.setTransmitPolicy(entry.port, covPolicy(entry.threshold));
    }

    // Discrete states, enumerations and status words: any change is meaningful,
    // so only the heartbeat and the minimum repetition time apply.
    GroupObjectTransmitPolicy discrete{};
    discrete.onChangeEnabled = true;
    discrete.changeThreshold = 0.0;  // <= 0 with onChange means "any change"
    discrete.cyclicIntervalMs = heartbeatMs;
    discrete.minIntervalMs = minRepMs;

    static constexpr HabinariPort kDiscretePorts[] = {
        HabinariPort::AirQualityAccuracy,
        HabinariPort::FloorMoistureAlarm,
        HabinariPort::FloorLimitActive,
        HabinariPort::FloorComfortActive,
        HabinariPort::DewPointAlarm,
        HabinariPort::FreeCoolingAvailable,
        HabinariPort::FreeDryingAvailable,
        HabinariPort::ControllerOnOff,
        HabinariPort::HvacModeStatus,
        HabinariPort::ContrModeStatus,
        HabinariPort::ContrModeSecondary,
        HabinariPort::HeatingControlValue,
        HabinariPort::CoolingControlValue,
        HabinariPort::HeatingRequest,
        HabinariPort::CoolingRequest,
        HabinariPort::HeatCoolModeStatus,
        HabinariPort::EnableHeatStatus,
        HabinariPort::EnableCoolStatus,
        HabinariPort::ControllerStatus,
        HabinariPort::VentilationDemand,
        HabinariPort::VentilationStage,
        HabinariPort::VentilationMode,
        HabinariPort::VentilationBoostRequest,
        HabinariPort::DehumidifyRequest,
        HabinariPort::AirQualityStatus,
        HabinariPort::RoomSensorStatus,
        HabinariPort::FloorProbeStatus,
        HabinariPort::AirQualitySensorStatus,
        HabinariPort::SensorHealthMask,
        HabinariPort::SensorDisagreementAlarm,
        HabinariPort::OccupancyDetected,
        HabinariPort::EstimatedOccupants,
        HabinariPort::WindowOpenDetected,
    };
    for (const auto port : kDiscretePorts) {
        (void)app.setTransmitPolicy(port, discrete);
    }

    // Alarms are exempt from the minimum repetition time. That floor exists to
    // keep a noisy measurement from flooding the bus, and applying it to a fire
    // alarm would delay it by up to that many seconds for no benefit — an alarm
    // that has changed state is exactly the telegram worth sending at once.
    GroupObjectTransmitPolicy alarm = discrete;
    alarm.minIntervalMs = 0;
    static constexpr HabinariPort kAlarmPorts[] = {
        HabinariPort::FireAlarm,
        HabinariPort::FirePreAlarm,
        HabinariPort::DeviceFault,
    };
    for (const auto port : kAlarmPorts) {
        (void)app.setTransmitPolicy(port, alarm);
    }
}


// ---------------------------------------------------------------------------
// The transport, and the only thing about this device that differs by medium.
//
// Everything else in this file — every group object, every parameter callback,
// the publish policy, programming mode — is the same code on TP1 and on
// KNXnet/IP. The two startProduct() overloads below are the seam: each brings
// its own transport up and hands back the identical started product handle, so
// nothing downstream of them knows which medium it is running on.
// ---------------------------------------------------------------------------

#if !defined(CONFIG_HABINARI_KNX_MEDIUM_IP)

util::Result<std::unique_ptr<physical::Tp1MacPhysical>> createTp1Physical(platform::Esp32Platform &platform)
{
    physical::Tp1BackendSelection selection{};

#if defined(CONFIG_KNX_TP1_TPUART)
    selection.family = physical::Tp1BackendFamily::Tpuart;
#elif defined(CONFIG_KNX_TP1_BITBANG)
    selection.family = physical::Tp1BackendFamily::Bitbang;
#else
    return util::ErrorCode::OperationNotSupported;
#endif

    physical::Tp1PlatformDependencies dependencies{};
    dependencies.platform = &platform;
    dependencies.uart = platform.uart();

#if defined(CONFIG_KNX_TP1_BITBANG)
    static physical::BitBangDriverTimerIsrEspIdf bitbangDriver;
    dependencies.bitbangDriver = &bitbangDriver;
    dependencies.bitbangTp1Driver = &bitbangDriver;
#endif

    auto physicalResult = physical::createTp1PhysicalForPlatform(selection, dependencies);
    if (physicalResult.isOk() && physicalResult.value()) {
        // STKNX drives PIN_KNX_OK (GPIO3) while the bus supply is good. The
        // stack uses it to stop transmitting into a dead bus; here it is only
        // reported, so a field log shows bus outages and returns explicitly
        // rather than as a burst of unacknowledged transmissions.
        physicalResult.value()->setLinkStateCallback(
            [](const physical::LinkStatus &status, void *) {
                if (status.state == physical::LinkState::Down) {
                    KNX_LOGW(TAG, "KNX bus power lost — transmission suspended");
                } else {
                    KNX_LOGI(TAG, "KNX bus power %s", physical::linkStateToString(status.state));
                }
            },
            nullptr);
    }

    return physicalResult;
}

#endif  // !CONFIG_HABINARI_KNX_MEDIUM_IP

using HabinariProductHandle =
    CommissionedProductHandle<std::remove_cvref_t<decltype(kHabinariProduct)>,
                              kDefaultBindingCapacity>;
using HabinariBindings =
    CommissionedBindingsBuilder<std::remove_cvref_t<decltype(kHabinariProduct)>,
                                kDefaultBindingCapacity>;

#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)

// --- KNXnet/IP settings ----------------------------------------------------
//
// The routing group is the one KNXnet/IP setting an installation genuinely
// changes: two KNX projects sharing an IP network have to be told apart, and
// the group is how. It is a device_config item rather than a Kconfig-only
// value because that change happens on site, and the service channel is how a
// device that has not joined the right group yet can still be reached.
//
// Everything else about the device — the individual address, the group object
// bindings, the parameters — is ETS's, and does not belong here.
constexpr const char *kKnxIpNvsNamespace = "knxip";
constexpr const char *kKnxIpNvsMulticast = "mcast";

esp_err_t knxIpConfigGet(const device_config_item_t *item, device_config_value_t *out)
{
    out->str[0] = '\0';

    nvs_handle_t handle = 0;
    if (nvs_open(kKnxIpNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
        // Nothing stored yet is a valid answer: the compiled-in default is in
        // force, and resolveMulticastGroup() reports which that is.
        return ESP_OK;
    }
    size_t len = sizeof(out->str);
    if (nvs_get_str(handle, static_cast<const char *>(item->ctx), out->str, &len) != ESP_OK) {
        out->str[0] = '\0';
    }
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t knxIpConfigSet(const device_config_item_t *item, const device_config_value_t *value)
{
    // Refuse a group the device could not join rather than storing it and
    // failing to come up after the reboot this setting asks for: the group is
    // joined at start-up, so a bad one is a device that never appears again.
    // An empty value is not a bad one — it means "use the compiled-in default".
    if (value->str[0] != '\0' && !knx_ip_is_multicast_text(value->str)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kKnxIpNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, static_cast<const char *>(item->ctx), value->str);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

const device_config_item_t kKnxIpConfigItems[] = {
    {
        .key = "knx.mcast",
        .label = "KNX routing multicast group",
        .unit = nullptr,
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .min = 0.0f,
        .max = 15.0f,  // "255.255.255.255"
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = knxIpConfigGet,
        .set = knxIpConfigSet,
        .ctx = const_cast<char *>(kKnxIpNvsMulticast),
    },
};

// A registered item's current string value, empty when unset.
bool readConfigString(const char *key, char *out, size_t outLen)
{
    out[0] = '\0';
    const device_config_item_t *item = device_config_find(key);
    if (item == nullptr) {
        return false;
    }
    device_config_value_t value{};
    if (device_config_get(item, &value) != ESP_OK || value.str[0] == '\0') {
        return false;
    }
    snprintf(out, outLen, "%s", value.str);
    return true;
}

// The device name an installer set, or the board's default. It is what ETS
// shows in a discovery list, so it should name this unit rather than the model.
std::string resolveFriendlyName()
{
    char name[DEVICE_CONFIG_STRING_MAX] = {0};
    if (readConfigString("dev.name", name, sizeof(name))) {
        return std::string(name);
    }

    // The same fallback every other channel shows: "<prefix> XXYYZZ" from the
    // last three MAC bytes, so a board nobody has named still appears in ETS
    // under something that tells two of them apart.
    uint8_t mac[6] = {};
    if (wifi_service_mac(mac) != ESP_OK) {
        return std::string();
    }
    // 30 octets is the KNXnet/IP friendly-name field; the encoder pads and the
    // BAU trims, so this only has to be big enough not to truncate first.
    char fallback[32] = {0};
    device_default_name(&mac[3], fallback, sizeof(fallback));
    return std::string(fallback);
}

IpAddress resolveMulticastGroup()
{
    char configured[DEVICE_CONFIG_STRING_MAX] = {0};
    if (readConfigString("knx.mcast", configured, sizeof(configured))) {
        const IpAddress parsed = IpAddress::fromString(configured);
        if (!parsed.isZero()) {
            return parsed;
        }
        KNX_LOGW(TAG, "Ignoring unparseable routing group '%s'", configured);
    }
    return IpAddress::fromString(CONFIG_HABINARI_KNXIP_MULTICAST_ADDR);
}

// Publish what the device currently answers on into the KNXnet/IP Parameter
// Object. ETS reads its address, its name and its multicast group from there,
// and none of it is knowable before the interface has a lease — so this runs
// after start and again whenever the address changes.
template <typename AppT>
void publishIpIdentity(AppT &app, IpAddress multicastGroup)
{
    esp_netif_ip_info_t info{};
    if (wifi_service_ip_info(&info) != ESP_OK || info.ip.addr == 0) {
        KNX_LOGD(TAG, "No address yet — KNXnet/IP parameters not published");
        return;
    }

    netip::KnxNetIpConfiguration config{};
    config.currentIpAddress = IpAddress(info.ip.addr);
    config.currentSubnetMask = IpAddress(info.netmask.addr);
    config.currentDefaultGateway = IpAddress(info.gw.addr);
    config.routingMulticastAddress = multicastGroup;
    config.timeToLive = static_cast<uint8_t>(CONFIG_HABINARI_KNXIP_TTL);
    config.ipAssignmentMethod = netip::ip_assignment::kDhcp;
    // Only what this build actually contains. Claiming Tunnelling here would
    // have ETS offer the device as an interface and then fail to connect to it.
    config.deviceCapabilities = netip::device_capability::kDhcp;
    config.friendlyName = resolveFriendlyName();
    (void)wifi_service_mac(config.macAddress.data());

    const auto result = app.applyKnxNetIpConfiguration(config);
    if (result.isError()) {
        KNX_LOGW(TAG, "KNXnet/IP parameter publish failed: %d", static_cast<int>(result.error()));
        return;
    }

    char text[16] = {0};
    config.currentIpAddress.toString(text);
    KNX_LOGI(TAG, "KNXnet/IP parameters published: %s", text);
}

util::Result<HabinariProductHandle> startProduct(platform::Esp32Platform &platform,
                                                 HabinariBindings &&bindings)
{
    // The stack needs an interface to join the multicast group on. Waiting is
    // not the same as requiring: the routing endpoint recovers when the
    // interface turns up, so running out of patience costs a slower start
    // rather than a dead device.
    if (CONFIG_HABINARI_KNXIP_WIFI_WAIT_S > 0 && !wifi_service_is_connected()) {
        KNX_LOGI(TAG, "Waiting up to %d s for Wi-Fi", CONFIG_HABINARI_KNXIP_WIFI_WAIT_S);
        g_knx.waitingForNetwork.store(true);
        const esp_err_t waited = wifi_service_wait_connected(CONFIG_HABINARI_KNXIP_WIFI_WAIT_S * 1000u);
        g_knx.waitingForNetwork.store(false);
        if (waited != ESP_OK) {
            KNX_LOGW(TAG, "Starting KNXnet/IP without a network — the device will "
                          "join the routing group once Wi-Fi associates");
        }
    }
    g_knx.startedWithoutNetwork = !wifi_service_is_connected();

    IpRoutingOptions options{};
    options.multicastGroup = resolveMulticastGroup();
    options.port = NetIpPort(CONFIG_HABINARI_KNXIP_PORT);
    options.timeToLive = static_cast<uint8_t>(CONFIG_HABINARI_KNXIP_TTL);
    options.friendlyName = resolveFriendlyName();

    char groupText[16] = {0};
    options.multicastGroup.toString(groupText);
    KNX_LOGI(TAG, "KNXnet/IP routing on %s:%d (TTL %d), announced as \"%s\"",
             groupText, CONFIG_HABINARI_KNXIP_PORT, CONFIG_HABINARI_KNXIP_TTL,
             options.friendlyName.c_str());

    return startCommissionedProduct(platform, kHabinariProduct, std::move(bindings),
                                    std::move(options));
}

#else  // TP1

util::Result<HabinariProductHandle> startProduct(platform::Esp32Platform &platform,
                                                 HabinariBindings &&bindings)
{
    auto physicalResult = createTp1Physical(platform);
    if (physicalResult.isError()) {
        KNX_LOGE(TAG, "TP1 physical init failed: %d", static_cast<int>(physicalResult.error()));
        return physicalResult.error();
    }

    return startCommissionedProduct(platform, kHabinariProduct, std::move(bindings),
                                    std::move(physicalResult.value()));
}

#endif

PublishSnapshot takePublishSnapshot()
{
    PublishSnapshot snapshot;
    LockGuard lock(g_state.mutex);
    snapshot.sensorData = g_state.latestSensorData;
    snapshot.settings = g_state.hvac;
    snapshot.out = g_state.out;
    snapshot.hvacOperatingMode = g_state.in.hvacOperatingMode;
    snapshot.controllerMode = g_state.in.controllerMode;
    snapshot.ventilationMode = g_state.in.ventilationMode;
    snapshot.controllerOnOff = g_state.in.controllerOnOff;
    return snapshot;
}

// Publish every transmitting port. The transmit policy decides what actually
// reaches the bus (see applyTransmitPolicies), so an unchanged value costs a
// compare and nothing else; a port the ETS project left unlinked is a no-op.
template <typename AppT>
void publishAllState(AppT &app, const PublishSnapshot &snapshot)
{
    const auto &out = snapshot.out;
    const auto &sensors = snapshot.sensorData;
    const control::CorrectedReadings readings = control::correctReadings(sensors, snapshot.settings);

    // --- Room air measurements (FB RTS / RRHS / RAQS) ---
    if (readings.roomTemperatureValid) {
        publishPort(app, HabinariPort::RoomTemperature, readings.roomTemperatureC,
                    "room temperature");
    }
    if (readings.roomHumidityValid) {
        publishPort(app, HabinariPort::RoomHumidity, readings.roomHumidityPct, "room humidity");
    }
    if (readings.co2Valid) {
        publishPort(app, HabinariPort::RoomCo2, readings.co2Ppm, "room co2");
    }
    if (readings.pressureValid) {
        publishPort(app, HabinariPort::RoomAirPressure, readings.pressurePa, "air pressure");
        publishPort(app, HabinariPort::RoomAirPressureSeaLevel, out.seaLevelPressurePa,
                    "sea-level pressure");
    }
    if (readings.iaqValid) {
        publishPort(app, HabinariPort::RoomAirQualityIndex,
                    static_cast<uint16_t>(readings.iaqIndex + 0.5f), "air quality index");
        publishPort(app, HabinariPort::AirQualityAccuracy, sensors.air_quality_accuracy,
                    "air quality accuracy");
    }
    if (sensors.co2_equivalent.valid) {
        publishPort(app, HabinariPort::RoomCo2Equivalent, sensors.co2_equivalent.value,
                    "co2 equivalent");
    }
    if (sensors.voc_equivalent.valid) {
        publishPort(app, HabinariPort::RoomVocEquivalent, sensors.voc_equivalent.value,
                    "voc equivalent");
    }

    // --- Derived room air values ---
    if (readings.roomAirValid()) {
        publishPort(app, HabinariPort::RoomDewPoint, out.roomDewPointC, "room dew point");
        publishPort(app, HabinariPort::RoomAbsoluteHumidity, out.roomAbsoluteHumidityGm3,
                    "room absolute humidity");
    }

    // --- Floor probe (FB FTS + slab moisture) ---
    if (readings.floorTemperatureValid) {
        publishPort(app, HabinariPort::FloorTemperature, readings.floorTemperatureC,
                    "floor temperature");
    }
    if (readings.floorHumidityValid) {
        publishPort(app, HabinariPort::FloorHumidity, readings.floorHumidityPct,
                    "floor humidity");
    }
    if (readings.floorProbeValid()) {
        publishPort(app, HabinariPort::FloorAbsoluteHumidity, out.floorAbsoluteHumidityGm3,
                    "floor absolute humidity");
    }
    publishPort(app, HabinariPort::FloorMoistureAlarm, out.floorMoistureAlarm,
                "floor moisture alarm");
    publishPort(app, HabinariPort::FloorLimitActive, out.floorLimitActive, "floor limit active");
    publishPort(app, HabinariPort::FloorComfortActive, out.floorComfortActive,
                "floor comfort active");

    // --- Condensation protection (FB DPS) ---
    publishPort(app, HabinariPort::DewPointAlarm, out.dewPointAlarm, "dew point alarm");
    publishPort(app, HabinariPort::DewPointMargin, out.dewPointMarginK, "dew point margin");
    publishPort(app, HabinariPort::FreeCoolingAvailable, out.freeCoolingAvailable,
                "free cooling available");
    publishPort(app, HabinariPort::FreeDryingAvailable, out.freeDryingAvailable,
                "free drying available");

    // --- Mode and setpoints (FB RTSM) ---
    publishPort(app, HabinariPort::ControllerOnOff, snapshot.controllerOnOff,
                "controller on/off");
    {
        // HvacModeStatus reports the mode the controller resolved to, which
        // window/presence handling can move away from the requested one.
        const auto result = app.publish(HabinariPort::HvacModeStatus,
                                        static_cast<Dpt20Mode>(out.activePreset));
        if (result.isError()) {
            KNX_LOGW(TAG, "hvac mode status publish failed: %s",
                     util::errorCodeToString(result.error()));
        }
    }
    publishPort(app, HabinariPort::ContrModeStatus,
                hvac_ns::controllerModeToContrMode(out.activeControllerMode), "contr mode status");
    publishPort(app, HabinariPort::ContrModeSecondary,
                hvac_ns::controllerModeToContrMode(out.activeControllerMode),
                "contr mode secondary");
    publishPort(app, HabinariPort::SetpointBase, snapshot.settings.comfortHeatingSetpointC,
                "base setpoint");
    publishPort(app, HabinariPort::SetpointShiftStatus, out.setpointShiftFeedbackK,
                "setpoint shift status");
    publishPort(app, HabinariPort::SetpointStatus, out.activeSetpointC, "active setpoint");
    publishPort(app, HabinariPort::SetpointHeatingStatus, out.heatingSetpointC,
                "heating setpoint");
    publishPort(app, HabinariPort::SetpointCoolingStatus, out.coolingSetpointC,
                "cooling setpoint");

    // --- Controller outputs (FB RTC) ---
    publishPort(app, HabinariPort::HeatingControlValue, out.heatingControlPercent,
                "heating control value");
    publishPort(app, HabinariPort::CoolingControlValue, out.coolingControlPercent,
                "cooling control value");
    publishPort(app, HabinariPort::HeatingRequest, out.heatingRequest, "heating request");
    publishPort(app, HabinariPort::CoolingRequest, out.coolingRequest, "cooling request");
    publishPort(app, HabinariPort::HeatCoolModeStatus, out.heatCoolModeHeating,
                "heat/cool mode status");
    publishPort(app, HabinariPort::EnableHeatStatus, out.enableHeat, "enable heat status");
    publishPort(app, HabinariPort::EnableCoolStatus, out.enableCool, "enable cool status");
    publishPort(app, HabinariPort::ControllerStatus, out.controllerStatus, "controller status");

    // --- Ventilation / air quality ---
    publishPort(app, HabinariPort::Co2Setpoint, snapshot.settings.ventilationSetpointPpm,
                "co2 setpoint");
    publishPort(app, HabinariPort::VentilationDemand, out.ventilationDemandPercent,
                "ventilation demand");
    publishPort(app, HabinariPort::VentilationStage, static_cast<uint8_t>(out.ventilationLevel),
                "ventilation stage");
    publishPort(app, HabinariPort::VentilationMode,
                static_cast<uint8_t>(snapshot.ventilationMode), "ventilation mode");
    publishPort(app, HabinariPort::VentilationBoostRequest, out.ventilationBoostRequest,
                "ventilation boost request");
    publishPort(app, HabinariPort::DehumidifyRequest, out.dehumidifyRequest,
                "dehumidify request");
    publishPort(app, HabinariPort::AirQualityStatus, out.airQualityStatus,
                "air quality status");

    // --- Device diagnostics ---
    publishPort(app, HabinariPort::DeviceFault, out.deviceFault, "device fault");
    publishPort(app, HabinariPort::RoomSensorStatus, out.roomSensorStatus, "room sensor status");
    publishPort(app, HabinariPort::FloorProbeStatus, out.floorProbeStatus, "floor probe status");
    publishPort(app, HabinariPort::AirQualitySensorStatus, out.airQualitySensorStatus,
                "air quality sensor status");
    publishPort(app, HabinariPort::SensorHealthMask, out.sensorHealthMask, "sensor health mask");
    publishPort(app, HabinariPort::SensorDisagreementAlarm, out.sensorDisagreement,
                "sensor disagreement");

    // --- Derived events ---
    // The alarms are published unconditionally so a cleared alarm is reported
    // too; the measured trend only once its window has filled, because a
    // half-filled regression is not a trend.
    publishPort(app, HabinariPort::FireAlarm, sensors.events.fire_alarm, "fire alarm");
    publishPort(app, HabinariPort::FirePreAlarm, sensors.events.fire_pre_alarm,
                "fire pre-alarm");
    if (sensors.trends.temperature.valid) {
        publishPort(app, HabinariPort::TemperatureTrend,
                    sensors.trends.temperature.per_minute * 60.0f, "temperature trend");
    }
    publishPort(app, HabinariPort::OccupancyDetected, sensors.events.occupancy_detected,
                "occupancy detected");
    publishPort(app, HabinariPort::EstimatedOccupants, sensors.events.estimated_occupants,
                "estimated occupants");
    publishPort(app, HabinariPort::WindowOpenDetected, sensors.events.window_open_detected,
                "window open detected");
}

// ---------------------------------------------------------------------------
// Inbound group writes.
//
// A write that has a control_state_write() equivalent goes through it, so a KNX
// telegram meets the same gate a Modbus or MQTT write does — finite, in range,
// a defined code point — and a refusal is logged instead of cast into the
// controller. What has no command (the neighbour-device measurements, the
// changeover inputs) is written directly, as before.
// ---------------------------------------------------------------------------

bool applyCommand(control_command_t command, float value, const char *label)
{
    const esp_err_t err = control_state_write(command, value);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s write %.2f rejected: %s", label, static_cast<double>(value),
                 esp_err_to_name(err));
        return false;
    }
    return true;
}

// A DPT 9 group write carrying 7FFFh, the "invalid data" code (knx_dpt9.hpp),
// is the sender saying it has no value — never a value to act on.
bool dpt9Usable(float value, const char *label)
{
    if (habinari_knx::dpt9IsInvalid(value)) {
        ESP_LOGW(TAG, "%s: sender reports invalid data (DPT 9 7FFFh), ignored", label);
        return false;
    }
    return true;
}

template <typename ReadT>
auto readLocked(ReadT read)
{
    LockGuard lock(g_state.mutex);
    return read();
}

// ---------------------------------------------------------------------------
// ETS parameters.
//
// One row per parameter, saying where its value lands in the control state.
// The same table drives the live onParameterChanged bindings and the one-time
// load at start-up; they used to be two hand-kept lists of the same 81
// mappings, which had to be kept in step by reading.
//
// A setter receives the value in the type knx_product.hpp declares for that
// parameter — Dpt9Float (KNX 2-byte half-float, implicitly a float), uint16_t
// for counts and seconds, uint8_t for enumerations and percentages — and runs
// with the control mutex held.
// ---------------------------------------------------------------------------

template <HabinariParameter Id, typename Apply>
struct ParameterRow {
    Apply apply;
};

template <HabinariParameter Id, typename Apply>
constexpr ParameterRow<Id, Apply> row(Apply apply)
{
    return ParameterRow<Id, Apply>{apply};
}

template <HabinariParameter Id>
using ParameterValue = typename std::remove_cvref_t<
    decltype(kHabinariProduct)>::template ParameterValueType<Id>;

// Fusion tuning lands in the fusion config rather than in Settings and is
// pushed down to the acquisition side by the control tick, hence the dirty flag.
constexpr auto kParameterTable = std::make_tuple(
    row<HabinariParameter::MeasurementHeartbeatSeconds>([](auto &g, auto v) {
        g.hvac.heartbeatSeconds = v;
    }),
    row<HabinariParameter::MeasurementMinRepTimeSeconds>([](auto &g, auto v) {
        g.hvac.minRepTimeSeconds = v;
    }),
    row<HabinariParameter::RoomTemperatureOffset>([](auto &g, auto v) {
        g.hvac.roomTemperatureOffsetK = v;
    }),
    row<HabinariParameter::RoomTemperatureCov>([](auto &g, auto v) {
        g.hvac.roomTemperatureCovK = v;
    }),
    row<HabinariParameter::RoomHumidityOffset>([](auto &g, auto v) {
        g.hvac.roomHumidityOffsetPct = v;
    }),
    row<HabinariParameter::RoomHumidityCov>([](auto &g, auto v) {
        g.hvac.roomHumidityCovPct = v;
    }),
    row<HabinariParameter::Co2Cov>([](auto &g, auto v) {
        g.hvac.co2CovPpm = static_cast<float>(v);
    }),
    row<HabinariParameter::PressureCov>([](auto &g, auto v) {
        g.hvac.pressureCovPa = static_cast<float>(v);
    }),
    row<HabinariParameter::AirQualityCov>([](auto &g, auto v) {
        g.hvac.airQualityCovIndex = static_cast<float>(v);
    }),
    row<HabinariParameter::FloorTemperatureOffset>([](auto &g, auto v) {
        g.hvac.floorTemperatureOffsetK = v;
    }),
    row<HabinariParameter::FloorTemperatureCov>([](auto &g, auto v) {
        g.hvac.floorTemperatureCovK = v;
    }),
    row<HabinariParameter::FloorHumidityCov>([](auto &g, auto v) {
        g.hvac.floorHumidityCovPct = v;
    }),
    row<HabinariParameter::DerivedValueCov>([](auto &g, auto v) {
        g.hvac.derivedCov = v;
    }),
    row<HabinariParameter::AltitudeM>([](auto &g, auto v) {
        g.hvac.altitudeM = static_cast<float>(v);
    }),
    row<HabinariParameter::ControllerDefaultEnable>([](auto &g, auto v) {
        g.hvac.controllerDefaultEnable = (v != 0);
    }),
    row<HabinariParameter::DefaultHvacOperatingMode>([](auto &g, auto v) {
        g.hvac.defaultHvacOperatingMode = static_cast<hvac_ns::OperatingPreset>(v);
    }),
    row<HabinariParameter::DefaultControllerMode>([](auto &g, auto v) {
        g.hvac.defaultControllerMode = static_cast<hvac_ns::ControllerMode>(v);
    }),
    row<HabinariParameter::HeatingEnabled>([](auto &g, auto v) {
        g.hvac.heatingEnabled = (v != 0);
    }),
    row<HabinariParameter::CoolingEnabled>([](auto &g, auto v) {
        g.hvac.coolingEnabled = (v != 0);
    }),
    row<HabinariParameter::HeatCoolChangeoverMode>([](auto &g, auto v) {
        g.hvac.heatCoolChangeoverMode = static_cast<hvac_ns::HeatCoolChangeoverMode>(v);
    }),
    row<HabinariParameter::HeatCoolChangeoverPolarity>([](auto &g, auto v) {
        g.hvac.heatCoolChangeoverPolarityInverted = (v != 0);
    }),
    row<HabinariParameter::MinimumHeatCoolChangeoverSeconds>([](auto &g, auto v) {
        g.hvac.minimumHeatCoolChangeoverSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::WindowOpenBehavior>([](auto &g, auto v) {
        g.hvac.windowOpenBehavior = static_cast<hvac_ns::WindowOpenBehavior>(v);
    }),
    row<HabinariParameter::PresenceBehavior>([](auto &g, auto v) {
        g.hvac.presenceBehavior = static_cast<hvac_ns::PresenceBehavior>(v);
    }),
    row<HabinariParameter::SensorFaultBehavior>([](auto &g, auto v) {
        g.hvac.sensorFaultBehavior = static_cast<hvac_ns::SensorFaultBehavior>(v);
    }),
    row<HabinariParameter::ComfortHeatingSetpoint>([](auto &g, auto v) {
        g.hvac.comfortHeatingSetpointC = v;
    }),
    row<HabinariParameter::StandbyHeatingReduction>([](auto &g, auto v) {
        g.hvac.standbyHeatingReductionK = v;
    }),
    row<HabinariParameter::EconomyHeatingReduction>([](auto &g, auto v) {
        g.hvac.economyHeatingReductionK = v;
    }),
    row<HabinariParameter::ProtectionHeatingSetpoint>([](auto &g, auto v) {
        g.hvac.protectionHeatingSetpointC = v;
    }),
    row<HabinariParameter::CoolingDeadband>([](auto &g, auto v) {
        g.hvac.coolingDeadbandK = v;
    }),
    row<HabinariParameter::StandbyCoolingIncrease>([](auto &g, auto v) {
        g.hvac.standbyCoolingIncreaseK = v;
    }),
    row<HabinariParameter::EconomyCoolingIncrease>([](auto &g, auto v) {
        g.hvac.economyCoolingIncreaseK = v;
    }),
    row<HabinariParameter::ProtectionCoolingSetpoint>([](auto &g, auto v) {
        g.hvac.protectionCoolingSetpointC = v;
    }),
    row<HabinariParameter::MinSetpoint>([](auto &g, auto v) {
        g.hvac.minSetpointC = v;
    }),
    row<HabinariParameter::MaxSetpoint>([](auto &g, auto v) {
        g.hvac.maxSetpointC = v;
    }),
    row<HabinariParameter::MaxSetpointShift>([](auto &g, auto v) {
        g.hvac.maxSetpointShiftK = v;
    }),
    row<HabinariParameter::HeatingControlAlgorithm>([](auto &g, auto v) {
        g.hvac.heatingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(v);
    }),
    row<HabinariParameter::HeatingKp>([](auto &g, auto v) {
        g.hvac.heatingKp = v;
    }),
    row<HabinariParameter::HeatingTiSeconds>([](auto &g, auto v) {
        g.hvac.heatingTiSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::HeatingTdSeconds>([](auto &g, auto v) {
        g.hvac.heatingTdSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::HeatingMinimumOutputPercent>([](auto &g, auto v) {
        g.hvac.heatingMinOutputPercent = v;
    }),
    row<HabinariParameter::HeatingMaximumOutputPercent>([](auto &g, auto v) {
        g.hvac.heatingMaxOutputPercent = v;
    }),
    row<HabinariParameter::CoolingControlAlgorithm>([](auto &g, auto v) {
        g.hvac.coolingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(v);
    }),
    row<HabinariParameter::CoolingKp>([](auto &g, auto v) {
        g.hvac.coolingKp = v;
    }),
    row<HabinariParameter::CoolingTiSeconds>([](auto &g, auto v) {
        g.hvac.coolingTiSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::CoolingTdSeconds>([](auto &g, auto v) {
        g.hvac.coolingTdSeconds = static_cast<float>(v);
    }),
    row<HabinariParameter::CoolingMinimumOutputPercent>([](auto &g, auto v) {
        g.hvac.coolingMinOutputPercent = v;
    }),
    row<HabinariParameter::CoolingMaximumOutputPercent>([](auto &g, auto v) {
        g.hvac.coolingMaxOutputPercent = v;
    }),
    row<HabinariParameter::ThermostatHysteresis>([](auto &g, auto v) {
        g.hvac.thermostatHysteresisC = v;
    }),
    row<HabinariParameter::BinaryDemandStrategy>([](auto &g, auto v) {
        g.hvac.binaryDemandStrategy = static_cast<hvac_ns::BinaryDemandStrategy>(v);
    }),
    row<HabinariParameter::BinaryDemandThresholdPercent>([](auto &g, auto v) {
        g.hvac.binaryDemandThresholdPercent = v;
    }),
    row<HabinariParameter::FrostAlarmTemperature>([](auto &g, auto v) {
        g.hvac.frostAlarmTemperatureC = v;
    }),
    row<HabinariParameter::OverheatAlarmTemperature>([](auto &g, auto v) {
        g.hvac.overheatAlarmTemperatureC = v;
    }),
    row<HabinariParameter::MaxFloorTemperature>([](auto &g, auto v) {
        g.hvac.maxFloorTemperatureC = v;
    }),
    row<HabinariParameter::MinFloorTemperature>([](auto &g, auto v) {
        g.hvac.minFloorTemperatureC = v;
    }),
    row<HabinariParameter::FloorHysteresis>([](auto &g, auto v) {
        g.hvac.floorHysteresisK = v;
    }),
    row<HabinariParameter::FloorComfortOutputPercent>([](auto &g, auto v) {
        g.hvac.floorComfortOutputPercent = v;
    }),
    row<HabinariParameter::DewPointSurfaceSource>([](auto &g, auto v) {
        g.hvac.dewPointSurfaceSource = static_cast<hvac_ns::DewPointSurfaceSource>(v);
    }),
    row<HabinariParameter::DewPointMargin>([](auto &g, auto v) {
        g.hvac.dewPointMarginK = v;
    }),
    row<HabinariParameter::DewPointHysteresis>([](auto &g, auto v) {
        g.hvac.dewPointHysteresisK = v;
    }),
    row<HabinariParameter::BlockCoolingOnDewPointAlarm>([](auto &g, auto v) {
        g.hvac.blockCoolingOnDewPointAlarm = (v != 0);
    }),
    row<HabinariParameter::FloorMoistureThreshold>([](auto &g, auto v) {
        g.hvac.floorMoistureThresholdPct = v;
    }),
    row<HabinariParameter::FloorMoistureHysteresis>([](auto &g, auto v) {
        g.hvac.floorMoistureHysteresisPct = v;
    }),
    row<HabinariParameter::FloorMoistureExcess>([](auto &g, auto v) {
        g.hvac.floorMoistureExcessGm3 = v;
    }),
    row<HabinariParameter::VentilationSetpoint>([](auto &g, auto v) {
        g.hvac.ventilationSetpointPpm = static_cast<float>(v);
    }),
    row<HabinariParameter::VentilationBand>([](auto &g, auto v) {
        g.hvac.ventilationBandPpm = static_cast<float>(v);
    }),
    row<HabinariParameter::HumidityBoostThreshold>([](auto &g, auto v) {
        g.hvac.humidityBoostPct = v;
    }),
    row<HabinariParameter::HumidityBoostBand>([](auto &g, auto v) {
        g.hvac.humidityBandPct = v;
    }),
    row<HabinariParameter::VocBoostThreshold>([](auto &g, auto v) {
        g.hvac.vocThresholdIndex = static_cast<float>(v);
    }),
    row<HabinariParameter::VocBoostBand>([](auto &g, auto v) {
        g.hvac.vocBandIndex = static_cast<float>(v);
    }),
    row<HabinariParameter::VentilationBaseDemandPercent>([](auto &g, auto v) {
        g.hvac.ventilationBaseDemandPercent = v;
    }),
    row<HabinariParameter::VentilationManualDemandPercent>([](auto &g, auto v) {
        g.hvac.ventilationManualDemandPercent = v;
    }),
    row<HabinariParameter::SensorFilterSeconds>([](auto &g, auto v) {
        g.fusion.filter_tau_seconds = static_cast<float>(v);
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::TemperatureCrossCheck>([](auto &g, auto v) {
        g.fusion.temperature_cross_check_k = v;
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::HumidityCrossCheck>([](auto &g, auto v) {
        g.fusion.humidity_cross_check_pct = v;
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::FireRateOfRise>([](auto &g, auto v) {
        g.fusion.fire_rate_of_rise_k_per_min = v;
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::FireAbsoluteTemperature>([](auto &g, auto v) {
        g.fusion.fire_absolute_alarm_c = v;
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::FireConfirmSeconds>([](auto &g, auto v) {
        g.fusion.fire_confirm_seconds = static_cast<float>(v);
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::FireRequireAirQuality>([](auto &g, auto v) {
        g.fusion.fire_require_air_quality = (v != 0);
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::Co2OccupancyEnabled>([](auto &g, auto v) {
        g.fusion.co2_occupancy_enabled = (v != 0);
        g.fusionConfigDirty = true;
    }),
    row<HabinariParameter::WindowDetectEnabled>([](auto &g, auto v) {
        g.fusion.window_detect_enabled = (v != 0);
        g.fusionConfigDirty = true;
    })
);

template <typename BuilderT, HabinariParameter Id, typename Apply>
void bindParameter(BuilderT &bindings, const ParameterRow<Id, Apply> &)
{
    bindings.template onParameterChanged<Id>([](ParameterValue<Id> value) {
        LockGuard lock(g_state.mutex);
        Apply{}(g_state, value);
    });
}

template <typename BuilderT>
void bindParameters(BuilderT &bindings)
{
    std::apply([&bindings](const auto &...rows) { (bindParameter(bindings, rows), ...); },
               kParameterTable);
}

// The caller holds the control mutex.
template <typename ParametersT, HabinariParameter Id, typename Apply>
void loadParameter(const ParametersT &params, const ParameterRow<Id, Apply> &)
{
    Apply{}(g_state, params.template get<Id>());
}

template <typename ParametersT>
void loadParameters(const ParametersT &params)
{
    std::apply([&params](const auto &...rows) { (loadParameter(params, rows), ...); },
               kParameterTable);
}

// Tells knxAdapterStart() how start-up went: ESP_OK once the product is
// running, or the error that stopped it — if it is still waiting. A result
// nobody waits for any more is only logged, and only when it is a failure:
// that is the case of a KNXnet/IP start that outlasted the boot sequence
// waiting for Wi-Fi (see kKnxStartTimeoutMs).
void reportStartResult(esp_err_t result)
{
    const TaskHandle_t waiter = g_knx.startWaiter.exchange(nullptr);
    if (waiter != nullptr) {
        (void)xTaskNotify(waiter, static_cast<uint32_t>(result), eSetValueWithOverwrite);
    } else if (result != ESP_OK) {
        KNX_LOGE(TAG, "KNX failed to start after boot had moved on: %s", esp_err_to_name(result));
    }
}

esp_err_t toEspErr(util::ErrorCode error)
{
    // The detail is in the log line above this; the code only has to be
    // something esp_err_to_name() can say and the registry can refuse on.
    switch (error) {
    case util::ErrorCode::Timeout:
        return ESP_ERR_TIMEOUT;
    case util::ErrorCode::OperationNotSupported:
        return ESP_ERR_NOT_SUPPORTED;
    default:
        return ESP_FAIL;
    }
}

void knxServiceTask(void *arg)
{
    (void)arg;
    // First, before anything that could lead to a notification: the control
    // tick and the stack's work callback both look this up to wake us.
    g_knx.taskHandle.store(xTaskGetCurrentTaskHandle());

    // DEBUG shows per-frame bus traffic (RX/TX dumps, DL-ACK diagnostics,
    // connection events) — the development default while the stack is being
    // brought up. Info for production builds: INFO carries only state changes
    // (address, lifecycle, load state) and WARN/ERROR faults. The esp_log
    // levels this routes through are set for the whole image in main.c.
    knx::log::setLevel(knx::log::Level::Info);

    platform::Esp32Platform platform;

    // startProduct() builds the runtime on the heap and hands back an owning
    // handle: at ~34 KB it cannot travel through a stack frame. The bindings
    // builder is still a stack object, so it stays scoped so its space is
    // released before the service loop runs.
    HabinariProductHandle appPtr;
    {
    // Declared first, then chained onto as an lvalue. Writing this as
    // `auto bindings = makeCommissionedBindings(...).provideState(...)...`
    // would put two builders in this frame — the chain's temporary and the
    // named object initialised from it — and the builder is ~9 KB for a
    // product this size.
    auto bindings = makeCommissionedBindings(kHabinariProduct);
    bindings
        // ---- Measurements: read requests are answered from the same
        // corrected readings the control loops use, so a bus read and a
        // spontaneous send can never disagree.
        .provideState<HabinariPort::RoomTemperature>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.temperature.value
                   + g_state.hvac.roomTemperatureOffsetK;
        })
        .provideState<HabinariPort::RoomHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::clampf(
                g_state.latestSensorData.humidity.value + g_state.hvac.roomHumidityOffsetPct,
                0.0f, 100.0f);
        })
        .provideState<HabinariPort::RoomCo2>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.co2.value;
        })
        .provideState<HabinariPort::RoomAirPressure>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.pressure.value;
        })
        .provideState<HabinariPort::RoomAirPressureSeaLevel>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.seaLevelPressurePa;
        })
        .provideState<HabinariPort::RoomAirQualityIndex>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint16_t>(g_state.latestSensorData.iaq.value + 0.5f);
        })
        .provideState<HabinariPort::RoomCo2Equivalent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.co2_equivalent.value;
        })
        .provideState<HabinariPort::RoomVocEquivalent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.voc_equivalent.value;
        })
        .provideState<HabinariPort::AirQualityAccuracy>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.air_quality_accuracy;
        })
        .provideState<HabinariPort::RoomDewPoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomDewPointC;
        })
        .provideState<HabinariPort::RoomAbsoluteHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomAbsoluteHumidityGm3;
        })
        .provideState<HabinariPort::FloorTemperature>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.probe_temperature.value
                   + g_state.hvac.floorTemperatureOffsetK;
        })
        .provideState<HabinariPort::FloorHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.probe_humidity.value;
        })
        .provideState<HabinariPort::FloorAbsoluteHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorAbsoluteHumidityGm3;
        })
        .provideState<HabinariPort::FloorMoistureAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorMoistureAlarm;
        })
        .provideState<HabinariPort::FloorLimitActive>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorLimitActive;
        })
        .provideState<HabinariPort::FloorComfortActive>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorComfortActive;
        })

        // ---- Condensation protection (FB DPS) ----
        .provideState<HabinariPort::DewPointAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dewPointAlarm;
        })
        .provideState<HabinariPort::DewPointMargin>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dewPointMarginK;
        })
        .onStateWrite<HabinariPort::DewPointStatusInput>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.externalDewPointAlarm = value;
        })

        // ---- Neighbour-device inputs ----
        // A neighbour that reports invalid data has no reading, so its last
        // one stops being used rather than being trusted indefinitely.
        .onStateWrite<HabinariPort::OutsideTemperature>([](float value) {
            const bool usable = dpt9Usable(value, "outside temperature");
            LockGuard lock(g_state.mutex);
            if (usable) {
                g_state.in.outsideTemperatureC = value;
            }
            g_state.in.outsideTemperatureValid = usable;
        })
        .onStateWrite<HabinariPort::OutsideHumidity>([](float value) {
            const bool usable = dpt9Usable(value, "outside humidity");
            LockGuard lock(g_state.mutex);
            if (usable) {
                g_state.in.outsideHumidityPct = hvac_ns::clampf(value, 0.0f, 100.0f);
            }
            g_state.in.outsideHumidityValid = usable;
        })
        .onStateWrite<HabinariPort::FlowTemperature>([](float value) {
            const bool usable = dpt9Usable(value, "flow temperature");
            LockGuard lock(g_state.mutex);
            if (usable) {
                g_state.in.flowTemperatureC = value;
            }
            g_state.in.flowTemperatureValid = usable;
        })
        .provideState<HabinariPort::FreeCoolingAvailable>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.freeCoolingAvailable;
        })
        .provideState<HabinariPort::FreeDryingAvailable>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.freeDryingAvailable;
        })

        // ---- Mode and setpoints (FB RTSM) ----
        .onStateWrite<HabinariPort::ControllerOnOff>([](bool value) {
            applyCommand(CONTROL_CMD_CONTROLLER_ON_OFF, value ? 1.0f : 0.0f, "controller on/off");
        })
        .provideState<HabinariPort::ControllerOnOff>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.in.controllerOnOff;
        })
        // The raw bus byte, unvalidated: control_state_write() refuses a code
        // point OperatingPreset does not define instead of casting it in.
        .onStateWrite<HabinariPort::HvacMode>([](Dpt20Mode value) {
            applyCommand(CONTROL_CMD_HVAC_MODE,
                         static_cast<float>(static_cast<uint8_t>(value)), "hvac mode");
        })
        .provideState<HabinariPort::HvacModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<Dpt20Mode>(g_state.out.activePreset);
        })
        // The bus objects carry DPT 20.105 (HVACContrMode) code points; the
        // internal enum is compact. Map at the binding boundary only.
        .onStateWrite<HabinariPort::ContrMode>([](uint8_t value) {
            applyCommand(CONTROL_CMD_CONTROLLER_MODE,
                         static_cast<float>(hvac_ns::controllerModeFromContrMode(value)),
                         "controller mode");
        })
        .provideState<HabinariPort::ContrModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::controllerModeToContrMode(g_state.out.activeControllerMode);
        })
        .provideState<HabinariPort::ContrModeSecondary>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::controllerModeToContrMode(g_state.out.activeControllerMode);
        })
        // Base setpoint write from an HMI or Home Assistant's
        // target_temperature: this is the comfort heating anchor of the KNX
        // setpoint ladder, so writing it moves standby/economy/cooling with it.
        // Clamped to the configured limits inside control_state_write(), the
        // same as a write from any other adapter.
        .onStateWrite<HabinariPort::SetpointBase>([](float value) {
            if (!dpt9Usable(value, "base setpoint")) {
                return;
            }
            const auto current = [] { return g_state.hvac.comfortHeatingSetpointC; };
            const float before = readLocked(current);
            if (applyCommand(CONTROL_CMD_SETPOINT_BASE, value, "base setpoint")) {
                const float after = readLocked(current);
                if (std::fabs(after - before) >= 0.01f) {
                    ESP_LOGI(TAG, "Base (comfort heating) setpoint updated to %.2f C", after);
                }
            }
        })
        .provideState<HabinariPort::SetpointBase>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.comfortHeatingSetpointC;
        })
        .onStateWrite<HabinariPort::SetpointShift>([](float value) {
            if (dpt9Usable(value, "setpoint shift")) {
                applyCommand(CONTROL_CMD_SETPOINT_SHIFT, value, "setpoint shift");
            }
        })
        .provideState<HabinariPort::SetpointShiftStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.setpointShiftFeedbackK;
        })
        .provideState<HabinariPort::SetpointStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.activeSetpointC;
        })
        .provideState<HabinariPort::SetpointHeatingStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingSetpointC;
        })
        .provideState<HabinariPort::SetpointCoolingStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingSetpointC;
        })

        // ---- Room inputs (FB WOS / PRD / WCOS) ----
        .onStateWrite<HabinariPort::WindowStatus>([](bool value) {
            applyCommand(CONTROL_CMD_WINDOW_STATUS, value ? 1.0f : 0.0f, "window status");
        })
        .onStateWrite<HabinariPort::PresenceStatus>([](bool value) {
            applyCommand(CONTROL_CMD_PRESENCE, value ? 1.0f : 0.0f, "presence");
        })
        .onStateWrite<HabinariPort::SwitchHeat>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.switchHeat = value;
        })
        .onStateWrite<HabinariPort::SwitchCool>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.switchCool = value;
        })
        .onStateWrite<HabinariPort::ChangeOverStatus>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.changeOverStatus = value;
        })

        // ---- Controller outputs (FB RTC) ----
        .provideState<HabinariPort::HeatingControlValue>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingControlPercent;
        })
        .provideState<HabinariPort::CoolingControlValue>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingControlPercent;
        })
        .provideState<HabinariPort::HeatingRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingRequest;
        })
        .provideState<HabinariPort::CoolingRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingRequest;
        })
        .provideState<HabinariPort::HeatCoolModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatCoolModeHeating;
        })
        .provideState<HabinariPort::EnableHeatStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.enableHeat;
        })
        .provideState<HabinariPort::EnableCoolStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.enableCool;
        })
        .provideState<HabinariPort::ControllerStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.controllerStatus;
        })

        // ---- Ventilation / air quality ----
        .onStateWrite<HabinariPort::Co2Setpoint>([](float value) {
            if (!dpt9Usable(value, "CO2 setpoint")) {
                return;
            }
            const auto current = [] { return g_state.hvac.ventilationSetpointPpm; };
            const float before = readLocked(current);
            if (applyCommand(CONTROL_CMD_CO2_SETPOINT, value, "CO2 setpoint")) {
                const float after = readLocked(current);
                if (std::fabs(after - before) >= 0.5f) {
                    ESP_LOGI(TAG, "CO2 setpoint updated to %.0f ppm", after);
                }
            }
        })
        .provideState<HabinariPort::Co2Setpoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.ventilationSetpointPpm;
        })
        .provideState<HabinariPort::VentilationDemand>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.ventilationDemandPercent;
        })
        .provideState<HabinariPort::VentilationStage>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.out.ventilationLevel);
        })
        .onStateWrite<HabinariPort::VentilationMode>([](uint8_t value) {
            applyCommand(CONTROL_CMD_VENTILATION_MODE, static_cast<float>(value),
                         "ventilation mode");
        })
        .provideState<HabinariPort::VentilationMode>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.in.ventilationMode);
        })
        .provideState<HabinariPort::VentilationBoostRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.ventilationBoostRequest;
        })
        .provideState<HabinariPort::DehumidifyRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dehumidifyRequest;
        })
        .provideState<HabinariPort::AirQualityStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.airQualityStatus;
        })

        // ---- Device diagnostics ----
        .provideState<HabinariPort::DeviceFault>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.deviceFault;
        })
        .provideState<HabinariPort::RoomSensorStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomSensorStatus;
        })
        .provideState<HabinariPort::FloorProbeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorProbeStatus;
        })
        .provideState<HabinariPort::AirQualitySensorStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.airQualitySensorStatus;
        })
        .provideState<HabinariPort::SensorHealthMask>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.sensorHealthMask;
        })
        .provideState<HabinariPort::SensorDisagreementAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.sensorDisagreement;
        })

        // ---- Derived events (see sensor_fusion.hpp) ----
        .provideState<HabinariPort::FireAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.fire_alarm;
        })
        .provideState<HabinariPort::FirePreAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.fire_pre_alarm;
        })
        .provideState<HabinariPort::TemperatureTrend>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.trends.temperature.per_minute * 60.0f;
        })
        .provideState<HabinariPort::OccupancyDetected>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.occupancy_detected;
        })
        .provideState<HabinariPort::EstimatedOccupants>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.estimated_occupants;
        })
        .provideState<HabinariPort::WindowOpenDetected>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.window_open_detected;
        })
        // Acknowledging clears the latched fire alarm. Handled on the control
        // tick rather than here so the acknowledge and the detector state
        // change happen in one place, in the task that owns the detector.
        .onStateWrite<HabinariPort::AlarmAcknowledge>([](bool value) {
            if (value) {
                applyCommand(CONTROL_CMD_ACKNOWLEDGE_ALARMS, 1.0f, "alarm acknowledge");
            }
        })

        .onProgrammingModeChanged([](bool enabled) {
            KNX_LOGI(TAG, "Programming mode: %s", enabled ? "ON" : "OFF");
            // ETS can enter programming mode over the bus, not only via the
            // button, so the service hears about it either way and the board LED
            // has one thing to follow.
            control_service_set_programming_mode(enabled);
        })
        .onLifecycleChanged([](DeviceLifecycleState state) {
            KNX_LOGI(TAG,
                     "Lifecycle: %s",
                     state == DeviceLifecycleState::Operational   ? "Operational" :
                     state == DeviceLifecycleState::Commissioning ? "Commissioning" :
                                                                  "Uncommissioned");
        })
        .onFault([](FaultInfo info) {
            KNX_LOGE(TAG,
                     "Fault: code=%d detail=%s",
                     static_cast<int>(info.code),
                     info.detail != nullptr ? info.detail : "");
        });
    bindParameters(bindings);

    auto appResult = startProduct(platform, std::move(bindings));

    if (appResult.isError()) {
        KNX_LOGE(TAG, "KNX start failed: %s", util::errorCodeToString(appResult.error()));
        g_knx.taskHandle.store(nullptr);
        reportStartResult(toEspErr(appResult.error()));
        vTaskDelete(nullptr);
        return;
    }

    // Already heap-constructed by startCommissionedProduct — just take ownership.
    appPtr = std::move(appResult.value());
    } // release the bindings builder's stack space
    auto &app = *appPtr;

    // Kept in scope for the Data Secure block below: the ETS device
    // certificate encodes the serial number together with the key.
    uint8_t serialNumber[6] = {};
    bool hasSerialNumber = false;

    {
        // KNX serial number (device object PID 11): derive from the
        // factory-programmed base MAC so every board reports a unique,
        // stable identity instead of all zeroes. The MAC is globally unique,
        // so this doubles as the device's manufacturing serial. It is logged
        // below so it can be paired with the Data Secure key during ETS
        // commissioning.
        uint8_t mac[6] = {};
        if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
            app.deviceObject().setSerialNumber(std::span<const uint8_t>(mac, sizeof(mac)));
            std::copy(std::begin(mac), std::end(mac), std::begin(serialNumber));
            hasSerialNumber = true;
            char serialHex[3 * 6] = {};
            toHex(std::span<const uint8_t>(mac, sizeof(mac)), ':', serialHex, sizeof(serialHex));
            KNX_LOGI(TAG, "KNX serial number: %s", serialHex);
        } else {
            KNX_LOGW(TAG, "KNX serial number unavailable: esp_read_mac failed");
        }
    }

    // Outbound transmit shaping. A monotonic millisecond clock is required for
    // the global rate limiter, per-object cyclic heartbeats and min-interval
    // floors to take effect (see applyTransmitPolicies + setTelegramRateLimit).
    app.setTimeSource([]() -> uint32_t {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000);
    });
    app.setTelegramRateLimit(application::TelegramRateLimitConfig{
        .maxTelegrams = kMaxUnsolicitedTelegramsPerWindow,
        .perWindowMs = kTelegramRateWindowMs,
        .minGapMs = 0,
    });

    {
        // Resolve the device-unique KNX Data Secure factory tool key and log it
        // in hex so an installer can capture it locally and enter it, together
        // with the serial number above, into ETS for secure commissioning.
        // This key is the device's FDSK / initial tool key: derived from the
        // eFuse root secret once that is provisioned, otherwise the legacy
        // NVS-stored random key.
        std::array<uint8_t, 16> toolKey{};
        bool created = false;
        ToolKeySource keySource = ToolKeySource::NvsFallback;
        if (resolveFactoryToolKey(hasSerialNumber ? serialNumber : nullptr, toolKey, keySource,
                                  created)) {
            char keyHex[3 * 16] = {};
            toHex(std::span<const uint8_t>(toolKey.data(), toolKey.size()), ' ', keyHex, sizeof(keyHex));

            // ETS installs its own tool key during secure commissioning and
            // then restarts the device; the stack restores that key from its
            // own persistence. Re-applying the factory key here would put the
            // device back on its FDSK behind ETS's back, and the first thing
            // ETS does after the restart — an S-A_Sync_Request — then fails to
            // verify ("no SyncResponse was received; probably the key did not
            // match"). So the factory key is only applied when the device does
            // not already carry one.
            //
            // Security Mode is deliberately not asserted here either. It is
            // ETS's property, it is persisted, and ETS turns it on when it
            // finishes a secure download. Forcing it on at every boot made the
            // device refuse every unsecured management service, so a project
            // that does not use security could not commission it at all: ETS
            // opened the connection, sent A_Authorize_Request in the clear,
            // and the device dropped it without an answer.
            // hasEtsToolKey() is only "some tool key is set", which is also
            // true of the factory key this firmware installed on an earlier
            // boot. What decides whether the certificate below is still worth
            // printing is whether the key in force IS the factory key.
            const bool factoryKeyInForce = app.toolKeyInForceIs(toolKey);
            const bool provisionedKeyInForce = app.hasEtsToolKey() && !factoryKeyInForce;
            const auto secureResult = provisionedKeyInForce
                                          ? util::Result<void>::ok()
                                          : app.installFactoryToolKey(toolKey);
            if (secureResult.isError()) {
                KNX_LOGE(TAG, "Data Secure: installFactoryToolKey failed: %d",
                         static_cast<int>(secureResult.error()));
            } else {
                if (provisionedKeyInForce) {
                    KNX_LOGW(TAG,
                             "KNX Data Secure available (mode %s, set by ETS). Keeping the "
                             "tool key already on the device; the factory key below was not "
                             "re-applied and is only valid again after a factory reset.",
                             app.isSecurityEnabled() ? "ON" : "off");
                } else {
                    KNX_LOGW(TAG,
                             "KNX Data Secure available (mode %s, set by ETS). "
                             "Tool key (%s): %s",
                             app.isSecurityEnabled() ? "ON" : "off",
                             keySource == ToolKeySource::EfuseDerived
                                 ? "derived from the eFuse root secret"
                                 : (created ? "NVS, generated" : "NVS, stored"),
                             keyHex);
                }

                if (keySource == ToolKeySource::NvsFallback) {
                    KNX_LOGW(TAG,
                             "This device has no eFuse root secret yet, so the key above lives "
                             "in NVS and a factory reset will replace it — the certificate "
                             "below is only valid until then.");
                }

                // The certificate is only usable while the factory key is the
                // one the device would actually accept. Printing it once ETS
                // has installed a Tool Key of its own is worse than printing
                // nothing: ETS answers an FDSK that is no longer in force with
                // "Device is secured with a key not known within this project",
                // and re-entering the same certificate can never change that.
                if (!hasSerialNumber) {
                    KNX_LOGW(TAG,
                             "KNX Data Secure device certificate unavailable: no serial number");
                } else if (provisionedKeyInForce) {
                    KNX_LOGW(TAG,
                             "KNX Data Secure device certificate NOT printed: the Tool Key in "
                             "force was installed by ETS, so the factory certificate would not "
                             "be accepted. Use the project that commissioned this device, or "
                             "factory-reset it and use the certificate printed on the next boot.");
                } else {
                    char certificate[habinari::identity::kCertificateBufferSize] = {};
                    formatDeviceCertificate(std::span<const uint8_t, 6>(serialNumber, 6),
                                            toolKey, certificate, sizeof(certificate));
                    KNX_LOGW(TAG,
                             "KNX Data Secure device certificate (enter in ETS): %s",
                             certificate);
                }
            }
        }
    }

    {
        const auto ownAddress = app.individualAddress();
        KNX_LOGI(TAG,
                 "KNX own individual address: 0x%04X (%s)",
                 ownAddress.raw,
                 isInitialIndividualAddress(ownAddress) ? "initial/uncommissioned" :
                 isIndividualBroadcastAddress(ownAddress) ? "broadcast" :
                 isOperationalIndividualAddress(ownAddress) ? "operational" :
                 "unknown");
    }

    {
        LockGuard lock(g_state.mutex);
        // Seed engineering values from the ETS parameters: the same table the
        // onParameterChanged bindings use, applied once. Fusion tuning starts
        // from the compiled-in defaults so the fields the ETS product does not
        // expose (staleness, inter-sensor offsets) are populated, and the
        // table then overlays the ones it does.
        sensor_fusion_default_config(&g_state.fusion);
        loadParameters(app.parameters());
        g_state.fusionConfigDirty = true;
        auto &s = g_state.hvac;
        // Seed the runtime mode/state inputs from their ETS defaults.
        g_state.in.controllerOnOff = s.controllerDefaultEnable;
        g_state.in.hvacOperatingMode = s.defaultHvacOperatingMode;
        g_state.in.controllerMode = s.defaultControllerMode;
    }

#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
    // The address is published now and again on every change: a DHCP renewal
    // that moves the device would otherwise leave ETS reading back an address
    // it no longer answers on.
    publishIpIdentity(app, resolveMulticastGroup());
    wifi_service_set_event_callback(
        [](bool connected, void *ctx) {
            if (!connected) {
                return;
            }
            // Runs on the default event loop, which must not block. Waking the
            // service task is enough; it republishes on its next pass.
            auto *state = static_cast<KnxAdapterState *>(ctx);
            if (state == nullptr) {
                return;
            }
            state->ipIdentityDirty.store(true);
            const TaskHandle_t handle = state->taskHandle.load();
            if (handle != nullptr) {
                xTaskNotifyGive(handle);
            }
        },
        &g_knx);

    // The association that beat the callback registration: it raised no event
    // anyone was listening for, and the socket's own join attempt had already
    // failed. Nothing else would ever ask for the membership again.
    if (g_knx.startedWithoutNetwork && wifi_service_is_connected()) {
        g_knx.ipIdentityDirty.store(true);
    }

    KNX_LOGI(TAG, "ETS-commissionable KNXnet/IP sensor bridge started");
#else
    KNX_LOGI(TAG, "ETS-commissionable TP1 sensor bridge started");
#endif

    DeviceLifecycleState previousLifecycle = app.lifecycleState();

    // The control loops do not run here any more — control_service.cpp owns
    // them, and this task is told when a cycle has completed (see
    // knxAdapterOnControlTick). That is what lets a Modbus-only or MQTT-only
    // image control the room with no KNX task in the picture at all.
    //
    // Publishing is still offered once per control tick, not once per loop
    // iteration: the loop also spins at a few milliseconds while the stack has
    // in-flight work, and re-offering every object that often would burn CPU on
    // suppression decisions that cannot have changed.
    constexpr TickType_t kControlIntervalTicks = pdMS_TO_TICKS(1000);
    bool controlTickDue = true;

    // Event-driven wake: the KNX stack signals when new work appears (an inbound
    // frame decoded by the RX worker task, a queued group auto-response, a
    // pending publish action). We turn that signal into a task notification so
    // this loop can park instead of busy-polling a mostly-empty queue every few
    // ms — that idle spinning wasted CPU and, more importantly, stole cycles
    // from the RX worker whose blocking management-response sends run there.
    // The callback fires from the RX worker or this task's context (never an
    // ISR), so xTaskNotifyGive is correct; the notification latches, so a signal
    // raised while we are mid-loop is coalesced into the next wake, not lost.
    app.setWorkAvailableCallback([]() {
        const TaskHandle_t handle = g_knx.taskHandle.load();  // set once at startup
        if (handle != nullptr) {
            xTaskNotifyGive(handle);
        }
    });

    // While the stack still has in-flight polled work (e.g. a group auto-response
    // progressing through its send state machine) we wake at this short cadence
    // to drive it to completion; when fully idle we instead sleep until the next
    // control tick or the next work notification, whichever comes first.
    const TickType_t kActiveWorkPollTicks =
        (pdMS_TO_TICKS(2) > 0) ? pdMS_TO_TICKS(2) : 1;

    // Boot-storm mitigation: hold the first full publish for a per-device random
    // delay so a bus-wide power-up doesn't make every device transmit at t=0.
    // The rate limiter above additionally spreads whatever burst does occur.
    const uint32_t startupJitterMs = esp_random() % (kStartupPublishMaxJitterMs + 1u);
    const TickType_t startupPublishReleaseTick =
        xTaskGetTickCount() + pdMS_TO_TICKS(startupJitterMs);
    bool startupDelayLogged = false;
#if defined(CONFIG_HABINARI_KNXIP_LINK_DIAGNOSTICS)
    uint64_t lastLinkReceived = 0;
    uint64_t lastLinkDropped = 0;
    uint32_t lastLinkPoolDropped = 0;
    TickType_t lastLinkLogTick = 0;
#endif

    // Startup is this task's stack peak, and it is now behind us. Report what
    // was left so kKnxServiceTaskStackSize can be retuned from measurement.
    KNX_LOGI(TAG,
             "Stack headroom after startup: %u of %u bytes free",
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
             static_cast<unsigned>(kKnxServiceTaskStackSize));

    reportStartResult(ESP_OK);

    for (;;) {
        app.loop();

#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
        if (g_knx.ipIdentityDirty.exchange(false)) {
            // The membership first, the identity second. The join is what makes
            // the device hear the group at all — it is established against
            // whichever interface exists when the routing socket opens, and on
            // a device that booted before the access point answered, that was
            // no interface. Publishing an address nobody can reach us on would
            // be the more visible half of the same event and the less useful.
            const auto rejoin = app.refreshIpMulticastMembership();
            if (rejoin.isError()) {
                KNX_LOGW(TAG, "Routing group rejoin failed: %d",
                         static_cast<int>(rejoin.error()));
            } else {
                KNX_LOGI(TAG, "Rejoined the routing group on the current interface");
            }
            publishIpIdentity(app, resolveMulticastGroup());
        }
#endif

        if (g_knx.controlTickPending.exchange(false)) {
            // The transmit policy is derived from the configured COV deltas and
            // repetition times, so it is refreshed when a tick may have changed
            // them — same cadence as before, just driven from the other side.
            Settings settings;
            {
                LockGuard lock(g_state.mutex);
                settings = g_state.hvac;
            }
            applyTransmitPolicies(app, settings);
            controlTickDue = true;
        }

        // Startup-delay gate: while closed nothing is published, so the first
        // full offer happens after the jitter delay. Sensor updates keep
        // accumulating in g_state meanwhile and go out together once it opens.
        const bool startupGateOpen =
            static_cast<int32_t>(xTaskGetTickCount() - startupPublishReleaseTick) >= 0;

        // Becoming Operational means the association table is now the project's
        // (fresh boot or a just-finished download), so this is the point at
        // which "which objects did ETS actually link?" can be answered.
        if (app.lifecycleState() == DeviceLifecycleState::Operational
            && previousLifecycle != DeviceLifecycleState::Operational) {
            reportUnlinkedPorts(app);
        }

        if (app.lifecycleState() == DeviceLifecycleState::Operational && !startupGateOpen) {
            if (!startupDelayLogged) {
                KNX_LOGI(TAG, "Holding initial KNX publish for %u ms (boot-storm mitigation)",
                         static_cast<unsigned>(startupJitterMs));
                startupDelayLogged = true;
            }
            // A programming-mode toggle (button) must still be honoured promptly.
            if (control::takeProgrammingModeToggleRequest()) {
                app.toggleProgrammingMode();
            }
        } else if (app.lifecycleState() == DeviceLifecycleState::Operational) {
            if (control::takeProgrammingModeToggleRequest()) {
                app.toggleProgrammingMode();
            }
            // Offered unconditionally once per control tick; the per-object
            // transmit policy decides what is actually worth a telegram. The
            // snapshot is taken only then: it copies the whole state under the
            // control mutex, and this loop also runs on every inbound frame.
            if (controlTickDue) {
                publishAllState(app, takePublishSnapshot());
                controlTickDue = false;
            }
        } else if (control::takeProgrammingModeToggleRequest()) {
            app.toggleProgrammingMode();
        }

#if defined(CONFIG_HABINARI_KNXIP_LINK_DIAGNOSTICS)
        // The question a capture taken on the network cannot answer: when the
        // device stops answering a client, did the telegram reach it at all?
        // A datagram count standing still while ETS is plainly transmitting is
        // a delivery problem below the stack; a rising drop count is this
        // device discarding what it did receive. Rate-limited to a second, and
        // silent while nothing moves, so it can be left on for a download.
        {
            const TickType_t nowTick = xTaskGetTickCount();
            if ((nowTick - lastLinkLogTick) >= kControlIntervalTicks) {
                const auto link = app.linkDiagnostics();
                if (link.mediumDatagramsReceived != lastLinkReceived
                    || link.mediumDatagramsDropped != lastLinkDropped
                    || link.rxPoolDropped != lastLinkPoolDropped) {
                    lastLinkReceived = link.mediumDatagramsReceived;
                    lastLinkDropped = link.mediumDatagramsDropped;
                    lastLinkPoolDropped = link.rxPoolDropped;
                    lastLinkLogTick = nowTick;
                    KNX_LOGI(TAG,
                             "Routing link: %llu datagrams in (%llu dropped, %u ms since "
                             "the last), data link: %u frames, %u filtered, %u pool-dropped, "
                             "%u undecodable",
                             static_cast<unsigned long long>(link.mediumDatagramsReceived),
                             static_cast<unsigned long long>(link.mediumDatagramsDropped),
                             static_cast<unsigned>(link.msSinceLastInbound),
                             static_cast<unsigned>(link.rxFrames),
                             static_cast<unsigned>(link.rxFilterDropped),
                             static_cast<unsigned>(link.rxPoolDropped),
                             static_cast<unsigned>(link.rxDecodeFailed));
                }
            }
        }
#endif

        previousLifecycle = app.lifecycleState();

        // Park until the stack signals new work or the control task announces
        // the next cycle — no fixed busy-poll. If polled work is still in
        // flight, wake at the short active cadence to service it to completion;
        // otherwise sleep for a whole control interval. Either a work
        // notification (an inbound frame) or the tick notification cuts the
        // sleep short, and both latch, so neither is ever missed. The interval
        // is a ceiling now rather than a schedule — the control task owns the
        // cadence, and this only bounds how long we sleep if it stops.
        const TickType_t blockTicks = app.ownerWorkHint().hasImmediateWork()
            ? kActiveWorkPollTicks
            : kControlIntervalTicks;
        (void)ulTaskNotifyTake(pdTRUE, blockTicks);
    }
}

} // namespace

// ---------------------------------------------------------------------------
// The adapter descriptor — this file's entire public surface.
//
// It used to be five knx_service_* entry points that main.c called directly,
// including NVS init and factory reset, which is what made KNX structurally
// mandatory. What is left is the mapping and nothing else.
// ---------------------------------------------------------------------------

namespace {

esp_err_t knxAdapterStart()
{
    if (g_knx.started) {
        return ESP_OK;
    }
    // NVS and the control state are already up: control_service_start() ran
    // first, which is the ordering protocol_adapters_start_all() guarantees.
    if (g_state.mutex == nullptr) {
        KNX_LOGE(TAG, "Control service is not running");
        return ESP_ERR_INVALID_STATE;
    }

#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
    // Registered before association is attempted, so a device with the wrong
    // credentials — or on the wrong routing group — is still reachable from the
    // service channel to be corrected. That is the whole point of the registry.
    const esp_err_t wifiCfg = wifi_service_register_config();
    if (wifiCfg != ESP_OK) {
        KNX_LOGW(TAG, "Wi-Fi settings not exposed out of band: %s", esp_err_to_name(wifiCfg));
    }
    const esp_err_t knxCfg = device_config_register(
        kKnxIpConfigItems, sizeof(kKnxIpConfigItems) / sizeof(*kKnxIpConfigItems));
    if (knxCfg != ESP_OK) {
        KNX_LOGW(TAG, "KNXnet/IP settings not exposed out of band: %s", esp_err_to_name(knxCfg));
    }

    // Not fatal. An unprovisioned device still boots, still senses, still
    // controls, and can still be given credentials over BLE — which is exactly
    // the situation a device out of the box is in.
    const esp_err_t wifiErr = wifi_service_start();
    if (wifiErr != ESP_OK) {
        KNX_LOGW(TAG, "Wi-Fi not started (%s) — KNXnet/IP will be unreachable until "
                      "credentials are provisioned", esp_err_to_name(wifiErr));
    }
#endif

    // Wait for the task to say whether the product actually started. Creating
    // the task used to count as success, so a stack that then failed to come
    // up deleted its task in silence and the required-adapter check never saw
    // it. The task reports through this task's notification value.
    (void)xTaskNotifyStateClear(nullptr);
    (void)ulTaskNotifyValueClear(nullptr, UINT32_MAX);
    g_knx.startWaiter.store(xTaskGetCurrentTaskHandle());
    const BaseType_t created = xTaskCreate(knxServiceTask, "knx_service",
                                           kKnxServiceTaskStackSize,
                                           nullptr, 8, nullptr);
    if (created != pdPASS) {
        g_knx.startWaiter.store(nullptr);
        return ESP_ERR_NO_MEM;
    }

    uint32_t result = 0;
    TickType_t wait = pdMS_TO_TICKS(kKnxStartTimeoutMs);
    while (xTaskNotifyWait(0, UINT32_MAX, &result, wait) != pdTRUE) {
        if (g_knx.startWaiter.exchange(nullptr) == nullptr) {
            // The task took the waiter just before we gave up; its report is
            // on its way, so take it rather than return without it.
            wait = portMAX_DELAY;
            continue;
        }
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
        if (g_knx.waitingForNetwork.load()) {
            KNX_LOGW(TAG, "KNX start-up is waiting for Wi-Fi; carrying on with boot. "
                          "A start-up failure from here on is only logged");
            g_knx.started = true;
            return ESP_OK;
        }
#endif
        KNX_LOGE(TAG, "KNX did not report start-up within %u ms",
                 static_cast<unsigned>(kKnxStartTimeoutMs));
        return ESP_ERR_TIMEOUT;
    }
    if (static_cast<esp_err_t>(result) != ESP_OK) {
        return static_cast<esp_err_t>(result);
    }

    g_knx.started = true;
    return ESP_OK;
}

// Called from the control task. Sets a flag and wakes the service loop; it must
// not do any KNX work itself, because the control task has no business blocking
// on a bus with a 104 us bit cell.
void knxAdapterOnControlTick(uint32_t)
{
    g_knx.controlTickPending.store(true);
    const TaskHandle_t handle = g_knx.taskHandle.load();
    if (handle != nullptr) {
        xTaskNotifyGive(handle);
    }
}

bool knxAdapterIdentifyActive()
{
    if (g_state.mutex == nullptr) {
        return false;
    }
    LockGuard lock(g_state.mutex);
    return g_state.programmingModeActive;
}

}  // namespace

extern "C" const protocol_adapter_t knx_protocol_adapter = {
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
    .name = "knx-ip",
#else
    .name = "knx-tp1",
#endif
    .start = knxAdapterStart,
    .on_control_tick = knxAdapterOnControlTick,
    // No on_sensor_data hook: measurements are published from the control tick
    // under the transmit policy, which is what decides whether a new reading is
    // worth a telegram. Publishing straight off the sensor task would bypass it.
    .on_sensor_data = nullptr,
    .identify_active = knxAdapterIdentifyActive,
    // KNX owns programming mode wherever it is compiled in. The flag lives in
    // the stack's device object, ETS can set it over the bus, and the stack is
    // only safe to touch from the KNX task — so the control service must not
    // toggle it behind the stack's back. This adapter consumes the button
    // request on its own task and reports the result back through
    // control_service_set_programming_mode().
    .owns_programming_mode = true,
    // Required: on a board wired to a TP1 bus, a device that silently fails to
    // join it is worse than one that refuses to boot and says why.
    //
    // The same holds on IP, and for the same reason — but note that "started"
    // there means the service task was created, not that the network exists.
    // A KNXnet/IP device that boots before its access point is a normal state,
    // not a failure, so it is not one this flag should turn into a dead boot.
    .required = true,
};
