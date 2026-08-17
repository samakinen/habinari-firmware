// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "board.h"
#include "control_core.hpp"
#include "control_service.hpp"
#include "device_identity.hpp"
#include "device_secret.hpp"
#include "hvac_control.hpp"
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
#include "knx/physical/bitbang_driver_timer_isr_espidf.hpp"
#include "knx/physical/physical_factory.hpp"
#include "knx/product/commissioned_product.hpp"
#include "knx/util/log.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <span>

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
static constexpr bool kEnableKnxDataSecure = true;
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
    TaskHandle_t taskHandle{nullptr};
    bool started{false};
    // Raised by the control task through the adapter's on_control_tick hook,
    // consumed by the service loop. Only ever set true from one context and
    // cleared from the other, so it needs no lock of its own.
    volatile bool controlTickPending{true};
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

void knxServiceTask(void *arg)
{
    (void)arg;

    // DEBUG shows per-frame bus traffic (RX/TX dumps, DL-ACK diagnostics,
    // connection events) — the development default while the stack is being
    // brought up. Switch to Info for production builds: INFO carries only
    // state changes (address, lifecycle, load state) and WARN/ERROR faults.
    knx::log::setLevel(knx::log::Level::Info);
    // knx::log routes through esp_log, whose runtime default level (Info)
    // would otherwise silently drop the Debug output enabled above.
    esp_log_level_set("*", ESP_LOG_INFO);
    // Suppress ModBus driver/infrastructure noise to focus on KNX application.
    // esp_log matches these tags exactly (no prefix matching), so every
    // component-specific tag needs its own entry.
    esp_log_level_set("mb_port.event", ESP_LOG_WARN);
    esp_log_level_set("mb_port.serial", ESP_LOG_WARN);
    esp_log_level_set("mb_port.timer", ESP_LOG_WARN);
    esp_log_level_set("mb_object.master", ESP_LOG_WARN);
    esp_log_level_set("mb_object.slave", ESP_LOG_WARN);
    esp_log_level_set("mb_transaction", ESP_LOG_WARN);
    esp_log_level_set("mb_transp.rtu_master", ESP_LOG_WARN);
    esp_log_level_set("mb_transp.rtu_slave", ESP_LOG_WARN);

    platform::Esp32Platform platform;

    auto physicalResult = createTp1Physical(platform);
    if (physicalResult.isError()) {
        KNX_LOGE(TAG, "TP1 physical init failed: %d", static_cast<int>(physicalResult.error()));
        vTaskDelete(nullptr);
        return;
    }

    // startCommissionedProduct() builds the runtime on the heap and hands back
    // an owning handle: at ~34 KB it cannot travel through a stack frame. The
    // bindings builder is still a stack object, so it stays scoped so its space
    // is released before the service loop runs.
    CommissionedProductHandle<std::remove_cvref_t<decltype(kHabinariProduct)>,
                              kDefaultBindingCapacity> appPtr;
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
        .onStateWrite<HabinariPort::OutsideTemperature>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.in.outsideTemperatureC = value;
            g_state.in.outsideTemperatureValid = true;
        })
        .onStateWrite<HabinariPort::OutsideHumidity>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.in.outsideHumidityPct = hvac_ns::clampf(value, 0.0f, 100.0f);
            g_state.in.outsideHumidityValid = true;
        })
        .onStateWrite<HabinariPort::FlowTemperature>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.in.flowTemperatureC = value;
            g_state.in.flowTemperatureValid = true;
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
            LockGuard lock(g_state.mutex);
            g_state.in.controllerOnOff = value;
        })
        .provideState<HabinariPort::ControllerOnOff>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.in.controllerOnOff;
        })
        .onStateWrite<HabinariPort::HvacMode>([](Dpt20Mode value) {
            LockGuard lock(g_state.mutex);
            g_state.in.hvacOperatingMode = static_cast<hvac_ns::OperatingPreset>(value);
        })
        .provideState<HabinariPort::HvacModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<Dpt20Mode>(g_state.out.activePreset);
        })
        // The bus objects carry DPT 20.105 (HVACContrMode) code points; the
        // internal enum is compact. Map at the binding boundary only.
        .onStateWrite<HabinariPort::ContrMode>([](uint8_t value) {
            LockGuard lock(g_state.mutex);
            g_state.in.controllerMode = hvac_ns::controllerModeFromContrMode(value);
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
        .onStateWrite<HabinariPort::SetpointBase>([](float value) {
            LockGuard lock(g_state.mutex);
            const float clamped =
                hvac_ns::clampf(value, g_state.hvac.minSetpointC, g_state.hvac.maxSetpointC);
            if (std::fabs(g_state.hvac.comfortHeatingSetpointC - clamped) < 0.01f) {
                return;
            }
            g_state.hvac.comfortHeatingSetpointC = clamped;
            ESP_LOGI(TAG, "Base (comfort heating) setpoint updated to %.2f C", clamped);
        })
        .provideState<HabinariPort::SetpointBase>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.comfortHeatingSetpointC;
        })
        .onStateWrite<HabinariPort::SetpointShift>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.in.setpointShiftK = value;
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
            LockGuard lock(g_state.mutex);
            g_state.in.windowOpen = value;
            g_state.in.windowStatusKnown = true;
        })
        .onStateWrite<HabinariPort::PresenceStatus>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.presence = value;
            g_state.in.presenceKnown = true;
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
            LockGuard lock(g_state.mutex);
            if (std::fabs(g_state.hvac.ventilationSetpointPpm - value) < 0.5f) {
                return;
            }
            g_state.hvac.ventilationSetpointPpm = value;
            ESP_LOGI(TAG, "CO2 setpoint updated to %.0f ppm", value);
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
            LockGuard lock(g_state.mutex);
            g_state.in.ventilationMode = static_cast<hvac_ns::VentilationMode>(value);
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
            if (!value) {
                return;
            }
            LockGuard lock(g_state.mutex);
            g_state.acknowledgeAlarmsRequested = true;
        })

        // ---- ETS parameters ----
        // Fractional parameters arrive as Dpt9Float (KNX 2-byte half-float,
        // implicitly convertible to float); counts and seconds as uint16_t;
        // enumerations and percentages as uint8_t.
        .onParameterChanged<HabinariParameter::MeasurementHeartbeatSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heartbeatSeconds = v;
        })
        .onParameterChanged<HabinariParameter::MeasurementMinRepTimeSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.minRepTimeSeconds = v;
        })
        .onParameterChanged<HabinariParameter::RoomTemperatureOffset>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.roomTemperatureOffsetK = v;
        })
        .onParameterChanged<HabinariParameter::RoomTemperatureCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.roomTemperatureCovK = v;
        })
        .onParameterChanged<HabinariParameter::RoomHumidityOffset>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.roomHumidityOffsetPct = v;
        })
        .onParameterChanged<HabinariParameter::RoomHumidityCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.roomHumidityCovPct = v;
        })
        .onParameterChanged<HabinariParameter::Co2Cov>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.co2CovPpm = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::PressureCov>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.pressureCovPa = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::AirQualityCov>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.airQualityCovIndex = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::FloorTemperatureOffset>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorTemperatureOffsetK = v;
        })
        .onParameterChanged<HabinariParameter::FloorTemperatureCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorTemperatureCovK = v;
        })
        .onParameterChanged<HabinariParameter::FloorHumidityCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorHumidityCovPct = v;
        })
        .onParameterChanged<HabinariParameter::DerivedValueCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.derivedCov = v;
        })
        .onParameterChanged<HabinariParameter::AltitudeM>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.altitudeM = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::ControllerDefaultEnable>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.controllerDefaultEnable = (v != 0);
        })
        .onParameterChanged<HabinariParameter::DefaultHvacOperatingMode>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.defaultHvacOperatingMode = static_cast<hvac_ns::OperatingPreset>(v);
        })
        .onParameterChanged<HabinariParameter::DefaultControllerMode>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.defaultControllerMode = static_cast<hvac_ns::ControllerMode>(v);
        })
        .onParameterChanged<HabinariParameter::HeatingEnabled>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingEnabled = (v != 0);
        })
        .onParameterChanged<HabinariParameter::CoolingEnabled>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingEnabled = (v != 0);
        })
        .onParameterChanged<HabinariParameter::HeatCoolChangeoverMode>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatCoolChangeoverMode = static_cast<hvac_ns::HeatCoolChangeoverMode>(v);
        })
        .onParameterChanged<HabinariParameter::HeatCoolChangeoverPolarity>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatCoolChangeoverPolarityInverted = (v != 0);
        })
        .onParameterChanged<HabinariParameter::MinimumHeatCoolChangeoverSeconds>(
            [](uint16_t v) {
                LockGuard lock(g_state.mutex);
                g_state.hvac.minimumHeatCoolChangeoverSeconds = static_cast<float>(v);
            })
        .onParameterChanged<HabinariParameter::WindowOpenBehavior>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.windowOpenBehavior = static_cast<hvac_ns::WindowOpenBehavior>(v);
        })
        .onParameterChanged<HabinariParameter::PresenceBehavior>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.presenceBehavior = static_cast<hvac_ns::PresenceBehavior>(v);
        })
        .onParameterChanged<HabinariParameter::SensorFaultBehavior>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.sensorFaultBehavior = static_cast<hvac_ns::SensorFaultBehavior>(v);
        })
        .onParameterChanged<HabinariParameter::ComfortHeatingSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.comfortHeatingSetpointC = v;
        })
        .onParameterChanged<HabinariParameter::StandbyHeatingReduction>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.standbyHeatingReductionK = v;
        })
        .onParameterChanged<HabinariParameter::EconomyHeatingReduction>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.economyHeatingReductionK = v;
        })
        .onParameterChanged<HabinariParameter::ProtectionHeatingSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.protectionHeatingSetpointC = v;
        })
        .onParameterChanged<HabinariParameter::CoolingDeadband>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingDeadbandK = v;
        })
        .onParameterChanged<HabinariParameter::StandbyCoolingIncrease>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.standbyCoolingIncreaseK = v;
        })
        .onParameterChanged<HabinariParameter::EconomyCoolingIncrease>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.economyCoolingIncreaseK = v;
        })
        .onParameterChanged<HabinariParameter::ProtectionCoolingSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.protectionCoolingSetpointC = v;
        })
        .onParameterChanged<HabinariParameter::MinSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.minSetpointC = v;
        })
        .onParameterChanged<HabinariParameter::MaxSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxSetpointC = v;
        })
        .onParameterChanged<HabinariParameter::MaxSetpointShift>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxSetpointShiftK = v;
        })
        .onParameterChanged<HabinariParameter::HeatingControlAlgorithm>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(v);
        })
        .onParameterChanged<HabinariParameter::HeatingKp>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingKp = v;
        })
        .onParameterChanged<HabinariParameter::HeatingTiSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingTiSeconds = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::HeatingTdSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingTdSeconds = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::HeatingMinimumOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingMinOutputPercent = v;
        })
        .onParameterChanged<HabinariParameter::HeatingMaximumOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingMaxOutputPercent = v;
        })
        .onParameterChanged<HabinariParameter::CoolingControlAlgorithm>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(v);
        })
        .onParameterChanged<HabinariParameter::CoolingKp>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingKp = v;
        })
        .onParameterChanged<HabinariParameter::CoolingTiSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingTiSeconds = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::CoolingTdSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingTdSeconds = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::CoolingMinimumOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingMinOutputPercent = v;
        })
        .onParameterChanged<HabinariParameter::CoolingMaximumOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingMaxOutputPercent = v;
        })
        .onParameterChanged<HabinariParameter::ThermostatHysteresis>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.thermostatHysteresisC = v;
        })
        .onParameterChanged<HabinariParameter::BinaryDemandStrategy>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.binaryDemandStrategy = static_cast<hvac_ns::BinaryDemandStrategy>(v);
        })
        .onParameterChanged<HabinariParameter::BinaryDemandThresholdPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.binaryDemandThresholdPercent = v;
        })
        .onParameterChanged<HabinariParameter::FrostAlarmTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.frostAlarmTemperatureC = v;
        })
        .onParameterChanged<HabinariParameter::OverheatAlarmTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.overheatAlarmTemperatureC = v;
        })
        .onParameterChanged<HabinariParameter::MaxFloorTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxFloorTemperatureC = v;
        })
        .onParameterChanged<HabinariParameter::MinFloorTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.minFloorTemperatureC = v;
        })
        .onParameterChanged<HabinariParameter::FloorHysteresis>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorHysteresisK = v;
        })
        .onParameterChanged<HabinariParameter::FloorComfortOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorComfortOutputPercent = v;
        })
        .onParameterChanged<HabinariParameter::DewPointSurfaceSource>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.dewPointSurfaceSource = static_cast<hvac_ns::DewPointSurfaceSource>(v);
        })
        .onParameterChanged<HabinariParameter::DewPointMargin>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.dewPointMarginK = v;
        })
        .onParameterChanged<HabinariParameter::DewPointHysteresis>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.dewPointHysteresisK = v;
        })
        .onParameterChanged<HabinariParameter::BlockCoolingOnDewPointAlarm>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.blockCoolingOnDewPointAlarm = (v != 0);
        })
        .onParameterChanged<HabinariParameter::FloorMoistureThreshold>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorMoistureThresholdPct = v;
        })
        .onParameterChanged<HabinariParameter::FloorMoistureHysteresis>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorMoistureHysteresisPct = v;
        })
        .onParameterChanged<HabinariParameter::FloorMoistureExcess>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorMoistureExcessGm3 = v;
        })
        .onParameterChanged<HabinariParameter::VentilationSetpoint>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationSetpointPpm = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::VentilationBand>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationBandPpm = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::HumidityBoostThreshold>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.humidityBoostPct = v;
        })
        .onParameterChanged<HabinariParameter::HumidityBoostBand>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.humidityBandPct = v;
        })
        .onParameterChanged<HabinariParameter::VocBoostThreshold>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.vocThresholdIndex = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::VocBoostBand>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.vocBandIndex = static_cast<float>(v);
        })
        .onParameterChanged<HabinariParameter::VentilationBaseDemandPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationBaseDemandPercent = v;
        })
        .onParameterChanged<HabinariParameter::VentilationManualDemandPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationManualDemandPercent = v;
        })

        // ---- Sensor fusion and detection ----
        // These land in the fusion config rather than in Settings and are
        // pushed down to the acquisition side by the control tick.
        .onParameterChanged<HabinariParameter::SensorFilterSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.filter_tau_seconds = static_cast<float>(v);
            g_state.fusionConfigDirty = true;
        })
        .onParameterChanged<HabinariParameter::TemperatureCrossCheck>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.temperature_cross_check_k = v;
            g_state.fusionConfigDirty = true;
        })
        .onParameterChanged<HabinariParameter::HumidityCrossCheck>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.humidity_cross_check_pct = v;
            g_state.fusionConfigDirty = true;
        })
        .onParameterChanged<HabinariParameter::FireRateOfRise>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.fire_rate_of_rise_k_per_min = v;
            g_state.fusionConfigDirty = true;
        })
        .onParameterChanged<HabinariParameter::FireAbsoluteTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.fire_absolute_alarm_c = v;
            g_state.fusionConfigDirty = true;
        })
        .onParameterChanged<HabinariParameter::FireConfirmSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.fire_confirm_seconds = static_cast<float>(v);
            g_state.fusionConfigDirty = true;
        })
        .onParameterChanged<HabinariParameter::FireRequireAirQuality>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.fire_require_air_quality = (v != 0);
            g_state.fusionConfigDirty = true;
        })
        .onParameterChanged<HabinariParameter::Co2OccupancyEnabled>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.co2_occupancy_enabled = (v != 0);
            g_state.fusionConfigDirty = true;
        })
        .onParameterChanged<HabinariParameter::WindowDetectEnabled>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.fusion.window_detect_enabled = (v != 0);
            g_state.fusionConfigDirty = true;
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

    auto appResult = startCommissionedProduct(
        platform,
        kHabinariProduct,
        std::move(bindings),
        std::move(physicalResult.value()));

    if (appResult.isError()) {
        KNX_LOGE(TAG, "startCommissionedProduct failed: %d", static_cast<int>(appResult.error()));
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

    if (kEnableKnxDataSecure) {
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
            const bool provisionedKeyInForce = app.hasEtsToolKey();
            const auto secureResult = provisionedKeyInForce ? app.enableSecurityMode()
                                                            : app.applyEtsToolKey(toolKey);
            if (secureResult.isError()) {
                KNX_LOGE(TAG, "Data Secure: applyEtsToolKey failed: %d",
                         static_cast<int>(secureResult.error()));
            } else {
                if (provisionedKeyInForce) {
                    KNX_LOGW(TAG,
                             "KNX Data Secure ENABLED. Keeping the tool key already on the "
                             "device; the factory key below was not re-applied and is only "
                             "valid again after a factory reset.");
                } else {
                    KNX_LOGW(TAG,
                             "KNX Data Secure ENABLED. Tool key (%s): %s",
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

                // The certificate is the device's factory identity and is what
                // an installer enters into ETS, so it is logged on every boot
                // whether or not ETS has since installed a key of its own.
                if (hasSerialNumber) {
                    char certificate[habinari::identity::kCertificateBufferSize] = {};
                    formatDeviceCertificate(std::span<const uint8_t, 6>(serialNumber, 6),
                                            toolKey, certificate, sizeof(certificate));
                    KNX_LOGW(TAG,
                             "KNX Data Secure device certificate (enter in ETS): %s",
                             certificate);
                } else {
                    KNX_LOGW(TAG,
                             "KNX Data Secure device certificate unavailable: no serial number");
                }
            }
        }
    } else {
        KNX_LOGI(TAG, "KNX Data Secure disabled (plain commissioning)");
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
        g_knx.taskHandle = xTaskGetCurrentTaskHandle();
        // Seed engineering values from the ETS parameters. Every parameter has
        // an onParameterChanged binding above for live re-parameterisation;
        // this is the one-time load at startup, so the two lists must stay in
        // step.
        auto params = app.parameters();
        auto &s = g_state.hvac;

        s.heartbeatSeconds = params.get<HabinariParameter::MeasurementHeartbeatSeconds>();
        s.minRepTimeSeconds = params.get<HabinariParameter::MeasurementMinRepTimeSeconds>();
        s.roomTemperatureOffsetK = params.get<HabinariParameter::RoomTemperatureOffset>();
        s.roomTemperatureCovK = params.get<HabinariParameter::RoomTemperatureCov>();
        s.roomHumidityOffsetPct = params.get<HabinariParameter::RoomHumidityOffset>();
        s.roomHumidityCovPct = params.get<HabinariParameter::RoomHumidityCov>();
        s.co2CovPpm = static_cast<float>(params.get<HabinariParameter::Co2Cov>());
        s.pressureCovPa = static_cast<float>(params.get<HabinariParameter::PressureCov>());
        s.airQualityCovIndex = static_cast<float>(params.get<HabinariParameter::AirQualityCov>());
        s.floorTemperatureOffsetK = params.get<HabinariParameter::FloorTemperatureOffset>();
        s.floorTemperatureCovK = params.get<HabinariParameter::FloorTemperatureCov>();
        s.floorHumidityCovPct = params.get<HabinariParameter::FloorHumidityCov>();
        s.derivedCov = params.get<HabinariParameter::DerivedValueCov>();
        s.altitudeM = static_cast<float>(params.get<HabinariParameter::AltitudeM>());

        s.controllerDefaultEnable = params.get<HabinariParameter::ControllerDefaultEnable>() != 0;
        s.defaultHvacOperatingMode = static_cast<hvac_ns::OperatingPreset>(
            params.get<HabinariParameter::DefaultHvacOperatingMode>());
        s.defaultControllerMode = static_cast<hvac_ns::ControllerMode>(
            params.get<HabinariParameter::DefaultControllerMode>());
        s.heatingEnabled = params.get<HabinariParameter::HeatingEnabled>() != 0;
        s.coolingEnabled = params.get<HabinariParameter::CoolingEnabled>() != 0;
        s.heatCoolChangeoverMode = static_cast<hvac_ns::HeatCoolChangeoverMode>(
            params.get<HabinariParameter::HeatCoolChangeoverMode>());
        s.heatCoolChangeoverPolarityInverted =
            params.get<HabinariParameter::HeatCoolChangeoverPolarity>() != 0;
        s.minimumHeatCoolChangeoverSeconds = static_cast<float>(
            params.get<HabinariParameter::MinimumHeatCoolChangeoverSeconds>());
        s.windowOpenBehavior = static_cast<hvac_ns::WindowOpenBehavior>(
            params.get<HabinariParameter::WindowOpenBehavior>());
        s.presenceBehavior = static_cast<hvac_ns::PresenceBehavior>(
            params.get<HabinariParameter::PresenceBehavior>());
        s.sensorFaultBehavior = static_cast<hvac_ns::SensorFaultBehavior>(
            params.get<HabinariParameter::SensorFaultBehavior>());

        s.comfortHeatingSetpointC = params.get<HabinariParameter::ComfortHeatingSetpoint>();
        s.standbyHeatingReductionK = params.get<HabinariParameter::StandbyHeatingReduction>();
        s.economyHeatingReductionK = params.get<HabinariParameter::EconomyHeatingReduction>();
        s.protectionHeatingSetpointC = params.get<HabinariParameter::ProtectionHeatingSetpoint>();
        s.coolingDeadbandK = params.get<HabinariParameter::CoolingDeadband>();
        s.standbyCoolingIncreaseK = params.get<HabinariParameter::StandbyCoolingIncrease>();
        s.economyCoolingIncreaseK = params.get<HabinariParameter::EconomyCoolingIncrease>();
        s.protectionCoolingSetpointC = params.get<HabinariParameter::ProtectionCoolingSetpoint>();
        s.minSetpointC = params.get<HabinariParameter::MinSetpoint>();
        s.maxSetpointC = params.get<HabinariParameter::MaxSetpoint>();
        s.maxSetpointShiftK = params.get<HabinariParameter::MaxSetpointShift>();

        s.heatingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(
            params.get<HabinariParameter::HeatingControlAlgorithm>());
        s.heatingKp = params.get<HabinariParameter::HeatingKp>();
        s.heatingTiSeconds = static_cast<float>(params.get<HabinariParameter::HeatingTiSeconds>());
        s.heatingTdSeconds = static_cast<float>(params.get<HabinariParameter::HeatingTdSeconds>());
        s.heatingMinOutputPercent = params.get<HabinariParameter::HeatingMinimumOutputPercent>();
        s.heatingMaxOutputPercent = params.get<HabinariParameter::HeatingMaximumOutputPercent>();
        s.coolingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(
            params.get<HabinariParameter::CoolingControlAlgorithm>());
        s.coolingKp = params.get<HabinariParameter::CoolingKp>();
        s.coolingTiSeconds = static_cast<float>(params.get<HabinariParameter::CoolingTiSeconds>());
        s.coolingTdSeconds = static_cast<float>(params.get<HabinariParameter::CoolingTdSeconds>());
        s.coolingMinOutputPercent = params.get<HabinariParameter::CoolingMinimumOutputPercent>();
        s.coolingMaxOutputPercent = params.get<HabinariParameter::CoolingMaximumOutputPercent>();
        s.thermostatHysteresisC = params.get<HabinariParameter::ThermostatHysteresis>();
        s.binaryDemandStrategy = static_cast<hvac_ns::BinaryDemandStrategy>(
            params.get<HabinariParameter::BinaryDemandStrategy>());
        s.binaryDemandThresholdPercent =
            params.get<HabinariParameter::BinaryDemandThresholdPercent>();
        s.frostAlarmTemperatureC = params.get<HabinariParameter::FrostAlarmTemperature>();
        s.overheatAlarmTemperatureC = params.get<HabinariParameter::OverheatAlarmTemperature>();

        s.maxFloorTemperatureC = params.get<HabinariParameter::MaxFloorTemperature>();
        s.minFloorTemperatureC = params.get<HabinariParameter::MinFloorTemperature>();
        s.floorHysteresisK = params.get<HabinariParameter::FloorHysteresis>();
        s.floorComfortOutputPercent = params.get<HabinariParameter::FloorComfortOutputPercent>();

        s.dewPointSurfaceSource = static_cast<hvac_ns::DewPointSurfaceSource>(
            params.get<HabinariParameter::DewPointSurfaceSource>());
        s.dewPointMarginK = params.get<HabinariParameter::DewPointMargin>();
        s.dewPointHysteresisK = params.get<HabinariParameter::DewPointHysteresis>();
        s.blockCoolingOnDewPointAlarm =
            params.get<HabinariParameter::BlockCoolingOnDewPointAlarm>() != 0;

        s.floorMoistureThresholdPct = params.get<HabinariParameter::FloorMoistureThreshold>();
        s.floorMoistureHysteresisPct = params.get<HabinariParameter::FloorMoistureHysteresis>();
        s.floorMoistureExcessGm3 = params.get<HabinariParameter::FloorMoistureExcess>();

        s.ventilationSetpointPpm =
            static_cast<float>(params.get<HabinariParameter::VentilationSetpoint>());
        s.ventilationBandPpm =
            static_cast<float>(params.get<HabinariParameter::VentilationBand>());
        s.humidityBoostPct = params.get<HabinariParameter::HumidityBoostThreshold>();
        s.humidityBandPct = params.get<HabinariParameter::HumidityBoostBand>();
        s.vocThresholdIndex =
            static_cast<float>(params.get<HabinariParameter::VocBoostThreshold>());
        s.vocBandIndex = static_cast<float>(params.get<HabinariParameter::VocBoostBand>());
        s.ventilationBaseDemandPercent =
            params.get<HabinariParameter::VentilationBaseDemandPercent>();
        s.ventilationManualDemandPercent =
            params.get<HabinariParameter::VentilationManualDemandPercent>();

        // Fusion tuning: start from the compiled-in defaults so the fields the
        // ETS product does not expose (staleness, inter-sensor offsets) are
        // populated, then overlay the ETS parameters that it does.
        sensor_fusion_default_config(&g_state.fusion);
        g_state.fusion.filter_tau_seconds =
            static_cast<float>(params.get<HabinariParameter::SensorFilterSeconds>());
        g_state.fusion.temperature_cross_check_k =
            params.get<HabinariParameter::TemperatureCrossCheck>();
        g_state.fusion.humidity_cross_check_pct =
            params.get<HabinariParameter::HumidityCrossCheck>();
        g_state.fusion.fire_rate_of_rise_k_per_min =
            params.get<HabinariParameter::FireRateOfRise>();
        g_state.fusion.fire_absolute_alarm_c =
            params.get<HabinariParameter::FireAbsoluteTemperature>();
        g_state.fusion.fire_confirm_seconds =
            static_cast<float>(params.get<HabinariParameter::FireConfirmSeconds>());
        g_state.fusion.fire_require_air_quality =
            params.get<HabinariParameter::FireRequireAirQuality>() != 0;
        g_state.fusion.co2_occupancy_enabled =
            params.get<HabinariParameter::Co2OccupancyEnabled>() != 0;
        g_state.fusion.window_detect_enabled =
            params.get<HabinariParameter::WindowDetectEnabled>() != 0;
        g_state.fusionConfigDirty = true;

        // Seed the runtime mode/state inputs from their ETS defaults.
        g_state.in.controllerOnOff = s.controllerDefaultEnable;
        g_state.in.hvacOperatingMode = s.defaultHvacOperatingMode;
        g_state.in.controllerMode = s.defaultControllerMode;
    }

    KNX_LOGI(TAG, "ETS-commissionable TP1 sensor bridge started");

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
        const TaskHandle_t handle = g_knx.taskHandle;  // set once at startup
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
    const uint32_t startupJitterMs =
        (kStartupPublishMaxJitterMs > 0) ? (esp_random() % (kStartupPublishMaxJitterMs + 1u)) : 0u;
    const TickType_t startupPublishReleaseTick =
        xTaskGetTickCount() + pdMS_TO_TICKS(startupJitterMs);
    bool startupDelayLogged = false;

    // Startup is this task's stack peak, and it is now behind us. Report what
    // was left so kKnxServiceTaskStackSize can be retuned from measurement.
    KNX_LOGI(TAG,
             "Stack headroom after startup: %u of %u bytes free",
             static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
             static_cast<unsigned>(kKnxServiceTaskStackSize));

    for (;;) {
        app.loop();

        if (g_knx.controlTickPending) {
            g_knx.controlTickPending = false;
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
            const PublishSnapshot snapshot = takePublishSnapshot();
            // Offered unconditionally once per control tick; the per-object
            // transmit policy decides what is actually worth a telegram.
            if (controlTickDue) {
                publishAllState(app, snapshot);
                controlTickDue = false;
            }
        } else if (control::takeProgrammingModeToggleRequest()) {
            app.toggleProgrammingMode();
        }

        previousLifecycle = app.lifecycleState();

        // Park until the stack signals new work or the control task announces
        // the next cycle — no fixed busy-poll. If polled work is still in
        // flight, wake at the short active cadence to service it to completion;
        // otherwise sleep for a whole control interval. Either a work
        // notification (an inbound frame) or the tick notification cuts the
        // sleep short, and both latch, so neither is ever missed. The interval
        // is a ceiling now rather than a schedule — the control task owns the
        // cadence, and this only bounds how long we sleep if it stops.
        TickType_t blockTicks = app.ownerWorkHint().hasImmediateWork()
            ? kActiveWorkPollTicks
            : kControlIntervalTicks;
        if (blockTicks == 0) {
            blockTicks = 1;  // control tick already due — take it on the next tick
        }
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

    const BaseType_t created = xTaskCreate(knxServiceTask, "knx_service",
                                           kKnxServiceTaskStackSize, nullptr, 8, nullptr);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    g_knx.started = true;
    return ESP_OK;
}

// Called from the control task. Sets a flag and wakes the service loop; it must
// not do any KNX work itself, because the control task has no business blocking
// on a bus with a 104 us bit cell.
void knxAdapterOnControlTick(uint32_t)
{
    g_knx.controlTickPending = true;
    const TaskHandle_t handle = g_knx.taskHandle;
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
    .name = "knx-tp1",
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
    .required = true,
};
