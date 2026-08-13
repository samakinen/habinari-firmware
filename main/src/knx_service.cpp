#include "knx_service.h"

#include "board.h"
#include "device_identity.hpp"
#include "device_secret.hpp"
#include "hvac_control.hpp"
#include "knx_product.hpp"

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
#include "nvs_flash.h"

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
using namespace sensor_board_knx;

namespace {

static constexpr const char *TAG = "knx_service";
// Startup, not the service loop, sets this task's stack peak. The commissioned
// runtime is ~34 KB for this product and is built directly on the heap by
// startCommissionedProduct(), so it never crosses this stack; what remains here
// is the bindings builder (~5.7 KB, plus the by-value copy handed to the start
// call) and the BAU init call tree with NVS/crypto/logging under it. Measured
// frames after the heap-handle change: knxServiceTask 12,352 B +
// startCommissionedProduct 96 B, against 85,712 B when the runtime was still
// returned by value — which overflowed even a 64 KB stack.
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
using sensor_board::identity::formatDeviceCertificate;
using sensor_board::identity::toHex;

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
            sensor_board::secret::resolveFdsk(std::span<const uint8_t, 6>(serial, 6));
        if (resolved.fdskValid) {
            key = resolved.fdsk;
            source = ToolKeySource::EfuseDerived;
            return true;
        }
    }
    source = ToolKeySource::NvsFallback;
    return loadOrCreateToolKey(key, created);
}

namespace hvac_ns = sensor_board::hvac;

// ETS-configurable tuning (see hvac_control.hpp). Snapshotted by the control
// tick each second, so parameter changes apply within one tick.
struct HvacSettings {
    // --- Measurement publishing (KNX sensor FB Heartbeat/MinRepTime/COV) ---
    uint32_t heartbeatSeconds{kDefaultMeasurementHeartbeatSeconds};
    uint32_t minRepTimeSeconds{kDefaultMeasurementMinRepTimeSeconds};
    float roomTemperatureOffsetK{kDefaultRoomTemperatureOffsetK};
    float roomTemperatureCovK{kDefaultRoomTemperatureCovK};
    float roomHumidityOffsetPct{kDefaultRoomHumidityOffsetPct};
    float roomHumidityCovPct{kDefaultRoomHumidityCovPct};
    float co2CovPpm{static_cast<float>(kDefaultCo2CovPpm)};
    float pressureCovPa{static_cast<float>(kDefaultPressureCovPa)};
    float airQualityCovIndex{static_cast<float>(kDefaultAirQualityCovIndex)};
    float floorTemperatureOffsetK{kDefaultFloorTemperatureOffsetK};
    float floorTemperatureCovK{kDefaultFloorTemperatureCovK};
    float floorHumidityCovPct{kDefaultFloorHumidityCovPct};
    float derivedCov{kDefaultDerivedCovK};
    float altitudeM{static_cast<float>(kDefaultAltitudeM)};

    // --- Room control: general ---
    bool controllerDefaultEnable{kDefaultControllerEnable != 0};
    hvac_ns::OperatingPreset defaultHvacOperatingMode{
        static_cast<hvac_ns::OperatingPreset>(kDefaultHvacOperatingMode)};
    hvac_ns::ControllerMode defaultControllerMode{
        static_cast<hvac_ns::ControllerMode>(kDefaultControllerMode)};
    bool heatingEnabled{kDefaultHeatingEnabled != 0};
    bool coolingEnabled{kDefaultCoolingEnabled != 0};
    hvac_ns::HeatCoolChangeoverMode heatCoolChangeoverMode{
        static_cast<hvac_ns::HeatCoolChangeoverMode>(kDefaultHeatCoolChangeoverMode)};
    bool heatCoolChangeoverPolarityInverted{kDefaultHeatCoolChangeoverPolarity != 0};
    float minimumHeatCoolChangeoverSeconds{
        static_cast<float>(kDefaultMinimumHeatCoolChangeoverSeconds)};
    hvac_ns::WindowOpenBehavior windowOpenBehavior{
        static_cast<hvac_ns::WindowOpenBehavior>(kDefaultWindowOpenBehavior)};
    hvac_ns::PresenceBehavior presenceBehavior{
        static_cast<hvac_ns::PresenceBehavior>(kDefaultPresenceBehavior)};
    hvac_ns::SensorFaultBehavior sensorFaultBehavior{
        static_cast<hvac_ns::SensorFaultBehavior>(kDefaultSensorFaultBehavior)};

    // --- Setpoint ladder ---
    float comfortHeatingSetpointC{kDefaultComfortHeatingSetpointC};
    float standbyHeatingReductionK{kDefaultStandbyHeatingReductionK};
    float economyHeatingReductionK{kDefaultEconomyHeatingReductionK};
    float protectionHeatingSetpointC{kDefaultProtectionHeatingSetpointC};
    float coolingDeadbandK{kDefaultCoolingDeadbandK};
    float standbyCoolingIncreaseK{kDefaultStandbyCoolingIncreaseK};
    float economyCoolingIncreaseK{kDefaultEconomyCoolingIncreaseK};
    float protectionCoolingSetpointC{kDefaultProtectionCoolingSetpointC};
    float minSetpointC{kDefaultMinSetpointC};
    float maxSetpointC{kDefaultMaxSetpointC};
    float maxSetpointShiftK{kDefaultMaxSetpointShiftK};

    // --- Heating / cooling loops ---
    hvac_ns::ControlAlgorithm heatingControlAlgorithm{
        static_cast<hvac_ns::ControlAlgorithm>(kDefaultHeatingControlAlgorithm)};
    float heatingKp{kDefaultHeatingKp};
    float heatingTiSeconds{static_cast<float>(kDefaultHeatingTiSeconds)};
    float heatingTdSeconds{static_cast<float>(kDefaultHeatingTdSeconds)};
    uint8_t heatingMinOutputPercent{kDefaultHeatingMinimumOutputPercent};
    uint8_t heatingMaxOutputPercent{kDefaultHeatingMaximumOutputPercent};
    hvac_ns::ControlAlgorithm coolingControlAlgorithm{
        static_cast<hvac_ns::ControlAlgorithm>(kDefaultCoolingControlAlgorithm)};
    float coolingKp{kDefaultCoolingKp};
    float coolingTiSeconds{static_cast<float>(kDefaultCoolingTiSeconds)};
    float coolingTdSeconds{static_cast<float>(kDefaultCoolingTdSeconds)};
    uint8_t coolingMinOutputPercent{kDefaultCoolingMinimumOutputPercent};
    uint8_t coolingMaxOutputPercent{kDefaultCoolingMaximumOutputPercent};
    float thermostatHysteresisC{kDefaultThermostatHysteresisC};
    hvac_ns::BinaryDemandStrategy binaryDemandStrategy{
        static_cast<hvac_ns::BinaryDemandStrategy>(kDefaultBinaryDemandStrategy)};
    uint8_t binaryDemandThresholdPercent{kDefaultBinaryDemandThresholdPercent};
    float frostAlarmTemperatureC{kDefaultFrostAlarmTemperatureC};
    float overheatAlarmTemperatureC{kDefaultOverheatAlarmTemperatureC};

    // --- Floor temperature ---
    float maxFloorTemperatureC{kDefaultMaxFloorTemperatureC};
    float minFloorTemperatureC{kDefaultMinFloorTemperatureC};
    float floorHysteresisK{kDefaultFloorHysteresisK};
    uint8_t floorComfortOutputPercent{kDefaultFloorComfortOutputPercent};

    // --- Condensation protection ---
    hvac_ns::DewPointSurfaceSource dewPointSurfaceSource{
        static_cast<hvac_ns::DewPointSurfaceSource>(kDefaultDewPointSurfaceSource)};
    float dewPointMarginK{kDefaultDewPointMarginK};
    float dewPointHysteresisK{kDefaultDewPointHysteresisK};
    bool blockCoolingOnDewPointAlarm{kDefaultBlockCoolingOnDewPointAlarm != 0};

    // --- Slab moisture ---
    float floorMoistureThresholdPct{kDefaultFloorMoistureThresholdPct};
    float floorMoistureHysteresisPct{kDefaultFloorMoistureHysteresisPct};
    float floorMoistureExcessGm3{kDefaultFloorMoistureExcessGm3};

    // --- Ventilation / air quality ---
    float ventilationSetpointPpm{static_cast<float>(kDefaultVentilationSetpointPpm)};
    float ventilationBandPpm{static_cast<float>(kDefaultVentilationBandPpm)};
    float humidityBoostPct{kDefaultHumidityBoostPct};
    float humidityBandPct{kDefaultHumidityBandPct};
    float vocThresholdIndex{static_cast<float>(kDefaultVocThresholdIndex)};
    float vocBandIndex{static_cast<float>(kDefaultVocBandIndex)};
    uint8_t ventilationBaseDemandPercent{kDefaultVentilationBaseDemandPercent};
    uint8_t ventilationManualDemandPercent{kDefaultVentilationManualDemandPercent};
};

// Everything the control tick computes and the bus can read back. Kept as one
// struct so provideState (read requests, arbitrary thread) and the publish path
// see exactly the same values.
struct ControlOutputs {
    // Derived measurements.
    float roomDewPointC{0.0f};
    float roomAbsoluteHumidityGm3{0.0f};
    float floorAbsoluteHumidityGm3{0.0f};
    float seaLevelPressurePa{0.0f};
    float dewPointMarginK{0.0f};
    bool dewPointAlarm{false};
    bool floorMoistureAlarm{false};
    bool freeCoolingAvailable{false};
    bool freeDryingAvailable{false};

    // Thermostat.
    bool heatingRequest{false};
    bool coolingRequest{false};
    uint8_t heatingControlPercent{0};
    uint8_t coolingControlPercent{0};
    bool floorLimitActive{false};
    bool floorComfortActive{false};
    bool heatCoolModeHeating{true};  // DPT 1.100: 1 = heat
    bool enableHeat{false};
    bool enableCool{false};
    float activeSetpointC{kDefaultComfortHeatingSetpointC};
    float heatingSetpointC{kDefaultComfortHeatingSetpointC};
    float coolingSetpointC{kDefaultComfortHeatingSetpointC + kDefaultCoolingDeadbandK};
    float setpointShiftFeedbackK{0.0f};
    hvac_ns::OperatingPreset activePreset{hvac_ns::OperatingPreset::Comfort};
    hvac_ns::ControllerMode activeControllerMode{hvac_ns::ControllerMode::Auto};
    uint16_t controllerStatus{0};  // DPT 22.101 StatusRHCC

    // Ventilation.
    uint8_t ventilationDemandPercent{0};
    hvac_ns::VentilationLevel ventilationLevel{hvac_ns::VentilationLevel::Off};
    bool ventilationBoostRequest{false};
    bool dehumidifyRequest{false};
    uint16_t airQualityStatus{0};

    // Diagnostics.
    bool deviceFault{false};
    uint8_t roomSensorStatus{0};
    uint8_t floorProbeStatus{0};
    uint8_t airQualitySensorStatus{0};
};

struct SharedState {
    SemaphoreHandle_t mutex{nullptr};
    TaskHandle_t taskHandle{nullptr};
    sensor_data_t latestSensorData{};
    uint32_t availableMask{0};
    bool hasSensorData{false};
    bool started{false};
    bool toggleProgrammingModeRequested{false};
    HvacSettings hvac{};

    // Bus-writable inputs, mirrored back via provideState.
    bool controllerOnOff{true};
    hvac_ns::OperatingPreset hvacOperatingMode{hvac_ns::OperatingPreset::Comfort};
    hvac_ns::ControllerMode controllerMode{hvac_ns::ControllerMode::Auto};
    bool changeOverStatus{false};
    bool windowOpen{false};
    bool presence{false};
    bool presenceKnown{false};  // true once a presence telegram has been received
    bool switchHeat{true};
    bool switchCool{true};
    bool externalDewPointAlarm{false};
    float setpointShiftK{0.0f};
    hvac_ns::VentilationMode ventilationMode{hvac_ns::VentilationMode::Auto};

    // Neighbour-device inputs. Each stays invalid until a telegram arrives, so
    // an unlinked object never fabricates an outside temperature of 0 °C.
    float outsideTemperatureC{0.0f};
    bool outsideTemperatureValid{false};
    float outsideHumidityPct{0.0f};
    bool outsideHumidityValid{false};
    float flowTemperatureC{0.0f};
    bool flowTemperatureValid{false};

    ControlOutputs out{};
    knx_programming_mode_callback_t programmingModeCallback{nullptr};
};

SharedState g_state;

class LockGuard {
public:
    explicit LockGuard(SemaphoreHandle_t mutex) : mutex_(mutex)
    {
        if (mutex_ != nullptr) {
            xSemaphoreTake(mutex_, portMAX_DELAY);
        }
    }

    ~LockGuard()
    {
        if (mutex_ != nullptr) {
            xSemaphoreGive(mutex_);
        }
    }

    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

private:
    SemaphoreHandle_t mutex_;
};

// The per-port "has this changed?" bookkeeping this snapshot used to carry is
// gone: the stack's GroupObjectTransmitPolicy already decides Send / Defer /
// Suppress per object from the COV delta, the minimum repetition time and the
// heartbeat (see applyTransmitPolicies), which is the same model the KNX sensor
// FBs specify. So the loop publishes every port every tick and lets the policy
// layer drop what has not moved — one place deciding, instead of two that could
// disagree.
struct PublishSnapshot {
    sensor_data_t sensorData{};
    uint32_t availableMask{0};
    HvacSettings settings{};
    ControlOutputs out{};
    hvac_ns::OperatingPreset hvacOperatingMode{hvac_ns::OperatingPreset::Comfort};
    hvac_ns::ControllerMode controllerMode{hvac_ns::ControllerMode::Auto};
    hvac_ns::VentilationMode ventilationMode{hvac_ns::VentilationMode::Auto};
    bool controllerOnOff{true};
    bool toggleProgrammingMode{false};
};

// A port the ETS project left unlinked publishes as a successful no-op, so
// nothing here fires for unused datapoints; see reportUnlinkedPorts for the
// one-shot inventory of those. Anything that does reach this log is a genuine
// send failure, printed by name because the bare enum value was unreadable.
template <typename AppT>
void publishPort(AppT &app, SensorBoardPort port, float value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %s", label, util::errorCodeToString(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, SensorBoardPort port, bool value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %s", label, util::errorCodeToString(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, SensorBoardPort port, uint8_t value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %s", label, util::errorCodeToString(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, SensorBoardPort port, uint16_t value, const char *label)
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
void applyTransmitPolicies(AppT &app, const HvacSettings &settings)
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
        SensorBoardPort port;
        float threshold;
    };
    const CovPort covPorts[] = {
        {SensorBoardPort::RoomTemperature, settings.roomTemperatureCovK},
        {SensorBoardPort::RoomHumidity, settings.roomHumidityCovPct},
        {SensorBoardPort::RoomCo2, settings.co2CovPpm},
        {SensorBoardPort::RoomAirPressure, settings.pressureCovPa},
        {SensorBoardPort::RoomAirPressureSeaLevel, settings.pressureCovPa},
        {SensorBoardPort::RoomAirQualityIndex, settings.airQualityCovIndex},
        {SensorBoardPort::RoomCo2Equivalent, settings.co2CovPpm},
        {SensorBoardPort::RoomVocEquivalent, settings.co2CovPpm},
        {SensorBoardPort::RoomDewPoint, settings.derivedCov},
        {SensorBoardPort::RoomAbsoluteHumidity, settings.derivedCov},
        {SensorBoardPort::FloorTemperature, settings.floorTemperatureCovK},
        {SensorBoardPort::FloorHumidity, settings.floorHumidityCovPct},
        {SensorBoardPort::FloorAbsoluteHumidity, settings.derivedCov},
        {SensorBoardPort::DewPointMargin, settings.derivedCov},
        {SensorBoardPort::SetpointBase, settings.roomTemperatureCovK},
        {SensorBoardPort::SetpointStatus, settings.roomTemperatureCovK},
        {SensorBoardPort::SetpointHeatingStatus, settings.roomTemperatureCovK},
        {SensorBoardPort::SetpointCoolingStatus, settings.roomTemperatureCovK},
        {SensorBoardPort::SetpointShiftStatus, settings.roomTemperatureCovK},
        {SensorBoardPort::Co2Setpoint, settings.co2CovPpm},
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

    static constexpr SensorBoardPort kDiscretePorts[] = {
        SensorBoardPort::AirQualityAccuracy,
        SensorBoardPort::FloorMoistureAlarm,
        SensorBoardPort::FloorLimitActive,
        SensorBoardPort::FloorComfortActive,
        SensorBoardPort::DewPointAlarm,
        SensorBoardPort::FreeCoolingAvailable,
        SensorBoardPort::FreeDryingAvailable,
        SensorBoardPort::ControllerOnOff,
        SensorBoardPort::HvacModeStatus,
        SensorBoardPort::ContrModeStatus,
        SensorBoardPort::ContrModeSecondary,
        SensorBoardPort::HeatingControlValue,
        SensorBoardPort::CoolingControlValue,
        SensorBoardPort::HeatingRequest,
        SensorBoardPort::CoolingRequest,
        SensorBoardPort::HeatCoolModeStatus,
        SensorBoardPort::EnableHeatStatus,
        SensorBoardPort::EnableCoolStatus,
        SensorBoardPort::ControllerStatus,
        SensorBoardPort::VentilationDemand,
        SensorBoardPort::VentilationStage,
        SensorBoardPort::VentilationMode,
        SensorBoardPort::VentilationBoostRequest,
        SensorBoardPort::DehumidifyRequest,
        SensorBoardPort::AirQualityStatus,
        SensorBoardPort::DeviceFault,
        SensorBoardPort::RoomSensorStatus,
        SensorBoardPort::FloorProbeStatus,
        SensorBoardPort::AirQualitySensorStatus,
    };
    for (const auto port : kDiscretePorts) {
        (void)app.setTransmitPolicy(port, discrete);
    }
}

// Corrected sensor readings: the ETS offsets are applied once, here, so every
// consumer (control loops, derived values, published measurements) sees the
// same number. Correcting only at the publish path would leave the controller
// working from an uncorrected temperature, which is exactly the reading the
// self-heating offset exists to fix.
struct CorrectedReadings {
    float roomTemperatureC{0.0f};
    bool roomTemperatureValid{false};
    float roomHumidityPct{0.0f};
    bool roomHumidityValid{false};
    float floorTemperatureC{0.0f};
    bool floorTemperatureValid{false};
    float floorHumidityPct{0.0f};
    bool floorHumidityValid{false};
    float co2Ppm{0.0f};
    bool co2Valid{false};
    float pressurePa{0.0f};
    bool pressureValid{false};
    float iaqIndex{0.0f};
    bool iaqValid{false};

    bool roomAirValid() const { return roomTemperatureValid && roomHumidityValid; }
    bool floorProbeValid() const { return floorTemperatureValid && floorHumidityValid; }
};

CorrectedReadings correctReadings(const sensor_data_t &data,
                                  uint32_t availableMask,
                                  const HvacSettings &settings)
{
    CorrectedReadings r;
    r.roomTemperatureValid = (availableMask & SENSOR_TEMPERATURE) != 0u;
    r.roomTemperatureC = data.temperature + settings.roomTemperatureOffsetK;
    r.roomHumidityValid = (availableMask & SENSOR_HUMIDITY) != 0u;
    r.roomHumidityPct = hvac_ns::clampf(data.humidity + settings.roomHumidityOffsetPct, 0.0f, 100.0f);
    r.floorTemperatureValid = (availableMask & SENSOR_EXT_PROBE_TEMPERATURE) != 0u;
    r.floorTemperatureC = data.ext_probe_temperature + settings.floorTemperatureOffsetK;
    r.floorHumidityValid = (availableMask & SENSOR_EXT_PROBE_HUMIDITY) != 0u;
    r.floorHumidityPct = hvac_ns::clampf(data.ext_probe_humidity, 0.0f, 100.0f);
    r.co2Valid = (availableMask & SENSOR_CO2) != 0u;
    r.co2Ppm = static_cast<float>(data.co2);
    r.pressureValid = (availableMask & SENSOR_PRESSURE) != 0u;
    r.pressurePa = data.pressure;
    r.iaqValid = (availableMask & SENSOR_IAQ) != 0u;
    r.iaqIndex = data.iaq;
    return r;
}

// Room-control tick: snapshot inputs + ETS tuning under the mutex, run the
// (task-owned, lock-free) controllers, store the outputs back. Called from the
// KNX task at ~1 Hz. Nothing here decides what to transmit — that is the
// transmit policy's job (see applyTransmitPolicies).
template <typename AppT>
void runHvacControlTick(AppT &app,
                        hvac_ns::ThermostatController &thermostat,
                        hvac_ns::VentilationController &ventilation,
                        hvac_ns::DewPointMonitor &dewPoint,
                        hvac_ns::FloorMoistureMonitor &floorMoisture,
                        float dtSeconds)
{
    HvacSettings settings;
    sensor_data_t sensorData{};
    uint32_t availableMask = 0;
    hvac_ns::ThermostatInputs thermostatIn;
    hvac_ns::VentilationInputs ventIn;
    float outsideTemperatureC = 0.0f;
    bool outsideTemperatureValid = false;
    float outsideHumidityPct = 0.0f;
    bool outsideHumidityValid = false;
    {
        LockGuard lock(g_state.mutex);
        settings = g_state.hvac;
        sensorData = g_state.latestSensorData;
        availableMask = g_state.availableMask;
        thermostatIn.controllerEnable = g_state.controllerOnOff;
        thermostatIn.hvacOperatingMode = g_state.hvacOperatingMode;
        thermostatIn.controllerMode = g_state.controllerMode;
        thermostatIn.heatCoolChangeoverBusValue = g_state.changeOverStatus;
        thermostatIn.windowOpen = g_state.windowOpen;
        thermostatIn.presence = g_state.presence;
        thermostatIn.presenceValid = g_state.presenceKnown;
        thermostatIn.setpointShiftK = g_state.setpointShiftK;
        thermostatIn.switchHeat = g_state.switchHeat;
        thermostatIn.switchCool = g_state.switchCool;
        thermostatIn.flowTemperatureC = g_state.flowTemperatureC;
        thermostatIn.flowTemperatureValid = g_state.flowTemperatureValid;
        thermostatIn.dewPointAlarm = g_state.externalDewPointAlarm;
        ventIn.mode = g_state.ventilationMode;
        ventIn.occupied = g_state.presenceKnown ? g_state.presence : false;
        outsideTemperatureC = g_state.outsideTemperatureC;
        outsideTemperatureValid = g_state.outsideTemperatureValid;
        outsideHumidityPct = g_state.outsideHumidityPct;
        outsideHumidityValid = g_state.outsideHumidityValid;
    }

    applyTransmitPolicies(app, settings);

    const CorrectedReadings readings = correctReadings(sensorData, availableMask, settings);

    thermostatIn.roomTemperatureC = readings.roomTemperatureC;
    thermostatIn.roomTemperatureValid = readings.roomTemperatureValid;
    thermostatIn.floorTemperatureC = readings.floorTemperatureC;
    thermostatIn.floorTemperatureValid = readings.floorTemperatureValid;
    ventIn.co2Ppm = readings.co2Ppm;
    ventIn.co2Valid = readings.co2Valid;
    ventIn.humidityPct = readings.roomHumidityPct;
    ventIn.humidityValid = readings.roomHumidityValid;
    ventIn.vocIndex = readings.iaqIndex;
    ventIn.vocValid = readings.iaqValid;

    // --- Derived values and condensation / moisture monitors -----------------
    dewPoint.configure({.surfaceSource = settings.dewPointSurfaceSource,
                        .marginK = settings.dewPointMarginK,
                        .hysteresisK = settings.dewPointHysteresisK});
    const auto dewOut = dewPoint.update({
        .roomTemperatureC = readings.roomTemperatureC,
        .roomHumidityPct = readings.roomHumidityPct,
        .roomAirValid = readings.roomAirValid(),
        .floorTemperatureC = readings.floorTemperatureC,
        .floorTemperatureValid = readings.floorTemperatureValid,
        .flowTemperatureC = thermostatIn.flowTemperatureC,
        .flowTemperatureValid = thermostatIn.flowTemperatureValid,
    });

    floorMoisture.configure({.thresholdPct = settings.floorMoistureThresholdPct,
                             .hysteresisPct = settings.floorMoistureHysteresisPct,
                             .absoluteExcessGm3 = settings.floorMoistureExcessGm3});
    const auto moistureOut = floorMoisture.update({
        .floorTemperatureC = readings.floorTemperatureC,
        .floorHumidityPct = readings.floorHumidityPct,
        .floorProbeValid = readings.floorProbeValid(),
        .roomTemperatureC = readings.roomTemperatureC,
        .roomHumidityPct = readings.roomHumidityPct,
        .roomAirValid = readings.roomAirValid(),
    });

    // A locally derived condensation risk carries the same weight as one
    // received on the DewPointStatus input; either blocks cooling.
    thermostatIn.dewPointAlarm = thermostatIn.dewPointAlarm || dewOut.alarm;

    // --- Thermostat ----------------------------------------------------------
    // Reconfigure from the ETS parameters every tick: configure() preserves
    // controller state (integrators, latches), so this is cheap and makes a
    // live ETS re-parameterisation take effect within one tick.
    hvac_ns::ThermostatConfig thermostatCfg;
    thermostatCfg.heatingPid = {.kp = settings.heatingKp,
                                .tiSeconds = settings.heatingTiSeconds,
                                .tdSeconds = settings.heatingTdSeconds};
    thermostatCfg.coolingPid = {.kp = settings.coolingKp,
                                .tiSeconds = settings.coolingTiSeconds,
                                .tdSeconds = settings.coolingTdSeconds,
                                .reverseActing = true};
    thermostatCfg.setpoints = {
        .comfortHeatingC = settings.comfortHeatingSetpointC,
        .standbyHeatingReductionK = settings.standbyHeatingReductionK,
        .economyHeatingReductionK = settings.economyHeatingReductionK,
        .protectionHeatingC = settings.protectionHeatingSetpointC,
        .coolingDeadbandK = settings.coolingDeadbandK,
        .standbyCoolingIncreaseK = settings.standbyCoolingIncreaseK,
        .economyCoolingIncreaseK = settings.economyCoolingIncreaseK,
        .protectionCoolingC = settings.protectionCoolingSetpointC,
    };
    thermostatCfg.minSetpointC = settings.minSetpointC;
    thermostatCfg.maxSetpointC = settings.maxSetpointC;
    thermostatCfg.maxSetpointShiftK = settings.maxSetpointShiftK;
    thermostatCfg.requestHysteresisK = settings.thermostatHysteresisC;
    thermostatCfg.maxFloorTemperatureC = settings.maxFloorTemperatureC;
    thermostatCfg.minFloorTemperatureC = settings.minFloorTemperatureC;
    thermostatCfg.floorHysteresisK = settings.floorHysteresisK;
    thermostatCfg.floorComfortOutputPercent = settings.floorComfortOutputPercent;
    thermostatCfg.frostAlarmTemperatureC = settings.frostAlarmTemperatureC;
    thermostatCfg.overheatAlarmTemperatureC = settings.overheatAlarmTemperatureC;
    thermostatCfg.blockCoolingOnDewPointAlarm = settings.blockCoolingOnDewPointAlarm;
    thermostatCfg.heatingEnabled = settings.heatingEnabled;
    thermostatCfg.coolingEnabled = settings.coolingEnabled;
    thermostatCfg.heatCoolChangeoverMode = settings.heatCoolChangeoverMode;
    thermostatCfg.heatCoolChangeoverPolarityInverted = settings.heatCoolChangeoverPolarityInverted;
    thermostatCfg.heatingAlgorithm = settings.heatingControlAlgorithm;
    thermostatCfg.coolingAlgorithm = settings.coolingControlAlgorithm;
    thermostatCfg.heatingMinOutputPercent = settings.heatingMinOutputPercent;
    thermostatCfg.heatingMaxOutputPercent = settings.heatingMaxOutputPercent;
    thermostatCfg.coolingMinOutputPercent = settings.coolingMinOutputPercent;
    thermostatCfg.coolingMaxOutputPercent = settings.coolingMaxOutputPercent;
    thermostatCfg.binaryDemandStrategy = settings.binaryDemandStrategy;
    thermostatCfg.binaryDemandThresholdPercent = settings.binaryDemandThresholdPercent;
    thermostatCfg.minimumHeatCoolChangeoverSeconds = settings.minimumHeatCoolChangeoverSeconds;
    thermostatCfg.windowOpenBehavior = settings.windowOpenBehavior;
    thermostatCfg.presenceBehavior = settings.presenceBehavior;
    thermostatCfg.sensorFaultBehavior = settings.sensorFaultBehavior;
    thermostat.configure(thermostatCfg);

    ventilation.configure({.co2SetpointPpm = settings.ventilationSetpointPpm,
                           .co2BandPpm = settings.ventilationBandPpm,
                           .humidityThresholdPct = settings.humidityBoostPct,
                           .humidityBandPct = settings.humidityBandPct,
                           .vocThresholdIndex = settings.vocThresholdIndex,
                           .vocBandIndex = settings.vocBandIndex,
                           .baseDemandPercent = settings.ventilationBaseDemandPercent,
                           .manualDemandPercent = settings.ventilationManualDemandPercent});

    const auto out = thermostat.update(thermostatIn, dtSeconds);
    const auto ventOut = ventilation.update(ventIn);

    // --- Outside-air opportunity flags ---------------------------------------
    // Both compare like with like: temperature for cooling, absolute humidity
    // for drying. Comparing relative humidities would say cold outdoor air is
    // "wetter" than warm indoor air, which is backwards.
    ControlOutputs computed;
    computed.freeCoolingAvailable =
        outsideTemperatureValid && readings.roomTemperatureValid
        && outsideTemperatureC < readings.roomTemperatureC - 1.0f
        && readings.roomTemperatureC > out.coolingSetpointC;
    if (outsideTemperatureValid && outsideHumidityValid && readings.roomAirValid()) {
        const float outsideAbs =
            hvac_ns::psychro::absoluteHumidityGm3(outsideTemperatureC, outsideHumidityPct);
        computed.freeDryingAvailable =
            outsideAbs < moistureOut.roomAbsoluteHumidityGm3 - 0.5f;
    }

    computed.roomDewPointC = dewOut.dewPointC;
    computed.roomAbsoluteHumidityGm3 = moistureOut.roomAbsoluteHumidityGm3;
    computed.floorAbsoluteHumidityGm3 = moistureOut.floorAbsoluteHumidityGm3;
    computed.dewPointMarginK = dewOut.marginK;
    computed.dewPointAlarm = thermostatIn.dewPointAlarm;
    computed.floorMoistureAlarm = moistureOut.alarm;
    computed.seaLevelPressurePa =
        readings.pressureValid
            ? hvac_ns::psychro::seaLevelPressurePa(readings.pressurePa, settings.altitudeM,
                                                   readings.roomTemperatureC)
            : 0.0f;

    computed.heatingRequest = out.heatingRequest;
    computed.coolingRequest = out.coolingRequest;
    computed.heatingControlPercent = out.heatingControlPercent;
    computed.coolingControlPercent = out.coolingControlPercent;
    computed.floorLimitActive = out.floorLimitActive;
    computed.floorComfortActive = out.floorComfortActive;
    // DPT 1.100 has no "neutral": the spec's own StatusRHCC wording treats
    // heating as the default, so only active cooling reports 0.
    computed.heatCoolModeHeating = out.heatCoolState != hvac_ns::HeatCoolState::Cooling;
    computed.enableHeat = !out.heatingBlocked;
    computed.enableCool = !out.coolingBlocked;
    computed.activeSetpointC = out.activeSetpointC;
    computed.heatingSetpointC = out.heatingSetpointC;
    computed.coolingSetpointC = out.coolingSetpointC;
    computed.setpointShiftFeedbackK = out.setpointShiftFeedbackK;
    computed.activePreset = out.activePreset;
    // ContrModeAct / ContrModeSecondary report what the controller resolved to,
    // not what was requested: a Secondary RTC in the same room needs the
    // decision, and Auto would tell it nothing.
    computed.activeControllerMode =
        out.heatCoolState == hvac_ns::HeatCoolState::Heating  ? hvac_ns::ControllerMode::Heat
        : out.heatCoolState == hvac_ns::HeatCoolState::Cooling ? hvac_ns::ControllerMode::Cool
        : (out.heatingBlocked && out.coolingBlocked)           ? hvac_ns::ControllerMode::Off
                                                               : hvac_ns::ControllerMode::Auto;
    computed.controllerStatus = hvac_ns::packStatusRHCC(out);

    computed.ventilationDemandPercent = ventOut.demandPercent;
    computed.ventilationLevel = ventOut.level;
    computed.ventilationBoostRequest = ventOut.boostRequest;
    computed.dehumidifyRequest = ventOut.dehumidifyRequest;

    uint16_t iaqBits = 0;
    if (ventOut.humidityHigh) iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::HumidityBoost);
    if (ventOut.co2High) iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::Co2Boost);
    if (ventOut.vocHigh) iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::VocBoost);
    if (ventOut.sensorFault) iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::SensorFault);
    if (ventOut.dehumidifyRequest) {
        iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::DehumidifyRequest);
    }
    computed.airQualityStatus = iaqBits;

    // Per-package StatusGen octets (DPT 21.001). "Never delivered a reading" is
    // reported as OutOfService rather than Fault: an installation without the
    // optional floor probe is correctly configured, not broken.
    const bool anySensorData = g_state.hasSensorData;
    computed.roomSensorStatus =
        hvac_ns::packStatusGen(anySensorData, readings.roomAirValid());
    computed.floorProbeStatus = hvac_ns::packStatusGen(
        readings.floorTemperatureValid || readings.floorHumidityValid,
        readings.floorProbeValid(), moistureOut.alarm);
    computed.airQualitySensorStatus =
        hvac_ns::packStatusGen(anySensorData, readings.co2Valid || readings.iaqValid);
    // The roll-up alarm covers only what stops the device doing its job: the
    // room sensor the control loops depend on. An absent floor probe is
    // reported through its own StatusGen, not as a device fault.
    computed.deviceFault = !readings.roomAirValid();

    {
        LockGuard lock(g_state.mutex);
        g_state.out = computed;
    }
}

esp_err_t initNvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
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
    snapshot.availableMask = g_state.availableMask;
    snapshot.settings = g_state.hvac;
    snapshot.out = g_state.out;
    snapshot.hvacOperatingMode = g_state.hvacOperatingMode;
    snapshot.controllerMode = g_state.controllerMode;
    snapshot.ventilationMode = g_state.ventilationMode;
    snapshot.controllerOnOff = g_state.controllerOnOff;
    snapshot.toggleProgrammingMode = g_state.toggleProgrammingModeRequested;
    g_state.toggleProgrammingModeRequested = false;
    return snapshot;
}

// Publish every transmitting port. The transmit policy decides what actually
// reaches the bus (see applyTransmitPolicies), so an unchanged value costs a
// compare and nothing else; a port the ETS project left unlinked is a no-op.
template <typename AppT>
void publishAllState(AppT &app, const PublishSnapshot &snapshot)
{
    const auto &out = snapshot.out;
    const CorrectedReadings readings =
        correctReadings(snapshot.sensorData, snapshot.availableMask, snapshot.settings);

    // --- Room air measurements (FB RTS / RRHS / RAQS) ---
    if (readings.roomTemperatureValid) {
        publishPort(app, SensorBoardPort::RoomTemperature, readings.roomTemperatureC,
                    "room temperature");
    }
    if (readings.roomHumidityValid) {
        publishPort(app, SensorBoardPort::RoomHumidity, readings.roomHumidityPct, "room humidity");
    }
    if (readings.co2Valid) {
        publishPort(app, SensorBoardPort::RoomCo2, readings.co2Ppm, "room co2");
    }
    if (readings.pressureValid) {
        publishPort(app, SensorBoardPort::RoomAirPressure, readings.pressurePa, "air pressure");
        publishPort(app, SensorBoardPort::RoomAirPressureSeaLevel, out.seaLevelPressurePa,
                    "sea-level pressure");
    }
    if (readings.iaqValid) {
        publishPort(app, SensorBoardPort::RoomAirQualityIndex,
                    static_cast<uint16_t>(readings.iaqIndex + 0.5f), "air quality index");
        publishPort(app, SensorBoardPort::AirQualityAccuracy,
                    snapshot.sensorData.air_quality_accuracy, "air quality accuracy");
    }
    if ((snapshot.availableMask & SENSOR_CO2_EQUIVALENT) != 0u) {
        publishPort(app, SensorBoardPort::RoomCo2Equivalent, snapshot.sensorData.co2_equivalent,
                    "co2 equivalent");
    }
    if ((snapshot.availableMask & SENSOR_VOC_EQUIVALENT) != 0u) {
        publishPort(app, SensorBoardPort::RoomVocEquivalent, snapshot.sensorData.voc_equivalent,
                    "voc equivalent");
    }

    // --- Derived room air values ---
    if (readings.roomAirValid()) {
        publishPort(app, SensorBoardPort::RoomDewPoint, out.roomDewPointC, "room dew point");
        publishPort(app, SensorBoardPort::RoomAbsoluteHumidity, out.roomAbsoluteHumidityGm3,
                    "room absolute humidity");
    }

    // --- Floor probe (FB FTS + slab moisture) ---
    if (readings.floorTemperatureValid) {
        publishPort(app, SensorBoardPort::FloorTemperature, readings.floorTemperatureC,
                    "floor temperature");
    }
    if (readings.floorHumidityValid) {
        publishPort(app, SensorBoardPort::FloorHumidity, readings.floorHumidityPct,
                    "floor humidity");
    }
    if (readings.floorProbeValid()) {
        publishPort(app, SensorBoardPort::FloorAbsoluteHumidity, out.floorAbsoluteHumidityGm3,
                    "floor absolute humidity");
    }
    publishPort(app, SensorBoardPort::FloorMoistureAlarm, out.floorMoistureAlarm,
                "floor moisture alarm");
    publishPort(app, SensorBoardPort::FloorLimitActive, out.floorLimitActive, "floor limit active");
    publishPort(app, SensorBoardPort::FloorComfortActive, out.floorComfortActive,
                "floor comfort active");

    // --- Condensation protection (FB DPS) ---
    publishPort(app, SensorBoardPort::DewPointAlarm, out.dewPointAlarm, "dew point alarm");
    publishPort(app, SensorBoardPort::DewPointMargin, out.dewPointMarginK, "dew point margin");
    publishPort(app, SensorBoardPort::FreeCoolingAvailable, out.freeCoolingAvailable,
                "free cooling available");
    publishPort(app, SensorBoardPort::FreeDryingAvailable, out.freeDryingAvailable,
                "free drying available");

    // --- Mode and setpoints (FB RTSM) ---
    publishPort(app, SensorBoardPort::ControllerOnOff, snapshot.controllerOnOff,
                "controller on/off");
    {
        // HvacModeStatus reports the mode the controller resolved to, which
        // window/presence handling can move away from the requested one.
        const auto result = app.publish(SensorBoardPort::HvacModeStatus,
                                        static_cast<Dpt20Mode>(out.activePreset));
        if (result.isError()) {
            KNX_LOGW(TAG, "hvac mode status publish failed: %s",
                     util::errorCodeToString(result.error()));
        }
    }
    publishPort(app, SensorBoardPort::ContrModeStatus,
                hvac_ns::controllerModeToContrMode(out.activeControllerMode), "contr mode status");
    publishPort(app, SensorBoardPort::ContrModeSecondary,
                hvac_ns::controllerModeToContrMode(out.activeControllerMode),
                "contr mode secondary");
    publishPort(app, SensorBoardPort::SetpointBase, snapshot.settings.comfortHeatingSetpointC,
                "base setpoint");
    publishPort(app, SensorBoardPort::SetpointShiftStatus, out.setpointShiftFeedbackK,
                "setpoint shift status");
    publishPort(app, SensorBoardPort::SetpointStatus, out.activeSetpointC, "active setpoint");
    publishPort(app, SensorBoardPort::SetpointHeatingStatus, out.heatingSetpointC,
                "heating setpoint");
    publishPort(app, SensorBoardPort::SetpointCoolingStatus, out.coolingSetpointC,
                "cooling setpoint");

    // --- Controller outputs (FB RTC) ---
    publishPort(app, SensorBoardPort::HeatingControlValue, out.heatingControlPercent,
                "heating control value");
    publishPort(app, SensorBoardPort::CoolingControlValue, out.coolingControlPercent,
                "cooling control value");
    publishPort(app, SensorBoardPort::HeatingRequest, out.heatingRequest, "heating request");
    publishPort(app, SensorBoardPort::CoolingRequest, out.coolingRequest, "cooling request");
    publishPort(app, SensorBoardPort::HeatCoolModeStatus, out.heatCoolModeHeating,
                "heat/cool mode status");
    publishPort(app, SensorBoardPort::EnableHeatStatus, out.enableHeat, "enable heat status");
    publishPort(app, SensorBoardPort::EnableCoolStatus, out.enableCool, "enable cool status");
    publishPort(app, SensorBoardPort::ControllerStatus, out.controllerStatus, "controller status");

    // --- Ventilation / air quality ---
    publishPort(app, SensorBoardPort::Co2Setpoint, snapshot.settings.ventilationSetpointPpm,
                "co2 setpoint");
    publishPort(app, SensorBoardPort::VentilationDemand, out.ventilationDemandPercent,
                "ventilation demand");
    publishPort(app, SensorBoardPort::VentilationStage, static_cast<uint8_t>(out.ventilationLevel),
                "ventilation stage");
    publishPort(app, SensorBoardPort::VentilationMode,
                static_cast<uint8_t>(snapshot.ventilationMode), "ventilation mode");
    publishPort(app, SensorBoardPort::VentilationBoostRequest, out.ventilationBoostRequest,
                "ventilation boost request");
    publishPort(app, SensorBoardPort::DehumidifyRequest, out.dehumidifyRequest,
                "dehumidify request");
    publishPort(app, SensorBoardPort::AirQualityStatus, out.airQualityStatus,
                "air quality status");

    // --- Device diagnostics ---
    publishPort(app, SensorBoardPort::DeviceFault, out.deviceFault, "device fault");
    publishPort(app, SensorBoardPort::RoomSensorStatus, out.roomSensorStatus, "room sensor status");
    publishPort(app, SensorBoardPort::FloorProbeStatus, out.floorProbeStatus, "floor probe status");
    publishPort(app, SensorBoardPort::AirQualitySensorStatus, out.airQualitySensorStatus,
                "air quality sensor status");
}

void knxServiceTask(void *arg)
{
    (void)arg;

    // DEBUG shows per-frame bus traffic (RX/TX dumps, DL-ACK diagnostics,
    // connection events) — the development default while the stack is being
    // brought up. Switch to Info for production builds: INFO carries only
    // state changes (address, lifecycle, load state) and WARN/ERROR faults.
    knx::log::setLevel(knx::log::Level::Debug);
    // knx::log routes through esp_log, whose runtime default level (Info)
    // would otherwise silently drop the Debug output enabled above.
    esp_log_level_set("*", ESP_LOG_DEBUG);
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
    CommissionedProductHandle<std::remove_cvref_t<decltype(kSensorBoardProduct)>,
                              kDefaultBindingCapacity> appPtr;
    {
    auto bindings = makeCommissionedBindings(kSensorBoardProduct)
        // ---- Measurements: read requests are answered from the same
        // corrected readings the control loops use, so a bus read and a
        // spontaneous send can never disagree.
        .provideState<SensorBoardPort::RoomTemperature>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.temperature + g_state.hvac.roomTemperatureOffsetK;
        })
        .provideState<SensorBoardPort::RoomHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::clampf(
                g_state.latestSensorData.humidity + g_state.hvac.roomHumidityOffsetPct, 0.0f,
                100.0f);
        })
        .provideState<SensorBoardPort::RoomCo2>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<float>(g_state.latestSensorData.co2);
        })
        .provideState<SensorBoardPort::RoomAirPressure>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.pressure;
        })
        .provideState<SensorBoardPort::RoomAirPressureSeaLevel>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.seaLevelPressurePa;
        })
        .provideState<SensorBoardPort::RoomAirQualityIndex>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint16_t>(g_state.latestSensorData.iaq + 0.5f);
        })
        .provideState<SensorBoardPort::RoomCo2Equivalent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.co2_equivalent;
        })
        .provideState<SensorBoardPort::RoomVocEquivalent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.voc_equivalent;
        })
        .provideState<SensorBoardPort::AirQualityAccuracy>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.air_quality_accuracy;
        })
        .provideState<SensorBoardPort::RoomDewPoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomDewPointC;
        })
        .provideState<SensorBoardPort::RoomAbsoluteHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomAbsoluteHumidityGm3;
        })
        .provideState<SensorBoardPort::FloorTemperature>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.ext_probe_temperature
                   + g_state.hvac.floorTemperatureOffsetK;
        })
        .provideState<SensorBoardPort::FloorHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.ext_probe_humidity;
        })
        .provideState<SensorBoardPort::FloorAbsoluteHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorAbsoluteHumidityGm3;
        })
        .provideState<SensorBoardPort::FloorMoistureAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorMoistureAlarm;
        })
        .provideState<SensorBoardPort::FloorLimitActive>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorLimitActive;
        })
        .provideState<SensorBoardPort::FloorComfortActive>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorComfortActive;
        })

        // ---- Condensation protection (FB DPS) ----
        .provideState<SensorBoardPort::DewPointAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dewPointAlarm;
        })
        .provideState<SensorBoardPort::DewPointMargin>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dewPointMarginK;
        })
        .onStateWrite<SensorBoardPort::DewPointStatusInput>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.externalDewPointAlarm = value;
        })

        // ---- Neighbour-device inputs ----
        .onStateWrite<SensorBoardPort::OutsideTemperature>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.outsideTemperatureC = value;
            g_state.outsideTemperatureValid = true;
        })
        .onStateWrite<SensorBoardPort::OutsideHumidity>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.outsideHumidityPct = hvac_ns::clampf(value, 0.0f, 100.0f);
            g_state.outsideHumidityValid = true;
        })
        .onStateWrite<SensorBoardPort::FlowTemperature>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.flowTemperatureC = value;
            g_state.flowTemperatureValid = true;
        })
        .provideState<SensorBoardPort::FreeCoolingAvailable>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.freeCoolingAvailable;
        })
        .provideState<SensorBoardPort::FreeDryingAvailable>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.freeDryingAvailable;
        })

        // ---- Mode and setpoints (FB RTSM) ----
        .onStateWrite<SensorBoardPort::ControllerOnOff>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.controllerOnOff = value;
        })
        .provideState<SensorBoardPort::ControllerOnOff>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.controllerOnOff;
        })
        .onStateWrite<SensorBoardPort::HvacMode>([](Dpt20Mode value) {
            LockGuard lock(g_state.mutex);
            g_state.hvacOperatingMode = static_cast<hvac_ns::OperatingPreset>(value);
        })
        .provideState<SensorBoardPort::HvacModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<Dpt20Mode>(g_state.out.activePreset);
        })
        // The bus objects carry DPT 20.105 (HVACContrMode) code points; the
        // internal enum is compact. Map at the binding boundary only.
        .onStateWrite<SensorBoardPort::ContrMode>([](uint8_t value) {
            LockGuard lock(g_state.mutex);
            g_state.controllerMode = hvac_ns::controllerModeFromContrMode(value);
        })
        .provideState<SensorBoardPort::ContrModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::controllerModeToContrMode(g_state.out.activeControllerMode);
        })
        .provideState<SensorBoardPort::ContrModeSecondary>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::controllerModeToContrMode(g_state.out.activeControllerMode);
        })
        // Base setpoint write from an HMI or Home Assistant's
        // target_temperature: this is the comfort heating anchor of the KNX
        // setpoint ladder, so writing it moves standby/economy/cooling with it.
        .onStateWrite<SensorBoardPort::SetpointBase>([](float value) {
            LockGuard lock(g_state.mutex);
            const float clamped =
                hvac_ns::clampf(value, g_state.hvac.minSetpointC, g_state.hvac.maxSetpointC);
            if (std::fabs(g_state.hvac.comfortHeatingSetpointC - clamped) < 0.01f) {
                return;
            }
            g_state.hvac.comfortHeatingSetpointC = clamped;
            ESP_LOGI(TAG, "Base (comfort heating) setpoint updated to %.2f C", clamped);
        })
        .provideState<SensorBoardPort::SetpointBase>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.comfortHeatingSetpointC;
        })
        .onStateWrite<SensorBoardPort::SetpointShift>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.setpointShiftK = value;
        })
        .provideState<SensorBoardPort::SetpointShiftStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.setpointShiftFeedbackK;
        })
        .provideState<SensorBoardPort::SetpointStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.activeSetpointC;
        })
        .provideState<SensorBoardPort::SetpointHeatingStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingSetpointC;
        })
        .provideState<SensorBoardPort::SetpointCoolingStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingSetpointC;
        })

        // ---- Room inputs (FB WOS / PRD / WCOS) ----
        .onStateWrite<SensorBoardPort::WindowStatus>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.windowOpen = value;
        })
        .onStateWrite<SensorBoardPort::PresenceStatus>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.presence = value;
            g_state.presenceKnown = true;
        })
        .onStateWrite<SensorBoardPort::SwitchHeat>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.switchHeat = value;
        })
        .onStateWrite<SensorBoardPort::SwitchCool>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.switchCool = value;
        })
        .onStateWrite<SensorBoardPort::ChangeOverStatus>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.changeOverStatus = value;
        })

        // ---- Controller outputs (FB RTC) ----
        .provideState<SensorBoardPort::HeatingControlValue>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingControlPercent;
        })
        .provideState<SensorBoardPort::CoolingControlValue>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingControlPercent;
        })
        .provideState<SensorBoardPort::HeatingRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingRequest;
        })
        .provideState<SensorBoardPort::CoolingRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingRequest;
        })
        .provideState<SensorBoardPort::HeatCoolModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatCoolModeHeating;
        })
        .provideState<SensorBoardPort::EnableHeatStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.enableHeat;
        })
        .provideState<SensorBoardPort::EnableCoolStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.enableCool;
        })
        .provideState<SensorBoardPort::ControllerStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.controllerStatus;
        })

        // ---- Ventilation / air quality ----
        .onStateWrite<SensorBoardPort::Co2Setpoint>([](float value) {
            LockGuard lock(g_state.mutex);
            if (std::fabs(g_state.hvac.ventilationSetpointPpm - value) < 0.5f) {
                return;
            }
            g_state.hvac.ventilationSetpointPpm = value;
            ESP_LOGI(TAG, "CO2 setpoint updated to %.0f ppm", value);
        })
        .provideState<SensorBoardPort::Co2Setpoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.ventilationSetpointPpm;
        })
        .provideState<SensorBoardPort::VentilationDemand>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.ventilationDemandPercent;
        })
        .provideState<SensorBoardPort::VentilationStage>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.out.ventilationLevel);
        })
        .onStateWrite<SensorBoardPort::VentilationMode>([](uint8_t value) {
            LockGuard lock(g_state.mutex);
            g_state.ventilationMode = static_cast<hvac_ns::VentilationMode>(value);
        })
        .provideState<SensorBoardPort::VentilationMode>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.ventilationMode);
        })
        .provideState<SensorBoardPort::VentilationBoostRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.ventilationBoostRequest;
        })
        .provideState<SensorBoardPort::DehumidifyRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dehumidifyRequest;
        })
        .provideState<SensorBoardPort::AirQualityStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.airQualityStatus;
        })

        // ---- Device diagnostics ----
        .provideState<SensorBoardPort::DeviceFault>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.deviceFault;
        })
        .provideState<SensorBoardPort::RoomSensorStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomSensorStatus;
        })
        .provideState<SensorBoardPort::FloorProbeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorProbeStatus;
        })
        .provideState<SensorBoardPort::AirQualitySensorStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.airQualitySensorStatus;
        })

        // ---- ETS parameters ----
        // Fractional parameters arrive as Dpt9Float (KNX 2-byte half-float,
        // implicitly convertible to float); counts and seconds as uint16_t;
        // enumerations and percentages as uint8_t.
        .onParameterChanged<SensorBoardParameter::MeasurementHeartbeatSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heartbeatSeconds = v;
        })
        .onParameterChanged<SensorBoardParameter::MeasurementMinRepTimeSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.minRepTimeSeconds = v;
        })
        .onParameterChanged<SensorBoardParameter::RoomTemperatureOffset>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.roomTemperatureOffsetK = v;
        })
        .onParameterChanged<SensorBoardParameter::RoomTemperatureCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.roomTemperatureCovK = v;
        })
        .onParameterChanged<SensorBoardParameter::RoomHumidityOffset>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.roomHumidityOffsetPct = v;
        })
        .onParameterChanged<SensorBoardParameter::RoomHumidityCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.roomHumidityCovPct = v;
        })
        .onParameterChanged<SensorBoardParameter::Co2Cov>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.co2CovPpm = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::PressureCov>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.pressureCovPa = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::AirQualityCov>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.airQualityCovIndex = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::FloorTemperatureOffset>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorTemperatureOffsetK = v;
        })
        .onParameterChanged<SensorBoardParameter::FloorTemperatureCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorTemperatureCovK = v;
        })
        .onParameterChanged<SensorBoardParameter::FloorHumidityCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorHumidityCovPct = v;
        })
        .onParameterChanged<SensorBoardParameter::DerivedValueCov>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.derivedCov = v;
        })
        .onParameterChanged<SensorBoardParameter::AltitudeM>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.altitudeM = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::ControllerDefaultEnable>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.controllerDefaultEnable = (v != 0);
        })
        .onParameterChanged<SensorBoardParameter::DefaultHvacOperatingMode>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.defaultHvacOperatingMode = static_cast<hvac_ns::OperatingPreset>(v);
        })
        .onParameterChanged<SensorBoardParameter::DefaultControllerMode>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.defaultControllerMode = static_cast<hvac_ns::ControllerMode>(v);
        })
        .onParameterChanged<SensorBoardParameter::HeatingEnabled>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingEnabled = (v != 0);
        })
        .onParameterChanged<SensorBoardParameter::CoolingEnabled>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingEnabled = (v != 0);
        })
        .onParameterChanged<SensorBoardParameter::HeatCoolChangeoverMode>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatCoolChangeoverMode = static_cast<hvac_ns::HeatCoolChangeoverMode>(v);
        })
        .onParameterChanged<SensorBoardParameter::HeatCoolChangeoverPolarity>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatCoolChangeoverPolarityInverted = (v != 0);
        })
        .onParameterChanged<SensorBoardParameter::MinimumHeatCoolChangeoverSeconds>(
            [](uint16_t v) {
                LockGuard lock(g_state.mutex);
                g_state.hvac.minimumHeatCoolChangeoverSeconds = static_cast<float>(v);
            })
        .onParameterChanged<SensorBoardParameter::WindowOpenBehavior>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.windowOpenBehavior = static_cast<hvac_ns::WindowOpenBehavior>(v);
        })
        .onParameterChanged<SensorBoardParameter::PresenceBehavior>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.presenceBehavior = static_cast<hvac_ns::PresenceBehavior>(v);
        })
        .onParameterChanged<SensorBoardParameter::SensorFaultBehavior>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.sensorFaultBehavior = static_cast<hvac_ns::SensorFaultBehavior>(v);
        })
        .onParameterChanged<SensorBoardParameter::ComfortHeatingSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.comfortHeatingSetpointC = v;
        })
        .onParameterChanged<SensorBoardParameter::StandbyHeatingReduction>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.standbyHeatingReductionK = v;
        })
        .onParameterChanged<SensorBoardParameter::EconomyHeatingReduction>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.economyHeatingReductionK = v;
        })
        .onParameterChanged<SensorBoardParameter::ProtectionHeatingSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.protectionHeatingSetpointC = v;
        })
        .onParameterChanged<SensorBoardParameter::CoolingDeadband>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingDeadbandK = v;
        })
        .onParameterChanged<SensorBoardParameter::StandbyCoolingIncrease>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.standbyCoolingIncreaseK = v;
        })
        .onParameterChanged<SensorBoardParameter::EconomyCoolingIncrease>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.economyCoolingIncreaseK = v;
        })
        .onParameterChanged<SensorBoardParameter::ProtectionCoolingSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.protectionCoolingSetpointC = v;
        })
        .onParameterChanged<SensorBoardParameter::MinSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.minSetpointC = v;
        })
        .onParameterChanged<SensorBoardParameter::MaxSetpoint>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxSetpointC = v;
        })
        .onParameterChanged<SensorBoardParameter::MaxSetpointShift>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxSetpointShiftK = v;
        })
        .onParameterChanged<SensorBoardParameter::HeatingControlAlgorithm>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(v);
        })
        .onParameterChanged<SensorBoardParameter::HeatingKp>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingKp = v;
        })
        .onParameterChanged<SensorBoardParameter::HeatingTiSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingTiSeconds = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::HeatingTdSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingTdSeconds = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::HeatingMinimumOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingMinOutputPercent = v;
        })
        .onParameterChanged<SensorBoardParameter::HeatingMaximumOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingMaxOutputPercent = v;
        })
        .onParameterChanged<SensorBoardParameter::CoolingControlAlgorithm>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(v);
        })
        .onParameterChanged<SensorBoardParameter::CoolingKp>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingKp = v;
        })
        .onParameterChanged<SensorBoardParameter::CoolingTiSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingTiSeconds = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::CoolingTdSeconds>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingTdSeconds = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::CoolingMinimumOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingMinOutputPercent = v;
        })
        .onParameterChanged<SensorBoardParameter::CoolingMaximumOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingMaxOutputPercent = v;
        })
        .onParameterChanged<SensorBoardParameter::ThermostatHysteresis>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.thermostatHysteresisC = v;
        })
        .onParameterChanged<SensorBoardParameter::BinaryDemandStrategy>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.binaryDemandStrategy = static_cast<hvac_ns::BinaryDemandStrategy>(v);
        })
        .onParameterChanged<SensorBoardParameter::BinaryDemandThresholdPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.binaryDemandThresholdPercent = v;
        })
        .onParameterChanged<SensorBoardParameter::FrostAlarmTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.frostAlarmTemperatureC = v;
        })
        .onParameterChanged<SensorBoardParameter::OverheatAlarmTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.overheatAlarmTemperatureC = v;
        })
        .onParameterChanged<SensorBoardParameter::MaxFloorTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxFloorTemperatureC = v;
        })
        .onParameterChanged<SensorBoardParameter::MinFloorTemperature>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.minFloorTemperatureC = v;
        })
        .onParameterChanged<SensorBoardParameter::FloorHysteresis>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorHysteresisK = v;
        })
        .onParameterChanged<SensorBoardParameter::FloorComfortOutputPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorComfortOutputPercent = v;
        })
        .onParameterChanged<SensorBoardParameter::DewPointSurfaceSource>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.dewPointSurfaceSource = static_cast<hvac_ns::DewPointSurfaceSource>(v);
        })
        .onParameterChanged<SensorBoardParameter::DewPointMargin>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.dewPointMarginK = v;
        })
        .onParameterChanged<SensorBoardParameter::DewPointHysteresis>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.dewPointHysteresisK = v;
        })
        .onParameterChanged<SensorBoardParameter::BlockCoolingOnDewPointAlarm>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.blockCoolingOnDewPointAlarm = (v != 0);
        })
        .onParameterChanged<SensorBoardParameter::FloorMoistureThreshold>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorMoistureThresholdPct = v;
        })
        .onParameterChanged<SensorBoardParameter::FloorMoistureHysteresis>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorMoistureHysteresisPct = v;
        })
        .onParameterChanged<SensorBoardParameter::FloorMoistureExcess>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorMoistureExcessGm3 = v;
        })
        .onParameterChanged<SensorBoardParameter::VentilationSetpoint>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationSetpointPpm = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::VentilationBand>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationBandPpm = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::HumidityBoostThreshold>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.humidityBoostPct = v;
        })
        .onParameterChanged<SensorBoardParameter::HumidityBoostBand>([](Dpt9Float v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.humidityBandPct = v;
        })
        .onParameterChanged<SensorBoardParameter::VocBoostThreshold>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.vocThresholdIndex = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::VocBoostBand>([](uint16_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.vocBandIndex = static_cast<float>(v);
        })
        .onParameterChanged<SensorBoardParameter::VentilationBaseDemandPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationBaseDemandPercent = v;
        })
        .onParameterChanged<SensorBoardParameter::VentilationManualDemandPercent>([](uint8_t v) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationManualDemandPercent = v;
        })
        .onProgrammingModeChanged([](bool enabled) {
            KNX_LOGI(TAG, "Programming mode: %s", enabled ? "ON" : "OFF");

            knx_programming_mode_callback_t callback = nullptr;
            {
                LockGuard lock(g_state.mutex);
                callback = g_state.programmingModeCallback;
            }
            if (callback != nullptr) {
                callback(enabled);
            }
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
        kSensorBoardProduct,
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
                    char certificate[sensor_board::identity::kCertificateBufferSize] = {};
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
        g_state.taskHandle = xTaskGetCurrentTaskHandle();
        // Seed engineering values from the ETS parameters. Every parameter has
        // an onParameterChanged binding above for live re-parameterisation;
        // this is the one-time load at startup, so the two lists must stay in
        // step.
        auto params = app.parameters();
        auto &s = g_state.hvac;

        s.heartbeatSeconds = params.get<SensorBoardParameter::MeasurementHeartbeatSeconds>();
        s.minRepTimeSeconds = params.get<SensorBoardParameter::MeasurementMinRepTimeSeconds>();
        s.roomTemperatureOffsetK = params.get<SensorBoardParameter::RoomTemperatureOffset>();
        s.roomTemperatureCovK = params.get<SensorBoardParameter::RoomTemperatureCov>();
        s.roomHumidityOffsetPct = params.get<SensorBoardParameter::RoomHumidityOffset>();
        s.roomHumidityCovPct = params.get<SensorBoardParameter::RoomHumidityCov>();
        s.co2CovPpm = static_cast<float>(params.get<SensorBoardParameter::Co2Cov>());
        s.pressureCovPa = static_cast<float>(params.get<SensorBoardParameter::PressureCov>());
        s.airQualityCovIndex = static_cast<float>(params.get<SensorBoardParameter::AirQualityCov>());
        s.floorTemperatureOffsetK = params.get<SensorBoardParameter::FloorTemperatureOffset>();
        s.floorTemperatureCovK = params.get<SensorBoardParameter::FloorTemperatureCov>();
        s.floorHumidityCovPct = params.get<SensorBoardParameter::FloorHumidityCov>();
        s.derivedCov = params.get<SensorBoardParameter::DerivedValueCov>();
        s.altitudeM = static_cast<float>(params.get<SensorBoardParameter::AltitudeM>());

        s.controllerDefaultEnable = params.get<SensorBoardParameter::ControllerDefaultEnable>() != 0;
        s.defaultHvacOperatingMode = static_cast<hvac_ns::OperatingPreset>(
            params.get<SensorBoardParameter::DefaultHvacOperatingMode>());
        s.defaultControllerMode = static_cast<hvac_ns::ControllerMode>(
            params.get<SensorBoardParameter::DefaultControllerMode>());
        s.heatingEnabled = params.get<SensorBoardParameter::HeatingEnabled>() != 0;
        s.coolingEnabled = params.get<SensorBoardParameter::CoolingEnabled>() != 0;
        s.heatCoolChangeoverMode = static_cast<hvac_ns::HeatCoolChangeoverMode>(
            params.get<SensorBoardParameter::HeatCoolChangeoverMode>());
        s.heatCoolChangeoverPolarityInverted =
            params.get<SensorBoardParameter::HeatCoolChangeoverPolarity>() != 0;
        s.minimumHeatCoolChangeoverSeconds = static_cast<float>(
            params.get<SensorBoardParameter::MinimumHeatCoolChangeoverSeconds>());
        s.windowOpenBehavior = static_cast<hvac_ns::WindowOpenBehavior>(
            params.get<SensorBoardParameter::WindowOpenBehavior>());
        s.presenceBehavior = static_cast<hvac_ns::PresenceBehavior>(
            params.get<SensorBoardParameter::PresenceBehavior>());
        s.sensorFaultBehavior = static_cast<hvac_ns::SensorFaultBehavior>(
            params.get<SensorBoardParameter::SensorFaultBehavior>());

        s.comfortHeatingSetpointC = params.get<SensorBoardParameter::ComfortHeatingSetpoint>();
        s.standbyHeatingReductionK = params.get<SensorBoardParameter::StandbyHeatingReduction>();
        s.economyHeatingReductionK = params.get<SensorBoardParameter::EconomyHeatingReduction>();
        s.protectionHeatingSetpointC = params.get<SensorBoardParameter::ProtectionHeatingSetpoint>();
        s.coolingDeadbandK = params.get<SensorBoardParameter::CoolingDeadband>();
        s.standbyCoolingIncreaseK = params.get<SensorBoardParameter::StandbyCoolingIncrease>();
        s.economyCoolingIncreaseK = params.get<SensorBoardParameter::EconomyCoolingIncrease>();
        s.protectionCoolingSetpointC = params.get<SensorBoardParameter::ProtectionCoolingSetpoint>();
        s.minSetpointC = params.get<SensorBoardParameter::MinSetpoint>();
        s.maxSetpointC = params.get<SensorBoardParameter::MaxSetpoint>();
        s.maxSetpointShiftK = params.get<SensorBoardParameter::MaxSetpointShift>();

        s.heatingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(
            params.get<SensorBoardParameter::HeatingControlAlgorithm>());
        s.heatingKp = params.get<SensorBoardParameter::HeatingKp>();
        s.heatingTiSeconds = static_cast<float>(params.get<SensorBoardParameter::HeatingTiSeconds>());
        s.heatingTdSeconds = static_cast<float>(params.get<SensorBoardParameter::HeatingTdSeconds>());
        s.heatingMinOutputPercent = params.get<SensorBoardParameter::HeatingMinimumOutputPercent>();
        s.heatingMaxOutputPercent = params.get<SensorBoardParameter::HeatingMaximumOutputPercent>();
        s.coolingControlAlgorithm = static_cast<hvac_ns::ControlAlgorithm>(
            params.get<SensorBoardParameter::CoolingControlAlgorithm>());
        s.coolingKp = params.get<SensorBoardParameter::CoolingKp>();
        s.coolingTiSeconds = static_cast<float>(params.get<SensorBoardParameter::CoolingTiSeconds>());
        s.coolingTdSeconds = static_cast<float>(params.get<SensorBoardParameter::CoolingTdSeconds>());
        s.coolingMinOutputPercent = params.get<SensorBoardParameter::CoolingMinimumOutputPercent>();
        s.coolingMaxOutputPercent = params.get<SensorBoardParameter::CoolingMaximumOutputPercent>();
        s.thermostatHysteresisC = params.get<SensorBoardParameter::ThermostatHysteresis>();
        s.binaryDemandStrategy = static_cast<hvac_ns::BinaryDemandStrategy>(
            params.get<SensorBoardParameter::BinaryDemandStrategy>());
        s.binaryDemandThresholdPercent =
            params.get<SensorBoardParameter::BinaryDemandThresholdPercent>();
        s.frostAlarmTemperatureC = params.get<SensorBoardParameter::FrostAlarmTemperature>();
        s.overheatAlarmTemperatureC = params.get<SensorBoardParameter::OverheatAlarmTemperature>();

        s.maxFloorTemperatureC = params.get<SensorBoardParameter::MaxFloorTemperature>();
        s.minFloorTemperatureC = params.get<SensorBoardParameter::MinFloorTemperature>();
        s.floorHysteresisK = params.get<SensorBoardParameter::FloorHysteresis>();
        s.floorComfortOutputPercent = params.get<SensorBoardParameter::FloorComfortOutputPercent>();

        s.dewPointSurfaceSource = static_cast<hvac_ns::DewPointSurfaceSource>(
            params.get<SensorBoardParameter::DewPointSurfaceSource>());
        s.dewPointMarginK = params.get<SensorBoardParameter::DewPointMargin>();
        s.dewPointHysteresisK = params.get<SensorBoardParameter::DewPointHysteresis>();
        s.blockCoolingOnDewPointAlarm =
            params.get<SensorBoardParameter::BlockCoolingOnDewPointAlarm>() != 0;

        s.floorMoistureThresholdPct = params.get<SensorBoardParameter::FloorMoistureThreshold>();
        s.floorMoistureHysteresisPct = params.get<SensorBoardParameter::FloorMoistureHysteresis>();
        s.floorMoistureExcessGm3 = params.get<SensorBoardParameter::FloorMoistureExcess>();

        s.ventilationSetpointPpm =
            static_cast<float>(params.get<SensorBoardParameter::VentilationSetpoint>());
        s.ventilationBandPpm =
            static_cast<float>(params.get<SensorBoardParameter::VentilationBand>());
        s.humidityBoostPct = params.get<SensorBoardParameter::HumidityBoostThreshold>();
        s.humidityBandPct = params.get<SensorBoardParameter::HumidityBoostBand>();
        s.vocThresholdIndex =
            static_cast<float>(params.get<SensorBoardParameter::VocBoostThreshold>());
        s.vocBandIndex = static_cast<float>(params.get<SensorBoardParameter::VocBoostBand>());
        s.ventilationBaseDemandPercent =
            params.get<SensorBoardParameter::VentilationBaseDemandPercent>();
        s.ventilationManualDemandPercent =
            params.get<SensorBoardParameter::VentilationManualDemandPercent>();

        // Seed the runtime mode/state inputs from their ETS defaults.
        g_state.controllerOnOff = s.controllerDefaultEnable;
        g_state.hvacOperatingMode = s.defaultHvacOperatingMode;
        g_state.controllerMode = s.defaultControllerMode;
    }

    KNX_LOGI(TAG, "ETS-commissionable TP1 sensor bridge started");

    DeviceLifecycleState previousLifecycle = app.lifecycleState();

    // Room-control loops (task-owned; configured each tick from ETS parameters).
    sensor_board::hvac::ThermostatController thermostat;
    sensor_board::hvac::VentilationController ventilation;
    sensor_board::hvac::DewPointMonitor dewPoint;
    sensor_board::hvac::FloorMoistureMonitor floorMoisture;
    constexpr TickType_t kControlIntervalTicks = pdMS_TO_TICKS(1000);
    TickType_t lastControlTick = xTaskGetTickCount();
    // Publishing is offered once per control tick, not once per loop iteration:
    // the loop also spins at a few milliseconds while the stack has in-flight
    // work, and re-offering every object that often would burn CPU on
    // suppression decisions that cannot have changed.
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
        const TaskHandle_t handle = g_state.taskHandle;  // set once at startup
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

        const TickType_t nowTick = xTaskGetTickCount();
        if ((nowTick - lastControlTick) >= kControlIntervalTicks) {
            const float dtSeconds =
                static_cast<float>(nowTick - lastControlTick) * portTICK_PERIOD_MS / 1000.0f;
            lastControlTick = nowTick;
            runHvacControlTick(app, thermostat, ventilation, dewPoint, floorMoisture, dtSeconds);
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
            bool toggleRequested = false;
            {
                LockGuard lock(g_state.mutex);
                toggleRequested = g_state.toggleProgrammingModeRequested;
                g_state.toggleProgrammingModeRequested = false;
            }
            if (toggleRequested) {
                app.toggleProgrammingMode();
            }
        } else if (app.lifecycleState() == DeviceLifecycleState::Operational) {
            const PublishSnapshot snapshot = takePublishSnapshot();
            if (snapshot.toggleProgrammingMode) {
                app.toggleProgrammingMode();
            }
            // Offered unconditionally once per control tick; the per-object
            // transmit policy decides what is actually worth a telegram.
            if (controlTickDue) {
                publishAllState(app, snapshot);
                controlTickDue = false;
            }
        } else {
            const PublishSnapshot snapshot = takePublishSnapshot();
            if (snapshot.toggleProgrammingMode) {
                app.toggleProgrammingMode();
            }
        }

        previousLifecycle = app.lifecycleState();

        // Park until the stack signals new work or the next control tick is due —
        // no fixed busy-poll. If polled work is still in flight, wake at the short
        // active cadence to service it to completion; otherwise sleep for the
        // remainder of the control interval (a work notification, e.g. an inbound
        // frame, cuts the sleep short and latches so it is never missed).
        const TickType_t elapsedTicks = xTaskGetTickCount() - lastControlTick;
        const TickType_t untilControlTick = (elapsedTicks < kControlIntervalTicks)
            ? (kControlIntervalTicks - elapsedTicks)
            : 0;
        TickType_t blockTicks = app.ownerWorkHint().hasImmediateWork()
            ? kActiveWorkPollTicks
            : untilControlTick;
        if (blockTicks == 0) {
            blockTicks = 1;  // control tick already due — take it on the next tick
        }
        (void)ulTaskNotifyTake(pdTRUE, blockTicks);
    }
}

} // namespace

extern "C" esp_err_t knx_service_start(void)
{
    if (g_state.started) {
        return ESP_OK;
    }

    if (g_state.mutex == nullptr) {
        g_state.mutex = xSemaphoreCreateMutex();
        if (g_state.mutex == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = initNvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t created = xTaskCreate(knxServiceTask, "knx_service", kKnxServiceTaskStackSize, nullptr, 8, nullptr);
    if (created != pdPASS) {
        return ESP_FAIL;
    }

    g_state.started = true;
    return ESP_OK;
}

extern "C" void knx_service_update_sensor_data(const sensor_data_t *data)
{
    if (data == nullptr || g_state.mutex == nullptr) {
        return;
    }

    LockGuard lock(g_state.mutex);
    g_state.latestSensorData = *data;
    g_state.availableMask |= data->updated_mask;
    g_state.hasSensorData = true;
    // Control outputs (requests, PID values) react on the next 1 s control
    // tick in the KNX task — no recalculation in the sensor task's context.
}

extern "C" void knx_service_toggle_programming_mode(void)
{
    if (g_state.mutex == nullptr) {
        return;
    }

    TaskHandle_t handle = nullptr;
    {
        LockGuard lock(g_state.mutex);
        g_state.toggleProgrammingModeRequested = true;
        handle = g_state.taskHandle;
    }
    // The service loop now parks on a notification instead of polling, so wake it
    // to act on the toggle promptly rather than at the next control tick.
    if (handle != nullptr) {
        xTaskNotifyGive(handle);
    }
}

extern "C" void knx_service_reset_nvm(void)
{
    KNX_LOGW(TAG, "Resetting KNX NVM state and rebooting");
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
        KNX_LOGE(TAG, "NVS erase failed: %s", esp_err_to_name(err));
    }
    esp_restart();
}

extern "C" void knx_service_set_programming_mode_callback(knx_programming_mode_callback_t callback)
{
    if (g_state.mutex == nullptr) {
        g_state.programmingModeCallback = callback;
        return;
    }

    LockGuard lock(g_state.mutex);
    g_state.programmingModeCallback = callback;
}