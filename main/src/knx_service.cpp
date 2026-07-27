#include "knx_service.h"

#include "board.h"
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

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

using namespace knx;
using namespace knx::application;
using namespace knx::product;
using namespace sensor_board_knx;

namespace {

static constexpr const char *TAG = "knx_service";
// The commissioned-product start path holds several multi-KB objects on this
// stack at once (bindings builder ~3 KB, Result<runtime> ~5 KB, plus the BAU
// init call tree with NVS/logging). With 15 ports and 15 ETS parameters the
// old 16 KB stack overflowed at startup (hardware stack guard → Breakpoint
// panic). The runtime itself is heap-allocated after start (see below); the
// larger stack covers the transient startup peak with margin.
//static constexpr uint32_t kKnxServiceTaskStackSize = 24576;
static constexpr uint32_t kKnxServiceTaskStackSize = 64*1024;

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
// When enabled the device provisions a unique 16-byte AES-128 key on first
// boot (hardware RNG), persists it in NVS, applies it as the KNX Data Secure
// tool key and logs it every boot so an installer can capture it locally and
// enter it together with the serial number into ETS for secure commissioning.
// The self-generated key plays the role of the device's FDSK / initial tool
// key. NOTE: knx_service_reset_nvm() erases the whole default NVS partition, so
// a factory reset regenerates a fresh key (re-logged on the next boot); a real
// immutable FDSK would live in eFuse or a dedicated protected partition.
static constexpr bool kEnableKnxDataSecure = true;
static constexpr const char *kSecureNvsNamespace = "knx_secure";
static constexpr const char *kSecureNvsToolKeyBlob = "tool_key";

// Render a byte buffer as an uppercase hex string (with optional separators)
// into caller storage, for logging identity/credentials.
void toHex(std::span<const uint8_t> bytes, char sep, char *out, size_t outLen)
{
    size_t pos = 0;
    for (size_t i = 0; i < bytes.size() && pos + 3 < outLen; ++i) {
        if (i != 0 && sep != '\0' && pos + 1 < outLen) {
            out[pos++] = sep;
        }
        pos += static_cast<size_t>(std::snprintf(out + pos, outLen - pos, "%02X", bytes[i]));
    }
    if (outLen > 0) {
        out[(pos < outLen) ? pos : (outLen - 1)] = '\0';
    }
}

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

// ETS-configurable room-control tuning (see hvac_control.hpp). Snapshotted by
// the control tick each second, so parameter changes apply within one tick.
struct HvacSettings {
    float ventilationSetpointPpm{kDefaultVentilationSetpointPpm};
    float thermostatHysteresisC{kDefaultThermostatHysteresisC};
    float ventilationHysteresisPpm{kDefaultVentilationHysteresisPpm};
    float heatingKp{kDefaultHeatingKp};
    float heatingTiSeconds{kDefaultHeatingTiSeconds};
    float heatingTdSeconds{kDefaultHeatingTdSeconds};
    float coolingKp{kDefaultCoolingKp};
    float coolingTiSeconds{kDefaultCoolingTiSeconds};
    float coolingTdSeconds{kDefaultCoolingTdSeconds};
    float coolingDeadbandK{kDefaultCoolingDeadbandK};
    float maxFloorTemperatureC{kDefaultMaxFloorTemperatureC};
    float floorHysteresisK{kDefaultFloorHysteresisK};
    float humidityBoostPct{kDefaultHumidityBoostPct};
    float humidityBoostHysteresisPct{kDefaultHumidityBoostHysteresisPct};

    // --- Append-only greenfield thermostat-model additions ---
    bool controllerDefaultEnable{kDefaultControllerEnable != 0};
    sensor_board::hvac::OperatingPreset defaultHvacOperatingMode{
        static_cast<sensor_board::hvac::OperatingPreset>(kDefaultHvacOperatingMode)};
    sensor_board::hvac::ControllerMode defaultControllerMode{
        static_cast<sensor_board::hvac::ControllerMode>(kDefaultControllerMode)};
    float comfortHeatingSetpointC{kDefaultComfortHeatingSetpointC};
    float standbyHeatingSetpointC{kDefaultStandbyHeatingSetpointC};
    float economyHeatingSetpointC{kDefaultEconomyHeatingSetpointC};
    float protectionHeatingSetpointC{kDefaultProtectionHeatingSetpointC};
    float comfortCoolingSetpointC{kDefaultComfortCoolingSetpointC};
    float standbyCoolingSetpointC{kDefaultStandbyCoolingSetpointC};
    float economyCoolingSetpointC{kDefaultEconomyCoolingSetpointC};
    float protectionCoolingSetpointC{kDefaultProtectionCoolingSetpointC};
    float minSetpointC{kDefaultMinSetpointC};
    float maxSetpointC{kDefaultMaxSetpointC};
    float maxSetpointShiftK{kDefaultMaxSetpointShiftK};
    bool heatingEnabled{kDefaultHeatingEnabled != 0};
    bool coolingEnabled{kDefaultCoolingEnabled != 0};
    sensor_board::hvac::HeatCoolChangeoverMode heatCoolChangeoverMode{
        static_cast<sensor_board::hvac::HeatCoolChangeoverMode>(kDefaultHeatCoolChangeoverMode)};
    bool heatCoolChangeoverPolarityInverted{kDefaultHeatCoolChangeoverPolarity != 0};
    sensor_board::hvac::ControlAlgorithm heatingControlAlgorithm{
        static_cast<sensor_board::hvac::ControlAlgorithm>(kDefaultHeatingControlAlgorithm)};
    sensor_board::hvac::ControlAlgorithm coolingControlAlgorithm{
        static_cast<sensor_board::hvac::ControlAlgorithm>(kDefaultCoolingControlAlgorithm)};
    uint8_t heatingMinOutputPercent{static_cast<uint8_t>(kDefaultHeatingMinimumOutputPercent)};
    uint8_t coolingMinOutputPercent{static_cast<uint8_t>(kDefaultCoolingMinimumOutputPercent)};
    uint8_t heatingMaxOutputPercent{static_cast<uint8_t>(kDefaultHeatingMaximumOutputPercent)};
    uint8_t coolingMaxOutputPercent{static_cast<uint8_t>(kDefaultCoolingMaximumOutputPercent)};
    sensor_board::hvac::BinaryDemandStrategy binaryDemandStrategy{
        static_cast<sensor_board::hvac::BinaryDemandStrategy>(kDefaultBinaryDemandStrategy)};
    uint8_t binaryDemandThresholdPercent{static_cast<uint8_t>(kDefaultBinaryDemandThresholdPercent)};
    float minimumHeatCoolChangeoverSeconds{static_cast<float>(kDefaultMinimumHeatCoolChangeoverSeconds)};
    sensor_board::hvac::WindowOpenBehavior windowOpenBehavior{
        static_cast<sensor_board::hvac::WindowOpenBehavior>(kDefaultWindowOpenBehavior)};
    sensor_board::hvac::PresenceBehavior presenceBehavior{
        static_cast<sensor_board::hvac::PresenceBehavior>(kDefaultPresenceBehavior)};
    sensor_board::hvac::SensorFaultBehavior sensorFaultBehavior{
        static_cast<sensor_board::hvac::SensorFaultBehavior>(kDefaultSensorFaultBehavior)};
    float temperatureChangeThresholdC{kDefaultTemperatureChangeThresholdCenti / 100.0f};
    uint32_t cyclicStatusIntervalSeconds{kDefaultCyclicStatusIntervalSeconds};
};

struct SharedState {
    SemaphoreHandle_t mutex{nullptr};
    TaskHandle_t taskHandle{nullptr};
    sensor_data_t latestSensorData{};
    uint32_t availableMask{0};
    uint32_t pendingSensorMask{0};
    bool hasSensorData{false};
    bool started{false};
    bool initialPublishPending{true};
    bool toggleProgrammingModeRequested{false};
    bool pendingThermostatSetpointPublish{true};
    bool pendingVentilationSetpointPublish{true};
    bool pendingThermostatRequestPublish{true};
    bool pendingVentilationBoostPublish{true};
    bool pendingCoolingRequestPublish{true};
    bool pendingHeatingControlPublish{true};
    bool pendingCoolingControlPublish{true};
    bool pendingFloorLimitPublish{true};
    // --- Append-only greenfield thermostat-model additions ---
    bool pendingControllerEnablePublish{true};
    bool pendingHvacOperatingModePublish{true};
    bool pendingControllerModePublish{true};
    bool pendingHeatCoolStatePublish{true};
    bool pendingActiveSetpointPublish{true};
    bool pendingSetpointShiftFeedbackPublish{true};
    bool pendingControllerFaultPublish{true};
    bool pendingSensorFaultStatusPublish{true};
    bool pendingHeatingBlockedPublish{true};
    bool pendingCoolingBlockedPublish{true};
    bool pendingVentilationDemandPublish{true};
    bool pendingVentilationLevelPublish{true};
    bool pendingVentilationModePublish{true};
    bool pendingVentilationActivePublish{true};
    bool pendingIaqStatusPublish{true};
    bool pendingHvacControllerStatusPublish{true};
    HvacSettings hvac{};
    // Controller outputs (written by the control tick, read by provideState
    // and the publish path).
    bool thermostatRequest{false};        // heating demand
    bool coolingRequest{false};
    bool ventilationBoostRequest{false};
    bool floorLimitActive{false};
    uint8_t heatingControlPercent{0};
    uint8_t coolingControlPercent{0};
    // --- Append-only greenfield thermostat-model additions ---
    // Mode/state inputs (bus-writable, mirrored back via provideState).
    bool controllerEnable{true};
    sensor_board::hvac::OperatingPreset hvacOperatingMode{sensor_board::hvac::OperatingPreset::Comfort};
    sensor_board::hvac::ControllerMode controllerMode{sensor_board::hvac::ControllerMode::Auto};
    bool heatCoolChangeover{false};
    bool windowOpen{false};
    bool presence{false};
    bool presenceKnown{false};  // true once a Presence telegram has been received
    float setpointShiftK{0.0f};
    sensor_board::hvac::VentilationMode ventilationMode{sensor_board::hvac::VentilationMode::Auto};
    // Controller outputs.
    sensor_board::hvac::HeatCoolState heatCoolState{sensor_board::hvac::HeatCoolState::Neutral};
    float activeSetpointC{22.0f};
    float setpointShiftFeedbackK{0.0f};
    bool controllerFault{false};
    bool heatingBlocked{false};
    bool coolingBlocked{false};
    uint16_t sensorFaultStatus{0};
    uint8_t ventilationDemandPercent{0};
    sensor_board::hvac::VentilationLevel ventilationLevel{sensor_board::hvac::VentilationLevel::Off};
    bool ventilationActive{false};
    uint16_t iaqStatus{0};
    uint16_t hvacControllerStatus{0};  // DPT 22.101 StatusRHCC word
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

struct PublishSnapshot {
    sensor_data_t sensorData{};
    uint32_t availableMask{0};
    uint32_t pendingSensorMask{0};
    bool publishAll{false};
    bool publishThermostatSetpoint{false};
    bool publishVentilationSetpoint{false};
    bool publishThermostatRequest{false};
    bool publishVentilationBoost{false};
    bool publishCoolingRequest{false};
    bool publishHeatingControl{false};
    bool publishCoolingControl{false};
    bool publishFloorLimit{false};
    float thermostatSetpointC{kDefaultComfortHeatingSetpointC};
    float ventilationSetpointPpm{kDefaultVentilationSetpointPpm};
    bool thermostatRequest{false};
    bool coolingRequest{false};
    bool ventilationBoostRequest{false};
    bool floorLimitActive{false};
    uint8_t heatingControlPercent{0};
    uint8_t coolingControlPercent{0};
    bool toggleProgrammingMode{false};
    // --- Append-only greenfield thermostat-model additions ---
    bool publishControllerEnable{false};
    bool publishHvacOperatingMode{false};
    bool publishControllerMode{false};
    bool publishHeatCoolState{false};
    bool publishActiveSetpoint{false};
    bool publishSetpointShiftFeedback{false};
    bool publishControllerFault{false};
    bool publishSensorFaultStatus{false};
    bool publishHeatingBlocked{false};
    bool publishCoolingBlocked{false};
    bool publishVentilationDemand{false};
    bool publishVentilationLevel{false};
    bool publishVentilationMode{false};
    bool publishVentilationActive{false};
    bool publishIaqStatus{false};
    bool publishHvacControllerStatus{false};
    bool controllerEnable{true};
    sensor_board::hvac::OperatingPreset hvacOperatingMode{sensor_board::hvac::OperatingPreset::Comfort};
    sensor_board::hvac::ControllerMode controllerMode{sensor_board::hvac::ControllerMode::Auto};
    sensor_board::hvac::HeatCoolState heatCoolState{sensor_board::hvac::HeatCoolState::Neutral};
    float activeSetpointC{22.0f};
    float setpointShiftFeedbackK{0.0f};
    bool controllerFault{false};
    uint16_t sensorFaultStatus{0};
    bool heatingBlocked{false};
    bool coolingBlocked{false};
    uint8_t ventilationDemandPercent{0};
    sensor_board::hvac::VentilationLevel ventilationLevel{sensor_board::hvac::VentilationLevel::Off};
    sensor_board::hvac::VentilationMode ventilationMode{sensor_board::hvac::VentilationMode::Auto};
    bool ventilationActive{false};
    uint16_t iaqStatus{0};
    uint16_t hvacControllerStatus{0};
};

template <typename AppT>
void publishPort(AppT &app, SensorBoardPort port, float value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %d", label, static_cast<int>(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, SensorBoardPort port, bool value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %d", label, static_cast<int>(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, SensorBoardPort port, uint8_t value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %d", label, static_cast<int>(result.error()));
    }
}

template <typename AppT>
void publishPort(AppT &app, SensorBoardPort port, uint16_t value, const char *label)
{
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %d", label, static_cast<int>(result.error()));
    }
}

// Push the send-on-change / cyclic-heartbeat transmit policy (see
// knx/application/group_object.hpp) to every transmitting port, driven from
// the ETS-configurable `TemperatureChangeThresholdCenti` /
// `CyclicStatusIntervalSeconds` parameters. Cheap and idempotent, so it is
// simply re-applied every control tick (~1 Hz) instead of only reacting to
// onParameterChanged - that keeps this in sync with the same
// "reconfigure-from-settings-every-tick" pattern used for the controllers
// below. Write-only CommandPorts (HeatCoolChangeover/SetpointShift/WindowOpen/
// Presence) never transmit, so they are intentionally excluded.
template <typename AppT>
void applyTransmitPolicies(AppT &app, const HvacSettings &settings)
{
    GroupObjectTransmitPolicy temperatureLike{};
    temperatureLike.onChangeEnabled = true;
    temperatureLike.changeThreshold = static_cast<double>(settings.temperatureChangeThresholdC);
    temperatureLike.cyclicIntervalMs = settings.cyclicStatusIntervalSeconds * 1000u;

    static constexpr SensorBoardPort kTemperatureLikePorts[] = {
        SensorBoardPort::AirTemperature,
        SensorBoardPort::ProbeTemperature,
        SensorBoardPort::ActiveSetpointFeedback,
        SensorBoardPort::SetpointShiftFeedback,
    };
    for (const auto port : kTemperatureLikePorts) {
        (void)app.setTransmitPolicy(port, temperatureLike);
    }

    // Cyclic heartbeat only (no meaningful engineering-unit delta) for the
    // remaining measurement/status/mode ports.
    GroupObjectTransmitPolicy cyclicOnly{};
    cyclicOnly.cyclicIntervalMs = settings.cyclicStatusIntervalSeconds * 1000u;

    static constexpr SensorBoardPort kCyclicOnlyPorts[] = {
        SensorBoardPort::AirHumidity,
        SensorBoardPort::AirPressure,
        SensorBoardPort::Co2,
        SensorBoardPort::AirQualityIndex,
        SensorBoardPort::Co2Equivalent,
        SensorBoardPort::VocEquivalent,
        SensorBoardPort::AirQualityAccuracy,
        SensorBoardPort::ProbeHumidity,
        SensorBoardPort::ThermostatSetpoint,
        SensorBoardPort::VentilationSetpoint,
        SensorBoardPort::ThermostatRequest,
        SensorBoardPort::VentilationBoostRequest,
        SensorBoardPort::CoolingRequest,
        SensorBoardPort::HeatingControlValue,
        SensorBoardPort::CoolingControlValue,
        SensorBoardPort::FloorLimitActive,
        SensorBoardPort::ControllerEnable,
        SensorBoardPort::HvacOperatingMode,
        SensorBoardPort::ControllerMode,
        SensorBoardPort::HeatCoolState,
        SensorBoardPort::ControllerFault,
        SensorBoardPort::SensorFaultStatus,
        SensorBoardPort::HeatingBlocked,
        SensorBoardPort::CoolingBlocked,
        SensorBoardPort::VentilationDemandPercent,
        SensorBoardPort::VentilationLevel,
        SensorBoardPort::VentilationMode,
        SensorBoardPort::VentilationActive,
        SensorBoardPort::IaqStatus,
        SensorBoardPort::HvacControllerStatus,
    };
    for (const auto port : kCyclicOnlyPorts) {
        (void)app.setTransmitPolicy(port, cyclicOnly);
    }
}

// Room-control tick: snapshot inputs + ETS tuning under the mutex, run the
// (task-owned, lock-free) controllers, store the outputs back and flag the
// changed ones for publishing. Called from the KNX task at ~1 Hz.
template <typename AppT>
void runHvacControlTick(AppT &app,
                        sensor_board::hvac::ThermostatController &thermostat,
                        sensor_board::hvac::VentilationController &ventilation,
                        float dtSeconds)
{
    namespace hvac = sensor_board::hvac;

    HvacSettings settings;
    hvac::ThermostatInputs thermostatIn;
    hvac::VentilationInputs ventIn;
    {
        LockGuard lock(g_state.mutex);
        settings = g_state.hvac;
        thermostatIn.roomTemperatureC = g_state.latestSensorData.temperature;
        thermostatIn.roomTemperatureValid = (g_state.availableMask & SENSOR_TEMPERATURE) != 0u;
        thermostatIn.floorTemperatureC = g_state.latestSensorData.ext_probe_temperature;
        thermostatIn.floorTemperatureValid = (g_state.availableMask & SENSOR_EXT_PROBE_TEMPERATURE) != 0u;
        thermostatIn.controllerEnable = g_state.controllerEnable;
        thermostatIn.hvacOperatingMode = g_state.hvacOperatingMode;
        thermostatIn.controllerMode = g_state.controllerMode;
        thermostatIn.heatCoolChangeoverBusValue = g_state.heatCoolChangeover;
        thermostatIn.windowOpen = g_state.windowOpen;
        thermostatIn.presence = g_state.presence;
        thermostatIn.presenceValid = g_state.presenceKnown;
        thermostatIn.setpointShiftK = g_state.setpointShiftK;

        ventIn.co2Ppm = static_cast<float>(g_state.latestSensorData.co2);
        ventIn.co2Valid = (g_state.availableMask & SENSOR_CO2) != 0u;
        ventIn.humidityPct = g_state.latestSensorData.humidity;
        ventIn.humidityValid = (g_state.availableMask & SENSOR_HUMIDITY) != 0u;
        ventIn.mode = g_state.ventilationMode;
    }

    applyTransmitPolicies(app, settings);

    // Reconfigure from the ETS parameters every tick: configure() preserves
    // controller state (integrators, latches), so this is cheap and makes a
    // live ETS re-parameterisation take effect within one tick.
    hvac::ThermostatConfig thermostatCfg;
    thermostatCfg.heatingPid = {.kp = settings.heatingKp,
                                .tiSeconds = settings.heatingTiSeconds,
                                .tdSeconds = settings.heatingTdSeconds};
    thermostatCfg.coolingPid = {.kp = settings.coolingKp,
                                .tiSeconds = settings.coolingTiSeconds,
                                .tdSeconds = settings.coolingTdSeconds,
                                .reverseActing = true};
    thermostatCfg.heatingSetpoints = {.comfortC = settings.comfortHeatingSetpointC,
                                      .standbyC = settings.standbyHeatingSetpointC,
                                      .economyC = settings.economyHeatingSetpointC,
                                      .protectionC = settings.protectionHeatingSetpointC};
    thermostatCfg.coolingSetpoints = {.comfortC = settings.comfortCoolingSetpointC,
                                      .standbyC = settings.standbyCoolingSetpointC,
                                      .economyC = settings.economyCoolingSetpointC,
                                      .protectionC = settings.protectionCoolingSetpointC};
    thermostatCfg.minSetpointC = settings.minSetpointC;
    thermostatCfg.maxSetpointC = settings.maxSetpointC;
    thermostatCfg.maxSetpointShiftK = settings.maxSetpointShiftK;
    thermostatCfg.coolingDeadbandK = settings.coolingDeadbandK;
    thermostatCfg.requestHysteresisK = settings.thermostatHysteresisC;
    thermostatCfg.maxFloorTemperatureC = settings.maxFloorTemperatureC;
    thermostatCfg.floorHysteresisK = settings.floorHysteresisK;
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
                           .co2HysteresisPpm = settings.ventilationHysteresisPpm,
                           .humidityThresholdPct = settings.humidityBoostPct,
                           .humidityHysteresisPct = settings.humidityBoostHysteresisPct});

    const auto out = thermostat.update(thermostatIn, dtSeconds);
    const auto ventOut = ventilation.update(ventIn);

    // Sensor-fault bitset (device-specific bit layout, documented next to
    // SensorBoardPort::SensorFaultStatus): bit0=AirTemp, bit1=Humidity,
    // bit2=Co2, bit3=ProbeTemperature, bit4=ProbeHumidity.
    uint16_t faultBits = 0;
    {
        LockGuard lock(g_state.mutex);
        if ((g_state.availableMask & SENSOR_TEMPERATURE) == 0u) faultBits |= (1u << 0);
        if ((g_state.availableMask & SENSOR_HUMIDITY) == 0u) faultBits |= (1u << 1);
        if ((g_state.availableMask & SENSOR_CO2) == 0u) faultBits |= (1u << 2);
        if ((g_state.availableMask & SENSOR_EXT_PROBE_TEMPERATURE) == 0u) faultBits |= (1u << 3);
        if ((g_state.availableMask & SENSOR_EXT_PROBE_HUMIDITY) == 0u) faultBits |= (1u << 4);
    }

    uint16_t iaqBits = 0;
    if (ventOut.humidityHigh) iaqBits |= static_cast<uint16_t>(hvac::IaqStatusBit::HumidityBoost);
    if (ventOut.co2High) iaqBits |= static_cast<uint16_t>(hvac::IaqStatusBit::Co2Boost);
    if (ventOut.sensorFault) iaqBits |= static_cast<uint16_t>(hvac::IaqStatusBit::SensorFault);

    // KNX-standard room heating/cooling controller status word (DPT 22.101).
    const uint16_t statusRhcc = hvac::packStatusRHCC(out);

    {
        LockGuard lock(g_state.mutex);
        if (out.heatingRequest != g_state.thermostatRequest) {
            g_state.thermostatRequest = out.heatingRequest;
            g_state.pendingThermostatRequestPublish = true;
        }
        if (out.coolingRequest != g_state.coolingRequest) {
            g_state.coolingRequest = out.coolingRequest;
            g_state.pendingCoolingRequestPublish = true;
        }
        if (out.heatingControlPercent != g_state.heatingControlPercent) {
            g_state.heatingControlPercent = out.heatingControlPercent;
            g_state.pendingHeatingControlPublish = true;
        }
        if (out.coolingControlPercent != g_state.coolingControlPercent) {
            g_state.coolingControlPercent = out.coolingControlPercent;
            g_state.pendingCoolingControlPublish = true;
        }
        if (out.floorLimitActive != g_state.floorLimitActive) {
            g_state.floorLimitActive = out.floorLimitActive;
            g_state.pendingFloorLimitPublish = true;
        }
        if (out.heatCoolState != g_state.heatCoolState) {
            g_state.heatCoolState = out.heatCoolState;
            g_state.pendingHeatCoolStatePublish = true;
        }
        if (std::fabs(out.activeSetpointC - g_state.activeSetpointC) >= settings.temperatureChangeThresholdC) {
            g_state.activeSetpointC = out.activeSetpointC;
            g_state.pendingActiveSetpointPublish = true;
        }
        if (std::fabs(out.setpointShiftFeedbackK - g_state.setpointShiftFeedbackK) >= 0.01f) {
            g_state.setpointShiftFeedbackK = out.setpointShiftFeedbackK;
            g_state.pendingSetpointShiftFeedbackPublish = true;
        }
        if (out.controllerFault != g_state.controllerFault) {
            g_state.controllerFault = out.controllerFault;
            g_state.pendingControllerFaultPublish = true;
        }
        if (out.heatingBlocked != g_state.heatingBlocked) {
            g_state.heatingBlocked = out.heatingBlocked;
            g_state.pendingHeatingBlockedPublish = true;
        }
        if (out.coolingBlocked != g_state.coolingBlocked) {
            g_state.coolingBlocked = out.coolingBlocked;
            g_state.pendingCoolingBlockedPublish = true;
        }
        if (faultBits != g_state.sensorFaultStatus) {
            g_state.sensorFaultStatus = faultBits;
            g_state.pendingSensorFaultStatusPublish = true;
        }
        if (ventOut.demandPercent != g_state.ventilationDemandPercent) {
            g_state.ventilationDemandPercent = ventOut.demandPercent;
            g_state.pendingVentilationDemandPublish = true;
        }
        if (ventOut.level != g_state.ventilationLevel) {
            g_state.ventilationLevel = ventOut.level;
            g_state.pendingVentilationLevelPublish = true;
        }
        if (ventOut.active != g_state.ventilationActive) {
            g_state.ventilationActive = ventOut.active;
            g_state.pendingVentilationActivePublish = true;
        }
        if (iaqBits != g_state.iaqStatus) {
            g_state.iaqStatus = iaqBits;
            g_state.pendingIaqStatusPublish = true;
        }
        if (statusRhcc != g_state.hvacControllerStatus) {
            g_state.hvacControllerStatus = statusRhcc;
            g_state.pendingHvacControllerStatusPublish = true;
        }
        if (ventOut.active != g_state.ventilationBoostRequest) {
            g_state.ventilationBoostRequest = ventOut.active;
            g_state.pendingVentilationBoostPublish = true;
        }
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

    return physical::createTp1PhysicalForPlatform(selection, dependencies);
}

PublishSnapshot takePublishSnapshot()
{
    PublishSnapshot snapshot;
    LockGuard lock(g_state.mutex);
    snapshot.sensorData = g_state.latestSensorData;
    snapshot.availableMask = g_state.availableMask;
    snapshot.pendingSensorMask = g_state.pendingSensorMask;
    snapshot.publishAll = g_state.initialPublishPending;
    snapshot.publishThermostatSetpoint = g_state.pendingThermostatSetpointPublish;
    snapshot.publishVentilationSetpoint = g_state.pendingVentilationSetpointPublish;
    snapshot.publishThermostatRequest = g_state.pendingThermostatRequestPublish;
    snapshot.publishVentilationBoost = g_state.pendingVentilationBoostPublish;
    snapshot.publishCoolingRequest = g_state.pendingCoolingRequestPublish;
    snapshot.publishHeatingControl = g_state.pendingHeatingControlPublish;
    snapshot.publishCoolingControl = g_state.pendingCoolingControlPublish;
    snapshot.publishFloorLimit = g_state.pendingFloorLimitPublish;
    snapshot.thermostatSetpointC = g_state.hvac.comfortHeatingSetpointC;
    snapshot.ventilationSetpointPpm = g_state.hvac.ventilationSetpointPpm;
    snapshot.thermostatRequest = g_state.thermostatRequest;
    snapshot.coolingRequest = g_state.coolingRequest;
    snapshot.ventilationBoostRequest = g_state.ventilationBoostRequest;
    snapshot.floorLimitActive = g_state.floorLimitActive;
    snapshot.heatingControlPercent = g_state.heatingControlPercent;
    snapshot.coolingControlPercent = g_state.coolingControlPercent;
    snapshot.toggleProgrammingMode = g_state.toggleProgrammingModeRequested;

    // --- Append-only greenfield thermostat-model additions ---
    snapshot.publishControllerEnable = g_state.pendingControllerEnablePublish;
    snapshot.publishHvacOperatingMode = g_state.pendingHvacOperatingModePublish;
    snapshot.publishControllerMode = g_state.pendingControllerModePublish;
    snapshot.publishHeatCoolState = g_state.pendingHeatCoolStatePublish;
    snapshot.publishActiveSetpoint = g_state.pendingActiveSetpointPublish;
    snapshot.publishSetpointShiftFeedback = g_state.pendingSetpointShiftFeedbackPublish;
    snapshot.publishControllerFault = g_state.pendingControllerFaultPublish;
    snapshot.publishSensorFaultStatus = g_state.pendingSensorFaultStatusPublish;
    snapshot.publishHeatingBlocked = g_state.pendingHeatingBlockedPublish;
    snapshot.publishCoolingBlocked = g_state.pendingCoolingBlockedPublish;
    snapshot.publishVentilationDemand = g_state.pendingVentilationDemandPublish;
    snapshot.publishVentilationLevel = g_state.pendingVentilationLevelPublish;
    snapshot.publishVentilationMode = g_state.pendingVentilationModePublish;
    snapshot.publishVentilationActive = g_state.pendingVentilationActivePublish;
    snapshot.publishIaqStatus = g_state.pendingIaqStatusPublish;
    snapshot.publishHvacControllerStatus = g_state.pendingHvacControllerStatusPublish;
    snapshot.controllerEnable = g_state.controllerEnable;
    snapshot.hvacOperatingMode = g_state.hvacOperatingMode;
    snapshot.controllerMode = g_state.controllerMode;
    snapshot.heatCoolState = g_state.heatCoolState;
    snapshot.activeSetpointC = g_state.activeSetpointC;
    snapshot.setpointShiftFeedbackK = g_state.setpointShiftFeedbackK;
    snapshot.controllerFault = g_state.controllerFault;
    snapshot.sensorFaultStatus = g_state.sensorFaultStatus;
    snapshot.heatingBlocked = g_state.heatingBlocked;
    snapshot.coolingBlocked = g_state.coolingBlocked;
    snapshot.ventilationDemandPercent = g_state.ventilationDemandPercent;
    snapshot.ventilationLevel = g_state.ventilationLevel;
    snapshot.ventilationMode = g_state.ventilationMode;
    snapshot.ventilationActive = g_state.ventilationActive;
    snapshot.iaqStatus = g_state.iaqStatus;
    snapshot.hvacControllerStatus = g_state.hvacControllerStatus;

    g_state.pendingSensorMask = 0;
    g_state.initialPublishPending = false;
    g_state.pendingThermostatSetpointPublish = false;
    g_state.pendingVentilationSetpointPublish = false;
    g_state.pendingThermostatRequestPublish = false;
    g_state.pendingVentilationBoostPublish = false;
    g_state.pendingCoolingRequestPublish = false;
    g_state.pendingHeatingControlPublish = false;
    g_state.pendingCoolingControlPublish = false;
    g_state.pendingFloorLimitPublish = false;
    g_state.toggleProgrammingModeRequested = false;
    g_state.pendingControllerEnablePublish = false;
    g_state.pendingHvacOperatingModePublish = false;
    g_state.pendingControllerModePublish = false;
    g_state.pendingHeatCoolStatePublish = false;
    g_state.pendingActiveSetpointPublish = false;
    g_state.pendingSetpointShiftFeedbackPublish = false;
    g_state.pendingControllerFaultPublish = false;
    g_state.pendingSensorFaultStatusPublish = false;
    g_state.pendingHeatingBlockedPublish = false;
    g_state.pendingCoolingBlockedPublish = false;
    g_state.pendingVentilationDemandPublish = false;
    g_state.pendingVentilationLevelPublish = false;
    g_state.pendingVentilationModePublish = false;
    g_state.pendingVentilationActivePublish = false;
    g_state.pendingIaqStatusPublish = false;
    g_state.pendingHvacControllerStatusPublish = false;

    return snapshot;
}

template <typename AppT>
void publishPendingState(AppT &app, const PublishSnapshot &snapshot)
{
    const uint32_t publishMask = snapshot.publishAll ? snapshot.availableMask : snapshot.pendingSensorMask;

    if ((publishMask & SENSOR_TEMPERATURE) != 0u) {
        publishPort(app, SensorBoardPort::AirTemperature, snapshot.sensorData.temperature, "air temperature");
    }
    if ((publishMask & SENSOR_HUMIDITY) != 0u) {
        publishPort(app, SensorBoardPort::AirHumidity, snapshot.sensorData.humidity, "air humidity");
    }
    if ((publishMask & SENSOR_PRESSURE) != 0u) {
        publishPort(app, SensorBoardPort::AirPressure, snapshot.sensorData.pressure, "air pressure");
    }
    if ((publishMask & SENSOR_IAQ) != 0u) {
        publishPort(app, SensorBoardPort::AirQualityIndex,
                   static_cast<uint16_t>(snapshot.sensorData.iaq + 0.5f), "air quality index");
        publishPort(app, SensorBoardPort::AirQualityAccuracy,
                   snapshot.sensorData.air_quality_accuracy, "air quality accuracy");
    }
    if ((publishMask & SENSOR_CO2_EQUIVALENT) != 0u) {
        publishPort(app, SensorBoardPort::Co2Equivalent, snapshot.sensorData.co2_equivalent, "co2 equivalent");
    }
    if ((publishMask & SENSOR_VOC_EQUIVALENT) != 0u) {
        publishPort(app, SensorBoardPort::VocEquivalent, snapshot.sensorData.voc_equivalent, "voc equivalent");
    }
    if ((publishMask & SENSOR_CO2) != 0u) {
        publishPort(app, SensorBoardPort::Co2, static_cast<float>(snapshot.sensorData.co2), "co2");
    }
    if ((publishMask & SENSOR_EXT_PROBE_TEMPERATURE) != 0u) {
        publishPort(app, SensorBoardPort::ProbeTemperature, snapshot.sensorData.ext_probe_temperature, "probe temperature");
    }
    if ((publishMask & SENSOR_EXT_PROBE_HUMIDITY) != 0u) {
        publishPort(app, SensorBoardPort::ProbeHumidity, snapshot.sensorData.ext_probe_humidity, "probe humidity");
    }

    if (snapshot.publishAll || snapshot.publishThermostatSetpoint) {
        publishPort(app, SensorBoardPort::ThermostatSetpoint, snapshot.thermostatSetpointC, "thermostat setpoint");
    }
    if (snapshot.publishAll || snapshot.publishVentilationSetpoint) {
        publishPort(app, SensorBoardPort::VentilationSetpoint, snapshot.ventilationSetpointPpm, "ventilation setpoint");
    }
    if (snapshot.publishAll || snapshot.publishThermostatRequest) {
        publishPort(app, SensorBoardPort::ThermostatRequest, snapshot.thermostatRequest, "thermostat request");
    }
    if (snapshot.publishAll || snapshot.publishVentilationBoost) {
        publishPort(app, SensorBoardPort::VentilationBoostRequest, snapshot.ventilationBoostRequest, "ventilation boost request");
    }
    if (snapshot.publishAll || snapshot.publishCoolingRequest) {
        publishPort(app, SensorBoardPort::CoolingRequest, snapshot.coolingRequest, "cooling request");
    }
    if (snapshot.publishAll || snapshot.publishHeatingControl) {
        publishPort(app, SensorBoardPort::HeatingControlValue, snapshot.heatingControlPercent, "heating control value");
    }
    if (snapshot.publishAll || snapshot.publishCoolingControl) {
        publishPort(app, SensorBoardPort::CoolingControlValue, snapshot.coolingControlPercent, "cooling control value");
    }
    if (snapshot.publishAll || snapshot.publishFloorLimit) {
        publishPort(app, SensorBoardPort::FloorLimitActive, snapshot.floorLimitActive, "floor limit active");
    }

    // --- Append-only greenfield thermostat-model additions ---
    if (snapshot.publishAll || snapshot.publishControllerEnable) {
        publishPort(app, SensorBoardPort::ControllerEnable, snapshot.controllerEnable, "controller enable");
    }
    if (snapshot.publishAll || snapshot.publishHvacOperatingMode) {
        const auto result = app.publish(SensorBoardPort::HvacOperatingMode,
                                        static_cast<Dpt20Mode>(snapshot.hvacOperatingMode));
        if (result.isError()) {
            KNX_LOGW(TAG, "hvac operating mode publish failed: %d", static_cast<int>(result.error()));
        }
    }
    if (snapshot.publishAll || snapshot.publishControllerMode) {
        publishPort(app, SensorBoardPort::ControllerMode,
                   sensor_board::hvac::controllerModeToContrMode(snapshot.controllerMode),
                   "controller mode");
    }
    if (snapshot.publishAll || snapshot.publishHeatCoolState) {
        publishPort(app, SensorBoardPort::HeatCoolState,
                   static_cast<uint8_t>(snapshot.heatCoolState), "heat/cool state");
    }
    if (snapshot.publishAll || snapshot.publishActiveSetpoint) {
        publishPort(app, SensorBoardPort::ActiveSetpointFeedback, snapshot.activeSetpointC, "active setpoint feedback");
    }
    if (snapshot.publishAll || snapshot.publishSetpointShiftFeedback) {
        publishPort(app, SensorBoardPort::SetpointShiftFeedback, snapshot.setpointShiftFeedbackK, "setpoint shift feedback");
    }
    if (snapshot.publishAll || snapshot.publishControllerFault) {
        publishPort(app, SensorBoardPort::ControllerFault, snapshot.controllerFault, "controller fault");
    }
    if (snapshot.publishAll || snapshot.publishSensorFaultStatus) {
        publishPort(app, SensorBoardPort::SensorFaultStatus, snapshot.sensorFaultStatus, "sensor fault status");
    }
    if (snapshot.publishAll || snapshot.publishHeatingBlocked) {
        publishPort(app, SensorBoardPort::HeatingBlocked, snapshot.heatingBlocked, "heating blocked");
    }
    if (snapshot.publishAll || snapshot.publishCoolingBlocked) {
        publishPort(app, SensorBoardPort::CoolingBlocked, snapshot.coolingBlocked, "cooling blocked");
    }
    if (snapshot.publishAll || snapshot.publishVentilationDemand) {
        publishPort(app, SensorBoardPort::VentilationDemandPercent, snapshot.ventilationDemandPercent, "ventilation demand");
    }
    if (snapshot.publishAll || snapshot.publishVentilationLevel) {
        publishPort(app, SensorBoardPort::VentilationLevel,
                   static_cast<uint8_t>(snapshot.ventilationLevel), "ventilation level");
    }
    if (snapshot.publishAll || snapshot.publishVentilationMode) {
        publishPort(app, SensorBoardPort::VentilationMode,
                   static_cast<uint8_t>(snapshot.ventilationMode), "ventilation mode");
    }
    if (snapshot.publishAll || snapshot.publishVentilationActive) {
        publishPort(app, SensorBoardPort::VentilationActive, snapshot.ventilationActive, "ventilation active");
    }
    if (snapshot.publishAll || snapshot.publishIaqStatus) {
        publishPort(app, SensorBoardPort::IaqStatus, snapshot.iaqStatus, "iaq status");
    }
    if (snapshot.publishAll || snapshot.publishHvacControllerStatus) {
        publishPort(app, SensorBoardPort::HvacControllerStatus, snapshot.hvacControllerStatus,
                   "hvac controller status");
    }
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

    // The runtime is kept on the heap: it is a multi-KB object that would
    // otherwise stay resident on this task's stack for the task's lifetime.
    // The bindings builder and the start Result (also multi-KB) are scoped so
    // their stack space is released before the service loop runs.
    using AppRuntime = CommissionedProductRuntime<std::remove_cvref_t<decltype(kSensorBoardProduct)>,
                                                  kDefaultBindingCapacity>;
    std::unique_ptr<AppRuntime> appPtr;
    {
    auto bindings = makeCommissionedBindings(kSensorBoardProduct)
        // Room setpoint write from an HMI / HA target_temperature: update the
        // live comfort heating setpoint (the base the controller resolves from),
        // clamped to the configured [min,max] band. The effective per-mode
        // setpoint is reported on ActiveSetpointFeedback.
        .onStateWrite<SensorBoardPort::ThermostatSetpoint>([](float value) {
            LockGuard lock(g_state.mutex);
            const float clamped =
                sensor_board::hvac::clampf(value, g_state.hvac.minSetpointC, g_state.hvac.maxSetpointC);
            if (std::fabs(g_state.hvac.comfortHeatingSetpointC - clamped) < 0.01f) {
                return;
            }
            g_state.hvac.comfortHeatingSetpointC = clamped;
            g_state.pendingThermostatSetpointPublish = true;
            ESP_LOGI(TAG, "Room comfort setpoint updated to %.2f C", clamped);
        })
        .provideState<SensorBoardPort::ThermostatSetpoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.comfortHeatingSetpointC;
        })
        .onStateWrite<SensorBoardPort::VentilationSetpoint>([](float value) {
            LockGuard lock(g_state.mutex);
            if (std::fabs(g_state.hvac.ventilationSetpointPpm - value) < 0.5f) {
                return;
            }
            g_state.hvac.ventilationSetpointPpm = value;
            g_state.pendingVentilationSetpointPublish = true;
            ESP_LOGI(TAG, "Ventilation setpoint updated to %.0f ppm", value);
        })
        .provideState<SensorBoardPort::VentilationSetpoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.ventilationSetpointPpm;
        })
        .provideState<SensorBoardPort::AirTemperature>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.temperature;
        })
        .provideState<SensorBoardPort::AirHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.humidity;
        })
        .provideState<SensorBoardPort::AirPressure>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.pressure;
        })
        .provideState<SensorBoardPort::Co2>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<float>(g_state.latestSensorData.co2);
        })
        .provideState<SensorBoardPort::AirQualityIndex>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint16_t>(g_state.latestSensorData.iaq + 0.5f);
        })
        .provideState<SensorBoardPort::Co2Equivalent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.co2_equivalent;
        })
        .provideState<SensorBoardPort::VocEquivalent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.voc_equivalent;
        })
        .provideState<SensorBoardPort::AirQualityAccuracy>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.air_quality_accuracy;
        })
        .provideState<SensorBoardPort::ProbeTemperature>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.ext_probe_temperature;
        })
        .provideState<SensorBoardPort::ProbeHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.ext_probe_humidity;
        })
        .provideState<SensorBoardPort::ThermostatRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.thermostatRequest;
        })
        .provideState<SensorBoardPort::VentilationBoostRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.ventilationBoostRequest;
        })
        .provideState<SensorBoardPort::CoolingRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.coolingRequest;
        })
        .provideState<SensorBoardPort::HeatingControlValue>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.heatingControlPercent;
        })
        .provideState<SensorBoardPort::CoolingControlValue>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.coolingControlPercent;
        })
        .provideState<SensorBoardPort::FloorLimitActive>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.floorLimitActive;
        })
        // --- Append-only greenfield thermostat-model additions ---
        .onStateWrite<SensorBoardPort::ControllerEnable>([](bool value) {
            LockGuard lock(g_state.mutex);
            if (g_state.controllerEnable == value) return;
            g_state.controllerEnable = value;
            g_state.pendingControllerEnablePublish = true;
        })
        .provideState<SensorBoardPort::ControllerEnable>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.controllerEnable;
        })
        .onStateWrite<SensorBoardPort::HvacOperatingMode>([](Dpt20Mode value) {
            const auto mode = static_cast<sensor_board::hvac::OperatingPreset>(value);
            LockGuard lock(g_state.mutex);
            if (g_state.hvacOperatingMode == mode) return;
            g_state.hvacOperatingMode = mode;
            g_state.pendingHvacOperatingModePublish = true;
        })
        .provideState<SensorBoardPort::HvacOperatingMode>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<Dpt20Mode>(g_state.hvacOperatingMode);
        })
        // ControllerMode on the bus is DPT 20.105 (HVACContrMode); map to/from
        // the compact internal enum at the binding boundary.
        .onStateWrite<SensorBoardPort::ControllerMode>([](uint8_t value) {
            const auto mode = sensor_board::hvac::controllerModeFromContrMode(value);
            LockGuard lock(g_state.mutex);
            if (g_state.controllerMode == mode) return;
            g_state.controllerMode = mode;
            g_state.pendingControllerModePublish = true;
        })
        .provideState<SensorBoardPort::ControllerMode>([]() {
            LockGuard lock(g_state.mutex);
            return sensor_board::hvac::controllerModeToContrMode(g_state.controllerMode);
        })
        .onStateWrite<SensorBoardPort::HeatCoolChangeover>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.heatCoolChangeover = value;
        })
        .provideState<SensorBoardPort::HeatCoolState>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.heatCoolState);
        })
        .provideState<SensorBoardPort::ActiveSetpointFeedback>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.activeSetpointC;
        })
        .onStateWrite<SensorBoardPort::SetpointShift>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.setpointShiftK = value;
        })
        .provideState<SensorBoardPort::SetpointShiftFeedback>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.setpointShiftFeedbackK;
        })
        .onStateWrite<SensorBoardPort::WindowOpen>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.windowOpen = value;
        })
        .onStateWrite<SensorBoardPort::Presence>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.presence = value;
            g_state.presenceKnown = true;
        })
        .provideState<SensorBoardPort::ControllerFault>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.controllerFault;
        })
        .provideState<SensorBoardPort::SensorFaultStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.sensorFaultStatus;
        })
        .provideState<SensorBoardPort::HeatingBlocked>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.heatingBlocked;
        })
        .provideState<SensorBoardPort::CoolingBlocked>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.coolingBlocked;
        })
        .provideState<SensorBoardPort::VentilationDemandPercent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.ventilationDemandPercent;
        })
        .provideState<SensorBoardPort::VentilationLevel>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.ventilationLevel);
        })
        .onStateWrite<SensorBoardPort::VentilationMode>([](uint8_t value) {
            const auto mode = static_cast<sensor_board::hvac::VentilationMode>(value);
            LockGuard lock(g_state.mutex);
            if (g_state.ventilationMode == mode) return;
            g_state.ventilationMode = mode;
            g_state.pendingVentilationModePublish = true;
        })
        .provideState<SensorBoardPort::VentilationMode>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.ventilationMode);
        })
        .provideState<SensorBoardPort::VentilationActive>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.ventilationActive;
        })
        .provideState<SensorBoardPort::IaqStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.iaqStatus;
        })
        .provideState<SensorBoardPort::HvacControllerStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvacControllerStatus;
        })
        // Fractional ETS parameters arrive as Dpt9Float (KNX 2-byte half-float,
        // implicitly convertible to float); ppm/second parameters as uint16_t.
        .onParameterChanged<SensorBoardParameter::DefaultVentilationSetpoint>([](uint16_t value) {
            const float setpointPpm = static_cast<float>(value);
            LockGuard lock(g_state.mutex);
            if (std::fabs(g_state.hvac.ventilationSetpointPpm - setpointPpm) < 0.5f) {
                return;
            }
            g_state.hvac.ventilationSetpointPpm = setpointPpm;
            g_state.pendingVentilationSetpointPublish = true;
        })
        .onParameterChanged<SensorBoardParameter::ThermostatHysteresis>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.thermostatHysteresisC = value;
        })
        .onParameterChanged<SensorBoardParameter::VentilationHysteresis>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.ventilationHysteresisPpm = static_cast<float>(value);
        })
        .onParameterChanged<SensorBoardParameter::HeatingKp>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingKp = value;
        })
        .onParameterChanged<SensorBoardParameter::HeatingTiSeconds>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingTiSeconds = static_cast<float>(value);
        })
        .onParameterChanged<SensorBoardParameter::HeatingTdSeconds>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingTdSeconds = static_cast<float>(value);
        })
        .onParameterChanged<SensorBoardParameter::CoolingKp>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingKp = value;
        })
        .onParameterChanged<SensorBoardParameter::CoolingTiSeconds>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingTiSeconds = static_cast<float>(value);
        })
        .onParameterChanged<SensorBoardParameter::CoolingTdSeconds>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingTdSeconds = static_cast<float>(value);
        })
        .onParameterChanged<SensorBoardParameter::CoolingDeadband>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingDeadbandK = value;
        })
        .onParameterChanged<SensorBoardParameter::MaxFloorTemperature>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxFloorTemperatureC = value;
        })
        .onParameterChanged<SensorBoardParameter::FloorHysteresis>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.floorHysteresisK = value;
        })
        .onParameterChanged<SensorBoardParameter::HumidityBoostThreshold>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.humidityBoostPct = value;
        })
        .onParameterChanged<SensorBoardParameter::HumidityBoostHysteresis>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.humidityBoostHysteresisPct = value;
        })
        // --- Append-only greenfield thermostat-model additions ---
        .onParameterChanged<SensorBoardParameter::ControllerDefaultEnable>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.controllerDefaultEnable = (value != 0);
        })
        .onParameterChanged<SensorBoardParameter::DefaultHvacOperatingMode>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.defaultHvacOperatingMode = static_cast<sensor_board::hvac::OperatingPreset>(value);
        })
        .onParameterChanged<SensorBoardParameter::DefaultControllerMode>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.defaultControllerMode = static_cast<sensor_board::hvac::ControllerMode>(value);
        })
        .onParameterChanged<SensorBoardParameter::ComfortHeatingSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.comfortHeatingSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::StandbyHeatingSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.standbyHeatingSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::EconomyHeatingSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.economyHeatingSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::ProtectionHeatingSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.protectionHeatingSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::ComfortCoolingSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.comfortCoolingSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::StandbyCoolingSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.standbyCoolingSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::EconomyCoolingSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.economyCoolingSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::ProtectionCoolingSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.protectionCoolingSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::MinSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.minSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::MaxSetpoint>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxSetpointC = value;
        })
        .onParameterChanged<SensorBoardParameter::MaxSetpointShift>([](Dpt9Float value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.maxSetpointShiftK = value;
        })
        .onParameterChanged<SensorBoardParameter::HeatingEnabled>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingEnabled = (value != 0);
        })
        .onParameterChanged<SensorBoardParameter::CoolingEnabled>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingEnabled = (value != 0);
        })
        .onParameterChanged<SensorBoardParameter::HeatCoolChangeoverMode>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatCoolChangeoverMode = static_cast<sensor_board::hvac::HeatCoolChangeoverMode>(value);
        })
        .onParameterChanged<SensorBoardParameter::HeatCoolChangeoverPolarity>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatCoolChangeoverPolarityInverted = (value != 0);
        })
        .onParameterChanged<SensorBoardParameter::HeatingControlAlgorithm>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingControlAlgorithm = static_cast<sensor_board::hvac::ControlAlgorithm>(value);
        })
        .onParameterChanged<SensorBoardParameter::CoolingControlAlgorithm>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingControlAlgorithm = static_cast<sensor_board::hvac::ControlAlgorithm>(value);
        })
        .onParameterChanged<SensorBoardParameter::HeatingMinimumOutputPercent>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingMinOutputPercent = static_cast<uint8_t>(value);
        })
        .onParameterChanged<SensorBoardParameter::CoolingMinimumOutputPercent>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingMinOutputPercent = static_cast<uint8_t>(value);
        })
        .onParameterChanged<SensorBoardParameter::HeatingMaximumOutputPercent>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.heatingMaxOutputPercent = static_cast<uint8_t>(value);
        })
        .onParameterChanged<SensorBoardParameter::CoolingMaximumOutputPercent>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.coolingMaxOutputPercent = static_cast<uint8_t>(value);
        })
        .onParameterChanged<SensorBoardParameter::BinaryDemandStrategy>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.binaryDemandStrategy = static_cast<sensor_board::hvac::BinaryDemandStrategy>(value);
        })
        .onParameterChanged<SensorBoardParameter::BinaryDemandThresholdPercent>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.binaryDemandThresholdPercent = static_cast<uint8_t>(value);
        })
        .onParameterChanged<SensorBoardParameter::MinimumHeatCoolChangeoverSeconds>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.minimumHeatCoolChangeoverSeconds = static_cast<float>(value);
        })
        .onParameterChanged<SensorBoardParameter::WindowOpenBehavior>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.windowOpenBehavior = static_cast<sensor_board::hvac::WindowOpenBehavior>(value);
        })
        .onParameterChanged<SensorBoardParameter::PresenceBehavior>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.presenceBehavior = static_cast<sensor_board::hvac::PresenceBehavior>(value);
        })
        .onParameterChanged<SensorBoardParameter::SensorFaultBehavior>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.sensorFaultBehavior = static_cast<sensor_board::hvac::SensorFaultBehavior>(value);
        })
        .onParameterChanged<SensorBoardParameter::TemperatureChangeThresholdCenti>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.temperatureChangeThresholdC = static_cast<float>(value) / 100.0f;
        })
        .onParameterChanged<SensorBoardParameter::CyclicStatusIntervalSeconds>([](uint16_t value) {
            LockGuard lock(g_state.mutex);
            g_state.hvac.cyclicStatusIntervalSeconds = value;
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

    // Same move the previous stack-local `app` performed — the runtime's move
    // constructor rewires its internal callbacks — just targeting heap storage.
    appPtr = std::make_unique<AppRuntime>(std::move(appResult.value()));
    } // release bindings builder + start Result stack space
    auto &app = *appPtr;

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
        // Provision (first boot) or reuse a device-unique KNX Data Secure tool
        // key and log it in hex so an installer can capture it locally and
        // enter it, together with the serial number above, into ETS for secure
        // commissioning. The generated key is the device's FDSK / initial tool
        // key.
        std::array<uint8_t, 16> toolKey{};
        bool created = false;
        if (loadOrCreateToolKey(toolKey, created)) {
            char keyHex[3 * 16] = {};
            toHex(std::span<const uint8_t>(toolKey.data(), toolKey.size()), ' ', keyHex, sizeof(keyHex));
            const auto secureResult = app.applyEtsToolKey(toolKey);
            if (secureResult.isError()) {
                KNX_LOGE(TAG, "Data Secure: applyEtsToolKey failed: %d",
                         static_cast<int>(secureResult.error()));
            } else {
                KNX_LOGW(TAG,
                         "KNX Data Secure ENABLED. Tool key (%s) — enter into ETS with the "
                         "serial number for secure commissioning: %s",
                         created ? "generated" : "stored", keyHex);
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
        // Seed engineering values from the ETS parameters: Dpt9Float ones
        // convert implicitly to float; ppm/second ones are plain uint16.
        auto params = app.parameters();
        g_state.hvac.ventilationSetpointPpm = static_cast<float>(params.get<SensorBoardParameter::DefaultVentilationSetpoint>());
        g_state.hvac.thermostatHysteresisC = params.get<SensorBoardParameter::ThermostatHysteresis>();
        g_state.hvac.ventilationHysteresisPpm = static_cast<float>(params.get<SensorBoardParameter::VentilationHysteresis>());
        g_state.hvac.heatingKp = params.get<SensorBoardParameter::HeatingKp>();
        g_state.hvac.heatingTiSeconds = static_cast<float>(params.get<SensorBoardParameter::HeatingTiSeconds>());
        g_state.hvac.heatingTdSeconds = static_cast<float>(params.get<SensorBoardParameter::HeatingTdSeconds>());
        g_state.hvac.coolingKp = params.get<SensorBoardParameter::CoolingKp>();
        g_state.hvac.coolingTiSeconds = static_cast<float>(params.get<SensorBoardParameter::CoolingTiSeconds>());
        g_state.hvac.coolingTdSeconds = static_cast<float>(params.get<SensorBoardParameter::CoolingTdSeconds>());
        g_state.hvac.coolingDeadbandK = params.get<SensorBoardParameter::CoolingDeadband>();
        g_state.hvac.maxFloorTemperatureC = params.get<SensorBoardParameter::MaxFloorTemperature>();
        g_state.hvac.floorHysteresisK = params.get<SensorBoardParameter::FloorHysteresis>();
        g_state.hvac.humidityBoostPct = params.get<SensorBoardParameter::HumidityBoostThreshold>();
        g_state.hvac.humidityBoostHysteresisPct = params.get<SensorBoardParameter::HumidityBoostHysteresis>();
        // --- Append-only greenfield thermostat-model additions ---
        g_state.hvac.controllerDefaultEnable = params.get<SensorBoardParameter::ControllerDefaultEnable>() != 0;
        g_state.hvac.defaultHvacOperatingMode =
            static_cast<sensor_board::hvac::OperatingPreset>(params.get<SensorBoardParameter::DefaultHvacOperatingMode>());
        g_state.hvac.defaultControllerMode =
            static_cast<sensor_board::hvac::ControllerMode>(params.get<SensorBoardParameter::DefaultControllerMode>());
        g_state.hvac.comfortHeatingSetpointC = params.get<SensorBoardParameter::ComfortHeatingSetpoint>();
        g_state.hvac.standbyHeatingSetpointC = params.get<SensorBoardParameter::StandbyHeatingSetpoint>();
        g_state.hvac.economyHeatingSetpointC = params.get<SensorBoardParameter::EconomyHeatingSetpoint>();
        g_state.hvac.protectionHeatingSetpointC = params.get<SensorBoardParameter::ProtectionHeatingSetpoint>();
        g_state.hvac.comfortCoolingSetpointC = params.get<SensorBoardParameter::ComfortCoolingSetpoint>();
        g_state.hvac.standbyCoolingSetpointC = params.get<SensorBoardParameter::StandbyCoolingSetpoint>();
        g_state.hvac.economyCoolingSetpointC = params.get<SensorBoardParameter::EconomyCoolingSetpoint>();
        g_state.hvac.protectionCoolingSetpointC = params.get<SensorBoardParameter::ProtectionCoolingSetpoint>();
        g_state.hvac.minSetpointC = params.get<SensorBoardParameter::MinSetpoint>();
        g_state.hvac.maxSetpointC = params.get<SensorBoardParameter::MaxSetpoint>();
        g_state.hvac.maxSetpointShiftK = params.get<SensorBoardParameter::MaxSetpointShift>();
        g_state.hvac.heatingEnabled = params.get<SensorBoardParameter::HeatingEnabled>() != 0;
        g_state.hvac.coolingEnabled = params.get<SensorBoardParameter::CoolingEnabled>() != 0;
        g_state.hvac.heatCoolChangeoverMode =
            static_cast<sensor_board::hvac::HeatCoolChangeoverMode>(params.get<SensorBoardParameter::HeatCoolChangeoverMode>());
        g_state.hvac.heatCoolChangeoverPolarityInverted =
            params.get<SensorBoardParameter::HeatCoolChangeoverPolarity>() != 0;
        g_state.hvac.heatingControlAlgorithm =
            static_cast<sensor_board::hvac::ControlAlgorithm>(params.get<SensorBoardParameter::HeatingControlAlgorithm>());
        g_state.hvac.coolingControlAlgorithm =
            static_cast<sensor_board::hvac::ControlAlgorithm>(params.get<SensorBoardParameter::CoolingControlAlgorithm>());
        g_state.hvac.heatingMinOutputPercent =
            static_cast<uint8_t>(params.get<SensorBoardParameter::HeatingMinimumOutputPercent>());
        g_state.hvac.coolingMinOutputPercent =
            static_cast<uint8_t>(params.get<SensorBoardParameter::CoolingMinimumOutputPercent>());
        g_state.hvac.heatingMaxOutputPercent =
            static_cast<uint8_t>(params.get<SensorBoardParameter::HeatingMaximumOutputPercent>());
        g_state.hvac.coolingMaxOutputPercent =
            static_cast<uint8_t>(params.get<SensorBoardParameter::CoolingMaximumOutputPercent>());
        g_state.hvac.binaryDemandStrategy =
            static_cast<sensor_board::hvac::BinaryDemandStrategy>(params.get<SensorBoardParameter::BinaryDemandStrategy>());
        g_state.hvac.binaryDemandThresholdPercent =
            static_cast<uint8_t>(params.get<SensorBoardParameter::BinaryDemandThresholdPercent>());
        g_state.hvac.minimumHeatCoolChangeoverSeconds =
            static_cast<float>(params.get<SensorBoardParameter::MinimumHeatCoolChangeoverSeconds>());
        g_state.hvac.windowOpenBehavior =
            static_cast<sensor_board::hvac::WindowOpenBehavior>(params.get<SensorBoardParameter::WindowOpenBehavior>());
        g_state.hvac.presenceBehavior =
            static_cast<sensor_board::hvac::PresenceBehavior>(params.get<SensorBoardParameter::PresenceBehavior>());
        g_state.hvac.sensorFaultBehavior =
            static_cast<sensor_board::hvac::SensorFaultBehavior>(params.get<SensorBoardParameter::SensorFaultBehavior>());
        g_state.hvac.temperatureChangeThresholdC =
            static_cast<float>(params.get<SensorBoardParameter::TemperatureChangeThresholdCenti>()) / 100.0f;
        g_state.hvac.cyclicStatusIntervalSeconds = params.get<SensorBoardParameter::CyclicStatusIntervalSeconds>();

        // Seed the runtime mode/state inputs from their ETS defaults.
        g_state.controllerEnable = g_state.hvac.controllerDefaultEnable;
        g_state.hvacOperatingMode = g_state.hvac.defaultHvacOperatingMode;
        g_state.controllerMode = g_state.hvac.defaultControllerMode;
    }

    KNX_LOGI(TAG, "ETS-commissionable TP1 sensor bridge started");

    DeviceLifecycleState previousLifecycle = app.lifecycleState();

    // Room-control loops (task-owned; configured each tick from ETS parameters).
    sensor_board::hvac::ThermostatController thermostat;
    sensor_board::hvac::VentilationController ventilation;
    constexpr TickType_t kControlIntervalTicks = pdMS_TO_TICKS(1000);
    TickType_t lastControlTick = xTaskGetTickCount();

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

    for (;;) {
        app.loop();

        const TickType_t nowTick = xTaskGetTickCount();
        if ((nowTick - lastControlTick) >= kControlIntervalTicks) {
            const float dtSeconds =
                static_cast<float>(nowTick - lastControlTick) * portTICK_PERIOD_MS / 1000.0f;
            lastControlTick = nowTick;
            runHvacControlTick(app, thermostat, ventilation, dtSeconds);
        }

        // Startup-delay gate: while closed we must NOT publish or consume the
        // pending-publish flags (that would clear initialPublishPending before
        // the burst is allowed out). Sensor updates keep accumulating in
        // g_state and go out together once the gate opens.
        const bool startupGateOpen =
            static_cast<int32_t>(xTaskGetTickCount() - startupPublishReleaseTick) >= 0;

        if (app.lifecycleState() == DeviceLifecycleState::Operational && !startupGateOpen) {
            if (previousLifecycle != DeviceLifecycleState::Operational) {
                LockGuard lock(g_state.mutex);
                g_state.initialPublishPending = true;
            }
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
            if (previousLifecycle != DeviceLifecycleState::Operational) {
                LockGuard lock(g_state.mutex);
                g_state.initialPublishPending = true;
            }

            const PublishSnapshot snapshot = takePublishSnapshot();
            if (snapshot.toggleProgrammingMode) {
                app.toggleProgrammingMode();
            }

            const bool hasPendingPublish = snapshot.publishAll
                || snapshot.pendingSensorMask != 0u
                || snapshot.publishThermostatSetpoint
                || snapshot.publishVentilationSetpoint
                || snapshot.publishThermostatRequest
                || snapshot.publishVentilationBoost
                || snapshot.publishCoolingRequest
                || snapshot.publishHeatingControl
                || snapshot.publishCoolingControl
                || snapshot.publishFloorLimit
                || snapshot.publishControllerEnable
                || snapshot.publishHvacOperatingMode
                || snapshot.publishControllerMode
                || snapshot.publishHeatCoolState
                || snapshot.publishActiveSetpoint
                || snapshot.publishSetpointShiftFeedback
                || snapshot.publishControllerFault
                || snapshot.publishSensorFaultStatus
                || snapshot.publishHeatingBlocked
                || snapshot.publishCoolingBlocked
                || snapshot.publishVentilationDemand
                || snapshot.publishVentilationLevel
                || snapshot.publishVentilationMode
                || snapshot.publishVentilationActive
                || snapshot.publishIaqStatus
                || snapshot.publishHvacControllerStatus;

            if (hasPendingPublish) {
                publishPendingState(app, snapshot);
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
    g_state.pendingSensorMask |= data->updated_mask;
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