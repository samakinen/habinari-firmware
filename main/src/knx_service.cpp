#include "knx_service.h"

#include "board.h"
#include "hvac_control.hpp"
#include "knx_product.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "knx/platform/esp32_platform.hpp"
#include "knx/physical/bitbang_driver_timer_isr_espidf.hpp"
#include "knx/physical/physical_factory.hpp"
#include "knx/product/commissioned_product.hpp"
#include "knx/util/log.hpp"

#include <cmath>
#include <cstdint>
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
static constexpr uint32_t kKnxServiceTaskStackSize = 24576;

// ETS-configurable room-control tuning (see hvac_control.hpp). Snapshotted by
// the control tick each second, so parameter changes apply within one tick.
struct HvacSettings {
    float thermostatSetpointC{kDefaultThermostatSetpointC};
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
    HvacSettings hvac{};
    // Controller outputs (written by the control tick, read by provideState
    // and the publish path).
    bool thermostatRequest{false};        // heating demand
    bool coolingRequest{false};
    bool ventilationBoostRequest{false};
    bool floorLimitActive{false};
    uint8_t heatingControlPercent{0};
    uint8_t coolingControlPercent{0};
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
    float thermostatSetpointC{kDefaultThermostatSetpointC};
    float ventilationSetpointPpm{kDefaultVentilationSetpointPpm};
    bool thermostatRequest{false};
    bool coolingRequest{false};
    bool ventilationBoostRequest{false};
    bool floorLimitActive{false};
    uint8_t heatingControlPercent{0};
    uint8_t coolingControlPercent{0};
    bool toggleProgrammingMode{false};
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

// Room-control tick: snapshot inputs + ETS tuning under the mutex, run the
// (task-owned, lock-free) controllers, store the outputs back and flag the
// changed ones for publishing. Called from the KNX task at ~1 Hz.
void runHvacControlTick(sensor_board::hvac::ThermostatController &thermostat,
                        sensor_board::hvac::AirQualityBoostController &airQuality,
                        float dtSeconds)
{
    namespace hvac = sensor_board::hvac;

    HvacSettings settings;
    hvac::ThermostatInputs thermostatIn;
    hvac::AirQualityInputs airIn;
    {
        LockGuard lock(g_state.mutex);
        settings = g_state.hvac;
        thermostatIn.setpointC = settings.thermostatSetpointC;
        thermostatIn.roomTemperatureC = g_state.latestSensorData.temperature;
        thermostatIn.roomTemperatureValid = (g_state.availableMask & SENSOR_TEMPERATURE) != 0u;
        thermostatIn.floorTemperatureC = g_state.latestSensorData.ext_probe_temperature;
        thermostatIn.floorTemperatureValid = (g_state.availableMask & SENSOR_EXT_PROBE_TEMPERATURE) != 0u;
        airIn.co2Ppm = static_cast<float>(g_state.latestSensorData.co2);
        airIn.co2Valid = (g_state.availableMask & SENSOR_CO2) != 0u;
        airIn.humidityPct = g_state.latestSensorData.humidity;
        airIn.humidityValid = (g_state.availableMask & SENSOR_HUMIDITY) != 0u;
    }

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
    thermostatCfg.coolingDeadbandK = settings.coolingDeadbandK;
    thermostatCfg.requestHysteresisK = settings.thermostatHysteresisC;
    thermostatCfg.maxFloorTemperatureC = settings.maxFloorTemperatureC;
    thermostatCfg.floorHysteresisK = settings.floorHysteresisK;
    thermostat.configure(thermostatCfg);

    airQuality.configure({.co2SetpointPpm = settings.ventilationSetpointPpm,
                          .co2HysteresisPpm = settings.ventilationHysteresisPpm,
                          .humidityThresholdPct = settings.humidityBoostPct,
                          .humidityHysteresisPct = settings.humidityBoostHysteresisPct});

    const auto out = thermostat.update(thermostatIn, dtSeconds);
    const bool boost = airQuality.update(airIn);

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
        if (boost != g_state.ventilationBoostRequest) {
            g_state.ventilationBoostRequest = boost;
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
    snapshot.thermostatSetpointC = g_state.hvac.thermostatSetpointC;
    snapshot.ventilationSetpointPpm = g_state.hvac.ventilationSetpointPpm;
    snapshot.thermostatRequest = g_state.thermostatRequest;
    snapshot.coolingRequest = g_state.coolingRequest;
    snapshot.ventilationBoostRequest = g_state.ventilationBoostRequest;
    snapshot.floorLimitActive = g_state.floorLimitActive;
    snapshot.heatingControlPercent = g_state.heatingControlPercent;
    snapshot.coolingControlPercent = g_state.coolingControlPercent;
    snapshot.toggleProgrammingMode = g_state.toggleProgrammingModeRequested;

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
    if ((publishMask & SENSOR_GAS_RESISTANCE) != 0u) {
        publishPort(app, SensorBoardPort::GasResistance, snapshot.sensorData.gas_resistance, "gas resistance");
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
        .onStateWrite<SensorBoardPort::ThermostatSetpoint>([](float value) {
            LockGuard lock(g_state.mutex);
            if (std::fabs(g_state.hvac.thermostatSetpointC - value) < 0.01f) {
                return;
            }
            g_state.hvac.thermostatSetpointC = value;
            g_state.pendingThermostatSetpointPublish = true;
            ESP_LOGI(TAG, "Thermostat setpoint updated to %.2f C", value);
        })
        .provideState<SensorBoardPort::ThermostatSetpoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.thermostatSetpointC;
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
        .provideState<SensorBoardPort::GasResistance>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.gas_resistance;
        })
        .provideState<SensorBoardPort::Co2>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<float>(g_state.latestSensorData.co2);
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
        // Fractional ETS parameters arrive as Dpt9Float (KNX 2-byte half-float,
        // implicitly convertible to float); ppm/second parameters as uint16_t.
        .onParameterChanged<SensorBoardParameter::DefaultThermostatSetpoint>([](Dpt9Float value) {
            const float setpointC = value;
            LockGuard lock(g_state.mutex);
            if (std::fabs(g_state.hvac.thermostatSetpointC - setpointC) < 0.01f) {
                return;
            }
            g_state.hvac.thermostatSetpointC = setpointC;
            g_state.pendingThermostatSetpointPublish = true;
        })
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
        // stable identity instead of all zeroes.
        uint8_t mac[6] = {};
        if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
            app.deviceObject().setSerialNumber(std::span<const uint8_t>(mac, sizeof(mac)));
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
        g_state.taskHandle = xTaskGetCurrentTaskHandle();
        // Seed engineering values from the ETS parameters: Dpt9Float ones
        // convert implicitly to float; ppm/second ones are plain uint16.
        auto params = app.parameters();
        g_state.hvac.thermostatSetpointC = params.get<SensorBoardParameter::DefaultThermostatSetpoint>();
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
    }

    KNX_LOGI(TAG, "ETS-commissionable TP1 sensor bridge started");

    DeviceLifecycleState previousLifecycle = app.lifecycleState();

    // Room-control loops (task-owned; configured each tick from ETS parameters).
    sensor_board::hvac::ThermostatController thermostat;
    sensor_board::hvac::AirQualityBoostController airQuality;
    constexpr TickType_t kControlIntervalTicks = pdMS_TO_TICKS(1000);
    TickType_t lastControlTick = xTaskGetTickCount();

    for (;;) {
        app.loop();

        const TickType_t nowTick = xTaskGetTickCount();
        if ((nowTick - lastControlTick) >= kControlIntervalTicks) {
            const float dtSeconds =
                static_cast<float>(nowTick - lastControlTick) * portTICK_PERIOD_MS / 1000.0f;
            lastControlTick = nowTick;
            runHvacControlTick(thermostat, airQuality, dtSeconds);
        }

        if (app.lifecycleState() == DeviceLifecycleState::Operational) {
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
                || snapshot.publishFloorLimit;

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
        platform.delay(5);
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

    LockGuard lock(g_state.mutex);
    g_state.toggleProgrammingModeRequested = true;
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