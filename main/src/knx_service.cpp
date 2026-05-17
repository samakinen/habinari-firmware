#include "knx_service.h"

#include "board.h"
#include "knx_product.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
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
    float thermostatSetpointC{kDefaultThermostatSetpointC};
    float ventilationSetpointPpm{kDefaultVentilationSetpointPpm};
    float thermostatHysteresisC{kDefaultThermostatHysteresisC};
    float ventilationHysteresisPpm{kDefaultVentilationHysteresisPpm};
    bool thermostatRequest{false};
    bool ventilationBoostRequest{false};
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
    float thermostatSetpointC{kDefaultThermostatSetpointC};
    float ventilationSetpointPpm{kDefaultVentilationSetpointPpm};
    bool thermostatRequest{false};
    bool ventilationBoostRequest{false};
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

void recalculateRequestsLocked(SharedState &state)
{
    const bool previousThermostatRequest = state.thermostatRequest;
    const bool previousVentilationBoost = state.ventilationBoostRequest;

    if ((state.availableMask & SENSOR_TEMPERATURE) != 0u) {
        const float measuredTemperature = state.latestSensorData.temperature;
        if (measuredTemperature <= (state.thermostatSetpointC - state.thermostatHysteresisC)) {
            state.thermostatRequest = true;
        } else if (measuredTemperature >= (state.thermostatSetpointC + state.thermostatHysteresisC)) {
            state.thermostatRequest = false;
        }
    } else {
        state.thermostatRequest = false;
    }

    if ((state.availableMask & SENSOR_CO2) != 0u) {
        const float measuredCo2 = static_cast<float>(state.latestSensorData.co2);
        if (measuredCo2 >= (state.ventilationSetpointPpm + state.ventilationHysteresisPpm)) {
            state.ventilationBoostRequest = true;
        } else if (measuredCo2 <= (state.ventilationSetpointPpm - state.ventilationHysteresisPpm)) {
            state.ventilationBoostRequest = false;
        }
    } else {
        state.ventilationBoostRequest = false;
    }

    if (state.thermostatRequest != previousThermostatRequest) {
        state.pendingThermostatRequestPublish = true;
    }
    if (state.ventilationBoostRequest != previousVentilationBoost) {
        state.pendingVentilationBoostPublish = true;
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
    snapshot.thermostatSetpointC = g_state.thermostatSetpointC;
    snapshot.ventilationSetpointPpm = g_state.ventilationSetpointPpm;
    snapshot.thermostatRequest = g_state.thermostatRequest;
    snapshot.ventilationBoostRequest = g_state.ventilationBoostRequest;
    snapshot.toggleProgrammingMode = g_state.toggleProgrammingModeRequested;

    g_state.pendingSensorMask = 0;
    g_state.initialPublishPending = false;
    g_state.pendingThermostatSetpointPublish = false;
    g_state.pendingVentilationSetpointPublish = false;
    g_state.pendingThermostatRequestPublish = false;
    g_state.pendingVentilationBoostPublish = false;
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
}

void knxServiceTask(void *arg)
{
    (void)arg;

    platform::Esp32Platform platform;

    auto physicalResult = createTp1Physical(platform);
    if (physicalResult.isError()) {
        KNX_LOGE(TAG, "TP1 physical init failed: %d", static_cast<int>(physicalResult.error()));
        vTaskDelete(nullptr);
        return;
    }

    auto bindings = makeCommissionedBindings(kSensorBoardProduct)
        .onStateWrite<SensorBoardPort::ThermostatSetpoint>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.thermostatSetpointC = value;
            g_state.pendingThermostatSetpointPublish = true;
            recalculateRequestsLocked(g_state);
            ESP_LOGI(TAG, "Thermostat setpoint updated to %.2f C", value);
        })
        .provideState<SensorBoardPort::ThermostatSetpoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.thermostatSetpointC;
        })
        .onStateWrite<SensorBoardPort::VentilationSetpoint>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.ventilationSetpointPpm = value;
            g_state.pendingVentilationSetpointPublish = true;
            recalculateRequestsLocked(g_state);
            ESP_LOGI(TAG, "Ventilation setpoint updated to %.0f ppm", value);
        })
        .provideState<SensorBoardPort::VentilationSetpoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.ventilationSetpointPpm;
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
        .onParameterChanged<SensorBoardParameter::DefaultThermostatSetpoint>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.thermostatSetpointC = value;
            g_state.pendingThermostatSetpointPublish = true;
            recalculateRequestsLocked(g_state);
        })
        .onParameterChanged<SensorBoardParameter::DefaultVentilationSetpoint>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.ventilationSetpointPpm = value;
            g_state.pendingVentilationSetpointPublish = true;
            recalculateRequestsLocked(g_state);
        })
        .onParameterChanged<SensorBoardParameter::ThermostatHysteresis>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.thermostatHysteresisC = value;
            recalculateRequestsLocked(g_state);
        })
        .onParameterChanged<SensorBoardParameter::VentilationHysteresis>([](float value) {
            LockGuard lock(g_state.mutex);
            g_state.ventilationHysteresisPpm = value;
            recalculateRequestsLocked(g_state);
        })
        .onProgrammingModeChanged([](bool enabled) {
            KNX_LOGI(TAG, "Programming mode: %s", enabled ? "ON" : "OFF");
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

    auto app = std::move(appResult.value());

    {
        LockGuard lock(g_state.mutex);
        g_state.taskHandle = xTaskGetCurrentTaskHandle();
        g_state.thermostatSetpointC = app.parameters().get<SensorBoardParameter::DefaultThermostatSetpoint>();
        g_state.ventilationSetpointPpm = app.parameters().get<SensorBoardParameter::DefaultVentilationSetpoint>();
        g_state.thermostatHysteresisC = app.parameters().get<SensorBoardParameter::ThermostatHysteresis>();
        g_state.ventilationHysteresisPpm = app.parameters().get<SensorBoardParameter::VentilationHysteresis>();
        recalculateRequestsLocked(g_state);
    }

    KNX_LOGI(TAG, "ETS-commissionable TP1 sensor bridge started");

    DeviceLifecycleState previousLifecycle = app.lifecycleState();

    for (;;) {
        app.loop();

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
                || snapshot.publishVentilationBoost;

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

    BaseType_t created = xTaskCreate(knxServiceTask, "knx_service", 8192, nullptr, 8, nullptr);
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
    recalculateRequestsLocked(g_state);
}

extern "C" void knx_service_toggle_programming_mode(void)
{
    if (g_state.mutex == nullptr) {
        return;
    }

    LockGuard lock(g_state.mutex);
    g_state.toggleProgrammingModeRequested = true;
}