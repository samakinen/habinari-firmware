/*
 * The control service: the device's one owner of state and time.
 *
 * What runs here used to run inside the KNX task, which is why a board without
 * KNX had no control loop at all. The tick now belongs to this task, the
 * settings belong to this file, and the field buses are clients.
 *
 * Cost of the split, since it is fair to ask: one extra task (3 kB stack, wakes
 * at 1 Hz) and one extra mutex acquisition per tick — the tick already took the
 * lock twice, to snapshot inputs and to store outputs, and it still does. The
 * control loops themselves run outside the lock exactly as before.
 */
#include "control_service.hpp"

#include "control_core.hpp"
#include "control_state.h"
#include "device_config.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "protocol_adapter.h"
#include "sensor_fusion_service.h"

namespace sensor_board {
namespace control {

namespace {

constexpr const char *TAG = "control";

// 1 Hz. The thermal processes this controls have time constants in minutes; the
// PID integrators are tuned in thousands of seconds. Faster buys nothing and
// costs a wake-up.
constexpr TickType_t kControlIntervalTicks = pdMS_TO_TICKS(1000);

// Measured peak for the tick is well under 2 kB — no protocol stack runs here,
// only the control loops, which are flat structs and float maths.
constexpr uint32_t kControlTaskStackSize = 4096;

// Below the KNX service task (8) on purpose: a late control tick shifts a valve
// position by a fraction of a percent, whereas a late KNX task misses a bus
// deadline. Above the Modbus and MQTT tasks, which are pure reporting.
constexpr UBaseType_t kControlTaskPriority = 6;

ServiceState g_state;
Core g_core;
bool g_started = false;
TaskHandle_t g_taskHandle = nullptr;

esp_err_t initNvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

// --- Installation name ------------------------------------------------------
//
// The one setting that belongs to the device rather than to a personality: what
// this board is called. It names the BLE advertisement, so an installer facing a
// row of identical enclosures can pick the right one, and it is the natural
// display name for any front end that wants one.
//
// Registered here because control_service owns NVS and owns "the device"; a
// name is not a property of whichever bus happens to be compiled in.

constexpr const char *kDeviceNvsNamespace = "device";
constexpr const char *kDeviceNvsKeyName = "name";

esp_err_t deviceNameGet(const device_config_item_t *, device_config_value_t *out)
{
    out->str[0] = '\0';

    nvs_handle_t handle;
    if (nvs_open(kDeviceNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return ESP_OK;
    }
    size_t length = sizeof(out->str);
    if (nvs_get_str(handle, kDeviceNvsKeyName, out->str, &length) != ESP_OK) {
        out->str[0] = '\0';
    }
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t deviceNameSet(const device_config_item_t *, const device_config_value_t *value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(kDeviceNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, kDeviceNvsKeyName, value->str);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

const device_config_item_t kDeviceConfigItems[] = {
    {
        .key = "dev.name",
        .label = "Device name",
        .unit = nullptr,
        .type = DEVICE_CONFIG_TYPE_STRING,
        // The advertised name is read once at start-up, so a rename shows up at
        // the next boot. Saying so is better than leaving somebody to wonder
        // why their phone still lists the old one.
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .min = 0.0f,
        .max = 20.0f,
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = deviceNameGet,
        .set = deviceNameSet,
        .ctx = nullptr,
    },
};

// --- Programming mode -------------------------------------------------------
//
// Runs on the control task, once per tick. Two jobs, and both exist because
// programming mode is now the single gate on every commissioning path — the KNX
// individual address, BLE pairing, Modbus re-addressing.
//
// The toggle is only acted on here when no adapter has claimed ownership. KNX
// claims it: ETS can set programming mode over the bus and the stack is only
// safe to touch from the KNX task, so the service would be fighting it.
// Everything else leaves it unclaimed, and before this the request was raised by
// the button and then consumed by nobody at all — a Modbus-only board's LED
// never lit.
//
// The timeout is expressed as a synthetic button press rather than as a second
// path into the flag. That way the KNX build leaves programming mode through
// exactly the code ETS and the button already use, instead of through a third
// mechanism whose interaction with the stack would have to be reasoned about.
/// Wrap-safe "has @p deadline passed?". The signed difference is what survives
/// the tick counter wrapping, which it does every 49 days at 1 kHz — well
/// inside the uptime of a wall-mounted controller.
bool tickReached(TickType_t now, TickType_t deadline)
{
    return static_cast<int32_t>(now - deadline) >= 0;
}

void serviceProgrammingMode()
{
    const TickType_t now = xTaskGetTickCount();
    bool expired = false;

    {
        LockGuard lock(g_state.mutex);
        if (!g_state.identifyActive) {
            g_state.identifyDeadline = 0;
        } else if (g_state.identifyDeadline != 0 && tickReached(now, g_state.identifyDeadline)) {
            g_state.identifyDeadline = 0;
            g_state.toggleIdentifyRequested = true;
            expired = true;
        }
    }
    if (expired) {
        ESP_LOGI(TAG, "Programming mode timed out after %d s",
                 CONFIG_SENSOR_BOARD_PROGRAMMING_MODE_TIMEOUT_S);
    }

    if (protocol_adapters_own_programming_mode()) {
        return;  // the owner consumes the request on its own task
    }
    if (takeIdentifyToggleRequest()) {
        bool active = false;
        {
            LockGuard lock(g_state.mutex);
            active = g_state.identifyActive;
        }
        control_service_set_identify_active(!active);
    }
}

/// One control cycle: snapshot under the lock, compute outside it, store back.
///
/// The two side effects the old in-KNX version did inline — reconfiguring the
/// fusion layer and acknowledging alarms — happen here rather than in Core::tick
/// so that the core stays a pure function and can be driven from a host test.
void runControlTick(float dtSeconds)
{
    Settings settings;
    Inputs inputs;
    sensor_data_t sensorData{};
    sensor_fusion_config_t fusionConfig{};
    bool fusionConfigDirty = false;
    bool acknowledgeAlarms = false;

    {
        LockGuard lock(g_state.mutex);
        settings = g_state.hvac;
        inputs = g_state.in;
        sensorData = g_state.latestSensorData;
        fusionConfig = g_state.fusion;
        fusionConfigDirty = g_state.fusionConfigDirty;
        g_state.fusionConfigDirty = false;
        acknowledgeAlarms = g_state.acknowledgeAlarmsRequested;
        g_state.acknowledgeAlarmsRequested = false;
    }

    // Push configuration changes down to the acquisition-side fusion layer. Only
    // on change: reconfiguring rebuilds every channel's config, which is wasted
    // work at 1 Hz when nothing has moved. Outside the lock — it takes its own.
    if (fusionConfigDirty) {
        sensor_fusion_configure(&fusionConfig);
    }
    if (acknowledgeAlarms) {
        sensor_fusion_acknowledge_alarms();
    }

    const Outputs computed = g_core.tick(settings, inputs, sensorData, dtSeconds);

    uint32_t generation = 0;
    {
        LockGuard lock(g_state.mutex);
        g_state.out = computed;
        generation = ++g_state.generation;
    }

    // Outside the lock: adapters must not block, but they may take their own.
    protocol_adapters_notify_tick(generation);
}

void controlTask(void *)
{
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastTick = lastWake;

    ESP_LOGI(TAG, "Control task started (%u ms cycle)",
             static_cast<unsigned>(kControlIntervalTicks * portTICK_PERIOD_MS));

    for (;;) {
        vTaskDelayUntil(&lastWake, kControlIntervalTicks);

        // Measure the interval actually elapsed rather than assuming the
        // nominal one. A tick delayed by a long flash write must not integrate
        // as if it were on time, or the PID output steps when NVS is busy.
        const TickType_t now = xTaskGetTickCount();
        const float dtSeconds =
            static_cast<float>(now - lastTick) * portTICK_PERIOD_MS / 1000.0f;
        lastTick = now;

        serviceProgrammingMode();
        runControlTick(dtSeconds > 0.0f ? dtSeconds : 1.0f);
    }
}

}  // namespace

ServiceState &state()
{
    return g_state;
}

bool takeIdentifyToggleRequest()
{
    if (g_state.mutex == nullptr) {
        return false;
    }
    LockGuard lock(g_state.mutex);
    const bool requested = g_state.toggleIdentifyRequested;
    g_state.toggleIdentifyRequested = false;
    return requested;
}

}  // namespace control
}  // namespace sensor_board

// ---------------------------------------------------------------------------
// control_service.h — the C lifecycle API.
// ---------------------------------------------------------------------------

namespace ctrl = sensor_board::control;

extern "C" esp_err_t control_service_start(void)
{
    if (ctrl::g_started) {
        return ESP_OK;
    }

    auto &s = ctrl::state();
    if (s.mutex == nullptr) {
        s.mutex = xSemaphoreCreateMutex();
        if (s.mutex == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    // NVS belongs to the service, not to any adapter: the KNX commissioned
    // state, the Modbus address and the Wi-Fi credentials all live in it, and
    // whichever of them is compiled in, somebody has to have initialised it.
    esp_err_t err = ctrl::initNvs();
    if (err != ESP_OK) {
        ESP_LOGE(ctrl::TAG, "Failed to initialize NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Before the adapters start, so the device's own settings come first in the
    // registry and an out-of-band client sees them at a stable index.
    const esp_err_t cfg = device_config_register(
        ctrl::kDeviceConfigItems, sizeof(ctrl::kDeviceConfigItems) / sizeof(device_config_item_t));
    if (cfg != ESP_OK) {
        ESP_LOGW(ctrl::TAG, "Device settings not exposed out of band: %s", esp_err_to_name(cfg));
    }

    // Seed the fusion configuration so the first tick pushes a complete config
    // down rather than a zeroed struct.
    {
        ctrl::LockGuard lock(s.mutex);
        sensor_fusion_default_config(&s.fusion);
        s.fusionConfigDirty = true;
    }

    const BaseType_t created =
        xTaskCreate(ctrl::controlTask, "control", ctrl::kControlTaskStackSize, nullptr,
                    ctrl::kControlTaskPriority, &ctrl::g_taskHandle);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ctrl::g_started = true;
    return ESP_OK;
}

extern "C" void control_service_update_sensor_data(const sensor_data_t *data)
{
    if (data == nullptr) {
        return;
    }

    auto &s = ctrl::state();
    if (s.mutex != nullptr) {
        ctrl::LockGuard lock(s.mutex);
        s.latestSensorData = *data;
        s.hasSensorData = true;
    }

    // Outside the lock. Adapters that want to report promptly on new data rather
    // than wait for the next tick hook here; they must not block.
    protocol_adapters_notify_sensor_data(data);
}

extern "C" void control_service_request_identify_toggle(void)
{
    auto &s = ctrl::state();
    if (s.mutex == nullptr) {
        return;
    }
    ctrl::LockGuard lock(s.mutex);
    s.toggleIdentifyRequested = true;
}

extern "C" void control_service_set_identify_active(bool active)
{
    auto &s = ctrl::state();
    control_identify_callback_t callback = nullptr;
    bool changed = false;
    {
        ctrl::LockGuard lock(s.mutex);
        changed = (s.identifyActive != active);
        s.identifyActive = active;
        // Armed on every entry, including one ETS made over the bus, so there is
        // no route into programming mode that leaves it open indefinitely.
        // Re-entering restarts the clock, which is what somebody pressing the
        // button again means.
        if (active && CONFIG_SENSOR_BOARD_PROGRAMMING_MODE_TIMEOUT_S > 0) {
            s.identifyDeadline =
                xTaskGetTickCount()
                + pdMS_TO_TICKS(CONFIG_SENSOR_BOARD_PROGRAMMING_MODE_TIMEOUT_S * 1000);
            if (s.identifyDeadline == 0) {
                s.identifyDeadline = 1;  // 0 is the "no deadline" sentinel
            }
        } else {
            s.identifyDeadline = 0;
        }
        callback = s.identifyCallback;
    }
    // Called outside the lock: the callback drives board GPIO and has no reason
    // to be serialised against the control tick.
    if (changed && callback != nullptr) {
        callback(active);
    }
}

extern "C" void control_service_set_identify_callback(control_identify_callback_t callback)
{
    auto &s = ctrl::state();
    ctrl::LockGuard lock(s.mutex);
    s.identifyCallback = callback;
}

extern "C" uint32_t control_service_generation(void)
{
    auto &s = ctrl::state();
    if (s.mutex == nullptr) {
        return 0;
    }
    ctrl::LockGuard lock(s.mutex);
    return s.generation;
}

extern "C" void control_service_factory_reset(void)
{
    ESP_LOGW(ctrl::TAG, "Factory reset: erasing all persisted state and rebooting");
    const esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
        ESP_LOGE(ctrl::TAG, "NVS erase failed: %s", esp_err_to_name(err));
    }
    esp_restart();
}

// ---------------------------------------------------------------------------
// control_state.h — the narrow, protocol-neutral C view.
//
// Implemented here rather than in an adapter because this file owns the state.
// That is the whole structural fix: a Modbus master and an MQTT client get the
// full room-controller model without either one depending on KNX being built.
// ---------------------------------------------------------------------------

extern "C" void control_state_get(control_state_t *out)
{
    if (out == nullptr) {
        return;
    }
    *out = control_state_t{};

    auto &g = ctrl::state();
    if (g.mutex == nullptr) {
        return;
    }

    ctrl::LockGuard lock(g.mutex);
    const auto &o = g.out;
    const auto &s = g.hvac;
    const auto &i = g.in;

    out->sensors = g.latestSensorData;
    out->has_sensor_data = g.hasSensorData;

    out->room_dew_point_c = o.roomDewPointC;
    out->room_absolute_humidity_gm3 = o.roomAbsoluteHumidityGm3;
    out->floor_absolute_humidity_gm3 = o.floorAbsoluteHumidityGm3;
    out->sea_level_pressure_pa = o.seaLevelPressurePa;
    out->dew_point_margin_k = o.dewPointMarginK;

    out->comfort_setpoint_c = s.comfortHeatingSetpointC;
    out->active_setpoint_c = o.activeSetpointC;
    out->heating_setpoint_c = o.heatingSetpointC;
    out->cooling_setpoint_c = o.coolingSetpointC;
    out->setpoint_shift_k = o.setpointShiftFeedbackK;
    out->co2_setpoint_ppm = s.ventilationSetpointPpm;

    out->heating_percent = o.heatingControlPercent;
    out->cooling_percent = o.coolingControlPercent;
    out->ventilation_percent = o.ventilationDemandPercent;
    out->ventilation_level = static_cast<uint8_t>(o.ventilationLevel);
    out->heating_request = o.heatingRequest;
    out->cooling_request = o.coolingRequest;
    out->heat_cool_mode_heating = o.heatCoolModeHeating;
    out->enable_heat = o.enableHeat;
    out->enable_cool = o.enableCool;
    out->ventilation_boost_request = o.ventilationBoostRequest;
    out->dehumidify_request = o.dehumidifyRequest;
    out->controller_status = o.controllerStatus;
    out->air_quality_status = o.airQualityStatus;

    out->dew_point_alarm = o.dewPointAlarm;
    out->floor_moisture_alarm = o.floorMoistureAlarm;
    out->floor_limit_active = o.floorLimitActive;
    out->floor_comfort_active = o.floorComfortActive;
    out->free_cooling_available = o.freeCoolingAvailable;
    out->free_drying_available = o.freeDryingAvailable;
    out->device_fault = o.deviceFault;
    out->room_sensor_status = o.roomSensorStatus;
    out->floor_probe_status = o.floorProbeStatus;
    out->air_quality_sensor_status = o.airQualitySensorStatus;

    out->controller_on = i.controllerOnOff;
    out->hvac_operating_mode = static_cast<uint8_t>(o.activePreset);
    out->controller_mode = static_cast<uint8_t>(o.activeControllerMode);
    out->ventilation_mode = static_cast<uint8_t>(i.ventilationMode);
    out->window_open = i.windowOpen;
    out->presence = i.presence;
    out->programming_mode = g.identifyActive;
}

extern "C" esp_err_t control_state_write(control_command_t command, float value)
{
    auto &g = ctrl::state();
    if (g.mutex == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    namespace hvac_ns = sensor_board::hvac;
    const bool flag = value != 0.0f;
    ctrl::LockGuard lock(g.mutex);
    switch (command) {
    case CONTROL_CMD_CONTROLLER_ON_OFF:
        g.in.controllerOnOff = flag;
        return ESP_OK;
    case CONTROL_CMD_HVAC_MODE:
        g.in.hvacOperatingMode = static_cast<hvac_ns::OperatingPreset>(static_cast<uint8_t>(value));
        return ESP_OK;
    case CONTROL_CMD_CONTROLLER_MODE:
        // Compact ControllerMode code points, the same ones control_state_t
        // reports back. This used to decode KNX DPT 20.105 here, which made the
        // command asymmetric with the reading: an adapter that read
        // controller_mode == 2 (Cool) and wrote 2 back got Auto, because 2 is
        // not a 20.105 code point. The 20.105 mapping belongs at the KNX edge —
        // where it already is, in knx_service.cpp — and not in the neutral API.
        g.in.controllerMode = (static_cast<uint8_t>(value) <= 3)
                                  ? static_cast<hvac_ns::ControllerMode>(
                                        static_cast<uint8_t>(value))
                                  : hvac_ns::ControllerMode::Auto;
        return ESP_OK;
    case CONTROL_CMD_SETPOINT_BASE:
        // Same clamping as the KNX SetpointBase write: the configured min/max
        // are a property of the installation, not of the protocol that asked.
        g.hvac.comfortHeatingSetpointC =
            hvac_ns::clampf(value, g.hvac.minSetpointC, g.hvac.maxSetpointC);
        return ESP_OK;
    case CONTROL_CMD_SETPOINT_SHIFT:
        g.in.setpointShiftK = value;
        return ESP_OK;
    case CONTROL_CMD_WINDOW_STATUS:
        g.in.windowOpen = flag;
        g.in.windowStatusKnown = true;
        return ESP_OK;
    case CONTROL_CMD_PRESENCE:
        g.in.presence = flag;
        g.in.presenceKnown = true;
        return ESP_OK;
    case CONTROL_CMD_VENTILATION_MODE:
        g.in.ventilationMode = static_cast<hvac_ns::VentilationMode>(static_cast<uint8_t>(value));
        return ESP_OK;
    case CONTROL_CMD_CO2_SETPOINT:
        g.hvac.ventilationSetpointPpm = value;
        return ESP_OK;
    case CONTROL_CMD_ACKNOWLEDGE_ALARMS:
        g.acknowledgeAlarmsRequested = true;
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}
