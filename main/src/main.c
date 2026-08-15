/*
 * Room HVAC sensor/controller board — application entry point.
 *
 * The work is done by three services, each owning its own task:
 *
 *   sensor_service   samples the sensor package and runs the fusion layer
 *   knx_service      owns the room-control model and speaks KNX TP1
 *   mb_rtu_slave     exposes the same model on Modbus RTU
 *
 * What remains here is the board's own user interface: the programming button
 * and the LED. Everything else is a service start-up call.
 */
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "board.h"
#include "control_state.h"
#include "knx_service.h"
#include "mb_rtu_slave.h"
#include "sensor_service.h"

static const char *TAG = "app";

static sensor_service_t s_sensor_service;
static mb_rtu_slave_t s_modbus_slave;
static volatile bool s_programming_mode_enabled = false;

// Called from the sensor task once per sampling cycle. Both protocol adapters
// get the same record; neither can see a measurement the other cannot.
static void on_sensor_data(const sensor_data_t *data)
{
    ESP_LOGI(TAG,
             "Room T=%.2f C (src%u%s) RH=%.1f %% CO2=%.0f ppm P=%.0f Pa IAQ=%.0f(acc%u) | "
             "Probe T=%.2f C RH=%.1f %% | dT=%+.2f K/min health=0x%02x%s%s",
             data->temperature.value, data->temperature.source,
             data->temperature.fallback ? ",fallback" : "", data->humidity.value,
             data->co2.value, data->pressure.value, data->iaq.value,
             (unsigned)data->air_quality_accuracy, data->probe_temperature.value,
             data->probe_humidity.value, data->trends.temperature.per_minute,
             data->health.healthy_mask, data->events.occupancy_detected ? " occupied" : "",
             data->events.fire_alarm ? " FIRE" : "");

    mb_rtu_slave_publish(&s_modbus_slave, data);
    knx_service_update_sensor_data(data);
}

static void on_programming_mode_changed(bool enabled)
{
    s_programming_mode_enabled = enabled;
}

// --- Programming button ----------------------------------------------------
// Active low. One press does two different things depending on how long it is
// held, and each action fires once per press rather than repeating while held.
#define BUTTON_POLL_PERIOD_MS 50
#define BUTTON_LONG_PRESS_MS 1000  // toggle KNX programming mode
#define BUTTON_RESET_PRESS_MS 5000 // erase NVM and reboot

typedef enum {
    BUTTON_ACTION_NONE = 0,
    BUTTON_ACTION_PROGRAMMING_MODE,
    BUTTON_ACTION_NVM_RESET,
} button_action_t;

static void handle_programming_button(void)
{
    static bool was_pressed = false;
    static TickType_t pressed_at = 0;
    static button_action_t handled = BUTTON_ACTION_NONE;

    const bool pressed = (gpio_get_level(PIN_PROG_BTN) == 0);
    if (pressed && !was_pressed) {
        pressed_at = xTaskGetTickCount();
        handled = BUTTON_ACTION_NONE;
    } else if (!pressed) {
        handled = BUTTON_ACTION_NONE;
    } else {
        const TickType_t held = xTaskGetTickCount() - pressed_at;
        if (handled < BUTTON_ACTION_PROGRAMMING_MODE && held >= pdMS_TO_TICKS(BUTTON_LONG_PRESS_MS)) {
            ESP_LOGI(TAG, "Programming button long-press: toggling programming mode");
            knx_service_toggle_programming_mode();
            handled = BUTTON_ACTION_PROGRAMMING_MODE;
        } else if (handled < BUTTON_ACTION_NVM_RESET && held >= pdMS_TO_TICKS(BUTTON_RESET_PRESS_MS)) {
            ESP_LOGW(TAG, "Programming button held %d ms: resetting NVM", BUTTON_RESET_PRESS_MS);
            knx_service_reset_nvm();  // does not return: reboots
            handled = BUTTON_ACTION_NVM_RESET;
        }
    }
    was_pressed = pressed;
}

// KNX programming mode owns the LED while it is active — that is what an
// installer looks for. Outside it the LED follows the Modbus coil.
static void update_led(void)
{
    const bool on = s_programming_mode_enabled || mb_rtu_slave_led_requested(&s_modbus_slave);
    gpio_set_level(PIN_LED, on ? 1 : 0);
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_pins());

    // KNX first: it initialises NVS, which the Modbus slave then reads its
    // persisted address and baud rate from.
    knx_service_set_programming_mode_callback(on_programming_mode_changed);
    ESP_ERROR_CHECK(knx_service_start());

    esp_err_t err = mb_rtu_slave_start(&s_modbus_slave);
    if (err != ESP_OK) {
        // A Modbus failure must not take the KNX side down with it: the board's
        // primary field bus is KNX and it is perfectly usable without RS-485.
        ESP_LOGE(TAG, "Modbus RTU slave failed to start: %s", esp_err_to_name(err));
    }

    ESP_ERROR_CHECK(sensor_service_init(&s_sensor_service, on_sensor_data));
    ESP_ERROR_CHECK(sensor_service_start(&s_sensor_service));

    for (;;) {
        handle_programming_button();
        update_led();
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_PERIOD_MS));
    }
}
