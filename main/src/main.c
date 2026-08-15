// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/*
 * Habinari room HVAC sensor/controller — application entry point.
 *
 * Three things start here, in this order and for this reason:
 *
 *   control_service   owns the room-control model, the settings, NVS and the
 *                     1 Hz control tick. The device *is* this; everything else
 *                     is a way of reaching it.
 *   protocol adapters whichever field-bus personalities this image was built
 *                     with — KNX TP1, Modbus RTU, MQTT over Wi-Fi. They are
 *                     alternatives, chosen in menuconfig; see
 *                     protocol_registry.c for why a KNX image cannot also
 *                     carry a radio.
 *   oob_service       the out-of-band service channel, if this image has one:
 *                     BLE GATT, for the settings that decide whether a field
 *                     bus works at all and therefore cannot be written over it.
 *                     Not a personality — see oob_service.h — and absent from
 *                     every KNX image, where ETS already does this job.
 *   sensor_service    samples the sensor package and runs the fusion layer.
 *
 * The order is the dependency order and nothing more: adapters need
 * control_state_get() to answer, the service channel needs the adapters to have
 * registered their settings, and the sensor service needs somewhere to deliver
 * to. An image with no adapter at all still boots, samples and controls — it
 * just has nobody to tell.
 *
 * What is left in this file is the board's own user interface: the programming
 * button and the LED. Neither knows which protocols are compiled in.
 */
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "board.h"
#include "control_service.h"
#include "control_state.h"
#include "oob_service.h"
#include "protocol_adapter.h"
#include "sensor_service.h"

static const char *TAG = "app";

static sensor_service_t s_sensor_service;
static volatile bool s_identify_active = false;

// Called from the sensor task once per sampling cycle. One call, one owner: the
// control service stores the record and fans it out to whichever adapters asked
// to see it. Adding a protocol does not add a line here.
static void on_sensor_data(const sensor_data_t *data)
{
    // The provenance fields are on the line because without them a change of
    // temperature source reads as an unexplained step: how many sources are live
    // and which of them are disagreeing is what says whether that step was a
    // sensor dying or the sensors merely arguing.
    ESP_LOGI(TAG,
             "Room T=%.2f C (src%u of %u%s%s) RH=%.1f %% CO2=%.0f ppm P=%.0f Pa IAQ=%.0f(acc%u) | "
             "Probe T=%.2f C RH=%.1f %% | dT=%+.2f K/min health=0x%02x suspect=0x%02x%s%s",
             data->temperature.value, data->temperature.source,
             (unsigned)data->temperature.healthy_count,
             data->temperature.fallback ? ",fallback" : "",
             data->temperature.disagreement ? ",disputed" : "", data->humidity.value,
             data->co2.value, data->pressure.value, data->iaq.value,
             (unsigned)data->air_quality_accuracy, data->probe_temperature.value,
             data->probe_humidity.value, data->trends.temperature.per_minute,
             data->health.healthy_mask, data->health.suspect_mask,
             data->events.occupancy_detected ? " occupied" : "",
             data->events.fire_alarm ? " FIRE" : "");

    control_service_update_sensor_data(data);
}

// The one place the board's programming mode fans out. control_service owns the
// state; this routes it to the LED and to the service channel, which advertises
// exactly while it is set. On an image with no channel the second call is an
// inline no-op, so this file still does not know which one it has.
static void on_identify_changed(bool active)
{
    s_identify_active = active;
    oob_service_set_programming_mode(active);
}

// --- Programming button ----------------------------------------------------
// Active low. One press does two different things depending on how long it is
// held, and each action fires once per press rather than repeating while held.
//
// Both are protocol-neutral, and the first one is the only commissioning gate
// this board has. Programming mode means "this device is selected", and every
// protocol reads it the same way: KNX enters ETS programming mode, the BLE
// service channel advertises, and an unaddressed Modbus slave comes up on the
// commissioning address. One gesture, one LED, one meaning — a device the
// installer has touched is the device that can be configured.
//
// It lapses by itself after CONFIG_HABINARI_PROGRAMMING_MODE_TIMEOUT_S, so
// walking away is not the same as leaving the door open.
#define BUTTON_POLL_PERIOD_MS 50
#define BUTTON_LONG_PRESS_MS 1000  // enter/leave programming mode
#define BUTTON_RESET_PRESS_MS 5000 // erase persisted state and reboot

typedef enum {
    BUTTON_ACTION_NONE = 0,
    BUTTON_ACTION_IDENTIFY,
    BUTTON_ACTION_FACTORY_RESET,
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
        if (handled < BUTTON_ACTION_IDENTIFY && held >= pdMS_TO_TICKS(BUTTON_LONG_PRESS_MS)) {
            ESP_LOGI(TAG, "Programming button long-press: toggling programming mode");
            control_service_request_identify_toggle();
            handled = BUTTON_ACTION_IDENTIFY;
        } else if (handled < BUTTON_ACTION_FACTORY_RESET
                   && held >= pdMS_TO_TICKS(BUTTON_RESET_PRESS_MS)) {
            ESP_LOGW(TAG, "Programming button held %d ms: factory reset", BUTTON_RESET_PRESS_MS);
            control_service_factory_reset();  // does not return: reboots
            handled = BUTTON_ACTION_FACTORY_RESET;
        }
    }
    was_pressed = pressed;
}

// Programming mode owns the LED while it is active — that is what an installer
// looks for, and what KNX requires. Outside it the LED follows whatever the
// adapters want (the Modbus LED coil, today).
//
// One state, one indication: because programming mode is now also what gates the
// service channel and Modbus addressing, a lit LED means exactly "this is the
// device that can be commissioned right now".
static void update_led(void)
{
    const bool on = s_identify_active || protocol_adapters_identify_active();
    gpio_set_level(PIN_LED, on ? 1 : 0);
}

static void log_enabled_protocols(void)
{
    size_t count = 0;
    const protocol_adapter_t *const *adapters = protocol_adapters(&count);
    if (count == 0) {
        ESP_LOGW(TAG, "No field-bus personality in this image");
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        ESP_LOGI(TAG, "Protocol %u/%u: %s%s", (unsigned)(i + 1), (unsigned)count,
                 adapters[i]->name, adapters[i]->required ? " (required)" : "");
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_pins());

    // First: the device itself. This initialises NVS, which several adapters
    // read their configuration from, and starts the control tick.
    control_service_set_identify_callback(on_identify_changed);
    ESP_ERROR_CHECK(control_service_start());

    log_enabled_protocols();
    // An optional adapter that fails to start is logged and skipped inside;
    // only a personality marked required can stop the boot.
    ESP_ERROR_CHECK(protocol_adapters_start_all());

    // After the adapters, because they are the ones that registered the
    // settings this channel exists to carry — and an adapter that failed to
    // start registered them anyway, which is the case that matters most.
    // Not ESP_ERROR_CHECK: losing the service hatch is a reason to complain,
    // never a reason to stop controlling the room.
    const esp_err_t oob_err = oob_service_start();
    if (oob_err != ESP_OK) {
        ESP_LOGE(TAG, "Out-of-band service channel unavailable: %s", esp_err_to_name(oob_err));
    }

    ESP_ERROR_CHECK(sensor_service_init(&s_sensor_service, on_sensor_data));
    ESP_ERROR_CHECK(sensor_service_start(&s_sensor_service));

    for (;;) {
        handle_programming_button();
        update_led();
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_PERIOD_MS));
    }
}
