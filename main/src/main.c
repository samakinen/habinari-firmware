/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "sensor_service.h"
#include <string.h>

static const char *TAG = "example";

int count = 0;
sensor_service_t sensor_service;
bool request_toggle = false;

void update_sensor_data(const sensor_data_t *data)
{
    ESP_LOGI(TAG, "Air (T=%.2f °C RH=%.2f %% P=%.2f Pa R=%.2f CO\u2082=%d ppm) Ground (T=%.2f °C RH=%.2f %%), Updated: %s%s%s%s%s%s%s",
             data->temperature, data->humidity, data->pressure, data->gas_resistance, data->co2, data->ext_probe_temperature, data->ext_probe_humidity,
             (data->updated_mask & SENSOR_TEMPERATURE) ? "T" : "",
             (data->updated_mask & SENSOR_HUMIDITY) ? "H" : "",
             (data->updated_mask & SENSOR_PRESSURE) ? "P" : "",
             (data->updated_mask & SENSOR_GAS_RESISTANCE) ? "R" : "",
             (data->updated_mask & SENSOR_CO2) ? "C" : "",
             (data->updated_mask & SENSOR_EXT_PROBE_TEMPERATURE) ? "t" : "",
             (data->updated_mask & SENSOR_EXT_PROBE_HUMIDITY) ? "h" : "");
}

void IRAM_ATTR button_isr_handler(void *arg)
{
    request_toggle = true;
}

void toggle_server(sensor_service_t *service)
{
    if (service->task_handle != NULL)
    {
        sensor_service_stop(service);
        service->task_handle = NULL;
    }
    else
    {
        sensor_service_start(service);
    }
}

#define BLINK_PERIOD 5000
#define BLINK_DURATION 50
void app_main(void)
{
    init_pins();
    sensor_service_init(&sensor_service, update_sensor_data);
    // sensor_service_start(&sensor_service);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_PROG_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_PROG_BTN, (gpio_isr_t)button_isr_handler, &sensor_service);

    while (1) {
        vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);
        if (request_toggle) {
            request_toggle = false;
            toggle_server(&sensor_service);
        }
        gpio_set_level(PIN_LED, 1);
        vTaskDelay(BLINK_DURATION / portTICK_PERIOD_MS);
        gpio_set_level(PIN_LED, 0);
    }
}