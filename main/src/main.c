/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY Kind, either express or implied.
*/
#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "sensor_service.h"
#include <string.h>
#include "mb_rtu_slave.h"
#include "knx_service.h"

static const char *TAG = "example";

int count = 0;
sensor_service_t sensor_service;
bool request_toggle = false;

mb_rtu_slave_t mb_rtu_slave = {0};
uint8_t slave_address = 0x01; // Modbus slave address

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
    mb_rtu_slave_set_sensor_data(&mb_rtu_slave, data); // Update Modbus registers with new sensor data
    knx_service_update_sensor_data(data);
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

#define BLINK_PERIOD 2000
#define BLINK_DURATION 50
bool led_status = false;

void app_main(void)
{
    uint8_t requested_slave_address = slave_address;
    init_pins();
    mb_rtu_slave_init(&mb_rtu_slave, slave_address); // Initialize Modbus RTU slave with address 0x01

    sensor_service_init(&sensor_service, update_sensor_data);
    sensor_service_start(&sensor_service);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_PROG_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    // Install GPIO ISR service before KNX initialization if KNstaX is not doing it.
    // This must happen before knx_service_start() when CONFIG_KNX_TP1_BITBANG_INSTALL_ISR_SERVICE is disabled.
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(PIN_PROG_BTN, (gpio_isr_t)button_isr_handler, &sensor_service);

    knx_service_start();

    while (1) {
        led_status = mb_rtu_slave.coil_reg_params.led_on;
        requested_slave_address = mb_rtu_slave.holding_reg_params.slave_address;
        if (requested_slave_address != slave_address) {
            sensor_service_stop(&sensor_service); // Stop sensor service if running
            mb_rtu_slave_deinit(&mb_rtu_slave); // Deinitialize Modbus RTU slave
            mb_rtu_slave_init(&mb_rtu_slave, requested_slave_address); // Reinitialize with new address
            sensor_service_init(&sensor_service, update_sensor_data); // Reinitialize sensor service
            ESP_LOGI(TAG, "Slave address changed to %d", requested_slave_address);
        }
        if (request_toggle) {
            ESP_LOGI(TAG, "Toggle server requested");
            request_toggle = false;
            knx_service_toggle_programming_mode();
        }
        // Performance monitoring moved to interface layer
        // Basic status check only

        // Drain any received bytes (demo printing raw stream)
        // Ring buffer monitoring moved to interface layer
        
        vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);

    }
}