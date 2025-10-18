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
//#include "knx_interface.h"
#include "knx_tp_bit_bang.h"

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

// KNX helper functions moved to knx_helpers.c
#include "knx_helpers.h"

void app_main(void)
{
    uint8_t requested_slave_address = slave_address;
    init_pins();
    mb_rtu_slave_init(&mb_rtu_slave, slave_address); // Initialize Modbus RTU slave with address 0x01

    sensor_service_init(&sensor_service, update_sensor_data);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_PROG_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };    gpio_config(&io_conf);    
    // Install GPIO ISR service at low priority so GPTimer ISR can preempt it
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(PIN_PROG_BTN, (gpio_isr_t)button_isr_handler, &sensor_service);

    // Define KNX pin configurations 
    // Initialize KNX TP bit-bang with compile-time optimized pin configuration
    knx_tp_bit_bang_t knx_bit_bang;
    knx_tp_bit_bang_init(&knx_bit_bang);

    // Configure KNX addressing and AUTO_ACK
    knx_tp_bit_bang_set_device_address(&knx_bit_bang, 0x1101);  // Set device address to 1.1.1
    knx_tp_bit_bang_add_group_address(&knx_bit_bang, 0x0801);   // Listen to group 1/0/1
    knx_tp_bit_bang_add_group_address(&knx_bit_bang, knx_tp_bit_bang_triplet_to_address(0, 0, 225));   // Listen to group 0/0/225
    knx_bit_bang.flags |= KNX_TP_BIT_BANG_FLAG_AUTO_ACK;        // Enable AUTO_ACK
    knx_bit_bang.flags |= KNX_TP_BIT_BANG_FLAG_PROMISCUOUS;      // Enable PROMISCUOUS mode

    mb_rtu_slave.holding_reg_params.knx_test_data = 0b0000000010110111;
    mb_rtu_slave.holding_reg_params.knx_lenght = 1;

    unsigned char sample_data[] = {
        0x9C, 0xFF, 0xFA, 0x11, 0x00, 0xE1, 0x00, 0x80, 0x16
        //0xaa, 0x55, 0xcc, 0x33
    };
    uint32_t sample_data_len = sizeof(sample_data) / sizeof(sample_data[0]);


    uint32_t last_counter = 0;

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
            //toggle_server(&sensor_service);
            ESP_LOGI(TAG, "Sending KNX data...");
            memcpy(knx_bit_bang.tx_buffer, sample_data, sample_data_len);
            knx_bit_bang.tx_telegram_length = sample_data_len;
            knx_bit_bang.tx_state = KNX_TP_BIT_BANG_TX_STATE_IDLE; // Reset TX status
            knx_bit_bang.rx_state = KNX_TP_BIT_BANG_RX_STATE_IDLE; // Reset RX status

            knx_tp_bit_bang_tx_enable(&knx_bit_bang);
            ESP_LOGI(TAG, "KNX bit-bang tx_state %d rx_state %d error flags: %d", 
                     knx_bit_bang.tx_state, knx_bit_bang.rx_state, (int)knx_bit_bang.errors);
        }
        if (last_counter != knx_bit_bang.rx_timer_count + knx_bit_bang.tx_timer_count) {
            last_counter = knx_bit_bang.rx_timer_count + knx_bit_bang.tx_timer_count;
            print_data(&knx_bit_bang);
        }

        knx_ring_buffer_entry_t telegram;
        while (knx_tp_bit_bang_pop_telegram(&knx_bit_bang, &telegram)) {
            ESP_LOGI(TAG, "=== KNX Telegram Received ===");
            ESP_LOGI(TAG, "  Length: %u bytes, Errors: 0x%02x, Timestamp: %llu us",
                     telegram.length, telegram.errors, telegram.timestamp);
            
            // Print telegram as binary string
            static char rx_bin_line[512];
            buffer_to_binary_string(telegram.data, telegram.length, rx_bin_line, sizeof(rx_bin_line));
            ESP_LOGI(TAG, "  Binary: %s", rx_bin_line);
            
            // Parse and display telegram details
            log_knx_tp1_telegram(telegram.data, telegram.length);
        }
        
        // Check for dropped telegrams
        uint32_t dropped = knx_tp_bit_bang_get_dropped_count(&knx_bit_bang);
        if (dropped > 0) {
            ESP_LOGW(TAG, "Ring buffer overflow! %lu telegrams dropped", dropped);
            knx_ring_buffer_reset_dropped_count(&knx_bit_bang.rx_ring_buffer);
        }
    vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);

    }
}