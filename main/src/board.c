// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "board.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "board";

// PIN_RS485_TX / PIN_RS485_RX / PIN_RS485_TEN are deliberately absent here.
// mb_rtu_slave_init() routes all three to the UART peripheral via
// uart_set_pin(), with TEN as RTS for RS485 half-duplex direction control, and
// that routing overrides whatever gpio_config() set. Claiming them here was at
// best redundant and at worst misleading: PIN_RS485_TX was being configured as
// an input, and driving TEN by hand would fight the driver's automatic DE
// control. The UART driver owns those pins; leave them to it.
esp_err_t init_pins(void)
{
    //set initial level of the pins
    gpio_set_level(PIN_PROBE_EN, 1);
    gpio_set_level(PIN_2V8_EN, 0);
    gpio_set_level(PIN_LED, 0);

    gpio_config_t io_conf;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << PIN_2V8_EN) |
                           (1ULL << PIN_LED) |
                           (1ULL << PIN_PROBE_EN);
    //enable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //configure GPIO with the given settings
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configuring pins: %s", esp_err_to_name(ret));
        return ret;
    }

    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << PIN_KNX_OK) |
                           (1ULL << PIN_SDA) |
                           (1ULL << PIN_SCL) |
                           (1ULL << PIN_PROG_BTN) |
                           //(1ULL << PIN_USB_DP) |
                           //(1ULL << PIN_USB_DM) |
                           (1ULL << PIN_PROBE_SDA) |
                           (1ULL << PIN_PROBE_SCL);
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error configuring pins: %s", esp_err_to_name(ret));
        return ret;
    }
    return ret;
}


esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus_handle, uint8_t sda_io, uint8_t scl_io, bool lp_i2c)
{
    esp_err_t ret;
    i2c_master_bus_config_t i2c_bus_config = {
        // Some targets (e.g., ESP32) don't support LP I2C; fall back gracefully
        #ifdef SOC_LP_I2C_SUPPORTED
        .i2c_port = lp_i2c ? LP_I2C_NUM_0 : I2C_NUM_0,
        #else
        .i2c_port = I2C_NUM_0,
        #endif
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        #ifdef SOC_LP_I2C_SUPPORTED
        .clk_source = lp_i2c ? LP_I2C_SCLK_DEFAULT : I2C_CLK_SRC_DEFAULT,
        #else
        .clk_source = I2C_CLK_SRC_DEFAULT,
        #endif
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    ret = i2c_new_master_bus(&i2c_bus_config, bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C master bus created");
    return ESP_OK;
}