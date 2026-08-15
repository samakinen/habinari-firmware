// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "ext_probe.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "ext_probe";

#define PROBE_BUS_SPEED 100000
#define PROBE_DEVICE_WAIT_TIME_MS 200
#define PROBE_MEASUREMENT_DELAY_MS 150

// Address order matters: the probe is auto-detected, and the highest address is
// tried first because that is what the current probe hardware straps.
static const uint8_t kProbeAddresses[] = {SHT4X_I2C_ADDR_2, SHT4X_I2C_ADDR_1, SHT4X_I2C_ADDR_0};

static ext_probe_status_t ext_probe_reg_disable()
{
    // set probe enable pin as input to set it to high impedance
    // do not drive low to avoid discharing the capacitors
    gpio_set_direction(PIN_PROBE_EN, GPIO_MODE_INPUT);
    return EXT_PROBE_POWER_OFF;
}

static ext_probe_status_t ext_probe_reg_enable()
{
    // set probe enable pin as output to enable output
    gpio_set_direction(PIN_PROBE_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PROBE_EN, 1);
    // wait for the probe power supply to stabilize
    vTaskDelay(pdMS_TO_TICKS(PROBE_STABILIZE_TIME));
    // check if the probe is shorted (both SDA and SCL are low)
    if (gpio_get_level(PIN_PROBE_SDA) + gpio_get_level(PIN_PROBE_SCL) == 0)
    {
        // if the probe is shorted, disable the probe power supply
        ext_probe_reg_disable();
        ESP_LOGE(TAG, "External probe short-circuit detected, disabling power supply");
        return EXT_PROBE_SHORT_CIRCUIT;
    }
    return EXT_PROBE_OPERATIONAL;
}

esp_err_t ext_probe_deinit(ext_probe_handle_t probe_handle)
{
    esp_err_t ret;
    if (probe_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (probe_handle->sht4x_device_handle != NULL)
    {
        ret = sht4x_device_delete(probe_handle->sht4x_device_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to delete SHT4x device: %s", esp_err_to_name(ret));
        }
        probe_handle->sht4x_device_handle = NULL;
    }
    if (probe_handle->bus_handle != NULL)
    {
        ret = i2c_del_master_bus(probe_handle->bus_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to delete I2C master bus: %s", esp_err_to_name(ret));
        }
        probe_handle->bus_handle = NULL;
    }
    // reg_disable reports an ext_probe_status_t, not an esp_err_t; comparing it
    // against ESP_OK (which this used to do through an esp_err_t variable) was
    // a type confusion that happened to look like it worked.
    const ext_probe_status_t power_status = ext_probe_reg_disable();
    if (power_status != EXT_PROBE_POWER_OFF)
    {
        ESP_LOGE(TAG, "Failed to disable probe power supply");
    }
    probe_handle->status = EXT_PROBE_NOT_INITIALIZED;
    return ESP_OK;
}

esp_err_t ext_probe_init(ext_probe_handle_t probe_handle)
{
    ext_probe_status_t status;
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing external probe...");
    if (probe_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    // Initialize the probe handle
    if (probe_handle->sht4x_device_handle != NULL || probe_handle->bus_handle != NULL)
    {
        ESP_LOGE(TAG, "Probe already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    probe_handle->status = EXT_PROBE_NOT_INITIALIZED;
    // Power on the regulator
    status = ext_probe_reg_enable();
    if (status != EXT_PROBE_OPERATIONAL)
    {
        probe_handle->status = status;
        return ESP_FAIL;
    }
    // Initialize the I2C bus
    ret = i2c_bus_init(&probe_handle->bus_handle, PIN_PROBE_SDA, PIN_PROBE_SCL, false);
    if (probe_handle->bus_handle == NULL)
    {
        probe_handle->status = EXT_PROBE_INITIALIZATION_FAILED;
        return ret;
    }
    // Initialize the SHT4x device
    for (size_t i = 0; i < sizeof(kProbeAddresses) / sizeof(kProbeAddresses[0]); i++)
    {
        const uint8_t address = kProbeAddresses[i];
        ret = i2c_master_probe(probe_handle->bus_handle, address, PROBE_DEVICE_WAIT_TIME_MS);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Probe found at address 0x%02x", address);
            probe_handle->sht4x_device_handle =
                sht4x_device_create(probe_handle->bus_handle, address, PROBE_BUS_SPEED);
            break;
        }
        ESP_LOGD(TAG, "Probe not found at address 0x%02x", address);
    }
    if (probe_handle->sht4x_device_handle == NULL)
    {
        probe_handle->status = EXT_PROBE_NOT_CONNECTED;
        i2c_del_master_bus(probe_handle->bus_handle);
        probe_handle->bus_handle = NULL;
        ext_probe_reg_disable();
        return ESP_OK;
    }
    probe_handle->status = EXT_PROBE_OPERATIONAL;
    ESP_LOGI(TAG, "Probe initialized successfully");
    return ESP_OK;
}

esp_err_t ext_probe_read(ext_probe_handle_t probe_handle, ext_probe_results_t *results)
{
    esp_err_t ret;
    if (probe_handle == NULL || results == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (probe_handle->status != EXT_PROBE_OPERATIONAL)
    {
        ESP_LOGE(TAG, "Probe not operational");
        return ESP_ERR_INVALID_STATE;
    }
    ret = sht4x_start_measurement(probe_handle->sht4x_device_handle,
                                  SHT4X_CMD_READ_MEASUREMENT_HIGH);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to start probe measurement: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(PROBE_MEASUREMENT_DELAY_MS));
    ret = sht4x_read_measurement(probe_handle->sht4x_device_handle, &results->temperature,
                                 &results->humidity);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to read probe measurement: %s", esp_err_to_name(ret));
    }
    return ret;
}
