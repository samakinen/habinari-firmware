#include <stdio.h>
#include "bme68x_esp.h"
#include "bme68x.h"
#include "bme68x_defs.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_INFO
#endif

static const char *TAG = "BME68X";

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, reg_data, len, -1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C set reg failed: %d", ret);
        return BME68X_E_COM_FAIL;
    }
    return BME68X_OK;
}

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)intf_ptr;
    i2c_master_transmit_multi_buffer_info_t multi_buffer_info[] = {
        {
            .write_buffer = &reg_addr,
            .buffer_size = 1,
        },
        {
            .write_buffer = (uint8_t*)reg_data,
            .buffer_size = len,
        },
    };
    esp_err_t ret = i2c_master_multi_buffer_transmit(dev_handle, multi_buffer_info, 2, -1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
        return BME68X_E_COM_FAIL;
    }
    return BME68X_OK;
}

void bme68x_delay_us(uint32_t period_us, void *intf_ptr) {
    vTaskDelay(pdMS_TO_TICKS(period_us / 1000));
}

esp_err_t bme68x_device_create_i2c(struct bme68x_dev *dev, i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
{
    dev->intf = BME68X_I2C_INTF;
    dev->read = bme68x_i2c_read;
    dev->write = bme68x_i2c_write;
    dev->delay_us = bme68x_delay_us;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = dev_speed,
    };
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, (i2c_master_dev_handle_t*)&dev->intf_ptr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Creating BME68x I2C device failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t bme68x_device_delete_i2c(struct bme68x_dev *dev)
{
    esp_err_t ret = i2c_master_bus_rm_device((i2c_master_dev_handle_t)dev->intf_ptr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Deleting BME68x I2C device failed: %s", esp_err_to_name(ret));
        return ret;
    }
    dev->intf_ptr = NULL;
    return ESP_OK;
}