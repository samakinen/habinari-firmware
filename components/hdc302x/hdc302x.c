#include <stdio.h>
#include "hdc302x.h"
#include "esp_log.h"

static const char *TAG = "HDC302X";

i2c_master_dev_handle_t hdc302x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = dev_speed,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    return dev_handle;
}

esp_err_t hdc302x_device_delete(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}

esp_err_t hdc302x_soft_reset(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    ret = hdc302x_send_cmd(dev_handle, HDC302X_SOFT_RESET);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send soft reset command");
    }
    return ret;
} 

esp_err_t hdc302x_start_measurement(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    ret = hdc302x_send_cmd(dev_handle, HDC302X_TRIGGER_ONDEMAND_LP_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start measurement command");
    }
    return ret;
}

uint8_t hdc302x_calculate_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xff;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

esp_err_t hdc302x_read_measurement(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity)
{
    const size_t len = 6; // number of bytes to read
    esp_err_t ret;
    uint8_t buffer[len];

    ret = i2c_master_receive(dev_handle, buffer, len, -1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temperature and humidity");
        return ret;
    }

    // Convert raw values to temperature and humidity
    uint16_t raw_temp = (buffer[0] << 8) | buffer[1];
    uint8_t crc_temp = buffer[2];
    uint16_t raw_hum = (buffer[3] << 8) | buffer[4];
    uint8_t crc_hum = buffer[5];
    uint8_t crc_temp_calc = hdc302x_calculate_crc(buffer, 2);
    uint8_t crc_hum_calc = hdc302x_calculate_crc(buffer + 3, 2);
    //ESP_LOGI(TAG, "raw: T: 0x%04x (0x%02x) H: 0x%04x (0x%02x))", raw_temp, crc_temp, raw_hum, crc_hum);
    if (crc_temp_calc != crc_temp) {
        ESP_LOGE(TAG, "CRC check failed for temperature 0x%02x 0x%02x", crc_temp, crc_temp_calc);
        return ESP_ERR_INVALID_CRC;
    }
    if (crc_hum_calc != crc_hum) {
        ESP_LOGE(TAG, "CRC check failed for humidity 0x%02x 0x%02x", crc_hum, crc_hum_calc);
        return ESP_ERR_INVALID_CRC;
    }
    
    *temperature = 175.0f * (float)raw_temp / 65535.0f - 45.0f;
    *humidity = 100.0f * (float)raw_hum / 65535.0f;

    return ESP_OK;
}

