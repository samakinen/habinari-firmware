#include <stdio.h>
#include "scd4x.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_INFO
#endif

static const char *TAG = "SCD4X";

static inline float scd4x_raw_to_temperature(uint16_t raw)
{
    return 175.0f * (float)raw / 65535.0f - 45.0f;
}

static inline float scd4x_raw_to_humidity(uint16_t raw)
{
    return 100.0f * (float)raw / 65535.0f;
}

static inline uint16_t scd4x_temperature_to_raw(float temperature)
{
    return (temperature + 45.0f) * 65535.0f / 175.0f;
}

static inline uint16_t scd4x_humidity_to_raw(float humidity)
{
    return humidity * 65535.0f / 100.0f;
}

uint8_t scd4x_calculate_crc(uint8_t *data, uint8_t len)
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


static inline esp_err_t scd4x_send_cmd(i2c_master_dev_handle_t dev_handle, scd4x_command_t cmd) {
    esp_err_t ret;
    uint8_t cmd_buf[2] = {SCD4X_CMD_MSB(cmd), SCD4X_CMD_LSB(cmd)};
    ret = i2c_master_transmit(dev_handle, cmd_buf, 2, -1);
    return ret;
}

static esp_err_t scd4x_query(i2c_master_dev_handle_t dev, scd4x_command_t cmd, int cmd_duration_ms, uint8_t *buffer, size_t len)
{
    esp_err_t ret;
    ret = scd4x_send_cmd(dev, cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(cmd_duration_ms));
    ret = i2c_master_receive(dev, buffer, len, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read response: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}


i2c_master_dev_handle_t scd4x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
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

esp_err_t scd4x_device_delete(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}

esp_err_t scd4x_reinit(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    ret = scd4x_send_cmd(dev_handle, SCD4X_REINIT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send reinit command: %s", esp_err_to_name(ret));
    }
    return ret;
} 

esp_err_t scd4x_start_periodic_measurement(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    ret = scd4x_send_cmd(dev_handle, SCD4X_START_PERIODIC_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start periodic measurement command: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t scd4x_start_low_power_periodic_measurement(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    ret = scd4x_send_cmd(dev_handle, SCD4X_START_LOW_POWER_PERIODIC_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send start low power periodic measurement command: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t scd4x_stop_periodic_measurement(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    ret = scd4x_send_cmd(dev_handle, SCD4X_STOP_PERIODIC_MEASUREMENT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send stop periodic measurement command: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t scd4x_set_ambient_pressure(i2c_master_dev_handle_t dev_handle, uint16_t pressure_hpa)
{
    const scd4x_command_t cmd = SCD4X_SET_AMBIENT_PRESSURE;
    uint8_t data[] = {
        SCD4X_CMD_MSB(cmd),
        SCD4X_CMD_LSB(cmd),
        pressure_hpa >> 8,
        pressure_hpa & 0xff,
        scd4x_calculate_crc(data+2, 2)
    };
    
    esp_err_t ret = i2c_master_transmit(dev_handle, data, sizeof(data), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ambient pressure: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t scd4x_set_automatic_self_calibration_enabled(i2c_master_dev_handle_t dev_handle, bool enabled)
{
    const scd4x_command_t cmd = SCD4X_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED;
    uint8_t data[] = {
        SCD4X_CMD_MSB(cmd),
        SCD4X_CMD_LSB(cmd),
        enabled ? 1 : 0,
        0
    };
    data[3] = scd4x_calculate_crc(data+2, 1);
    
    esp_err_t ret = i2c_master_transmit(dev_handle, data, sizeof(data), -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set automatic self calibration: %s", esp_err_to_name(ret));
    }
    return ret;
}

bool scd4x_get_automatic_calibration_enabled(i2c_master_dev_handle_t dev_handle)
{
    const size_t len = 3;
    uint8_t data[len];
    esp_err_t ret = scd4x_query(dev_handle, SCD4X_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED, 1, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to query automatic self calibration: %s", esp_err_to_name(ret));
        return false;
    }
    uint8_t crc = scd4x_calculate_crc(data, 2);
    if (crc != data[2]) {
        ESP_LOGE(TAG, "CRC check failed for automatic self calibration 0x%02x 0x%02x", data[2], crc);
        return false;
    }
    ESP_LOGI(TAG, "Automatic self calibration enabled: %02x %02x", data[0], data[1]);
    return data[1] & 0x01;
}

bool scd4x_data_ready_status(i2c_master_dev_handle_t dev_handle)
{
    const size_t len = 3;
    uint8_t status[len];
    esp_err_t ret = scd4x_query(dev_handle, SCD4X_GET_DATA_READY_STATUS, 1, status, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to query data ready status: %s", esp_err_to_name(ret));
        return false;
    }
    uint8_t crc = scd4x_calculate_crc(status, 2);
    if (crc != status[2]) {
        ESP_LOGE(TAG, "CRC check failed for data ready status 0x%02x 0x%02x", status[2], crc);
        return false;
    }
    return (status[0] & 0x0F) || (status[1] & 0xFF);
}

esp_err_t scd4x_read_measurement(i2c_master_dev_handle_t dev_handle, uint16_t *co2_ppm, float *temperature, float *humidity)
{
    const size_t len = 9; // number of bytes to read
    esp_err_t ret;
    uint8_t buffer[len];
    
    ret = scd4x_query(dev_handle, SCD4X_READ_MEASUREMENT, 1, buffer, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement: %s", esp_err_to_name(ret));
        return ret;
    }
    
    uint16_t raw_co2 = (buffer[0] << 8) | buffer[1];
    uint16_t raw_temperature = (buffer[3] << 8) | buffer[4];
    uint16_t raw_humidity = (buffer[6] << 8) | buffer[7];
    //ESP_LOGI(TAG, "raw: CO2: 0x%04x (0x%02x) T: 0x%04x (0x%02x) H: 0x%04x (0x%02x))", raw_co2, crc_co2, raw_temperature, crc_temperature, raw_humidity, crc_humidity);
    if (buffer[2] != scd4x_calculate_crc(buffer, 2)) {
        ESP_LOGE(TAG, "CRC check failed for CO2");
        return ESP_ERR_INVALID_CRC;
    }
    if (buffer[5] != scd4x_calculate_crc(buffer + 3, 2)) {
        ESP_LOGE(TAG, "CRC check failed for temperature");
        return ESP_ERR_INVALID_CRC;
    }
    if (buffer[8] != scd4x_calculate_crc(buffer + 6, 2)) {
        ESP_LOGE(TAG, "CRC check failed for humidity");
        return ESP_ERR_INVALID_CRC;
    }
    *co2_ppm = raw_co2;
    *temperature = scd4x_raw_to_temperature(raw_temperature);
    *humidity = scd4x_raw_to_humidity(raw_humidity);
    
    return ESP_OK;
}

esp_err_t scd4x_get_serial_number(i2c_master_dev_handle_t dev_handle, uint16_t *serial_0, uint16_t *serial_1, uint16_t *serial_2)
{
    const size_t len = 18;
    uint8_t data[len];
    esp_err_t ret = scd4x_query(dev_handle, SCD4X_GET_SERIAL_NUMBER, 1, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read serial number: %s", esp_err_to_name(ret));
        return ret;
    }
    uint8_t crc = scd4x_calculate_crc(data, 2);
    if (crc != data[2]) {
        ESP_LOGE(TAG, "CRC check failed for serial 0");
        return ESP_ERR_INVALID_CRC;
    }
    crc = scd4x_calculate_crc(data + 3, 2);
    if (crc != data[5]) {
        ESP_LOGE(TAG, "CRC check failed for serial 1");
        return ESP_ERR_INVALID_CRC;
    }
    crc = scd4x_calculate_crc(data + 6, 2);
    if (crc != data[8]) {
        ESP_LOGE(TAG, "CRC check failed for serial 2");
        return ESP_ERR_INVALID_CRC;
    }
    *serial_0 = (data[0] << 8) | data[1];
    *serial_1 = (data[3] << 8) | data[4];
    *serial_2 = (data[6] << 8) | data[7];
    return ESP_OK;
}
