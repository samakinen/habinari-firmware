// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/i2c_master.h>

#define HDC203X_ADDR_0 0x44
#define HDC203X_ADDR_1 0x45
#define HDC203X_ADDR_2 0x46
#define HDC203X_ADDR_3 0x47

typedef enum {
    // Single shot mode for low power mode 0 to 3
    HDC302X_TRIGGER_ONDEMAND_LP_0 = 0x2400,
    HDC302X_TRIGGER_ONDEMAND_LP_1 = 0x240B,
    HDC302X_TRIGGER_ONDEMAND_LP_2 = 0x2416,
    HDC302X_TRIGGER_ONDEMAND_LP_3 = 0x24FF,

    // Auto measurement mode for 0.5Hz sampling rate
    HDC302X_AUTO_MEASUREMENT_MODE_0_5_HZ_LP_0 = 0x2032,
    HDC302X_AUTO_MEASUREMENT_MODE_0_5_HZ_LP_1 = 0x2024,
    HDC302X_AUTO_MEASUREMENT_MODE_0_5_HZ_LP_2 = 0x202F,
    HDC302X_AUTO_MEASUREMENT_MODE_0_5_HZ_LP_3 = 0x20FF,

    // Auto measurement mode for 1Hz sampling rate
    HDC302X_AUTO_MEASUREMENT_MODE_1_HZ_LP_0 = 0x2130,
    HDC302X_AUTO_MEASUREMENT_MODE_1_HZ_LP_1 = 0x2126,
    HDC302X_AUTO_MEASUREMENT_MODE_1_HZ_LP_2 = 0x212D,
    HDC302X_AUTO_MEASUREMENT_MODE_1_HZ_LP_3 = 0x21FF,

    // Auto measurement mode for 2Hz sampling rate
    HDC302X_AUTO_MEASUREMENT_MODE_2_HZ_LP_0 = 0x2236,
    HDC302X_AUTO_MEASUREMENT_MODE_2_HZ_LP_1 = 0x2220,
    HDC302X_AUTO_MEASUREMENT_MODE_2_HZ_LP_2 = 0x222B,
    HDC302X_AUTO_MEASUREMENT_MODE_2_HZ_LP_3 = 0x22FF,

    // Auto measurement mode for 4Hz sampling rate
    HDC302X_AUTO_MEASUREMENT_MODE_4_HZ_LP_0 = 0x2334,
    HDC302X_AUTO_MEASUREMENT_MODE_4_HZ_LP_1 = 0x2322,
    HDC302X_AUTO_MEASUREMENT_MODE_4_HZ_LP_2 = 0x2329,
    HDC302X_AUTO_MEASUREMENT_MODE_4_HZ_LP_3 = 0x23FF,

    // Auto measurement mode for 10Hz sampling rate
    HDC302X_AUTO_MEASUREMENT_MODE_10_HZ_LP_0 = 0x2737,
    HDC302X_AUTO_MEASUREMENT_MODE_10_HZ_LP_1 = 0x2721,
    HDC302X_AUTO_MEASUREMENT_MODE_10_HZ_LP_2 = 0x272A,
    HDC302X_AUTO_MEASUREMENT_MODE_10_HZ_LP_3 = 0x27FF,

    // Auto measurement mode exit, then return to Trigger-on Demand Mode
    HDC302X_AUTO_MEASUREMENT_MODE_EXIT = 0x3093,

    // Measurement readout of T and RH
    HDC302X_MEASUREMENT_READOUT_T_RH = 0xE000,
    HDC302X_MEASUREMENT_READOUT_RH = 0xE001,
    HDC302X_MEASUREMENT_HISTORY_READOUT_MIN_T = 0xE002,
    HDC302X_MEASUREMENT_HISTORY_READOUT_MAX_T = 0xE003,
    HDC302X_MEASUREMENT_HISTORY_READOUT_MIN_RH = 0xE004,
    HDC302X_MEASUREMENT_HISTORY_READOUT_MAX_RH = 0xE005,

    // Configure ALERT thresholds of T and RH
    HDC302X_CONFIGURE_ALERT_THRESHOLDS_T_RH = 0x6100,
    HDC302X_CONFIGURE_ALERT_THRESHOLDS_T_RH_SET_LOW = 0x611D,
    HDC302X_CONFIGURE_ALERT_THRESHOLDS_T_RH_SET_HIGH = 0x611D,
    HDC302X_CONFIGURE_ALERT_THRESHOLDS_T_RH_CLEAR_LOW = 0x610B,
    HDC302X_CONFIGURE_ALERT_THRESHOLDS_T_RH_CLEAR_HIGH = 0x6116,

    // Read ALERT thresholds of T and RH
    HDC302X_READ_ALERT_THRESHOLDS_T_RH = 0xE102,
    HDC302X_READ_ALERT_THRESHOLDS_T_RH_SET_LOW = 0xE11F,
    HDC302X_READ_ALERT_THRESHOLDS_T_RH_SET_HIGH = 0xE11F,
    HDC302X_READ_ALERT_THRESHOLDS_T_RH_CLEAR_LOW = 0xE109,
    HDC302X_READ_ALERT_THRESHOLDS_T_RH_CLEAR_HIGH = 0xE114,

    // Integrated heater
    HDC302X_INTEGRATED_HEATER_ENABLE = 0x306D,
    HDC302X_INTEGRATED_HEATER_DISABLE = 0x3066,
    HDC302X_INTEGRATED_HEATER_CONFIGURE = 0x306E,
    
    // Status register
    HDC302X_STATUS_REGISTER_READ_CONTENT = 0xF32D,
    HDC302X_STATUS_REGISTER_CLEAR_CONTENT = 0x3041,
    HDC302X_SOFT_RESET = 0x30A2,

    // Read NIST ID (Serial Number) Bytes 5 and 4
    HDC302X_READ_NIST_ID_5_4 = 0x3683,
    HDC302X_READ_NIST_ID_3_2 = 0x3684,
    HDC302X_READ_NIST_ID_1_0 = 0x3685,
    HDC302X_READ_MANUFACTURER_ID = 0x3781,
    HDC302X_PROGRAM_ALERT_THRESHOLDS = 0x611D,

    // EEPROM Programming
    HDC302X_NVM_ALERT_THRESHOLD = 0x6155,
    HDC302X_NVM_OFFSET_VALUE = 0xA004,
    HDC302X_NVM_DEFAULT_STATE = 0x61BB,


} HDC302x_command_t;

#define HDC302X_CMD_MSB(cmd) ((cmd) >> 8)
#define HDC302X_CMD_LSB(cmd) ((cmd) & 0xFF)

/**
 * @brief Send a command to the HDC302x device.
 * 
 * This function sends a command to the HDC302x device using the I2C bus.
 * 
 * @param dev_handle Handle to the HDC302x device.
 * @param cmd Command to send to the device.
 * 
 * @return
 *    - ESP_OK: Success
 *    - ESP_FAIL: Failed to send the command
 *    - ESP_ERR_INVALID_ARG: Invalid argument
 * 
 * @note This function is not intended to be called directly by the user.
 */
static inline esp_err_t hdc302x_send_cmd(i2c_master_dev_handle_t dev_handle, HDC302x_command_t cmd) {
    esp_err_t ret;
    uint8_t cmd_buf[2] = {HDC302X_CMD_MSB(cmd), HDC302X_CMD_LSB(cmd)};
    ret = i2c_master_transmit(dev_handle, cmd_buf, 2, -1);
    return ret;
}

/**
 * @brief Create a handle for the HDC302x device on the I2C bus.
 *
 * This function initializes and returns a handle for the HDC302x device
 * connected to the specified I2C bus. The device address and speed are
 * also specified as parameters.
 *
 * @param bus_handle Handle to the I2C master bus.
 * @param dev_addr I2C address of the HDC302x device.
 * @param dev_speed I2C communication speed for the device.
 * 
 * @return Handle to the HDC302x device, or NULL if creation fails.
 */
i2c_master_dev_handle_t hdc302x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);

/**
 * @brief Deletes the HDC302x device instance.
 *
 * This function deletes the HDC302x device instance associated with the given I2C master device handle.
 *
 * @param[in] dev_handle Handle to the I2C master device.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Deletion failed
 */
esp_err_t hdc302x_device_delete(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Start a measurement on the HDC302x device.
 *
 * This function sends a command to the HDC302x device to start a measurement.
 * The device will then perform the measurement and store the results internally.
 *
 * @param dev_handle Handle to the HDC302x device.
 * 
 * @return ESP_OK if the command was sent successfully, ESP_FAIL otherwise.
 */
esp_err_t hdc302x_start_measurement(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Calculate the CRC for the given data.
 *
 * This function computes the CRC for the provided data array using initial
 * value of 0xff anb polynomial 0x31.
 *
 * @param data Pointer to the data array for which the CRC needs to be calculated.
 * @param len Length of the data array.
 * @return The calculated CRC value.
 */
uint8_t hdc302x_calculate_crc(uint8_t *data, uint8_t len);

/**
 * @brief Read the measurement data from the HDC302x device.
 *
 * This function reads the measurement data from the HDC302x device and stores
 * the temperature and humidity values in the provided pointers.
 *
 * @param dev_handle Handle to the HDC302x device.
 * @param temperature Pointer to the variable where the temperature value will be stored.
 * @param humidity Pointer to the variable where the humidity value will be stored.
 * 
 * @return ESP_OK if the data was read successfully, ESP_FAIL otherwise.
 */
esp_err_t hdc302x_read_measurement(i2c_master_dev_handle_t dev_handle, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif
