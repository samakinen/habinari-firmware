#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

#define SCD4X_ADDR_0 0x62

typedef enum {
    SCD4X_START_PERIODIC_MEASUREMENT = 0x21b1,
    SCD4X_READ_MEASUREMENT = 0xec05,
    SCD4X_STOP_PERIODIC_MEASUREMENT = 0x3f86,
    SCD4X_SET_TEMPERATURE_OFFSET = 0x241d,
    SCD4X_GET_TEMPERATURE_OFFSET = 0x2318,
    SCD4X_SET_SENSOR_ALTITUDE = 0x2427,
    SCD4X_GET_SENSOR_ALTITUDE = 0x2322,
    SCD4X_SET_AMBIENT_PRESSURE = 0xe000,
    SCD4X_GET_AMBIENT_PRESSURE = 0xe000,
    SCD4X_PERFORM_FORCED_RECALIBRATION = 0x362f,
    SCD4X_SET_AUTOMATIC_SELF_CALIBRATION_ENABLED = 0x2416,
    SCD4X_GET_AUTOMATIC_SELF_CALIBRATION_ENABLED = 0x2313,
    SCD4X_START_LOW_POWER_PERIODIC_MEASUREMENT = 0x21ac,
    SCD4X_GET_DATA_READY_STATUS = 0xe4b8,
    SCD4X_PERSIST_SETTINGS = 0x3615,
    SCD4X_GET_SERIAL_NUMBER = 0x3682,
    SCD4X_PERFORM_SELF_TEST = 0x3639,
    SCD4X_PERFORM_FACTORY_RESET = 0x3632,
    SCD4X_REINIT = 0x3646,
    SCD4X_MEASURE_SINGLE_SHOT = 0x219d,
    SCD4X_MEASURE_SINGLE_SHOT_RHT_ONLY = 0x2196,
    SCD4X_POWER_DOWN = 0x36e0,
    SCD4X_WAKE_UP = 0x36f6
} scd4x_command_t;

#define SCD4X_CMD_MSB(cmd) ((cmd) >> 8)
#define SCD4X_CMD_LSB(cmd) ((cmd) & 0xFF)



/**
 * @brief Create a handle for the SCD4x device on the I2C bus.
 *
 * This function initializes and returns a handle for the SCD4x device
 * connected to the specified I2C bus. The device address and speed are
 * also specified as parameters.
 *
 * @param bus_handle Handle to the I2C master bus.
 * @param dev_addr I2C address of the SCD4x device.
 * @param dev_speed I2C communication speed for the device.
 * 
 * @return Handle to the SCD4x device, or NULL if creation fails.
 */
i2c_master_dev_handle_t scd4x_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);

/**
 * @brief Deletes the SCD4x device instance.
 *
 * This function deletes the SCD4x device instance associated with the given I2C master device handle.
 *
 * @param[in] dev_handle Handle to the I2C master device.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Deletion failed
 */
esp_err_t scd4x_device_delete(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Start periodic measurements on the SCD4x device.
 *
 * This function sends a command to the SCD4x device to start periodic measurements.
 * The device will then perform the measurements every 5 seconds and store the results internally.
 *
 * @param dev_handle Handle to the SCD4x device.
 * 
 * @return ESP_OK if the command was sent successfully, ESP_FAIL otherwise.
 */
esp_err_t scd4x_start_periodic_measurement(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Check if the SCD4x device has new measurement data available.
 *
 * This function queries the SCD4x device to check if new measurement data is available.
 *
 * @param dev_handle Handle to the SCD4x device.
 * 
 * @return true if new data is available, false otherwise.
 */
bool scd4x_data_ready_status(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Start low power periodic measurements on the SCD4x device.
 *
 * This function sends a command to the SCD4x device to start low power periodic measurements.
 * The device will then perform the measurements every 30 seconds and store the results internally.
 *
 * @param dev_handle Handle to the SCD4x device.
 * 
 * @return ESP_OK if the command was sent successfully, ESP_FAIL otherwise.
 */
esp_err_t scd4x_start_low_power_periodic_measurement(i2c_master_dev_handle_t dev_handle);


/**
 * @brief Stop the periodic measurement on the SCD4x device.
 *
 * This function sends a command to the SCD4x device to stop the periodic measurement.
 *
 * @param dev_handle Handle to the SCD4x device.
 * 
 * @return ESP_OK if the command was sent successfully, ESP_FAIL otherwise.
 */
esp_err_t scd4x_stop_periodic_measurement(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Reinitialize the SCD4x device.
 *
 * This function sends a command to the SCD4x device to reinitialize it.
 *
 * @param dev_handle Handle to the SCD4x device.
 * 
 * @return ESP_OK if the command was sent successfully, ESP_FAIL otherwise.
 */
esp_err_t scd4x_reinit(i2c_master_dev_handle_t dev_handle);


/**
 * @brief Read the measurement data from the SCD4x device.
 *
 * This function reads the measurement data from the SCD4x device and stores
 * the CO2, temperature and humidity values in the provided pointers.
 *
 * @param dev_handle Handle to the SCD4x device.
 * @param co2_ppm Pointer to the variable where the CO2 value will be stored.
 * @param temperature Pointer to the variable where the temperature value will be stored.
 * @param humidity Pointer to the variable where the humidity value will be stored.
 * 
 * @return ESP_OK if the data was read successfully, ESP_FAIL otherwise.
 */
esp_err_t scd4x_read_measurement(i2c_master_dev_handle_t dev_handle, uint16_t *co2_ppm, float *temperature, float *humidity);

/**
 * @brief Set the ambient pressure on the SCD4x device.
 *
 * This function sends a command to the SCD4x device to set the ambient pressure.
 *
 * @param dev_handle Handle to the SCD4x device.
 * @param pressure_hpa Ambient pressure in hPa.
 * 
 * @return ESP_OK if the command was sent successfully, ESP_FAIL otherwise.
 */
esp_err_t scd4x_set_ambient_pressure(i2c_master_dev_handle_t dev_handle, uint16_t pressure_hpa);

/**
 * @brief Perform a forced recalibration on the SCD4x device.
 *
 * This function sends a command to the SCD4x device to perform a forced recalibration.
 *
 * @param dev_handle Handle to the SCD4x device.
 * 
 * @return ESP_OK if the command was sent successfully, ESP_FAIL otherwise.
 */
esp_err_t scd4x_perform_forced_recalibration(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Set the automatic self calibration enabled status on the SCD4x device.
 *
 * This function sends a command to the SCD4x device to set the automatic self calibration enabled status.
 *
 * @param dev_handle Handle to the SCD4x device.
 * @param enabled true to enable automatic self calibration, false to disable.
 * 
 * @return ESP_OK if the command was sent successfully, ESP_FAIL otherwise.
 */
esp_err_t scd4x_set_automatic_self_calibration_enabled(i2c_master_dev_handle_t dev_handle, bool enabled);

/**
 * @brief Get the automatic self calibration enabled status from the SCD4x device.
 *
 * This function queries the SCD4x device to get the automatic self calibration enabled status.
 *
 * @param dev_handle Handle to the SCD4x device.
 * 
 * @return true if automatic self calibration is enabled, false otherwise.
 */
bool scd4x_get_automatic_calibration_enabled(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Retrieve the serial number of the SCD4x sensor.
 *
 * This function reads the serial number of the SCD4x sensor and stores it in
 * the provided pointers. The serial number is split into three 16-bit parts.
 *
 * @param[in] dev_handle Handle to the I2C master device.
 * @param[out] serial_0 Pointer to store the first 16 bits of the serial number.
 * @param[out] serial_1 Pointer to store the second 16 bits of the serial number.
 * @param[out] serial_2 Pointer to store the third 16 bits of the serial number.
 *
 * @return
 *     - ESP_OK: Success.
 *     - ESP_ERR_INVALID_ARG: Invalid arguments.
 *     - ESP_FAIL: Failed to communicate with the sensor.
 */
esp_err_t scd4x_get_serial_number(i2c_master_dev_handle_t dev_handle, uint16_t *serial_0, uint16_t *serial_1, uint16_t *serial_2);


#ifdef __cplusplus
}
#endif
