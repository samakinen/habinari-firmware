#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"
#include "bme68x_esp.h"

#define SENSOR_BUS_SPEED 400000 // 400kHz
#define SENSOR_BUS_STARTUP_TIME 100 // 100ms

typedef struct 
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t hdc302x_device_handle;
    i2c_master_dev_handle_t scd4x_device_handle;
    struct bme68x_dev bme68x_dev;
    uint32_t bme68x_wait_time;
} sensor_bus_t;

typedef sensor_bus_t *sensor_bus_handle_t;

#define SENSOR_HDC302X_TEMPERATURE    (1 << 2)
#define SENSOR_HDC302X_HUMIDITY       (1 << 3)
#define SENSOR_SCD4X_TEMPERATURE      (1 << 4)
#define SENSOR_SCD4X_HUMIDITY         (1 << 5)
#define SENSOR_BME68X_TEMPERATURE     (1 << 6)
#define SENSOR_BME68X_HUMIDITY        (1 << 7)
#define SENSOR_BME68X_PRESSURE        (1 << 8)
#define SENSOR_BME68X_GAS_RESISTANCE  (1 << 9)
#define SENSOR_SCD4X_CO2              (1 << 10)

typedef struct
{
    float hdc302x_temperature;
    float hdc302x_humidity;
    float scd4x_temperature;
    float scd4x_humidity;
    float bme68x_temperature;
    float bme68x_humidity;
    int bme68x_pressure;
    int bme68x_gas_resistance;
    uint16_t scd4x_co2;
    uint16_t updated_mask; // Values updated in last measurement
} sensor_bus_results_t;

esp_err_t sensor_bus_init(sensor_bus_handle_t sensor_bus_handle);
esp_err_t sensor_bus_enable(sensor_bus_handle_t sensor_bus_handle);
esp_err_t sensor_bus_disable(sensor_bus_handle_t sensor_bus_handle);
esp_err_t sensor_bus_read(sensor_bus_handle_t sensor_bus_handle, sensor_bus_results_t *results);
esp_err_t sensor_bus_deinit(sensor_bus_handle_t sensor_bus_handle);

#ifdef __cplusplus
}
#endif