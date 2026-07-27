#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/i2c_master.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
    // Serialises BME688 access between this reader and the BSEC task (which,
    // when compiled in, owns the BME688 gas-measurement cadence).
    SemaphoreHandle_t bme68x_mutex;
} sensor_bus_t;

typedef sensor_bus_t *sensor_bus_handle_t;

#define SENSOR_HDC302X_TEMPERATURE    (1 << 2)
#define SENSOR_HDC302X_HUMIDITY       (1 << 3)
#define SENSOR_SCD4X_TEMPERATURE      (1 << 4)
#define SENSOR_SCD4X_HUMIDITY         (1 << 5)
#define SENSOR_BME68X_TEMPERATURE     (1 << 6)
#define SENSOR_BME68X_HUMIDITY        (1 << 7)
#define SENSOR_BME68X_PRESSURE        (1 << 8)
#define SENSOR_SCD4X_CO2              (1 << 10)
// BME688 air-quality outputs (from BSEC; see components/bsec2). Only produced
// when the firmware is built with CONFIG_BME688_USE_BSEC.
#define SENSOR_BME68X_IAQ             (1 << 11)
#define SENSOR_BME68X_CO2_EQUIVALENT  (1 << 12)
#define SENSOR_BME68X_VOC_EQUIVALENT  (1 << 13)

typedef struct
{
    float hdc302x_temperature;
    float hdc302x_humidity;
    float scd4x_temperature;
    float scd4x_humidity;
    float bme68x_temperature;
    float bme68x_humidity;
    float bme68x_pressure;
    // BME688 air-quality (BSEC). iaq_accuracy (0..3) travels with iaq.
    float bme68x_iaq;
    uint8_t bme68x_iaq_accuracy;
    float bme68x_co2_equivalent;
    float bme68x_voc_equivalent;
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