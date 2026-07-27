#pragma once

#include <esp_err.h>
#include "sensor_bus.h"
#include "ext_probe.h"

#define SENSOR_TEMPERATURE           (1 << 0)
#define SENSOR_HUMIDITY              (1 << 1)
#define SENSOR_PRESSURE              (1 << 2)
#define SENSOR_CO2                   (1 << 4)
#define SENSOR_EXT_PROBE_TEMPERATURE (1 << 5)
#define SENSOR_EXT_PROBE_HUMIDITY    (1 << 6)
// BME688 air-quality (BSEC-derived; see components/bsec2). Only populated when
// built with CONFIG_BME688_USE_BSEC. air_quality_accuracy travels with iaq.
#define SENSOR_IAQ                   (1 << 7)
#define SENSOR_CO2_EQUIVALENT        (1 << 8)
#define SENSOR_VOC_EQUIVALENT        (1 << 9)

typedef struct {
    float temperature; // Internal temperature sensor in C
    float humidity; // Internal humidity sensor in %RH
    float pressure; // Internal pressure sensor in Pa
    int co2; // Internal CO2 sensor in ppm
    float ext_probe_temperature; // External probe temperature in C
    float ext_probe_humidity; // External probe humidity in %RH
    // BME688 air quality (BSEC). Complementary to the SCD4x true CO2 above.
    float iaq; // Indoor air-quality index 0..500 (lower = cleaner)
    uint8_t air_quality_accuracy; // BSEC calibration status 0..3
    float co2_equivalent; // Estimated CO2-equivalent in ppm
    float voc_equivalent; // Estimated breath-VOC-equivalent in ppm
    unsigned int updated_mask; // Values updated in last measurement
} sensor_data_t;

typedef void (*sensor_data_updated_callback_t)(const sensor_data_t *data);

typedef struct {
    sensor_bus_t sensor_bus; // Handle for the sensor bus
    ext_probe_t ext_probe; // Handle for the external probe
    sensor_data_updated_callback_t callback; // Callback function for data updates
    TaskHandle_t task_handle; // Handle for the FreeRTOS task
} sensor_service_t;

typedef sensor_service_t *sensor_service_handle_t;

esp_err_t sensor_service_init(sensor_service_handle_t handle, sensor_data_updated_callback_t callback);
esp_err_t sensor_service_start(sensor_service_handle_t service_handle);
esp_err_t sensor_service_stop(sensor_service_handle_t service_handle);
