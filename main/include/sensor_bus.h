// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/i2c_master.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "bme68x_esp.h"
#include "sensor_readings.h"

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
    // SENSOR_BUS_PRESENT_* bits for the parts that came up in
    // sensor_bus_enable(). A part that did not is never addressed again, and
    // the fusion layer reports its measurands as unavailable.
    uint8_t present_mask;
} sensor_bus_t;

typedef sensor_bus_t *sensor_bus_handle_t;

// Which parts of the sensor package answered at start-up.
#define SENSOR_BUS_PRESENT_HDC302X (1u << 0)
#define SENSOR_BUS_PRESENT_SCD4X   (1u << 1)
#define SENSOR_BUS_PRESENT_BME68X  (1u << 2)

// The per-cycle readings (SENSOR_BUS_* bits, sensor_bus_results_t) are in
// sensor_readings.h.

esp_err_t sensor_bus_init(sensor_bus_handle_t sensor_bus_handle);
// Powers the package and brings up each part. A part that does not come up is
// logged, released and left out of present_mask; that is ESP_OK, because one
// missing sensor is a degraded device, not a dead one. Only a bus that cannot
// be created at all is an error.
esp_err_t sensor_bus_enable(sensor_bus_handle_t sensor_bus_handle);
esp_err_t sensor_bus_disable(sensor_bus_handle_t sensor_bus_handle);
// Runs one acquisition cycle. Always fills @p results, including a per-cycle
// error count, and returns ESP_OK whenever at least one reading was obtained —
// a single dead sensor is not a failure of the bus.
esp_err_t sensor_bus_read(sensor_bus_handle_t sensor_bus_handle, sensor_bus_results_t *results);

#ifdef __cplusplus
}
#endif
