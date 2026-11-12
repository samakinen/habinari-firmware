// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file sensor_readings.h
 * @brief The raw readings one acquisition cycle hands to the fusion layer.
 *
 * Kept apart from sensor_bus.h and ext_probe.h, which carry the I2C handles,
 * so the fusion glue (sensor_fusion_service.h) and its host test see the
 * readings without the drivers.
 */

// Which raw readings this acquisition cycle produced. These bits describe the
// *bus transaction*, not the measurement's usefulness: deciding what is valid,
// which source to prefer and when a sensor has gone quiet is the fusion layer's
// job (sensor_fusion.hpp). Consumers outside sensor_bus should be reading
// sensor_data_t, not this.
#define SENSOR_BUS_HDC302X_TEMPERATURE    (1u << 0)
#define SENSOR_BUS_HDC302X_HUMIDITY       (1u << 1)
#define SENSOR_BUS_SCD4X_TEMPERATURE      (1u << 2)
#define SENSOR_BUS_SCD4X_HUMIDITY         (1u << 3)
#define SENSOR_BUS_SCD4X_CO2              (1u << 4)
#define SENSOR_BUS_BME68X_TEMPERATURE     (1u << 5)
#define SENSOR_BUS_BME68X_HUMIDITY        (1u << 6)
#define SENSOR_BUS_BME68X_PRESSURE        (1u << 7)
// BME688 air-quality outputs (from BSEC; see components/bsec2). Only produced
// when the firmware is built with CONFIG_BME688_USE_BSEC.
#define SENSOR_BUS_BME68X_IAQ             (1u << 8)
#define SENSOR_BUS_BME68X_CO2_EQUIVALENT  (1u << 9)
#define SENSOR_BUS_BME68X_VOC_EQUIVALENT  (1u << 10)

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
    uint16_t updated_mask;  // SENSOR_BUS_* bits produced by this cycle
    uint8_t error_count;    // failed transactions in this cycle
} sensor_bus_results_t;

typedef struct {
    float temperature;
    float humidity;
} ext_probe_results_t;

#ifdef __cplusplus
}
#endif
