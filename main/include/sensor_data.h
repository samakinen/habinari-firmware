// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file sensor_data.h
 * @brief The one measurement record the whole firmware passes around.
 *
 * This replaces the three overlapping "updated_mask" vocabularies the firmware
 * used to carry — one in sensor_bus.h, a differently numbered one in
 * sensor_service.h, and a third accumulated inside knx_service — with a value
 * that states its own validity. Each measurand travels as a sensor_value_t, so
 * "is this number real?" is answered next to the number instead of by a bit in
 * a mask that a consumer had to remember to translate.
 *
 * Every field here is post-fusion: conditioned, cross-validated and, where the
 * board has more than one sensor for the same quantity, taken from whichever
 * part is currently healthy (see sensor_fusion.hpp). Consumers — the KNX
 * service and the Modbus slave — therefore never see raw sensor readings and
 * never need to know which physical part produced a value, except when they
 * report diagnostics.
 */

/// Physical sensor packages on the board, as reported in sensor_value_t.source
/// and in the health bitmasks. The numbering is part of the Modbus register map
/// and of the KNX diagnostic objects, so it is stable.
typedef enum {
    SENSOR_SOURCE_HDC302X = 0,  ///< reference room air temperature / humidity
    SENSOR_SOURCE_BME68X = 1,   ///< pressure, gas/IAQ, plus T/RH
    SENSOR_SOURCE_SCD4X = 2,    ///< true CO2, plus T/RH from its compensation
    SENSOR_SOURCE_SHT4X = 3,    ///< optional external/in-slab probe
    SENSOR_SOURCE_COUNT = 4,
    SENSOR_SOURCE_NONE = 0xFF,
} sensor_source_t;

#define SENSOR_SOURCE_BIT(source) ((uint8_t)(1u << (source)))

/// One fused measurand, with the provenance a diagnostic read needs.
typedef struct {
    float value;
    bool valid;             ///< false = no healthy source; `value` is the last known one
    uint8_t source;         ///< sensor_source_t actually in use, or SENSOR_SOURCE_NONE
    uint8_t healthy_count;  ///< how many sources are currently delivering this measurand
    uint8_t suspect_mask;   ///< sensor_source_t bits disagreeing with the published value
    bool fallback;          ///< the preferred sensor for this measurand is not the one in use
    bool disagreement;      ///< healthy sources disagree by more than the configured tolerance
} sensor_value_t;

/// A rate of change. Separated from the measurand because a value can be valid
/// while its trend is not yet (the window has to fill first).
typedef struct {
    float per_minute;
    bool valid;
} sensor_trend_t;

typedef struct {
    sensor_trend_t temperature;  ///< K/min
    sensor_trend_t humidity;     ///< %RH/min
    sensor_trend_t co2;          ///< ppm/min
    sensor_trend_t air_quality;  ///< IAQ index/min
} sensor_trends_t;

/// Events inferred from the measurements rather than measured directly.
/// See sensor_fusion.hpp for what each one is and is not allowed to claim.
typedef struct {
    bool fire_alarm;             ///< confirmed rapid rise / over-temperature (advisory, not EN 54)
    bool fire_pre_alarm;         ///< condition present, confirmation time not yet elapsed
    uint8_t fire_reason_mask;    ///< habinari::fusion::FireReasonBit values
    bool occupancy_detected;     ///< inferred from the CO2 signal
    uint8_t estimated_occupants; ///< order-of-magnitude only, never used for control
    float co2_baseline_ppm;      ///< tracked fresh-air level the excess is measured against
    bool window_open_detected;   ///< inferred from a ventilating temperature fall
} sensor_events_t;

/// Sensor package health, as bitmasks over sensor_source_t.
typedef struct {
    uint8_t present_mask;  ///< has answered at least once since boot (i.e. is fitted)
    uint8_t healthy_mask;  ///< currently delivering accepted readings
    uint8_t suspect_mask;  ///< disagreeing with its peers on some measurand
    uint32_t sample_count; ///< sampling cycles completed
    uint32_t error_count;  ///< cycles in which at least one bus transaction failed
} sensor_health_t;

typedef struct {
    // --- Room air, fused across every sensor that measures it --------------
    sensor_value_t temperature;      ///< °C
    sensor_value_t humidity;         ///< %RH
    sensor_value_t pressure;         ///< Pa, station pressure
    sensor_value_t co2;              ///< ppm, SCD4x true CO2 (no redundant source)
    sensor_value_t iaq;              ///< BSEC index 0..500
    sensor_value_t co2_equivalent;   ///< ppm, BSEC
    sensor_value_t voc_equivalent;   ///< ppm, BSEC
    uint8_t air_quality_accuracy;    ///< BSEC calibration status 0..3

    // --- External probe (separate location, never a room-air fallback) -----
    sensor_value_t probe_temperature;  ///< °C
    sensor_value_t probe_humidity;     ///< %RH

    sensor_trends_t trends;
    sensor_events_t events;
    sensor_health_t health;
} sensor_data_t;

#ifdef __cplusplus
}
#endif
