#pragma once

#include "esp_err.h"
#include "ext_probe.h"
#include "sensor_bus.h"
#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file sensor_fusion_service.h
 * @brief C entry point to the fusion layer in sensor_fusion.hpp.
 *
 * The fusion algorithms are header-only C++ so they can be host-tested without
 * ESP-IDF (main/test/test_sensor_fusion.cpp). This is the thin, stateful glue
 * that owns one instance of them, maps the board's four physical sensors onto
 * the redundant measurands, and produces the sensor_data_t the rest of the
 * firmware consumes.
 *
 * Threading: sensor_fusion_update() is called only from the sensor task.
 * sensor_fusion_configure() may be called from any task (the KNX task pushes
 * ETS parameter changes into it) and takes the config lock.
 */

/// Runtime tuning. Everything an integrator can reach from ETS ends up here;
/// sensor_fusion_default_config() fills in the compiled-in defaults.
typedef struct {
    // --- Conditioning ------------------------------------------------------
    /// Exponential smoothing time constant for the room air measurands. This is
    /// the oversampling knob: sample fast, smooth here, publish on change.
    float filter_tau_seconds;
    /// A source with no accepted reading for this long drops out of the fusion
    /// and the next healthy sensor takes over.
    float stale_after_seconds;

    // --- Cross-validation --------------------------------------------------
    /// Sources differing by more than this are reported as disagreeing, and
    /// with three sources the outlier is voted out. 0 disables validation.
    float temperature_cross_check_k;
    float humidity_cross_check_pct;

    // --- Inter-sensor corrections -----------------------------------------
    /// Systematic offsets between the packages, applied before comparison. The
    /// BME688 (gas heater) and SCD4x (photoacoustic lamp) both read high next
    /// to their own heat sources; without these the cross-check would fire
    /// permanently on healthy parts. Distinct from the ETS installation offset,
    /// which corrects the whole board against a reference thermometer.
    float bme68x_temperature_offset_k;
    float scd4x_temperature_offset_k;
    float bme68x_humidity_offset_pct;
    float scd4x_humidity_offset_pct;

    // --- Fire / rapid rise -------------------------------------------------
    float fire_rate_of_rise_k_per_min;  ///< 0 disables the rate path
    float fire_absolute_alarm_c;        ///< 0 disables the fixed-temperature path
    float fire_confirm_seconds;
    float fire_clear_seconds;
    float fire_arm_above_c;
    bool fire_require_air_quality;      ///< also require volatiles to be rising

    // --- Derived room events ----------------------------------------------
    bool co2_occupancy_enabled;
    bool window_detect_enabled;
    float window_fall_k_per_min;        ///< 0 disables detection
} sensor_fusion_config_t;

/// Fill @p out with the compiled-in defaults.
void sensor_fusion_default_config(sensor_fusion_config_t *out);

/// Apply new tuning. Safe to call at any time and from any task; takes effect
/// on the next sampling cycle without disturbing filter or detector state.
void sensor_fusion_configure(const sensor_fusion_config_t *config);

/// Drop all conditioning and detector state (filters, trends, latched alarms).
void sensor_fusion_reset(void);

/// Clear latched alarms (currently the fire alarm) without disturbing the
/// filters or trends. This is what a bus acknowledge or a button press does.
void sensor_fusion_acknowledge_alarms(void);

/**
 * Fold one sampling cycle's raw readings into the fused record.
 *
 * @param bus         raw per-sensor readings from the internal I2C bus
 * @param probe       external probe readings, or NULL when it is not fitted
 * @param dt_seconds  time since the previous call
 * @param out         receives the complete fused record
 */
void sensor_fusion_update(const sensor_bus_results_t *bus,
                          const ext_probe_results_t *probe,
                          float dt_seconds,
                          sensor_data_t *out);

#ifdef __cplusplus
}
#endif
