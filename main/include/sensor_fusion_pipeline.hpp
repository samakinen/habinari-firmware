// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "sensor_fusion.hpp"
#include "sensor_fusion_service.h"

#include <cstdint>

namespace habinari {
namespace fusion {

/**
 * The fusion layer's state and its per-cycle step, without the lock.
 *
 * sensor_fusion_service.cpp owns the one instance the firmware runs and holds
 * the fusion lock around every call; this class is what it locks. It has no
 * ESP-IDF or FreeRTOS dependency, so main/test/test_sensor_fusion_service.cpp
 * drives it directly.
 */
class Pipeline {
public:
    /// Apply new tuning without disturbing filter or detector state.
    void configure(const sensor_fusion_config_t &config);
    /// Clear latched alarms (the fire alarm).
    void acknowledgeAlarms();
    /// One sampling cycle; see sensor_fusion_update().
    void update(const sensor_bus_results_t *bus,
                const ext_probe_results_t *probe,
                float dt_seconds,
                sensor_data_t *out);

private:
    // Trend windows. Long enough that the least-squares slope is not noise, short
    // enough that a fire is not averaged away: 20 samples at the 5 s sampling
    // interval is a 100 s window on temperature. CO2 and IAQ move slowly and their
    // sources update every ~30 s and ~3 s respectively, so they get more samples.
    using TemperatureTrend = TrendMonitor<20>;
    using SlowTrend = TrendMonitor<16>;

    struct State {
        sensor_fusion_config_t config{};

        RedundantMeasurement temperature;
        RedundantMeasurement humidity;
        FilteredChannel pressure;
        FilteredChannel co2;
        FilteredChannel iaq;
        FilteredChannel co2Equivalent;
        FilteredChannel vocEquivalent;
        FilteredChannel probeTemperature;
        FilteredChannel probeHumidity;

        TemperatureTrend temperatureTrend;
        SlowTrend humidityTrend;
        SlowTrend co2Trend;
        SlowTrend iaqTrend;

        FireDetector fire;
        OccupancyEstimator occupancy;
        WindowDetector window;

        // BSEC reports its calibration status alongside the air-quality outputs, so
        // it is only meaningful while those outputs are arriving. Latched here so a
        // cycle that simply carried no new BSEC result does not read as "accuracy
        // dropped to 0".
        uint8_t airQualityAccuracy{0};

        uint32_t sampleCount{0};
        uint32_t errorCount{0};
    };

    State state_;
};

} // namespace fusion
} // namespace habinari
