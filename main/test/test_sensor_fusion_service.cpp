// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// Host tests for the fusion glue in main/src/sensor_fusion_pipeline.cpp: the
// mapping of the board's sensors onto measurands, what each cycle exports into
// sensor_data_t, the health masks, the event switches and the alarm
// acknowledge. The algorithms underneath have their own tests in
// test_sensor_fusion.cpp; these check that the board is wired onto them the
// way the rest of the firmware expects.
//
// sensor_fusion_service.cpp adds only the FreeRTOS lock and a debug log line
// around the same calls, so it is not built here.

#include "sensor_fusion_pipeline.hpp"

#include "unity.h"

#include <memory>

using habinari::fusion::Pipeline;

namespace {

constexpr float kDt = 5.0f;  // the sensor task's sampling interval

std::unique_ptr<Pipeline> makePipeline(void (*tune)(sensor_fusion_config_t &) = nullptr)
{
    auto pipeline = std::make_unique<Pipeline>();
    sensor_fusion_config_t config;
    sensor_fusion_default_config(&config);
    if (tune != nullptr) {
        tune(config);
    }
    pipeline->configure(config);
    return pipeline;
}

// A healthy internal package: all three parts answering, in agreement.
sensor_bus_results_t healthyBus(float temperatureC = 22.0f)
{
    sensor_bus_results_t bus{};
    bus.hdc302x_temperature = temperatureC;
    bus.hdc302x_humidity = 45.0f;
    bus.bme68x_temperature = temperatureC;
    bus.bme68x_humidity = 45.0f;
    bus.bme68x_pressure = 101325.0f;
    bus.scd4x_temperature = temperatureC;
    bus.scd4x_humidity = 45.0f;
    bus.scd4x_co2 = 600;
    bus.updated_mask = SENSOR_BUS_HDC302X_TEMPERATURE | SENSOR_BUS_HDC302X_HUMIDITY
                       | SENSOR_BUS_BME68X_TEMPERATURE | SENSOR_BUS_BME68X_HUMIDITY
                       | SENSOR_BUS_BME68X_PRESSURE | SENSOR_BUS_SCD4X_TEMPERATURE
                       | SENSOR_BUS_SCD4X_HUMIDITY | SENSOR_BUS_SCD4X_CO2;
    return bus;
}

constexpr uint16_t kHdcBits = SENSOR_BUS_HDC302X_TEMPERATURE | SENSOR_BUS_HDC302X_HUMIDITY;

sensor_data_t run(Pipeline &pipeline,
                  const sensor_bus_results_t *bus,
                  const ext_probe_results_t *probe,
                  int cycles)
{
    sensor_data_t out{};
    for (int i = 0; i < cycles; ++i) {
        pipeline.update(bus, probe, kDt, &out);
    }
    return out;
}

constexpr uint8_t kInternalPackages = SENSOR_SOURCE_BIT(SENSOR_SOURCE_HDC302X)
                                      | SENSOR_SOURCE_BIT(SENSOR_SOURCE_BME68X)
                                      | SENSOR_SOURCE_BIT(SENSOR_SOURCE_SCD4X);

} // namespace

// --- Mapping and export -----------------------------------------------------

void test_healthy_board_publishes_the_reference_sensor()
{
    auto pipeline = makePipeline();
    const auto bus = healthyBus(22.0f);
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 3);

    TEST_ASSERT_TRUE(out.temperature.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 22.0f, out.temperature.value);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_HDC302X, out.temperature.source);
    TEST_ASSERT_EQUAL(3, out.temperature.healthy_count);
    TEST_ASSERT_FALSE(out.temperature.fallback);
    TEST_ASSERT_FALSE(out.temperature.disagreement);
    TEST_ASSERT_TRUE(out.humidity.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_HDC302X, out.humidity.source);

    TEST_ASSERT_TRUE(out.pressure.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_BME68X, out.pressure.source);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 101325.0f, out.pressure.value);
    TEST_ASSERT_TRUE(out.co2.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_SCD4X, out.co2.source);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 600.0f, out.co2.value);

    // No BSEC on this bus: the air-quality outputs stay unavailable.
    TEST_ASSERT_FALSE(out.iaq.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_NONE, out.iaq.source);
    TEST_ASSERT_EQUAL(0, out.air_quality_accuracy);

    TEST_ASSERT_EQUAL_HEX8(kInternalPackages, out.health.present_mask);
    TEST_ASSERT_EQUAL_HEX8(kInternalPackages, out.health.healthy_mask);
    TEST_ASSERT_EQUAL_HEX8(0, out.health.suspect_mask);
    TEST_ASSERT_EQUAL_UINT32(3, out.health.sample_count);
    TEST_ASSERT_EQUAL_UINT32(0, out.health.error_count);
}

void test_bus_errors_are_counted_per_cycle()
{
    auto pipeline = makePipeline();
    auto bus = healthyBus();
    bus.error_count = 4;  // four failed transactions are still one bad cycle
    run(*pipeline, &bus, nullptr, 2);
    bus.error_count = 0;
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 1);
    TEST_ASSERT_EQUAL_UINT32(3, out.health.sample_count);
    TEST_ASSERT_EQUAL_UINT32(2, out.health.error_count);
}

void test_no_bus_reports_nothing_valid()
{
    auto pipeline = makePipeline();
    const sensor_data_t out = run(*pipeline, nullptr, nullptr, 2);
    TEST_ASSERT_FALSE(out.temperature.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_NONE, out.temperature.source);
    TEST_ASSERT_FALSE(out.co2.valid);
    TEST_ASSERT_FALSE(out.pressure.valid);
    TEST_ASSERT_EQUAL_HEX8(0, out.health.present_mask);
    TEST_ASSERT_EQUAL_UINT32(2, out.health.sample_count);
}

// --- Missing and failing sensors -------------------------------------------

void test_missing_reference_sensor_falls_back_to_the_bme688()
{
    auto pipeline = makePipeline();
    auto bus = healthyBus(21.0f);
    bus.updated_mask &= static_cast<uint16_t>(~kHdcBits);  // HDC3020 not fitted
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 3);

    TEST_ASSERT_TRUE(out.temperature.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_BME68X, out.temperature.source);
    TEST_ASSERT_TRUE(out.temperature.fallback);
    TEST_ASSERT_EQUAL(2, out.temperature.healthy_count);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_BME68X, out.humidity.source);

    // Never answered: neither present nor counted as sick.
    const uint8_t expected = SENSOR_SOURCE_BIT(SENSOR_SOURCE_BME68X)
                             | SENSOR_SOURCE_BIT(SENSOR_SOURCE_SCD4X);
    TEST_ASSERT_EQUAL_HEX8(expected, out.health.present_mask);
    TEST_ASSERT_EQUAL_HEX8(expected, out.health.healthy_mask);
}

void test_reference_sensor_that_stops_is_present_but_unhealthy()
{
    auto pipeline = makePipeline();
    auto bus = healthyBus(22.0f);
    run(*pipeline, &bus, nullptr, 3);

    // The HDC3020 stops answering; after the staleness allowance (120 s by
    // default) the BME688 carries the measurand.
    bus.updated_mask &= static_cast<uint16_t>(~kHdcBits);
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 30);  // 150 s

    TEST_ASSERT_TRUE(out.temperature.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_BME68X, out.temperature.source);
    TEST_ASSERT_TRUE(out.temperature.fallback);
    TEST_ASSERT_EQUAL_HEX8(kInternalPackages, out.health.present_mask);
    TEST_ASSERT_EQUAL_HEX8(kInternalPackages & ~SENSOR_SOURCE_BIT(SENSOR_SOURCE_HDC302X),
                           out.health.healthy_mask);
}

void test_a_drifting_sensor_is_named_suspect()
{
    auto pipeline = makePipeline();
    auto bus = healthyBus(22.0f);
    bus.scd4x_temperature = 26.0f;  // 4 K off, beyond the 1.5 K cross-check
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 3);

    // Strict priority: the reference is still what is published.
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_HDC302X, out.temperature.source);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 22.0f, out.temperature.value);
    TEST_ASSERT_TRUE(out.temperature.disagreement);
    TEST_ASSERT_EQUAL_HEX8(SENSOR_SOURCE_BIT(SENSOR_SOURCE_SCD4X), out.temperature.suspect_mask);
    TEST_ASSERT_EQUAL_HEX8(SENSOR_SOURCE_BIT(SENSOR_SOURCE_SCD4X), out.health.suspect_mask);
}

void test_package_offsets_are_applied_before_the_cross_check()
{
    auto pipeline = makePipeline([](sensor_fusion_config_t &config) {
        config.scd4x_temperature_offset_k = -4.0f;  // the lamp warms it by 4 K
    });
    auto bus = healthyBus(22.0f);
    bus.scd4x_temperature = 26.0f;
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 3);
    TEST_ASSERT_FALSE(out.temperature.disagreement);
    TEST_ASSERT_EQUAL_HEX8(0, out.health.suspect_mask);
}

void test_probe_is_its_own_measurand_and_never_a_room_fallback()
{
    auto pipeline = makePipeline();
    sensor_bus_results_t bus{};  // internal package silent
    const ext_probe_results_t probe{19.5f, 60.0f};
    const sensor_data_t out = run(*pipeline, &bus, &probe, 3);

    TEST_ASSERT_TRUE(out.probe_temperature.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_SHT4X, out.probe_temperature.source);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 19.5f, out.probe_temperature.value);
    TEST_ASSERT_TRUE(out.probe_humidity.valid);
    TEST_ASSERT_FALSE(out.temperature.valid);
    TEST_ASSERT_EQUAL_HEX8(SENSOR_SOURCE_BIT(SENSOR_SOURCE_SHT4X), out.health.present_mask);
}

void test_absent_probe_is_unavailable()
{
    auto pipeline = makePipeline();
    const auto bus = healthyBus();
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 3);
    TEST_ASSERT_FALSE(out.probe_temperature.valid);
    TEST_ASSERT_EQUAL(SENSOR_SOURCE_NONE, out.probe_temperature.source);
    TEST_ASSERT_FALSE(out.probe_humidity.valid);
    TEST_ASSERT_EQUAL(0, out.health.present_mask & SENSOR_SOURCE_BIT(SENSOR_SOURCE_SHT4X));
}

// --- Air quality -------------------------------------------------------------

void test_air_quality_accuracy_is_latched_and_dropped_when_iaq_goes_stale()
{
    auto pipeline = makePipeline();
    auto bus = healthyBus();
    bus.bme68x_iaq = 50.0f;
    bus.bme68x_iaq_accuracy = 3;
    bus.updated_mask |= SENSOR_BUS_BME68X_IAQ;
    sensor_data_t out = run(*pipeline, &bus, nullptr, 2);
    TEST_ASSERT_TRUE(out.iaq.valid);
    TEST_ASSERT_EQUAL(3, out.air_quality_accuracy);

    // A cycle without a BSEC result keeps the last status...
    bus.updated_mask &= static_cast<uint16_t>(~SENSOR_BUS_BME68X_IAQ);
    bus.bme68x_iaq_accuracy = 0;
    out = run(*pipeline, &bus, nullptr, 1);
    TEST_ASSERT_EQUAL(3, out.air_quality_accuracy);

    // ...until the IAQ channel itself goes stale (2 x 120 s for BSEC outputs).
    out = run(*pipeline, &bus, nullptr, 60);  // 300 s
    TEST_ASSERT_FALSE(out.iaq.valid);
    TEST_ASSERT_EQUAL(0, out.air_quality_accuracy);
}

// --- Events --------------------------------------------------------------------

void test_fire_alarm_latches_and_acknowledge_clears_it()
{
    auto pipeline = makePipeline([](sensor_fusion_config_t &config) {
        config.fire_rate_of_rise_k_per_min = 0.0f;  // absolute path only
        config.fire_absolute_alarm_c = 25.0f;
        config.fire_confirm_seconds = 30.0f;
    });
    const auto bus = healthyBus(30.0f);

    sensor_data_t out = run(*pipeline, &bus, nullptr, 1);
    TEST_ASSERT_TRUE(out.events.fire_pre_alarm);
    TEST_ASSERT_FALSE(out.events.fire_alarm);

    out = run(*pipeline, &bus, nullptr, 7);  // 40 s in total, past the 30 s confirmation
    TEST_ASSERT_TRUE(out.events.fire_alarm);
    TEST_ASSERT_NOT_EQUAL(0, out.events.fire_reason_mask);

    pipeline->acknowledgeAlarms();
    out = run(*pipeline, &bus, nullptr, 1);
    // Acknowledged: the latch is gone, and a condition that is still present
    // has to be confirmed again from the start.
    TEST_ASSERT_FALSE(out.events.fire_alarm);
    TEST_ASSERT_TRUE(out.events.fire_pre_alarm);
}

void test_disabled_occupancy_and_window_report_nothing()
{
    auto pipeline = makePipeline([](sensor_fusion_config_t &config) {
        config.co2_occupancy_enabled = false;
        config.window_detect_enabled = false;
    });
    auto bus = healthyBus();
    bus.scd4x_co2 = 1500;
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 10);
    TEST_ASSERT_FALSE(out.events.occupancy_detected);
    TEST_ASSERT_EQUAL(0, out.events.estimated_occupants);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.events.co2_baseline_ppm);
    TEST_ASSERT_FALSE(out.events.window_open_detected);
}

void test_enabled_occupancy_tracks_a_baseline()
{
    auto pipeline = makePipeline();
    const auto bus = healthyBus();
    const sensor_data_t out = run(*pipeline, &bus, nullptr, 10);
    TEST_ASSERT_TRUE(out.events.co2_baseline_ppm > 0.0f);
}

void test_trends_become_valid_once_the_window_fills()
{
    auto pipeline = makePipeline();
    const auto bus = healthyBus();
    sensor_data_t out = run(*pipeline, &bus, nullptr, 1);
    TEST_ASSERT_FALSE(out.trends.temperature.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.trends.temperature.per_minute);

    out = run(*pipeline, &bus, nullptr, 40);
    TEST_ASSERT_TRUE(out.trends.temperature.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.trends.temperature.per_minute);
}

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_healthy_board_publishes_the_reference_sensor);
    RUN_TEST(test_bus_errors_are_counted_per_cycle);
    RUN_TEST(test_no_bus_reports_nothing_valid);

    RUN_TEST(test_missing_reference_sensor_falls_back_to_the_bme688);
    RUN_TEST(test_reference_sensor_that_stops_is_present_but_unhealthy);
    RUN_TEST(test_a_drifting_sensor_is_named_suspect);
    RUN_TEST(test_package_offsets_are_applied_before_the_cross_check);
    RUN_TEST(test_probe_is_its_own_measurand_and_never_a_room_fallback);
    RUN_TEST(test_absent_probe_is_unavailable);

    RUN_TEST(test_air_quality_accuracy_is_latched_and_dropped_when_iaq_goes_stale);

    RUN_TEST(test_fire_alarm_latches_and_acknowledge_clears_it);
    RUN_TEST(test_disabled_occupancy_and_window_report_nothing);
    RUN_TEST(test_enabled_occupancy_tracks_a_baseline);
    RUN_TEST(test_trends_become_valid_once_the_window_fills);

    return UNITY_END();
}
