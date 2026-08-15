// Host tests for the sensor conditioning / redundancy / event-detection layer
// in main/include/sensor_fusion.hpp.
//
// Everything under test is plain C++ on floats, so the whole layer runs here at
// simulated time: each test drives update() with an explicit dt instead of
// waiting on hardware, which is what makes the multi-minute behaviours (fire
// confirmation, occupancy vacate delay, window hold-off) testable at all.

#include "sensor_fusion.hpp"

#include "unity.h"

#include <array>
#include <cstdio>
#include <vector>

using namespace sensor_board::fusion;

namespace {

ChannelConfig temperatureChannelConfig()
{
    ChannelConfig config;
    config.minValid = -40.0f;
    config.maxValid = 125.0f;
    config.maxStepPerSecond = 1.0f;
    config.filterTauSeconds = 10.0f;
    config.staleAfterSeconds = 60.0f;
    config.maxConsecutiveRejects = 3;
    return config;
}

// Feed a channel `count` identical samples at `dt` spacing.
void feed(FilteredChannel &channel, float value, int count, float dt = 1.0f)
{
    for (int i = 0; i < count; ++i) {
        channel.update(dt, value, true);
    }
}

} // namespace

// --- FilteredChannel -------------------------------------------------------

void test_channel_adopts_first_sample_without_filtering(void)
{
    FilteredChannel channel;
    channel.configure(temperatureChannelConfig());

    TEST_ASSERT_FALSE(channel.valid());
    TEST_ASSERT_TRUE(channel.update(1.0f, 21.5f, true));
    TEST_ASSERT_TRUE(channel.valid());
    // No lag on the very first reading: a fresh boot must not spend a filter
    // time constant ramping up from zero.
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 21.5f, channel.value());
}

void test_channel_rejects_out_of_range_readings(void)
{
    FilteredChannel channel;
    channel.configure(temperatureChannelConfig());
    feed(channel, 21.0f, 5);

    TEST_ASSERT_FALSE(channel.update(1.0f, 999.0f, true));
    TEST_ASSERT_FALSE(channel.update(1.0f, -273.0f, true));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 21.0f, channel.value());
    TEST_ASSERT_EQUAL_UINT32(2, channel.rejectedSamples());
}

void test_channel_smooths_noise(void)
{
    FilteredChannel channel;
    channel.configure(temperatureChannelConfig());
    feed(channel, 21.0f, 30);

    // Alternating +-0.4 K of noise around 21.0 should barely move a 10 s EMA
    // sampled at 1 Hz — this is the oversampling that keeps a 0.2 K COV
    // threshold from transmitting on noise alone.
    for (int i = 0; i < 40; ++i) {
        channel.update(1.0f, (i % 2 == 0) ? 21.4f : 20.6f, true);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.15f, 21.0f, channel.value());
}

void test_channel_rejects_spike_then_resyncs(void)
{
    FilteredChannel channel;
    channel.configure(temperatureChannelConfig());  // 1 K/s step allowance
    feed(channel, 21.0f, 20);

    // A single corrupted transfer is dropped...
    TEST_ASSERT_FALSE(channel.update(1.0f, 60.0f, true));
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 21.0f, channel.value());

    // ...but a reading that stays there is eventually believed, because the
    // alternative is a sensor that silently never updates again.
    channel.update(1.0f, 60.0f, true);
    channel.update(1.0f, 60.0f, true);
    TEST_ASSERT_TRUE(channel.update(1.0f, 60.0f, true));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 60.0f, channel.value());
}

void test_channel_goes_stale_without_samples(void)
{
    FilteredChannel channel;
    channel.configure(temperatureChannelConfig());  // stale after 60 s
    feed(channel, 21.0f, 5);
    TEST_ASSERT_TRUE(channel.valid());

    for (int i = 0; i < 59; ++i) {
        channel.update(1.0f, 0.0f, false);
    }
    TEST_ASSERT_TRUE(channel.valid());
    channel.update(2.0f, 0.0f, false);
    TEST_ASSERT_FALSE(channel.valid());
    // "Was fitted and answered once" survives going stale, so the diagnostics
    // can tell a dead sensor from one that was never there.
    TEST_ASSERT_TRUE(channel.everSeen());
}

// --- RedundantMeasurement --------------------------------------------------

namespace {

RedundantMeasurement makeTemperatureFusion(float tolerance = 1.5f)
{
    RedundantMeasurement fusion;
    for (size_t i = 0; i < kMaxSources; ++i) {
        fusion.configureSource(i, temperatureChannelConfig());
    }
    RedundancyConfig config;
    config.crossCheckTolerance = tolerance;
    fusion.configure(config);
    return fusion;
}

FusedValue drive(RedundantMeasurement &fusion,
                 std::array<SourceSample, kMaxSources> samples,
                 int cycles,
                 float dt = 1.0f)
{
    FusedValue out;
    for (int i = 0; i < cycles; ++i) {
        out = fusion.update(dt, std::span<const SourceSample>(samples.data(), samples.size()));
    }
    return out;
}

} // namespace

void test_fusion_prefers_primary_source(void)
{
    auto fusion = makeTemperatureFusion();
    const auto out = drive(fusion,
                           {SourceSample{21.0f, true},
                            SourceSample{21.4f, true},
                            SourceSample{20.8f, true}},
                           30);

    TEST_ASSERT_TRUE(out.valid);
    TEST_ASSERT_EQUAL_UINT8(3, out.healthyCount);
    TEST_ASSERT_FALSE(out.disagreement);
    TEST_ASSERT_EQUAL(static_cast<int>(FusionQuality::Validated), static_cast<int>(out.quality));
    // With three agreeing sources the median is published; all three are within
    // the tolerance of each other so the value is the middle one.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 21.0f, out.value);
}

void test_fusion_falls_back_when_primary_dies(void)
{
    auto fusion = makeTemperatureFusion();
    drive(fusion,
          {SourceSample{21.0f, true}, SourceSample{21.2f, true}, SourceSample{20.9f, true}},
          30);

    // HDC3020 stops answering; the BME688 and SCD4x in the same enclosure are
    // still measuring the same air, so the room does not lose its temperature.
    const auto out = drive(fusion,
                           {SourceSample{0.0f, false},
                            SourceSample{21.2f, true},
                            SourceSample{20.9f, true}},
                           80);

    TEST_ASSERT_TRUE(out.valid);
    TEST_ASSERT_TRUE(out.usingFallback);
    TEST_ASSERT_EQUAL_UINT8(2, out.healthyCount);
    TEST_ASSERT_EQUAL_UINT8(1, out.sourceIndex);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 21.2f, out.value);
}

void test_fusion_reports_no_data_when_all_sources_fail(void)
{
    auto fusion = makeTemperatureFusion();
    drive(fusion, {SourceSample{21.0f, true}, SourceSample{21.0f, true}, SourceSample{21.0f, true}},
          10);

    const auto out = drive(
        fusion, {SourceSample{}, SourceSample{}, SourceSample{}}, 80);

    TEST_ASSERT_FALSE(out.valid);
    TEST_ASSERT_EQUAL_UINT8(0, out.healthyCount);
    TEST_ASSERT_EQUAL(static_cast<int>(FusionQuality::NoData), static_cast<int>(out.quality));
    // The last known value is still handed back for display purposes.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 21.0f, out.value);
}

void test_fusion_flags_disagreement_between_two_sources(void)
{
    auto fusion = makeTemperatureFusion(1.5f);
    // Only two sources fitted, and they differ by 4 K. There is no majority to
    // arbitrate with, so the preferred source is kept and the disagreement is
    // reported rather than silently averaged away.
    const auto out = drive(
        fusion, {SourceSample{21.0f, true}, SourceSample{25.0f, true}, SourceSample{}}, 40);

    TEST_ASSERT_TRUE(out.valid);
    TEST_ASSERT_TRUE(out.disagreement);
    TEST_ASSERT_EQUAL(static_cast<int>(FusionQuality::Disputed), static_cast<int>(out.quality));
    TEST_ASSERT_EQUAL_UINT8(0, out.sourceIndex);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 21.0f, out.value);
}

void test_fusion_outvotes_a_drifting_primary(void)
{
    auto fusion = makeTemperatureFusion(1.5f);
    // The HDC3020 has drifted 5 K high; the other two agree with each other.
    // The median rule drops it out of the published value automatically and
    // names it in suspectMask so the drift is actionable.
    const auto out = drive(fusion,
                           {SourceSample{26.0f, true},
                            SourceSample{21.2f, true},
                            SourceSample{21.0f, true}},
                           60);

    TEST_ASSERT_TRUE(out.valid);
    TEST_ASSERT_TRUE(out.disagreement);
    TEST_ASSERT_TRUE(out.usingFallback);
    TEST_ASSERT_FLOAT_WITHIN(0.4f, 21.2f, out.value);
    TEST_ASSERT_EQUAL_UINT8(0x01, out.suspectMask & 0x01);  // source 0 is the outlier
    TEST_ASSERT_EQUAL_UINT8(0, out.suspectMask & 0x06);     // the other two are fine
}

void test_fusion_applies_per_source_offsets(void)
{
    RedundantMeasurement fusion;
    for (size_t i = 0; i < kMaxSources; ++i) {
        fusion.configureSource(i, temperatureChannelConfig());
    }
    RedundancyConfig config;
    config.crossCheckTolerance = 1.0f;
    // The BME688 and SCD4x read high from their own self-heating; correcting
    // that here is what keeps the cross-check from firing on healthy parts.
    config.sourceOffsets = {0.0f, -1.2f, -0.8f};
    fusion.configure(config);

    const auto out = drive(fusion,
                           {SourceSample{21.0f, true},
                            SourceSample{22.2f, true},
                            SourceSample{21.8f, true}},
                           40);

    TEST_ASSERT_FALSE(out.disagreement);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 21.0f, out.value);
}

// --- TrendMonitor ----------------------------------------------------------

void test_trend_measures_a_linear_rise(void)
{
    TrendMonitor<16> trend;
    float value = 20.0f;
    for (int i = 0; i < 40; ++i) {
        trend.update(5.0f, value, true);
        value += 0.25f;  // 0.25 K per 5 s = 3 K/min
    }
    TEST_ASSERT_TRUE(trend.ready());
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 3.0f, trend.ratePerMinute());
}

void test_trend_is_quiet_on_a_flat_signal(void)
{
    TrendMonitor<16> trend;
    for (int i = 0; i < 40; ++i) {
        trend.update(5.0f, 21.0f + ((i % 2) ? 0.05f : -0.05f), true);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, trend.ratePerMinute());
}

void test_trend_resets_when_the_source_goes_invalid(void)
{
    TrendMonitor<16> trend;
    for (int i = 0; i < 20; ++i) {
        trend.update(5.0f, 20.0f + 0.5f * static_cast<float>(i), true);
    }
    TEST_ASSERT_TRUE(trend.ready());
    trend.update(5.0f, 0.0f, false);
    // A dead sensor must not leave a stale slope behind that a fire detector
    // would then act on.
    TEST_ASSERT_FALSE(trend.ready());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, trend.ratePerMinute());
}

// --- FireDetector ----------------------------------------------------------

namespace {

FireDetectorConfig fireConfig()
{
    FireDetectorConfig config;
    config.rateOfRiseKPerMin = 4.0f;
    config.absoluteAlarmC = 55.0f;
    config.confirmSeconds = 30.0f;
    config.clearSeconds = 120.0f;
    config.rateArmAboveC = 28.0f;
    return config;
}

} // namespace

void test_fire_ignores_normal_room_warmup(void)
{
    FireDetector detector;
    detector.configure(fireConfig());

    FireDetectorOutputs out;
    for (int i = 0; i < 600; ++i) {
        // Morning boost: 1 K/min from 18 °C. Below both thresholds.
        out = detector.update({.temperatureC = 18.0f + static_cast<float>(i) / 60.0f,
                               .temperatureValid = true,
                               .temperatureRateKPerMin = 1.0f},
                              1.0f);
    }
    TEST_ASSERT_FALSE(out.alarm);
    TEST_ASSERT_FALSE(out.preAlarm);
}

void test_fire_needs_confirmation_before_latching(void)
{
    FireDetector detector;
    detector.configure(fireConfig());

    FireDetectorOutputs out;
    for (int i = 0; i < 20; ++i) {  // 20 s < 30 s confirmation
        out = detector.update({.temperatureC = 35.0f,
                               .temperatureValid = true,
                               .temperatureRateKPerMin = 9.0f},
                              1.0f);
    }
    // A hairdryer or a sunbeam crossing the sensor gets this far and no further.
    TEST_ASSERT_FALSE(out.alarm);
    TEST_ASSERT_TRUE(out.preAlarm);

    for (int i = 0; i < 15; ++i) {
        out = detector.update({.temperatureC = 40.0f,
                               .temperatureValid = true,
                               .temperatureRateKPerMin = 9.0f},
                              1.0f);
    }
    TEST_ASSERT_TRUE(out.alarm);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(FireReasonBit::RateOfRise),
                            out.reasonMask & static_cast<uint8_t>(FireReasonBit::RateOfRise));
}

void test_fire_rate_path_is_disarmed_at_low_temperature(void)
{
    FireDetector detector;
    detector.configure(fireConfig());  // armed only above 28 °C

    FireDetectorOutputs out;
    for (int i = 0; i < 120; ++i) {
        out = detector.update({.temperatureC = 15.0f,
                               .temperatureValid = true,
                               .temperatureRateKPerMin = 9.0f},
                              1.0f);
    }
    TEST_ASSERT_FALSE(out.alarm);
}

void test_fire_absolute_threshold_alarms_without_a_rate(void)
{
    FireDetector detector;
    detector.configure(fireConfig());

    FireDetectorOutputs out;
    for (int i = 0; i < 40; ++i) {
        out = detector.update({.temperatureC = 58.0f,
                               .temperatureValid = true,
                               .temperatureRateKPerMin = 0.0f},
                              1.0f);
    }
    TEST_ASSERT_TRUE(out.alarm);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(FireReasonBit::AbsoluteTemperature),
        out.reasonMask & static_cast<uint8_t>(FireReasonBit::AbsoluteTemperature));
}

void test_fire_air_quality_corroboration_suppresses_a_hot_appliance(void)
{
    FireDetector detector;
    auto config = fireConfig();
    config.requireAirQualityRise = true;
    config.vocRiseIndexPerMin = 20.0f;
    detector.configure(config);

    // A fan heater: fast rise, clean air. No alarm.
    FireDetectorOutputs out;
    for (int i = 0; i < 120; ++i) {
        out = detector.update({.temperatureC = 35.0f,
                               .temperatureValid = true,
                               .temperatureRateKPerMin = 9.0f,
                               .airQualityRatePerMin = 1.0f,
                               .airQualityValid = true},
                              1.0f);
    }
    TEST_ASSERT_FALSE(out.alarm);

    // Same rise with volatiles climbing: that is combustion.
    for (int i = 0; i < 40; ++i) {
        out = detector.update({.temperatureC = 35.0f,
                               .temperatureValid = true,
                               .temperatureRateKPerMin = 9.0f,
                               .airQualityRatePerMin = 45.0f,
                               .airQualityValid = true},
                              1.0f);
    }
    TEST_ASSERT_TRUE(out.alarm);
}

void test_fire_alarm_latches_then_auto_clears(void)
{
    FireDetector detector;
    detector.configure(fireConfig());  // clears after 120 s

    for (int i = 0; i < 40; ++i) {
        detector.update({.temperatureC = 58.0f, .temperatureValid = true}, 1.0f);
    }

    FireDetectorOutputs out =
        detector.update({.temperatureC = 22.0f, .temperatureValid = true}, 1.0f);
    TEST_ASSERT_TRUE(out.alarm);  // latched: does not drop the moment it cools

    for (int i = 0; i < 130; ++i) {
        out = detector.update({.temperatureC = 22.0f, .temperatureValid = true}, 1.0f);
    }
    TEST_ASSERT_FALSE(out.alarm);
}

void test_fire_holds_the_latch_when_the_sensor_dies(void)
{
    FireDetector detector;
    detector.configure(fireConfig());
    for (int i = 0; i < 40; ++i) {
        detector.update({.temperatureC = 58.0f, .temperatureValid = true}, 1.0f);
    }

    // A sensor that stops answering during a fire is the expected failure, not
    // an all-clear.
    FireDetectorOutputs out;
    for (int i = 0; i < 600; ++i) {
        out = detector.update({.temperatureValid = false}, 1.0f);
    }
    TEST_ASSERT_TRUE(out.alarm);

    detector.acknowledge();
    out = detector.update({.temperatureC = 22.0f, .temperatureValid = true}, 1.0f);
    TEST_ASSERT_FALSE(out.alarm);
}

// --- OccupancyEstimator ----------------------------------------------------

void test_occupancy_detects_a_co2_rise_and_releases_after_decay(void)
{
    OccupancyEstimator estimator;
    OccupancyConfig config;
    config.confirmSeconds = 120.0f;
    config.vacateSeconds = 300.0f;
    estimator.configure(config);

    // Empty room at the outdoor baseline.
    OccupancyOutputs out;
    for (int i = 0; i < 60; ++i) {
        out = estimator.update(430.0f, true, 0.0f, 10.0f);
    }
    TEST_ASSERT_FALSE(out.occupied);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 430.0f, out.baselinePpm);

    // Two people come in: CO2 climbs.
    float co2 = 430.0f;
    for (int i = 0; i < 60; ++i) {
        co2 += 5.0f;
        out = estimator.update(co2, true, 30.0f, 10.0f);
    }
    TEST_ASSERT_TRUE(out.occupied);
    TEST_ASSERT_TRUE(out.estimatedOccupants >= 1);

    // They leave; CO2 decays back. Occupancy is held through the decay, then
    // released — a slow release is correct, because CO2 lags the room emptying.
    for (int i = 0; i < 150; ++i) {
        co2 = co2 > 440.0f ? co2 - 6.0f : 435.0f;
        out = estimator.update(co2, true, -20.0f, 10.0f);
    }
    TEST_ASSERT_FALSE(out.occupied);
}

void test_occupancy_baseline_does_not_chase_a_long_occupancy(void)
{
    OccupancyEstimator estimator;
    OccupancyConfig config;
    config.baselineTrackingPpmPerHour = 20.0f;
    estimator.configure(config);

    estimator.update(420.0f, true, 0.0f, 10.0f);
    // Eight hours at 1100 ppm. The baseline may creep at most 20 ppm/h, so it
    // must not converge on the occupied level and blind the detector.
    OccupancyOutputs out;
    for (int i = 0; i < 8 * 360; ++i) {
        out = estimator.update(1100.0f, true, 0.0f, 10.0f);
    }
    TEST_ASSERT_TRUE(out.baselinePpm < 600.0f);
    TEST_ASSERT_TRUE(out.occupied);
}

// --- WindowDetector --------------------------------------------------------

void test_window_detects_a_ventilating_temperature_fall(void)
{
    WindowDetector detector;
    WindowDetectConfig config;
    config.fallKPerMin = 0.35f;
    config.co2FallPpmPerMin = 15.0f;
    config.confirmSeconds = 120.0f;
    config.minOpenSeconds = 300.0f;
    detector.configure(config);

    WindowDetectOutputs out;
    for (int i = 0; i < 20; ++i) {
        out = detector.update(-0.8f, true, -40.0f, true, 10.0f);
    }
    TEST_ASSERT_TRUE(out.windowOpen);
    TEST_ASSERT_TRUE(out.corroboratedByCo2);
}

void test_window_ignores_cooling_without_ventilation(void)
{
    WindowDetector detector;
    WindowDetectConfig config;
    config.fallKPerMin = 0.35f;
    config.co2FallPpmPerMin = 15.0f;
    config.confirmSeconds = 120.0f;
    detector.configure(config);

    // The heating switched off: the room cools, but nobody opened anything, so
    // the CO2 keeps climbing with the people still in it.
    WindowDetectOutputs out;
    for (int i = 0; i < 60; ++i) {
        out = detector.update(-0.8f, true, +5.0f, true, 10.0f);
    }
    TEST_ASSERT_FALSE(out.windowOpen);
}

void test_window_stays_open_for_the_minimum_hold(void)
{
    WindowDetector detector;
    WindowDetectConfig config;
    config.fallKPerMin = 0.35f;
    config.co2FallPpmPerMin = 0.0f;  // no corroboration required
    config.confirmSeconds = 60.0f;
    config.minOpenSeconds = 300.0f;
    detector.configure(config);

    for (int i = 0; i < 10; ++i) {
        detector.update(-0.8f, true, 0.0f, false, 10.0f);
    }
    // The fall stops as soon as inside meets outside; without the hold the
    // controller would flap between "window open" and "window closed".
    WindowDetectOutputs out = detector.update(0.0f, true, 0.0f, false, 10.0f);
    TEST_ASSERT_TRUE(out.windowOpen);

    for (int i = 0; i < 40; ++i) {
        out = detector.update(0.0f, true, 0.0f, false, 10.0f);
    }
    TEST_ASSERT_FALSE(out.windowOpen);
}

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_channel_adopts_first_sample_without_filtering);
    RUN_TEST(test_channel_rejects_out_of_range_readings);
    RUN_TEST(test_channel_smooths_noise);
    RUN_TEST(test_channel_rejects_spike_then_resyncs);
    RUN_TEST(test_channel_goes_stale_without_samples);

    RUN_TEST(test_fusion_prefers_primary_source);
    RUN_TEST(test_fusion_falls_back_when_primary_dies);
    RUN_TEST(test_fusion_reports_no_data_when_all_sources_fail);
    RUN_TEST(test_fusion_flags_disagreement_between_two_sources);
    RUN_TEST(test_fusion_outvotes_a_drifting_primary);
    RUN_TEST(test_fusion_applies_per_source_offsets);

    RUN_TEST(test_trend_measures_a_linear_rise);
    RUN_TEST(test_trend_is_quiet_on_a_flat_signal);
    RUN_TEST(test_trend_resets_when_the_source_goes_invalid);

    RUN_TEST(test_fire_ignores_normal_room_warmup);
    RUN_TEST(test_fire_needs_confirmation_before_latching);
    RUN_TEST(test_fire_rate_path_is_disarmed_at_low_temperature);
    RUN_TEST(test_fire_absolute_threshold_alarms_without_a_rate);
    RUN_TEST(test_fire_air_quality_corroboration_suppresses_a_hot_appliance);
    RUN_TEST(test_fire_alarm_latches_then_auto_clears);
    RUN_TEST(test_fire_holds_the_latch_when_the_sensor_dies);

    RUN_TEST(test_occupancy_detects_a_co2_rise_and_releases_after_decay);
    RUN_TEST(test_occupancy_baseline_does_not_chase_a_long_occupancy);

    RUN_TEST(test_window_detects_a_ventilating_temperature_fall);
    RUN_TEST(test_window_ignores_cooling_without_ventilation);
    RUN_TEST(test_window_stays_open_for_the_minimum_hold);

    return UNITY_END();
}
