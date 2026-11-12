// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "sensor_fusion_pipeline.hpp"

#include <array>
#include <cstring>
#include <span>

// Glue between the raw acquisition layer (sensor_bus / ext_probe) and the
// platform-free fusion algorithms in sensor_fusion.hpp. The mapping from the
// board's four physical sensors onto redundant measurands lives here and
// nowhere else, so "which sensor backs the room temperature, and in what
// order?" is one table rather than a rule spread across the consumers.

namespace {

namespace fu = habinari::fusion;

// Priority order per measurand. Index 0 is the reference part and is published
// whenever it is delivering; the rest keep the measurand alive if it dies and
// cross-check it while it lives. Selection is strict priority, never a vote —
// see the RedundantMeasurement contract in sensor_fusion.hpp for why.
//
// Room temperature/humidity: the HDC3020 is the reference. It is fitted for
// exactly this measurement and placed away from anything on the board that
// dissipates power, and it is the only one of the three whose reading is not a
// by-product of some other job. The BME688 runs a gas heater and the SCD4x an
// optical source; both report part of their own self-heating as room
// temperature, and both are humidity-compensating for their own sake rather
// than reporting the room. So they rank below the HDC3020 — BME688 first, since
// its BSEC output is at least heat-compensated — and they exist here as a
// fallback and a drift witness, not as an equal vote.
//
// The SHT4x probe is deliberately NOT in this list. It measures a different
// place (in-slab or remote), so promoting it to a room-air fallback would
// silently control the room from the floor temperature — a plausible-looking
// number that is simply the wrong quantity.
constexpr std::array<sensor_source_t, fu::kMaxSources> kTemperatureSources = {
    SENSOR_SOURCE_HDC302X, SENSOR_SOURCE_BME68X, SENSOR_SOURCE_SCD4X};
constexpr std::array<sensor_source_t, fu::kMaxSources> kHumiditySources = {
    SENSOR_SOURCE_HDC302X, SENSOR_SOURCE_BME68X, SENSOR_SOURCE_SCD4X};

fu::ChannelConfig channelConfig(float minValid,
                                float maxValid,
                                float maxStepPerSecond,
                                float tauSeconds,
                                float staleSeconds)
{
    fu::ChannelConfig config;
    config.minValid = minValid;
    config.maxValid = maxValid;
    config.maxStepPerSecond = maxStepPerSecond;
    config.filterTauSeconds = tauSeconds;
    config.staleAfterSeconds = staleSeconds;
    config.maxConsecutiveRejects = 3;
    return config;
}

// Copy a FusedValue into the C-facing record, translating the fusion layer's
// source index into the board-wide sensor_source_t the bus objects report.
void exportValue(const fu::FusedValue &fused,
                 std::span<const sensor_source_t> sources,
                 sensor_value_t *out)
{
    out->value = fused.value;
    out->valid = fused.valid;
    out->healthy_count = fused.healthyCount;
    out->fallback = fused.valid && fused.usingFallback;
    out->disagreement = fused.disagreement;
    out->source = fused.valid && fused.sourceIndex < sources.size()
                      ? static_cast<uint8_t>(sources[fused.sourceIndex])
                      : static_cast<uint8_t>(SENSOR_SOURCE_NONE);

    // Re-express the suspect bits in board-wide source numbering, so a
    // diagnostic read names the part rather than a position in a list.
    uint8_t suspect = 0;
    for (size_t i = 0; i < sources.size(); ++i) {
        if ((fused.suspectMask & (1u << i)) != 0u) {
            suspect |= SENSOR_SOURCE_BIT(sources[i]);
        }
    }
    out->suspect_mask = suspect;
}

void exportChannel(const fu::FilteredChannel &channel,
                   sensor_source_t source,
                   sensor_value_t *out)
{
    out->value = channel.value();
    out->valid = channel.valid();
    out->healthy_count = channel.valid() ? 1 : 0;
    out->fallback = false;
    out->disagreement = false;
    out->suspect_mask = 0;
    out->source = channel.valid() ? static_cast<uint8_t>(source)
                                  : static_cast<uint8_t>(SENSOR_SOURCE_NONE);
}

// Liveness of one physical package, accumulated over the channels it feeds.
//
// A package is healthy only when *every* measurand it has been seen to produce
// is producing now — an OR across its channels would report a BME688 whose
// temperature has gone stale as fully healthy on the strength of its pressure
// reading, which is exactly the degraded state a health mask exists to expose.
//
// The bar is what the part has actually delivered, not a static list of what it
// could deliver. Otherwise the BME688 would read unhealthy throughout the BSEC
// warm-up, when nothing is wrong with it — "never calibrated" and "stopped
// working" are different service calls.
struct PackageHealth {
    bool everSeen{false};
    bool healthy{true};

    void observe(const fu::FilteredChannel &channel)
    {
        if (!channel.everSeen()) {
            return;
        }
        everSeen = true;
        if (!channel.valid()) {
            healthy = false;
        }
    }
};

sensor_trend_t exportTrend(float perMinute, bool ready, bool sourceValid)
{
    sensor_trend_t trend{};
    trend.valid = ready && sourceValid;
    trend.per_minute = trend.valid ? perMinute : 0.0f;
    return trend;
}

} // namespace

extern "C" void sensor_fusion_default_config(sensor_fusion_config_t *out)
{
    if (out == nullptr) {
        return;
    }
    *out = sensor_fusion_config_t{};
    // 30 s smooths a 5 s sampler hard enough that a 0.2 K COV threshold stops
    // transmitting on sensor noise, without adding lag a room can feel.
    out->filter_tau_seconds = 30.0f;
    out->stale_after_seconds = 120.0f;
    // 1.5 K / 6 %RH: wide enough that healthy parts in one enclosure never trip
    // it, tight enough to catch the drift that makes a room control 2 K off.
    out->temperature_cross_check_k = 1.5f;
    out->humidity_cross_check_pct = 6.0f;
    out->bme68x_temperature_offset_k = 0.0f;
    out->scd4x_temperature_offset_k = 0.0f;
    out->bme68x_humidity_offset_pct = 0.0f;
    out->scd4x_humidity_offset_pct = 0.0f;
    out->fire_rate_of_rise_k_per_min = 4.0f;
    out->fire_absolute_alarm_c = 55.0f;
    out->fire_confirm_seconds = 30.0f;
    out->fire_clear_seconds = 300.0f;
    out->fire_arm_above_c = 28.0f;
    out->fire_require_air_quality = false;
    out->co2_occupancy_enabled = true;
    out->window_detect_enabled = true;
    out->window_fall_k_per_min = 0.35f;
}

namespace habinari {
namespace fusion {

void Pipeline::configure(const sensor_fusion_config_t &newConfig)
{
    state_.config = newConfig;
    const sensor_fusion_config_t &config = state_.config;
    const float tau = config.filter_tau_seconds;
    const float stale = config.stale_after_seconds;

    // Plausibility windows are the sensors' own datasheet ranges, widened
    // slightly: anything outside them is a driver fault, not a measurement.
    const auto temperatureChannel = channelConfig(-40.0f, 125.0f, 2.0f, tau, stale);
    const auto humidityChannel = channelConfig(0.0f, 100.0f, 5.0f, tau, stale);

    for (size_t i = 0; i < fu::kMaxSources; ++i) {
        state_.temperature.configureSource(i, temperatureChannel);
        state_.humidity.configureSource(i, humidityChannel);
    }

    fu::RedundancyConfig temperatureRedundancy;
    temperatureRedundancy.crossCheckTolerance = config.temperature_cross_check_k;
    temperatureRedundancy.sourceOffsets = {0.0f, config.bme68x_temperature_offset_k,
                                           config.scd4x_temperature_offset_k};
    state_.temperature.configure(temperatureRedundancy);

    fu::RedundancyConfig humidityRedundancy;
    humidityRedundancy.crossCheckTolerance = config.humidity_cross_check_pct;
    humidityRedundancy.sourceOffsets = {0.0f, config.bme68x_humidity_offset_pct,
                                        config.scd4x_humidity_offset_pct};
    state_.humidity.configure(humidityRedundancy);

    // Pressure changes slowly and matters to two decimal places for the
    // sea-level reduction, so it gets a long time constant. The staleness
    // allowance is generous for CO2: the SCD4x low-power cadence is 30 s.
    state_.pressure.configure(channelConfig(30000.0f, 120000.0f, 500.0f, tau * 2.0f, stale));
    state_.co2.configure(channelConfig(300.0f, 40000.0f, 200.0f, tau, stale * 2.0f));
    state_.iaq.configure(channelConfig(0.0f, 500.0f, 50.0f, tau, stale * 2.0f));
    state_.co2Equivalent.configure(channelConfig(300.0f, 60000.0f, 500.0f, tau, stale * 2.0f));
    state_.vocEquivalent.configure(channelConfig(0.0f, 1000.0f, 50.0f, tau, stale * 2.0f));
    state_.probeTemperature.configure(channelConfig(-40.0f, 125.0f, 2.0f, tau, stale));
    state_.probeHumidity.configure(channelConfig(0.0f, 100.0f, 5.0f, tau, stale));

    fu::FireDetectorConfig fire;
    fire.rateOfRiseKPerMin = config.fire_rate_of_rise_k_per_min;
    fire.absoluteAlarmC = config.fire_absolute_alarm_c;
    fire.confirmSeconds = config.fire_confirm_seconds;
    fire.clearSeconds = config.fire_clear_seconds;
    fire.rateArmAboveC = config.fire_arm_above_c;
    fire.requireAirQualityRise = config.fire_require_air_quality;
    state_.fire.configure(fire);

    fu::WindowDetectConfig window;
    window.fallKPerMin = config.window_detect_enabled ? config.window_fall_k_per_min : 0.0f;
    state_.window.configure(window);

    state_.occupancy.configure(fu::OccupancyConfig{});
}

void Pipeline::acknowledgeAlarms()
{
    state_.fire.acknowledge();
}

void Pipeline::update(const sensor_bus_results_t *bus,
                      const ext_probe_results_t *probe,
                      float dt_seconds,
                      sensor_data_t *out)
{
    const sensor_fusion_config_t &config = state_.config;

    const bool haveBus = bus != nullptr;
    const uint16_t mask = haveBus ? bus->updated_mask : 0;
    const auto has = [mask](uint16_t bit) { return (mask & bit) != 0u; };

    // --- Redundant room air ------------------------------------------------
    const std::array<fu::SourceSample, fu::kMaxSources> temperatureSamples = {
        fu::SourceSample{haveBus ? bus->hdc302x_temperature : 0.0f,
                         has(SENSOR_BUS_HDC302X_TEMPERATURE)},
        fu::SourceSample{haveBus ? bus->bme68x_temperature : 0.0f,
                         has(SENSOR_BUS_BME68X_TEMPERATURE)},
        fu::SourceSample{haveBus ? bus->scd4x_temperature : 0.0f,
                         has(SENSOR_BUS_SCD4X_TEMPERATURE)},
    };
    const std::array<fu::SourceSample, fu::kMaxSources> humiditySamples = {
        fu::SourceSample{haveBus ? bus->hdc302x_humidity : 0.0f,
                         has(SENSOR_BUS_HDC302X_HUMIDITY)},
        fu::SourceSample{haveBus ? bus->bme68x_humidity : 0.0f,
                         has(SENSOR_BUS_BME68X_HUMIDITY)},
        fu::SourceSample{haveBus ? bus->scd4x_humidity : 0.0f, has(SENSOR_BUS_SCD4X_HUMIDITY)},
    };

    const auto temperature = state_.temperature.update(
        dt_seconds, std::span<const fu::SourceSample>(temperatureSamples));
    const auto humidity =
        state_.humidity.update(dt_seconds, std::span<const fu::SourceSample>(humiditySamples));

    exportValue(temperature, std::span<const sensor_source_t>(kTemperatureSources),
                &out->temperature);
    exportValue(humidity, std::span<const sensor_source_t>(kHumiditySources), &out->humidity);

    // --- Single-source measurands -----------------------------------------
    state_.pressure.update(dt_seconds, haveBus ? bus->bme68x_pressure : 0.0f,
                            has(SENSOR_BUS_BME68X_PRESSURE));
    state_.co2.update(dt_seconds, haveBus ? static_cast<float>(bus->scd4x_co2) : 0.0f,
                       has(SENSOR_BUS_SCD4X_CO2));
    state_.iaq.update(dt_seconds, haveBus ? bus->bme68x_iaq : 0.0f, has(SENSOR_BUS_BME68X_IAQ));
    state_.co2Equivalent.update(dt_seconds, haveBus ? bus->bme68x_co2_equivalent : 0.0f,
                                 has(SENSOR_BUS_BME68X_CO2_EQUIVALENT));
    state_.vocEquivalent.update(dt_seconds, haveBus ? bus->bme68x_voc_equivalent : 0.0f,
                                 has(SENSOR_BUS_BME68X_VOC_EQUIVALENT));

    exportChannel(state_.pressure, SENSOR_SOURCE_BME68X, &out->pressure);
    exportChannel(state_.co2, SENSOR_SOURCE_SCD4X, &out->co2);
    exportChannel(state_.iaq, SENSOR_SOURCE_BME68X, &out->iaq);
    exportChannel(state_.co2Equivalent, SENSOR_SOURCE_BME68X, &out->co2_equivalent);
    exportChannel(state_.vocEquivalent, SENSOR_SOURCE_BME68X, &out->voc_equivalent);
    // Only sample the calibration status on a cycle that actually carried BSEC
    // output, and drop it once the IAQ channel goes stale: consumers gate on
    // `accuracy > 0`, so a value left over from a sensor that has stopped
    // answering would keep an untrustworthy index looking trustworthy.
    if (haveBus && has(SENSOR_BUS_BME68X_IAQ)) {
        state_.airQualityAccuracy = bus->bme68x_iaq_accuracy;
    }
    if (!state_.iaq.valid()) {
        state_.airQualityAccuracy = 0;
    }
    out->air_quality_accuracy = state_.airQualityAccuracy;

    // --- External probe ----------------------------------------------------
    const bool haveProbe = probe != nullptr;
    state_.probeTemperature.update(dt_seconds, haveProbe ? probe->temperature : 0.0f, haveProbe);
    state_.probeHumidity.update(dt_seconds, haveProbe ? probe->humidity : 0.0f, haveProbe);
    exportChannel(state_.probeTemperature, SENSOR_SOURCE_SHT4X, &out->probe_temperature);
    exportChannel(state_.probeHumidity, SENSOR_SOURCE_SHT4X, &out->probe_humidity);

    // --- Trends ------------------------------------------------------------
    // Fed from the filtered values: a rate computed on raw readings is mostly
    // a measure of the sensor's own noise.
    state_.temperatureTrend.update(dt_seconds, out->temperature.value, out->temperature.valid);
    state_.humidityTrend.update(dt_seconds, out->humidity.value, out->humidity.valid);
    state_.co2Trend.update(dt_seconds, out->co2.value, out->co2.valid);
    state_.iaqTrend.update(dt_seconds, out->iaq.value, out->iaq.valid);

    out->trends.temperature = exportTrend(state_.temperatureTrend.ratePerMinute(),
                                          state_.temperatureTrend.ready(),
                                          out->temperature.valid);
    out->trends.humidity = exportTrend(state_.humidityTrend.ratePerMinute(),
                                       state_.humidityTrend.ready(), out->humidity.valid);
    out->trends.co2 =
        exportTrend(state_.co2Trend.ratePerMinute(), state_.co2Trend.ready(), out->co2.valid);
    out->trends.air_quality =
        exportTrend(state_.iaqTrend.ratePerMinute(), state_.iaqTrend.ready(), out->iaq.valid);

    // --- Events ------------------------------------------------------------
    // The fire detector reads the same filtered value everything else does,
    // which costs roughly one filter time constant of lag on a step — about 2 K
    // at the 4 K/min alarm threshold. That is the right trade here: a detector
    // fed raw readings would fire on a single corrupted I2C transfer, and this
    // alarm's whole design problem is false positives, not milliseconds.
    const auto fire = state_.fire.update(
        fu::FireDetectorInputs{
            .temperatureC = out->temperature.valid ? out->temperature.value : 0.0f,
            .temperatureValid = out->temperature.valid,
            .temperatureRateKPerMin = out->trends.temperature.per_minute,
            .airQualityRatePerMin = out->trends.air_quality.per_minute,
            .airQualityValid = out->trends.air_quality.valid && out->air_quality_accuracy > 0,
        },
        dt_seconds);
    out->events.fire_alarm = fire.alarm;
    out->events.fire_pre_alarm = fire.preAlarm;
    out->events.fire_reason_mask = fire.reasonMask;

    if (config.co2_occupancy_enabled) {
        const auto occupancy = state_.occupancy.update(
            out->co2.value, out->co2.valid, out->trends.co2.per_minute, dt_seconds);
        out->events.occupancy_detected = occupancy.occupied;
        out->events.estimated_occupants = occupancy.estimatedOccupants;
        out->events.co2_baseline_ppm = occupancy.baselinePpm;
    } else {
        out->events.occupancy_detected = false;
        out->events.estimated_occupants = 0;
        out->events.co2_baseline_ppm = 0.0f;
    }

    const auto window = state_.window.update(out->trends.temperature.per_minute,
                                              out->trends.temperature.valid,
                                              out->trends.co2.per_minute, out->trends.co2.valid,
                                              dt_seconds);
    out->events.window_open_detected = config.window_detect_enabled && window.windowOpen;

    // --- Health ------------------------------------------------------------
    // Every channel is attributed to the package that produced it, so a part
    // that has lost one of its measurands stops counting as healthy while still
    // counting as present.
    std::array<PackageHealth, SENSOR_SOURCE_COUNT> packages{};
    for (size_t i = 0; i < fu::kMaxSources; ++i) {
        packages[kTemperatureSources[i]].observe(state_.temperature.source(i));
        packages[kHumiditySources[i]].observe(state_.humidity.source(i));
    }
    packages[SENSOR_SOURCE_BME68X].observe(state_.pressure);
    packages[SENSOR_SOURCE_BME68X].observe(state_.iaq);
    packages[SENSOR_SOURCE_BME68X].observe(state_.co2Equivalent);
    packages[SENSOR_SOURCE_BME68X].observe(state_.vocEquivalent);
    packages[SENSOR_SOURCE_SCD4X].observe(state_.co2);
    packages[SENSOR_SOURCE_SHT4X].observe(state_.probeTemperature);
    packages[SENSOR_SOURCE_SHT4X].observe(state_.probeHumidity);

    uint8_t present = 0;
    uint8_t healthy = 0;
    for (size_t i = 0; i < static_cast<size_t>(SENSOR_SOURCE_COUNT); ++i) {
        if (!packages[i].everSeen) {
            continue;  // not fitted, or has never answered: neither present nor sick
        }
        present |= SENSOR_SOURCE_BIT(i);
        if (packages[i].healthy) {
            healthy |= SENSOR_SOURCE_BIT(i);
        }
    }
    const uint8_t suspect = out->temperature.suspect_mask | out->humidity.suspect_mask;

    ++state_.sampleCount;
    if (haveBus && bus->error_count > 0) {
        ++state_.errorCount;
    }

    out->health.present_mask = present;
    out->health.healthy_mask = healthy;
    out->health.suspect_mask = suspect;
    out->health.sample_count = state_.sampleCount;
    out->health.error_count = state_.errorCount;
}

} // namespace fusion
} // namespace habinari
