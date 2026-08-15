#include "sensor_fusion_service.h"

#include "sensor_fusion.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <array>
#include <cstring>
#include <span>

// Glue between the raw acquisition layer (sensor_bus / ext_probe) and the
// platform-free fusion algorithms in sensor_fusion.hpp. The mapping from the
// board's four physical sensors onto redundant measurands lives here and
// nowhere else, so "which sensor backs the room temperature, and in what
// order?" is one table rather than a rule spread across the consumers.

namespace {

namespace fu = sensor_board::fusion;

static const char *TAG = "sensor_fusion";

// Priority order per measurand. Index 0 is the preferred part; the rest are
// fallbacks used only when the ones above them stop delivering.
//
// Room temperature/humidity: the HDC3020 is the reference part — it is the only
// one of the three not sitting next to its own heat source. The BME688 comes
// next (its BSEC output is heat-compensated), then the SCD4x, whose T/RH is a
// by-product of its own compensation and the least trustworthy of the three.
//
// The SHT4x probe is deliberately NOT in this list. It measures a different
// place (in-slab or remote), so promoting it to a room-air fallback would
// silently control the room from the floor temperature — a plausible-looking
// number that is simply the wrong quantity.
constexpr std::array<sensor_source_t, fu::kMaxSources> kTemperatureSources = {
    SENSOR_SOURCE_HDC302X, SENSOR_SOURCE_BME68X, SENSOR_SOURCE_SCD4X};
constexpr std::array<sensor_source_t, fu::kMaxSources> kHumiditySources = {
    SENSOR_SOURCE_HDC302X, SENSOR_SOURCE_BME68X, SENSOR_SOURCE_SCD4X};

// Trend windows. Long enough that the least-squares slope is not noise, short
// enough that a fire is not averaged away: 20 samples at the 5 s sampling
// interval is a 100 s window on temperature. CO2 and IAQ move slowly and their
// sources update every ~30 s and ~3 s respectively, so they get more samples.
using TemperatureTrend = fu::TrendMonitor<20>;
using SlowTrend = fu::TrendMonitor<16>;

struct FusionState {
    SemaphoreHandle_t configMutex{nullptr};
    sensor_fusion_config_t config{};
    bool configured{false};

    fu::RedundantMeasurement temperature;
    fu::RedundantMeasurement humidity;
    fu::FilteredChannel pressure;
    fu::FilteredChannel co2;
    fu::FilteredChannel iaq;
    fu::FilteredChannel co2Equivalent;
    fu::FilteredChannel vocEquivalent;
    fu::FilteredChannel probeTemperature;
    fu::FilteredChannel probeHumidity;

    TemperatureTrend temperatureTrend;
    SlowTrend humidityTrend;
    SlowTrend co2Trend;
    SlowTrend iaqTrend;

    fu::FireDetector fire;
    fu::OccupancyEstimator occupancy;
    fu::WindowDetector window;

    uint8_t everSeenMask{0};
    uint32_t sampleCount{0};
    uint32_t errorCount{0};
};

FusionState g_state;

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

void applyConfig(const sensor_fusion_config_t &config)
{
    const float tau = config.filter_tau_seconds;
    const float stale = config.stale_after_seconds;

    // Plausibility windows are the sensors' own datasheet ranges, widened
    // slightly: anything outside them is a driver fault, not a measurement.
    const auto temperatureChannel = channelConfig(-40.0f, 125.0f, 2.0f, tau, stale);
    const auto humidityChannel = channelConfig(0.0f, 100.0f, 5.0f, tau, stale);

    for (size_t i = 0; i < fu::kMaxSources; ++i) {
        g_state.temperature.configureSource(i, temperatureChannel);
        g_state.humidity.configureSource(i, humidityChannel);
    }

    fu::RedundancyConfig temperatureRedundancy;
    temperatureRedundancy.crossCheckTolerance = config.temperature_cross_check_k;
    temperatureRedundancy.sourceOffsets = {0.0f, config.bme68x_temperature_offset_k,
                                           config.scd4x_temperature_offset_k};
    g_state.temperature.configure(temperatureRedundancy);

    fu::RedundancyConfig humidityRedundancy;
    humidityRedundancy.crossCheckTolerance = config.humidity_cross_check_pct;
    humidityRedundancy.sourceOffsets = {0.0f, config.bme68x_humidity_offset_pct,
                                        config.scd4x_humidity_offset_pct};
    g_state.humidity.configure(humidityRedundancy);

    // Pressure changes slowly and matters to two decimal places for the
    // sea-level reduction, so it gets a long time constant. The staleness
    // allowance is generous for CO2: the SCD4x low-power cadence is 30 s.
    g_state.pressure.configure(channelConfig(30000.0f, 120000.0f, 500.0f, tau * 2.0f, stale));
    g_state.co2.configure(channelConfig(300.0f, 40000.0f, 200.0f, tau, stale * 2.0f));
    g_state.iaq.configure(channelConfig(0.0f, 500.0f, 50.0f, tau, stale * 2.0f));
    g_state.co2Equivalent.configure(channelConfig(300.0f, 60000.0f, 500.0f, tau, stale * 2.0f));
    g_state.vocEquivalent.configure(channelConfig(0.0f, 1000.0f, 50.0f, tau, stale * 2.0f));
    g_state.probeTemperature.configure(channelConfig(-40.0f, 125.0f, 2.0f, tau, stale));
    g_state.probeHumidity.configure(channelConfig(0.0f, 100.0f, 5.0f, tau, stale));

    fu::FireDetectorConfig fire;
    fire.rateOfRiseKPerMin = config.fire_rate_of_rise_k_per_min;
    fire.absoluteAlarmC = config.fire_absolute_alarm_c;
    fire.confirmSeconds = config.fire_confirm_seconds;
    fire.clearSeconds = config.fire_clear_seconds;
    fire.rateArmAboveC = config.fire_arm_above_c;
    fire.requireAirQualityRise = config.fire_require_air_quality;
    g_state.fire.configure(fire);

    fu::WindowDetectConfig window;
    window.fallKPerMin = config.window_detect_enabled ? config.window_fall_k_per_min : 0.0f;
    g_state.window.configure(window);

    g_state.occupancy.configure(fu::OccupancyConfig{});
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

extern "C" void sensor_fusion_configure(const sensor_fusion_config_t *config)
{
    if (config == nullptr) {
        return;
    }
    if (g_state.configMutex == nullptr) {
        g_state.configMutex = xSemaphoreCreateMutex();
    }
    if (g_state.configMutex != nullptr) {
        xSemaphoreTake(g_state.configMutex, portMAX_DELAY);
    }
    g_state.config = *config;
    g_state.configured = true;
    applyConfig(g_state.config);
    if (g_state.configMutex != nullptr) {
        xSemaphoreGive(g_state.configMutex);
    }
}

extern "C" void sensor_fusion_reset(void)
{
    g_state.temperature.reset();
    g_state.humidity.reset();
    g_state.pressure.reset();
    g_state.co2.reset();
    g_state.iaq.reset();
    g_state.co2Equivalent.reset();
    g_state.vocEquivalent.reset();
    g_state.probeTemperature.reset();
    g_state.probeHumidity.reset();
    g_state.temperatureTrend.reset();
    g_state.humidityTrend.reset();
    g_state.co2Trend.reset();
    g_state.iaqTrend.reset();
    g_state.occupancy.reset();
    g_state.window.reset();
    g_state.fire.acknowledge();
    g_state.everSeenMask = 0;
    g_state.sampleCount = 0;
    g_state.errorCount = 0;
}

extern "C" void sensor_fusion_acknowledge_alarms(void)
{
    g_state.fire.acknowledge();
}

extern "C" void sensor_fusion_update(const sensor_bus_results_t *bus,
                                     const ext_probe_results_t *probe,
                                     float dt_seconds,
                                     sensor_data_t *out)
{
    if (out == nullptr) {
        return;
    }
    if (!g_state.configured) {
        sensor_fusion_config_t defaults;
        sensor_fusion_default_config(&defaults);
        sensor_fusion_configure(&defaults);
    }

    sensor_fusion_config_t config;
    if (g_state.configMutex != nullptr) {
        xSemaphoreTake(g_state.configMutex, portMAX_DELAY);
        config = g_state.config;
        xSemaphoreGive(g_state.configMutex);
    } else {
        config = g_state.config;
    }

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

    const auto temperature = g_state.temperature.update(
        dt_seconds, std::span<const fu::SourceSample>(temperatureSamples));
    const auto humidity =
        g_state.humidity.update(dt_seconds, std::span<const fu::SourceSample>(humiditySamples));

    exportValue(temperature, std::span<const sensor_source_t>(kTemperatureSources),
                &out->temperature);
    exportValue(humidity, std::span<const sensor_source_t>(kHumiditySources), &out->humidity);

    // --- Single-source measurands -----------------------------------------
    g_state.pressure.update(dt_seconds, haveBus ? bus->bme68x_pressure : 0.0f,
                            has(SENSOR_BUS_BME68X_PRESSURE));
    g_state.co2.update(dt_seconds, haveBus ? static_cast<float>(bus->scd4x_co2) : 0.0f,
                       has(SENSOR_BUS_SCD4X_CO2));
    g_state.iaq.update(dt_seconds, haveBus ? bus->bme68x_iaq : 0.0f, has(SENSOR_BUS_BME68X_IAQ));
    g_state.co2Equivalent.update(dt_seconds, haveBus ? bus->bme68x_co2_equivalent : 0.0f,
                                 has(SENSOR_BUS_BME68X_CO2_EQUIVALENT));
    g_state.vocEquivalent.update(dt_seconds, haveBus ? bus->bme68x_voc_equivalent : 0.0f,
                                 has(SENSOR_BUS_BME68X_VOC_EQUIVALENT));

    exportChannel(g_state.pressure, SENSOR_SOURCE_BME68X, &out->pressure);
    exportChannel(g_state.co2, SENSOR_SOURCE_SCD4X, &out->co2);
    exportChannel(g_state.iaq, SENSOR_SOURCE_BME68X, &out->iaq);
    exportChannel(g_state.co2Equivalent, SENSOR_SOURCE_BME68X, &out->co2_equivalent);
    exportChannel(g_state.vocEquivalent, SENSOR_SOURCE_BME68X, &out->voc_equivalent);
    out->air_quality_accuracy = haveBus ? bus->bme68x_iaq_accuracy : 0;

    // --- External probe ----------------------------------------------------
    const bool haveProbe = probe != nullptr;
    g_state.probeTemperature.update(dt_seconds, haveProbe ? probe->temperature : 0.0f, haveProbe);
    g_state.probeHumidity.update(dt_seconds, haveProbe ? probe->humidity : 0.0f, haveProbe);
    exportChannel(g_state.probeTemperature, SENSOR_SOURCE_SHT4X, &out->probe_temperature);
    exportChannel(g_state.probeHumidity, SENSOR_SOURCE_SHT4X, &out->probe_humidity);

    // --- Trends ------------------------------------------------------------
    // Fed from the filtered values: a rate computed on raw readings is mostly
    // a measure of the sensor's own noise.
    g_state.temperatureTrend.update(dt_seconds, out->temperature.value, out->temperature.valid);
    g_state.humidityTrend.update(dt_seconds, out->humidity.value, out->humidity.valid);
    g_state.co2Trend.update(dt_seconds, out->co2.value, out->co2.valid);
    g_state.iaqTrend.update(dt_seconds, out->iaq.value, out->iaq.valid);

    out->trends.temperature = exportTrend(g_state.temperatureTrend.ratePerMinute(),
                                          g_state.temperatureTrend.ready(),
                                          out->temperature.valid);
    out->trends.humidity = exportTrend(g_state.humidityTrend.ratePerMinute(),
                                       g_state.humidityTrend.ready(), out->humidity.valid);
    out->trends.co2 =
        exportTrend(g_state.co2Trend.ratePerMinute(), g_state.co2Trend.ready(), out->co2.valid);
    out->trends.air_quality =
        exportTrend(g_state.iaqTrend.ratePerMinute(), g_state.iaqTrend.ready(), out->iaq.valid);

    // --- Events ------------------------------------------------------------
    // The fire detector reads the same filtered value everything else does,
    // which costs roughly one filter time constant of lag on a step — about 2 K
    // at the 4 K/min alarm threshold. That is the right trade here: a detector
    // fed raw readings would fire on a single corrupted I2C transfer, and this
    // alarm's whole design problem is false positives, not milliseconds.
    const auto fire = g_state.fire.update(
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
        const auto occupancy = g_state.occupancy.update(
            out->co2.value, out->co2.valid, out->trends.co2.per_minute, dt_seconds);
        out->events.occupancy_detected = occupancy.occupied;
        out->events.estimated_occupants = occupancy.estimatedOccupants;
        out->events.co2_baseline_ppm = occupancy.baselinePpm;
    } else {
        out->events.occupancy_detected = false;
        out->events.estimated_occupants = 0;
        out->events.co2_baseline_ppm = 0.0f;
    }

    const auto window = g_state.window.update(out->trends.temperature.per_minute,
                                              out->trends.temperature.valid,
                                              out->trends.co2.per_minute, out->trends.co2.valid,
                                              dt_seconds);
    out->events.window_open_detected = config.window_detect_enabled && window.windowOpen;

    // --- Health ------------------------------------------------------------
    uint8_t healthy = 0;
    uint8_t suspect = out->temperature.suspect_mask | out->humidity.suspect_mask;
    for (size_t i = 0; i < fu::kMaxSources; ++i) {
        if (g_state.temperature.source(i).valid() || g_state.humidity.source(i).valid()) {
            healthy |= SENSOR_SOURCE_BIT(kTemperatureSources[i]);
        }
    }
    if (g_state.pressure.valid() || g_state.iaq.valid()) {
        healthy |= SENSOR_SOURCE_BIT(SENSOR_SOURCE_BME68X);
    }
    if (g_state.co2.valid()) {
        healthy |= SENSOR_SOURCE_BIT(SENSOR_SOURCE_SCD4X);
    }
    if (g_state.probeTemperature.valid() || g_state.probeHumidity.valid()) {
        healthy |= SENSOR_SOURCE_BIT(SENSOR_SOURCE_SHT4X);
    }
    g_state.everSeenMask |= healthy;

    ++g_state.sampleCount;
    if (haveBus && bus->error_count > 0) {
        ++g_state.errorCount;
    }

    out->health.present_mask = g_state.everSeenMask;
    out->health.healthy_mask = healthy;
    out->health.suspect_mask = suspect;
    out->health.sample_count = g_state.sampleCount;
    out->health.error_count = g_state.errorCount;

    // One line per cycle at Debug; the fault transitions that matter are logged
    // by the KNX diagnostics objects, not here.
    ESP_LOGD(TAG,
             "T=%.2f(src%u%s) RH=%.2f(src%u) CO2=%.0f dT=%.2fK/min healthy=0x%02x suspect=0x%02x",
             out->temperature.value, out->temperature.source,
             out->temperature.disagreement ? " !" : "", out->humidity.value, out->humidity.source,
             out->co2.value, out->trends.temperature.per_minute, healthy, suspect);
}
