// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

/**
 * @file sensor_fusion.hpp
 * @brief Signal conditioning, redundancy and event detection for the board's
 *        overlapping sensor package.
 *
 * The board carries four sensors whose measurands overlap heavily:
 *
 *   HDC3020  temperature, humidity          (the reference room air sensor)
 *   BME688   temperature, humidity, pressure, gas/IAQ
 *   SCD4x    CO2, and temperature/humidity as a by-product of its compensation
 *   SHT4x    temperature, humidity          (optional in-slab / remote probe)
 *
 * Before this layer existed the extra sources were simply discarded: the room
 * temperature was whatever the HDC3020 last returned, unfiltered, and a failed
 * HDC3020 meant the controller lost its input entirely even though two other
 * parts in the same enclosure were still measuring the same air. What is here:
 *
 *   1. Per-source conditioning (FilteredChannel) — plausibility window, spike
 *      gate, exponential smoothing and staleness, so one bad I2C transaction
 *      cannot move a control loop.
 *   2. Redundancy and validation (RedundantMeasurement) — priority fallback
 *      when a source dies, and cross-comparison between sources that are all
 *      alive, which is the only way a slowly drifting sensor is ever noticed.
 *   3. Trends (TrendMonitor) — least-squares rate of change, which turns the
 *      same readings into events: fire/rapid temperature rise, CO2-derived
 *      occupancy, and open-window detection.
 *
 * Deliberately free of ESP-IDF and of the KNX stack: everything here is plain
 * C++23 on floats and is covered by main/test/test_sensor_fusion.cpp. The
 * ESP-side glue that feeds it lives in main/src/sensor_fusion_service.cpp.
 */

namespace habinari {
namespace fusion {

inline float clampf(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

// ---------------------------------------------------------------------------
// Per-source signal conditioning
// ---------------------------------------------------------------------------

/**
 * Conditioning for one measurand from one physical sensor.
 *
 * The three guards are deliberately different in kind, because the failure
 * modes are: a sensor reporting nonsense (range), a corrupted single transfer
 * (spike), and a sensor that has stopped answering (staleness).
 */
struct ChannelConfig {
    // Physically possible range for this measurand. Readings outside it are
    // rejected outright — they are a driver or wiring fault, never weather.
    float minValid{-100.0f};
    float maxValid{200.0f};
    // Largest believable change per second. A step larger than this is treated
    // as a spike and dropped, but only maxConsecutiveRejects times in a row:
    // after that the sensor is believed and the filter re-syncs, so a genuine
    // fast change (or a sensor reset) is not suppressed forever.
    float maxStepPerSecond{0.0f};  // 0 disables the spike gate
    // Exponential-smoothing time constant. This is the oversampling knob: the
    // sampler runs several times faster than the publish rate and the noise is
    // averaged out here rather than by publishing raw values less often.
    float filterTauSeconds{0.0f};  // 0 = pass through unfiltered
    // A source with no accepted sample for this long is no longer valid, which
    // is what makes the redundancy layer fall through to the next source.
    float staleAfterSeconds{120.0f};
    uint8_t maxConsecutiveRejects{3};
};

class FilteredChannel {
public:
    void configure(const ChannelConfig &config) { config_ = config; }

    const ChannelConfig &config() const { return config_; }

    /**
     * Advance by dtSeconds, optionally with a fresh raw sample.
     *
     * @param raw     the new reading, ignored when hasSample is false
     * @returns true when the sample was accepted into the filtered value
     */
    bool update(float dtSeconds, float raw, bool hasSample)
    {
        if (dtSeconds > 0.0f) {
            ageSeconds_ += dtSeconds;
        }

        if (hasSample) {
            if (accept(raw, dtSeconds)) {
                ++acceptedSamples_;
                consecutiveRejects_ = 0;
                ageSeconds_ = 0.0f;
                return true;
            }
            ++rejectedSamples_;
            if (consecutiveRejects_ < 0xFFu) {
                ++consecutiveRejects_;
            }
        }

        if (valid_ && config_.staleAfterSeconds > 0.0f
            && ageSeconds_ > config_.staleAfterSeconds) {
            valid_ = false;
        }
        return false;
    }

    // Filtered value. Only meaningful while valid().
    float value() const { return filtered_; }
    // Last accepted reading before smoothing — what an alarm path that must not
    // lag (fire detection) should look at alongside the filtered trend.
    float rawValue() const { return lastRaw_; }
    bool valid() const { return valid_; }
    float ageSeconds() const { return ageSeconds_; }
    uint32_t acceptedSamples() const { return acceptedSamples_; }
    uint32_t rejectedSamples() const { return rejectedSamples_; }
    // True once the sensor has answered at least once since the last reset,
    // which separates "not fitted" from "fitted and broken".
    bool everSeen() const { return acceptedSamples_ > 0; }

    void reset()
    {
        valid_ = false;
        filtered_ = 0.0f;
        lastRaw_ = 0.0f;
        ageSeconds_ = 0.0f;
        consecutiveRejects_ = 0;
        acceptedSamples_ = 0;
        rejectedSamples_ = 0;
    }

private:
    bool accept(float raw, float dtSeconds)
    {
        if (!std::isfinite(raw) || raw < config_.minValid || raw > config_.maxValid) {
            return false;
        }

        // Once the gate has rejected its quota in a row, the sensor is telling
        // us the same "impossible" thing repeatedly and is more likely right
        // than the filter is. Believe it AND drop the smoothing history: a
        // resync that then crawls to the new value over a filter time constant
        // is the same lag the gate was supposed to avoid.
        const bool resync = valid_ && config_.maxStepPerSecond > 0.0f
                            && consecutiveRejects_ >= config_.maxConsecutiveRejects;
        const bool spikeGateArmed = valid_ && config_.maxStepPerSecond > 0.0f && !resync;
        if (spikeGateArmed) {
            // Age, not dt: after a stale gap the allowance has to cover the
            // whole gap, otherwise every resumed source looks like a spike.
            const float elapsed = ageSeconds_ > dtSeconds ? ageSeconds_ : dtSeconds;
            const float allowance = config_.maxStepPerSecond * (elapsed > 0.0f ? elapsed : 1.0f);
            if (std::fabs(raw - filtered_) > allowance) {
                return false;
            }
        }

        lastRaw_ = raw;
        if (!valid_ || resync || config_.filterTauSeconds <= 0.0f || dtSeconds <= 0.0f) {
            filtered_ = raw;  // first sample, or filtering disabled: adopt it
        } else {
            const float alpha = dtSeconds / (config_.filterTauSeconds + dtSeconds);
            filtered_ += alpha * (raw - filtered_);
        }
        valid_ = true;
        return true;
    }

    ChannelConfig config_{};
    float filtered_{0.0f};
    float lastRaw_{0.0f};
    float ageSeconds_{0.0f};
    uint32_t acceptedSamples_{0};
    uint32_t rejectedSamples_{0};
    uint8_t consecutiveRejects_{0};
    bool valid_{false};
};

// ---------------------------------------------------------------------------
// Redundancy and cross-validation
// ---------------------------------------------------------------------------

inline constexpr size_t kMaxSources = 3;

// One raw reading offered to a redundant measurand this cycle. `present` is
// false when the driver produced nothing (sensor absent, bus error, or simply
// no new conversion yet), which is different from producing a bad value.
struct SourceSample {
    float value{0.0f};
    bool present{false};
};

struct RedundancyConfig {
    // Sources that disagree by more than this are cross-flagged. Set per
    // measurand: 1.5 K between two temperature sensors in one enclosure is
    // already suspicious, 5 %RH between two humidity sensors is not.
    float crossCheckTolerance{0.0f};  // 0 disables validation
    // Per-source systematic correction, applied before comparison. The BME688
    // and SCD4x both sit next to their own heaters and read consistently high;
    // without this the cross-check would fire permanently on healthy parts.
    std::array<float, kMaxSources> sourceOffsets{};
};

enum class FusionQuality : uint8_t {
    NoData = 0,     // nothing usable
    Single = 1,     // one source alive, nothing to validate against
    Validated = 2,  // several sources alive and agreeing
    Disputed = 3,   // several sources alive and disagreeing
};

struct FusedValue {
    float value{0.0f};
    bool valid{false};
    // Which source the published value came from, in priority order.
    uint8_t sourceIndex{0};
    // The preferred (index 0) source is not the one being used.
    bool usingFallback{false};
    // At least one healthy pair differs by more than the tolerance.
    bool disagreement{false};
    // Bitmask of sources currently outside the agreed value by more than the
    // tolerance — the drift report an installer can act on.
    uint8_t suspectMask{0};
    uint8_t healthyCount{0};
    FusionQuality quality{FusionQuality::NoData};
};

/**
 * A measurand backed by up to kMaxSources physical sensors in priority order.
 *
 * Selection rules, in order:
 *   - no healthy source            -> invalid, last value held for reference
 *   - one healthy source           -> use it (nothing to validate against)
 *   - two healthy sources          -> use the preferred one; if they disagree
 *                                     say so, because with two sensors there is
 *                                     no way to tell which one is lying
 *   - three healthy sources        -> use the median, which drops an outlier
 *                                     automatically, and name the outlier
 *
 * The median rule is what makes the third source worth having: with an odd
 * number of sensors a single drifting part stops affecting the output at all
 * instead of being averaged into it.
 */
class RedundantMeasurement {
public:
    void configureSource(size_t index, const ChannelConfig &config)
    {
        if (index < kMaxSources) {
            channels_[index].configure(config);
        }
    }

    void configure(const RedundancyConfig &config) { config_ = config; }

    const FilteredChannel &source(size_t index) const { return channels_[index]; }

    FusedValue update(float dtSeconds, std::span<const SourceSample> samples)
    {
        for (size_t i = 0; i < kMaxSources; ++i) {
            const bool has = i < samples.size() && samples[i].present;
            const float raw = has ? samples[i].value + config_.sourceOffsets[i] : 0.0f;
            channels_[i].update(dtSeconds, raw, has);
        }

        std::array<size_t, kMaxSources> healthy{};
        size_t healthyCount = 0;
        for (size_t i = 0; i < kMaxSources; ++i) {
            if (channels_[i].valid()) {
                healthy[healthyCount++] = i;
            }
        }

        FusedValue out;
        out.healthyCount = static_cast<uint8_t>(healthyCount);
        if (healthyCount == 0) {
            out.value = last_.value;  // held for display; valid stays false
            out.quality = FusionQuality::NoData;
            last_.valid = false;
            last_.quality = FusionQuality::NoData;
            return out;
        }

        // Cross-comparison first: the decision below depends on who agrees.
        const float tolerance = config_.crossCheckTolerance;
        if (tolerance > 0.0f && healthyCount >= 2) {
            for (size_t a = 0; a < healthyCount; ++a) {
                for (size_t b = a + 1; b < healthyCount; ++b) {
                    const float delta = std::fabs(channels_[healthy[a]].value()
                                                  - channels_[healthy[b]].value());
                    if (delta > tolerance) {
                        out.disagreement = true;
                    }
                }
            }
        }

        size_t chosen = healthy[0];
        if (healthyCount >= 3) {
            chosen = middleOfThree(healthy[0], healthy[1], healthy[2]);
        }

        out.value = channels_[chosen].value();
        out.valid = true;
        out.sourceIndex = static_cast<uint8_t>(chosen);
        out.usingFallback = (chosen != 0) || !channels_[0].valid();

        if (tolerance > 0.0f && healthyCount >= 2) {
            for (size_t i = 0; i < healthyCount; ++i) {
                const size_t index = healthy[i];
                if (std::fabs(channels_[index].value() - out.value) > tolerance) {
                    out.suspectMask |= static_cast<uint8_t>(1u << index);
                }
            }
        }

        out.quality = healthyCount == 1 ? FusionQuality::Single
                      : out.disagreement ? FusionQuality::Disputed
                                         : FusionQuality::Validated;
        last_ = out;
        return out;
    }

    void reset()
    {
        for (auto &channel : channels_) {
            channel.reset();
        }
        last_ = FusedValue{};
    }

private:
    // kMaxSources is 3, so an explicit middle-of-three is clearer and cheaper
    // than sorting. Ties resolve towards the higher-priority source.
    size_t middleOfThree(size_t a, size_t b, size_t c) const
    {
        const float va = channels_[a].value();
        const float vb = channels_[b].value();
        const float vc = channels_[c].value();
        if ((va <= vb && vb <= vc) || (vc <= vb && vb <= va)) return b;
        if ((vb <= va && va <= vc) || (vc <= va && va <= vb)) return a;
        return c;
    }

    std::array<FilteredChannel, kMaxSources> channels_{};
    RedundancyConfig config_{};
    FusedValue last_{};
};

// ---------------------------------------------------------------------------
// Trends
// ---------------------------------------------------------------------------

/**
 * Least-squares rate of change over a sliding window.
 *
 * Endpoint differencing would be a line shorter but rides on whatever noise
 * happens to be on the two samples it picks; a regression over the whole window
 * is what makes a 4 K/min fire threshold safe to act on.
 */
template <size_t N>
class TrendMonitor {
public:
    static_assert(N >= 3, "a rate needs at least three samples to mean anything");

    void update(float dtSeconds, float value, bool valid)
    {
        if (!valid) {
            reset();
            return;
        }
        elapsed_ += dtSeconds;
        times_[head_] = elapsed_;
        values_[head_] = value;
        head_ = (head_ + 1) % N;
        if (count_ < N) {
            ++count_;
        }
    }

    bool ready() const { return count_ >= 3 && spanSeconds() > 0.0f; }

    // Slope in units per second. Zero until the window has filled enough to
    // mean anything, so callers never act on a two-sample "trend".
    float ratePerSecond() const
    {
        if (!ready()) {
            return 0.0f;
        }
        float sumT = 0.0f, sumV = 0.0f, sumTT = 0.0f, sumTV = 0.0f;
        for (size_t i = 0; i < count_; ++i) {
            const size_t slot = (head_ + N - count_ + i) % N;
            const float t = times_[slot];
            const float v = values_[slot];
            sumT += t;
            sumV += v;
            sumTT += t * t;
            sumTV += t * v;
        }
        const float n = static_cast<float>(count_);
        const float denominator = n * sumTT - sumT * sumT;
        if (std::fabs(denominator) < 1e-6f) {
            return 0.0f;
        }
        return (n * sumTV - sumT * sumV) / denominator;
    }

    float ratePerMinute() const { return ratePerSecond() * 60.0f; }
    float ratePerHour() const { return ratePerSecond() * 3600.0f; }

    float spanSeconds() const
    {
        if (count_ < 2) {
            return 0.0f;
        }
        const size_t oldest = (head_ + N - count_) % N;
        const size_t newest = (head_ + N - 1) % N;
        return times_[newest] - times_[oldest];
    }

    void reset()
    {
        count_ = 0;
        head_ = 0;
        elapsed_ = 0.0f;
    }

private:
    std::array<float, N> times_{};
    std::array<float, N> values_{};
    size_t head_{0};
    size_t count_{0};
    float elapsed_{0.0f};
};

// ---------------------------------------------------------------------------
// Fire / rapid temperature rise
// ---------------------------------------------------------------------------

/**
 * Rate-of-rise and fixed-temperature heat detection, modelled on the two
 * behaviours EN 54-5 defines for heat detectors, plus an air-quality
 * corroboration path the standard heat detector does not have.
 *
 * This is explicitly NOT a certified fire detector and must not replace one:
 * an air-quality board in a room corner is in the wrong place and has the wrong
 * response time for life safety. What it is worth is an early, corroborated
 * hint on a bus that is already wired — a runaway heater, a hob left on, a
 * fault in the very underfloor circuit this board controls — reported minutes
 * before anyone smells it, and cheap because the readings already exist.
 *
 * False alarms are the entire design problem. Three defences:
 *   - a confirmation time, so a hairdryer or a sunbeam crossing the sensor has
 *     to sustain the rise rather than merely cause it,
 *   - optional corroboration from the BME688 VOC signal, which is what actually
 *     separates combustion from a hot appliance,
 *   - a clear time before the latch drops, so the alarm does not chatter.
 */
struct FireDetectorConfig {
    float rateOfRiseKPerMin{4.0f};   // 0 disables the rate path
    float absoluteAlarmC{55.0f};     // 0 disables the fixed-temperature path
    float confirmSeconds{30.0f};     // rise must persist this long
    float clearSeconds{300.0f};      // and be gone this long before resetting
    // When set, a rate-of-rise alarm additionally needs the air-quality signal
    // to be climbing. Combustion always produces volatiles; a fan heater does
    // not. Off by default because it needs a calibrated BSEC (accuracy > 0).
    bool requireAirQualityRise{false};
    float vocRiseIndexPerMin{20.0f};
    // Minimum temperature before the rate path may fire at all. A 4 K/min rise
    // from 12 °C is a room warming up after a night setback, not a fire.
    float rateArmAboveC{28.0f};
};

enum class FireReasonBit : uint8_t {
    RateOfRise = 1 << 0,
    AbsoluteTemperature = 1 << 1,
    AirQualityRise = 1 << 2,
};

struct FireDetectorInputs {
    float temperatureC{0.0f};
    bool temperatureValid{false};
    float temperatureRateKPerMin{0.0f};
    float airQualityRatePerMin{0.0f};
    bool airQualityValid{false};
};

struct FireDetectorOutputs {
    bool alarm{false};
    // Condition present but not yet confirmed — worth a log line and a status
    // bit, not worth waking anyone.
    bool preAlarm{false};
    uint8_t reasonMask{0};
    float confirmProgressSeconds{0.0f};
};

class FireDetector {
public:
    void configure(const FireDetectorConfig &config) { config_ = config; }

    FireDetectorOutputs update(const FireDetectorInputs &in, float dtSeconds)
    {
        FireDetectorOutputs out;
        if (!in.temperatureValid) {
            // No temperature means no opinion: hold the latch, stop counting.
            out.alarm = alarmLatched_;
            out.confirmProgressSeconds = confirmTimer_;
            return out;
        }

        const bool airQualityRising = in.airQualityValid
                                      && config_.vocRiseIndexPerMin > 0.0f
                                      && in.airQualityRatePerMin >= config_.vocRiseIndexPerMin;

        bool rateHit = config_.rateOfRiseKPerMin > 0.0f
                       && in.temperatureRateKPerMin >= config_.rateOfRiseKPerMin
                       && in.temperatureC >= config_.rateArmAboveC;
        if (rateHit && config_.requireAirQualityRise && !airQualityRising) {
            rateHit = false;
        }

        const bool absoluteHit =
            config_.absoluteAlarmC > 0.0f && in.temperatureC >= config_.absoluteAlarmC;

        uint8_t reason = 0;
        if (rateHit) reason |= static_cast<uint8_t>(FireReasonBit::RateOfRise);
        if (absoluteHit) reason |= static_cast<uint8_t>(FireReasonBit::AbsoluteTemperature);
        if (airQualityRising) reason |= static_cast<uint8_t>(FireReasonBit::AirQualityRise);

        const bool conditionPresent = rateHit || absoluteHit;
        if (conditionPresent) {
            confirmTimer_ += dtSeconds;
            clearTimer_ = 0.0f;
            if (confirmTimer_ >= config_.confirmSeconds) {
                alarmLatched_ = true;
                latchedReason_ = reason;
            }
        } else {
            confirmTimer_ = 0.0f;
            if (alarmLatched_) {
                clearTimer_ += dtSeconds;
                if (clearTimer_ >= config_.clearSeconds) {
                    alarmLatched_ = false;
                    latchedReason_ = 0;
                    clearTimer_ = 0.0f;
                }
            }
        }

        out.alarm = alarmLatched_;
        out.preAlarm = conditionPresent && !alarmLatched_;
        out.reasonMask = alarmLatched_ ? static_cast<uint8_t>(latchedReason_ | reason) : reason;
        out.confirmProgressSeconds = confirmTimer_;
        return out;
    }

    // Manual acknowledge, e.g. from the programming button or a bus command.
    void acknowledge()
    {
        alarmLatched_ = false;
        latchedReason_ = 0;
        confirmTimer_ = 0.0f;
        clearTimer_ = 0.0f;
    }

private:
    FireDetectorConfig config_{};
    float confirmTimer_{0.0f};
    float clearTimer_{0.0f};
    uint8_t latchedReason_{0};
    bool alarmLatched_{false};
};

// ---------------------------------------------------------------------------
// CO2-derived occupancy
// ---------------------------------------------------------------------------

/**
 * Occupancy from the CO2 signal.
 *
 * People are the only meaningful indoor CO2 source, so a sustained rise above
 * the decayed outdoor baseline means the room is occupied and a decay towards
 * that baseline means it is not. Unlike a PIR this does not miss someone
 * sitting still — which is exactly the case where a PIR-driven setback wrongly
 * drops the room to standby. It feeds the RTC presence input when no PIR is
 * linked to the bus, and can validate one that is.
 *
 * The baseline tracks the lowest recent CO2 (typically the small hours) rather
 * than assuming 400 ppm, so the estimate survives a leaky sensor calibration
 * and a naturally high-CO2 site.
 */
struct OccupancyConfig {
    // Rise above the tracked baseline that counts as occupied.
    float occupiedDeltaPpm{150.0f};
    // Hysteresis on the way down, so a quiet room does not flicker.
    float vacatedDeltaPpm{75.0f};
    // A clear upward slope is evidence on its own, before the absolute level
    // has had time to build.
    float risingPpmPerMin{15.0f};
    // How fast the baseline may follow the signal, ppm per hour. Slow enough
    // that a full working day of occupancy cannot drag it up.
    float baselineTrackingPpmPerHour{20.0f};
    float confirmSeconds{120.0f};
    float vacateSeconds{900.0f};  // CO2 decays slowly; do not release early
};

struct OccupancyOutputs {
    bool occupied{false};
    float baselinePpm{0.0f};
    float excessPpm{0.0f};
    // Rough headcount from the excess, for a visualisation. One adult at rest
    // in a normally ventilated room lifts CO2 by roughly 150 ppm; this is an
    // order-of-magnitude indication and is never used for control.
    uint8_t estimatedOccupants{0};
};

class OccupancyEstimator {
public:
    void configure(const OccupancyConfig &config) { config_ = config; }

    OccupancyOutputs update(float co2Ppm, bool valid, float ratePpmPerMin, float dtSeconds)
    {
        OccupancyOutputs out;
        if (!valid) {
            out.occupied = occupied_;
            out.baselinePpm = baseline_;
            return out;
        }

        if (!baselineInitialised_) {
            baseline_ = co2Ppm;
            baselineInitialised_ = true;
        } else {
            const float maxStep = config_.baselineTrackingPpmPerHour * (dtSeconds / 3600.0f);
            if (co2Ppm < baseline_) {
                // Fall to a new low quickly: that low IS the fresh-air level.
                baseline_ = co2Ppm;
            } else {
                baseline_ = baseline_ + maxStep;  // creep up, capped
                if (baseline_ > co2Ppm) {
                    baseline_ = co2Ppm;
                }
            }
        }

        const float excess = co2Ppm - baseline_;
        const bool occupiedEvidence = excess >= config_.occupiedDeltaPpm
                                      || ratePpmPerMin >= config_.risingPpmPerMin;
        const bool vacatedEvidence = excess <= config_.vacatedDeltaPpm && ratePpmPerMin <= 0.0f;

        if (!occupied_ && occupiedEvidence) {
            confirmTimer_ += dtSeconds;
            vacateTimer_ = 0.0f;
            if (confirmTimer_ >= config_.confirmSeconds) {
                occupied_ = true;
                confirmTimer_ = 0.0f;
            }
        } else if (occupied_ && vacatedEvidence) {
            vacateTimer_ += dtSeconds;
            confirmTimer_ = 0.0f;
            if (vacateTimer_ >= config_.vacateSeconds) {
                occupied_ = false;
                vacateTimer_ = 0.0f;
            }
        } else {
            confirmTimer_ = 0.0f;
            vacateTimer_ = 0.0f;
        }

        out.occupied = occupied_;
        out.baselinePpm = baseline_;
        out.excessPpm = excess > 0.0f ? excess : 0.0f;
        const float perPerson = config_.occupiedDeltaPpm > 0.0f ? config_.occupiedDeltaPpm : 150.0f;
        const float people = out.excessPpm / perPerson;
        out.estimatedOccupants = static_cast<uint8_t>(clampf(people + 0.5f, 0.0f, 255.0f));
        return out;
    }

    void reset()
    {
        occupied_ = false;
        baselineInitialised_ = false;
        baseline_ = 0.0f;
        confirmTimer_ = 0.0f;
        vacateTimer_ = 0.0f;
    }

private:
    OccupancyConfig config_{};
    float baseline_{0.0f};
    float confirmTimer_{0.0f};
    float vacateTimer_{0.0f};
    bool occupied_{false};
    bool baselineInitialised_{false};
};

// ---------------------------------------------------------------------------
// Open-window detection
// ---------------------------------------------------------------------------

/**
 * Window-open inference from a sharp temperature fall, corroborated by CO2
 * falling at the same time.
 *
 * The KNX RTC already takes a WindowStatus input, but that object is only ever
 * linked in installations that bought contacts for every window. Where they
 * were not fitted, heating into an open window is the single most expensive
 * failure this device can quietly allow. Requiring the CO2 to drop as well is
 * what separates an open window from the heating simply being off: turning a
 * radiator off cools the room without ventilating it.
 */
struct WindowDetectConfig {
    float fallKPerMin{0.35f};      // 0 disables detection
    float co2FallPpmPerMin{15.0f}; // 0 = do not require CO2 corroboration
    float confirmSeconds{120.0f};
    float minOpenSeconds{300.0f};  // hold once detected, so it cannot chatter
    float closeKPerMin{0.05f};     // fall has clearly stopped
};

struct WindowDetectOutputs {
    bool windowOpen{false};
    bool corroboratedByCo2{false};
};

class WindowDetector {
public:
    void configure(const WindowDetectConfig &config) { config_ = config; }

    WindowDetectOutputs update(float temperatureRateKPerMin,
                               bool temperatureValid,
                               float co2RatePpmPerMin,
                               bool co2Valid,
                               float dtSeconds)
    {
        WindowDetectOutputs out;
        if (!temperatureValid || config_.fallKPerMin <= 0.0f) {
            out.windowOpen = open_;
            return out;
        }

        const bool falling = temperatureRateKPerMin <= -config_.fallKPerMin;
        const bool co2Falling = co2Valid && config_.co2FallPpmPerMin > 0.0f
                                && co2RatePpmPerMin <= -config_.co2FallPpmPerMin;
        const bool needsCo2 = config_.co2FallPpmPerMin > 0.0f && co2Valid;
        const bool evidence = falling && (!needsCo2 || co2Falling);

        if (!open_) {
            if (evidence) {
                confirmTimer_ += dtSeconds;
                if (confirmTimer_ >= config_.confirmSeconds) {
                    open_ = true;
                    openTimer_ = 0.0f;
                    corroborated_ = co2Falling;
                }
            } else {
                confirmTimer_ = 0.0f;
            }
        } else {
            openTimer_ += dtSeconds;
            const bool stopped = temperatureRateKPerMin > -config_.closeKPerMin;
            if (stopped && openTimer_ >= config_.minOpenSeconds) {
                open_ = false;
                confirmTimer_ = 0.0f;
                corroborated_ = false;
            }
        }

        out.windowOpen = open_;
        out.corroboratedByCo2 = corroborated_;
        return out;
    }

    void reset()
    {
        open_ = false;
        confirmTimer_ = 0.0f;
        openTimer_ = 0.0f;
        corroborated_ = false;
    }

private:
    WindowDetectConfig config_{};
    float confirmTimer_{0.0f};
    float openTimer_{0.0f};
    bool open_{false};
    bool corroborated_{false};
};

} // namespace fusion
} // namespace habinari
