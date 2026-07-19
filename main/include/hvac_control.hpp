/**
 * @file hvac_control.hpp
 * @brief Device-side room-control logic: PID heating/cooling, two-point
 *        requests with hysteresis, floor-temperature limit, air-quality boost.
 *
 * This is application logic, deliberately NOT part of KNstaX: nothing here is
 * KNX-specific. The KNX stack supplies the ports (DPT 5.001 control values,
 * switch requests) and the ETS parameters that configure these controllers;
 * the control engineering lives with the device firmware.
 *
 * Everything is pure and platform-free (no ESP-IDF, no clock, no allocation):
 * callers pass measurements and a dt, so the logic is host-testable.
 */

#pragma once

#include <cstdint>

namespace sensor_board {
namespace hvac {

inline float clampf(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

// ---------------------------------------------------------------------------
// PID controller producing a 0..100 % control value (KNX DPT 5.001 semantics).
// ---------------------------------------------------------------------------

struct PidConfig {
    // Proportional gain in %/K. KNX room controllers usually parameterise a
    // proportional band instead: kp = 100 / band (4 K band → 25 %/K).
    float kp{25.0f};
    // Integral time in seconds (KNX convention: reset time). 0 disables the
    // I term. Default 9000 s = 150 min, a common radiator-heating value.
    float tiSeconds{9000.0f};
    // Derivative time in seconds. 0 disables the D term (PI behaviour, the
    // usual choice for room temperature control).
    float tdSeconds{0.0f};
    // Reverse-acting loop: output grows as the measurement RISES above the
    // setpoint (cooling). Direct-acting (heating) when false.
    bool reverseActing{false};
};

class PidController {
public:
    void configure(const PidConfig& config) { config_ = config; }
    const PidConfig& config() const { return config_; }

    void reset()
    {
        integral_ = 0.0f;
        hasLastMeasured_ = false;
    }

    /// Advance the loop by dtSeconds and return the control value in 0..100 %.
    float update(float setpoint, float measured, float dtSeconds)
    {
        // Uniform sign convention: a positive error always calls for more output.
        const float sign = config_.reverseActing ? -1.0f : 1.0f;
        const float error = sign * (setpoint - measured);

        const float p = config_.kp * error;

        // Derivative on the measurement (not the error) so setpoint steps do
        // not kick the output.
        float d = 0.0f;
        if (config_.tdSeconds > 0.0f && hasLastMeasured_ && dtSeconds > 0.0f) {
            const float measuredDelta = sign * (measured - lastMeasured_);
            d = -config_.kp * config_.tdSeconds * measuredDelta / dtSeconds;
        }
        lastMeasured_ = measured;
        hasLastMeasured_ = true;

        // Conditional-integration anti-windup: stop accumulating while the
        // output is saturated in the direction the error is pushing.
        if (config_.tiSeconds > 0.0f && dtSeconds > 0.0f) {
            const float unsaturated = p + integral_ + d;
            const bool windingUp = unsaturated >= 100.0f && error > 0.0f;
            const bool windingDown = unsaturated <= 0.0f && error < 0.0f;
            if (!windingUp && !windingDown) {
                integral_ += config_.kp * error * dtSeconds / config_.tiSeconds;
                integral_ = clampf(integral_, 0.0f, 100.0f);
            }
        } else {
            integral_ = 0.0f;
        }

        return clampf(p + integral_ + d, 0.0f, 100.0f);
    }

private:
    PidConfig config_{};
    float integral_{0.0f};
    float lastMeasured_{0.0f};
    bool hasLastMeasured_{false};
};

// ---------------------------------------------------------------------------
// Thermostat: heating + cooling around one setpoint with a dead band, PID
// control values, two-point requests with hysteresis, floor-temperature limit.
// ---------------------------------------------------------------------------

struct ThermostatConfig {
    PidConfig heatingPid{};                       // direct-acting
    PidConfig coolingPid{.reverseActing = true};  // reverse-acting
    // Cooling setpoint = setpoint + dead band. Keeps heating and cooling from
    // fighting each other around a single comfort setpoint.
    float coolingDeadbandK{2.0f};
    // Two-point request hysteresis: heating request switches ON at
    // setpoint − h and OFF at setpoint + h (mirrored for cooling). Prevents
    // rapid toggling of the boolean heat/cool demand outputs.
    float requestHysteresisK{0.5f};
    // Floor-heating protection: heating is inhibited while the floor probe
    // reads at/above this limit; re-enabled below (limit − floorHysteresisK).
    // 0 disables the limit (e.g. no floor probe connected).
    float maxFloorTemperatureC{28.0f};
    float floorHysteresisK{1.0f};
};

struct ThermostatInputs {
    float setpointC{21.5f};
    float roomTemperatureC{0.0f};
    bool roomTemperatureValid{false};
    float floorTemperatureC{0.0f};
    bool floorTemperatureValid{false};
};

struct ThermostatOutputs {
    uint8_t heatingControlPercent{0};  // DPT 5.001, 0..100
    uint8_t coolingControlPercent{0};  // DPT 5.001, 0..100
    bool heatingRequest{false};
    bool coolingRequest{false};
    bool floorLimitActive{false};
};

class ThermostatController {
public:
    void configure(const ThermostatConfig& config)
    {
        config_ = config;
        config_.heatingPid.reverseActing = false;
        config_.coolingPid.reverseActing = true;
        heatingPid_.configure(config_.heatingPid);
        coolingPid_.configure(config_.coolingPid);
    }
    const ThermostatConfig& config() const { return config_; }

    void reset()
    {
        heatingPid_.reset();
        coolingPid_.reset();
        outputs_ = ThermostatOutputs{};
    }

    const ThermostatOutputs& outputs() const { return outputs_; }

    const ThermostatOutputs& update(const ThermostatInputs& in, float dtSeconds)
    {
        if (!in.roomTemperatureValid) {
            // No measurement: fail safe — everything off, loops reset so the
            // integrators do not wind up against a stale reading.
            heatingPid_.reset();
            coolingPid_.reset();
            outputs_ = ThermostatOutputs{};
            return outputs_;
        }

        const float heatSetpoint = in.setpointC;
        const float coolSetpoint = in.setpointC + config_.coolingDeadbandK;
        const float t = in.roomTemperatureC;
        const float h = config_.requestHysteresisK;

        // Floor-temperature limit (only meaningful with a valid floor probe).
        if (config_.maxFloorTemperatureC > 0.0f && in.floorTemperatureValid) {
            if (in.floorTemperatureC >= config_.maxFloorTemperatureC) {
                outputs_.floorLimitActive = true;
            } else if (in.floorTemperatureC
                       <= config_.maxFloorTemperatureC - config_.floorHysteresisK) {
                outputs_.floorLimitActive = false;
            }
        } else {
            outputs_.floorLimitActive = false;
        }

        // Continuous control values.
        float heating = heatingPid_.update(heatSetpoint, t, dtSeconds);
        float cooling = coolingPid_.update(coolSetpoint, t, dtSeconds);

        // Two-point requests with hysteresis (hold state inside the band).
        if (t <= heatSetpoint - h) {
            outputs_.heatingRequest = true;
        } else if (t >= heatSetpoint + h) {
            outputs_.heatingRequest = false;
        }
        if (t >= coolSetpoint + h) {
            outputs_.coolingRequest = true;
        } else if (t <= coolSetpoint - h) {
            outputs_.coolingRequest = false;
        }

        // Heating/cooling interlock: with a sane dead band both cannot demand
        // at once; if configuration makes them overlap, heating wins.
        if (outputs_.heatingRequest) {
            outputs_.coolingRequest = false;
        }
        if (heating > 0.0f && cooling > 0.0f) {
            cooling = 0.0f;
        }

        // Floor limit inhibits heating only; reset the loop so it restarts
        // cleanly (no accumulated integral) when the floor cools down.
        if (outputs_.floorLimitActive) {
            heating = 0.0f;
            outputs_.heatingRequest = false;
            heatingPid_.reset();
        }

        outputs_.heatingControlPercent = static_cast<uint8_t>(heating + 0.5f);
        outputs_.coolingControlPercent = static_cast<uint8_t>(cooling + 0.5f);
        return outputs_;
    }

private:
    ThermostatConfig config_{};
    PidController heatingPid_{};
    PidController coolingPid_{};
    ThermostatOutputs outputs_{};
};

// ---------------------------------------------------------------------------
// Air-quality ventilation boost: CO2 and humidity channels with hysteresis.
// ---------------------------------------------------------------------------

struct AirQualityConfig {
    // Boost switches ON at setpoint + hysteresis and OFF at
    // setpoint − hysteresis (per channel).
    float co2SetpointPpm{900.0f};
    float co2HysteresisPpm{75.0f};
    float humidityThresholdPct{70.0f};
    float humidityHysteresisPct{5.0f};
};

struct AirQualityInputs {
    float co2Ppm{0.0f};
    bool co2Valid{false};
    float humidityPct{0.0f};
    bool humidityValid{false};
};

class AirQualityBoostController {
public:
    void configure(const AirQualityConfig& config) { config_ = config; }
    const AirQualityConfig& config() const { return config_; }

    bool boostActive() const { return co2High_ || humidityHigh_; }
    bool co2High() const { return co2High_; }
    bool humidityHigh() const { return humidityHigh_; }

    /// Returns the boost demand: any channel above its threshold requests it.
    bool update(const AirQualityInputs& in)
    {
        if (in.co2Valid) {
            if (in.co2Ppm >= config_.co2SetpointPpm + config_.co2HysteresisPpm) {
                co2High_ = true;
            } else if (in.co2Ppm <= config_.co2SetpointPpm - config_.co2HysteresisPpm) {
                co2High_ = false;
            }
        } else {
            co2High_ = false;
        }

        if (in.humidityValid) {
            if (in.humidityPct >= config_.humidityThresholdPct + config_.humidityHysteresisPct) {
                humidityHigh_ = true;
            } else if (in.humidityPct <= config_.humidityThresholdPct - config_.humidityHysteresisPct) {
                humidityHigh_ = false;
            }
        } else {
            humidityHigh_ = false;
        }

        return boostActive();
    }

private:
    AirQualityConfig config_{};
    bool co2High_{false};
    bool humidityHigh_{false};
};

} // namespace hvac
} // namespace sensor_board
