/**
 * @file hvac_control.hpp
 * @brief Device-side room-control logic: mode/preset resolution, PI/two-point
 *        heating & cooling, floor-temperature limit, ventilation/IAQ control.
 *
 * This is application logic, deliberately NOT part of KNstaX: nothing here is
 * KNX-specific. The KNX stack supplies the ports (DPT values) and the ETS
 * parameters that configure these controllers; the control engineering lives
 * with the device firmware.
 *
 * Everything is pure and platform-free (no ESP-IDF, no clock, no allocation):
 * callers pass measurements and a dt, so the logic is host-testable. Enum
 * values below match the raw wire bytes documented next to the corresponding
 * `SensorBoardPort`/`SensorBoardParameter` entries in knx_product.hpp; the
 * KNX binding layer (knx_service.cpp) is responsible for casting between
 * KNstaX DPT types (e.g. `Dpt20Mode`) and these plain enums.
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
// Enums shared with the KNX binding layer (see knx_product.hpp comments).
// ---------------------------------------------------------------------------

// HVAC operating mode / preset (DPT 20.102 semantics on the bus).
enum class OperatingPreset : uint8_t {
    Auto = 0,
    Comfort = 1,
    Standby = 2,
    Economy = 3,
    BuildingProtection = 4,
};

enum class ControllerMode : uint8_t {
    Auto = 0,
    Heat = 1,
    Cool = 2,
    Off = 3,
};

// KNX DPT 20.105 (DPT_HVACContrMode) transport mapping for the controller
// heat/cool/auto/off mode. The internal ControllerMode enum is compact (0..3);
// the bus object uses the standardised 20.105 code points so HMIs and Home
// Assistant's `controller_mode` interpret it correctly (KNX Spec v2.1 Vol 3/7.2:
// 0=Auto, 1=Heat, 3=Cool, 6=Off).
constexpr uint8_t controllerModeToContrMode(ControllerMode mode)
{
    switch (mode) {
        case ControllerMode::Heat: return 1;
        case ControllerMode::Cool: return 3;
        case ControllerMode::Off:  return 6;
        case ControllerMode::Auto:
        default:                   return 0;
    }
}

constexpr ControllerMode controllerModeFromContrMode(uint8_t value)
{
    switch (value) {
        case 1:  return ControllerMode::Heat;
        case 3:  return ControllerMode::Cool;
        case 6:  return ControllerMode::Off;
        case 0:
        default: return ControllerMode::Auto;  // unknown 20.105 codes → Auto
    }
}

enum class HeatCoolState : uint8_t {
    Neutral = 0,
    Heating = 1,
    Cooling = 2,
};

enum class ControlAlgorithm : uint8_t {
    TwoPoint = 0,
    Pi = 1,
};

enum class BinaryDemandStrategy : uint8_t {
    Hysteresis = 0,
    ControlValueThreshold = 1,
};

enum class HeatCoolChangeoverMode : uint8_t {
    Auto = 0,       // internal deadband, both loops may run (interlocked)
    BusInput = 1,   // follow HeatCoolChangeover bus bit (+ polarity)
    FixedHeat = 2,
    FixedCool = 3,
};

enum class WindowOpenBehavior : uint8_t {
    Ignore = 0,
    ProtectionSetpoint = 1,
    BlockOutputs = 2,
};

enum class PresenceBehavior : uint8_t {
    ComfortStandby = 0,
    ComfortEconomy = 1,
};

// Shared by room-temperature-sensor faults and floor-probe faults (kept as a
// single ETS parameter/behavior model to avoid doubling the parameter count).
enum class SensorFaultBehavior : uint8_t {
    HoldLast = 0,
    ForceOff = 1,
    Passthrough = 2,
};

enum class VentilationMode : uint8_t {
    Auto = 0,
    Manual = 1,
    Off = 2,
    Boost = 3,
};

enum class VentilationLevel : uint8_t {
    Off = 0,
    Low = 1,
    Medium = 2,
    High = 3,
    Boost = 4,
};

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
// Thermostat: mode/preset resolution, setpoint model, heating + cooling with
// PI or two-point algorithms, floor-temperature limit.
// ---------------------------------------------------------------------------

struct SetpointTable {
    float comfortC{22.0f};
    float standbyC{20.0f};
    float economyC{18.0f};
    float protectionC{7.0f};

    float forPreset(OperatingPreset preset) const
    {
        switch (preset) {
            case OperatingPreset::Standby: return standbyC;
            case OperatingPreset::Economy: return economyC;
            case OperatingPreset::BuildingProtection: return protectionC;
            case OperatingPreset::Comfort:
            case OperatingPreset::Auto:
            default: return comfortC;
        }
    }
};

struct ThermostatConfig {
    PidConfig heatingPid{};                       // direct-acting
    PidConfig coolingPid{.reverseActing = true};  // reverse-acting

    SetpointTable heatingSetpoints{};
    SetpointTable coolingSetpoints{.comfortC = 24.0f,
                                   .standbyC = 26.0f,
                                   .economyC = 28.0f,
                                   .protectionC = 35.0f};
    float minSetpointC{7.0f};
    float maxSetpointC{35.0f};
    float maxSetpointShiftK{3.0f};

    // Kept for backward compatibility / minimum-gap sanity in Auto changeover
    // mode (the heating and cooling setpoint tables are otherwise independent
    // now — see todo_feature_set.md section 3.1).
    float coolingDeadbandK{2.0f};

    // Two-point request hysteresis: heating request switches ON at
    // setpoint − h and OFF at setpoint + h (mirrored for cooling). Also used
    // for the two-point control algorithm and the binary-demand hysteresis
    // strategy.
    float requestHysteresisK{0.5f};

    // Floor-heating protection: heating is inhibited while the floor probe
    // reads at/above this limit; re-enabled below (limit − floorHysteresisK).
    // 0 disables the limit (e.g. no floor probe connected).
    float maxFloorTemperatureC{28.0f};
    float floorHysteresisK{1.0f};

    bool heatingEnabled{true};
    bool coolingEnabled{false};
    HeatCoolChangeoverMode heatCoolChangeoverMode{HeatCoolChangeoverMode::Auto};
    bool heatCoolChangeoverPolarityInverted{false};  // false: bus 1 = Heat

    ControlAlgorithm heatingAlgorithm{ControlAlgorithm::Pi};
    ControlAlgorithm coolingAlgorithm{ControlAlgorithm::Pi};

    uint8_t heatingMinOutputPercent{0};
    uint8_t heatingMaxOutputPercent{100};
    uint8_t coolingMinOutputPercent{0};
    uint8_t coolingMaxOutputPercent{100};

    BinaryDemandStrategy binaryDemandStrategy{BinaryDemandStrategy::Hysteresis};
    uint8_t binaryDemandThresholdPercent{1};

    float minimumHeatCoolChangeoverSeconds{300.0f};

    WindowOpenBehavior windowOpenBehavior{WindowOpenBehavior::BlockOutputs};
    PresenceBehavior presenceBehavior{PresenceBehavior::ComfortStandby};
    SensorFaultBehavior sensorFaultBehavior{SensorFaultBehavior::ForceOff};
};

struct ThermostatInputs {
    // Room/floor measurements.
    float roomTemperatureC{0.0f};
    bool roomTemperatureValid{false};
    float floorTemperatureC{0.0f};
    bool floorTemperatureValid{false};

    // Mode/state inputs (all bus-writable; see SensorBoardPort comments).
    bool controllerEnable{true};
    OperatingPreset hvacOperatingMode{OperatingPreset::Comfort};  // bus/schedule request
    ControllerMode controllerMode{ControllerMode::Auto};
    bool heatCoolChangeoverBusValue{false};  // DPT 1.100, 1 = Heat (before polarity)
    bool windowOpen{false};
    bool presence{false};
    bool presenceValid{false};  // false until a presence telegram has been received
    float setpointShiftK{0.0f};
};

struct ThermostatOutputs {
    uint8_t heatingControlPercent{0};  // DPT 5.001, 0..100
    uint8_t coolingControlPercent{0};  // DPT 5.001, 0..100
    bool heatingRequest{false};
    bool coolingRequest{false};
    bool floorLimitActive{false};

    OperatingPreset activePreset{OperatingPreset::Comfort};
    HeatCoolState heatCoolState{HeatCoolState::Neutral};
    float activeSetpointC{22.0f};        // feedback for HA/ETS (current direction's setpoint)
    float setpointShiftFeedbackK{0.0f};  // clamped shift actually applied
    bool controllerFault{false};         // room-temperature sensor invalid
    bool heatingBlocked{false};
    bool coolingBlocked{false};
};

// KNX DPT 22.101 (DPT_StatusRHCC) — the standardised room heating/cooling
// controller status word reported to HMIs. Bit numbers per KNX Spec v2.1
// Vol 3/7.2. Only the bits this controller can source are populated; the rest
// stay 0 (their spec-defined default for optional fields).
enum class StatusRHCCBit : uint16_t {
    Fault           = 1u << 0,
    StatusEcoHeat   = 1u << 1,
    FlowTempLimit   = 1u << 2,
    ReturnTempLimit = 1u << 3,
    MorningBoostH   = 1u << 4,
    StartOptim      = 1u << 5,
    StopOptim       = 1u << 6,
    HeatingDisabled = 1u << 7,
    HeatCoolMode    = 1u << 8,   // 1 = heating, 0 = cooling
    StatusEcoCool   = 1u << 9,
    PreCool         = 1u << 10,
    CoolingDisabled = 1u << 11,
    FrostAlarm      = 1u << 13,
    OverheatAlarm   = 1u << 14,
};

constexpr uint16_t packStatusRHCC(const ThermostatOutputs& out)
{
    uint16_t status = 0;
    if (out.controllerFault) status |= static_cast<uint16_t>(StatusRHCCBit::Fault);
    // Floor-temperature limit maps onto the flow-temperature-limitation bit
    // (its spec meaning is "max flow temperature limitation for floor heating").
    if (out.floorLimitActive) status |= static_cast<uint16_t>(StatusRHCCBit::FlowTempLimit);
    if (out.heatingBlocked)   status |= static_cast<uint16_t>(StatusRHCCBit::HeatingDisabled);
    if (out.coolingBlocked)   status |= static_cast<uint16_t>(StatusRHCCBit::CoolingDisabled);
    // HeatCoolMode: 1 = heating (spec default), 0 = cooling. Report cooling only
    // while actively cooling; neutral and heating both read as heating.
    if (out.heatCoolState != HeatCoolState::Cooling) {
        status |= static_cast<uint16_t>(StatusRHCCBit::HeatCoolMode);
    }
    return status;
}

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
        secondsSinceHeating_ = 1e9f;
        secondsSinceCooling_ = 1e9f;
        hasLastValidRoomTemperature_ = false;
    }

    const ThermostatOutputs& outputs() const { return outputs_; }

    const ThermostatOutputs& update(const ThermostatInputs& in, float dtSeconds)
    {
        // ---- Priority 1: device fault / critical sensor invalid ----------
        float roomTemperatureC = in.roomTemperatureC;
        bool haveRoomTemperature = in.roomTemperatureValid;
        outputs_.controllerFault = !in.roomTemperatureValid;

        if (!haveRoomTemperature) {
            switch (config_.sensorFaultBehavior) {
                case SensorFaultBehavior::Passthrough:
                    if (hasLastValidRoomTemperature_) {
                        roomTemperatureC = lastValidRoomTemperatureC_;
                        haveRoomTemperature = true;
                    }
                    break;
                case SensorFaultBehavior::HoldLast:
                    // Keep last computed outputs untouched; still advance the
                    // changeover timers so a fault does not distort them.
                    advanceChangeoverTimers(dtSeconds);
                    return outputs_;
                case SensorFaultBehavior::ForceOff:
                default:
                    heatingPid_.reset();
                    coolingPid_.reset();
                    outputs_.heatingControlPercent = 0;
                    outputs_.coolingControlPercent = 0;
                    outputs_.heatingRequest = false;
                    outputs_.coolingRequest = false;
                    outputs_.heatCoolState = HeatCoolState::Neutral;
                    outputs_.heatingBlocked = true;
                    outputs_.coolingBlocked = true;
                    advanceChangeoverTimers(dtSeconds);
                    return outputs_;
            }
        } else {
            lastValidRoomTemperatureC_ = roomTemperatureC;
            hasLastValidRoomTemperature_ = true;
        }

        // ---- Floor-temperature limit (independent of the above) ----------
        updateFloorLimit(in);

        // ---- Priorities 2..8: resolve the active preset -------------------
        const OperatingPreset preset = resolveActivePreset(in);
        outputs_.activePreset = preset;

        // ---- Setpoint model: table lookup + clamped shift ------------------
        const float shift = clampf(in.setpointShiftK, -config_.maxSetpointShiftK, config_.maxSetpointShiftK);
        outputs_.setpointShiftFeedbackK = shift;

        float heatSetpoint = clampf(config_.heatingSetpoints.forPreset(preset) + shift,
                                    config_.minSetpointC, config_.maxSetpointC);
        float coolSetpoint = clampf(config_.coolingSetpoints.forPreset(preset) + shift,
                                    config_.minSetpointC, config_.maxSetpointC);
        // Guarantee a minimum gap between heat/cool setpoints so a shared
        // deadband-style Auto changeover cannot make both loops demand at once.
        if (coolSetpoint < heatSetpoint + config_.coolingDeadbandK) {
            coolSetpoint = heatSetpoint + config_.coolingDeadbandK;
        }

        // ---- Direction gating: controller mode + changeover ---------------
        bool allowHeat = false;
        bool allowCool = false;
        resolveDirection(in, allowHeat, allowCool);
        allowHeat = allowHeat && config_.heatingEnabled;
        allowCool = allowCool && config_.coolingEnabled;

        // Window-open block (BlockOutputs behavior forces both off outright).
        const bool windowBlocksOutputs =
            in.windowOpen && config_.windowOpenBehavior == WindowOpenBehavior::BlockOutputs;
        // Controller-disabled or manual Off also block everything.
        const bool controllerOff = !in.controllerEnable || in.controllerMode == ControllerMode::Off;
        if (windowBlocksOutputs || controllerOff) {
            allowHeat = false;
            allowCool = false;
        }

        // Minimum heat/cool changeover delay: once a direction has been
        // active, forbid switching to the other direction until the timer
        // elapses (does not affect turning a direction off).
        if (allowHeat && secondsSinceCooling_ < config_.minimumHeatCoolChangeoverSeconds
            && outputs_.heatCoolState != HeatCoolState::Heating) {
            allowHeat = (secondsSinceCooling_ >= config_.minimumHeatCoolChangeoverSeconds);
        }
        if (allowCool && secondsSinceHeating_ < config_.minimumHeatCoolChangeoverSeconds
            && outputs_.heatCoolState != HeatCoolState::Cooling) {
            allowCool = (secondsSinceHeating_ >= config_.minimumHeatCoolChangeoverSeconds);
        }

        outputs_.heatingBlocked = !allowHeat;
        outputs_.coolingBlocked = !allowCool;

        // ---- Continuous control values -------------------------------------
        float heating = 0.0f;
        float cooling = 0.0f;

        if (allowHeat) {
            heating = (config_.heatingAlgorithm == ControlAlgorithm::Pi)
                          ? heatingPid_.update(heatSetpoint, roomTemperatureC, dtSeconds)
                          : (roomTemperatureC <= heatSetpoint - config_.requestHysteresisK ? 100.0f : 0.0f);
        } else {
            heatingPid_.reset();
        }

        if (allowCool) {
            cooling = (config_.coolingAlgorithm == ControlAlgorithm::Pi)
                          ? coolingPid_.update(coolSetpoint, roomTemperatureC, dtSeconds)
                          : (roomTemperatureC >= coolSetpoint + config_.requestHysteresisK ? 100.0f : 0.0f);
        } else {
            coolingPid_.reset();
        }

        // ---- Two-point requests (hysteresis strategy) -----------------------
        const float h = config_.requestHysteresisK;
        if (config_.binaryDemandStrategy == BinaryDemandStrategy::Hysteresis) {
            if (!allowHeat) {
                outputs_.heatingRequest = false;
            } else if (roomTemperatureC <= heatSetpoint - h) {
                outputs_.heatingRequest = true;
            } else if (roomTemperatureC >= heatSetpoint + h) {
                outputs_.heatingRequest = false;
            }
            if (!allowCool) {
                outputs_.coolingRequest = false;
            } else if (roomTemperatureC >= coolSetpoint + h) {
                outputs_.coolingRequest = true;
            } else if (roomTemperatureC <= coolSetpoint - h) {
                outputs_.coolingRequest = false;
            }
        } else {
            outputs_.heatingRequest = allowHeat && heating >= static_cast<float>(config_.binaryDemandThresholdPercent);
            outputs_.coolingRequest = allowCool && cooling >= static_cast<float>(config_.binaryDemandThresholdPercent);
        }

        // ---- Heating/cooling mutual exclusion: heating wins ------------------
        if (outputs_.heatingRequest || heating > 0.0f) {
            outputs_.coolingRequest = false;
            cooling = 0.0f;
        }

        // ---- Floor limit inhibits heating only --------------------------------
        if (outputs_.floorLimitActive) {
            heating = 0.0f;
            outputs_.heatingRequest = false;
            outputs_.heatingBlocked = true;
            heatingPid_.reset();
        }

        // ---- Output clamps (minimum actuator opening / maximum limit) --------
        heating = clampOutputPercent(heating, config_.heatingMinOutputPercent, config_.heatingMaxOutputPercent);
        cooling = clampOutputPercent(cooling, config_.coolingMinOutputPercent, config_.coolingMaxOutputPercent);

        outputs_.heatingControlPercent = static_cast<uint8_t>(heating + 0.5f);
        outputs_.coolingControlPercent = static_cast<uint8_t>(cooling + 0.5f);

        // ---- Heat/cool state + changeover timers ------------------------------
        if (outputs_.heatingControlPercent > 0 || outputs_.heatingRequest) {
            outputs_.heatCoolState = HeatCoolState::Heating;
        } else if (outputs_.coolingControlPercent > 0 || outputs_.coolingRequest) {
            outputs_.heatCoolState = HeatCoolState::Cooling;
        } else {
            outputs_.heatCoolState = HeatCoolState::Neutral;
        }
        advanceChangeoverTimers(dtSeconds);

        // ---- Active setpoint feedback (direction-appropriate) -----------------
        outputs_.activeSetpointC =
            (outputs_.heatCoolState == HeatCoolState::Cooling) ? coolSetpoint : heatSetpoint;

        return outputs_;
    }

private:
    static float clampOutputPercent(float value, uint8_t minPercent, uint8_t maxPercent)
    {
        float out = clampf(value, 0.0f, static_cast<float>(maxPercent));
        if (out > 0.0f && out < static_cast<float>(minPercent)) {
            out = static_cast<float>(minPercent);
        }
        return out;
    }

    void updateFloorLimit(const ThermostatInputs& in)
    {
        if (config_.maxFloorTemperatureC <= 0.0f) {
            outputs_.floorLimitActive = false;
            return;
        }

        if (in.floorTemperatureValid) {
            if (in.floorTemperatureC >= config_.maxFloorTemperatureC) {
                outputs_.floorLimitActive = true;
            } else if (in.floorTemperatureC
                       <= config_.maxFloorTemperatureC - config_.floorHysteresisK) {
                outputs_.floorLimitActive = false;
            }
            return;
        }

        // Floor probe invalid: reuse the shared SensorFaultBehavior model
        // (see todo_feature_set.md section 7).
        switch (config_.sensorFaultBehavior) {
            case SensorFaultBehavior::Passthrough:
                outputs_.floorLimitActive = false;  // ignore limit, fault still reported via ControllerFault path
                break;
            case SensorFaultBehavior::HoldLast:
                break;  // keep last computed state
            case SensorFaultBehavior::ForceOff:
            default:
                outputs_.floorLimitActive = true;  // safest: block heating
                break;
        }
    }

    OperatingPreset resolveActivePreset(const ThermostatInputs& in) const
    {
        // Priority 3: window open.
        if (in.windowOpen && config_.windowOpenBehavior == WindowOpenBehavior::ProtectionSetpoint) {
            return OperatingPreset::BuildingProtection;
        }
        // Priority 4: explicit building-protection request always honored.
        if (in.hvacOperatingMode == OperatingPreset::BuildingProtection) {
            return OperatingPreset::BuildingProtection;
        }
        // Priority 6: presence/occupancy (only once a presence telegram has
        // actually been received — otherwise fall through to the schedule).
        if (in.presenceValid) {
            if (in.presence) {
                return OperatingPreset::Comfort;
            }
            return (config_.presenceBehavior == PresenceBehavior::ComfortEconomy)
                       ? OperatingPreset::Economy
                       : OperatingPreset::Standby;
        }
        // Priority 7: scheduled/bus-requested HVAC operating mode.
        if (in.hvacOperatingMode != OperatingPreset::Auto) {
            return in.hvacOperatingMode;
        }
        // Priority 8: ETS default (Auto resolves to Comfort when nothing else
        // is configured/known — the ETS default parameter seeds the initial
        // hvacOperatingMode value at startup, so this is effectively already
        // applied upstream).
        return OperatingPreset::Comfort;
    }

    void resolveDirection(const ThermostatInputs& in, bool& allowHeat, bool& allowCool) const
    {
        switch (in.controllerMode) {
            case ControllerMode::Off:
                allowHeat = false;
                allowCool = false;
                return;
            case ControllerMode::Heat:
                allowHeat = true;
                allowCool = false;
                return;
            case ControllerMode::Cool:
                allowHeat = false;
                allowCool = true;
                return;
            case ControllerMode::Auto:
            default:
                break;
        }

        switch (config_.heatCoolChangeoverMode) {
            case HeatCoolChangeoverMode::FixedHeat:
                allowHeat = true;
                allowCool = false;
                return;
            case HeatCoolChangeoverMode::FixedCool:
                allowHeat = false;
                allowCool = true;
                return;
            case HeatCoolChangeoverMode::BusInput: {
                const bool wantHeat = config_.heatCoolChangeoverPolarityInverted
                                          ? !in.heatCoolChangeoverBusValue
                                          : in.heatCoolChangeoverBusValue;
                allowHeat = wantHeat;
                allowCool = !wantHeat;
                return;
            }
            case HeatCoolChangeoverMode::Auto:
            default:
                // Internal deadband: both loops may run; the mutual-exclusion
                // step later in update() guarantees only one is ever active.
                allowHeat = true;
                allowCool = true;
                return;
        }
    }

    void advanceChangeoverTimers(float dtSeconds)
    {
        if (outputs_.heatCoolState == HeatCoolState::Heating) {
            secondsSinceHeating_ = 0.0f;
            secondsSinceCooling_ += dtSeconds;
        } else if (outputs_.heatCoolState == HeatCoolState::Cooling) {
            secondsSinceCooling_ = 0.0f;
            secondsSinceHeating_ += dtSeconds;
        } else {
            secondsSinceHeating_ += dtSeconds;
            secondsSinceCooling_ += dtSeconds;
        }
    }

    ThermostatConfig config_{};
    PidController heatingPid_{};
    PidController coolingPid_{};
    ThermostatOutputs outputs_{};
    float secondsSinceHeating_{1e9f};
    float secondsSinceCooling_{1e9f};
    float lastValidRoomTemperatureC_{0.0f};
    bool hasLastValidRoomTemperature_{false};
};

// ---------------------------------------------------------------------------
// Ventilation / IAQ control: CO2 + humidity demand, mode override, level
// bucketing, IAQ status bitset.
// ---------------------------------------------------------------------------

struct VentilationConfig {
    // Boost switches ON at setpoint + hysteresis and OFF at
    // setpoint − hysteresis (per channel).
    float co2SetpointPpm{900.0f};
    float co2HysteresisPpm{75.0f};
    float humidityThresholdPct{70.0f};
    float humidityHysteresisPct{5.0f};
};

struct VentilationInputs {
    float co2Ppm{0.0f};
    bool co2Valid{false};
    float humidityPct{0.0f};
    bool humidityValid{false};
    VentilationMode mode{VentilationMode::Auto};
};

struct VentilationOutputs {
    uint8_t demandPercent{0};
    VentilationLevel level{VentilationLevel::Off};
    bool active{false};
    bool humidityHigh{false};
    bool co2High{false};
    bool sensorFault{false};
};

// IaqStatus bitset bits (SensorBoardPort::IaqStatus).
enum class IaqStatusBit : uint16_t {
    HumidityBoost = 1u << 0,
    Co2Boost = 1u << 1,
    SensorFault = 1u << 2,
};

class VentilationController {
public:
    void configure(const VentilationConfig& config) { config_ = config; }
    const VentilationConfig& config() const { return config_; }
    const VentilationOutputs& outputs() const { return outputs_; }

    const VentilationOutputs& update(const VentilationInputs& in)
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

        // Demand = max(humidity demand, CO2 demand), each represented as a
        // binary boost right now (no proportional CO2/humidity model yet);
        // this still satisfies the ABI (0/100 % demand maps cleanly onto the
        // level buckets below).
        uint8_t autoDemand = 0;
        if (humidityHigh_ || co2High_) {
            autoDemand = 100;
        }

        uint8_t demand = 0;
        switch (in.mode) {
            case VentilationMode::Off:
                demand = 0;
                break;
            case VentilationMode::Boost:
                demand = 100;
                break;
            case VentilationMode::Manual:
                // No dedicated manual-percent input exists on the current ABI;
                // fall back to the automatic calculation (see
                // todo_feature_set.md section 8 / knx_product.hpp comments).
            case VentilationMode::Auto:
            default:
                demand = autoDemand;
                break;
        }

        outputs_.demandPercent = demand;
        outputs_.active = demand > 0;
        outputs_.humidityHigh = humidityHigh_;
        outputs_.co2High = co2High_;
        outputs_.sensorFault = !in.co2Valid && !in.humidityValid;

        if (demand == 0) {
            outputs_.level = VentilationLevel::Off;
        } else if (demand < 34) {
            outputs_.level = VentilationLevel::Low;
        } else if (demand < 67) {
            outputs_.level = VentilationLevel::Medium;
        } else if (demand < 100) {
            outputs_.level = VentilationLevel::High;
        } else {
            outputs_.level = VentilationLevel::Boost;
        }

        return outputs_;
    }

private:
    VentilationConfig config_{};
    VentilationOutputs outputs_{};
    bool co2High_{false};
    bool humidityHigh_{false};
};

} // namespace hvac
} // namespace sensor_board
