// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

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
 * `HabinariPort`/`HabinariParameter` entries in knx_product.hpp; the
 * KNX binding layer (knx_service.cpp) is responsible for casting between
 * KNstaX DPT types (e.g. `Dpt20Mode`) and these plain enums.
 */

#pragma once

#include <cmath>
#include <cstdint>

namespace habinari {
namespace hvac {

inline float clampf(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

// ---------------------------------------------------------------------------
// Psychrometrics: the quantities the board can derive from the raw
// temperature / relative-humidity / pressure readings.
//
// Relative humidity on its own is a poor control and diagnostic signal — it
// changes with temperature at constant moisture content, so two rooms at
// "60 %RH" can hold very different amounts of water. The derived quantities
// below are the ones the rest of the installation can actually act on:
//
//   dew point         condensation protection for cooled surfaces (KNX FB
//                     "Dew Point Status Sensor", Vol 7/19/20 clause 3.4)
//   absolute humidity moisture *content* (g of water per m³ of air), so two air
//                     masses at different temperatures can be compared at all —
//                     which relative humidity cannot do (KNX DPT 9.029). Note
//                     it is a per-volume density, so it is not exactly
//                     conserved when a parcel is cooled; the vapour pressure
//                     is. The drift is a few percent over a 10 K change, far
//                     below the thresholds anything here compares against.
//   sea-level pressure the only barometric value that is comparable between
//                     sites, and what a weather visualisation expects
//
// Magnus/Sonntag coefficients over water; the stated fit error is < 0.1 % of
// e_s across -45..+60 °C, far below the sensor accuracy of any part on this
// board. All functions are pure and total: callers are responsible for only
// feeding them readings they have already marked valid.
// ---------------------------------------------------------------------------

namespace psychro {

inline constexpr float kMagnusA = 17.62f;      // dimensionless
inline constexpr float kMagnusB = 243.12f;     // °C
inline constexpr float kMagnusEs0Hpa = 6.112f; // saturation pressure at 0 °C

/// Saturation vapour pressure over water, in hPa.
inline float saturationVapourPressureHpa(float temperatureC)
{
    return kMagnusEs0Hpa
           * std::exp((kMagnusA * temperatureC) / (kMagnusB + temperatureC));
}

/// Partial water-vapour pressure of moist air, in hPa.
inline float vapourPressureHpa(float temperatureC, float relativeHumidityPct)
{
    const float rh = clampf(relativeHumidityPct, 0.0f, 100.0f);
    return (rh / 100.0f) * saturationVapourPressureHpa(temperatureC);
}

/// Dew-point temperature in °C: the temperature a surface must fall to before
/// water condenses out of this air.
inline float dewPointC(float temperatureC, float relativeHumidityPct)
{
    // Clamp the low end away from zero: ln(0) is -inf, and a 0 %RH reading is
    // a broken sensor rather than genuinely dry air.
    const float rh = clampf(relativeHumidityPct, 0.5f, 100.0f);
    const float gamma = std::log(rh / 100.0f)
                        + (kMagnusA * temperatureC) / (kMagnusB + temperatureC);
    return (kMagnusB * gamma) / (kMagnusA - gamma);
}

/// Absolute humidity in g/m³ (KNX DPT 9.029), from the ideal gas law applied
/// to the vapour partial pressure: rho_v = e * M_w / (R * T).
/// 216.7 = 100 (hPa→Pa) * 18.015 (g/mol) / 8.31446 (J/(mol·K)).
inline float absoluteHumidityGm3(float temperatureC, float relativeHumidityPct)
{
    const float e = vapourPressureHpa(temperatureC, relativeHumidityPct);
    return 216.7f * e / (temperatureC + 273.15f);
}

/// Station pressure reduced to mean sea level, in Pa (KNX DPT 9.006), using
/// the international barometric formula with the measured air temperature.
/// Returns the input unchanged at zero altitude, so the default configuration
/// costs nothing.
inline float seaLevelPressurePa(float stationPressurePa, float altitudeM, float temperatureC)
{
    if (altitudeM == 0.0f) {
        return stationPressurePa;
    }
    const float lapse = 0.0065f * altitudeM;  // K over the reduced column
    const float ratio = 1.0f - lapse / (temperatureC + lapse + 273.15f);
    if (ratio <= 0.0f) {
        return stationPressurePa;  // non-physical altitude/temperature pair
    }
    return stationPressurePa * std::pow(ratio, -5.257f);
}

} // namespace psychro

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

// Which surface temperature the dew-point monitor guards. Underfloor cooling
// condenses on the slab (floor probe); a chilled ceiling or fan coil condenses
// on the coldest wetted surface, which only the flow-temperature input knows.
enum class DewPointSurfaceSource : uint8_t {
    Off = 0,
    FloorProbe = 1,
    FlowTemperature = 2,
    Coldest = 3,  // whichever of the two is available and lower
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

// KNX setpoint ladder (KNX Vol 7/19/20 clause 5, FB Room Temperature Setpoint
// Manager). The four operating modes are NOT eight independent absolute
// temperatures: the standard models Standby/Economy as *shifts* away from the
// comfort setpoint (DPT_TempRoomSetpSetShiftF16, 275.101), and only Comfort and
// Building Protection are absolute. Following that here means a single write to
// the base setpoint — from an HMI, a scheduler, or Home Assistant's
// target_temperature — moves the whole ladder coherently, instead of silently
// leaving Standby above the new Comfort value.
struct SetpointLadder {
    // Absolute anchor: the comfort heating setpoint. Bus-writable.
    float comfortHeatingC{21.0f};
    // Heating reductions below the comfort setpoint (positive = cooler).
    float standbyHeatingReductionK{2.0f};
    float economyHeatingReductionK{4.0f};
    // Absolute: frost protection has no meaningful relation to comfort.
    float protectionHeatingC{7.0f};

    // Cooling comfort sits a dead band above heating comfort, so heating and
    // cooling can never demand at the same temperature.
    float coolingDeadbandK{2.0f};
    // Cooling increases above the cooling comfort setpoint (positive = warmer).
    float standbyCoolingIncreaseK{2.0f};
    float economyCoolingIncreaseK{4.0f};
    float protectionCoolingC{35.0f};

    float comfortCoolingC() const { return comfortHeatingC + coolingDeadbandK; }

    float heatingForPreset(OperatingPreset preset) const
    {
        switch (preset) {
            case OperatingPreset::Standby: return comfortHeatingC - standbyHeatingReductionK;
            case OperatingPreset::Economy: return comfortHeatingC - economyHeatingReductionK;
            case OperatingPreset::BuildingProtection: return protectionHeatingC;
            case OperatingPreset::Comfort:
            case OperatingPreset::Auto:
            default: return comfortHeatingC;
        }
    }

    float coolingForPreset(OperatingPreset preset) const
    {
        switch (preset) {
            case OperatingPreset::Standby: return comfortCoolingC() + standbyCoolingIncreaseK;
            case OperatingPreset::Economy: return comfortCoolingC() + economyCoolingIncreaseK;
            case OperatingPreset::BuildingProtection: return protectionCoolingC;
            case OperatingPreset::Comfort:
            case OperatingPreset::Auto:
            default: return comfortCoolingC();
        }
    }
};

struct ThermostatConfig {
    PidConfig heatingPid{};                       // direct-acting
    PidConfig coolingPid{.reverseActing = true};  // reverse-acting

    SetpointLadder setpoints{};
    float minSetpointC{7.0f};
    float maxSetpointC{35.0f};
    float maxSetpointShiftK{3.0f};

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

    // Floor comfort temperering: hold a minimum slab temperature regardless of
    // room demand (a bathroom floor that is merely "not cold" still feels cold
    // underfoot). 0 disables. Heating is forced to at least
    // floorComfortOutputPercent while the slab is below the limit.
    float minFloorTemperatureC{0.0f};
    uint8_t floorComfortOutputPercent{30};

    // Room-temperature alarm thresholds reported in DPT 22.101 bits 13/14.
    // 0 disables the corresponding alarm.
    float frostAlarmTemperatureC{5.0f};
    float overheatAlarmTemperatureC{35.0f};

    // Suspend cooling while a condensation risk is signalled (either derived
    // locally or received on the DewPointStatus input).
    bool blockCoolingOnDewPointAlarm{true};

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

    // Mode/state inputs (all bus-writable; see HabinariPort comments).
    bool controllerEnable{true};
    OperatingPreset hvacOperatingMode{OperatingPreset::Comfort};  // bus/schedule request
    ControllerMode controllerMode{ControllerMode::Auto};
    bool heatCoolChangeoverBusValue{false};  // DPT 1.100, 1 = Heat (before polarity)
    bool windowOpen{false};
    bool presence{false};
    bool presenceValid{false};  // false until a presence telegram has been received
    float setpointShiftK{0.0f};

    // RTC control inputs (KNX Vol 7/19/20 clause 6.3.4). SwitchHeat/SwitchCool
    // are the standard "switch this sequence off entirely" signals; they
    // default to enabled so an unlinked object never disables the controller.
    bool switchHeat{true};
    bool switchCool{true};

    // Condensation risk: either derived on-board (see DewPointMonitor) or
    // received from a system-wide dew point sensor on the DewPointStatus input.
    bool dewPointAlarm{false};

    // Cooling flow / coldest-surface temperature, when the installation
    // publishes one. Feeds the dew-point monitor and the flow-limit status bit.
    float flowTemperatureC{0.0f};
    bool flowTemperatureValid{false};
};

struct ThermostatOutputs {
    uint8_t heatingControlPercent{0};  // DPT 5.001, 0..100
    uint8_t coolingControlPercent{0};  // DPT 5.001, 0..100
    bool heatingRequest{false};
    bool coolingRequest{false};
    bool floorLimitActive{false};

    bool floorComfortActive{false};

    OperatingPreset activePreset{OperatingPreset::Comfort};
    HeatCoolState heatCoolState{HeatCoolState::Neutral};
    // RTC.TempRoomSetpAct: the setpoint the controller is working to right now.
    float activeSetpointC{22.0f};
    // RTSM.TempRoomSetpHeatEff / TempRoomSetpCoolEff: both effective setpoints,
    // published unconditionally so a visualisation can draw the dead band even
    // while the controller sits in the middle of it.
    float heatingSetpointC{22.0f};
    float coolingSetpointC{24.0f};
    float setpointShiftFeedbackK{0.0f};  // clamped shift actually applied
    bool controllerFault{false};         // room-temperature sensor invalid
    bool heatingBlocked{false};
    bool coolingBlocked{false};
    bool frostAlarm{false};
    bool overheatAlarm{false};
    bool dewPointAlarm{false};  // mirrored into the status word (bit 12)
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
    DewPointStatus  = 1u << 12,
    FrostAlarm      = 1u << 13,
    OverheatAlarm   = 1u << 14,
};

constexpr uint16_t packStatusRHCC(const ThermostatOutputs& out)
{
    uint16_t status = 0;
    if (out.controllerFault) status |= static_cast<uint16_t>(StatusRHCCBit::Fault);
    // Floor-temperature limit maps onto the flow-temperature-limitation bit:
    // its spec wording is literally "max. flow temperature limitation for floor
    // heating protection", which is exactly what the floor probe enforces here.
    if (out.floorLimitActive) status |= static_cast<uint16_t>(StatusRHCCBit::FlowTempLimit);
    if (out.heatingBlocked)   status |= static_cast<uint16_t>(StatusRHCCBit::HeatingDisabled);
    if (out.coolingBlocked)   status |= static_cast<uint16_t>(StatusRHCCBit::CoolingDisabled);
    if (out.dewPointAlarm)    status |= static_cast<uint16_t>(StatusRHCCBit::DewPointStatus);
    if (out.frostAlarm)       status |= static_cast<uint16_t>(StatusRHCCBit::FrostAlarm);
    if (out.overheatAlarm)    status |= static_cast<uint16_t>(StatusRHCCBit::OverheatAlarm);
    // HeatCoolMode: 1 = heating (spec default), 0 = cooling. Report cooling only
    // while actively cooling; neutral and heating both read as heating.
    if (out.heatCoolState != HeatCoolState::Cooling) {
        status |= static_cast<uint16_t>(StatusRHCCBit::HeatCoolMode);
    }
    return status;
}

// KNX DPT 21.001 (DPT_StatusGen) — the per-Datapoint status octet every KNX
// sensor Functional Block pairs with its measurement Output.
enum class StatusGenBit : uint8_t {
    OutOfService = 1u << 0,
    Fault        = 1u << 1,
    Overridden   = 1u << 2,
    InAlarm      = 1u << 3,
    AlarmUnAck   = 1u << 4,
};

/// Build a StatusGen octet for one physical sensor. `present` is false when the
/// sensor is not fitted at all (an unconnected floor probe is out of service,
/// not faulty); `healthy` is false when a fitted sensor stopped delivering
/// readings; `inAlarm` carries a measurand-specific alarm (e.g. slab moisture).
constexpr uint8_t packStatusGen(bool present, bool healthy, bool inAlarm = false)
{
    uint8_t status = 0;
    if (!present) {
        status |= static_cast<uint8_t>(StatusGenBit::OutOfService);
    } else if (!healthy) {
        status |= static_cast<uint8_t>(StatusGenBit::Fault);
    }
    if (inAlarm) status |= static_cast<uint8_t>(StatusGenBit::InAlarm);
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
                    // No trustworthy room temperature means no trustworthy
                    // temperature alarms either; reporting stale ones would
                    // send an installer chasing the wrong fault.
                    outputs_.frostAlarm = false;
                    outputs_.overheatAlarm = false;
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

        // ---- Setpoint model: ladder lookup + clamped shift -----------------
        const float shift = clampf(in.setpointShiftK, -config_.maxSetpointShiftK, config_.maxSetpointShiftK);
        outputs_.setpointShiftFeedbackK = shift;

        float heatSetpoint = clampf(config_.setpoints.heatingForPreset(preset) + shift,
                                    config_.minSetpointC, config_.maxSetpointC);
        float coolSetpoint = clampf(config_.setpoints.coolingForPreset(preset) + shift,
                                    config_.minSetpointC, config_.maxSetpointC);
        // The ladder already places cooling a dead band above heating, but the
        // [min, max] clamp above can squeeze the two together. Re-open the gap
        // so a deadband-style Auto changeover can never demand both at once.
        if (coolSetpoint < heatSetpoint + config_.setpoints.coolingDeadbandK) {
            coolSetpoint = heatSetpoint + config_.setpoints.coolingDeadbandK;
        }
        outputs_.heatingSetpointC = heatSetpoint;
        outputs_.coolingSetpointC = coolSetpoint;

        // ---- Room-temperature alarms (status word bits 13/14) --------------
        outputs_.frostAlarm = config_.frostAlarmTemperatureC > 0.0f
                              && roomTemperatureC < config_.frostAlarmTemperatureC;
        outputs_.overheatAlarm = config_.overheatAlarmTemperatureC > 0.0f
                                 && roomTemperatureC > config_.overheatAlarmTemperatureC;

        // ---- Direction gating: controller mode + changeover ---------------
        bool allowHeat = false;
        bool allowCool = false;
        resolveDirection(in, allowHeat, allowCool);
        allowHeat = allowHeat && config_.heatingEnabled && in.switchHeat;
        allowCool = allowCool && config_.coolingEnabled && in.switchCool;

        // Condensation protection outranks comfort: cooling a surface below the
        // room dew point puts water on the floor or the ceiling.
        outputs_.dewPointAlarm = in.dewPointAlarm;
        if (in.dewPointAlarm && config_.blockCoolingOnDewPointAlarm) {
            allowCool = false;
        }

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

        // ---- Floor comfort temperering ----------------------------------------
        // A slab that is merely "not cold" still feels cold underfoot, so an
        // optional minimum floor temperature can call for heat that the room
        // temperature alone would not. Deliberately evaluated before the floor
        // *limit* below, which always wins: the limit protects the floor
        // covering, the minimum only protects comfort.
        if (outputs_.floorComfortActive && allowHeat) {
            heating = (heating > static_cast<float>(config_.floorComfortOutputPercent))
                          ? heating
                          : static_cast<float>(config_.floorComfortOutputPercent);
            outputs_.heatingRequest = true;
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
        // Minimum-floor-temperature comfort band (independent of the maximum).
        if (config_.minFloorTemperatureC > 0.0f && in.floorTemperatureValid) {
            if (in.floorTemperatureC <= config_.minFloorTemperatureC - config_.floorHysteresisK) {
                outputs_.floorComfortActive = true;
            } else if (in.floorTemperatureC >= config_.minFloorTemperatureC) {
                outputs_.floorComfortActive = false;
            }
        } else {
            outputs_.floorComfortActive = false;
        }

        if (config_.maxFloorTemperatureC <= 0.0f) {
            outputs_.floorLimitActive = false;
            return;
        }

        if (in.floorTemperatureValid) {
            floorProbeSeen_ = true;
            if (in.floorTemperatureC >= config_.maxFloorTemperatureC) {
                outputs_.floorLimitActive = true;
            } else if (in.floorTemperatureC
                       <= config_.maxFloorTemperatureC - config_.floorHysteresisK) {
                outputs_.floorLimitActive = false;
            }
            return;
        }

        // No probe has ever reported. The floor probe is optional hardware, so
        // this is an installation without one, not a broken one — the same
        // distinction packStatusGen() already draws between OutOfService and
        // Fault. There is no floor to protect, so there is no limit to apply.
        //
        // Getting this wrong is not a subtle failure: with the default maximum
        // floor temperature enabled and the default ForceOff fault behaviour, a
        // factory-fresh board with no probe fitted latched floorLimitActive at
        // boot and never called for heat again.
        if (!floorProbeSeen_) {
            outputs_.floorLimitActive = false;
            return;
        }

        // A probe that reported and then stopped is a genuine fault: reuse the
        // shared SensorFaultBehavior model (see todo_feature_set.md section 7).
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
    // Tells "no floor probe is fitted" apart from "the fitted probe failed".
    // Only the second is a fault; see updateFloorLimit().
    bool floorProbeSeen_{false};
};

// ---------------------------------------------------------------------------
// Dew-point monitor: KNX FB "Dew Point Status Sensor" (Vol 7/19/20 clause 3.4).
//
// The room's dew point is a property of the *air*; condensation is a property
// of the coldest wetted *surface*. This board can see both — room air from the
// HDC3020, slab temperature from the floor probe, and the system flow
// temperature if the installation publishes one — which is exactly the pairing
// a chilled-ceiling or floor-cooling controller needs and rarely has.
// ---------------------------------------------------------------------------

struct DewPointConfig {
    DewPointSurfaceSource surfaceSource{DewPointSurfaceSource::Coldest};
    // Alarm while surface temperature <= dew point + margin; clears again at
    // dew point + margin + hysteresis.
    float marginK{2.0f};
    float hysteresisK{1.0f};
};

struct DewPointInputs {
    float roomTemperatureC{0.0f};
    float roomHumidityPct{0.0f};
    bool roomAirValid{false};
    float floorTemperatureC{0.0f};
    bool floorTemperatureValid{false};
    float flowTemperatureC{0.0f};
    bool flowTemperatureValid{false};
};

struct DewPointOutputs {
    float dewPointC{0.0f};
    bool dewPointValid{false};
    /// Surface temperature minus dew point. Positive is safe; publishing it
    /// lets a visualisation show how much headroom is left rather than only
    /// that the alarm has already tripped.
    float marginK{0.0f};
    bool marginValid{false};
    bool alarm{false};
};

class DewPointMonitor {
public:
    void configure(const DewPointConfig& config) { config_ = config; }
    const DewPointConfig& config() const { return config_; }
    const DewPointOutputs& outputs() const { return outputs_; }

    void reset()
    {
        outputs_ = DewPointOutputs{};
    }

    const DewPointOutputs& update(const DewPointInputs& in)
    {
        outputs_.dewPointValid = in.roomAirValid;
        if (in.roomAirValid) {
            outputs_.dewPointC = psychro::dewPointC(in.roomTemperatureC, in.roomHumidityPct);
        }

        float surfaceC = 0.0f;
        bool surfaceValid = false;
        const bool useFloor = config_.surfaceSource == DewPointSurfaceSource::FloorProbe
                              || config_.surfaceSource == DewPointSurfaceSource::Coldest;
        const bool useFlow = config_.surfaceSource == DewPointSurfaceSource::FlowTemperature
                             || config_.surfaceSource == DewPointSurfaceSource::Coldest;

        if (useFloor && in.floorTemperatureValid) {
            surfaceC = in.floorTemperatureC;
            surfaceValid = true;
        }
        if (useFlow && in.flowTemperatureValid) {
            surfaceC = surfaceValid ? (in.flowTemperatureC < surfaceC ? in.flowTemperatureC : surfaceC)
                                    : in.flowTemperatureC;
            surfaceValid = true;
        }

        outputs_.marginValid = outputs_.dewPointValid && surfaceValid;
        if (!outputs_.marginValid) {
            // Nothing to compare. Hold the alarm rather than clearing it: a
            // probe that drops out mid-alarm has not made the water go away.
            outputs_.marginK = 0.0f;
            return outputs_;
        }

        outputs_.marginK = surfaceC - outputs_.dewPointC;
        if (outputs_.marginK <= config_.marginK) {
            outputs_.alarm = true;
        } else if (outputs_.marginK >= config_.marginK + config_.hysteresisK) {
            outputs_.alarm = false;
        }
        return outputs_;
    }

private:
    DewPointConfig config_{};
    DewPointOutputs outputs_{};
};

// ---------------------------------------------------------------------------
// Floor slab moisture monitor.
//
// The floor probe sits in a conduit inside the concrete slab, so its humidity
// channel — free with the SHT4x — sees the moisture state of the structure
// rather than the room. Two independent detectors, because each catches a
// different failure:
//
//   absolute threshold   slow, cumulative damp (a slab that never dried, or a
//                        leak that has already saturated the conduit)
//   excess over the room a leak in progress: liquid water evaporating into the
//                        conduit drives its absolute humidity above the room's
//                        long before the relative reading looks alarming,
//                        because the slab is also colder than the room
// ---------------------------------------------------------------------------

struct FloorMoistureConfig {
    /// Relative-humidity alarm threshold inside the slab. 0 disables.
    float thresholdPct{85.0f};
    float hysteresisPct{5.0f};
    /// Alarm when slab absolute humidity exceeds room absolute humidity by more
    /// than this, in g/m³. 0 disables.
    float absoluteExcessGm3{2.0f};
    float absoluteExcessHysteresisGm3{0.5f};
};

struct FloorMoistureInputs {
    float floorTemperatureC{0.0f};
    float floorHumidityPct{0.0f};
    bool floorProbeValid{false};
    float roomTemperatureC{0.0f};
    float roomHumidityPct{0.0f};
    bool roomAirValid{false};
};

struct FloorMoistureOutputs {
    float floorAbsoluteHumidityGm3{0.0f};
    float roomAbsoluteHumidityGm3{0.0f};
    float excessGm3{0.0f};
    bool excessValid{false};
    bool alarm{false};
    bool relativeAlarm{false};
    bool excessAlarm{false};
};

class FloorMoistureMonitor {
public:
    void configure(const FloorMoistureConfig& config) { config_ = config; }
    const FloorMoistureConfig& config() const { return config_; }
    const FloorMoistureOutputs& outputs() const { return outputs_; }

    void reset() { outputs_ = FloorMoistureOutputs{}; }

    const FloorMoistureOutputs& update(const FloorMoistureInputs& in)
    {
        if (in.floorProbeValid) {
            outputs_.floorAbsoluteHumidityGm3 =
                psychro::absoluteHumidityGm3(in.floorTemperatureC, in.floorHumidityPct);
        }
        if (in.roomAirValid) {
            outputs_.roomAbsoluteHumidityGm3 =
                psychro::absoluteHumidityGm3(in.roomTemperatureC, in.roomHumidityPct);
        }

        if (in.floorProbeValid && config_.thresholdPct > 0.0f) {
            if (in.floorHumidityPct >= config_.thresholdPct) {
                outputs_.relativeAlarm = true;
            } else if (in.floorHumidityPct <= config_.thresholdPct - config_.hysteresisPct) {
                outputs_.relativeAlarm = false;
            }
        } else if (!in.floorProbeValid) {
            outputs_.relativeAlarm = false;
        }

        outputs_.excessValid = in.floorProbeValid && in.roomAirValid;
        if (outputs_.excessValid && config_.absoluteExcessGm3 > 0.0f) {
            outputs_.excessGm3 =
                outputs_.floorAbsoluteHumidityGm3 - outputs_.roomAbsoluteHumidityGm3;
            if (outputs_.excessGm3 >= config_.absoluteExcessGm3) {
                outputs_.excessAlarm = true;
            } else if (outputs_.excessGm3
                       <= config_.absoluteExcessGm3 - config_.absoluteExcessHysteresisGm3) {
                outputs_.excessAlarm = false;
            }
        } else {
            if (!outputs_.excessValid) {
                outputs_.excessGm3 = 0.0f;
            }
            outputs_.excessAlarm = false;
        }

        outputs_.alarm = outputs_.relativeAlarm || outputs_.excessAlarm;
        return outputs_;
    }

private:
    FloorMoistureConfig config_{};
    FloorMoistureOutputs outputs_{};
};

// ---------------------------------------------------------------------------
// Ventilation / IAQ control: proportional CO2 + humidity + VOC demand, mode
// override, stage bucketing, IAQ status bitset.
// ---------------------------------------------------------------------------

struct VentilationConfig {
    // Proportional CO2 band: demand ramps 0 → 100 % across
    // [setpoint, setpoint + band]. A modern AHU or damper actuator takes a
    // continuous percentage, and a proportional ramp avoids the airflow (and
    // noise) steps a pure on/off boost produces.
    float co2SetpointPpm{900.0f};
    float co2BandPpm{400.0f};
    // Humidity band, same shape. Drives both the ventilation demand and the
    // separate dehumidification request.
    float humidityThresholdPct{65.0f};
    float humidityBandPct{15.0f};
    // BSEC IAQ index band (0..500, lower is cleaner). 0 disables the channel,
    // which is the right default on boards built without BSEC.
    float vocThresholdIndex{150.0f};
    float vocBandIndex{150.0f};
    // Floor of the automatic demand while the room is treated as occupied.
    uint8_t baseDemandPercent{0};
    // Fixed demand used in Manual mode.
    uint8_t manualDemandPercent{50};
};

struct VentilationInputs {
    float co2Ppm{0.0f};
    bool co2Valid{false};
    float humidityPct{0.0f};
    bool humidityValid{false};
    float vocIndex{0.0f};
    bool vocValid{false};
    bool occupied{false};
    VentilationMode mode{VentilationMode::Auto};
};

struct VentilationOutputs {
    uint8_t demandPercent{0};
    VentilationLevel level{VentilationLevel::Off};
    bool active{false};
    bool boostRequest{false};
    bool dehumidifyRequest{false};
    bool humidityHigh{false};
    bool co2High{false};
    bool vocHigh{false};
    bool sensorFault{false};
};

// IaqStatus bitset bits (HabinariPort::AirQualityStatus).
enum class IaqStatusBit : uint16_t {
    HumidityBoost = 1u << 0,
    Co2Boost = 1u << 1,
    SensorFault = 1u << 2,
    VocBoost = 1u << 3,
    DehumidifyRequest = 1u << 4,
};

class VentilationController {
public:
    void configure(const VentilationConfig& config) { config_ = config; }
    const VentilationConfig& config() const { return config_; }
    const VentilationOutputs& outputs() const { return outputs_; }

    const VentilationOutputs& update(const VentilationInputs& in)
    {
        const float co2Demand = in.co2Valid
                                    ? ramp(in.co2Ppm, config_.co2SetpointPpm, config_.co2BandPpm)
                                    : 0.0f;
        const float humidityDemand =
            in.humidityValid
                ? ramp(in.humidityPct, config_.humidityThresholdPct, config_.humidityBandPct)
                : 0.0f;
        const float vocDemand =
            (in.vocValid && config_.vocThresholdIndex > 0.0f)
                ? ramp(in.vocIndex, config_.vocThresholdIndex, config_.vocBandIndex)
                : 0.0f;

        // Worst channel wins: the point of measuring three pollutants is that
        // any one of them alone justifies air change.
        float autoDemand = co2Demand;
        if (humidityDemand > autoDemand) autoDemand = humidityDemand;
        if (vocDemand > autoDemand) autoDemand = vocDemand;
        if (in.occupied && autoDemand < static_cast<float>(config_.baseDemandPercent)) {
            autoDemand = static_cast<float>(config_.baseDemandPercent);
        }

        float demand = 0.0f;
        switch (in.mode) {
            case VentilationMode::Off:
                demand = 0.0f;
                break;
            case VentilationMode::Boost:
                demand = 100.0f;
                break;
            case VentilationMode::Manual:
                demand = static_cast<float>(config_.manualDemandPercent);
                break;
            case VentilationMode::Auto:
            default:
                demand = autoDemand;
                break;
        }

        outputs_.demandPercent = static_cast<uint8_t>(clampf(demand, 0.0f, 100.0f) + 0.5f);
        outputs_.active = outputs_.demandPercent > 0;
        outputs_.co2High = co2Demand > 0.0f;
        outputs_.humidityHigh = humidityDemand > 0.0f;
        outputs_.vocHigh = vocDemand > 0.0f;
        outputs_.boostRequest = outputs_.demandPercent >= 100;
        // Dehumidification is a distinct service from air change: it is what a
        // reversible heat pump or a dehumidifier acts on, and it must not be
        // asserted just because CO2 is high.
        outputs_.dehumidifyRequest = humidityDemand > 0.0f;
        outputs_.sensorFault = !in.co2Valid && !in.humidityValid;

        const uint8_t d = outputs_.demandPercent;
        if (d == 0) {
            outputs_.level = VentilationLevel::Off;
        } else if (d < 34) {
            outputs_.level = VentilationLevel::Low;
        } else if (d < 67) {
            outputs_.level = VentilationLevel::Medium;
        } else if (d < 100) {
            outputs_.level = VentilationLevel::High;
        } else {
            outputs_.level = VentilationLevel::Boost;
        }

        return outputs_;
    }

private:
    /// 0 % at or below `threshold`, 100 % at `threshold + band`, linear between.
    /// A zero or negative band degrades to the original on/off behaviour.
    static float ramp(float value, float threshold, float band)
    {
        if (value <= threshold) {
            return 0.0f;
        }
        if (band <= 0.0f) {
            return 100.0f;
        }
        return clampf(100.0f * (value - threshold) / band, 0.0f, 100.0f);
    }

    VentilationConfig config_{};
    VentilationOutputs outputs_{};
};

} // namespace hvac
} // namespace habinari
