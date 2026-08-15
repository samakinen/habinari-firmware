// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "control_defaults.hpp"
#include "hvac_control.hpp"
#include "sensor_data.h"

/**
 * @file control_core.hpp
 * @brief The room controller itself: settings in, measurements in, outputs out.
 *
 * This is the whole behaviour of the device, and it is deliberately the part
 * that knows nothing about how anyone talks to it. It used to live inside
 * knx_service.cpp, which meant a board built without the KNX adapter had no
 * control loop, no parameters and nothing to publish — the field bus was not a
 * way *into* the device, it *was* the device.
 *
 * Everything here is platform-free: no FreeRTOS, no ESP-IDF, no NVS, no clock.
 * `Core::tick()` is a pure function of (settings, inputs, measurements, dt) plus
 * the controller state it carries between calls, so the entire control model can
 * be driven at simulated time from a host test — which is what
 * main/test/test_control_core.cpp does.
 *
 * The three structs are the contract:
 *
 *   Settings   what the integrator configured, whatever configured it. ETS
 *              parameters, Modbus holding registers and MQTT retained config
 *              are three front ends onto this one struct.
 *   Inputs     what the installation is telling us right now — window contacts,
 *              presence, changeover, neighbour-device measurements.
 *   Outputs    everything the device has decided. Adapters publish from this and
 *              never recompute anything.
 *
 * Concurrency is somebody else's problem on purpose: control_service.cpp owns
 * the mutex, the task and the 1 Hz cadence. Nothing in this header locks.
 */

namespace habinari {
namespace control {

namespace hvac_ns = ::habinari::hvac;

// Factory defaults are the member initialisers below, so Settings{} is exactly
// what an un-commissioned device runs on.
using namespace ::habinari::config;

// ---------------------------------------------------------------------------
// Settings — the configured tuning. Snapshotted by the control tick each
// second, so a change from any configuration front end applies within one tick.
// ---------------------------------------------------------------------------
struct Settings {
    // --- Measurement reporting (KNX sensor FB Heartbeat/MinRepTime/COV) -----
    // Generic reporting policy, not a KNX one: "resend at least this often",
    // "never faster than this", "report when it has moved by this much". The
    // MQTT adapter uses the identical three knobs.
    uint32_t heartbeatSeconds{kDefaultMeasurementHeartbeatSeconds};
    uint32_t minRepTimeSeconds{kDefaultMeasurementMinRepTimeSeconds};
    float roomTemperatureOffsetK{kDefaultRoomTemperatureOffsetK};
    float roomTemperatureCovK{kDefaultRoomTemperatureCovK};
    float roomHumidityOffsetPct{kDefaultRoomHumidityOffsetPct};
    float roomHumidityCovPct{kDefaultRoomHumidityCovPct};
    float co2CovPpm{static_cast<float>(kDefaultCo2CovPpm)};
    float pressureCovPa{static_cast<float>(kDefaultPressureCovPa)};
    float airQualityCovIndex{static_cast<float>(kDefaultAirQualityCovIndex)};
    float floorTemperatureOffsetK{kDefaultFloorTemperatureOffsetK};
    float floorTemperatureCovK{kDefaultFloorTemperatureCovK};
    float floorHumidityCovPct{kDefaultFloorHumidityCovPct};
    float derivedCov{kDefaultDerivedCovK};
    float altitudeM{static_cast<float>(kDefaultAltitudeM)};

    // --- Room control: general ---
    bool controllerDefaultEnable{kDefaultControllerEnable != 0};
    hvac_ns::OperatingPreset defaultHvacOperatingMode{
        static_cast<hvac_ns::OperatingPreset>(kDefaultHvacOperatingMode)};
    hvac_ns::ControllerMode defaultControllerMode{
        static_cast<hvac_ns::ControllerMode>(kDefaultControllerMode)};
    bool heatingEnabled{kDefaultHeatingEnabled != 0};
    bool coolingEnabled{kDefaultCoolingEnabled != 0};
    hvac_ns::HeatCoolChangeoverMode heatCoolChangeoverMode{
        static_cast<hvac_ns::HeatCoolChangeoverMode>(kDefaultHeatCoolChangeoverMode)};
    bool heatCoolChangeoverPolarityInverted{kDefaultHeatCoolChangeoverPolarity != 0};
    float minimumHeatCoolChangeoverSeconds{
        static_cast<float>(kDefaultMinimumHeatCoolChangeoverSeconds)};
    hvac_ns::WindowOpenBehavior windowOpenBehavior{
        static_cast<hvac_ns::WindowOpenBehavior>(kDefaultWindowOpenBehavior)};
    hvac_ns::PresenceBehavior presenceBehavior{
        static_cast<hvac_ns::PresenceBehavior>(kDefaultPresenceBehavior)};
    hvac_ns::SensorFaultBehavior sensorFaultBehavior{
        static_cast<hvac_ns::SensorFaultBehavior>(kDefaultSensorFaultBehavior)};

    // --- Setpoint ladder ---
    float comfortHeatingSetpointC{kDefaultComfortHeatingSetpointC};
    float standbyHeatingReductionK{kDefaultStandbyHeatingReductionK};
    float economyHeatingReductionK{kDefaultEconomyHeatingReductionK};
    float protectionHeatingSetpointC{kDefaultProtectionHeatingSetpointC};
    float coolingDeadbandK{kDefaultCoolingDeadbandK};
    float standbyCoolingIncreaseK{kDefaultStandbyCoolingIncreaseK};
    float economyCoolingIncreaseK{kDefaultEconomyCoolingIncreaseK};
    float protectionCoolingSetpointC{kDefaultProtectionCoolingSetpointC};
    float minSetpointC{kDefaultMinSetpointC};
    float maxSetpointC{kDefaultMaxSetpointC};
    float maxSetpointShiftK{kDefaultMaxSetpointShiftK};

    // --- Heating / cooling loops ---
    hvac_ns::ControlAlgorithm heatingControlAlgorithm{
        static_cast<hvac_ns::ControlAlgorithm>(kDefaultHeatingControlAlgorithm)};
    float heatingKp{kDefaultHeatingKp};
    float heatingTiSeconds{static_cast<float>(kDefaultHeatingTiSeconds)};
    float heatingTdSeconds{static_cast<float>(kDefaultHeatingTdSeconds)};
    uint8_t heatingMinOutputPercent{kDefaultHeatingMinimumOutputPercent};
    uint8_t heatingMaxOutputPercent{kDefaultHeatingMaximumOutputPercent};
    hvac_ns::ControlAlgorithm coolingControlAlgorithm{
        static_cast<hvac_ns::ControlAlgorithm>(kDefaultCoolingControlAlgorithm)};
    float coolingKp{kDefaultCoolingKp};
    float coolingTiSeconds{static_cast<float>(kDefaultCoolingTiSeconds)};
    float coolingTdSeconds{static_cast<float>(kDefaultCoolingTdSeconds)};
    uint8_t coolingMinOutputPercent{kDefaultCoolingMinimumOutputPercent};
    uint8_t coolingMaxOutputPercent{kDefaultCoolingMaximumOutputPercent};
    float thermostatHysteresisC{kDefaultThermostatHysteresisC};
    hvac_ns::BinaryDemandStrategy binaryDemandStrategy{
        static_cast<hvac_ns::BinaryDemandStrategy>(kDefaultBinaryDemandStrategy)};
    uint8_t binaryDemandThresholdPercent{kDefaultBinaryDemandThresholdPercent};
    float frostAlarmTemperatureC{kDefaultFrostAlarmTemperatureC};
    float overheatAlarmTemperatureC{kDefaultOverheatAlarmTemperatureC};

    // --- Floor temperature ---
    float maxFloorTemperatureC{kDefaultMaxFloorTemperatureC};
    float minFloorTemperatureC{kDefaultMinFloorTemperatureC};
    float floorHysteresisK{kDefaultFloorHysteresisK};
    uint8_t floorComfortOutputPercent{kDefaultFloorComfortOutputPercent};

    // --- Condensation protection ---
    hvac_ns::DewPointSurfaceSource dewPointSurfaceSource{
        static_cast<hvac_ns::DewPointSurfaceSource>(kDefaultDewPointSurfaceSource)};
    float dewPointMarginK{kDefaultDewPointMarginK};
    float dewPointHysteresisK{kDefaultDewPointHysteresisK};
    bool blockCoolingOnDewPointAlarm{kDefaultBlockCoolingOnDewPointAlarm != 0};

    // --- Slab moisture ---
    float floorMoistureThresholdPct{kDefaultFloorMoistureThresholdPct};
    float floorMoistureHysteresisPct{kDefaultFloorMoistureHysteresisPct};
    float floorMoistureExcessGm3{kDefaultFloorMoistureExcessGm3};

    // --- Ventilation / air quality ---
    float ventilationSetpointPpm{static_cast<float>(kDefaultVentilationSetpointPpm)};
    float ventilationBandPpm{static_cast<float>(kDefaultVentilationBandPpm)};
    float humidityBoostPct{kDefaultHumidityBoostPct};
    float humidityBandPct{kDefaultHumidityBandPct};
    float vocThresholdIndex{static_cast<float>(kDefaultVocThresholdIndex)};
    float vocBandIndex{static_cast<float>(kDefaultVocBandIndex)};
    uint8_t ventilationBaseDemandPercent{kDefaultVentilationBaseDemandPercent};
    uint8_t ventilationManualDemandPercent{kDefaultVentilationManualDemandPercent};
};

// ---------------------------------------------------------------------------
// Inputs — what the installation is currently telling the device. Every field
// is written by some adapter and read by the tick; none of them is computed
// here. The `...Known` / `...Valid` companions exist so an object nobody linked
// stays absent rather than reading as a plausible zero.
// ---------------------------------------------------------------------------
struct Inputs {
    bool controllerOnOff{true};
    hvac_ns::OperatingPreset hvacOperatingMode{hvac_ns::OperatingPreset::Comfort};
    hvac_ns::ControllerMode controllerMode{hvac_ns::ControllerMode::Auto};
    bool changeOverStatus{false};
    bool windowOpen{false};
    bool windowStatusKnown{false};  ///< true once a window telegram has arrived
    bool presence{false};
    bool presenceKnown{false};  ///< true once a presence telegram has arrived
    bool switchHeat{true};
    bool switchCool{true};
    bool externalDewPointAlarm{false};
    float setpointShiftK{0.0f};
    hvac_ns::VentilationMode ventilationMode{hvac_ns::VentilationMode::Auto};

    // Neighbour-device measurements. Each stays invalid until a telegram
    // arrives, so an unlinked object never fabricates an outside temperature
    // of 0 °C and talk the controller into free cooling in January.
    float outsideTemperatureC{0.0f};
    bool outsideTemperatureValid{false};
    float outsideHumidityPct{0.0f};
    bool outsideHumidityValid{false};
    float flowTemperatureC{0.0f};
    bool flowTemperatureValid{false};
};

// ---------------------------------------------------------------------------
// Outputs — everything the tick decided. Adapters map this onto their wire
// format and never recompute anything, which is what keeps two field buses from
// disagreeing about what the device is doing.
// ---------------------------------------------------------------------------
struct Outputs {
    // Derived measurements.
    float roomDewPointC{0.0f};
    float roomAbsoluteHumidityGm3{0.0f};
    float floorAbsoluteHumidityGm3{0.0f};
    float seaLevelPressurePa{0.0f};
    float dewPointMarginK{0.0f};
    bool dewPointAlarm{false};
    bool floorMoistureAlarm{false};
    bool freeCoolingAvailable{false};
    bool freeDryingAvailable{false};

    // Thermostat.
    bool heatingRequest{false};
    bool coolingRequest{false};
    uint8_t heatingControlPercent{0};
    uint8_t coolingControlPercent{0};
    bool floorLimitActive{false};
    bool floorComfortActive{false};
    bool heatCoolModeHeating{true};  // DPT 1.100: 1 = heat
    bool enableHeat{false};
    bool enableCool{false};
    float activeSetpointC{kDefaultComfortHeatingSetpointC};
    float heatingSetpointC{kDefaultComfortHeatingSetpointC};
    float coolingSetpointC{kDefaultComfortHeatingSetpointC + kDefaultCoolingDeadbandK};
    float setpointShiftFeedbackK{0.0f};
    hvac_ns::OperatingPreset activePreset{hvac_ns::OperatingPreset::Comfort};
    hvac_ns::ControllerMode activeControllerMode{hvac_ns::ControllerMode::Auto};
    uint16_t controllerStatus{0};  // DPT 22.101 StatusRHCC

    // Ventilation.
    uint8_t ventilationDemandPercent{0};
    hvac_ns::VentilationLevel ventilationLevel{hvac_ns::VentilationLevel::Off};
    bool ventilationBoostRequest{false};
    bool dehumidifyRequest{false};
    uint16_t airQualityStatus{0};

    // Diagnostics.
    bool deviceFault{false};
    uint8_t roomSensorStatus{0};
    uint8_t floorProbeStatus{0};
    uint8_t airQualitySensorStatus{0};
    uint8_t sensorHealthMask{0};
    bool sensorDisagreement{false};
};

// ---------------------------------------------------------------------------
// Corrected sensor readings: the configured offsets are applied once, here, so
// every consumer (control loops, derived values, published measurements) sees
// the same number. Correcting only at the publish path would leave the
// controller working from an uncorrected temperature, which is exactly the
// reading the self-heating offset exists to fix.
// ---------------------------------------------------------------------------
struct CorrectedReadings {
    float roomTemperatureC{0.0f};
    bool roomTemperatureValid{false};
    float roomHumidityPct{0.0f};
    bool roomHumidityValid{false};
    float floorTemperatureC{0.0f};
    bool floorTemperatureValid{false};
    float floorHumidityPct{0.0f};
    bool floorHumidityValid{false};
    float co2Ppm{0.0f};
    bool co2Valid{false};
    float pressurePa{0.0f};
    bool pressureValid{false};
    float iaqIndex{0.0f};
    bool iaqValid{false};

    bool roomAirValid() const { return roomTemperatureValid && roomHumidityValid; }
    bool floorProbeValid() const { return floorTemperatureValid && floorHumidityValid; }
};

// The offsets applied here are the *installation* correction — the board
// against a reference thermometer on the wall it is mounted on. The corrections
// between the board's own sensors are a different quantity and are applied one
// layer down, in the fusion config, before the sources are compared with each
// other (see sensor_fusion_service.h).
inline CorrectedReadings correctReadings(const sensor_data_t &data, const Settings &settings)
{
    CorrectedReadings r;
    r.roomTemperatureValid = data.temperature.valid;
    r.roomTemperatureC = data.temperature.value + settings.roomTemperatureOffsetK;
    r.roomHumidityValid = data.humidity.valid;
    r.roomHumidityPct =
        hvac_ns::clampf(data.humidity.value + settings.roomHumidityOffsetPct, 0.0f, 100.0f);
    r.floorTemperatureValid = data.probe_temperature.valid;
    r.floorTemperatureC = data.probe_temperature.value + settings.floorTemperatureOffsetK;
    r.floorHumidityValid = data.probe_humidity.valid;
    r.floorHumidityPct = hvac_ns::clampf(data.probe_humidity.value, 0.0f, 100.0f);
    r.co2Valid = data.co2.valid;
    r.co2Ppm = data.co2.value;
    r.pressureValid = data.pressure.valid;
    r.pressurePa = data.pressure.value;
    r.iaqValid = data.iaq.valid;
    r.iaqIndex = data.iaq.value;
    return r;
}

// ---------------------------------------------------------------------------
// Core — the controllers plus the one function that drives them.
//
// The controller objects hold state between ticks (PID integrators, hysteresis
// latches, changeover timers), which is why this is a class and not a free
// function. It is owned by exactly one task; it does no locking of its own.
// ---------------------------------------------------------------------------
class Core {
public:
    /**
     * Run one control cycle.
     *
     * Pure with respect to the outside world: it reads nothing but its
     * arguments and writes nothing but its return value and its own controller
     * state. All the I/O the old in-KNX version did inline — reconfiguring the
     * fusion layer, acknowledging alarms — is the caller's job now, which is
     * what makes this callable from a host test.
     *
     * @param settings    configured tuning, as of this instant
     * @param inputs      installation inputs, as of this instant
     * @param sensors     latest fused measurement record
     * @param dtSeconds   elapsed time since the previous tick
     */
    Outputs tick(const Settings &settings, const Inputs &inputs, const sensor_data_t &sensors,
                 float dtSeconds)
    {
        const CorrectedReadings readings = correctReadings(sensors, settings);

        hvac_ns::ThermostatInputs thermostatIn;
        thermostatIn.controllerEnable = inputs.controllerOnOff;
        thermostatIn.hvacOperatingMode = inputs.hvacOperatingMode;
        thermostatIn.controllerMode = inputs.controllerMode;
        thermostatIn.heatCoolChangeoverBusValue = inputs.changeOverStatus;
        thermostatIn.windowOpen = inputs.windowOpen;
        thermostatIn.presence = inputs.presence;
        thermostatIn.presenceValid = inputs.presenceKnown;
        thermostatIn.setpointShiftK = inputs.setpointShiftK;
        thermostatIn.switchHeat = inputs.switchHeat;
        thermostatIn.switchCool = inputs.switchCool;
        thermostatIn.flowTemperatureC = inputs.flowTemperatureC;
        thermostatIn.flowTemperatureValid = inputs.flowTemperatureValid;
        thermostatIn.dewPointAlarm = inputs.externalDewPointAlarm;
        thermostatIn.roomTemperatureC = readings.roomTemperatureC;
        thermostatIn.roomTemperatureValid = readings.roomTemperatureValid;
        thermostatIn.floorTemperatureC = readings.floorTemperatureC;
        thermostatIn.floorTemperatureValid = readings.floorTemperatureValid;

        hvac_ns::VentilationInputs ventIn;
        ventIn.mode = inputs.ventilationMode;
        ventIn.occupied = inputs.presenceKnown ? inputs.presence : false;

        // Derived room events stand in for the bus inputs that were never
        // linked. A real contact or PIR always wins: once a telegram has
        // arrived on those objects the installation has the real thing, and an
        // inference must not override it. Where nothing is linked, this is the
        // difference between heating into an open window and noticing.
        if (!inputs.windowStatusKnown && sensors.events.window_open_detected) {
            thermostatIn.windowOpen = true;
        }
        if (!inputs.presenceKnown && sensors.events.occupancy_detected) {
            thermostatIn.presence = true;
            thermostatIn.presenceValid = true;
            ventIn.occupied = true;
        }
        ventIn.co2Ppm = readings.co2Ppm;
        ventIn.co2Valid = readings.co2Valid;
        ventIn.humidityPct = readings.roomHumidityPct;
        ventIn.humidityValid = readings.roomHumidityValid;
        ventIn.vocIndex = readings.iaqIndex;
        ventIn.vocValid = readings.iaqValid;

        // --- Derived values and condensation / moisture monitors -------------
        dewPoint_.configure({.surfaceSource = settings.dewPointSurfaceSource,
                             .marginK = settings.dewPointMarginK,
                             .hysteresisK = settings.dewPointHysteresisK});
        const auto dewOut = dewPoint_.update({
            .roomTemperatureC = readings.roomTemperatureC,
            .roomHumidityPct = readings.roomHumidityPct,
            .roomAirValid = readings.roomAirValid(),
            .floorTemperatureC = readings.floorTemperatureC,
            .floorTemperatureValid = readings.floorTemperatureValid,
            .flowTemperatureC = thermostatIn.flowTemperatureC,
            .flowTemperatureValid = thermostatIn.flowTemperatureValid,
        });

        floorMoisture_.configure({.thresholdPct = settings.floorMoistureThresholdPct,
                                  .hysteresisPct = settings.floorMoistureHysteresisPct,
                                  .absoluteExcessGm3 = settings.floorMoistureExcessGm3});
        const auto moistureOut = floorMoisture_.update({
            .floorTemperatureC = readings.floorTemperatureC,
            .floorHumidityPct = readings.floorHumidityPct,
            .floorProbeValid = readings.floorProbeValid(),
            .roomTemperatureC = readings.roomTemperatureC,
            .roomHumidityPct = readings.roomHumidityPct,
            .roomAirValid = readings.roomAirValid(),
        });

        // A locally derived condensation risk carries the same weight as one
        // received on the DewPointStatus input; either blocks cooling.
        thermostatIn.dewPointAlarm = thermostatIn.dewPointAlarm || dewOut.alarm;

        // --- Thermostat ------------------------------------------------------
        // Reconfigure from the settings every tick: configure() preserves
        // controller state (integrators, latches), so this is cheap and makes a
        // live re-parameterisation take effect within one tick.
        hvac_ns::ThermostatConfig thermostatCfg;
        thermostatCfg.heatingPid = {.kp = settings.heatingKp,
                                    .tiSeconds = settings.heatingTiSeconds,
                                    .tdSeconds = settings.heatingTdSeconds};
        thermostatCfg.coolingPid = {.kp = settings.coolingKp,
                                    .tiSeconds = settings.coolingTiSeconds,
                                    .tdSeconds = settings.coolingTdSeconds,
                                    .reverseActing = true};
        thermostatCfg.setpoints = {
            .comfortHeatingC = settings.comfortHeatingSetpointC,
            .standbyHeatingReductionK = settings.standbyHeatingReductionK,
            .economyHeatingReductionK = settings.economyHeatingReductionK,
            .protectionHeatingC = settings.protectionHeatingSetpointC,
            .coolingDeadbandK = settings.coolingDeadbandK,
            .standbyCoolingIncreaseK = settings.standbyCoolingIncreaseK,
            .economyCoolingIncreaseK = settings.economyCoolingIncreaseK,
            .protectionCoolingC = settings.protectionCoolingSetpointC,
        };
        thermostatCfg.minSetpointC = settings.minSetpointC;
        thermostatCfg.maxSetpointC = settings.maxSetpointC;
        thermostatCfg.maxSetpointShiftK = settings.maxSetpointShiftK;
        thermostatCfg.requestHysteresisK = settings.thermostatHysteresisC;
        thermostatCfg.maxFloorTemperatureC = settings.maxFloorTemperatureC;
        thermostatCfg.minFloorTemperatureC = settings.minFloorTemperatureC;
        thermostatCfg.floorHysteresisK = settings.floorHysteresisK;
        thermostatCfg.floorComfortOutputPercent = settings.floorComfortOutputPercent;
        thermostatCfg.frostAlarmTemperatureC = settings.frostAlarmTemperatureC;
        thermostatCfg.overheatAlarmTemperatureC = settings.overheatAlarmTemperatureC;
        thermostatCfg.blockCoolingOnDewPointAlarm = settings.blockCoolingOnDewPointAlarm;
        thermostatCfg.heatingEnabled = settings.heatingEnabled;
        thermostatCfg.coolingEnabled = settings.coolingEnabled;
        thermostatCfg.heatCoolChangeoverMode = settings.heatCoolChangeoverMode;
        thermostatCfg.heatCoolChangeoverPolarityInverted =
            settings.heatCoolChangeoverPolarityInverted;
        thermostatCfg.heatingAlgorithm = settings.heatingControlAlgorithm;
        thermostatCfg.coolingAlgorithm = settings.coolingControlAlgorithm;
        thermostatCfg.heatingMinOutputPercent = settings.heatingMinOutputPercent;
        thermostatCfg.heatingMaxOutputPercent = settings.heatingMaxOutputPercent;
        thermostatCfg.coolingMinOutputPercent = settings.coolingMinOutputPercent;
        thermostatCfg.coolingMaxOutputPercent = settings.coolingMaxOutputPercent;
        thermostatCfg.binaryDemandStrategy = settings.binaryDemandStrategy;
        thermostatCfg.binaryDemandThresholdPercent = settings.binaryDemandThresholdPercent;
        thermostatCfg.minimumHeatCoolChangeoverSeconds = settings.minimumHeatCoolChangeoverSeconds;
        thermostatCfg.windowOpenBehavior = settings.windowOpenBehavior;
        thermostatCfg.presenceBehavior = settings.presenceBehavior;
        thermostatCfg.sensorFaultBehavior = settings.sensorFaultBehavior;
        thermostat_.configure(thermostatCfg);

        ventilation_.configure({.co2SetpointPpm = settings.ventilationSetpointPpm,
                                .co2BandPpm = settings.ventilationBandPpm,
                                .humidityThresholdPct = settings.humidityBoostPct,
                                .humidityBandPct = settings.humidityBandPct,
                                .vocThresholdIndex = settings.vocThresholdIndex,
                                .vocBandIndex = settings.vocBandIndex,
                                .baseDemandPercent = settings.ventilationBaseDemandPercent,
                                .manualDemandPercent = settings.ventilationManualDemandPercent});

        const auto out = thermostat_.update(thermostatIn, dtSeconds);
        const auto ventOut = ventilation_.update(ventIn);

        // --- Outside-air opportunity flags -----------------------------------
        // Both compare like with like: temperature for cooling, absolute
        // humidity for drying. Comparing relative humidities would say cold
        // outdoor air is "wetter" than warm indoor air, which is backwards.
        Outputs computed;
        computed.freeCoolingAvailable =
            inputs.outsideTemperatureValid && readings.roomTemperatureValid
            && inputs.outsideTemperatureC < readings.roomTemperatureC - 1.0f
            && readings.roomTemperatureC > out.coolingSetpointC;
        if (inputs.outsideTemperatureValid && inputs.outsideHumidityValid
            && readings.roomAirValid()) {
            const float outsideAbs = hvac_ns::psychro::absoluteHumidityGm3(
                inputs.outsideTemperatureC, inputs.outsideHumidityPct);
            computed.freeDryingAvailable = outsideAbs < moistureOut.roomAbsoluteHumidityGm3 - 0.5f;
        }

        computed.roomDewPointC = dewOut.dewPointC;
        computed.roomAbsoluteHumidityGm3 = moistureOut.roomAbsoluteHumidityGm3;
        computed.floorAbsoluteHumidityGm3 = moistureOut.floorAbsoluteHumidityGm3;
        computed.dewPointMarginK = dewOut.marginK;
        computed.dewPointAlarm = thermostatIn.dewPointAlarm;
        computed.floorMoistureAlarm = moistureOut.alarm;
        computed.seaLevelPressurePa =
            readings.pressureValid
                ? hvac_ns::psychro::seaLevelPressurePa(readings.pressurePa, settings.altitudeM,
                                                       readings.roomTemperatureC)
                : 0.0f;

        computed.heatingRequest = out.heatingRequest;
        computed.coolingRequest = out.coolingRequest;
        computed.heatingControlPercent = out.heatingControlPercent;
        computed.coolingControlPercent = out.coolingControlPercent;
        computed.floorLimitActive = out.floorLimitActive;
        computed.floorComfortActive = out.floorComfortActive;
        // DPT 1.100 has no "neutral": the spec's own StatusRHCC wording treats
        // heating as the default, so only active cooling reports 0.
        computed.heatCoolModeHeating = out.heatCoolState != hvac_ns::HeatCoolState::Cooling;
        computed.enableHeat = !out.heatingBlocked;
        computed.enableCool = !out.coolingBlocked;
        computed.activeSetpointC = out.activeSetpointC;
        computed.heatingSetpointC = out.heatingSetpointC;
        computed.coolingSetpointC = out.coolingSetpointC;
        computed.setpointShiftFeedbackK = out.setpointShiftFeedbackK;
        computed.activePreset = out.activePreset;
        // ContrModeAct / ContrModeSecondary report what the controller resolved
        // to, not what was requested: a Secondary RTC in the same room needs the
        // decision, and Auto would tell it nothing.
        computed.activeControllerMode =
            out.heatCoolState == hvac_ns::HeatCoolState::Heating   ? hvac_ns::ControllerMode::Heat
            : out.heatCoolState == hvac_ns::HeatCoolState::Cooling ? hvac_ns::ControllerMode::Cool
            : (out.heatingBlocked && out.coolingBlocked)           ? hvac_ns::ControllerMode::Off
                                                                  : hvac_ns::ControllerMode::Auto;
        computed.controllerStatus = hvac_ns::packStatusRHCC(out);

        computed.ventilationDemandPercent = ventOut.demandPercent;
        computed.ventilationLevel = ventOut.level;
        computed.ventilationBoostRequest = ventOut.boostRequest;
        computed.dehumidifyRequest = ventOut.dehumidifyRequest;

        uint16_t iaqBits = 0;
        if (ventOut.humidityHigh) {
            iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::HumidityBoost);
        }
        if (ventOut.co2High) iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::Co2Boost);
        if (ventOut.vocHigh) iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::VocBoost);
        if (ventOut.sensorFault) {
            iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::SensorFault);
        }
        if (ventOut.dehumidifyRequest) {
            iaqBits |= static_cast<uint16_t>(hvac_ns::IaqStatusBit::DehumidifyRequest);
        }
        computed.airQualityStatus = iaqBits;

        // Per-package StatusGen octets (DPT 21.001). "Never delivered a reading"
        // is reported as OutOfService rather than Fault: an installation without
        // the optional floor probe is correctly configured, not broken. The
        // InAlarm bit also carries a cross-check failure, because a sensor that
        // disagrees with its peers is faulty even while it answers perfectly.
        const bool anySensorData = sensors.health.sample_count > 0;
        const bool roomAirDisputed =
            sensors.temperature.disagreement || sensors.humidity.disagreement;
        computed.sensorDisagreement = roomAirDisputed;
        computed.sensorHealthMask = sensors.health.healthy_mask;
        computed.roomSensorStatus =
            hvac_ns::packStatusGen(anySensorData, readings.roomAirValid(), roomAirDisputed);
        computed.floorProbeStatus = hvac_ns::packStatusGen(
            readings.floorTemperatureValid || readings.floorHumidityValid,
            readings.floorProbeValid(), moistureOut.alarm);
        computed.airQualitySensorStatus =
            hvac_ns::packStatusGen(anySensorData, readings.co2Valid || readings.iaqValid);
        // The roll-up alarm covers only what stops the device doing its job: the
        // room sensor the control loops depend on. A fallback source keeping the
        // room measured is explicitly NOT a device fault — that is the
        // redundancy working — but losing every source, or a confirmed fire, is.
        computed.deviceFault = !readings.roomAirValid() || sensors.events.fire_alarm;

        return computed;
    }

private:
    hvac_ns::ThermostatController thermostat_;
    hvac_ns::VentilationController ventilation_;
    hvac_ns::DewPointMonitor dewPoint_;
    hvac_ns::FloorMoistureMonitor floorMoisture_;
};

}  // namespace control
}  // namespace habinari
