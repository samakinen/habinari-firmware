#pragma once

#include <cstdint>

/**
 * @file control_defaults.hpp
 * @brief Factory defaults for every configurable value in the device.
 *
 * These used to live in knx_product.hpp, which made ETS the only thing that
 * could define what "default" means. That was fine while KNX was the only way
 * in; it is not fine now that the same control core is driven by Modbus, MQTT
 * and whatever comes next, because a device built without the KNX adapter would
 * have had no defaults at all.
 *
 * So the numbers live here, in a header with no dependency on any protocol
 * stack, and every configuration front end is a *view* of them:
 *
 *   knx_product.hpp   exports them as ETS parameter defaults (`using namespace`
 *                     below keeps every `kDefault...` reference there working)
 *   control_core.hpp  uses them as the member initialisers of Settings, which
 *                     is what an un-commissioned device actually runs on
 *
 * Changing a number here changes both, so the ETS catalogue entry and the
 * running firmware can never disagree about what a factory-fresh device does.
 *
 * `kParameterLayoutVersion` must be bumped whenever the *meaning* or order of
 * a stored parameter changes; see control_service.cpp for the migration hook.
 */

namespace sensor_board {
namespace config {

// ---------------------------------------------------------------------------
// Parameter defaults
//
// NOTE: integer-valued defaults on purpose. ETS parameter defaults are exported
// as text and some knxprod tooling parses them with the system locale, so
// separator-free integers parse identically everywhere. Fractional values stay
// fully settable by the integrator (DPT 9 parameters render as decimal editors).
// ---------------------------------------------------------------------------

inline constexpr uint16_t kParameterLayoutVersion = 5;

// --- Measurement publishing ------------------------------------------------
inline constexpr uint16_t kDefaultMeasurementHeartbeatSeconds = 900;  // KNX FB default
inline constexpr uint16_t kDefaultMeasurementMinRepTimeSeconds = 10;  // KNX FB default
inline constexpr float kDefaultRoomTemperatureOffsetK = 0.0f;
inline constexpr float kDefaultRoomTemperatureCovK = 0.2f;  // KNX FB recommended
inline constexpr float kDefaultRoomHumidityOffsetPct = 0.0f;
inline constexpr float kDefaultRoomHumidityCovPct = 2.0f;
inline constexpr uint16_t kDefaultCo2CovPpm = 25;
inline constexpr uint16_t kDefaultPressureCovPa = 50;
inline constexpr uint16_t kDefaultAirQualityCovIndex = 5;
inline constexpr float kDefaultFloorTemperatureOffsetK = 0.0f;
inline constexpr float kDefaultFloorTemperatureCovK = 0.2f;
inline constexpr float kDefaultFloorHumidityCovPct = 2.0f;
inline constexpr float kDefaultDerivedCovK = 0.2f;
inline constexpr uint16_t kDefaultAltitudeM = 0;

// --- Room control: general -------------------------------------------------
inline constexpr uint8_t kDefaultControllerEnable = 1;
inline constexpr uint8_t kDefaultHvacOperatingMode = 1;  // Dpt20Mode::Comfort
inline constexpr uint8_t kDefaultControllerMode = 0;     // ControllerMode::Auto
inline constexpr uint8_t kDefaultHeatingEnabled = 1;
inline constexpr uint8_t kDefaultCoolingEnabled = 0;
inline constexpr uint8_t kDefaultHeatCoolChangeoverMode = 0;      // Auto (internal deadband)
inline constexpr uint8_t kDefaultHeatCoolChangeoverPolarity = 0;  // Normal (bus 1 = heat)
inline constexpr uint16_t kDefaultMinimumHeatCoolChangeoverSeconds = 300;
inline constexpr uint8_t kDefaultWindowOpenBehavior = 2;  // BlockOutputs
inline constexpr uint8_t kDefaultPresenceBehavior = 0;    // Comfort/Standby
inline constexpr uint8_t kDefaultSensorFaultBehavior = 1;  // ForceOff

// --- Setpoint ladder (KNX RTSM model: comfort anchor + mode reductions) -----
inline constexpr float kDefaultComfortHeatingSetpointC = 21.0f;
inline constexpr float kDefaultStandbyHeatingReductionK = 2.0f;
inline constexpr float kDefaultEconomyHeatingReductionK = 4.0f;
inline constexpr float kDefaultProtectionHeatingSetpointC = 7.0f;
inline constexpr float kDefaultCoolingDeadbandK = 2.0f;
inline constexpr float kDefaultStandbyCoolingIncreaseK = 2.0f;
inline constexpr float kDefaultEconomyCoolingIncreaseK = 4.0f;
inline constexpr float kDefaultProtectionCoolingSetpointC = 35.0f;
inline constexpr float kDefaultMinSetpointC = 7.0f;
inline constexpr float kDefaultMaxSetpointC = 35.0f;
inline constexpr float kDefaultMaxSetpointShiftK = 3.0f;

// --- Heating / cooling loops ----------------------------------------------
// PID gain in %/K = 100 / proportional band; a 4 K band gives 25 %/K.
// Ti is the KNX "reset time"; 9000 s suits the thermal mass of a heated slab.
inline constexpr uint8_t kDefaultHeatingControlAlgorithm = 1;  // PI
inline constexpr float kDefaultHeatingKp = 25.0f;
inline constexpr uint16_t kDefaultHeatingTiSeconds = 9000;
inline constexpr uint16_t kDefaultHeatingTdSeconds = 0;
inline constexpr uint8_t kDefaultHeatingMinimumOutputPercent = 0;
inline constexpr uint8_t kDefaultHeatingMaximumOutputPercent = 100;
inline constexpr uint8_t kDefaultCoolingControlAlgorithm = 1;  // PI
inline constexpr float kDefaultCoolingKp = 25.0f;
inline constexpr uint16_t kDefaultCoolingTiSeconds = 9000;
inline constexpr uint16_t kDefaultCoolingTdSeconds = 0;
inline constexpr uint8_t kDefaultCoolingMinimumOutputPercent = 0;
inline constexpr uint8_t kDefaultCoolingMaximumOutputPercent = 100;
inline constexpr float kDefaultThermostatHysteresisC = 0.5f;
inline constexpr uint8_t kDefaultBinaryDemandStrategy = 0;  // Hysteresis
inline constexpr uint8_t kDefaultBinaryDemandThresholdPercent = 1;
inline constexpr float kDefaultFrostAlarmTemperatureC = 5.0f;
inline constexpr float kDefaultOverheatAlarmTemperatureC = 35.0f;

// --- Floor temperature -----------------------------------------------------
inline constexpr float kDefaultMaxFloorTemperatureC = 28.0f;  // 0 disables
inline constexpr float kDefaultMinFloorTemperatureC = 0.0f;   // 0 disables
inline constexpr float kDefaultFloorHysteresisK = 1.0f;
inline constexpr uint8_t kDefaultFloorComfortOutputPercent = 30;

// --- Condensation protection ----------------------------------------------
inline constexpr uint8_t kDefaultDewPointSurfaceSource = 3;  // Coldest available
inline constexpr float kDefaultDewPointMarginK = 2.0f;
inline constexpr float kDefaultDewPointHysteresisK = 1.0f;
inline constexpr uint8_t kDefaultBlockCoolingOnDewPointAlarm = 1;

// --- Slab moisture detection ----------------------------------------------
inline constexpr float kDefaultFloorMoistureThresholdPct = 85.0f;
inline constexpr float kDefaultFloorMoistureHysteresisPct = 5.0f;
inline constexpr float kDefaultFloorMoistureExcessGm3 = 2.0f;  // 0 disables

// --- Ventilation / air quality --------------------------------------------
inline constexpr uint16_t kDefaultVentilationSetpointPpm = 900;
inline constexpr uint16_t kDefaultVentilationBandPpm = 400;
inline constexpr float kDefaultHumidityBoostPct = 65.0f;
inline constexpr float kDefaultHumidityBandPct = 15.0f;
inline constexpr uint16_t kDefaultVocThresholdIndex = 150;  // 0 disables
inline constexpr uint16_t kDefaultVocBandIndex = 150;
inline constexpr uint8_t kDefaultVentilationBaseDemandPercent = 0;
inline constexpr uint8_t kDefaultVentilationManualDemandPercent = 50;

// --- Sensor fusion and derived events --------------------------------------
// The board has three sensors measuring room temperature and three measuring
// humidity. These parameters govern how those are combined, and what is
// inferred from how the combined signal moves. See sensor_fusion.hpp.
inline constexpr uint16_t kDefaultSensorFilterSeconds = 30;
inline constexpr float kDefaultTemperatureCrossCheckK = 1.5f;
inline constexpr float kDefaultHumidityCrossCheckPct = 6.0f;
inline constexpr float kDefaultFireRateOfRiseKPerMin = 4.0f;
inline constexpr float kDefaultFireAbsoluteTemperatureC = 55.0f;  // 0 disables
inline constexpr uint16_t kDefaultFireConfirmSeconds = 30;
inline constexpr uint8_t kDefaultFireRequireAirQuality = 0;
inline constexpr uint8_t kDefaultCo2OccupancyEnabled = 1;
inline constexpr uint8_t kDefaultWindowDetectEnabled = 1;

}  // namespace config
}  // namespace sensor_board
