#pragma once

#include "knx/product/commissioned_product.hpp"

#include <cstdint>

/**
 * @file knx_product.hpp
 * @brief ETS product model for the room HVAC sensor/controller board.
 *
 * The board is modelled as a KNX S-Mode room heating/cooling controller with an
 * integrated sensor package, following "The KNX Standard v3.0.0", Volume 7/19/20
 * "S-Mode HVAC FBs" (v01.03.01). The group objects below are grouped by the
 * standard Functional Blocks they realise, so the device binds to any
 * conformant partner without a product-specific integration note:
 *
 *   RTS   Room Temperature Sensor            clause 3.1   HDC3020
 *   RRHS  Room Relative Humidity Sensor      7/10/1 3.20  HDC3020
 *   RAQS  Room Air Quality Sensor            7/10/1 3.14  SCD4x / BME688
 *   FTS   Floor Temperature Sensor           clause 3.2   SHT4x in-slab probe
 *   DPS   Dew Point Status Sensor            clause 3.4   derived on board
 *   RTSM  Room Temperature Setpoint Manager  clause 5     (setpoint ladder)
 *   RTC   Room Temperature Controller        clause 6     (heat/cool loops)
 *
 * Two deliberate deviations from the standard FB model, both in the interest of
 * the "wide compatibility" goal rather than against it:
 *
 *   - The four per-mode setpoints are NOT published as DPT_TempRoomSetpSetF16[4]
 *     (275.100). That DPT is referenced by the S-Mode FB standard but is only
 *     defined in AN188, which is still a Draft for Voting and is absent from the
 *     ratified Datapoint Types chapter (3/7/2 v02.02.01) shipped in the same
 *     v3.0.0 release. Individual DPT 9.001 setpoint objects are what shipping
 *     room controllers and Home Assistant's KNX climate platform actually speak.
 *   - Fan/ventilation staging uses DPT 5.010 rather than DPT_FanStage (5.100),
 *     for the same reason: 5.100 is not in the ratified DPT list. The primary
 *     ventilation signal is a DPT 5.001 percentage, which every AHU accepts.
 *
 * Prototype/greenfield: port numbers and parameter ids are free to be renumbered
 * as the model evolves. The knxprod is regenerated from this header on every
 * build (see cmake/ets_export.cmake) and re-imported into ETS; no old-layout
 * migration is kept. Bump kParameterLayoutVersion when the byte layout changes.
 */

namespace sensor_board_knx {

using namespace knx;
using namespace knx::application;
using namespace knx::product;

// ---------------------------------------------------------------------------
// Parameter defaults
//
// NOTE: integer-valued defaults on purpose. ETS parameter defaults are exported
// as text and some knxprod tooling parses them with the system locale, so
// separator-free integers parse identically everywhere. Fractional values stay
// fully settable by the integrator (DPT 9 parameters render as decimal editors).
// ---------------------------------------------------------------------------

inline constexpr uint16_t kParameterLayoutVersion = 4;

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

// ---------------------------------------------------------------------------
// KNX group-object (logical port) identity.
// ---------------------------------------------------------------------------

enum class SensorBoardPort : uint16_t {
    // --- Room air sensors: FB RTS / RRHS / RAQS ---------------------------
    RoomTemperature = 0,          // RTS.TempRoom, DPT 9.001
    RoomHumidity = 1,             // RRHS.HumRelRoom, DPT 9.007
    RoomCo2 = 2,                  // RAQS.AQRoom (SCD4x true CO2), DPT 9.008
    RoomAirPressure = 3,          // station pressure as measured, DPT 9.006
    RoomAirPressureSeaLevel = 4,  // altitude-reduced, DPT 9.006
    RoomAirQualityIndex = 5,      // BME688/BSEC IAQ 0..500 (lower = cleaner)
    RoomCo2Equivalent = 6,        // BSEC CO2-equivalent, DPT 9.008
    RoomVocEquivalent = 7,        // BSEC breath-VOC-equivalent, DPT 9.008
    AirQualityAccuracy = 8,       // BSEC calibration status 0..3

    // --- Derived room air values ------------------------------------------
    RoomDewPoint = 9,             // DPT 9.001 — condensation reference
    RoomAbsoluteHumidity = 10,    // DPT 9.029 — moisture content, g/m³

    // --- Floor probe: FB FTS + slab moisture -------------------------------
    FloorTemperature = 11,        // FTS.TempFloor, DPT 9.001
    FloorHumidity = 12,           // in-conduit RH, DPT 9.007
    FloorAbsoluteHumidity = 13,   // DPT 9.029 — comparable with the room value
    FloorMoistureAlarm = 14,      // DPT 1.005 — damp/leak in the slab
    FloorLimitActive = 15,        // DPT 1.011 — max floor temperature engaged
    FloorComfortActive = 16,      // DPT 1.011 — min floor temperature engaged

    // --- Condensation protection: FB DPS -----------------------------------
    DewPointAlarm = 17,           // DPS.DewPointStatus out, DPT 1.005
    DewPointMargin = 18,          // surface − dew point, DPT 9.002
    DewPointStatusInput = 19,     // RTC.DewPointStatus in, DPT 1.005

    // --- System / neighbour inputs -----------------------------------------
    OutsideTemperature = 20,      // OTS.TempOutside in, DPT 9.001
    OutsideHumidity = 21,         // ORHS in, DPT 9.007
    FlowTemperature = 22,         // FWTS.TempFlowWater in, DPT 9.001
    FreeCoolingAvailable = 23,    // DPT 1.011 — outside air can cool this room
    FreeDryingAvailable = 24,     // DPT 1.011 — outside air is drier than inside

    // --- Mode and setpoints: FB RTSM ---------------------------------------
    ControllerOnOff = 25,         // DPT 1.001, in/out (HA on_off_address)
    HvacMode = 26,                // RTSM.HVACMode in, DPT 20.102
    HvacModeStatus = 27,          // RTC.HVACModeAct out, DPT 20.102
    ContrMode = 28,               // RTC.ContrMode in, DPT 20.105
    ContrModeStatus = 29,         // RTC.ContrModeAct out, DPT 20.105
    ContrModeSecondary = 30,      // RTC.ContrModeSecondary out, DPT 20.105
    SetpointBase = 31,            // RTSM.TempRoomSetpUserAbs, DPT 9.001, in/out
    SetpointShift = 32,           // RTSM.TempRoomSetpUserOffset in, DPT 9.002
    SetpointShiftStatus = 33,     // ...UserOffsetEff out, DPT 9.002
    SetpointStatus = 34,          // RTC.TempRoomSetpAct out, DPT 9.001
    SetpointHeatingStatus = 35,   // RTSM.TempRoomSetpHeatEff out, DPT 9.001
    SetpointCoolingStatus = 36,   // RTSM.TempRoomSetpCoolEff out, DPT 9.001

    // --- Room inputs: FB WOS / PRD / WCOS ----------------------------------
    WindowStatus = 37,            // WOS.WindowStatus in, DPT 1.019
    PresenceStatus = 38,          // PRD.PresenceStatus in, DPT 1.018
    SwitchHeat = 39,              // RTC.SwitchHeat in, DPT 1.003
    SwitchCool = 40,              // RTC.SwitchCool in, DPT 1.003
    ChangeOverStatus = 41,        // WCOS.ChangeOverStatus in, DPT 1.100

    // --- Controller outputs: FB RTC ----------------------------------------
    HeatingControlValue = 42,     // RTC.ActTargetPosHeatStageA, DPT 5.001
    CoolingControlValue = 43,     // RTC.ActTargetPosCoolStageA, DPT 5.001
    HeatingRequest = 44,          // binary heat demand, DPT 1.001
    CoolingRequest = 45,          // binary cool demand, DPT 1.001
    HeatCoolModeStatus = 46,      // RTC.HeatCoolModeAct out, DPT 1.100
    EnableHeatStatus = 47,        // RTC.EnableHeatEff out, DPT 1.003
    EnableCoolStatus = 48,        // RTC.EnableCoolEff out, DPT 1.003
    ControllerStatus = 49,        // RTC.CombinedStatus_RTC, DPT 22.101

    // --- Ventilation / air quality ------------------------------------------
    Co2Setpoint = 50,             // AQSetp, DPT 9.008, in/out
    VentilationDemand = 51,       // continuous demand, DPT 5.001
    VentilationStage = 52,        // 0=Off,1=Low,2=Medium,3=High,4=Boost
    VentilationMode = 53,         // 0=Auto,1=Manual,2=Off,3=Boost, in/out
    VentilationBoostRequest = 54, // DPT 1.001
    DehumidifyRequest = 55,       // DPT 1.001
    AirQualityStatus = 56,        // bitset, see IaqStatusBit

    // --- Device diagnostics --------------------------------------------------
    DeviceFault = 57,             // roll-up alarm, DPT 1.005
    RoomSensorStatus = 58,        // HDC3020 StatusGO, DPT 21.001
    FloorProbeStatus = 59,        // SHT4x StatusGO, DPT 21.001
    AirQualitySensorStatus = 60,  // SCD4x + BME688 StatusGO, DPT 21.001
};

// ---------------------------------------------------------------------------
// ETS parameter identity. Declaration order in makeParameterSchema below
// defines the ProgramData / RS-0000 byte layout.
// ---------------------------------------------------------------------------

enum class SensorBoardParameter : uint16_t {
    ParameterLayoutVersion = 0,

    // Measurements
    MeasurementHeartbeatSeconds = 1,
    MeasurementMinRepTimeSeconds = 2,
    RoomTemperatureOffset = 3,
    RoomTemperatureCov = 4,
    RoomHumidityOffset = 5,
    RoomHumidityCov = 6,
    Co2Cov = 7,
    PressureCov = 8,
    AirQualityCov = 9,
    FloorTemperatureOffset = 10,
    FloorTemperatureCov = 11,
    FloorHumidityCov = 12,
    DerivedValueCov = 13,
    AltitudeM = 14,

    // Room control: general
    ControllerDefaultEnable = 15,
    DefaultHvacOperatingMode = 16,
    DefaultControllerMode = 17,
    HeatingEnabled = 18,
    CoolingEnabled = 19,
    HeatCoolChangeoverMode = 20,
    HeatCoolChangeoverPolarity = 21,
    MinimumHeatCoolChangeoverSeconds = 22,
    WindowOpenBehavior = 23,
    PresenceBehavior = 24,
    SensorFaultBehavior = 25,

    // Setpoints
    ComfortHeatingSetpoint = 26,
    StandbyHeatingReduction = 27,
    EconomyHeatingReduction = 28,
    ProtectionHeatingSetpoint = 29,
    CoolingDeadband = 30,
    StandbyCoolingIncrease = 31,
    EconomyCoolingIncrease = 32,
    ProtectionCoolingSetpoint = 33,
    MinSetpoint = 34,
    MaxSetpoint = 35,
    MaxSetpointShift = 36,

    // Heating loop
    HeatingControlAlgorithm = 37,
    HeatingKp = 38,
    HeatingTiSeconds = 39,
    HeatingTdSeconds = 40,
    HeatingMinimumOutputPercent = 41,
    HeatingMaximumOutputPercent = 42,

    // Cooling loop
    CoolingControlAlgorithm = 43,
    CoolingKp = 44,
    CoolingTiSeconds = 45,
    CoolingTdSeconds = 46,
    CoolingMinimumOutputPercent = 47,
    CoolingMaximumOutputPercent = 48,

    // Shared loop behaviour
    ThermostatHysteresis = 49,
    BinaryDemandStrategy = 50,
    BinaryDemandThresholdPercent = 51,
    FrostAlarmTemperature = 52,
    OverheatAlarmTemperature = 53,

    // Floor temperature
    MaxFloorTemperature = 54,
    MinFloorTemperature = 55,
    FloorHysteresis = 56,
    FloorComfortOutputPercent = 57,

    // Condensation protection
    DewPointSurfaceSource = 58,
    DewPointMargin = 59,
    DewPointHysteresis = 60,
    BlockCoolingOnDewPointAlarm = 61,

    // Slab moisture
    FloorMoistureThreshold = 62,
    FloorMoistureHysteresis = 63,
    FloorMoistureExcess = 64,

    // Ventilation / air quality
    VentilationSetpoint = 65,
    VentilationBand = 66,
    HumidityBoostThreshold = 67,
    HumidityBoostBand = 68,
    VocBoostThreshold = 69,
    VocBoostBand = 70,
    VentilationBaseDemandPercent = 71,
    VentilationManualDemandPercent = 72,
};

// Shorthand for the visibility conditions below: ETS hides a parameter unless
// the named parameter currently holds the given value.
constexpr uint16_t paramId(SensorBoardParameter id)
{
    return static_cast<uint16_t>(id);
}

// ETS section headings. Named once so a typo cannot silently split a block.
inline constexpr std::string_view kGroupMeasurement = "Measurements and sending behaviour";
inline constexpr std::string_view kGroupControlGeneral = "Room control - general";
inline constexpr std::string_view kGroupSetpoints = "Setpoints";
inline constexpr std::string_view kGroupHeating = "Heating control";
inline constexpr std::string_view kGroupCooling = "Cooling control";
inline constexpr std::string_view kGroupLoop = "Control loop behaviour";
inline constexpr std::string_view kGroupFloor = "Floor temperature";
inline constexpr std::string_view kGroupDewPoint = "Condensation protection";
inline constexpr std::string_view kGroupMoisture = "Slab moisture detection";
inline constexpr std::string_view kGroupVentilation = "Ventilation and air quality";
inline constexpr std::string_view kGroupDevice = "Device";

inline constexpr auto kSensorBoardProduct =
    makeCommissionedProduct(
        makeEndpointDefinition<
            SensorBoardPort,
            // ---- FB RTS / RRHS / RAQS: room air measurements ---------------
            endpoint::semantics::TemperatureState<SensorBoardPort::RoomTemperature,
                                                  "room_temperature",
                                                  "Room Air Temperature",
                                                  false>,
            endpoint::semantics::HumidityState<SensorBoardPort::RoomHumidity,
                                               "room_humidity",
                                               "Room Relative Humidity",
                                               false>,
            endpoint::semantics::Co2State<SensorBoardPort::RoomCo2,
                                          "room_co2",
                                          "Room CO2 (SCD4x)",
                                          false>,
            endpoint::StatePort<SensorBoardPort::RoomAirPressure,
                                float,
                                "room_air_pressure",
                                "Air Pressure (station)",
                                application::dptids::Pressure,
                                false>,
            // Only the sea-level-reduced value is comparable with a forecast or
            // with another site, which is what a visualisation actually wants.
            endpoint::StatePort<SensorBoardPort::RoomAirPressureSeaLevel,
                                float,
                                "room_air_pressure_sea_level",
                                "Air Pressure (sea level)",
                                application::dptids::Pressure,
                                false>,
            endpoint::StatePort<SensorBoardPort::RoomAirQualityIndex,
                                uint16_t,
                                "room_air_quality_index",
                                "Air Quality Index (BSEC IAQ 0..500)",
                                application::dptids::Counter16,
                                false>,
            endpoint::StatePort<SensorBoardPort::RoomCo2Equivalent,
                                float,
                                "room_co2_equivalent",
                                "CO2 Equivalent (BSEC)",
                                application::dptids::CO2,
                                false>,
            endpoint::StatePort<SensorBoardPort::RoomVocEquivalent,
                                float,
                                "room_voc_equivalent",
                                "Breath-VOC Equivalent (BSEC)",
                                application::dptids::CO2,
                                false>,
            endpoint::StatePort<SensorBoardPort::AirQualityAccuracy,
                                uint8_t,
                                "air_quality_accuracy",
                                "Air Quality Accuracy (BSEC 0..3)",
                                application::dptids::Level,
                                false>,

            // ---- Derived room air values -----------------------------------
            endpoint::semantics::TemperatureState<SensorBoardPort::RoomDewPoint,
                                                  "room_dew_point",
                                                  "Room Dew Point",
                                                  false>,
            endpoint::StatePort<SensorBoardPort::RoomAbsoluteHumidity,
                                float,
                                "room_absolute_humidity",
                                "Room Absolute Humidity",
                                application::dptids::AbsoluteHumidity,
                                false>,

            // ---- FB FTS + slab moisture -------------------------------------
            endpoint::semantics::TemperatureState<SensorBoardPort::FloorTemperature,
                                                  "floor_temperature",
                                                  "Floor Temperature",
                                                  false>,
            endpoint::semantics::HumidityState<SensorBoardPort::FloorHumidity,
                                               "floor_humidity",
                                               "Floor Slab Relative Humidity",
                                               false>,
            endpoint::StatePort<SensorBoardPort::FloorAbsoluteHumidity,
                                float,
                                "floor_absolute_humidity",
                                "Floor Slab Absolute Humidity",
                                application::dptids::AbsoluteHumidity,
                                false>,
            endpoint::semantics::AlarmState<SensorBoardPort::FloorMoistureAlarm,
                                            "floor_moisture_alarm",
                                            "Floor Slab Moisture Alarm",
                                            false>,
            endpoint::StatePort<SensorBoardPort::FloorLimitActive,
                                bool,
                                "floor_limit_active",
                                "Max Floor Temperature Limit Active",
                                application::dptids::State,
                                false>,
            endpoint::StatePort<SensorBoardPort::FloorComfortActive,
                                bool,
                                "floor_comfort_active",
                                "Min Floor Temperature Active",
                                application::dptids::State,
                                false>,

            // ---- FB DPS: condensation protection ----------------------------
            endpoint::semantics::AlarmState<SensorBoardPort::DewPointAlarm,
                                            "dew_point_alarm",
                                            "Dew Point Alarm",
                                            false>,
            endpoint::StatePort<SensorBoardPort::DewPointMargin,
                                float,
                                "dew_point_margin",
                                "Dew Point Margin (surface - dew point)",
                                application::dptids::TemperatureDelta,
                                false>,
            endpoint::CommandPort<SensorBoardPort::DewPointStatusInput,
                                  bool,
                                  "dew_point_status_input",
                                  "Dew Point Alarm Input (external sensor)",
                                  application::dptids::Alarm,
                                  false>,

            // ---- System / neighbour inputs -----------------------------------
            endpoint::CommandPort<SensorBoardPort::OutsideTemperature,
                                  float,
                                  "outside_temperature",
                                  "Outside Temperature Input",
                                  application::dptids::Temperature,
                                  false>,
            endpoint::CommandPort<SensorBoardPort::OutsideHumidity,
                                  float,
                                  "outside_humidity",
                                  "Outside Relative Humidity Input",
                                  application::dptids::Humidity,
                                  false>,
            endpoint::CommandPort<SensorBoardPort::FlowTemperature,
                                  float,
                                  "flow_temperature",
                                  "Cooling Flow Temperature Input",
                                  application::dptids::Temperature,
                                  false>,
            endpoint::StatePort<SensorBoardPort::FreeCoolingAvailable,
                                bool,
                                "free_cooling_available",
                                "Free Cooling Available (outside air)",
                                application::dptids::State,
                                false>,
            endpoint::StatePort<SensorBoardPort::FreeDryingAvailable,
                                bool,
                                "free_drying_available",
                                "Free Drying Available (outside air)",
                                application::dptids::State,
                                false>,

            // ---- FB RTSM: mode and setpoints ---------------------------------
            endpoint::semantics::SwitchStateInOut<SensorBoardPort::ControllerOnOff,
                                                  "controller_on_off",
                                                  "Controller On/Off",
                                                  true>,
            // Split input/status pair: the requested mode and the mode the
            // controller actually resolved to can legitimately differ (window
            // open, presence, building protection), and a single in/out object
            // cannot express that without lying to whichever end reads it.
            endpoint::CommandPort<SensorBoardPort::HvacMode,
                                  application::Dpt20Mode,
                                  "hvac_mode",
                                  "HVAC Operating Mode Input",
                                  application::dptids::HvacModeComfort,
                                  true>,
            endpoint::StatePort<SensorBoardPort::HvacModeStatus,
                                application::Dpt20Mode,
                                "hvac_mode_status",
                                "HVAC Operating Mode Status",
                                application::dptids::HvacModeComfort,
                                true>,
            endpoint::CommandPort<SensorBoardPort::ContrMode,
                                  uint8_t,
                                  "contr_mode",
                                  "Controller Mode Input (0=Auto 1=Heat 3=Cool 6=Off)",
                                  application::dptids::HvacContrMode,
                                  true>,
            endpoint::StatePort<SensorBoardPort::ContrModeStatus,
                                uint8_t,
                                "contr_mode_status",
                                "Controller Mode Status",
                                application::dptids::HvacContrMode,
                                true>,
            // Drives a second RTC in the same room (e.g. radiator alongside the
            // underfloor circuit) so the two can never fight — KNX Vol 7/19/20
            // clause 6.3.4.2, Main/Secondary RTC.
            endpoint::StatePort<SensorBoardPort::ContrModeSecondary,
                                uint8_t,
                                "contr_mode_secondary",
                                "Controller Mode for Secondary Controller",
                                application::dptids::HvacContrMode,
                                false>,
            endpoint::StateInOutPort<SensorBoardPort::SetpointBase,
                                     float,
                                     "setpoint_base",
                                     "Base Setpoint (Comfort heating)",
                                     application::dptids::Temperature,
                                     true>,
            endpoint::CommandPort<SensorBoardPort::SetpointShift,
                                  float,
                                  "setpoint_shift",
                                  "Setpoint Shift Input",
                                  application::dptids::TemperatureDelta,
                                  false>,
            endpoint::StatePort<SensorBoardPort::SetpointShiftStatus,
                                float,
                                "setpoint_shift_status",
                                "Setpoint Shift Status",
                                application::dptids::TemperatureDelta,
                                false>,
            endpoint::semantics::TemperatureState<SensorBoardPort::SetpointStatus,
                                                  "setpoint_status",
                                                  "Active Setpoint Status",
                                                  false>,
            endpoint::semantics::TemperatureState<SensorBoardPort::SetpointHeatingStatus,
                                                  "setpoint_heating_status",
                                                  "Effective Heating Setpoint",
                                                  false>,
            endpoint::semantics::TemperatureState<SensorBoardPort::SetpointCoolingStatus,
                                                  "setpoint_cooling_status",
                                                  "Effective Cooling Setpoint",
                                                  false>,

            // ---- FB WOS / PRD / WCOS: room inputs ----------------------------
            endpoint::CommandPort<SensorBoardPort::WindowStatus,
                                  bool,
                                  "window_status",
                                  "Window Contact Input",
                                  application::dptids::WindowDoor,
                                  false>,
            endpoint::CommandPort<SensorBoardPort::PresenceStatus,
                                  bool,
                                  "presence_status",
                                  "Presence Input",
                                  application::dptids::Occupancy,
                                  false>,
            endpoint::CommandPort<SensorBoardPort::SwitchHeat,
                                  bool,
                                  "switch_heat",
                                  "Heating Enable Input",
                                  application::dptids::Enable,
                                  false>,
            endpoint::CommandPort<SensorBoardPort::SwitchCool,
                                  bool,
                                  "switch_cool",
                                  "Cooling Enable Input",
                                  application::dptids::Enable,
                                  false>,
            endpoint::CommandPort<SensorBoardPort::ChangeOverStatus,
                                  bool,
                                  "change_over_status",
                                  "Water Changeover Input (1=Heat)",
                                  application::dptids::HeatCool,
                                  false>,

            // ---- FB RTC: controller outputs ----------------------------------
            endpoint::semantics::PercentState<SensorBoardPort::HeatingControlValue,
                                              "heating_control_value",
                                              "Heating Control Value",
                                              false>,
            endpoint::semantics::PercentState<SensorBoardPort::CoolingControlValue,
                                              "cooling_control_value",
                                              "Cooling Control Value",
                                              false>,
            endpoint::semantics::SwitchState<SensorBoardPort::HeatingRequest,
                                             "heating_request",
                                             "Heating Demand",
                                             false>,
            endpoint::semantics::SwitchState<SensorBoardPort::CoolingRequest,
                                             "cooling_request",
                                             "Cooling Demand",
                                             false>,
            endpoint::StatePort<SensorBoardPort::HeatCoolModeStatus,
                                bool,
                                "heat_cool_mode_status",
                                "Heat/Cool Mode Status (1=Heat)",
                                application::dptids::HeatCool,
                                false>,
            endpoint::StatePort<SensorBoardPort::EnableHeatStatus,
                                bool,
                                "enable_heat_status",
                                "Heating Enabled Status",
                                application::dptids::Enable,
                                false>,
            endpoint::StatePort<SensorBoardPort::EnableCoolStatus,
                                bool,
                                "enable_cool_status",
                                "Cooling Enabled Status",
                                application::dptids::Enable,
                                false>,
            endpoint::StatePort<SensorBoardPort::ControllerStatus,
                                uint16_t,
                                "controller_status",
                                "Controller Status (DPT 22.101 StatusRHCC)",
                                application::dptids::StatusRHCC,
                                false>,

            // ---- Ventilation / air quality -------------------------------------
            endpoint::StateInOutPort<SensorBoardPort::Co2Setpoint,
                                     float,
                                     "co2_setpoint",
                                     "CO2 Setpoint",
                                     application::dptids::CO2,
                                     true>,
            endpoint::semantics::PercentState<SensorBoardPort::VentilationDemand,
                                              "ventilation_demand",
                                              "Ventilation Demand",
                                              false>,
            endpoint::StatePort<SensorBoardPort::VentilationStage,
                                uint8_t,
                                "ventilation_stage",
                                "Ventilation Stage (0=Off..4=Boost)",
                                application::dptids::Level,
                                false>,
            endpoint::StateInOutPort<SensorBoardPort::VentilationMode,
                                     uint8_t,
                                     "ventilation_mode",
                                     "Ventilation Mode (0=Auto 1=Manual 2=Off 3=Boost)",
                                     application::dptids::Level,
                                     true>,
            endpoint::semantics::SwitchState<SensorBoardPort::VentilationBoostRequest,
                                             "ventilation_boost_request",
                                             "Ventilation Boost Request",
                                             false>,
            endpoint::semantics::SwitchState<SensorBoardPort::DehumidifyRequest,
                                             "dehumidify_request",
                                             "Dehumidification Request",
                                             false>,
            endpoint::StatePort<SensorBoardPort::AirQualityStatus,
                                uint16_t,
                                "air_quality_status",
                                "Air Quality Status (bitset)",
                                application::dptids::Counter16,
                                false>,

            // ---- Device diagnostics ---------------------------------------------
            endpoint::semantics::AlarmState<SensorBoardPort::DeviceFault,
                                            "device_fault",
                                            "Device Fault",
                                            false>,
            // KNX FB sensor status octets (DPT 21.001 StatusGen), one per
            // physical sensor package rather than per measurand: a failure is a
            // property of the part, and this keeps the object count honest.
            endpoint::StatePort<SensorBoardPort::RoomSensorStatus,
                                uint8_t,
                                "room_sensor_status",
                                "Room Sensor Status (DPT 21.001)",
                                application::dptids::StatusGen,
                                false>,
            endpoint::StatePort<SensorBoardPort::FloorProbeStatus,
                                uint8_t,
                                "floor_probe_status",
                                "Floor Probe Status (DPT 21.001)",
                                application::dptids::StatusGen,
                                false>,
            endpoint::StatePort<SensorBoardPort::AirQualitySensorStatus,
                                uint8_t,
                                "air_quality_sensor_status",
                                "Air Quality Sensor Status (DPT 21.001)",
                                application::dptids::StatusGen,
                                false>>(
            ProductIdentity{
                .productKey = "sensor_board_tp1",
                .productDisplayName = "Room HVAC Sensor/Controller TP1",
                .manufacturerId = ManufacturerId(0x00FA),
                .medium = endpoint::Medium::TP1,
                .applicationNumber = 21,
                .applicationVersion = 4,
                .firmwareRevision = 1,
                .maxApduLength = 254,
                // Feeds both the device's PID_HARDWARE_TYPE / PID_VERSION /
                // PID_ORDER_INFO and the generated knxprod's hardware entry,
                // which ETS cross-checks before allowing a download.
                .hardwareSerialNumber = 1,
                .hardwareVersion = 1,
                .orderNumber = "SBTP1",
            },
            PersistencePolicy{
                .namespacePrefix = "sensorboard_tp1",
                .schemaVersion = 2,
                .persistKnxState = true,
            },
            // Must stay in step with kEnableKnxDataSecure in knx_service.cpp:
            // this is only what the ETS catalogue entry advertises, and a
            // device that claims Data Secure but cannot honour it fails
            // commissioning. No per-object requirement, so enabling Data
            // Secure stays the integrator's choice and the board still
            // commissions into plain, non-secure installations.
            SecurityPolicy{
                .dataSecureCapable = true,
                .groupObjectRequirement = SecurityRequirement::None,
                .individualAddressEntries = 8,
                // 0 → one group-key slot per group object.
                .groupKeyTableEntries = 0,
                .p2pKeyTableEntries = 1,
            }),
        makeParameterSchema(
            // ---- Device ------------------------------------------------------
            ParameterDescriptor<SensorBoardParameter::ParameterLayoutVersion, uint16_t>{
                .key = "parameter_layout_version",
                .displayName = "Parameter Layout Version (do not change)",
                .defaultValue = kParameterLayoutVersion,
                .group = kGroupDevice},

            // ---- Measurements and sending behaviour --------------------------
            // The KNX sensor FBs parameterise Heartbeat/MinRepTime per
            // Datapoint. Held device-wide here instead: sixty per-sensor timing
            // fields would bury the handful of settings an integrator actually
            // changes, and the per-measurand knob that does matter — the COV
            // delta, whose sensible value differs by orders of magnitude
            // between K, %RH and ppm — is kept individually.
            ParameterDescriptor<SensorBoardParameter::MeasurementHeartbeatSeconds, uint16_t>{
                .key = "measurement_heartbeat_seconds",
                .displayName = "Cyclic retransmission (heartbeat)",
                .defaultValue = kDefaultMeasurementHeartbeatSeconds,
                .minValue = 0,
                .maxValue = 65535,
                .unit = "s",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::MeasurementMinRepTimeSeconds, uint16_t>{
                .key = "measurement_min_rep_time_seconds",
                .displayName = "Minimum time between transmissions",
                .defaultValue = kDefaultMeasurementMinRepTimeSeconds,
                .minValue = 0,
                .maxValue = 3600,
                .unit = "s",
                .group = kGroupMeasurement},
            // Self-heating is the dominant error on a wall-mounted board that
            // also powers a TP1 transceiver, so an offset is not optional here.
            ParameterDescriptor<SensorBoardParameter::RoomTemperatureOffset, Dpt9Float>{
                .key = "room_temperature_offset",
                .displayName = "Room temperature correction",
                .defaultValue = Dpt9Float{kDefaultRoomTemperatureOffsetK},
                .minValue = -10,
                .maxValue = 10,
                .unit = "K",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::RoomTemperatureCov, Dpt9Float>{
                .key = "room_temperature_cov",
                .displayName = "Room temperature change to send",
                .defaultValue = Dpt9Float{kDefaultRoomTemperatureCovK},
                .minValue = 0,
                .maxValue = 10,
                .unit = "K",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::RoomHumidityOffset, Dpt9Float>{
                .key = "room_humidity_offset",
                .displayName = "Room humidity correction",
                .defaultValue = Dpt9Float{kDefaultRoomHumidityOffsetPct},
                .minValue = -20,
                .maxValue = 20,
                .unit = "%",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::RoomHumidityCov, Dpt9Float>{
                .key = "room_humidity_cov",
                .displayName = "Room humidity change to send",
                .defaultValue = Dpt9Float{kDefaultRoomHumidityCovPct},
                .minValue = 0,
                .maxValue = 50,
                .unit = "%",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::Co2Cov, uint16_t>{
                .key = "co2_cov",
                .displayName = "CO2 change to send",
                .defaultValue = kDefaultCo2CovPpm,
                .minValue = 0,
                .maxValue = 5000,
                .unit = "ppm",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::PressureCov, uint16_t>{
                .key = "pressure_cov",
                .displayName = "Air pressure change to send",
                .defaultValue = kDefaultPressureCovPa,
                .minValue = 0,
                .maxValue = 10000,
                .unit = "Pa",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::AirQualityCov, uint16_t>{
                .key = "air_quality_cov",
                .displayName = "Air quality index change to send",
                .defaultValue = kDefaultAirQualityCovIndex,
                .minValue = 0,
                .maxValue = 500,
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::FloorTemperatureOffset, Dpt9Float>{
                .key = "floor_temperature_offset",
                .displayName = "Floor temperature correction",
                .defaultValue = Dpt9Float{kDefaultFloorTemperatureOffsetK},
                .minValue = -10,
                .maxValue = 10,
                .unit = "K",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::FloorTemperatureCov, Dpt9Float>{
                .key = "floor_temperature_cov",
                .displayName = "Floor temperature change to send",
                .defaultValue = Dpt9Float{kDefaultFloorTemperatureCovK},
                .minValue = 0,
                .maxValue = 10,
                .unit = "K",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::FloorHumidityCov, Dpt9Float>{
                .key = "floor_humidity_cov",
                .displayName = "Floor humidity change to send",
                .defaultValue = Dpt9Float{kDefaultFloorHumidityCovPct},
                .minValue = 0,
                .maxValue = 50,
                .unit = "%",
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::DerivedValueCov, Dpt9Float>{
                .key = "derived_value_cov",
                .displayName = "Dew point / absolute humidity change to send",
                .defaultValue = Dpt9Float{kDefaultDerivedCovK},
                .minValue = 0,
                .maxValue = 10,
                .group = kGroupMeasurement},
            ParameterDescriptor<SensorBoardParameter::AltitudeM, uint16_t>{
                .key = "altitude_m",
                .displayName = "Installation altitude above sea level",
                .defaultValue = kDefaultAltitudeM,
                .minValue = 0,
                .maxValue = 4000,
                .unit = "m",
                .group = kGroupMeasurement},

            // ---- Room control: general ---------------------------------------
            ParameterDescriptor<SensorBoardParameter::ControllerDefaultEnable, uint8_t>{
                .key = "controller_default_enable",
                .displayName = "Controller state after download",
                .defaultValue = kDefaultControllerEnable,
                .options = parameterOptions(ParameterOption{0, "Off"}, ParameterOption{1, "On"}),
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::DefaultHvacOperatingMode, uint8_t>{
                .key = "default_hvac_operating_mode",
                .displayName = "HVAC operating mode after download",
                .defaultValue = kDefaultHvacOperatingMode,
                .options = parameterOptions(ParameterOption{0, "Auto"},
                                            ParameterOption{1, "Comfort"},
                                            ParameterOption{2, "Standby"},
                                            ParameterOption{3, "Economy"},
                                            ParameterOption{4, "Building protection"}),
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::DefaultControllerMode, uint8_t>{
                .key = "default_controller_mode",
                .displayName = "Controller mode after download",
                .defaultValue = kDefaultControllerMode,
                .options = parameterOptions(ParameterOption{0, "Auto"},
                                            ParameterOption{1, "Heat"},
                                            ParameterOption{2, "Cool"},
                                            ParameterOption{3, "Off"}),
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::HeatingEnabled, uint8_t>{
                .key = "heating_enabled",
                .displayName = "Heating sequence",
                .defaultValue = kDefaultHeatingEnabled,
                .options = parameterOptions(ParameterOption{0, "Not used"},
                                            ParameterOption{1, "Used"}),
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::CoolingEnabled, uint8_t>{
                .key = "cooling_enabled",
                .displayName = "Cooling sequence",
                .defaultValue = kDefaultCoolingEnabled,
                .options = parameterOptions(ParameterOption{0, "Not used"},
                                            ParameterOption{1, "Used"}),
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::HeatCoolChangeoverMode, uint8_t>{
                .key = "heat_cool_changeover_mode",
                .displayName = "Heat/cool changeover",
                .defaultValue = kDefaultHeatCoolChangeoverMode,
                .options = parameterOptions(ParameterOption{0, "Automatic (internal dead band)"},
                                            ParameterOption{1, "Follow changeover input"},
                                            ParameterOption{2, "Fixed: heating only"},
                                            ParameterOption{3, "Fixed: cooling only"}),
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::HeatCoolChangeoverPolarity, uint8_t>{
                .key = "heat_cool_changeover_polarity",
                .displayName = "Changeover input polarity",
                .defaultValue = kDefaultHeatCoolChangeoverPolarity,
                .options = parameterOptions(ParameterOption{0, "1 = heating (DPT 1.100)"},
                                            ParameterOption{1, "1 = cooling (inverted)"}),
                .group = kGroupControlGeneral,
                .visibleWhenParameterId = paramId(SensorBoardParameter::HeatCoolChangeoverMode),
                .visibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::MinimumHeatCoolChangeoverSeconds, uint16_t>{
                .key = "minimum_heat_cool_changeover_seconds",
                .displayName = "Minimum delay between heating and cooling",
                .defaultValue = kDefaultMinimumHeatCoolChangeoverSeconds,
                .minValue = 0,
                .maxValue = 65535,
                .unit = "s",
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::WindowOpenBehavior, uint8_t>{
                .key = "window_open_behavior",
                .displayName = "Behaviour with window open",
                .defaultValue = kDefaultWindowOpenBehavior,
                .options = parameterOptions(ParameterOption{0, "Ignore"},
                                            ParameterOption{1, "Building protection setpoint"},
                                            ParameterOption{2, "Switch outputs off"}),
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::PresenceBehavior, uint8_t>{
                .key = "presence_behavior",
                .displayName = "Mode when the room is unoccupied",
                .defaultValue = kDefaultPresenceBehavior,
                .options = parameterOptions(ParameterOption{0, "Standby"},
                                            ParameterOption{1, "Economy"}),
                .group = kGroupControlGeneral},
            ParameterDescriptor<SensorBoardParameter::SensorFaultBehavior, uint8_t>{
                .key = "sensor_fault_behavior",
                .displayName = "Behaviour on sensor failure",
                .defaultValue = kDefaultSensorFaultBehavior,
                .options = parameterOptions(ParameterOption{0, "Hold last outputs"},
                                            ParameterOption{1, "Switch outputs off"},
                                            ParameterOption{2, "Keep controlling on last reading"}),
                .group = kGroupControlGeneral},

            // ---- Setpoints ------------------------------------------------------
            // KNX RTSM ladder: Comfort is the absolute anchor, Standby/Economy
            // are shifts away from it, and only building protection is absolute
            // again. One setpoint write from an HMI or Home Assistant then moves
            // the whole ladder coherently.
            ParameterDescriptor<SensorBoardParameter::ComfortHeatingSetpoint, Dpt9Float>{
                .key = "comfort_heating_setpoint",
                .displayName = "Comfort heating setpoint",
                .defaultValue = Dpt9Float{kDefaultComfortHeatingSetpointC},
                .minValue = 5,
                .maxValue = 40,
                .unit = "°C",
                .group = kGroupSetpoints},
            ParameterDescriptor<SensorBoardParameter::StandbyHeatingReduction, Dpt9Float>{
                .key = "standby_heating_reduction",
                .displayName = "Standby: reduction below comfort",
                .defaultValue = Dpt9Float{kDefaultStandbyHeatingReductionK},
                .minValue = 0,
                .maxValue = 15,
                .unit = "K",
                .group = kGroupSetpoints},
            ParameterDescriptor<SensorBoardParameter::EconomyHeatingReduction, Dpt9Float>{
                .key = "economy_heating_reduction",
                .displayName = "Economy: reduction below comfort",
                .defaultValue = Dpt9Float{kDefaultEconomyHeatingReductionK},
                .minValue = 0,
                .maxValue = 20,
                .unit = "K",
                .group = kGroupSetpoints},
            ParameterDescriptor<SensorBoardParameter::ProtectionHeatingSetpoint, Dpt9Float>{
                .key = "protection_heating_setpoint",
                .displayName = "Building protection: frost setpoint",
                .defaultValue = Dpt9Float{kDefaultProtectionHeatingSetpointC},
                .minValue = 3,
                .maxValue = 15,
                .unit = "°C",
                .group = kGroupSetpoints},
            ParameterDescriptor<SensorBoardParameter::CoolingDeadband, Dpt9Float>{
                .key = "cooling_deadband",
                .displayName = "Dead band between heating and cooling",
                .defaultValue = Dpt9Float{kDefaultCoolingDeadbandK},
                .minValue = 0,
                .maxValue = 10,
                .unit = "K",
                .group = kGroupSetpoints},
            ParameterDescriptor<SensorBoardParameter::StandbyCoolingIncrease, Dpt9Float>{
                .key = "standby_cooling_increase",
                .displayName = "Standby: increase above comfort cooling",
                .defaultValue = Dpt9Float{kDefaultStandbyCoolingIncreaseK},
                .minValue = 0,
                .maxValue = 15,
                .unit = "K",
                .group = kGroupSetpoints,
                .visibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .visibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::EconomyCoolingIncrease, Dpt9Float>{
                .key = "economy_cooling_increase",
                .displayName = "Economy: increase above comfort cooling",
                .defaultValue = Dpt9Float{kDefaultEconomyCoolingIncreaseK},
                .minValue = 0,
                .maxValue = 20,
                .unit = "K",
                .group = kGroupSetpoints,
                .visibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .visibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::ProtectionCoolingSetpoint, Dpt9Float>{
                .key = "protection_cooling_setpoint",
                .displayName = "Building protection: heat setpoint",
                .defaultValue = Dpt9Float{kDefaultProtectionCoolingSetpointC},
                .minValue = 25,
                .maxValue = 45,
                .unit = "°C",
                .group = kGroupSetpoints,
                .visibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .visibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::MinSetpoint, Dpt9Float>{
                .key = "min_setpoint",
                .displayName = "Lowest allowed setpoint",
                .defaultValue = Dpt9Float{kDefaultMinSetpointC},
                .minValue = 3,
                .maxValue = 25,
                .unit = "°C",
                .group = kGroupSetpoints},
            ParameterDescriptor<SensorBoardParameter::MaxSetpoint, Dpt9Float>{
                .key = "max_setpoint",
                .displayName = "Highest allowed setpoint",
                .defaultValue = Dpt9Float{kDefaultMaxSetpointC},
                .minValue = 15,
                .maxValue = 45,
                .unit = "°C",
                .group = kGroupSetpoints},
            ParameterDescriptor<SensorBoardParameter::MaxSetpointShift, Dpt9Float>{
                .key = "max_setpoint_shift",
                .displayName = "Maximum manual setpoint shift",
                .defaultValue = Dpt9Float{kDefaultMaxSetpointShiftK},
                .minValue = 0,
                .maxValue = 10,
                .unit = "K",
                .group = kGroupSetpoints},

            // ---- Heating control -------------------------------------------------
            // The whole section is gated on the heating sequence being in use, so
            // an integrator who switched heating off does not get a section of
            // settings that the controller ignores. Cooling below is deliberately
            // identical in shape: the two sequences differ only in which enable
            // and which algorithm parameter they follow.
            ParameterDescriptor<SensorBoardParameter::HeatingControlAlgorithm, uint8_t>{
                .key = "heating_control_algorithm",
                .displayName = "Heating control algorithm",
                .defaultValue = kDefaultHeatingControlAlgorithm,
                .options = parameterOptions(ParameterOption{0, "Two-point (on/off)"},
                                            ParameterOption{1, "Continuous PI"}),
                .group = kGroupHeating,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::HeatingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::HeatingKp, Dpt9Float>{
                .key = "heating_pid_kp",
                .displayName = "Heating proportional gain",
                .defaultValue = Dpt9Float{kDefaultHeatingKp},
                .minValue = 1,
                .maxValue = 200,
                .unit = "%/K",
                .group = kGroupHeating,
                .visibleWhenParameterId = paramId(SensorBoardParameter::HeatingControlAlgorithm),
                .visibleWhenValue = 1,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::HeatingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::HeatingTiSeconds, uint16_t>{
                .key = "heating_pid_ti_seconds",
                .displayName = "Heating reset time (0 = P only)",
                .defaultValue = kDefaultHeatingTiSeconds,
                .minValue = 0,
                .maxValue = 65535,
                .unit = "s",
                .group = kGroupHeating,
                .visibleWhenParameterId = paramId(SensorBoardParameter::HeatingControlAlgorithm),
                .visibleWhenValue = 1,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::HeatingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::HeatingTdSeconds, uint16_t>{
                .key = "heating_pid_td_seconds",
                .displayName = "Heating derivative time (0 = off)",
                .defaultValue = kDefaultHeatingTdSeconds,
                .minValue = 0,
                .maxValue = 65535,
                .unit = "s",
                .group = kGroupHeating,
                .visibleWhenParameterId = paramId(SensorBoardParameter::HeatingControlAlgorithm),
                .visibleWhenValue = 1,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::HeatingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::HeatingMinimumOutputPercent, uint8_t>{
                .key = "heating_minimum_output_percent",
                .displayName = "Heating minimum control value",
                .defaultValue = kDefaultHeatingMinimumOutputPercent,
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupHeating,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::HeatingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::HeatingMaximumOutputPercent, uint8_t>{
                .key = "heating_maximum_output_percent",
                .displayName = "Heating maximum control value",
                .defaultValue = kDefaultHeatingMaximumOutputPercent,
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupHeating,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::HeatingEnabled),
                .groupVisibleWhenValue = 1},

            // ---- Cooling control -------------------------------------------------
            // Mirror image of the heating section above, gated on the cooling
            // sequence. The PI terms follow the cooling algorithm rather than the
            // cooling enable: gating them on the enable made them appear under a
            // two-point configuration, where they do nothing.
            ParameterDescriptor<SensorBoardParameter::CoolingControlAlgorithm, uint8_t>{
                .key = "cooling_control_algorithm",
                .displayName = "Cooling control algorithm",
                .defaultValue = kDefaultCoolingControlAlgorithm,
                .options = parameterOptions(ParameterOption{0, "Two-point (on/off)"},
                                            ParameterOption{1, "Continuous PI"}),
                .group = kGroupCooling,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::CoolingKp, Dpt9Float>{
                .key = "cooling_pid_kp",
                .displayName = "Cooling proportional gain",
                .defaultValue = Dpt9Float{kDefaultCoolingKp},
                .minValue = 1,
                .maxValue = 200,
                .unit = "%/K",
                .group = kGroupCooling,
                .visibleWhenParameterId = paramId(SensorBoardParameter::CoolingControlAlgorithm),
                .visibleWhenValue = 1,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::CoolingTiSeconds, uint16_t>{
                .key = "cooling_pid_ti_seconds",
                .displayName = "Cooling reset time (0 = P only)",
                .defaultValue = kDefaultCoolingTiSeconds,
                .minValue = 0,
                .maxValue = 65535,
                .unit = "s",
                .group = kGroupCooling,
                .visibleWhenParameterId = paramId(SensorBoardParameter::CoolingControlAlgorithm),
                .visibleWhenValue = 1,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::CoolingTdSeconds, uint16_t>{
                .key = "cooling_pid_td_seconds",
                .displayName = "Cooling derivative time (0 = off)",
                .defaultValue = kDefaultCoolingTdSeconds,
                .minValue = 0,
                .maxValue = 65535,
                .unit = "s",
                .group = kGroupCooling,
                .visibleWhenParameterId = paramId(SensorBoardParameter::CoolingControlAlgorithm),
                .visibleWhenValue = 1,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::CoolingMinimumOutputPercent, uint8_t>{
                .key = "cooling_minimum_output_percent",
                .displayName = "Cooling minimum control value",
                .defaultValue = kDefaultCoolingMinimumOutputPercent,
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupCooling,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .groupVisibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::CoolingMaximumOutputPercent, uint8_t>{
                .key = "cooling_maximum_output_percent",
                .displayName = "Cooling maximum control value",
                .defaultValue = kDefaultCoolingMaximumOutputPercent,
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupCooling,
                .groupVisibleWhenParameterId = paramId(SensorBoardParameter::CoolingEnabled),
                .groupVisibleWhenValue = 1},

            // ---- Control loop behaviour ------------------------------------------
            ParameterDescriptor<SensorBoardParameter::ThermostatHysteresis, Dpt9Float>{
                .key = "thermostat_hysteresis",
                .displayName = "Two-point switching hysteresis",
                .defaultValue = Dpt9Float{kDefaultThermostatHysteresisC},
                .minValue = 0,
                .maxValue = 5,
                .unit = "K",
                .group = kGroupLoop},
            ParameterDescriptor<SensorBoardParameter::BinaryDemandStrategy, uint8_t>{
                .key = "binary_demand_strategy",
                .displayName = "Binary demand derived from",
                .defaultValue = kDefaultBinaryDemandStrategy,
                .options = parameterOptions(ParameterOption{0, "Temperature hysteresis"},
                                            ParameterOption{1, "Control value threshold"}),
                .group = kGroupLoop},
            ParameterDescriptor<SensorBoardParameter::BinaryDemandThresholdPercent, uint8_t>{
                .key = "binary_demand_threshold_percent",
                .displayName = "Binary demand control-value threshold",
                .defaultValue = kDefaultBinaryDemandThresholdPercent,
                .minValue = 1,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupLoop,
                .visibleWhenParameterId = paramId(SensorBoardParameter::BinaryDemandStrategy),
                .visibleWhenValue = 1},
            ParameterDescriptor<SensorBoardParameter::FrostAlarmTemperature, Dpt9Float>{
                .key = "frost_alarm_temperature",
                .displayName = "Frost alarm below (0 = off)",
                .defaultValue = Dpt9Float{kDefaultFrostAlarmTemperatureC},
                .minValue = 0,
                .maxValue = 20,
                .unit = "°C",
                .group = kGroupLoop},
            ParameterDescriptor<SensorBoardParameter::OverheatAlarmTemperature, Dpt9Float>{
                .key = "overheat_alarm_temperature",
                .displayName = "Overheat alarm above (0 = off)",
                .defaultValue = Dpt9Float{kDefaultOverheatAlarmTemperatureC},
                .minValue = 0,
                .maxValue = 60,
                .unit = "°C",
                .group = kGroupLoop},

            // ---- Floor temperature -------------------------------------------------
            ParameterDescriptor<SensorBoardParameter::MaxFloorTemperature, Dpt9Float>{
                .key = "max_floor_temperature",
                .displayName = "Maximum floor temperature (0 = off)",
                .defaultValue = Dpt9Float{kDefaultMaxFloorTemperatureC},
                .minValue = 0,
                .maxValue = 45,
                .unit = "°C",
                .group = kGroupFloor},
            ParameterDescriptor<SensorBoardParameter::MinFloorTemperature, Dpt9Float>{
                .key = "min_floor_temperature",
                .displayName = "Minimum floor temperature (0 = off)",
                .defaultValue = Dpt9Float{kDefaultMinFloorTemperatureC},
                .minValue = 0,
                .maxValue = 35,
                .unit = "°C",
                .group = kGroupFloor},
            ParameterDescriptor<SensorBoardParameter::FloorHysteresis, Dpt9Float>{
                .key = "floor_hysteresis",
                .displayName = "Floor temperature hysteresis",
                .defaultValue = Dpt9Float{kDefaultFloorHysteresisK},
                .minValue = 0,
                .maxValue = 10,
                .unit = "K",
                .group = kGroupFloor},
            ParameterDescriptor<SensorBoardParameter::FloorComfortOutputPercent, uint8_t>{
                .key = "floor_comfort_output_percent",
                .displayName = "Control value while holding minimum floor temperature",
                .defaultValue = kDefaultFloorComfortOutputPercent,
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupFloor},

            // ---- Condensation protection -------------------------------------------
            ParameterDescriptor<SensorBoardParameter::DewPointSurfaceSource, uint8_t>{
                .key = "dew_point_surface_source",
                .displayName = "Surface temperature used for condensation check",
                .defaultValue = kDefaultDewPointSurfaceSource,
                .options = parameterOptions(ParameterOption{0, "Off (no local dew point alarm)"},
                                            ParameterOption{1, "Floor probe"},
                                            ParameterOption{2, "Flow temperature input"},
                                            ParameterOption{3, "Coldest available"}),
                .group = kGroupDewPoint},
            ParameterDescriptor<SensorBoardParameter::DewPointMargin, Dpt9Float>{
                .key = "dew_point_margin",
                .displayName = "Alarm when surface is within",
                .defaultValue = Dpt9Float{kDefaultDewPointMarginK},
                .minValue = 0,
                .maxValue = 10,
                .unit = "K of dew point",
                .group = kGroupDewPoint},
            ParameterDescriptor<SensorBoardParameter::DewPointHysteresis, Dpt9Float>{
                .key = "dew_point_hysteresis",
                .displayName = "Dew point alarm hysteresis",
                .defaultValue = Dpt9Float{kDefaultDewPointHysteresisK},
                .minValue = 0,
                .maxValue = 10,
                .unit = "K",
                .group = kGroupDewPoint},
            ParameterDescriptor<SensorBoardParameter::BlockCoolingOnDewPointAlarm, uint8_t>{
                .key = "block_cooling_on_dew_point_alarm",
                .displayName = "Cooling during a dew point alarm",
                .defaultValue = kDefaultBlockCoolingOnDewPointAlarm,
                .options = parameterOptions(ParameterOption{0, "Continue (report only)"},
                                            ParameterOption{1, "Block cooling"}),
                .group = kGroupDewPoint},

            // ---- Slab moisture detection ---------------------------------------------
            ParameterDescriptor<SensorBoardParameter::FloorMoistureThreshold, Dpt9Float>{
                .key = "floor_moisture_threshold",
                .displayName = "Slab humidity alarm above (0 = off)",
                .defaultValue = Dpt9Float{kDefaultFloorMoistureThresholdPct},
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupMoisture},
            ParameterDescriptor<SensorBoardParameter::FloorMoistureHysteresis, Dpt9Float>{
                .key = "floor_moisture_hysteresis",
                .displayName = "Slab humidity alarm hysteresis",
                .defaultValue = Dpt9Float{kDefaultFloorMoistureHysteresisPct},
                .minValue = 0,
                .maxValue = 25,
                .unit = "%",
                .group = kGroupMoisture},
            // Absolute humidity is a moisture *content*, so comparing the slab
            // against the room stays meaningful even though the slab is the
            // colder of the two — which is exactly where a relative-humidity
            // comparison misleads. Detects evaporating liquid water long before
            // the relative reading looks alarming.
            ParameterDescriptor<SensorBoardParameter::FloorMoistureExcess, Dpt9Float>{
                .key = "floor_moisture_excess",
                .displayName = "Alarm when slab is wetter than the room by (0 = off)",
                .defaultValue = Dpt9Float{kDefaultFloorMoistureExcessGm3},
                .minValue = 0,
                .maxValue = 20,
                .unit = "g/m³",
                .group = kGroupMoisture},

            // ---- Ventilation and air quality --------------------------------------------
            ParameterDescriptor<SensorBoardParameter::VentilationSetpoint, uint16_t>{
                .key = "ventilation_setpoint",
                .displayName = "CO2 setpoint (demand starts here)",
                .defaultValue = kDefaultVentilationSetpointPpm,
                .minValue = 400,
                .maxValue = 5000,
                .unit = "ppm",
                .group = kGroupVentilation},
            ParameterDescriptor<SensorBoardParameter::VentilationBand, uint16_t>{
                .key = "ventilation_band",
                .displayName = "CO2 band to full demand",
                .defaultValue = kDefaultVentilationBandPpm,
                .minValue = 0,
                .maxValue = 5000,
                .unit = "ppm",
                .group = kGroupVentilation},
            ParameterDescriptor<SensorBoardParameter::HumidityBoostThreshold, Dpt9Float>{
                .key = "humidity_boost_threshold",
                .displayName = "Humidity setpoint (demand starts here)",
                .defaultValue = Dpt9Float{kDefaultHumidityBoostPct},
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupVentilation},
            ParameterDescriptor<SensorBoardParameter::HumidityBoostBand, Dpt9Float>{
                .key = "humidity_boost_band",
                .displayName = "Humidity band to full demand",
                .defaultValue = Dpt9Float{kDefaultHumidityBandPct},
                .minValue = 0,
                .maxValue = 50,
                .unit = "%",
                .group = kGroupVentilation},
            ParameterDescriptor<SensorBoardParameter::VocBoostThreshold, uint16_t>{
                .key = "voc_boost_threshold",
                .displayName = "Air quality index setpoint (0 = off)",
                .defaultValue = kDefaultVocThresholdIndex,
                .minValue = 0,
                .maxValue = 500,
                .group = kGroupVentilation},
            ParameterDescriptor<SensorBoardParameter::VocBoostBand, uint16_t>{
                .key = "voc_boost_band",
                .displayName = "Air quality index band to full demand",
                .defaultValue = kDefaultVocBandIndex,
                .minValue = 0,
                .maxValue = 500,
                .group = kGroupVentilation},
            ParameterDescriptor<SensorBoardParameter::VentilationBaseDemandPercent, uint8_t>{
                .key = "ventilation_base_demand_percent",
                .displayName = "Base ventilation while the room is occupied",
                .defaultValue = kDefaultVentilationBaseDemandPercent,
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupVentilation},
            ParameterDescriptor<SensorBoardParameter::VentilationManualDemandPercent, uint8_t>{
                .key = "ventilation_manual_demand_percent",
                .displayName = "Demand in manual ventilation mode",
                .defaultValue = kDefaultVentilationManualDemandPercent,
                .minValue = 0,
                .maxValue = 100,
                .unit = "%",
                .group = kGroupVentilation}));

} // namespace sensor_board_knx
