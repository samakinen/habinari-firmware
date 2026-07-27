#pragma once

#include "knx/product/commissioned_product.hpp"

#include <cstdint>

namespace sensor_board_knx {

using namespace knx;
using namespace knx::application;
using namespace knx::product;

// NOTE: defaults are integer-valued on purpose. ETS parameter defaults are
// exported as text and some knxprod tooling (Kaenx-Creator) parses them with
// the system locale — separator-free integers parse identically everywhere.
// Fractional values remain fully settable by the integrator in ETS (DPT9
// parameters render as decimal editors there).
inline constexpr float kDefaultVentilationSetpointPpm = 900.0f;
inline constexpr float kDefaultThermostatHysteresisC = 1.0f;
inline constexpr float kDefaultVentilationHysteresisPpm = 75.0f;
// Room-controller defaults (see main/include/hvac_control.hpp for semantics).
// PID gain in %/K = 100 / proportional band; 4 K band → 25. Ti = reset time.
inline constexpr float kDefaultHeatingKp = 25.0f;
inline constexpr float kDefaultHeatingTiSeconds = 9000.0f;
inline constexpr float kDefaultHeatingTdSeconds = 0.0f;
inline constexpr float kDefaultCoolingKp = 25.0f;
inline constexpr float kDefaultCoolingTiSeconds = 9000.0f;
inline constexpr float kDefaultCoolingTdSeconds = 0.0f;
inline constexpr float kDefaultCoolingDeadbandK = 2.0f;
inline constexpr float kDefaultMaxFloorTemperatureC = 28.0f;  // 0 disables
inline constexpr float kDefaultFloorHysteresisK = 1.0f;
inline constexpr float kDefaultHumidityBoostPct = 70.0f;
inline constexpr float kDefaultHumidityBoostHysteresisPct = 5.0f;

inline constexpr uint16_t kParameterLayoutVersion = 3;
inline constexpr uint16_t kDefaultControllerEnable = 1;
// DefaultHvacOperatingMode / DefaultControllerMode store the underlying byte
// of Dpt20Mode / ControllerMode (see enums in hvac_control.hpp).
inline constexpr uint16_t kDefaultHvacOperatingMode = 1;  // Dpt20Mode::Comfort
inline constexpr uint16_t kDefaultControllerMode = 0;     // ControllerMode::Auto
inline constexpr float kDefaultComfortHeatingSetpointC = 22.0f;
inline constexpr float kDefaultStandbyHeatingSetpointC = 20.0f;
inline constexpr float kDefaultEconomyHeatingSetpointC = 18.0f;
inline constexpr float kDefaultProtectionHeatingSetpointC = 7.0f;
inline constexpr float kDefaultComfortCoolingSetpointC = 24.0f;
inline constexpr float kDefaultStandbyCoolingSetpointC = 26.0f;
inline constexpr float kDefaultEconomyCoolingSetpointC = 28.0f;
inline constexpr float kDefaultProtectionCoolingSetpointC = 35.0f;
inline constexpr float kDefaultMinSetpointC = 7.0f;
inline constexpr float kDefaultMaxSetpointC = 35.0f;
inline constexpr float kDefaultMaxSetpointShiftK = 3.0f;
inline constexpr uint16_t kDefaultHeatingEnabled = 1;
inline constexpr uint16_t kDefaultCoolingEnabled = 0;
inline constexpr uint16_t kDefaultHeatCoolChangeoverMode = 0;      // Auto (no bus changeover)
inline constexpr uint16_t kDefaultHeatCoolChangeoverPolarity = 0;  // Normal
inline constexpr uint16_t kDefaultHeatingControlAlgorithm = 1;     // PI
inline constexpr uint16_t kDefaultCoolingControlAlgorithm = 1;     // PI
inline constexpr uint16_t kDefaultHeatingMinimumOutputPercent = 0;
inline constexpr uint16_t kDefaultCoolingMinimumOutputPercent = 0;
inline constexpr uint16_t kDefaultHeatingMaximumOutputPercent = 100;
inline constexpr uint16_t kDefaultCoolingMaximumOutputPercent = 100;
inline constexpr uint16_t kDefaultBinaryDemandStrategy = 0;  // Hysteresis
inline constexpr uint16_t kDefaultBinaryDemandThresholdPercent = 1;
inline constexpr uint16_t kDefaultMinimumHeatCoolChangeoverSeconds = 300;
inline constexpr uint16_t kDefaultWindowOpenBehavior = 2;      // BlockOutputs
inline constexpr uint16_t kDefaultPresenceBehavior = 0;        // Comfort/Standby
inline constexpr uint16_t kDefaultSensorFaultBehavior = 1;     // ForceOff
inline constexpr uint16_t kDefaultTemperatureChangeThresholdCenti = 20;  // 0.2 K
inline constexpr uint16_t kDefaultCyclicStatusIntervalSeconds = 300;

// Fractional ETS parameters use knx::product::Dpt9Float — the KNX-native
// 2-byte half-float parameter encoding (TypeFloat "DPT 9"), which ETS renders
// as a decimal editor and tooling handles most consistently. Inherently
// integral quantities (ppm, seconds) stay plain Unsigned16.

// KNX group-object (logical port) identity. Prototype/greenfield: renumber and
// regroup freely — the knxprod regenerates from this enum and is re-imported.
enum class SensorBoardPort : uint16_t {
    // --- Measurements ---
    AirTemperature = 0,
    AirHumidity = 1,
    AirPressure = 2,
    Co2 = 3,                        // SCD4x true CO2, ppm
    AirQualityIndex = 4,            // BME688/BSEC IAQ 0..500 (lower = cleaner)
    Co2Equivalent = 5,              // BME688/BSEC CO2-equivalent, ppm
    VocEquivalent = 6,              // BME688/BSEC breath-VOC-equivalent, ppm
    AirQualityAccuracy = 7,         // BSEC calibration status 0..3
    ProbeTemperature = 8,
    ProbeHumidity = 9,

    // --- Room controller: mode & setpoints ---
    ControllerEnable = 10,          // bool, in/out
    HvacOperatingMode = 11,         // Dpt20Mode, in/out (DPT 20.102)
    ControllerMode = 12,            // DPT 20.105 HVACContrMode, in/out
    HeatCoolChangeover = 13,        // bool DPT 1.100, bus input (1=Heat)
    ThermostatSetpoint = 14,        // room/comfort base setpoint, DPT 9.001, in/out
    SetpointShift = 15,             // float DPT 9.002, bus input
    ActiveSetpointFeedback = 16,    // effective setpoint after mode/shift/clamp, out
    SetpointShiftFeedback = 17,     // clamped shift actually applied, DPT 9.002, out
    HeatCoolState = 18,             // uint8_t: 0=Neutral,1=Heating,2=Cooling, out

    // --- Room controller: outputs ---
    ThermostatRequest = 19,         // heating demand (two-point)
    CoolingRequest = 20,            // cooling demand (two-point)
    HeatingControlValue = 21,       // PID heating output, DPT 5.001
    CoolingControlValue = 22,       // PID cooling output, DPT 5.001

    // --- Room inputs ---
    WindowOpen = 23,                // bool DPT 1.019, bus input
    Presence = 24,                  // bool DPT 1.018, bus input

    // --- Ventilation / IAQ control ---
    VentilationSetpoint = 25,       // CO2 setpoint, ppm, in/out
    VentilationDemandPercent = 26,  // uint8_t 0..100, out
    VentilationLevel = 27,          // uint8_t: 0=Off,1=Low,2=Medium,3=High,4=Boost, out
    VentilationMode = 28,           // uint8_t: 0=Auto,1=Manual,2=Off,3=Boost, in/out
    VentilationActive = 29,         // bool, out
    VentilationBoostRequest = 30,   // bool, out
    IaqStatus = 31,                 // uint16_t bitset (see IaqStatusBit), out

    // --- Diagnostics / status ---
    FloorLimitActive = 32,          // floor-temperature limit engaged, out
    ControllerFault = 33,           // bool, out
    HeatingBlocked = 34,            // bool, out
    CoolingBlocked = 35,            // bool, out
    SensorFaultStatus = 36,         // uint16_t bitset (see SensorFaultBit), out
    HvacControllerStatus = 37,      // uint16_t DPT 22.101 StatusRHCC, out
};

// ETS parameter identity. Declaration order in makeParameterSchema below defines
// the ProgramData / RS-0000 byte layout; prototype/greenfield: reorder/renumber
// freely and bump kParameterLayoutVersion — no old-layout migration is kept.
enum class SensorBoardParameter : uint16_t {
    DefaultVentilationSetpoint = 0,
    ThermostatHysteresis = 1,
    VentilationHysteresis = 2,
    HeatingKp = 3,
    HeatingTiSeconds = 4,
    HeatingTdSeconds = 5,
    CoolingKp = 6,
    CoolingTiSeconds = 7,
    CoolingTdSeconds = 8,
    CoolingDeadband = 9,
    MaxFloorTemperature = 10,
    FloorHysteresis = 11,
    HumidityBoostThreshold = 12,
    HumidityBoostHysteresis = 13,
    ParameterLayoutVersion = 14,
    ControllerDefaultEnable = 15,
    DefaultHvacOperatingMode = 16,
    DefaultControllerMode = 17,
    ComfortHeatingSetpoint = 18,
    StandbyHeatingSetpoint = 19,
    EconomyHeatingSetpoint = 20,
    ProtectionHeatingSetpoint = 21,
    ComfortCoolingSetpoint = 22,
    StandbyCoolingSetpoint = 23,
    EconomyCoolingSetpoint = 24,
    ProtectionCoolingSetpoint = 25,
    MinSetpoint = 26,
    MaxSetpoint = 27,
    MaxSetpointShift = 28,
    HeatingEnabled = 29,
    CoolingEnabled = 30,
    HeatCoolChangeoverMode = 31,
    HeatCoolChangeoverPolarity = 32,
    HeatingControlAlgorithm = 33,
    CoolingControlAlgorithm = 34,
    HeatingMinimumOutputPercent = 35,
    CoolingMinimumOutputPercent = 36,
    HeatingMaximumOutputPercent = 37,
    CoolingMaximumOutputPercent = 38,
    BinaryDemandStrategy = 39,
    BinaryDemandThresholdPercent = 40,
    MinimumHeatCoolChangeoverSeconds = 41,
    WindowOpenBehavior = 42,
    PresenceBehavior = 43,
    SensorFaultBehavior = 44,
    TemperatureChangeThresholdCenti = 45,
    CyclicStatusIntervalSeconds = 46,
};

inline constexpr auto kSensorBoardProduct =
    makeCommissionedProduct(
        makeEndpointDefinition<
            SensorBoardPort,
            endpoint::semantics::TemperatureState<SensorBoardPort::AirTemperature,
                                                  "air_temperature",
                                                  "Air Temperature",
                                                  false>,
            endpoint::semantics::HumidityState<SensorBoardPort::AirHumidity,
                                               "air_humidity",
                                               "Air Humidity",
                                               false>,
            endpoint::StatePort<SensorBoardPort::AirPressure,
                                float,
                                "air_pressure",
                                "Air Pressure",
                                application::dptids::Pressure,
                                false>,
            endpoint::semantics::Co2State<SensorBoardPort::Co2,
                                          "co2",
                                          "CO2",
                                          false>,
            // BME688 air quality via BSEC (see components/bsec2). Complementary
            // to the SCD4x true CO2 above; only published in BSEC builds.
            endpoint::StatePort<SensorBoardPort::AirQualityIndex,
                                uint16_t,
                                "air_quality_index",
                                "Air Quality Index (BSEC IAQ 0..500)",
                                application::dptids::Counter16,
                                false>,
            endpoint::StatePort<SensorBoardPort::Co2Equivalent,
                                float,
                                "co2_equivalent",
                                "CO2 Equivalent (BSEC, ppm)",
                                application::dptids::CO2,
                                false>,
            endpoint::StatePort<SensorBoardPort::VocEquivalent,
                                float,
                                "voc_equivalent",
                                "Breath-VOC Equivalent (BSEC, ppm)",
                                application::dptids::CO2,
                                false>,
            endpoint::StatePort<SensorBoardPort::AirQualityAccuracy,
                                uint8_t,
                                "air_quality_accuracy",
                                "Air Quality Accuracy (BSEC 0..3)",
                                application::dptids::Unsigned8,
                                false>,
            endpoint::semantics::TemperatureState<SensorBoardPort::ProbeTemperature,
                                                  "probe_temperature",
                                                  "Probe Temperature",
                                                  false>,
            endpoint::semantics::HumidityState<SensorBoardPort::ProbeHumidity,
                                               "probe_humidity",
                                               "Probe Humidity",
                                               false>,
            // Room/comfort base setpoint (DPT 9.001). Writable by HMIs and Home
            // Assistant's target_temperature: a write updates the live comfort
            // heating setpoint, clamped to [MinSetpoint, MaxSetpoint]. The
            // effective per-mode setpoint (base + shift) is reported separately
            // on ActiveSetpointFeedback.
            endpoint::StateInOutPort<SensorBoardPort::ThermostatSetpoint,
                                     float,
                                     "thermostat_setpoint",
                                     "Room Setpoint (Comfort, °C)",
                                     application::dptids::Temperature,
                                     false>,
            endpoint::StateInOutPort<SensorBoardPort::VentilationSetpoint,
                                     float,
                                     "ventilation_setpoint",
                                     "Ventilation CO2 Setpoint",
                                     application::dptids::CO2,
                                     false>,
            endpoint::semantics::SwitchState<SensorBoardPort::ThermostatRequest,
                                             "thermostat_request",
                                             "Thermostat Request",
                                             false>,
            endpoint::semantics::SwitchState<SensorBoardPort::VentilationBoostRequest,
                                             "ventilation_boost_request",
                                             "Ventilation Boost Request",
                                             false>,
            endpoint::semantics::SwitchState<SensorBoardPort::CoolingRequest,
                                             "cooling_request",
                                             "Cooling Request",
                                             false>,
            endpoint::semantics::PercentState<SensorBoardPort::HeatingControlValue,
                                              "heating_control_value",
                                              "Heating Control Value",
                                              false>,
            endpoint::semantics::PercentState<SensorBoardPort::CoolingControlValue,
                                              "cooling_control_value",
                                              "Cooling Control Value",
                                              false>,
            endpoint::semantics::SwitchState<SensorBoardPort::FloorLimitActive,
                                             "floor_limit_active",
                                             "Floor Temperature Limit Active",
                                             false>,
            // --- Append-only greenfield thermostat-model additions ---
            endpoint::semantics::SwitchStateInOut<SensorBoardPort::ControllerEnable,
                                                  "controller_enable",
                                                  "Controller Enable",
                                                  true>,
            endpoint::semantics::HvacModeStateInOut<SensorBoardPort::HvacOperatingMode,
                                                    "hvac_operating_mode",
                                                    "HVAC Operating Mode (Auto/Comfort/Standby/Economy/Protection)",
                                                    true>,
            // Heat/cool/auto/off controller mode as KNX DPT 20.105
            // (DPT_HVACContrMode): 0=Auto, 1=Heat, 3=Cool, 6=Off. The binding
            // layer maps these bus code points to/from the compact internal
            // ControllerMode enum (see hvac_control.hpp).
            endpoint::StateInOutPort<SensorBoardPort::ControllerMode,
                                     uint8_t,
                                     "controller_mode",
                                     "Controller Mode (DPT 20.105: 0=Auto,1=Heat,3=Cool,6=Off)",
                                     application::dptids::HvacContrMode,
                                     true>,
            endpoint::CommandPort<SensorBoardPort::HeatCoolChangeover,
                                  bool,
                                  "heat_cool_changeover",
                                  "Heat/Cool Changeover Input (1=Heat)",
                                  application::dptids::HeatCool,
                                  false>,
            endpoint::StatePort<SensorBoardPort::HeatCoolState,
                               uint8_t,
                               "heat_cool_state",
                               "Heat/Cool State (0=Neutral,1=Heating,2=Cooling)",
                               application::dptids::Unsigned8,
                               false>,
            endpoint::semantics::TemperatureState<SensorBoardPort::ActiveSetpointFeedback,
                                                  "active_setpoint_feedback",
                                                  "Active Setpoint Feedback",
                                                  false>,
            endpoint::CommandPort<SensorBoardPort::SetpointShift,
                                  float,
                                  "setpoint_shift",
                                  "Setpoint Shift Input (K)",
                                  application::dptids::TemperatureDelta,
                                  false>,
            endpoint::StatePort<SensorBoardPort::SetpointShiftFeedback,
                               float,
                               "setpoint_shift_feedback",
                               "Setpoint Shift Feedback (K)",
                               application::dptids::TemperatureDelta,
                               false>,
            endpoint::CommandPort<SensorBoardPort::WindowOpen,
                                  bool,
                                  "window_open",
                                  "Window Open Input",
                                  application::dptids::WindowDoor,
                                  false>,
            endpoint::CommandPort<SensorBoardPort::Presence,
                                  bool,
                                  "presence",
                                  "Presence Input",
                                  application::dptids::Occupancy,
                                  false>,
            endpoint::semantics::SwitchState<SensorBoardPort::ControllerFault,
                                             "controller_fault",
                                             "Controller Fault",
                                             false>,
            endpoint::StatePort<SensorBoardPort::SensorFaultStatus,
                               uint16_t,
                               "sensor_fault_status",
                               "Sensor Fault Status (bitset)",
                               application::dptids::Unsigned16,
                               false>,
            endpoint::semantics::SwitchState<SensorBoardPort::HeatingBlocked,
                                             "heating_blocked",
                                             "Heating Blocked",
                                             false>,
            endpoint::semantics::SwitchState<SensorBoardPort::CoolingBlocked,
                                             "cooling_blocked",
                                             "Cooling Blocked",
                                             false>,
            endpoint::semantics::PercentState<SensorBoardPort::VentilationDemandPercent,
                                              "ventilation_demand_percent",
                                              "Ventilation Demand",
                                              false>,
            endpoint::StatePort<SensorBoardPort::VentilationLevel,
                               uint8_t,
                               "ventilation_level",
                               "Ventilation Level (0=Off,1=Low,2=Medium,3=High,4=Boost)",
                               application::dptids::Unsigned8,
                               false>,
            endpoint::StateInOutPort<SensorBoardPort::VentilationMode,
                                     uint8_t,
                                     "ventilation_mode",
                                     "Ventilation Mode (0=Auto,1=Manual,2=Off,3=Boost)",
                                     application::dptids::Unsigned8,
                                     true>,
            endpoint::semantics::SwitchState<SensorBoardPort::VentilationActive,
                                             "ventilation_active",
                                             "Ventilation Active",
                                             false>,
            endpoint::StatePort<SensorBoardPort::IaqStatus,
                               uint16_t,
                               "iaq_status",
                               "IAQ Status (bitset)",
                               application::dptids::Unsigned16,
                               false>,
            // Standard KNX room heating/cooling controller status word for HMIs
            // (packs fault/heat-cool mode/heating+cooling disabled/flow limit —
            // see packStatusRHCC in hvac_control.hpp).
            endpoint::StatePort<SensorBoardPort::HvacControllerStatus,
                               uint16_t,
                               "hvac_controller_status",
                               "HVAC Controller Status (DPT 22.101 StatusRHCC)",
                               application::dptids::StatusRHCC,
                               false>>(
            ProductIdentity{
                .productKey = "sensor_board_tp1",
                .productDisplayName = "Sensor Board TP1",
                .manufacturerId = ManufacturerId(0x00FA),
                .medium = endpoint::Medium::TP1,
                .applicationNumber = 21,
                .applicationVersion = 3,
                .firmwareRevision = 1,
                .maxApduLength = 254,
            },
            PersistencePolicy{
                .namespacePrefix = "sensorboard_tp1",
                .schemaVersion = 1,
                .persistKnxState = true,
            }),
        makeParameterSchema(
            parameter<SensorBoardParameter::DefaultVentilationSetpoint>(
                "default_ventilation_setpoint", "Default Ventilation CO2 Setpoint (ppm)",
                static_cast<uint16_t>(kDefaultVentilationSetpointPpm)),
            parameter<SensorBoardParameter::ThermostatHysteresis>(
                "thermostat_hysteresis", "Thermostat Hysteresis (K)",
                Dpt9Float{kDefaultThermostatHysteresisC}),
            parameter<SensorBoardParameter::VentilationHysteresis>(
                "ventilation_hysteresis", "Ventilation CO2 Hysteresis (ppm)",
                static_cast<uint16_t>(kDefaultVentilationHysteresisPpm)),
            parameter<SensorBoardParameter::HeatingKp>(
                "heating_pid_kp", "Heating PID Gain (%/K)",
                Dpt9Float{kDefaultHeatingKp}),
            parameter<SensorBoardParameter::HeatingTiSeconds>(
                "heating_pid_ti_seconds", "Heating PID Integral Time (s, 0 = off)",
                static_cast<uint16_t>(kDefaultHeatingTiSeconds)),
            parameter<SensorBoardParameter::HeatingTdSeconds>(
                "heating_pid_td_seconds", "Heating PID Derivative Time (s, 0 = off)",
                static_cast<uint16_t>(kDefaultHeatingTdSeconds)),
            parameter<SensorBoardParameter::CoolingKp>(
                "cooling_pid_kp", "Cooling PID Gain (%/K)",
                Dpt9Float{kDefaultCoolingKp}),
            parameter<SensorBoardParameter::CoolingTiSeconds>(
                "cooling_pid_ti_seconds", "Cooling PID Integral Time (s, 0 = off)",
                static_cast<uint16_t>(kDefaultCoolingTiSeconds)),
            parameter<SensorBoardParameter::CoolingTdSeconds>(
                "cooling_pid_td_seconds", "Cooling PID Derivative Time (s, 0 = off)",
                static_cast<uint16_t>(kDefaultCoolingTdSeconds)),
            parameter<SensorBoardParameter::CoolingDeadband>(
                "cooling_deadband", "Cooling Deadband (K)",
                Dpt9Float{kDefaultCoolingDeadbandK}),
            parameter<SensorBoardParameter::MaxFloorTemperature>(
                "max_floor_temperature", "Max Floor Temperature (°C, 0 = off)",
                Dpt9Float{kDefaultMaxFloorTemperatureC}),
            parameter<SensorBoardParameter::FloorHysteresis>(
                "floor_hysteresis", "Floor Limit Hysteresis (K)",
                Dpt9Float{kDefaultFloorHysteresisK}),
            parameter<SensorBoardParameter::HumidityBoostThreshold>(
                "humidity_boost_threshold", "Humidity Boost Threshold (%RH)",
                Dpt9Float{kDefaultHumidityBoostPct}),
            parameter<SensorBoardParameter::HumidityBoostHysteresis>(
                "humidity_boost_hysteresis", "Humidity Boost Hysteresis (%RH)",
                Dpt9Float{kDefaultHumidityBoostHysteresisPct}),
            // --- Append-only greenfield thermostat-model additions ---
            parameter<SensorBoardParameter::ParameterLayoutVersion>(
                "parameter_layout_version", "Parameter Layout Version (do not change)",
                kParameterLayoutVersion),
            parameter<SensorBoardParameter::ControllerDefaultEnable>(
                "controller_default_enable", "Controller Enabled by Default (0/1)",
                kDefaultControllerEnable),
            parameter<SensorBoardParameter::DefaultHvacOperatingMode>(
                "default_hvac_operating_mode",
                "Default HVAC Operating Mode (0=Auto,1=Comfort,2=Standby,3=Economy,4=Protection)",
                kDefaultHvacOperatingMode),
            parameter<SensorBoardParameter::DefaultControllerMode>(
                "default_controller_mode", "Default Controller Mode (0=Auto,1=Heat,2=Cool,3=Off)",
                kDefaultControllerMode),
            parameter<SensorBoardParameter::ComfortHeatingSetpoint>(
                "comfort_heating_setpoint", "Comfort Heating Setpoint (°C)",
                Dpt9Float{kDefaultComfortHeatingSetpointC}),
            parameter<SensorBoardParameter::StandbyHeatingSetpoint>(
                "standby_heating_setpoint", "Standby Heating Setpoint (°C)",
                Dpt9Float{kDefaultStandbyHeatingSetpointC}),
            parameter<SensorBoardParameter::EconomyHeatingSetpoint>(
                "economy_heating_setpoint", "Economy Heating Setpoint (°C)",
                Dpt9Float{kDefaultEconomyHeatingSetpointC}),
            parameter<SensorBoardParameter::ProtectionHeatingSetpoint>(
                "protection_heating_setpoint", "Building Protection Heating Setpoint (°C)",
                Dpt9Float{kDefaultProtectionHeatingSetpointC}),
            parameter<SensorBoardParameter::ComfortCoolingSetpoint>(
                "comfort_cooling_setpoint", "Comfort Cooling Setpoint (°C)",
                Dpt9Float{kDefaultComfortCoolingSetpointC}),
            parameter<SensorBoardParameter::StandbyCoolingSetpoint>(
                "standby_cooling_setpoint", "Standby Cooling Setpoint (°C)",
                Dpt9Float{kDefaultStandbyCoolingSetpointC}),
            parameter<SensorBoardParameter::EconomyCoolingSetpoint>(
                "economy_cooling_setpoint", "Economy Cooling Setpoint (°C)",
                Dpt9Float{kDefaultEconomyCoolingSetpointC}),
            parameter<SensorBoardParameter::ProtectionCoolingSetpoint>(
                "protection_cooling_setpoint", "Building Protection Cooling Setpoint (°C)",
                Dpt9Float{kDefaultProtectionCoolingSetpointC}),
            parameter<SensorBoardParameter::MinSetpoint>(
                "min_setpoint", "Minimum Allowed Setpoint (°C)",
                Dpt9Float{kDefaultMinSetpointC}),
            parameter<SensorBoardParameter::MaxSetpoint>(
                "max_setpoint", "Maximum Allowed Setpoint (°C)",
                Dpt9Float{kDefaultMaxSetpointC}),
            parameter<SensorBoardParameter::MaxSetpointShift>(
                "max_setpoint_shift", "Maximum Setpoint Shift (K)",
                Dpt9Float{kDefaultMaxSetpointShiftK}),
            parameter<SensorBoardParameter::HeatingEnabled>(
                "heating_enabled", "Heating Enabled (0/1)",
                kDefaultHeatingEnabled),
            parameter<SensorBoardParameter::CoolingEnabled>(
                "cooling_enabled", "Cooling Enabled (0/1)",
                kDefaultCoolingEnabled),
            parameter<SensorBoardParameter::HeatCoolChangeoverMode>(
                "heat_cool_changeover_mode",
                "Heat/Cool Changeover Mode (0=Auto,1=BusInput,2=FixedHeat,3=FixedCool)",
                kDefaultHeatCoolChangeoverMode),
            parameter<SensorBoardParameter::HeatCoolChangeoverPolarity>(
                "heat_cool_changeover_polarity",
                "Heat/Cool Changeover Polarity (0=Normal(1=Heat),1=Inverted(1=Cool))",
                kDefaultHeatCoolChangeoverPolarity),
            parameter<SensorBoardParameter::HeatingControlAlgorithm>(
                "heating_control_algorithm", "Heating Control Algorithm (0=TwoPoint,1=PI)",
                kDefaultHeatingControlAlgorithm),
            parameter<SensorBoardParameter::CoolingControlAlgorithm>(
                "cooling_control_algorithm", "Cooling Control Algorithm (0=TwoPoint,1=PI)",
                kDefaultCoolingControlAlgorithm),
            parameter<SensorBoardParameter::HeatingMinimumOutputPercent>(
                "heating_minimum_output_percent", "Heating Minimum Output (%)",
                kDefaultHeatingMinimumOutputPercent),
            parameter<SensorBoardParameter::CoolingMinimumOutputPercent>(
                "cooling_minimum_output_percent", "Cooling Minimum Output (%)",
                kDefaultCoolingMinimumOutputPercent),
            parameter<SensorBoardParameter::HeatingMaximumOutputPercent>(
                "heating_maximum_output_percent", "Heating Maximum Output (%)",
                kDefaultHeatingMaximumOutputPercent),
            parameter<SensorBoardParameter::CoolingMaximumOutputPercent>(
                "cooling_maximum_output_percent", "Cooling Maximum Output (%)",
                kDefaultCoolingMaximumOutputPercent),
            parameter<SensorBoardParameter::BinaryDemandStrategy>(
                "binary_demand_strategy", "Binary Demand Strategy (0=Hysteresis,1=ControlValueThreshold)",
                kDefaultBinaryDemandStrategy),
            parameter<SensorBoardParameter::BinaryDemandThresholdPercent>(
                "binary_demand_threshold_percent", "Binary Demand Control-Value Threshold (%)",
                kDefaultBinaryDemandThresholdPercent),
            parameter<SensorBoardParameter::MinimumHeatCoolChangeoverSeconds>(
                "minimum_heat_cool_changeover_seconds", "Minimum Heat/Cool Changeover Delay (s)",
                kDefaultMinimumHeatCoolChangeoverSeconds),
            parameter<SensorBoardParameter::WindowOpenBehavior>(
                "window_open_behavior", "Window Open Behavior (0=Ignore,1=ProtectionSetpoint,2=BlockOutputs)",
                kDefaultWindowOpenBehavior),
            parameter<SensorBoardParameter::PresenceBehavior>(
                "presence_behavior",
                "Presence Behavior (0=Comfort/Standby,1=Comfort/Economy)",
                kDefaultPresenceBehavior),
            parameter<SensorBoardParameter::SensorFaultBehavior>(
                "sensor_fault_behavior", "Sensor Fault Behavior (0=HoldLast,1=ForceOff,2=Passthrough)",
                kDefaultSensorFaultBehavior),
            parameter<SensorBoardParameter::TemperatureChangeThresholdCenti>(
                "temperature_change_threshold_centi", "Temperature Publish Threshold (0.01 K)",
                kDefaultTemperatureChangeThresholdCenti),
            parameter<SensorBoardParameter::CyclicStatusIntervalSeconds>(
                "cyclic_status_interval_seconds", "Cyclic Status Publish Interval (s, 0 = off)",
                kDefaultCyclicStatusIntervalSeconds)));

} // namespace sensor_board_knx