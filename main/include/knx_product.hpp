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
inline constexpr float kDefaultThermostatSetpointC = 22.0f;
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

// Fractional ETS parameters use knx::product::Dpt9Float — the KNX-native
// 2-byte half-float parameter encoding (TypeFloat "DPT 9"), which ETS renders
// as a decimal editor and tooling handles most consistently. Inherently
// integral quantities (ppm, seconds) stay plain Unsigned16.

enum class SensorBoardPort : uint16_t {
    AirTemperature = 0,
    AirHumidity = 1,
    AirPressure = 2,
    GasResistance = 3,
    Co2 = 4,
    ProbeTemperature = 5,
    ProbeHumidity = 6,
    ThermostatSetpoint = 7,
    VentilationSetpoint = 8,
    ThermostatRequest = 9,          // heating demand (two-point)
    VentilationBoostRequest = 10,
    CoolingRequest = 11,            // cooling demand (two-point)
    HeatingControlValue = 12,       // PID heating output, DPT 5.001
    CoolingControlValue = 13,       // PID cooling output, DPT 5.001
    FloorLimitActive = 14,          // floor-temperature limit engaged
};

// NOTE: parameter declaration order defines the ETS ProgramData / RS-0000 byte
// layout — only append, never reorder, or previously-commissioned devices
// decode stale offsets.
enum class SensorBoardParameter : uint16_t {
    DefaultThermostatSetpoint = 0,
    DefaultVentilationSetpoint = 1,
    ThermostatHysteresis = 2,
    VentilationHysteresis = 3,
    HeatingKp = 4,
    HeatingTiSeconds = 5,
    HeatingTdSeconds = 6,
    CoolingKp = 7,
    CoolingTiSeconds = 8,
    CoolingTdSeconds = 9,
    CoolingDeadband = 10,
    MaxFloorTemperature = 11,
    FloorHysteresis = 12,
    HumidityBoostThreshold = 13,
    HumidityBoostHysteresis = 14,
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
            endpoint::StatePort<SensorBoardPort::GasResistance,
                                float,
                                "gas_resistance",
                                "Gas Resistance",
                                application::dptids::Resistance,
                                false>,
            endpoint::semantics::Co2State<SensorBoardPort::Co2,
                                          "co2",
                                          "CO2",
                                          false>,
            endpoint::semantics::TemperatureState<SensorBoardPort::ProbeTemperature,
                                                  "probe_temperature",
                                                  "Probe Temperature",
                                                  false>,
            endpoint::semantics::HumidityState<SensorBoardPort::ProbeHumidity,
                                               "probe_humidity",
                                               "Probe Humidity",
                                               false>,
            endpoint::StateInOutPort<SensorBoardPort::ThermostatSetpoint,
                                     float,
                                     "thermostat_setpoint",
                                     "Thermostat Setpoint",
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
                                             false>>(
            ProductIdentity{
                .productKey = "sensor_board_tp1",
                .productDisplayName = "Sensor Board TP1",
                .manufacturerId = ManufacturerId(0x00FA),
                .medium = endpoint::Medium::TP1,
                .applicationNumber = 21,
                .applicationVersion = 1,
                .firmwareRevision = 1,
                .maxApduLength = 254,
            },
            PersistencePolicy{
                .namespacePrefix = "sensorboard_tp1",
                .schemaVersion = 1,
                .persistKnxState = true,
            }),
        makeParameterSchema(
            parameter<SensorBoardParameter::DefaultThermostatSetpoint>(
                "default_thermostat_setpoint", "Default Thermostat Setpoint (°C)",
                Dpt9Float{kDefaultThermostatSetpointC}),
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
                Dpt9Float{kDefaultHumidityBoostHysteresisPct})));

} // namespace sensor_board_knx