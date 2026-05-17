#pragma once

#include "knx/product/commissioned_product.hpp"

#include <cstdint>

namespace sensor_board_knx {

using namespace knx;
using namespace knx::application;
using namespace knx::product;

inline constexpr float kDefaultThermostatSetpointC = 21.5f;
inline constexpr float kDefaultVentilationSetpointPpm = 900.0f;
inline constexpr float kDefaultThermostatHysteresisC = 0.5f;
inline constexpr float kDefaultVentilationHysteresisPpm = 75.0f;

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
    ThermostatRequest = 9,
    VentilationBoostRequest = 10,
};

enum class SensorBoardParameter : uint16_t {
    DefaultThermostatSetpoint = 0,
    DefaultVentilationSetpoint = 1,
    ThermostatHysteresis = 2,
    VentilationHysteresis = 3,
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
            parameter<SensorBoardParameter::DefaultThermostatSetpoint>("default_thermostat_setpoint", kDefaultThermostatSetpointC),
            parameter<SensorBoardParameter::DefaultVentilationSetpoint>("default_ventilation_setpoint", kDefaultVentilationSetpointPpm),
            parameter<SensorBoardParameter::ThermostatHysteresis>("thermostat_hysteresis", kDefaultThermostatHysteresisC),
            parameter<SensorBoardParameter::VentilationHysteresis>("ventilation_hysteresis", kDefaultVentilationHysteresisPpm)));

} // namespace sensor_board_knx