// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "control_core.hpp"
#include "knx_dpt9.hpp"
#include "knx_product.hpp"
#include "sensor_data.h"

#include "knx/product/commissioned_product.hpp"

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

/**
 * @file knx_port_values.hpp
 * @brief The value every transmitting KNX group object carries — one
 * definition, for both the spontaneous send and the read response.
 *
 * These used to be two hand-kept lists: the publish path went through the
 * corrected readings and skipped a measurement with no valid reading, while the
 * read responses recomputed each value from the raw state — without the
 * validity check, and in places without the same correction. A bus read could
 * then answer with a number the device would never have sent.
 *
 * Now PortValues::get<Port>() is the only place a port's value is derived:
 *
 *   - Publishing offers get<Port>() when it has a value, and nothing when it
 *     does not (no valid reading): sending nothing is how the device keeps a
 *     dead sensor's last value off the bus.
 *   - A read response must answer something, so readResponse<Port>() answers
 *     get<Port>() — or, with no valid reading, the "no reading" form of the
 *     port's type: DPT 9 7FFFh "invalid data" (03_07_02 §3.10) for every float
 *     port, all of which are DPT 9.xxx, and 0 for the two integer ones
 *     (air quality index and its BSEC accuracy, where 0 is "unreliable").
 *
 * Platform-free, so the host tests check that the two agree for every port.
 */

namespace habinari_knx {

/// Everything a transmitting port's value is derived from: one coherent copy,
/// taken under the control lock.
struct PortState {
    sensor_data_t sensorData{};
    habinari::control::Settings settings{};
    habinari::control::Outputs out{};
    habinari::hvac::VentilationMode ventilationMode{habinari::hvac::VentilationMode::Auto};
    bool controllerOnOff{true};
};

using HabinariEndpointDefinition =
    typename std::remove_cvref_t<decltype(kHabinariProduct)>::endpoint_definition_type;

/// The value type knx_product.hpp declares for @p Port.
template <HabinariPort Port>
using PortValue = knx::product::port_value_t<HabinariEndpointDefinition, Port>;

/// A port's value, or nothing when there is no valid reading behind it.
template <HabinariPort Port>
using OptionalPortValue = std::optional<PortValue<Port>>;

/// The transmitting ports, in publication order, with the label a failed send
/// is logged under. Exactly the product's readable ports (checked below).
struct TransmitPort {
    HabinariPort port;
    const char *label;
};

inline constexpr TransmitPort kTransmitPorts[] = {
    // --- Room air measurements (FB RTS / RRHS / RAQS) ---
    {HabinariPort::RoomTemperature, "room temperature"},
    {HabinariPort::RoomHumidity, "room humidity"},
    {HabinariPort::RoomCo2, "room co2"},
    {HabinariPort::RoomAirPressure, "air pressure"},
    {HabinariPort::RoomAirPressureSeaLevel, "sea-level pressure"},
    {HabinariPort::RoomAirQualityIndex, "air quality index"},
    {HabinariPort::AirQualityAccuracy, "air quality accuracy"},
    {HabinariPort::RoomCo2Equivalent, "co2 equivalent"},
    {HabinariPort::RoomVocEquivalent, "voc equivalent"},
    // --- Derived room air values ---
    {HabinariPort::RoomDewPoint, "room dew point"},
    {HabinariPort::RoomAbsoluteHumidity, "room absolute humidity"},
    // --- Floor probe (FB FTS + slab moisture) ---
    {HabinariPort::FloorTemperature, "floor temperature"},
    {HabinariPort::FloorHumidity, "floor humidity"},
    {HabinariPort::FloorAbsoluteHumidity, "floor absolute humidity"},
    {HabinariPort::FloorMoistureAlarm, "floor moisture alarm"},
    {HabinariPort::FloorLimitActive, "floor limit active"},
    {HabinariPort::FloorComfortActive, "floor comfort active"},
    // --- Condensation protection (FB DPS) ---
    {HabinariPort::DewPointAlarm, "dew point alarm"},
    {HabinariPort::DewPointMargin, "dew point margin"},
    {HabinariPort::FreeCoolingAvailable, "free cooling available"},
    {HabinariPort::FreeDryingAvailable, "free drying available"},
    // --- Mode and setpoints (FB RTSM) ---
    {HabinariPort::ControllerOnOff, "controller on/off"},
    {HabinariPort::HvacModeStatus, "hvac mode status"},
    {HabinariPort::ContrModeStatus, "contr mode status"},
    {HabinariPort::ContrModeSecondary, "contr mode secondary"},
    {HabinariPort::SetpointBase, "base setpoint"},
    {HabinariPort::SetpointShiftStatus, "setpoint shift status"},
    {HabinariPort::SetpointStatus, "active setpoint"},
    {HabinariPort::SetpointHeatingStatus, "heating setpoint"},
    {HabinariPort::SetpointCoolingStatus, "cooling setpoint"},
    // --- Controller outputs (FB RTC) ---
    {HabinariPort::HeatingControlValue, "heating control value"},
    {HabinariPort::CoolingControlValue, "cooling control value"},
    {HabinariPort::HeatingRequest, "heating request"},
    {HabinariPort::CoolingRequest, "cooling request"},
    {HabinariPort::HeatCoolModeStatus, "heat/cool mode status"},
    {HabinariPort::EnableHeatStatus, "enable heat status"},
    {HabinariPort::EnableCoolStatus, "enable cool status"},
    {HabinariPort::ControllerStatus, "controller status"},
    // --- Ventilation / air quality ---
    {HabinariPort::Co2Setpoint, "co2 setpoint"},
    {HabinariPort::VentilationDemand, "ventilation demand"},
    {HabinariPort::VentilationStage, "ventilation stage"},
    {HabinariPort::VentilationMode, "ventilation mode"},
    {HabinariPort::VentilationBoostRequest, "ventilation boost request"},
    {HabinariPort::DehumidifyRequest, "dehumidify request"},
    {HabinariPort::AirQualityStatus, "air quality status"},
    // --- Device diagnostics ---
    {HabinariPort::DeviceFault, "device fault"},
    {HabinariPort::RoomSensorStatus, "room sensor status"},
    {HabinariPort::FloorProbeStatus, "floor probe status"},
    {HabinariPort::AirQualitySensorStatus, "air quality sensor status"},
    {HabinariPort::SensorHealthMask, "sensor health mask"},
    {HabinariPort::SensorDisagreementAlarm, "sensor disagreement"},
    // --- Derived events (see sensor_fusion.hpp) ---
    {HabinariPort::FireAlarm, "fire alarm"},
    {HabinariPort::FirePreAlarm, "fire pre-alarm"},
    {HabinariPort::TemperatureTrend, "temperature trend"},
    {HabinariPort::OccupancyDetected, "occupancy detected"},
    {HabinariPort::EstimatedOccupants, "estimated occupants"},
    {HabinariPort::WindowOpenDetected, "window open detected"},
};

/// Call `f.template operator()<Port>(label)` for every transmitting port, in
/// publication order.
template <typename F>
constexpr void forEachTransmitPort(F &&f)
{
    [&]<std::size_t... I>(std::index_sequence<I...>) {
        (f.template operator()<kTransmitPorts[I].port>(kTransmitPorts[I].label), ...);
    }(std::make_index_sequence<std::size(kTransmitPorts)>{});
}

class PortValues {
public:
    explicit PortValues(const PortState &state)
        : state_(state)
        , readings_(habinari::control::correctReadings(state.sensorData, state.settings))
    {
    }

    /// The value @p Port carries for this state, or nothing when there is no
    /// valid reading to send.
    template <HabinariPort Port>
    OptionalPortValue<Port> get() const;

private:
    // Each port's value, exactly in its declared type: a conversion has to be
    // written out here rather than happen implicitly on the way to the stack.
    template <HabinariPort Port, typename V>
    static OptionalPortValue<Port> always(V value)
    {
        static_assert(std::is_same_v<V, PortValue<Port>>,
                      "port value must be produced in the port's declared type");
        return value;
    }

    template <HabinariPort Port, typename V>
    static OptionalPortValue<Port> when(bool valid, V value)
    {
        return valid ? always<Port>(value) : std::nullopt;
    }

    PortState state_;
    habinari::control::CorrectedReadings readings_;
};

template <HabinariPort Port>
OptionalPortValue<Port> PortValues::get() const
{
    namespace hvac = habinari::hvac;
    using enum HabinariPort;
    const auto &out = state_.out;
    const auto &sensors = state_.sensorData;
    const auto &settings = state_.settings;
    const auto &r = readings_;

    // --- Room air measurements: the same corrected readings the control loops
    // use, and only while there is a valid one.
    if constexpr (Port == RoomTemperature) {
        return when<Port>(r.roomTemperatureValid, r.roomTemperatureC);
    } else if constexpr (Port == RoomHumidity) {
        return when<Port>(r.roomHumidityValid, r.roomHumidityPct);
    } else if constexpr (Port == RoomCo2) {
        return when<Port>(r.co2Valid, r.co2Ppm);
    } else if constexpr (Port == RoomAirPressure) {
        return when<Port>(r.pressureValid, r.pressurePa);
    } else if constexpr (Port == RoomAirPressureSeaLevel) {
        return when<Port>(r.pressureValid, out.seaLevelPressurePa);
    } else if constexpr (Port == RoomAirQualityIndex) {
        return when<Port>(r.iaqValid, static_cast<uint16_t>(r.iaqIndex + 0.5f));
    } else if constexpr (Port == AirQualityAccuracy) {
        return when<Port>(r.iaqValid, sensors.air_quality_accuracy);
    } else if constexpr (Port == RoomCo2Equivalent) {
        return when<Port>(sensors.co2_equivalent.valid, sensors.co2_equivalent.value);
    } else if constexpr (Port == RoomVocEquivalent) {
        return when<Port>(sensors.voc_equivalent.valid, sensors.voc_equivalent.value);
    }
    // --- Derived room air values ---
    else if constexpr (Port == RoomDewPoint) {
        return when<Port>(r.roomAirValid(), out.roomDewPointC);
    } else if constexpr (Port == RoomAbsoluteHumidity) {
        return when<Port>(r.roomAirValid(), out.roomAbsoluteHumidityGm3);
    }
    // --- Floor probe ---
    else if constexpr (Port == FloorTemperature) {
        return when<Port>(r.floorTemperatureValid, r.floorTemperatureC);
    } else if constexpr (Port == FloorHumidity) {
        return when<Port>(r.floorHumidityValid, r.floorHumidityPct);
    } else if constexpr (Port == FloorAbsoluteHumidity) {
        return when<Port>(r.floorProbeValid(), out.floorAbsoluteHumidityGm3);
    } else if constexpr (Port == FloorMoistureAlarm) {
        return always<Port>(out.floorMoistureAlarm);
    } else if constexpr (Port == FloorLimitActive) {
        return always<Port>(out.floorLimitActive);
    } else if constexpr (Port == FloorComfortActive) {
        return always<Port>(out.floorComfortActive);
    }
    // --- Condensation protection ---
    else if constexpr (Port == DewPointAlarm) {
        return always<Port>(out.dewPointAlarm);
    } else if constexpr (Port == DewPointMargin) {
        return always<Port>(out.dewPointMarginK);
    } else if constexpr (Port == FreeCoolingAvailable) {
        return always<Port>(out.freeCoolingAvailable);
    } else if constexpr (Port == FreeDryingAvailable) {
        return always<Port>(out.freeDryingAvailable);
    }
    // --- Mode and setpoints ---
    else if constexpr (Port == ControllerOnOff) {
        return always<Port>(state_.controllerOnOff);
    } else if constexpr (Port == HvacModeStatus) {
        // The mode the controller resolved to, which window/presence handling
        // can move away from the requested one.
        return always<Port>(static_cast<knx::application::Dpt20Mode>(out.activePreset));
    } else if constexpr (Port == ContrModeStatus || Port == ContrModeSecondary) {
        return always<Port>(hvac::controllerModeToContrMode(out.activeControllerMode));
    } else if constexpr (Port == SetpointBase) {
        return always<Port>(settings.comfortHeatingSetpointC);
    } else if constexpr (Port == SetpointShiftStatus) {
        return always<Port>(out.setpointShiftFeedbackK);
    } else if constexpr (Port == SetpointStatus) {
        return always<Port>(out.activeSetpointC);
    } else if constexpr (Port == SetpointHeatingStatus) {
        return always<Port>(out.heatingSetpointC);
    } else if constexpr (Port == SetpointCoolingStatus) {
        return always<Port>(out.coolingSetpointC);
    }
    // --- Controller outputs ---
    else if constexpr (Port == HeatingControlValue) {
        return always<Port>(out.heatingControlPercent);
    } else if constexpr (Port == CoolingControlValue) {
        return always<Port>(out.coolingControlPercent);
    } else if constexpr (Port == HeatingRequest) {
        return always<Port>(out.heatingRequest);
    } else if constexpr (Port == CoolingRequest) {
        return always<Port>(out.coolingRequest);
    } else if constexpr (Port == HeatCoolModeStatus) {
        return always<Port>(out.heatCoolModeHeating);
    } else if constexpr (Port == EnableHeatStatus) {
        return always<Port>(out.enableHeat);
    } else if constexpr (Port == EnableCoolStatus) {
        return always<Port>(out.enableCool);
    } else if constexpr (Port == ControllerStatus) {
        return always<Port>(out.controllerStatus);
    }
    // --- Ventilation / air quality ---
    else if constexpr (Port == Co2Setpoint) {
        return always<Port>(settings.ventilationSetpointPpm);
    } else if constexpr (Port == VentilationDemand) {
        return always<Port>(out.ventilationDemandPercent);
    } else if constexpr (Port == VentilationStage) {
        return always<Port>(static_cast<uint8_t>(out.ventilationLevel));
    } else if constexpr (Port == VentilationMode) {
        return always<Port>(static_cast<uint8_t>(state_.ventilationMode));
    } else if constexpr (Port == VentilationBoostRequest) {
        return always<Port>(out.ventilationBoostRequest);
    } else if constexpr (Port == DehumidifyRequest) {
        return always<Port>(out.dehumidifyRequest);
    } else if constexpr (Port == AirQualityStatus) {
        return always<Port>(out.airQualityStatus);
    }
    // --- Device diagnostics ---
    else if constexpr (Port == DeviceFault) {
        return always<Port>(out.deviceFault);
    } else if constexpr (Port == RoomSensorStatus) {
        return always<Port>(out.roomSensorStatus);
    } else if constexpr (Port == FloorProbeStatus) {
        return always<Port>(out.floorProbeStatus);
    } else if constexpr (Port == AirQualitySensorStatus) {
        return always<Port>(out.airQualitySensorStatus);
    } else if constexpr (Port == SensorHealthMask) {
        return always<Port>(out.sensorHealthMask);
    } else if constexpr (Port == SensorDisagreementAlarm) {
        return always<Port>(out.sensorDisagreement);
    }
    // --- Derived events ---
    // The alarms are sent unconditionally so a cleared alarm is reported too;
    // the measured trend only once its window has filled, because a half-filled
    // regression is not a trend.
    else if constexpr (Port == FireAlarm) {
        return always<Port>(sensors.events.fire_alarm);
    } else if constexpr (Port == FirePreAlarm) {
        return always<Port>(sensors.events.fire_pre_alarm);
    } else if constexpr (Port == TemperatureTrend) {
        return when<Port>(sensors.trends.temperature.valid,
                          sensors.trends.temperature.per_minute * 60.0f);
    } else if constexpr (Port == OccupancyDetected) {
        return always<Port>(sensors.events.occupancy_detected);
    } else if constexpr (Port == EstimatedOccupants) {
        return always<Port>(sensors.events.estimated_occupants);
    } else if constexpr (Port == WindowOpenDetected) {
        return always<Port>(sensors.events.window_open_detected);
    } else {
        static_assert(Port == RoomTemperature, "PortValues::get: not a transmitting port");
    }
}

/// What a read response carries when there is no valid reading: DPT 9 7FFFh
/// for a float (every float port is DPT 9.xxx), otherwise 0.
template <typename T>
constexpr T noReading()
{
    if constexpr (std::is_same_v<T, float>) {
        return kDpt9InvalidDecoded;
    } else {
        static_assert(std::is_integral_v<T>, "noReading: no 'no reading' form for this type");
        return T{0};
    }
}

/// The value a read request on @p Port is answered with.
template <HabinariPort Port>
PortValue<Port> readResponse(const PortValues &values)
{
    const OptionalPortValue<Port> value = values.template get<Port>();
    if constexpr (std::is_same_v<PortValue<Port>, float> || std::is_integral_v<PortValue<Port>>) {
        return value.has_value() ? *value : noReading<PortValue<Port>>();
    } else {
        return *value;  // never optional: get<Port>() always has a value for these
    }
}

namespace detail {

constexpr bool isTransmitPort(HabinariPort port)
{
    for (const auto &entry : kTransmitPorts) {
        if (entry.port == port) {
            return true;
        }
    }
    return false;
}

constexpr bool transmitPortsAreUnique()
{
    for (std::size_t i = 0; i < std::size(kTransmitPorts); ++i) {
        for (std::size_t j = i + 1; j < std::size(kTransmitPorts); ++j) {
            if (kTransmitPorts[i].port == kTransmitPorts[j].port) {
                return false;
            }
        }
    }
    return true;
}

template <typename PortsTuple>
struct ReadablePortsAreTransmitPorts;

template <typename... Ports>
struct ReadablePortsAreTransmitPorts<std::tuple<Ports...>> {
    static constexpr bool value = ((Ports::readable == isTransmitPort(Ports::logicalId)) && ...);
};

}  // namespace detail

static_assert(detail::transmitPortsAreUnique(), "kTransmitPorts lists a port twice");
static_assert(
    detail::ReadablePortsAreTransmitPorts<HabinariEndpointDefinition::ports_tuple>::value,
    "kTransmitPorts must list exactly the product's readable ports");

}  // namespace habinari_knx
