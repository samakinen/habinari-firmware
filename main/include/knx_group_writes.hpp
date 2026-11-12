// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "control_core.hpp"
#include "control_state.h"
#include "knx_dpt9.hpp"
#include "knx_product.hpp"

#include <cstdint>

/**
 * @file knx_group_writes.hpp
 * @brief What an inbound KNX group write means for the control state.
 *
 * A write that has a control_state_write() equivalent goes through it, so a KNX
 * telegram meets the same gate a Modbus or MQTT write does — finite, in range,
 * a defined code point — and a refusal is logged instead of cast into the
 * controller. This header is the KNX half of that: which command a group object
 * stands for, and how its bus value becomes the command's float. What has no
 * command (the neighbour-device measurements) is stored into Inputs here too.
 *
 * Kept apart from the KNX service, which needs the stack and FreeRTOS, so the
 * host tests can pin the mapping — including the DPT 9 7FFFh "invalid data"
 * code, which is never a value to act on (knx_dpt9.hpp). Nothing here locks or
 * logs; the service does both around it.
 */

namespace habinari_knx {

/// What an inbound write on a command port asks of control_state_write().
struct GroupCommand {
    enum class Kind : uint8_t {
        Apply,        ///< write `value` to `command`
        InvalidData,  ///< DPT 9 7FFFh: the sender has no value, so nothing is applied
        Ignore,       ///< a telegram that asks for nothing (alarm acknowledge = 0)
    };

    Kind kind{Kind::Ignore};
    control_command_t command{CONTROL_CMD_CONTROLLER_ON_OFF};
    float value{0.0f};
};

namespace detail {

constexpr GroupCommand applyCommand(control_command_t command, float value)
{
    return GroupCommand{GroupCommand::Kind::Apply, command, value};
}

constexpr GroupCommand flagCommand(control_command_t command, bool value)
{
    return applyCommand(command, value ? 1.0f : 0.0f);
}

/// A DPT 9 value, or the "invalid data" code standing in for one.
constexpr GroupCommand dpt9Command(control_command_t command, float value)
{
    return dpt9IsInvalid(value) ? GroupCommand{GroupCommand::Kind::InvalidData, command, value}
                                : applyCommand(command, value);
}

}  // namespace detail

/// The group objects that are commands. Each names its control command, the
/// label the service logs a refusal under, and how the bus value maps. A port
/// with no specialisation here is not a command port.
template <HabinariPort Port>
struct CommandPort;

template <>
struct CommandPort<HabinariPort::ControllerOnOff> {
    static constexpr const char *label = "controller on/off";
    static constexpr GroupCommand map(bool value)
    {
        return detail::flagCommand(CONTROL_CMD_CONTROLLER_ON_OFF, value);
    }
};

template <>
struct CommandPort<HabinariPort::HvacMode> {
    static constexpr const char *label = "hvac mode";
    // The raw bus byte, unvalidated: control_state_write() refuses a code
    // point OperatingPreset does not define instead of casting it in.
    static constexpr GroupCommand map(application::Dpt20Mode value)
    {
        return detail::applyCommand(CONTROL_CMD_HVAC_MODE,
                                    static_cast<float>(static_cast<uint8_t>(value)));
    }
};

template <>
struct CommandPort<HabinariPort::ContrMode> {
    static constexpr const char *label = "controller mode";
    // The bus objects carry DPT 20.105 (HVACContrMode) code points; the
    // internal enum is compact. Map at the binding boundary only.
    static constexpr GroupCommand map(uint8_t value)
    {
        return detail::applyCommand(
            CONTROL_CMD_CONTROLLER_MODE,
            static_cast<float>(habinari::hvac::controllerModeFromContrMode(value)));
    }
};

template <>
struct CommandPort<HabinariPort::SetpointBase> {
    static constexpr const char *label = "base setpoint";
    // Clamped to the configured limits inside control_state_write(), the same
    // as a write from any other adapter.
    static constexpr GroupCommand map(float value)
    {
        return detail::dpt9Command(CONTROL_CMD_SETPOINT_BASE, value);
    }
};

template <>
struct CommandPort<HabinariPort::SetpointShift> {
    static constexpr const char *label = "setpoint shift";
    static constexpr GroupCommand map(float value)
    {
        return detail::dpt9Command(CONTROL_CMD_SETPOINT_SHIFT, value);
    }
};

template <>
struct CommandPort<HabinariPort::WindowStatus> {
    static constexpr const char *label = "window status";
    static constexpr GroupCommand map(bool value)
    {
        return detail::flagCommand(CONTROL_CMD_WINDOW_STATUS, value);
    }
};

template <>
struct CommandPort<HabinariPort::PresenceStatus> {
    static constexpr const char *label = "presence";
    static constexpr GroupCommand map(bool value)
    {
        return detail::flagCommand(CONTROL_CMD_PRESENCE, value);
    }
};

template <>
struct CommandPort<HabinariPort::Co2Setpoint> {
    static constexpr const char *label = "CO2 setpoint";
    static constexpr GroupCommand map(float value)
    {
        return detail::dpt9Command(CONTROL_CMD_CO2_SETPOINT, value);
    }
};

template <>
struct CommandPort<HabinariPort::VentilationMode> {
    static constexpr const char *label = "ventilation mode";
    static constexpr GroupCommand map(uint8_t value)
    {
        return detail::applyCommand(CONTROL_CMD_VENTILATION_MODE, static_cast<float>(value));
    }
};

template <>
struct CommandPort<HabinariPort::AlarmAcknowledge> {
    static constexpr const char *label = "alarm acknowledge";
    // Only a 1 acknowledges; a 0 is the button being released.
    static constexpr GroupCommand map(bool value)
    {
        return value ? detail::applyCommand(CONTROL_CMD_ACKNOWLEDGE_ALARMS, 1.0f) : GroupCommand{};
    }
};

/// The command an inbound write on @p Port carries.
template <HabinariPort Port, typename T>
constexpr GroupCommand groupCommand(T value)
{
    return CommandPort<Port>::map(value);
}

/// A neighbouring device's measurement — outside temperature and humidity,
/// flow temperature — stored into @p inputs. A neighbour that reports invalid
/// data has no reading, so its last one stops being used rather than being
/// trusted indefinitely: the value is left as it was and marked invalid.
/// Returns whether the reading was usable. The caller holds the control lock.
template <HabinariPort Port>
constexpr bool storeNeighbourReading(habinari::control::Inputs &inputs, float value)
{
    const bool usable = !dpt9IsInvalid(value);
    if constexpr (Port == HabinariPort::OutsideTemperature) {
        if (usable) {
            inputs.outsideTemperatureC = value;
        }
        inputs.outsideTemperatureValid = usable;
    } else if constexpr (Port == HabinariPort::OutsideHumidity) {
        if (usable) {
            inputs.outsideHumidityPct = habinari::hvac::clampf(value, 0.0f, 100.0f);
        }
        inputs.outsideHumidityValid = usable;
    } else if constexpr (Port == HabinariPort::FlowTemperature) {
        if (usable) {
            inputs.flowTemperatureC = value;
        }
        inputs.flowTemperatureValid = usable;
    } else {
        static_assert(Port == HabinariPort::OutsideTemperature,
                      "storeNeighbourReading: not a neighbour-device input");
    }
    return usable;
}

}  // namespace habinari_knx
