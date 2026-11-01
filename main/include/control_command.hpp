// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "control_core.hpp"
#include "control_state.h"

#include <cmath>

/**
 * @file control_command.hpp
 * @brief Validation and application of control_state_write() commands.
 *
 * Split out of control_service.cpp so the gate every adapter's writes pass
 * through can be host-tested: the service adds only the lock around it. A C++
 * adapter that writes ServiceState directly (the KNX one does, for its own
 * group objects) can call validateCommand() with the same rules, so a NaN or an
 * out-of-range code point is refused the same way whichever bus it came from.
 *
 * Platform-free, like control_core.hpp. Nothing here locks.
 */

namespace habinari {
namespace control {

// The widest values the ETS parameters allow (knx_product.hpp): MinSetpoint
// goes down to 3 °C, MaxSetpoint up to 45 °C, MaxSetpointShift up to 10 K and
// VentilationSetpoint spans 400..5000 ppm. Anything beyond these cannot be a
// legitimate request on any installation, so it is refused rather than clamped:
// a master writing 600 °C has a scaling bug, and silently adopting 35 °C would
// hide it.
inline constexpr float kCommandSetpointMinC = 3.0f;
inline constexpr float kCommandSetpointMaxC = 45.0f;
inline constexpr float kCommandSetpointShiftMaxK = 10.0f;
inline constexpr float kCommandCo2SetpointMinPpm = 400.0f;
inline constexpr float kCommandCo2SetpointMaxPpm = 5000.0f;

namespace detail {

/// A code point is a whole number inside [0, max]. Checked on the float, before
/// any cast: converting an out-of-range float to an integer is undefined
/// behaviour, so the check cannot come after it.
inline bool isCodePoint(float value, float max)
{
    return value >= 0.0f && value <= max && std::floor(value) == value;
}

inline bool inRange(float value, float lo, float hi)
{
    return value >= lo && value <= hi;
}

}  // namespace detail

/// ESP_OK if @p value is acceptable for @p command, ESP_ERR_INVALID_ARG if not
/// (including an unknown command). See control_state_write() for the ranges.
inline esp_err_t validateCommand(control_command_t command, float value)
{
    // Every payload, booleans included: NaN != 0 is true, so an unchecked NaN
    // would have switched the controller on.
    if (!std::isfinite(value)) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (command) {
    case CONTROL_CMD_CONTROLLER_ON_OFF:
    case CONTROL_CMD_WINDOW_STATUS:
    case CONTROL_CMD_PRESENCE:
    case CONTROL_CMD_ACKNOWLEDGE_ALARMS:
        return ESP_OK;
    case CONTROL_CMD_HVAC_MODE:
        return detail::isCodePoint(value, static_cast<float>(hvac_ns::OperatingPreset::BuildingProtection))
                   ? ESP_OK
                   : ESP_ERR_INVALID_ARG;
    case CONTROL_CMD_CONTROLLER_MODE:
        return detail::isCodePoint(value, static_cast<float>(hvac_ns::ControllerMode::Off))
                   ? ESP_OK
                   : ESP_ERR_INVALID_ARG;
    case CONTROL_CMD_VENTILATION_MODE:
        return detail::isCodePoint(value, static_cast<float>(hvac_ns::VentilationMode::Boost))
                   ? ESP_OK
                   : ESP_ERR_INVALID_ARG;
    case CONTROL_CMD_SETPOINT_BASE:
        return detail::inRange(value, kCommandSetpointMinC, kCommandSetpointMaxC)
                   ? ESP_OK
                   : ESP_ERR_INVALID_ARG;
    case CONTROL_CMD_SETPOINT_SHIFT:
        return detail::inRange(value, -kCommandSetpointShiftMaxK, kCommandSetpointShiftMaxK)
                   ? ESP_OK
                   : ESP_ERR_INVALID_ARG;
    case CONTROL_CMD_CO2_SETPOINT:
        return detail::inRange(value, kCommandCo2SetpointMinPpm, kCommandCo2SetpointMaxPpm)
                   ? ESP_OK
                   : ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_INVALID_ARG;
}

/// Validate, then apply @p command to the shared model. Nothing is changed when
/// validation fails. The caller holds whatever lock guards the three targets.
inline esp_err_t applyCommand(control_command_t command,
                              float value,
                              Settings &settings,
                              Inputs &inputs,
                              bool &acknowledgeAlarmsRequested)
{
    const esp_err_t valid = validateCommand(command, value);
    if (valid != ESP_OK) {
        return valid;
    }

    const bool flag = value != 0.0f;
    switch (command) {
    case CONTROL_CMD_CONTROLLER_ON_OFF:
        inputs.controllerOnOff = flag;
        break;
    case CONTROL_CMD_HVAC_MODE:
        inputs.hvacOperatingMode =
            static_cast<hvac_ns::OperatingPreset>(static_cast<uint8_t>(value));
        break;
    case CONTROL_CMD_CONTROLLER_MODE:
        // Compact ControllerMode code points, the same ones control_state_t
        // reports back, so read-modify-write round-trips. KNX DPT 20.105 is
        // translated at the KNX edge (knx_service.cpp), not here.
        inputs.controllerMode = static_cast<hvac_ns::ControllerMode>(static_cast<uint8_t>(value));
        break;
    case CONTROL_CMD_SETPOINT_BASE:
        // Same clamping as the KNX SetpointBase write: the configured min/max
        // are a property of the installation, not of the protocol that asked.
        settings.comfortHeatingSetpointC =
            hvac_ns::clampf(value, settings.minSetpointC, settings.maxSetpointC);
        break;
    case CONTROL_CMD_SETPOINT_SHIFT:
        // Stored as asked; the thermostat clamps it to the configured maximum
        // shift on every tick, so lowering that limit later still takes effect.
        inputs.setpointShiftK = value;
        break;
    case CONTROL_CMD_WINDOW_STATUS:
        inputs.windowOpen = flag;
        inputs.windowStatusKnown = true;
        break;
    case CONTROL_CMD_PRESENCE:
        inputs.presence = flag;
        inputs.presenceKnown = true;
        break;
    case CONTROL_CMD_VENTILATION_MODE:
        inputs.ventilationMode = static_cast<hvac_ns::VentilationMode>(static_cast<uint8_t>(value));
        break;
    case CONTROL_CMD_CO2_SETPOINT:
        settings.ventilationSetpointPpm = value;
        break;
    case CONTROL_CMD_ACKNOWLEDGE_ALARMS:
        acknowledgeAlarmsRequested = true;
        break;
    }
    return ESP_OK;
}

}  // namespace control
}  // namespace habinari
