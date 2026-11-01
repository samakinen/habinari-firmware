// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * Host tests for the command gate in main/include/control_command.hpp, which is
 * all of control_state_write() except the lock.
 *
 * Every adapter's writes land here — Modbus registers, MQTT JSON, BLE — so this
 * is where a NaN from a "nan" string, an infinity from a float register, or a
 * code point nobody defined has to stop. Before it did, a NaN setpoint shift
 * reached the PID integrator and an out-of-range float was cast straight to an
 * enumeration, which is undefined behaviour.
 */

#include "control_command.hpp"

#include <unity.h>

#include <cmath>
#include <limits>

using namespace habinari::control;
namespace hvac = habinari::hvac;

namespace {

constexpr float kNan = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

/// The model a command lands on, with the flag the service keeps beside it.
struct Model {
    Settings settings{};
    Inputs inputs{};
    bool acknowledge{false};

    esp_err_t write(control_command_t command, float value)
    {
        return applyCommand(command, value, settings, inputs, acknowledge);
    }
};

constexpr control_command_t kAllCommands[] = {
    CONTROL_CMD_CONTROLLER_ON_OFF, CONTROL_CMD_HVAC_MODE,       CONTROL_CMD_CONTROLLER_MODE,
    CONTROL_CMD_SETPOINT_BASE,     CONTROL_CMD_SETPOINT_SHIFT,  CONTROL_CMD_WINDOW_STATUS,
    CONTROL_CMD_PRESENCE,          CONTROL_CMD_VENTILATION_MODE, CONTROL_CMD_CO2_SETPOINT,
    CONTROL_CMD_ACKNOWLEDGE_ALARMS,
};

void test_non_finite_values_are_refused_by_every_command()
{
    for (const control_command_t command : kAllCommands) {
        for (const float bad : {kNan, kInf, -kInf}) {
            Model model;
            const Model before = model;
            TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(command, bad));
            // Refused means untouched: NaN != 0 would have read as "on".
            TEST_ASSERT_EQUAL(before.inputs.controllerOnOff, model.inputs.controllerOnOff);
            TEST_ASSERT_EQUAL(before.inputs.windowStatusKnown, model.inputs.windowStatusKnown);
            TEST_ASSERT_EQUAL(before.inputs.presenceKnown, model.inputs.presenceKnown);
            // Exact float comparisons: the harness's tolerance check is false
            // for NaN, so TEST_ASSERT_EQUAL_FLOAT would let a stored NaN through.
            TEST_ASSERT_TRUE(before.inputs.setpointShiftK == model.inputs.setpointShiftK);
            TEST_ASSERT_TRUE(before.settings.comfortHeatingSetpointC
                             == model.settings.comfortHeatingSetpointC);
            TEST_ASSERT_TRUE(before.settings.ventilationSetpointPpm
                             == model.settings.ventilationSetpointPpm);
            TEST_ASSERT_FALSE(model.acknowledge);
        }
    }
}

void test_an_unknown_command_is_refused()
{
    Model model;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          model.write(static_cast<control_command_t>(99), 1.0f));
}

void test_enumerations_accept_only_their_code_points()
{
    Model model;

    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_HVAC_MODE, 4.0f));
    TEST_ASSERT_EQUAL(hvac::OperatingPreset::BuildingProtection, model.inputs.hvacOperatingMode);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_HVAC_MODE, 5.0f));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_HVAC_MODE, -1.0f));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_HVAC_MODE, 1.5f));
    // 256 used to wrap through uint8_t to 0 (Auto); 1e10 was undefined behaviour.
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_HVAC_MODE, 256.0f));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_HVAC_MODE, 1e10f));
    TEST_ASSERT_EQUAL(hvac::OperatingPreset::BuildingProtection, model.inputs.hvacOperatingMode);

    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_CONTROLLER_MODE, 2.0f));
    TEST_ASSERT_EQUAL(hvac::ControllerMode::Cool, model.inputs.controllerMode);
    // Out of range is refused, not quietly turned into Auto.
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_CONTROLLER_MODE, 4.0f));
    TEST_ASSERT_EQUAL(hvac::ControllerMode::Cool, model.inputs.controllerMode);

    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_VENTILATION_MODE, 3.0f));
    TEST_ASSERT_EQUAL(hvac::VentilationMode::Boost, model.inputs.ventilationMode);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_VENTILATION_MODE, 4.0f));
    TEST_ASSERT_EQUAL(hvac::VentilationMode::Boost, model.inputs.ventilationMode);
}

void test_setpoint_base_is_range_checked_then_clamped_to_the_installation()
{
    Model model;
    model.settings.minSetpointC = 16.0f;
    model.settings.maxSetpointC = 26.0f;

    // Inside the ETS envelope but outside this installation's limits: clamped,
    // exactly as the KNX write does.
    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_SETPOINT_BASE, 30.0f));
    TEST_ASSERT_EQUAL_FLOAT(26.0f, model.settings.comfortHeatingSetpointC);
    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_SETPOINT_BASE, 21.5f));
    TEST_ASSERT_EQUAL_FLOAT(21.5f, model.settings.comfortHeatingSetpointC);

    // Outside anything ETS can configure: a scaling bug at the other end.
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_SETPOINT_BASE, 2.9f));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_SETPOINT_BASE, 45.1f));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_SETPOINT_BASE, 215.0f));
    TEST_ASSERT_EQUAL_FLOAT(21.5f, model.settings.comfortHeatingSetpointC);
}

void test_setpoint_shift_and_co2_setpoint_are_range_checked()
{
    Model model;

    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_SETPOINT_SHIFT, -2.5f));
    TEST_ASSERT_EQUAL_FLOAT(-2.5f, model.inputs.setpointShiftK);
    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_SETPOINT_SHIFT, 10.0f));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_SETPOINT_SHIFT, 10.5f));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_SETPOINT_SHIFT, -10.5f));
    TEST_ASSERT_EQUAL_FLOAT(10.0f, model.inputs.setpointShiftK);

    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_CO2_SETPOINT, 1000.0f));
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, model.settings.ventilationSetpointPpm);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_CO2_SETPOINT, 399.0f));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_CO2_SETPOINT, 5001.0f));
    // DPT 9's "invalid data" 0x7FFF decodes to 670 760; it must not land as a setpoint.
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, model.write(CONTROL_CMD_CO2_SETPOINT, 670760.0f));
    TEST_ASSERT_EQUAL_FLOAT(1000.0f, model.settings.ventilationSetpointPpm);
}

void test_booleans_and_acknowledge_still_work()
{
    Model model;
    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_CONTROLLER_ON_OFF, 0.0f));
    TEST_ASSERT_FALSE(model.inputs.controllerOnOff);
    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_WINDOW_STATUS, 1.0f));
    TEST_ASSERT_TRUE(model.inputs.windowOpen);
    TEST_ASSERT_TRUE(model.inputs.windowStatusKnown);
    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_PRESENCE, 1.0f));
    TEST_ASSERT_TRUE(model.inputs.presence);
    TEST_ASSERT_TRUE(model.inputs.presenceKnown);
    TEST_ASSERT_EQUAL_INT(ESP_OK, model.write(CONTROL_CMD_ACKNOWLEDGE_ALARMS, 1.0f));
    TEST_ASSERT_TRUE(model.acknowledge);
}

}  // namespace

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_non_finite_values_are_refused_by_every_command);
    RUN_TEST(test_an_unknown_command_is_refused);
    RUN_TEST(test_enumerations_accept_only_their_code_points);
    RUN_TEST(test_setpoint_base_is_range_checked_then_clamped_to_the_installation);
    RUN_TEST(test_setpoint_shift_and_co2_setpoint_are_range_checked);
    RUN_TEST(test_booleans_and_acknowledge_still_work);

    return UNITY_END();
}
