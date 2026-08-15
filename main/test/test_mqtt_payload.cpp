// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * Host tests for the MQTT wire format in main/include/mqtt_payload.h.
 *
 * The Wi-Fi association and the broker session need hardware; the format does
 * not, and the format is where integration bugs actually live. Everything
 * asserted here is a failure that would be silent in the field: a reading
 * published as 0 instead of omitted, a payload with a trailing newline parsed
 * as nothing, a mode name published in one vocabulary and accepted in another.
 *
 * The enum code points are cross-checked against hvac_control.hpp, because
 * mqtt_payload.c is C and has to mirror them by hand.
 */

#include "mqtt_payload.h"

#include "hvac_control.hpp"

#include <unity.h>

#include <cstring>
#include <string>

namespace hvac = habinari::hvac;

namespace {

std::string emit(const control_state_t &state)
{
    char buf[MQTT_PAYLOAD_STATE_MAX];
    const int n = mqtt_payload_state_json(buf, sizeof(buf), &state);
    TEST_ASSERT_TRUE(n > 0);
    TEST_ASSERT_EQUAL_INT(n, static_cast<int>(std::strlen(buf)));
    return std::string(buf);
}

bool contains(const std::string &haystack, const char *needle)
{
    return haystack.find(needle) != std::string::npos;
}

control_state_t liveState()
{
    control_state_t s{};
    s.has_sensor_data = true;
    s.sensors.temperature.value = 21.5f;
    s.sensors.temperature.valid = true;
    s.sensors.humidity.value = 44.0f;
    s.sensors.humidity.valid = true;
    s.sensors.co2.value = 780.0f;
    s.sensors.co2.valid = true;
    s.active_setpoint_c = 21.0f;
    s.comfort_setpoint_c = 21.0f;
    s.controller_on = true;
    s.controller_mode = static_cast<uint8_t>(hvac::ControllerMode::Heat);
    s.hvac_operating_mode = static_cast<uint8_t>(hvac::OperatingPreset::Comfort);
    s.room_dew_point_c = 8.9f;
    s.room_absolute_humidity_gm3 = 8.2f;
    return s;
}

// ---------------------------------------------------------------------------
// The mirrored code points
// ---------------------------------------------------------------------------

void test_preset_names_cover_every_operating_preset()
{
    TEST_ASSERT_EQUAL_STRING(
        "auto", mqtt_payload_preset_name(static_cast<uint8_t>(hvac::OperatingPreset::Auto)));
    TEST_ASSERT_EQUAL_STRING(
        "comfort", mqtt_payload_preset_name(static_cast<uint8_t>(hvac::OperatingPreset::Comfort)));
    TEST_ASSERT_EQUAL_STRING(
        "standby", mqtt_payload_preset_name(static_cast<uint8_t>(hvac::OperatingPreset::Standby)));
    TEST_ASSERT_EQUAL_STRING(
        "eco", mqtt_payload_preset_name(static_cast<uint8_t>(hvac::OperatingPreset::Economy)));
    TEST_ASSERT_EQUAL_STRING("away", mqtt_payload_preset_name(static_cast<uint8_t>(
                                         hvac::OperatingPreset::BuildingProtection)));
    // Anything outside the enum must be absent, not guessed at.
    TEST_ASSERT_NULL(mqtt_payload_preset_name(200));
}

void test_mode_names_cover_every_controller_mode()
{
    TEST_ASSERT_EQUAL_STRING(
        "auto", mqtt_payload_mode_name(static_cast<uint8_t>(hvac::ControllerMode::Auto), true));
    TEST_ASSERT_EQUAL_STRING(
        "heat", mqtt_payload_mode_name(static_cast<uint8_t>(hvac::ControllerMode::Heat), true));
    TEST_ASSERT_EQUAL_STRING(
        "cool", mqtt_payload_mode_name(static_cast<uint8_t>(hvac::ControllerMode::Cool), true));
    TEST_ASSERT_EQUAL_STRING(
        "off", mqtt_payload_mode_name(static_cast<uint8_t>(hvac::ControllerMode::Off), true));
    // Home Assistant models "disabled" as the off hvac_mode, not a separate
    // switch, so a disabled controller reports off whatever mode it holds.
    TEST_ASSERT_EQUAL_STRING(
        "off", mqtt_payload_mode_name(static_cast<uint8_t>(hvac::ControllerMode::Heat), false));
}

// ---------------------------------------------------------------------------
// State document
// ---------------------------------------------------------------------------

void test_state_document_carries_the_valid_measurements()
{
    const std::string json = emit(liveState());
    TEST_ASSERT_TRUE(contains(json, "\"temperature\":21.50"));
    TEST_ASSERT_TRUE(contains(json, "\"humidity\":44.0"));
    TEST_ASSERT_TRUE(contains(json, "\"co2\":780"));
    TEST_ASSERT_TRUE(contains(json, "\"mode\":\"heat\""));
    TEST_ASSERT_TRUE(contains(json, "\"preset\":\"comfort\""));
    TEST_ASSERT_TRUE(json.front() == '{' && json.back() == '}');
}

void test_an_invalid_measurement_is_omitted_rather_than_published_as_zero()
{
    // The redundancy layer can genuinely report "there is no room temperature".
    // A subscriber that cannot tell that from 0 °C will heat the room to the
    // setpoint against a fabricated reading.
    control_state_t s = liveState();
    s.sensors.temperature.valid = false;
    s.sensors.co2.valid = false;

    const std::string json = emit(s);
    TEST_ASSERT_FALSE(contains(json, "\"temperature\""));
    TEST_ASSERT_FALSE(contains(json, "\"co2\":"));
    // The valid neighbours must survive its absence.
    TEST_ASSERT_TRUE(contains(json, "\"humidity\":44.0"));
}

void test_derived_values_are_omitted_without_a_valid_room_reading()
{
    control_state_t s = liveState();
    s.sensors.humidity.valid = false;
    const std::string json = emit(s);
    TEST_ASSERT_FALSE(contains(json, "\"dew_point\""));
}

void test_no_sensor_data_still_produces_a_valid_document()
{
    // A board that has not completed its first sampling cycle must still report
    // its setpoints and mode, so a UI can render before the first reading.
    control_state_t s{};
    s.controller_on = true;
    s.active_setpoint_c = 21.0f;

    const std::string json = emit(s);
    TEST_ASSERT_TRUE(contains(json, "\"setpoint\":21.00"));
    TEST_ASSERT_FALSE(contains(json, "\"temperature\""));
}

void test_action_reflects_what_the_device_is_actually_doing()
{
    control_state_t s = liveState();
    TEST_ASSERT_EQUAL_STRING("idle", mqtt_payload_action_name(&s));

    s.heating_percent = 40;
    TEST_ASSERT_EQUAL_STRING("heating", mqtt_payload_action_name(&s));

    s.heating_percent = 0;
    s.cooling_percent = 25;
    TEST_ASSERT_EQUAL_STRING("cooling", mqtt_payload_action_name(&s));

    s.controller_on = false;
    TEST_ASSERT_EQUAL_STRING("off", mqtt_payload_action_name(&s));
}

void test_a_short_buffer_fails_cleanly_rather_than_truncating()
{
    // A truncated JSON document is worse than none: a subscriber would parse
    // half a state and act on it.
    char buf[64];
    const control_state_t s = liveState();
    TEST_ASSERT_EQUAL_INT(-1, mqtt_payload_state_json(buf, sizeof(buf), &s));
    TEST_ASSERT_EQUAL_INT(0, buf[0]);
}

// ---------------------------------------------------------------------------
// Command parsing
// ---------------------------------------------------------------------------

void expectCommand(const char *topic, const std::string &payload, control_command_t expected,
                   float expectedValue)
{
    control_command_t command;
    float value = 0.0f;
    const bool ok =
        mqtt_payload_parse_command(topic, payload.data(), payload.size(), &command, &value);
    if (!ok) {
        char message[128];
        std::snprintf(message, sizeof(message), "topic '%s' payload '%s' was not accepted", topic,
                      payload.c_str());
        TEST_FAIL_MESSAGE(message);
    }
    TEST_ASSERT_EQUAL_INT(expected, command);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expectedValue, value);
}

void test_numeric_commands_parse()
{
    expectCommand("setpoint", "21.5", CONTROL_CMD_SETPOINT_BASE, 21.5f);
    expectCommand("setpoint_shift", "-1.5", CONTROL_CMD_SETPOINT_SHIFT, -1.5f);
    expectCommand("co2_setpoint", "800", CONTROL_CMD_CO2_SETPOINT, 800.0f);
}

void test_payload_whitespace_and_newlines_are_tolerated()
{
    // Brokers and bridges routinely append a newline. "21.5\n" parsing as
    // nothing — or worse, as 0 — is a setpoint quietly ignored.
    expectCommand("setpoint", "21.5\n", CONTROL_CMD_SETPOINT_BASE, 21.5f);
    expectCommand("setpoint", " 21.5\r\n", CONTROL_CMD_SETPOINT_BASE, 21.5f);
    expectCommand("setpoint", "\t21.5 ", CONTROL_CMD_SETPOINT_BASE, 21.5f);
}

void test_home_assistant_boolean_vocabularies_all_work()
{
    expectCommand("power", "ON", CONTROL_CMD_CONTROLLER_ON_OFF, 1.0f);
    expectCommand("power", "off", CONTROL_CMD_CONTROLLER_ON_OFF, 0.0f);
    expectCommand("power", "true", CONTROL_CMD_CONTROLLER_ON_OFF, 1.0f);
    expectCommand("power", "0", CONTROL_CMD_CONTROLLER_ON_OFF, 0.0f);
    expectCommand("window", "open", CONTROL_CMD_WINDOW_STATUS, 1.0f);
    expectCommand("window", "closed", CONTROL_CMD_WINDOW_STATUS, 0.0f);
}

void test_named_modes_map_to_the_same_code_points_that_are_published()
{
    // The round trip that matters: what the device publishes as `mode` must be
    // accepted back on cmd/mode and land on the same enum value. Publishing in
    // one vocabulary and accepting another is the classic silent integration
    // failure.
    for (uint8_t mode = 0; mode <= 3; ++mode) {
        const char *name = mqtt_payload_mode_name(mode, true);
        TEST_ASSERT_NOT_NULL(name);
        expectCommand("mode", name, CONTROL_CMD_CONTROLLER_MODE, static_cast<float>(mode));
    }

    for (uint8_t preset = 0; preset <= 4; ++preset) {
        const char *name = mqtt_payload_preset_name(preset);
        TEST_ASSERT_NOT_NULL(name);
        expectCommand("preset", name, CONTROL_CMD_HVAC_MODE, static_cast<float>(preset));
    }
}

void test_named_commands_are_case_insensitive_and_accept_raw_code_points()
{
    expectCommand("mode", "HEAT", CONTROL_CMD_CONTROLLER_MODE,
                  static_cast<float>(hvac::ControllerMode::Heat));
    // Non-Home-Assistant clients may prefer the numbers.
    expectCommand("mode", "2", CONTROL_CMD_CONTROLLER_MODE,
                  static_cast<float>(hvac::ControllerMode::Cool));
}

void test_alarm_acknowledgement_accepts_an_empty_payload()
{
    // A bare publish is the natural way to express "acknowledge"; the command
    // ignores its value by definition.
    expectCommand("acknowledge_alarms", "", CONTROL_CMD_ACKNOWLEDGE_ALARMS, 1.0f);
    expectCommand("acknowledge_alarms", "PRESS", CONTROL_CMD_ACKNOWLEDGE_ALARMS, 1.0f);
}

void test_unknown_topics_and_unparseable_payloads_are_rejected()
{
    control_command_t command;
    float value = 0.0f;

    TEST_ASSERT_FALSE(mqtt_payload_parse_command("nonsense", "1", 1, &command, &value));
    TEST_ASSERT_FALSE(mqtt_payload_parse_command("setpoint", "warm", 4, &command, &value));
    TEST_ASSERT_FALSE(mqtt_payload_parse_command("setpoint", "", 0, &command, &value));
    // A partially numeric payload must not parse as its numeric prefix: "21x"
    // is a mistake, and silently applying 21 hides it.
    TEST_ASSERT_FALSE(mqtt_payload_parse_command("setpoint", "21x", 3, &command, &value));
    TEST_ASSERT_FALSE(mqtt_payload_parse_command("mode", "warmish", 7, &command, &value));
}

void test_an_overlong_payload_is_rejected_rather_than_truncated()
{
    const std::string huge(200, '9');
    control_command_t command;
    float value = 0.0f;
    TEST_ASSERT_FALSE(
        mqtt_payload_parse_command("setpoint", huge.data(), huge.size(), &command, &value));
}

}  // namespace

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_preset_names_cover_every_operating_preset);
    RUN_TEST(test_mode_names_cover_every_controller_mode);

    RUN_TEST(test_state_document_carries_the_valid_measurements);
    RUN_TEST(test_an_invalid_measurement_is_omitted_rather_than_published_as_zero);
    RUN_TEST(test_derived_values_are_omitted_without_a_valid_room_reading);
    RUN_TEST(test_no_sensor_data_still_produces_a_valid_document);
    RUN_TEST(test_action_reflects_what_the_device_is_actually_doing);
    RUN_TEST(test_a_short_buffer_fails_cleanly_rather_than_truncating);

    RUN_TEST(test_numeric_commands_parse);
    RUN_TEST(test_payload_whitespace_and_newlines_are_tolerated);
    RUN_TEST(test_home_assistant_boolean_vocabularies_all_work);
    RUN_TEST(test_named_modes_map_to_the_same_code_points_that_are_published);
    RUN_TEST(test_named_commands_are_case_insensitive_and_accept_raw_code_points);
    RUN_TEST(test_alarm_acknowledgement_accepts_an_empty_payload);
    RUN_TEST(test_unknown_topics_and_unparseable_payloads_are_rejected);
    RUN_TEST(test_an_overlong_payload_is_rejected_rather_than_truncated);

    return UNITY_END();
}
