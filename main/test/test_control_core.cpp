// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * Host tests for the control core in main/include/control_core.hpp.
 *
 * This is the file that made the refactor worth doing. The whole room-control
 * model used to sit inside knx_service.cpp, wrapped in a FreeRTOS task, a mutex
 * and a KNX stack, and the only way to find out what the device would do with a
 * cold room and an open window was to build the firmware and open a window.
 *
 * Core::tick() is a pure function of (settings, inputs, measurements, dt) plus
 * the controller state it carries, so those questions are answerable here, in
 * milliseconds, at simulated time.
 *
 * The assertions below deliberately concentrate on the *structural* invariants
 * — the ones a protocol adapter could plausibly break — rather than on the
 * control maths, which test_hvac_control.cpp already covers against the
 * underlying controllers.
 */

#include "control_core.hpp"

#include <unity.h>

#include <cmath>
#include <cstdio>

using namespace habinari::control;
namespace hvac = habinari::hvac;

namespace {

/// A measurement record with the room air valid and everything else absent —
/// the state of a board with no floor probe fitted, which is the common case.
sensor_data_t roomAir(float temperatureC, float humidityPct)
{
    sensor_data_t d{};
    d.temperature.value = temperatureC;
    d.temperature.valid = true;
    d.humidity.value = humidityPct;
    d.humidity.valid = true;
    d.health.sample_count = 10;
    return d;
}

/// Run the core for `seconds` at 1 Hz and return the final outputs. Every test
/// that involves an integrator needs this: a PI loop tells you nothing after one
/// tick, and sleeping for real would make the suite unusable.
Outputs settle(Core &core, const Settings &settings, const Inputs &inputs,
               const sensor_data_t &sensors, int seconds)
{
    Outputs out;
    for (int i = 0; i < seconds; ++i) {
        out = core.tick(settings, inputs, sensors, 1.0f);
    }
    return out;
}

// ---------------------------------------------------------------------------
// Defaults and basic behaviour
// ---------------------------------------------------------------------------

void test_factory_defaults_produce_a_working_controller()
{
    // An un-commissioned device — Settings{} straight from control_defaults.hpp
    // — must still control the room. It used to be that a board nobody had
    // downloaded an ETS project into had no parameters at all.
    Core core;
    const Settings settings;
    const Inputs inputs;

    const Outputs out = settle(core, settings, inputs, roomAir(21.0f, 45.0f), 5);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.0f, out.activeSetpointC);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(hvac::OperatingPreset::Comfort),
                            static_cast<uint8_t>(out.activePreset));
    TEST_ASSERT_FALSE(out.deviceFault);
}

void test_a_cold_room_calls_for_heat_and_a_warm_one_does_not()
{
    const Settings settings;
    const Inputs inputs;

    Core cold;
    const Outputs coldOut = settle(cold, settings, inputs, roomAir(17.0f, 45.0f), 120);
    TEST_ASSERT_TRUE(coldOut.heatingControlPercent > 0);
    TEST_ASSERT_TRUE(coldOut.heatingRequest);

    Core warm;
    const Outputs warmOut = settle(warm, settings, inputs, roomAir(24.0f, 45.0f), 120);
    TEST_ASSERT_EQUAL_UINT8(0, warmOut.heatingControlPercent);
}

void test_losing_every_room_reading_is_a_device_fault()
{
    Core core;
    const Settings settings;
    const Inputs inputs;

    sensor_data_t dead{};
    dead.health.sample_count = 10;  // sampling, but nothing valid came back
    const Outputs out = settle(core, settings, inputs, dead, 5);

    TEST_ASSERT_TRUE(out.deviceFault);
    // SensorFaultBehavior::ForceOff is the default: an unmeasured room must not
    // be heated against a fabricated temperature.
    TEST_ASSERT_EQUAL_UINT8(0, out.heatingControlPercent);
}

void test_a_fallback_source_is_not_a_device_fault()
{
    // Three sensors measure room temperature. One dying and being replaced is
    // the redundancy working, and must not raise the roll-up alarm — otherwise
    // every installation eventually reports a fault it should not.
    Core core;
    sensor_data_t data = roomAir(21.0f, 45.0f);
    data.temperature.fallback = true;

    const Outputs out = settle(core, Settings{}, Inputs{}, data, 5);
    TEST_ASSERT_FALSE(out.deviceFault);
}


void test_a_board_with_no_floor_probe_still_heats()
{
    // Regression. The floor-temperature limit is enabled by default (28 °C) and
    // the default sensor-fault behaviour is ForceOff. The floor probe is
    // optional hardware, so on a board without one the limit used to read
    // "probe faulty" and latch floorLimitActive at boot — and a factory-fresh
    // device with no probe fitted never called for heat, in any room, at any
    // temperature. roomAir() deliberately leaves the probe absent, so almost
    // every other test here would have caught it too.
    Settings settings;
    settings.maxFloorTemperatureC = 28.0f;
    settings.sensorFaultBehavior = hvac::SensorFaultBehavior::ForceOff;

    Core core;
    const Outputs out = settle(core, settings, Inputs{}, roomAir(17.0f, 45.0f), 120);

    TEST_ASSERT_FALSE(out.floorLimitActive);
    TEST_ASSERT_TRUE(out.heatingControlPercent > 0);
}

void test_a_floor_probe_that_fails_after_reporting_is_a_real_fault()
{
    // The other half of the distinction: a probe that *has* reported and then
    // stops is a genuine failure, and ForceOff must still block heating. There
    // is a real slab under there and no way to know how hot it is.
    Settings settings;
    settings.maxFloorTemperatureC = 28.0f;
    settings.sensorFaultBehavior = hvac::SensorFaultBehavior::ForceOff;

    sensor_data_t withProbe = roomAir(17.0f, 45.0f);
    withProbe.probe_temperature.value = 24.0f;
    withProbe.probe_temperature.valid = true;
    withProbe.probe_humidity.value = 50.0f;
    withProbe.probe_humidity.valid = true;

    Core core;
    const Outputs healthy = settle(core, settings, Inputs{}, withProbe, 60);
    TEST_ASSERT_FALSE(healthy.floorLimitActive);
    TEST_ASSERT_TRUE(healthy.heatingControlPercent > 0);

    // Same core — the probe drops out mid-life.
    const Outputs failed = settle(core, settings, Inputs{}, roomAir(17.0f, 45.0f), 10);
    TEST_ASSERT_TRUE(failed.floorLimitActive);
    TEST_ASSERT_EQUAL_UINT8(0, failed.heatingControlPercent);
}

// ---------------------------------------------------------------------------
// Inputs the adapters write — the ones most likely to be mis-mapped
// ---------------------------------------------------------------------------

void test_an_open_window_blocks_heating()
{
    Settings settings;
    settings.windowOpenBehavior = hvac::WindowOpenBehavior::BlockOutputs;

    Inputs inputs;
    inputs.windowOpen = true;
    inputs.windowStatusKnown = true;

    Core core;
    const Outputs out = settle(core, settings, inputs, roomAir(15.0f, 45.0f), 120);
    TEST_ASSERT_EQUAL_UINT8(0, out.heatingControlPercent);
    TEST_ASSERT_FALSE(out.enableHeat);
}

void test_a_detected_window_substitutes_only_while_nothing_is_linked()
{
    // The board can infer an open window from a temperature slump. That
    // inference must yield to a real contact the moment one reports — an
    // installation with a hardware contact has the truth, and an inference that
    // overrode it would block heating in a closed room.
    Settings settings;
    settings.windowOpenBehavior = hvac::WindowOpenBehavior::BlockOutputs;

    sensor_data_t data = roomAir(15.0f, 45.0f);
    data.events.window_open_detected = true;

    // Nothing linked: the inference applies.
    Inputs unlinked;
    Core inferred;
    const Outputs inferredOut = settle(inferred, settings, unlinked, data, 60);
    TEST_ASSERT_EQUAL_UINT8(0, inferredOut.heatingControlPercent);

    // A contact has reported "closed": the inference must be ignored.
    Inputs linked;
    linked.windowStatusKnown = true;
    linked.windowOpen = false;
    Core reported;
    const Outputs reportedOut = settle(reported, settings, linked, data, 60);
    TEST_ASSERT_TRUE(reportedOut.heatingControlPercent > 0);
}

void test_setpoint_shift_is_clamped_to_the_configured_limit()
{
    Settings settings;
    settings.maxSetpointShiftK = 3.0f;

    Inputs inputs;
    inputs.setpointShiftK = 10.0f;  // a UI with no limits of its own

    Core core;
    const Outputs out = settle(core, settings, inputs, roomAir(21.0f, 45.0f), 3);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, out.setpointShiftFeedbackK);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, settings.comfortHeatingSetpointC + 3.0f, out.activeSetpointC);
}

void test_an_unlinked_outside_temperature_never_offers_free_cooling()
{
    // outsideTemperatureC defaults to 0.0f. If validity were not checked, every
    // device with that object unlinked would report free cooling available in
    // any room warmer than 1 °C — in January, all of them.
    Settings settings;
    settings.coolingEnabled = true;

    Inputs unlinked;  // outsideTemperatureValid == false
    Core core;
    const Outputs out = settle(core, settings, unlinked, roomAir(26.0f, 45.0f), 5);
    TEST_ASSERT_FALSE(out.freeCoolingAvailable);

    Inputs linked;
    linked.outsideTemperatureC = 15.0f;
    linked.outsideTemperatureValid = true;
    Core core2;
    const Outputs out2 = settle(core2, settings, linked, roomAir(26.0f, 45.0f), 5);
    TEST_ASSERT_TRUE(out2.freeCoolingAvailable);
}

void test_the_measurement_offset_reaches_the_control_loop_not_just_the_report()
{
    // The self-heating correction exists because the board reads high. If it
    // were applied only where measurements are published, the controller would
    // keep working from the uncorrected value — which is the exact reading the
    // offset exists to fix.
    Settings settings;
    settings.roomTemperatureOffsetK = -2.0f;

    Core corrected;
    const Outputs correctedOut = settle(corrected, settings, Inputs{}, roomAir(21.0f, 45.0f), 120);

    Core uncorrected;
    const Outputs uncorrectedOut =
        settle(uncorrected, Settings{}, Inputs{}, roomAir(21.0f, 45.0f), 120);

    // Correcting 21 °C down to 19 °C must make the controller call for more heat.
    TEST_ASSERT_TRUE(correctedOut.heatingControlPercent > uncorrectedOut.heatingControlPercent);
}

// ---------------------------------------------------------------------------
// Properties every protocol adapter depends on
// ---------------------------------------------------------------------------

void test_the_tick_is_deterministic()
{
    // Two cores fed identical histories must agree exactly. Adapters publish
    // Outputs verbatim, so any hidden state here would let two field buses
    // disagree about what the device is doing.
    const Settings settings;
    const Inputs inputs;
    const sensor_data_t data = roomAir(18.5f, 52.0f);

    Core a;
    Core b;
    for (int i = 0; i < 200; ++i) {
        const Outputs oa = a.tick(settings, inputs, data, 1.0f);
        const Outputs ob = b.tick(settings, inputs, data, 1.0f);
        TEST_ASSERT_EQUAL_UINT8(oa.heatingControlPercent, ob.heatingControlPercent);
        TEST_ASSERT_FLOAT_WITHIN(0.0001f, oa.activeSetpointC, ob.activeSetpointC);
    }
}

void test_heating_and_cooling_are_never_both_active()
{
    Settings settings;
    settings.heatingEnabled = true;
    settings.coolingEnabled = true;

    // Sweep the whole band either side of the setpoint; at no point may both
    // sequences drive an output. An actuator pair told to heat and cool at once
    // is a fault an installer will blame on the room controller, correctly.
    for (float t = 15.0f; t <= 30.0f; t += 0.25f) {
        Core core;
        const Outputs out = settle(core, settings, Inputs{}, roomAir(t, 45.0f), 60);
        if (out.heatingControlPercent > 0 && out.coolingControlPercent > 0) {
            char message[128];
            std::snprintf(message, sizeof(message),
                          "both sequences active at %.2f C: heat %u %%, cool %u %%", t,
                          out.heatingControlPercent, out.coolingControlPercent);
            TEST_FAIL_MESSAGE(message);
        }
    }
}

void test_the_controller_mode_reported_is_one_an_adapter_can_write_back()
{
    // control_state_write(CONTROL_CMD_CONTROLLER_MODE) takes the same compact
    // code points activeControllerMode reports. This asserts the range they
    // both have to live in; a value outside it means an adapter doing
    // read-modify-write would silently change the mode.
    Settings settings;
    settings.coolingEnabled = true;

    for (float t = 15.0f; t <= 30.0f; t += 1.0f) {
        Core core;
        const Outputs out = settle(core, settings, Inputs{}, roomAir(t, 45.0f), 60);
        const uint8_t mode = static_cast<uint8_t>(out.activeControllerMode);
        TEST_ASSERT_TRUE(mode <= 3);
    }
}

void test_disabling_the_controller_stops_the_outputs()
{
    Inputs inputs;
    inputs.controllerOnOff = false;

    Core core;
    const Outputs out = settle(core, Settings{}, inputs, roomAir(15.0f, 45.0f), 120);
    TEST_ASSERT_EQUAL_UINT8(0, out.heatingControlPercent);
    TEST_ASSERT_FALSE(out.heatingRequest);
}

void test_a_long_delayed_tick_does_not_step_the_output()
{
    // The control task measures the interval actually elapsed rather than
    // assuming 1 s, because a flash write can hold it off. Integrating a 10 s
    // gap as if it were 1 s would be wrong, but integrating it as 10 s must
    // still not produce a discontinuity larger than 10 ordinary ticks.
    const Settings settings;
    const Inputs inputs;
    const sensor_data_t data = roomAir(18.0f, 45.0f);

    Core steady;
    for (int i = 0; i < 30; ++i) {
        (void)steady.tick(settings, inputs, data, 1.0f);
    }
    const Outputs beforeGap = steady.tick(settings, inputs, data, 1.0f);
    const Outputs afterGap = steady.tick(settings, inputs, data, 10.0f);

    Core reference;
    for (int i = 0; i < 31; ++i) {
        (void)reference.tick(settings, inputs, data, 1.0f);
    }
    Outputs referenceOut{};
    for (int i = 0; i < 10; ++i) {
        referenceOut = reference.tick(settings, inputs, data, 1.0f);
    }

    (void)beforeGap;
    // One 10 s tick and ten 1 s ticks must land in the same place, within the
    // rounding of a percentage.
    const int drift = static_cast<int>(referenceOut.heatingControlPercent)
                      - static_cast<int>(afterGap.heatingControlPercent);
    TEST_ASSERT_TRUE(drift >= -1 && drift <= 1);
}

}  // namespace

extern "C" void setUp() {}
extern "C" void tearDown() {}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_factory_defaults_produce_a_working_controller);
    RUN_TEST(test_a_cold_room_calls_for_heat_and_a_warm_one_does_not);
    RUN_TEST(test_losing_every_room_reading_is_a_device_fault);
    RUN_TEST(test_a_fallback_source_is_not_a_device_fault);
    RUN_TEST(test_a_board_with_no_floor_probe_still_heats);
    RUN_TEST(test_a_floor_probe_that_fails_after_reporting_is_a_real_fault);

    RUN_TEST(test_an_open_window_blocks_heating);
    RUN_TEST(test_a_detected_window_substitutes_only_while_nothing_is_linked);
    RUN_TEST(test_setpoint_shift_is_clamped_to_the_configured_limit);
    RUN_TEST(test_an_unlinked_outside_temperature_never_offers_free_cooling);
    RUN_TEST(test_the_measurement_offset_reaches_the_control_loop_not_just_the_report);

    RUN_TEST(test_the_tick_is_deterministic);
    RUN_TEST(test_heating_and_cooling_are_never_both_active);
    RUN_TEST(test_the_controller_mode_reported_is_one_an_adapter_can_write_back);
    RUN_TEST(test_disabling_the_controller_stops_the_outputs);
    RUN_TEST(test_a_long_delayed_tick_does_not_step_the_output);

    return UNITY_END();
}
