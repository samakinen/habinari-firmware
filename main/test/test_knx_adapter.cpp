// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * Host tests for the KNX adapter's platform-free logic.
 *
 * The adapter itself needs FreeRTOS, a bus and the stack's runtime. What it
 * decides does not:
 *
 *   - knx_parameter_table.hpp: where each ETS parameter lands. Driven here with
 *     the stack's own ParameterState over the real product schema, so the
 *     defaults are the ones ETS exports.
 *   - knx_group_writes.hpp: which control command an inbound group write is,
 *     how its bus value maps, and the DPT 9 7FFFh "invalid data" code.
 *   - knx_publish_schedule.hpp: when the service loop publishes.
 */

#include "control_command.hpp"
#include "control_core.hpp"
#include "knx_dpt9.hpp"
#include "knx_group_writes.hpp"
#include "knx_parameter_table.hpp"
#include "knx_product.hpp"
#include "knx_publish_schedule.hpp"
#include "sensor_fusion_service.h"

#include "knx/product/commissioned_product.hpp"

#include <unity.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <set>
#include <tuple>
#include <type_traits>

using namespace habinari_knx;
namespace control = habinari::control;
namespace hvac = habinari::hvac;

namespace {

// ---------------------------------------------------------------------------
// ETS parameter table
// ---------------------------------------------------------------------------

// What the table writes to: the three ServiceState members it touches, and
// nothing that needs FreeRTOS.
struct ParameterTarget {
    control::Settings hvac{};
    sensor_fusion_config_t fusion{};
    bool fusionConfigDirty{false};
};

using ProductDefinition = std::remove_cvref_t<decltype(kHabinariProduct)>;
using ParameterSchema = ProductDefinition::parameter_schema_type;
using Parameters = knx::product::detail::ParameterState<ParameterSchema>;

// A target with the compiled-in defaults, built in zeroed storage so that two
// copies of it compare equal byte for byte, padding included.
void makeFactoryTarget(ParameterTarget &target)
{
    std::memset(static_cast<void *>(&target), 0, sizeof(target));
    target.hvac = control::Settings{};
    sensor_fusion_default_config(&target.fusion);
    target.fusionConfigDirty = false;
}

void copyTarget(ParameterTarget &to, const ParameterTarget &from)
{
    std::memcpy(static_cast<void *>(&to), static_cast<const void *>(&from), sizeof(to));
}

std::set<size_t> changedBytes(const ParameterTarget &a, const ParameterTarget &b)
{
    std::set<size_t> changed;
    const auto *pa = reinterpret_cast<const unsigned char *>(&a);
    const auto *pb = reinterpret_cast<const unsigned char *>(&b);
    for (size_t i = 0; i < sizeof(ParameterTarget); ++i) {
        if (pa[i] != pb[i]) {
            changed.insert(i);
        }
    }
    return changed;
}

// Something other than @p value that every setter must notice: a different
// number for a measurement, the other side of zero for a flag or an enum.
template <typename T>
T differentFrom(T value)
{
    if constexpr (std::is_same_v<T, knx::product::Dpt9Float>) {
        return T{value.value + 1.5f};
    } else {
        return value == 0 ? T{1} : T{0};
    }
}

void test_ets_defaults_load_as_the_factory_settings()
{
    // control_defaults.hpp feeds both the ETS parameter defaults and the
    // Settings initialisers. Loading an uncommissioned device's parameters
    // through the table must therefore change nothing — which also catches a
    // row writing the right value to the wrong field, whenever the two
    // fields' defaults differ.
    const Parameters params(kHabinariProduct.parameterSchema);
    ParameterTarget factory;
    makeFactoryTarget(factory);
    ParameterTarget loaded;
    copyTarget(loaded, factory);

    loadParameters(params, loaded);

    TEST_ASSERT_EQUAL_MEMORY(&factory.hvac, &loaded.hvac, sizeof(control::Settings));
    // Fusion rows mark the config for pushing down to the acquisition side.
    TEST_ASSERT_TRUE(loaded.fusionConfigDirty);
}

void test_every_row_writes_a_field_of_its_own()
{
    // Each parameter, changed on its own, must change the target, and no two
    // parameters may land on the same bytes. The dirty flag is the one byte the
    // fusion rows deliberately share.
    const Parameters defaults(kHabinariProduct.parameterSchema);
    ParameterTarget baseline;
    makeFactoryTarget(baseline);
    loadParameters(defaults, baseline);
    baseline.fusionConfigDirty = false;

    const size_t dirtyOffset = offsetof(ParameterTarget, fusionConfigDirty);
    std::set<size_t> claimed;
    size_t rows = 0;

    std::apply(
        [&](const auto &...row) {
            (
                [&]<HabinariParameter Id, typename Apply>(const ParameterRow<Id, Apply> &) {
                    ++rows;
                    ParameterTarget changed;
                    copyTarget(changed, baseline);
                    Apply{}(changed, differentFrom(defaults.template get<Id>()));

                    std::set<size_t> bytes = changedBytes(baseline, changed);
                    bytes.erase(dirtyOffset);
                    char message[64];
                    std::snprintf(message, sizeof(message), "parameter id %u",
                                  static_cast<unsigned>(Id));
                    std::strncat(message, bytes.empty() ? ": writes nothing" : "", 20);
                    if (bytes.empty()) {
                        TEST_FAIL_MESSAGE(message);
                    }
                    for (const size_t byte : bytes) {
                        if (!claimed.insert(byte).second) {
                            std::strncat(message, ": shares a field", 20);
                            TEST_FAIL_MESSAGE(message);
                        }
                    }
                }(row),
                ...);
        },
        kParameterTable);

    // The compile-time check in the header already pins this; restated so the
    // count is visible in the test output if it ever changes.
    TEST_ASSERT_EQUAL_UINT32(ParameterSchema::kParameterCount - 1, rows);
}

void test_parameters_convert_to_the_setting_types()
{
    Parameters params(kHabinariProduct.parameterSchema);
    (void)params.apply<HabinariParameter::HeatingTiSeconds>(uint16_t{1234});
    (void)params.apply<HabinariParameter::Co2Cov>(uint16_t{75});
    (void)params.apply<HabinariParameter::ComfortHeatingSetpoint>(knx::product::Dpt9Float{22.5f});
    (void)params.apply<HabinariParameter::DefaultHvacOperatingMode>(
        static_cast<uint8_t>(hvac::OperatingPreset::Economy));
    (void)params.apply<HabinariParameter::HeatCoolChangeoverPolarity>(uint8_t{1});
    (void)params.apply<HabinariParameter::CoolingEnabled>(uint8_t{0});
    (void)params.apply<HabinariParameter::MeasurementHeartbeatSeconds>(uint16_t{600});
    (void)params.apply<HabinariParameter::SensorFilterSeconds>(uint16_t{45});
    (void)params.apply<HabinariParameter::WindowDetectEnabled>(uint8_t{0});

    ParameterTarget target;
    makeFactoryTarget(target);
    loadParameters(params, target);

    TEST_ASSERT_EQUAL_FLOAT(1234.0f, target.hvac.heatingTiSeconds);
    TEST_ASSERT_EQUAL_FLOAT(75.0f, target.hvac.co2CovPpm);
    TEST_ASSERT_EQUAL_FLOAT(22.5f, target.hvac.comfortHeatingSetpointC);
    TEST_ASSERT_TRUE(target.hvac.defaultHvacOperatingMode == hvac::OperatingPreset::Economy);
    TEST_ASSERT_TRUE(target.hvac.heatCoolChangeoverPolarityInverted);
    TEST_ASSERT_FALSE(target.hvac.coolingEnabled);
    TEST_ASSERT_EQUAL_UINT32(600, target.hvac.heartbeatSeconds);
    TEST_ASSERT_EQUAL_FLOAT(45.0f, target.fusion.filter_tau_seconds);
    TEST_ASSERT_FALSE(target.fusion.window_detect_enabled);
    TEST_ASSERT_TRUE(target.fusionConfigDirty);
}

void test_a_settings_only_row_leaves_the_fusion_config_clean()
{
    ParameterTarget target;
    makeFactoryTarget(target);
    std::get<0>(kParameterTable).apply(target, uint16_t{30});  // MeasurementHeartbeatSeconds
    TEST_ASSERT_EQUAL_UINT32(30, target.hvac.heartbeatSeconds);
    TEST_ASSERT_FALSE(target.fusionConfigDirty);
}

// ---------------------------------------------------------------------------
// Inbound group writes
// ---------------------------------------------------------------------------

void assertApply(const GroupCommand &command, control_command_t expected, float value)
{
    TEST_ASSERT_TRUE(command.kind == GroupCommand::Kind::Apply);
    TEST_ASSERT_EQUAL_INT(expected, command.command);
    TEST_ASSERT_EQUAL_FLOAT(value, command.value);
}

void test_each_command_port_names_its_command()
{
    assertApply(groupCommand<HabinariPort::ControllerOnOff>(true), CONTROL_CMD_CONTROLLER_ON_OFF, 1.0f);
    assertApply(groupCommand<HabinariPort::ControllerOnOff>(false), CONTROL_CMD_CONTROLLER_ON_OFF, 0.0f);
    assertApply(groupCommand<HabinariPort::WindowStatus>(true), CONTROL_CMD_WINDOW_STATUS, 1.0f);
    assertApply(groupCommand<HabinariPort::PresenceStatus>(false), CONTROL_CMD_PRESENCE, 0.0f);
    assertApply(groupCommand<HabinariPort::SetpointBase>(21.5f), CONTROL_CMD_SETPOINT_BASE, 21.5f);
    assertApply(groupCommand<HabinariPort::SetpointShift>(-1.5f), CONTROL_CMD_SETPOINT_SHIFT, -1.5f);
    assertApply(groupCommand<HabinariPort::Co2Setpoint>(900.0f), CONTROL_CMD_CO2_SETPOINT, 900.0f);
    assertApply(groupCommand<HabinariPort::VentilationMode>(uint8_t{2}), CONTROL_CMD_VENTILATION_MODE,
                2.0f);
    assertApply(groupCommand<HabinariPort::HvacMode>(
                    static_cast<knx::application::Dpt20Mode>(hvac::OperatingPreset::Standby)),
                CONTROL_CMD_HVAC_MODE, static_cast<float>(hvac::OperatingPreset::Standby));
}

void test_hvac_mode_passes_the_raw_byte_to_the_gate()
{
    // An undefined DPT 20.102 code point is not cast in here: it reaches
    // control_state_write() as it came, and the gate refuses it.
    const GroupCommand command =
        groupCommand<HabinariPort::HvacMode>(static_cast<knx::application::Dpt20Mode>(0x55));
    assertApply(command, CONTROL_CMD_HVAC_MODE, 85.0f);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, control::validateCommand(command.command, command.value));
}

void test_controller_mode_maps_dpt_20_105_to_the_compact_enum()
{
    const auto mode = [](uint8_t bus) {
        const GroupCommand command = groupCommand<HabinariPort::ContrMode>(bus);
        TEST_ASSERT_TRUE(command.kind == GroupCommand::Kind::Apply);
        TEST_ASSERT_EQUAL_INT(CONTROL_CMD_CONTROLLER_MODE, command.command);
        return static_cast<hvac::ControllerMode>(static_cast<uint8_t>(command.value));
    };
    TEST_ASSERT_TRUE(mode(0) == hvac::ControllerMode::Auto);
    TEST_ASSERT_TRUE(mode(1) == hvac::ControllerMode::Heat);
    TEST_ASSERT_TRUE(mode(3) == hvac::ControllerMode::Cool);
    TEST_ASSERT_TRUE(mode(6) == hvac::ControllerMode::Off);
    // Codes the controller has no mode for fall back to Auto rather than
    // reaching the gate as a refusal.
    TEST_ASSERT_TRUE(mode(9) == hvac::ControllerMode::Auto);
    TEST_ASSERT_TRUE(mode(0xFF) == hvac::ControllerMode::Auto);
}

void test_dpt9_invalid_data_is_never_a_command()
{
    const float invalid = kDpt9InvalidDecoded;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    TEST_ASSERT_TRUE(groupCommand<HabinariPort::SetpointBase>(invalid).kind ==
                     GroupCommand::Kind::InvalidData);
    TEST_ASSERT_TRUE(groupCommand<HabinariPort::SetpointShift>(invalid).kind ==
                     GroupCommand::Kind::InvalidData);
    TEST_ASSERT_TRUE(groupCommand<HabinariPort::Co2Setpoint>(invalid).kind ==
                     GroupCommand::Kind::InvalidData);
    TEST_ASSERT_TRUE(groupCommand<HabinariPort::Co2Setpoint>(nan).kind ==
                     GroupCommand::Kind::InvalidData);

    // The top of the valid range is a value, not the invalid code: it goes to
    // the gate, which refuses it as out of range — a different log line, and
    // the right one.
    const GroupCommand top = groupCommand<HabinariPort::Co2Setpoint>(kDpt9MaxValid);
    assertApply(top, CONTROL_CMD_CO2_SETPOINT, kDpt9MaxValid);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, control::validateCommand(top.command, top.value));
}

void test_alarm_acknowledge_acts_only_on_one()
{
    assertApply(groupCommand<HabinariPort::AlarmAcknowledge>(true), CONTROL_CMD_ACKNOWLEDGE_ALARMS,
                1.0f);
    TEST_ASSERT_TRUE(groupCommand<HabinariPort::AlarmAcknowledge>(false).kind ==
                     GroupCommand::Kind::Ignore);
}

void test_a_neighbour_reading_is_stored_and_marked_valid()
{
    control::Inputs in{};
    TEST_ASSERT_TRUE(storeNeighbourReading<HabinariPort::OutsideTemperature>(in, -7.5f));
    TEST_ASSERT_EQUAL_FLOAT(-7.5f, in.outsideTemperatureC);
    TEST_ASSERT_TRUE(in.outsideTemperatureValid);

    TEST_ASSERT_TRUE(storeNeighbourReading<HabinariPort::FlowTemperature>(in, 32.0f));
    TEST_ASSERT_EQUAL_FLOAT(32.0f, in.flowTemperatureC);
    TEST_ASSERT_TRUE(in.flowTemperatureValid);

    // Relative humidity is clamped to what it can physically be.
    TEST_ASSERT_TRUE(storeNeighbourReading<HabinariPort::OutsideHumidity>(in, 104.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, in.outsideHumidityPct);
    TEST_ASSERT_TRUE(storeNeighbourReading<HabinariPort::OutsideHumidity>(in, -2.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, in.outsideHumidityPct);
    TEST_ASSERT_TRUE(in.outsideHumidityValid);
}

void test_a_neighbour_reporting_invalid_data_stops_being_used()
{
    control::Inputs in{};
    (void)storeNeighbourReading<HabinariPort::OutsideTemperature>(in, 4.0f);
    (void)storeNeighbourReading<HabinariPort::OutsideHumidity>(in, 60.0f);
    (void)storeNeighbourReading<HabinariPort::FlowTemperature>(in, 30.0f);

    TEST_ASSERT_FALSE(storeNeighbourReading<HabinariPort::OutsideTemperature>(in, kDpt9InvalidDecoded));
    TEST_ASSERT_FALSE(storeNeighbourReading<HabinariPort::OutsideHumidity>(in, kDpt9InvalidDecoded));
    TEST_ASSERT_FALSE(storeNeighbourReading<HabinariPort::FlowTemperature>(in, kDpt9InvalidDecoded));

    // Not overwritten with 670 760, and no longer trusted.
    TEST_ASSERT_EQUAL_FLOAT(4.0f, in.outsideTemperatureC);
    TEST_ASSERT_FALSE(in.outsideTemperatureValid);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, in.outsideHumidityPct);
    TEST_ASSERT_FALSE(in.outsideHumidityValid);
    TEST_ASSERT_EQUAL_FLOAT(30.0f, in.flowTemperatureC);
    TEST_ASSERT_FALSE(in.flowTemperatureValid);

    // A valid reading afterwards is used again.
    TEST_ASSERT_TRUE(storeNeighbourReading<HabinariPort::OutsideTemperature>(in, 5.0f));
    TEST_ASSERT_TRUE(in.outsideTemperatureValid);
}

// ---------------------------------------------------------------------------
// Publish schedule
// ---------------------------------------------------------------------------

void test_nothing_is_published_outside_operational()
{
    PublishSchedule schedule(0, false);
    for (uint32_t tick = 0; tick < 5; ++tick) {
        schedule.controlTick();
        const auto pass = schedule.begin(tick, false);
        TEST_ASSERT_FALSE(pass.publish);
        TEST_ASSERT_FALSE(pass.announceHold);
        TEST_ASSERT_FALSE(pass.reportUnlinkedPorts);
        schedule.end(false);
    }
}

void test_the_startup_gate_holds_then_publishes_once_per_tick()
{
    PublishSchedule schedule(100, true);

    // Held: announced once, nothing published, however many ticks arrive.
    auto pass = schedule.begin(10, true);
    TEST_ASSERT_TRUE(pass.announceHold);
    TEST_ASSERT_FALSE(pass.publish);
    schedule.end(true);
    schedule.controlTick();
    pass = schedule.begin(99, true);
    TEST_ASSERT_FALSE(pass.announceHold);
    TEST_ASSERT_FALSE(pass.publish);
    schedule.end(true);

    // Released: the first pass publishes what accumulated, once.
    pass = schedule.begin(100, true);
    TEST_ASSERT_TRUE(pass.publish);
    schedule.end(true);
    pass = schedule.begin(101, true);
    TEST_ASSERT_FALSE(pass.publish);
    schedule.end(true);

    // Then once per control tick, not once per pass.
    schedule.controlTick();
    TEST_ASSERT_TRUE(schedule.begin(102, true).publish);
    schedule.end(true);
    TEST_ASSERT_FALSE(schedule.begin(103, true).publish);
    schedule.end(true);
}

void test_the_first_open_pass_publishes_without_waiting_for_a_tick()
{
    PublishSchedule schedule(0, true);
    TEST_ASSERT_TRUE(schedule.begin(0, true).publish);
}

void test_the_startup_gate_survives_tick_wraparound()
{
    const uint32_t release = 5;            // wrapped past zero
    const uint32_t before = 0xFFFFFFF0u;   // 21 ticks earlier
    PublishSchedule schedule(release, true);
    TEST_ASSERT_FALSE(schedule.begin(before, true).publish);
    schedule.end(true);
    TEST_ASSERT_TRUE(schedule.begin(release, true).publish);

    TEST_ASSERT_FALSE(PublishSchedule::released(0xFFFFFFFFu, 0));
    TEST_ASSERT_TRUE(PublishSchedule::released(0, 0xFFFFFFFFu));
}

void test_unlinked_ports_are_reported_on_entering_operational()
{
    // A device that starts Operational made no transition.
    PublishSchedule commissioned(0, true);
    TEST_ASSERT_FALSE(commissioned.begin(0, true).reportUnlinkedPorts);
    commissioned.end(true);

    // Leaving and re-entering (a download in programming mode) does.
    TEST_ASSERT_FALSE(commissioned.begin(1, false).reportUnlinkedPorts);
    commissioned.end(false);
    TEST_ASSERT_TRUE(commissioned.begin(2, true).reportUnlinkedPorts);
    commissioned.end(true);
    TEST_ASSERT_FALSE(commissioned.begin(3, true).reportUnlinkedPorts);
    commissioned.end(true);

    // A fresh device is reported once ETS gives it an address.
    PublishSchedule fresh(0, false);
    TEST_ASSERT_FALSE(fresh.begin(0, false).reportUnlinkedPorts);
    fresh.end(false);
    TEST_ASSERT_TRUE(fresh.begin(1, true).reportUnlinkedPorts);
}

void test_the_transition_is_measured_from_the_end_of_the_pass()
{
    // The loop closes each pass with the lifecycle as it stands after the
    // programming-mode toggle. A toggle that ends programming mode within a
    // pass therefore counts as Operational already, and raises no report.
    PublishSchedule schedule(0, false);
    TEST_ASSERT_FALSE(schedule.begin(0, false).reportUnlinkedPorts);
    schedule.end(true);
    TEST_ASSERT_FALSE(schedule.begin(1, true).reportUnlinkedPorts);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_ets_defaults_load_as_the_factory_settings);
    RUN_TEST(test_every_row_writes_a_field_of_its_own);
    RUN_TEST(test_parameters_convert_to_the_setting_types);
    RUN_TEST(test_a_settings_only_row_leaves_the_fusion_config_clean);
    RUN_TEST(test_each_command_port_names_its_command);
    RUN_TEST(test_hvac_mode_passes_the_raw_byte_to_the_gate);
    RUN_TEST(test_controller_mode_maps_dpt_20_105_to_the_compact_enum);
    RUN_TEST(test_dpt9_invalid_data_is_never_a_command);
    RUN_TEST(test_alarm_acknowledge_acts_only_on_one);
    RUN_TEST(test_a_neighbour_reading_is_stored_and_marked_valid);
    RUN_TEST(test_a_neighbour_reporting_invalid_data_stops_being_used);
    RUN_TEST(test_nothing_is_published_outside_operational);
    RUN_TEST(test_the_startup_gate_holds_then_publishes_once_per_tick);
    RUN_TEST(test_the_first_open_pass_publishes_without_waiting_for_a_tick);
    RUN_TEST(test_the_startup_gate_survives_tick_wraparound);
    RUN_TEST(test_unlinked_ports_are_reported_on_entering_operational);
    RUN_TEST(test_the_transition_is_measured_from_the_end_of_the_pass);
    return UNITY_END();
}
