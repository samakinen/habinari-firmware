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
 *   - knx_publish_schedule.hpp: when the service loop publishes, and when it
 *     reports the objects ETS left unlinked.
 *   - knx_port_values.hpp: what each transmitting port carries — and that a
 *     read response answers exactly what publication sends.
 */

#include "control_command.hpp"
#include "control_core.hpp"
#include "knx_dpt9.hpp"
#include "knx_group_writes.hpp"
#include "knx_parameter_table.hpp"
#include "knx_port_values.hpp"
#include "knx_product.hpp"
#include "knx_publish_schedule.hpp"
#include "sensor_fusion_service.h"

#include "knx/application/dpt.hpp"
#include "knx/product/commissioned_product.hpp"

#include <unity.h>

#include <array>
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

// end() is gone: each pass takes the lifecycle read at its top, before the loop
// acts on the button. These helpers drive one pass the way the loop does.
PublishSchedule::Pass pass(PublishSchedule &schedule, uint32_t tick, bool operational)
{
    return schedule.begin(tick, operational);
}

void test_nothing_is_published_outside_operational()
{
    PublishSchedule schedule(0);
    for (uint32_t tick = 0; tick < 5; ++tick) {
        schedule.controlTick();
        const auto p = pass(schedule, tick, false);
        TEST_ASSERT_FALSE(p.publish);
        TEST_ASSERT_FALSE(p.announceHold);
        TEST_ASSERT_FALSE(p.reportUnlinkedPorts);
    }
}

void test_the_startup_gate_holds_then_publishes_once_per_tick()
{
    PublishSchedule schedule(100);

    // Held: announced once, nothing published, however many ticks arrive.
    auto p = pass(schedule, 10, true);
    TEST_ASSERT_TRUE(p.announceHold);
    TEST_ASSERT_FALSE(p.publish);
    schedule.controlTick();
    p = pass(schedule, 99, true);
    TEST_ASSERT_FALSE(p.announceHold);
    TEST_ASSERT_FALSE(p.publish);

    // Released: the first pass publishes what accumulated, once.
    TEST_ASSERT_TRUE(pass(schedule, 100, true).publish);
    TEST_ASSERT_FALSE(pass(schedule, 101, true).publish);

    // Then once per control tick, not once per pass.
    schedule.controlTick();
    TEST_ASSERT_TRUE(pass(schedule, 102, true).publish);
    TEST_ASSERT_FALSE(pass(schedule, 103, true).publish);
}

void test_the_first_open_pass_publishes_without_waiting_for_a_tick()
{
    PublishSchedule schedule(0);
    TEST_ASSERT_TRUE(pass(schedule, 0, true).publish);
}

void test_the_startup_gate_survives_tick_wraparound()
{
    const uint32_t release = 5;           // wrapped past zero
    const uint32_t before = 0xFFFFFFF0u;  // 21 ticks earlier
    PublishSchedule schedule(release);
    TEST_ASSERT_FALSE(pass(schedule, before, true).publish);
    TEST_ASSERT_TRUE(pass(schedule, release, true).publish);

    TEST_ASSERT_FALSE(PublishSchedule::released(0xFFFFFFFFu, 0));
    TEST_ASSERT_TRUE(PublishSchedule::released(0, 0xFFFFFFFFu));
}

void test_a_commissioned_device_reports_unlinked_ports_on_every_boot()
{
    // Nothing has been observed before the first pass, so a device that
    // boots Operational has just become Operational — once.
    PublishSchedule schedule(0);
    TEST_ASSERT_TRUE(pass(schedule, 0, true).reportUnlinkedPorts);
    TEST_ASSERT_FALSE(pass(schedule, 1, true).reportUnlinkedPorts);
    TEST_ASSERT_FALSE(pass(schedule, 2, true).reportUnlinkedPorts);
}

void test_a_fresh_device_reports_once_ets_gives_it_an_address()
{
    PublishSchedule schedule(0);
    TEST_ASSERT_FALSE(pass(schedule, 0, false).reportUnlinkedPorts);
    TEST_ASSERT_FALSE(pass(schedule, 1, false).reportUnlinkedPorts);
    TEST_ASSERT_TRUE(pass(schedule, 2, true).reportUnlinkedPorts);
    TEST_ASSERT_FALSE(pass(schedule, 3, true).reportUnlinkedPorts);
}

void test_leaving_programming_mode_from_ets_reports()
{
    // ETS ends programming mode over the bus, inside the stack's loop(): the
    // next pass's top-of-pass reading already sees Operational.
    PublishSchedule schedule(0);
    (void)pass(schedule, 0, true);
    TEST_ASSERT_FALSE(pass(schedule, 1, false).reportUnlinkedPorts);  // ETS entered it
    TEST_ASSERT_FALSE(pass(schedule, 2, false).reportUnlinkedPorts);  // download
    TEST_ASSERT_TRUE(pass(schedule, 3, true).reportUnlinkedPorts);    // ETS left it
}

void test_leaving_programming_mode_with_the_button_reports()
{
    // The button is acted on after the top-of-pass reading. The pass that
    // toggles programming mode off still read Commissioning; the next one
    // reads Operational and sees the transition. (The old schedule closed each
    // pass with a reading taken after the toggle, and missed this one.)
    PublishSchedule schedule(0);
    (void)pass(schedule, 0, true);
    // Button: on. This pass read Operational; programming mode starts after it.
    TEST_ASSERT_FALSE(pass(schedule, 1, true).reportUnlinkedPorts);
    TEST_ASSERT_FALSE(pass(schedule, 2, false).reportUnlinkedPorts);
    // Button: off. This pass read Commissioning, toggles, and reports nothing.
    TEST_ASSERT_FALSE(pass(schedule, 3, false).reportUnlinkedPorts);
    // The next pass reads Operational: reported.
    TEST_ASSERT_TRUE(pass(schedule, 4, true).reportUnlinkedPorts);
}

// ---------------------------------------------------------------------------
// Port values: read responses answer what publication sends
// ---------------------------------------------------------------------------

sensor_value_t reading(float value, bool valid)
{
    sensor_value_t r{};
    r.value = value;
    r.valid = valid;
    return r;
}

// A state in which every value is distinct and in range, so a port answering
// from the wrong field shows.
PortState healthyState()
{
    PortState s;
    auto &d = s.sensorData;
    d.temperature = reading(21.3f, true);
    d.humidity = reading(48.0f, true);
    d.co2 = reading(812.0f, true);
    d.pressure = reading(98650.0f, true);
    d.iaq = reading(57.6f, true);
    d.air_quality_accuracy = 3;
    d.co2_equivalent = reading(640.0f, true);
    d.voc_equivalent = reading(0.9f, true);
    d.probe_temperature = reading(24.1f, true);
    d.probe_humidity = reading(61.0f, true);
    d.trends.temperature.valid = true;
    d.trends.temperature.per_minute = 0.02f;
    d.events.fire_alarm = false;
    d.events.fire_pre_alarm = true;
    d.events.occupancy_detected = true;
    d.events.estimated_occupants = 2;
    d.events.window_open_detected = false;

    s.settings.roomTemperatureOffsetK = -0.4f;
    s.settings.roomHumidityOffsetPct = 3.0f;
    s.settings.floorTemperatureOffsetK = 0.6f;
    s.settings.comfortHeatingSetpointC = 21.5f;
    s.settings.ventilationSetpointPpm = 900.0f;

    auto &o = s.out;
    o.seaLevelPressurePa = 101200.0f;
    o.roomDewPointC = 9.8f;
    o.roomAbsoluteHumidityGm3 = 8.9f;
    o.floorAbsoluteHumidityGm3 = 13.2f;
    o.dewPointMarginK = 4.4f;
    o.setpointShiftFeedbackK = 0.5f;
    o.activeSetpointC = 22.0f;
    o.heatingSetpointC = 22.0f;
    o.coolingSetpointC = 24.0f;
    o.activePreset = hvac::OperatingPreset::Comfort;
    o.activeControllerMode = hvac::ControllerMode::Heat;
    o.floorMoistureAlarm = true;
    o.dewPointAlarm = true;
    o.heatingRequest = true;
    o.enableHeat = true;
    s.ventilationMode = hvac::VentilationMode::Boost;
    s.controllerOnOff = true;
    return s;
}

// The same room with every measurement gone: sensors failed, trend window
// empty. Publication sends none of them.
PortState failedSensorState()
{
    PortState s = healthyState();
    auto &d = s.sensorData;
    d.temperature.valid = false;
    d.humidity.valid = false;
    d.co2.valid = false;
    d.pressure.valid = false;
    d.iaq.valid = false;
    d.co2_equivalent.valid = false;
    d.voc_equivalent.valid = false;
    d.probe_temperature.valid = false;
    d.probe_humidity.valid = false;
    d.trends.temperature.valid = false;
    return s;
}

// The value publication would hand the stack for each port, keyed by port.
// Mirrors the firmware's publishValues(): every transmitting port, offered
// only when it has a value.
template <typename T>
bool sameValue(const T &a, const T &b)
{
    if constexpr (std::is_same_v<T, float>) {
        return std::memcmp(&a, &b, sizeof(float)) == 0;
    } else {
        return a == b;
    }
}

uint16_t dpt9Raw(float value)
{
    std::array<uint8_t, 2> bytes{};
    const auto encoded = knx::application::Dpt9::encode(value, bytes);
    TEST_ASSERT_TRUE(encoded.isOk());
    return static_cast<uint16_t>((bytes[0] << 8) | bytes[1]);
}

void checkReadsMatchPublication(const PortState &state, size_t expectedUnpublished)
{
    const PortValues values(state);
    size_t ports = 0;
    size_t unpublished = 0;
    forEachTransmitPort([&]<HabinariPort Port>(const char *label) {
        ++ports;
        const auto published = values.template get<Port>();
        const auto read = readResponse<Port>(values);
        using T = PortValue<Port>;
        if (published.has_value()) {
            if (!sameValue<T>(*published, read)) {
                TEST_FAIL_MESSAGE(label);
            }
            return;
        }
        // Nothing published: the read carries the port's "no reading" form.
        ++unpublished;
        if constexpr (std::is_same_v<T, float>) {
            if (dpt9Raw(read) != 0x7FFF) {
                TEST_FAIL_MESSAGE(label);
            }
        } else if constexpr (std::is_integral_v<T>) {
            if (read != T{0}) {
                TEST_FAIL_MESSAGE(label);
            }
        } else {
            TEST_FAIL_MESSAGE(label);  // only measurements may be without a value
        }
    });
    TEST_ASSERT_EQUAL_UINT32(std::size(kTransmitPorts), ports);
    TEST_ASSERT_EQUAL_UINT32(expectedUnpublished, unpublished);
}

void test_reads_answer_what_publication_sends()
{
    checkReadsMatchPublication(healthyState(), 0);
}

void test_reads_with_failed_sensors_answer_invalid_data()
{
    // Room temperature, humidity, CO2, pressure, sea-level pressure, IAQ and
    // its accuracy, CO2 and VOC equivalents, dew point, absolute humidity,
    // floor temperature, humidity and absolute humidity, and the trend.
    checkReadsMatchPublication(failedSensorState(), 15);
}

void test_reads_use_the_corrected_readings()
{
    // The bug this replaced: the read path added offsets to raw values but
    // skipped the clamp, and ignored validity. Corrected and clamped now.
    PortState state = healthyState();
    state.sensorData.humidity = reading(99.0f, true);  // + 3 % offset -> clamped to 100
    state.sensorData.probe_humidity = reading(104.0f, true);
    const PortValues values(state);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, readResponse<HabinariPort::RoomHumidity>(values));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, readResponse<HabinariPort::FloorHumidity>(values));
    TEST_ASSERT_EQUAL_FLOAT(20.9f, readResponse<HabinariPort::RoomTemperature>(values));
    TEST_ASSERT_EQUAL_FLOAT(24.7f, readResponse<HabinariPort::FloorTemperature>(values));
    TEST_ASSERT_EQUAL_UINT16(58, readResponse<HabinariPort::RoomAirQualityIndex>(values));
    TEST_ASSERT_EQUAL_FLOAT(1.2f, readResponse<HabinariPort::TemperatureTrend>(values));
}

void test_the_invalid_data_marker_encodes_as_7fff()
{
    // What a float read with no reading puts on the bus, through the stack's
    // own DPT 9 encoder.
    TEST_ASSERT_EQUAL_HEX16(0x7FFF, dpt9Raw(noReading<float>()));
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
    RUN_TEST(test_a_commissioned_device_reports_unlinked_ports_on_every_boot);
    RUN_TEST(test_a_fresh_device_reports_once_ets_gives_it_an_address);
    RUN_TEST(test_leaving_programming_mode_from_ets_reports);
    RUN_TEST(test_leaving_programming_mode_with_the_button_reports);
    RUN_TEST(test_reads_answer_what_publication_sends);
    RUN_TEST(test_reads_with_failed_sensors_answer_invalid_data);
    RUN_TEST(test_reads_use_the_corrected_readings);
    RUN_TEST(test_the_invalid_data_marker_encodes_as_7fff);
    return UNITY_END();
}
