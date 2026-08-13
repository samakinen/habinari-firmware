/**
 * Host tests for the portable room-control logic in main/include/hvac_control.hpp.
 *
 * The psychrometric functions are checked against independently computed
 * reference values rather than against themselves, because the whole point of
 * publishing a dew point is that another device acts on it: a formula that is
 * self-consistently wrong would put water on a customer's floor.
 */

#include "hvac_control.hpp"

#include <unity.h>

#include <cmath>

using namespace sensor_board::hvac;

namespace {

void assertClose(float expected, float actual, float tolerance, const char *what)
{
    if (std::fabs(expected - actual) > tolerance) {
        char message[192];
        std::snprintf(message, sizeof(message), "%s: expected %.4f, got %.4f (tolerance %.4f)",
                      what, expected, actual, tolerance);
        TEST_FAIL_MESSAGE(message);
    }
}

// ---------------------------------------------------------------------------
// Psychrometrics
// ---------------------------------------------------------------------------

void test_saturation_vapour_pressure_matches_reference_table()
{
    // Reference saturation pressures over water (hPa), WMO/Sonntag.
    assertClose(6.112f, psychro::saturationVapourPressureHpa(0.0f), 0.01f, "e_s(0 C)");
    assertClose(12.28f, psychro::saturationVapourPressureHpa(10.0f), 0.05f, "e_s(10 C)");
    assertClose(23.39f, psychro::saturationVapourPressureHpa(20.0f), 0.1f, "e_s(20 C)");
    assertClose(42.47f, psychro::saturationVapourPressureHpa(30.0f), 0.2f, "e_s(30 C)");
}

void test_dew_point_at_saturation_equals_temperature()
{
    // 100 %RH means the air is already at its dew point.
    for (float t = -5.0f; t <= 35.0f; t += 5.0f) {
        assertClose(t, psychro::dewPointC(t, 100.0f), 0.02f, "dew point at 100 %RH");
    }
}

void test_dew_point_reference_values()
{
    // Independently computed Magnus results for typical indoor conditions.
    assertClose(12.0f, psychro::dewPointC(21.0f, 56.0f), 0.3f, "dew point 21 C / 56 %RH");
    assertClose(9.3f, psychro::dewPointC(20.0f, 50.0f), 0.3f, "dew point 20 C / 50 %RH");
    assertClose(16.7f, psychro::dewPointC(25.0f, 60.0f), 0.3f, "dew point 25 C / 60 %RH");
    // Dew point is always at or below the dry-bulb temperature.
    TEST_ASSERT_TRUE(psychro::dewPointC(22.0f, 40.0f) < 22.0f);
}

void test_absolute_humidity_reference_values()
{
    // 20 C / 50 %RH holds ~8.65 g/m3; 25 C / 60 %RH ~13.8 g/m3.
    assertClose(8.65f, psychro::absoluteHumidityGm3(20.0f, 50.0f), 0.1f, "AH 20 C / 50 %RH");
    assertClose(13.8f, psychro::absoluteHumidityGm3(25.0f, 60.0f), 0.2f, "AH 25 C / 60 %RH");
    // Dry air holds nothing.
    assertClose(0.0f, psychro::absoluteHumidityGm3(20.0f, 0.0f), 0.15f, "AH at 0 %RH");
}

void test_absolute_humidity_barely_moves_where_relative_humidity_doubles()
{
    // The property the slab-moisture detector relies on. Take one air parcel at
    // 22 C / 50 %RH and cool it to its dew point: the relative reading runs from
    // 50 % to 100 %, while the absolute reading moves only a few percent.
    //
    // Absolute humidity is mass per unit *volume*, so it is not exactly
    // conserved under cooling — the parcel contracts, concentrating the same
    // water into less space. The exactly conserved quantity is the vapour
    // pressure, which is asserted below. The point stands either way: comparing
    // two air masses at different temperatures by relative humidity is
    // meaningless, and by absolute humidity is very nearly right.
    const float warmAh = psychro::absoluteHumidityGm3(22.0f, 50.0f);
    const float dewPoint = psychro::dewPointC(22.0f, 50.0f);
    const float cooledAh = psychro::absoluteHumidityGm3(dewPoint, 100.0f);

    const float relativeDrift = std::fabs(cooledAh - warmAh) / warmAh;
    TEST_ASSERT_TRUE(relativeDrift < 0.05f);  // vs a 100 % swing in the RH reading

    // Vapour pressure is the exact invariant for the same parcel.
    assertClose(psychro::vapourPressureHpa(22.0f, 50.0f),
                psychro::vapourPressureHpa(dewPoint, 100.0f), 0.01f,
                "vapour pressure across a temperature change");
}

void test_sea_level_pressure_reduction()
{
    // Zero altitude must be an exact identity, so the default costs nothing.
    TEST_ASSERT_EQUAL_FLOAT(98000.0f, psychro::seaLevelPressurePa(98000.0f, 0.0f, 20.0f));
    // ~12 Pa per metre near sea level: 100 m should add roughly 1200 Pa.
    const float reduced = psychro::seaLevelPressurePa(90000.0f, 100.0f, 15.0f);
    TEST_ASSERT_TRUE(reduced > 90000.0f);
    assertClose(91200.0f, reduced, 200.0f, "sea-level reduction at 100 m");
}

// ---------------------------------------------------------------------------
// Setpoint ladder
// ---------------------------------------------------------------------------

void test_setpoint_ladder_derives_modes_from_the_comfort_anchor()
{
    SetpointLadder ladder;
    ladder.comfortHeatingC = 21.0f;
    ladder.standbyHeatingReductionK = 2.0f;
    ladder.economyHeatingReductionK = 4.0f;
    ladder.coolingDeadbandK = 2.0f;
    ladder.standbyCoolingIncreaseK = 2.0f;

    TEST_ASSERT_EQUAL_FLOAT(21.0f, ladder.heatingForPreset(OperatingPreset::Comfort));
    TEST_ASSERT_EQUAL_FLOAT(19.0f, ladder.heatingForPreset(OperatingPreset::Standby));
    TEST_ASSERT_EQUAL_FLOAT(17.0f, ladder.heatingForPreset(OperatingPreset::Economy));
    TEST_ASSERT_EQUAL_FLOAT(23.0f, ladder.coolingForPreset(OperatingPreset::Comfort));
    TEST_ASSERT_EQUAL_FLOAT(25.0f, ladder.coolingForPreset(OperatingPreset::Standby));

    // Moving the anchor moves the whole ladder — the reason for this model.
    ladder.comfortHeatingC = 23.0f;
    TEST_ASSERT_EQUAL_FLOAT(21.0f, ladder.heatingForPreset(OperatingPreset::Standby));
    TEST_ASSERT_EQUAL_FLOAT(25.0f, ladder.coolingForPreset(OperatingPreset::Comfort));

    // Building protection stays absolute in both directions.
    TEST_ASSERT_EQUAL_FLOAT(ladder.protectionHeatingC,
                            ladder.heatingForPreset(OperatingPreset::BuildingProtection));
    TEST_ASSERT_EQUAL_FLOAT(ladder.protectionCoolingC,
                            ladder.coolingForPreset(OperatingPreset::BuildingProtection));
}

// ---------------------------------------------------------------------------
// Dew-point monitor
// ---------------------------------------------------------------------------

DewPointInputs warmRoomWithFloorAt(float floorC)
{
    DewPointInputs in;
    in.roomTemperatureC = 25.0f;
    in.roomHumidityPct = 60.0f;  // dew point ~17 C
    in.roomAirValid = true;
    in.floorTemperatureC = floorC;
    in.floorTemperatureValid = true;
    return in;
}

void test_dew_point_monitor_alarms_when_the_floor_approaches_the_dew_point()
{
    DewPointMonitor monitor;
    monitor.configure({.surfaceSource = DewPointSurfaceSource::FloorProbe,
                       .marginK = 2.0f,
                       .hysteresisK = 1.0f});

    // Floor at 22 C is 5 K clear of the ~17 C dew point: no alarm.
    TEST_ASSERT_FALSE(monitor.update(warmRoomWithFloorAt(22.0f)).alarm);

    // Floor cooled to 18 C is within the 2 K margin: alarm.
    const auto alarmed = monitor.update(warmRoomWithFloorAt(18.0f));
    TEST_ASSERT_TRUE(alarmed.alarm);
    TEST_ASSERT_TRUE(alarmed.marginValid);
    assertClose(1.0f, alarmed.marginK, 0.4f, "dew point margin at 18 C floor");

    // Hysteresis: clearing the margin alone is not enough to release.
    TEST_ASSERT_TRUE(monitor.update(warmRoomWithFloorAt(19.5f)).alarm);
    // Margin + hysteresis clears it.
    TEST_ASSERT_FALSE(monitor.update(warmRoomWithFloorAt(21.0f)).alarm);
}

void test_dew_point_monitor_holds_the_alarm_when_the_surface_reading_disappears()
{
    DewPointMonitor monitor;
    monitor.configure({.surfaceSource = DewPointSurfaceSource::FloorProbe,
                       .marginK = 2.0f,
                       .hysteresisK = 1.0f});
    TEST_ASSERT_TRUE(monitor.update(warmRoomWithFloorAt(18.0f)).alarm);

    DewPointInputs probeLost = warmRoomWithFloorAt(18.0f);
    probeLost.floorTemperatureValid = false;
    const auto out = monitor.update(probeLost);
    TEST_ASSERT_FALSE(out.marginValid);
    TEST_ASSERT_TRUE(out.alarm);  // a lost probe does not dry the surface
}

void test_dew_point_monitor_uses_the_coldest_available_surface()
{
    DewPointMonitor monitor;
    monitor.configure({.surfaceSource = DewPointSurfaceSource::Coldest,
                       .marginK = 2.0f,
                       .hysteresisK = 1.0f});

    DewPointInputs in = warmRoomWithFloorAt(24.0f);  // floor is safe
    in.flowTemperatureC = 16.0f;                     // chilled flow is not
    in.flowTemperatureValid = true;

    const auto out = monitor.update(in);
    TEST_ASSERT_TRUE(out.alarm);
    TEST_ASSERT_TRUE(out.marginK < 0.0f);  // flow is already below the dew point
}

void test_dew_point_monitor_can_be_switched_off()
{
    DewPointMonitor monitor;
    monitor.configure({.surfaceSource = DewPointSurfaceSource::Off});
    const auto out = monitor.update(warmRoomWithFloorAt(10.0f));
    TEST_ASSERT_FALSE(out.alarm);
    TEST_ASSERT_FALSE(out.marginValid);
    TEST_ASSERT_TRUE(out.dewPointValid);  // still published for other devices
}

// ---------------------------------------------------------------------------
// Floor slab moisture
// ---------------------------------------------------------------------------

void test_floor_moisture_relative_threshold_with_hysteresis()
{
    FloorMoistureMonitor monitor;
    monitor.configure({.thresholdPct = 85.0f,
                       .hysteresisPct = 5.0f,
                       .absoluteExcessGm3 = 0.0f});  // excess detector off

    FloorMoistureInputs in;
    in.floorTemperatureC = 20.0f;
    in.floorProbeValid = true;
    in.roomTemperatureC = 22.0f;
    in.roomHumidityPct = 45.0f;
    in.roomAirValid = true;

    in.floorHumidityPct = 70.0f;
    TEST_ASSERT_FALSE(monitor.update(in).alarm);

    in.floorHumidityPct = 86.0f;
    TEST_ASSERT_TRUE(monitor.update(in).alarm);

    in.floorHumidityPct = 82.0f;  // inside the hysteresis band
    TEST_ASSERT_TRUE(monitor.update(in).alarm);

    in.floorHumidityPct = 79.0f;
    TEST_ASSERT_FALSE(monitor.update(in).alarm);
}

void test_floor_moisture_excess_detects_a_wetter_slab_than_the_room()
{
    FloorMoistureMonitor monitor;
    monitor.configure({.thresholdPct = 0.0f,  // relative detector off
                       .hysteresisPct = 5.0f,
                       .absoluteExcessGm3 = 2.0f,
                       .absoluteExcessHysteresisGm3 = 0.5f});

    FloorMoistureInputs in;
    in.roomTemperatureC = 22.0f;
    in.roomHumidityPct = 45.0f;  // ~8.7 g/m3
    in.roomAirValid = true;
    in.floorTemperatureC = 18.0f;
    in.floorProbeValid = true;

    // A cold slab at 40 %RH holds *less* water than the warmer room, even
    // though 40 % in a slab sounds high — no alarm.
    in.floorHumidityPct = 40.0f;
    const auto dry = monitor.update(in);
    TEST_ASSERT_FALSE(dry.alarm);
    TEST_ASSERT_TRUE(dry.floorAbsoluteHumidityGm3 < dry.roomAbsoluteHumidityGm3);

    // Liquid water evaporating in the conduit pushes it well past the room.
    in.floorHumidityPct = 95.0f;  // ~14.6 g/m3 at 18 C
    const auto wet = monitor.update(in);
    TEST_ASSERT_TRUE(wet.alarm);
    TEST_ASSERT_TRUE(wet.excessAlarm);
    TEST_ASSERT_TRUE(wet.excessGm3 > 2.0f);
}

void test_floor_moisture_needs_both_readings_for_the_excess_check()
{
    FloorMoistureMonitor monitor;
    monitor.configure({.thresholdPct = 0.0f, .absoluteExcessGm3 = 2.0f});

    FloorMoistureInputs in;
    in.floorTemperatureC = 18.0f;
    in.floorHumidityPct = 99.0f;
    in.floorProbeValid = true;
    in.roomAirValid = false;  // no room reference

    const auto out = monitor.update(in);
    TEST_ASSERT_FALSE(out.excessValid);
    TEST_ASSERT_FALSE(out.alarm);
}

// ---------------------------------------------------------------------------
// Ventilation
// ---------------------------------------------------------------------------

VentilationInputs autoModeWithCo2(float ppm)
{
    VentilationInputs in;
    in.co2Ppm = ppm;
    in.co2Valid = true;
    in.mode = VentilationMode::Auto;
    return in;
}

void test_ventilation_demand_is_proportional_across_the_co2_band()
{
    VentilationController controller;
    controller.configure({.co2SetpointPpm = 900.0f, .co2BandPpm = 400.0f});

    TEST_ASSERT_EQUAL_UINT8(0, controller.update(autoModeWithCo2(800.0f)).demandPercent);
    TEST_ASSERT_EQUAL_UINT8(0, controller.update(autoModeWithCo2(900.0f)).demandPercent);
    TEST_ASSERT_EQUAL_UINT8(50, controller.update(autoModeWithCo2(1100.0f)).demandPercent);
    TEST_ASSERT_EQUAL_UINT8(100, controller.update(autoModeWithCo2(1300.0f)).demandPercent);
    TEST_ASSERT_EQUAL_UINT8(100, controller.update(autoModeWithCo2(2000.0f)).demandPercent);
}

void test_ventilation_takes_the_worst_channel_and_separates_dehumidification()
{
    VentilationController controller;
    controller.configure({.co2SetpointPpm = 900.0f,
                          .co2BandPpm = 400.0f,
                          .humidityThresholdPct = 65.0f,
                          .humidityBandPct = 15.0f});

    VentilationInputs in = autoModeWithCo2(1000.0f);  // 25 % from CO2
    in.humidityPct = 74.0f;                           // 60 % from humidity
    in.humidityValid = true;

    const auto out = controller.update(in);
    TEST_ASSERT_EQUAL_UINT8(60, out.demandPercent);
    TEST_ASSERT_TRUE(out.co2High);
    TEST_ASSERT_TRUE(out.humidityHigh);
    // Dehumidification must follow humidity alone, never CO2.
    TEST_ASSERT_TRUE(out.dehumidifyRequest);

    VentilationInputs co2Only = autoModeWithCo2(1300.0f);
    co2Only.humidityPct = 40.0f;
    co2Only.humidityValid = true;
    const auto dryAir = controller.update(co2Only);
    TEST_ASSERT_EQUAL_UINT8(100, dryAir.demandPercent);
    TEST_ASSERT_FALSE(dryAir.dehumidifyRequest);
}

void test_ventilation_modes_override_the_automatic_demand()
{
    VentilationController controller;
    controller.configure({.co2SetpointPpm = 900.0f,
                          .co2BandPpm = 400.0f,
                          .manualDemandPercent = 40});

    VentilationInputs in = autoModeWithCo2(2000.0f);
    in.mode = VentilationMode::Off;
    TEST_ASSERT_EQUAL_UINT8(0, controller.update(in).demandPercent);

    in.mode = VentilationMode::Manual;
    TEST_ASSERT_EQUAL_UINT8(40, controller.update(in).demandPercent);

    in.co2Ppm = 400.0f;
    in.mode = VentilationMode::Boost;
    const auto boosted = controller.update(in);
    TEST_ASSERT_EQUAL_UINT8(100, boosted.demandPercent);
    TEST_ASSERT_TRUE(boosted.boostRequest);
}

void test_ventilation_base_demand_applies_only_while_occupied()
{
    VentilationController controller;
    controller.configure({.co2SetpointPpm = 900.0f,
                          .co2BandPpm = 400.0f,
                          .baseDemandPercent = 20});

    VentilationInputs in = autoModeWithCo2(500.0f);
    in.occupied = false;
    TEST_ASSERT_EQUAL_UINT8(0, controller.update(in).demandPercent);

    in.occupied = true;
    TEST_ASSERT_EQUAL_UINT8(20, controller.update(in).demandPercent);
}

// ---------------------------------------------------------------------------
// Thermostat: the behaviours the new group objects expose
// ---------------------------------------------------------------------------

ThermostatConfig coolingCapableConfig()
{
    ThermostatConfig cfg;
    cfg.heatingEnabled = true;
    cfg.coolingEnabled = true;
    cfg.setpoints.comfortHeatingC = 21.0f;
    cfg.setpoints.coolingDeadbandK = 2.0f;
    cfg.heatingAlgorithm = ControlAlgorithm::TwoPoint;
    cfg.coolingAlgorithm = ControlAlgorithm::TwoPoint;
    cfg.minimumHeatCoolChangeoverSeconds = 0.0f;
    cfg.maxFloorTemperatureC = 0.0f;
    cfg.sensorFaultBehavior = SensorFaultBehavior::ForceOff;
    return cfg;
}

ThermostatInputs warmRoomInputs()
{
    ThermostatInputs in;
    in.roomTemperatureC = 26.0f;  // above the 23 C cooling setpoint
    in.roomTemperatureValid = true;
    in.controllerEnable = true;
    in.hvacOperatingMode = OperatingPreset::Comfort;
    in.controllerMode = ControllerMode::Auto;
    return in;
}

void test_dew_point_alarm_blocks_cooling()
{
    ThermostatController thermostat;
    ThermostatConfig cfg = coolingCapableConfig();
    cfg.blockCoolingOnDewPointAlarm = true;
    thermostat.configure(cfg);

    ThermostatInputs in = warmRoomInputs();
    const auto cooling = thermostat.update(in, 1.0f);
    TEST_ASSERT_TRUE(cooling.coolingRequest);

    in.dewPointAlarm = true;
    const auto blocked = thermostat.update(in, 1.0f);
    TEST_ASSERT_FALSE(blocked.coolingRequest);
    TEST_ASSERT_EQUAL_UINT8(0, blocked.coolingControlPercent);
    TEST_ASSERT_TRUE(blocked.coolingBlocked);
    // Bit 12 of DPT 22.101 must carry the reason to the visualisation.
    TEST_ASSERT_TRUE((packStatusRHCC(blocked)
                      & static_cast<uint16_t>(StatusRHCCBit::DewPointStatus)) != 0);
}

void test_dew_point_alarm_can_be_report_only()
{
    ThermostatController thermostat;
    ThermostatConfig cfg = coolingCapableConfig();
    cfg.blockCoolingOnDewPointAlarm = false;
    thermostat.configure(cfg);

    ThermostatInputs in = warmRoomInputs();
    in.dewPointAlarm = true;
    const auto out = thermostat.update(in, 1.0f);
    TEST_ASSERT_TRUE(out.coolingRequest);
    TEST_ASSERT_TRUE((packStatusRHCC(out)
                      & static_cast<uint16_t>(StatusRHCCBit::DewPointStatus)) != 0);
}

void test_switch_heat_and_switch_cool_inputs_disable_their_sequence()
{
    ThermostatController thermostat;
    thermostat.configure(coolingCapableConfig());

    ThermostatInputs in = warmRoomInputs();
    in.switchCool = false;
    const auto out = thermostat.update(in, 1.0f);
    TEST_ASSERT_FALSE(out.coolingRequest);
    TEST_ASSERT_TRUE(out.coolingBlocked);
    TEST_ASSERT_TRUE((packStatusRHCC(out)
                      & static_cast<uint16_t>(StatusRHCCBit::CoolingDisabled)) != 0);
}

void test_minimum_floor_temperature_calls_for_heat_in_a_warm_room()
{
    ThermostatController thermostat;
    ThermostatConfig cfg = coolingCapableConfig();
    cfg.coolingEnabled = false;
    cfg.minFloorTemperatureC = 24.0f;
    cfg.floorHysteresisK = 1.0f;
    cfg.floorComfortOutputPercent = 30;
    thermostat.configure(cfg);

    ThermostatInputs in;
    in.roomTemperatureC = 22.0f;  // room is already at setpoint
    in.roomTemperatureValid = true;
    in.floorTemperatureC = 20.0f;  // but the floor is cold underfoot
    in.floorTemperatureValid = true;
    in.controllerEnable = true;
    in.hvacOperatingMode = OperatingPreset::Comfort;

    const auto out = thermostat.update(in, 1.0f);
    TEST_ASSERT_TRUE(out.floorComfortActive);
    TEST_ASSERT_TRUE(out.heatingRequest);
    TEST_ASSERT_EQUAL_UINT8(30, out.heatingControlPercent);
}

void test_max_floor_temperature_overrides_the_minimum()
{
    ThermostatController thermostat;
    ThermostatConfig cfg = coolingCapableConfig();
    cfg.coolingEnabled = false;
    cfg.maxFloorTemperatureC = 28.0f;
    cfg.minFloorTemperatureC = 24.0f;
    thermostat.configure(cfg);

    ThermostatInputs in;
    in.roomTemperatureC = 15.0f;  // room is cold, heating would run flat out
    in.roomTemperatureValid = true;
    in.floorTemperatureC = 29.0f;  // but the covering is at its limit
    in.floorTemperatureValid = true;
    in.controllerEnable = true;

    const auto out = thermostat.update(in, 1.0f);
    TEST_ASSERT_TRUE(out.floorLimitActive);
    TEST_ASSERT_FALSE(out.heatingRequest);
    TEST_ASSERT_EQUAL_UINT8(0, out.heatingControlPercent);
    TEST_ASSERT_TRUE((packStatusRHCC(out)
                      & static_cast<uint16_t>(StatusRHCCBit::FlowTempLimit)) != 0);
}

void test_frost_and_overheat_alarms_reach_the_status_word()
{
    ThermostatController thermostat;
    ThermostatConfig cfg = coolingCapableConfig();
    cfg.frostAlarmTemperatureC = 5.0f;
    cfg.overheatAlarmTemperatureC = 35.0f;
    thermostat.configure(cfg);

    ThermostatInputs in = warmRoomInputs();
    in.roomTemperatureC = 3.0f;
    const auto frost = thermostat.update(in, 1.0f);
    TEST_ASSERT_TRUE(frost.frostAlarm);
    TEST_ASSERT_TRUE((packStatusRHCC(frost)
                      & static_cast<uint16_t>(StatusRHCCBit::FrostAlarm)) != 0);

    in.roomTemperatureC = 40.0f;
    const auto hot = thermostat.update(in, 1.0f);
    TEST_ASSERT_TRUE(hot.overheatAlarm);
    TEST_ASSERT_TRUE((packStatusRHCC(hot)
                      & static_cast<uint16_t>(StatusRHCCBit::OverheatAlarm)) != 0);
}

void test_both_effective_setpoints_are_reported()
{
    ThermostatController thermostat;
    ThermostatConfig cfg = coolingCapableConfig();
    thermostat.configure(cfg);

    ThermostatInputs in = warmRoomInputs();
    in.roomTemperatureC = 22.0f;  // sitting inside the dead band
    const auto out = thermostat.update(in, 1.0f);

    TEST_ASSERT_EQUAL_FLOAT(21.0f, out.heatingSetpointC);
    TEST_ASSERT_EQUAL_FLOAT(23.0f, out.coolingSetpointC);
    // The dead band must survive whatever the controller is currently doing.
    TEST_ASSERT_TRUE(out.coolingSetpointC - out.heatingSetpointC
                     >= cfg.setpoints.coolingDeadbandK - 0.001f);
}

void test_status_gen_distinguishes_missing_from_faulty()
{
    // Never fitted: out of service, not a fault.
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(StatusGenBit::OutOfService),
                            packStatusGen(false, false));
    // Fitted but not delivering: fault.
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(StatusGenBit::Fault),
                            packStatusGen(true, false));
    // Healthy: clean.
    TEST_ASSERT_EQUAL_UINT8(0, packStatusGen(true, true));
    // Healthy but reporting an alarm on its measurand.
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(StatusGenBit::InAlarm),
                            packStatusGen(true, true, true));
}

} // namespace

// Required by the Unity harness; nothing here needs per-test fixturing.
extern "C" void setUp() {}
extern "C" void tearDown() {}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_saturation_vapour_pressure_matches_reference_table);
    RUN_TEST(test_dew_point_at_saturation_equals_temperature);
    RUN_TEST(test_dew_point_reference_values);
    RUN_TEST(test_absolute_humidity_reference_values);
    RUN_TEST(test_absolute_humidity_barely_moves_where_relative_humidity_doubles);
    RUN_TEST(test_sea_level_pressure_reduction);

    RUN_TEST(test_setpoint_ladder_derives_modes_from_the_comfort_anchor);

    RUN_TEST(test_dew_point_monitor_alarms_when_the_floor_approaches_the_dew_point);
    RUN_TEST(test_dew_point_monitor_holds_the_alarm_when_the_surface_reading_disappears);
    RUN_TEST(test_dew_point_monitor_uses_the_coldest_available_surface);
    RUN_TEST(test_dew_point_monitor_can_be_switched_off);

    RUN_TEST(test_floor_moisture_relative_threshold_with_hysteresis);
    RUN_TEST(test_floor_moisture_excess_detects_a_wetter_slab_than_the_room);
    RUN_TEST(test_floor_moisture_needs_both_readings_for_the_excess_check);

    RUN_TEST(test_ventilation_demand_is_proportional_across_the_co2_band);
    RUN_TEST(test_ventilation_takes_the_worst_channel_and_separates_dehumidification);
    RUN_TEST(test_ventilation_modes_override_the_automatic_demand);
    RUN_TEST(test_ventilation_base_demand_applies_only_while_occupied);

    RUN_TEST(test_dew_point_alarm_blocks_cooling);
    RUN_TEST(test_dew_point_alarm_can_be_report_only);
    RUN_TEST(test_switch_heat_and_switch_cool_inputs_disable_their_sequence);
    RUN_TEST(test_minimum_floor_temperature_calls_for_heat_in_a_warm_room);
    RUN_TEST(test_max_floor_temperature_overrides_the_minimum);
    RUN_TEST(test_frost_and_overheat_alarms_reach_the_status_word);
    RUN_TEST(test_both_effective_setpoints_are_reported);
    RUN_TEST(test_status_gen_distinguishes_missing_from_faulty);

    return UNITY_END();
}
