// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// Group-object read responses. Every transmitting port answers an
// A_GroupValue_Read from the control state, under the control lock; what is
// sent spontaneously is knx_publish.cpp's business.

#include "knx_service_internal.hpp"

#include "control_service.hpp"
#include "hvac_control.hpp"
#include "knx_product.hpp"

#include <cstdint>

using namespace knx::application;

namespace habinari_knx::service {

namespace {
namespace hvac_ns = habinari::hvac;
using habinari::control::LockGuard;
}  // namespace

void bindGroupObjectReads(HabinariBindings &bindings)
{
    bindings
        // ---- Measurements: read requests are answered from the same
        // corrected readings the control loops use, so a bus read and a
        // spontaneous send can never disagree.
        .provideState<HabinariPort::RoomTemperature>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.temperature.value
                   + g_state.hvac.roomTemperatureOffsetK;
        })
        .provideState<HabinariPort::RoomHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::clampf(
                g_state.latestSensorData.humidity.value + g_state.hvac.roomHumidityOffsetPct,
                0.0f, 100.0f);
        })
        .provideState<HabinariPort::RoomCo2>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.co2.value;
        })
        .provideState<HabinariPort::RoomAirPressure>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.pressure.value;
        })
        .provideState<HabinariPort::RoomAirPressureSeaLevel>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.seaLevelPressurePa;
        })
        .provideState<HabinariPort::RoomAirQualityIndex>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint16_t>(g_state.latestSensorData.iaq.value + 0.5f);
        })
        .provideState<HabinariPort::RoomCo2Equivalent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.co2_equivalent.value;
        })
        .provideState<HabinariPort::RoomVocEquivalent>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.voc_equivalent.value;
        })
        .provideState<HabinariPort::AirQualityAccuracy>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.air_quality_accuracy;
        })
        .provideState<HabinariPort::RoomDewPoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomDewPointC;
        })
        .provideState<HabinariPort::RoomAbsoluteHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomAbsoluteHumidityGm3;
        })
        .provideState<HabinariPort::FloorTemperature>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.probe_temperature.value
                   + g_state.hvac.floorTemperatureOffsetK;
        })
        .provideState<HabinariPort::FloorHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.probe_humidity.value;
        })
        .provideState<HabinariPort::FloorAbsoluteHumidity>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorAbsoluteHumidityGm3;
        })
        .provideState<HabinariPort::FloorMoistureAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorMoistureAlarm;
        })
        .provideState<HabinariPort::FloorLimitActive>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorLimitActive;
        })
        .provideState<HabinariPort::FloorComfortActive>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorComfortActive;
        })

        // ---- Condensation protection (FB DPS) ----
        .provideState<HabinariPort::DewPointAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dewPointAlarm;
        })
        .provideState<HabinariPort::DewPointMargin>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dewPointMarginK;
        })

        // Derived from the neighbour-device inputs (see knx_group_writes.cpp).
        .provideState<HabinariPort::FreeCoolingAvailable>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.freeCoolingAvailable;
        })
        .provideState<HabinariPort::FreeDryingAvailable>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.freeDryingAvailable;
        })

        // ---- Mode and setpoints (FB RTSM) ----
        .provideState<HabinariPort::ControllerOnOff>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.in.controllerOnOff;
        })
        .provideState<HabinariPort::HvacModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<Dpt20Mode>(g_state.out.activePreset);
        })
        .provideState<HabinariPort::ContrModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::controllerModeToContrMode(g_state.out.activeControllerMode);
        })
        .provideState<HabinariPort::ContrModeSecondary>([]() {
            LockGuard lock(g_state.mutex);
            return hvac_ns::controllerModeToContrMode(g_state.out.activeControllerMode);
        })
        .provideState<HabinariPort::SetpointBase>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.comfortHeatingSetpointC;
        })
        .provideState<HabinariPort::SetpointShiftStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.setpointShiftFeedbackK;
        })
        .provideState<HabinariPort::SetpointStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.activeSetpointC;
        })
        .provideState<HabinariPort::SetpointHeatingStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingSetpointC;
        })
        .provideState<HabinariPort::SetpointCoolingStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingSetpointC;
        })

        // ---- Controller outputs (FB RTC) ----
        .provideState<HabinariPort::HeatingControlValue>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingControlPercent;
        })
        .provideState<HabinariPort::CoolingControlValue>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingControlPercent;
        })
        .provideState<HabinariPort::HeatingRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatingRequest;
        })
        .provideState<HabinariPort::CoolingRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.coolingRequest;
        })
        .provideState<HabinariPort::HeatCoolModeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.heatCoolModeHeating;
        })
        .provideState<HabinariPort::EnableHeatStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.enableHeat;
        })
        .provideState<HabinariPort::EnableCoolStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.enableCool;
        })
        .provideState<HabinariPort::ControllerStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.controllerStatus;
        })

        // ---- Ventilation / air quality ----
        .provideState<HabinariPort::Co2Setpoint>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.hvac.ventilationSetpointPpm;
        })
        .provideState<HabinariPort::VentilationDemand>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.ventilationDemandPercent;
        })
        .provideState<HabinariPort::VentilationStage>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.out.ventilationLevel);
        })
        .provideState<HabinariPort::VentilationMode>([]() {
            LockGuard lock(g_state.mutex);
            return static_cast<uint8_t>(g_state.in.ventilationMode);
        })
        .provideState<HabinariPort::VentilationBoostRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.ventilationBoostRequest;
        })
        .provideState<HabinariPort::DehumidifyRequest>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.dehumidifyRequest;
        })
        .provideState<HabinariPort::AirQualityStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.airQualityStatus;
        })

        // ---- Device diagnostics ----
        .provideState<HabinariPort::DeviceFault>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.deviceFault;
        })
        .provideState<HabinariPort::RoomSensorStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.roomSensorStatus;
        })
        .provideState<HabinariPort::FloorProbeStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.floorProbeStatus;
        })
        .provideState<HabinariPort::AirQualitySensorStatus>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.airQualitySensorStatus;
        })
        .provideState<HabinariPort::SensorHealthMask>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.sensorHealthMask;
        })
        .provideState<HabinariPort::SensorDisagreementAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.out.sensorDisagreement;
        })

        // ---- Derived events (see sensor_fusion.hpp) ----
        .provideState<HabinariPort::FireAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.fire_alarm;
        })
        .provideState<HabinariPort::FirePreAlarm>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.fire_pre_alarm;
        })
        .provideState<HabinariPort::TemperatureTrend>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.trends.temperature.per_minute * 60.0f;
        })
        .provideState<HabinariPort::OccupancyDetected>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.occupancy_detected;
        })
        .provideState<HabinariPort::EstimatedOccupants>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.estimated_occupants;
        })
        .provideState<HabinariPort::WindowOpenDetected>([]() {
            LockGuard lock(g_state.mutex);
            return g_state.latestSensorData.events.window_open_detected;
        });
}

}  // namespace habinari_knx::service
