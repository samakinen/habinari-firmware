// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// Outbound publication: what the device sends spontaneously, and the shaping
// that keeps it a good bus citizen. When it is offered is decided by
// PublishSchedule (knx_publish_schedule.hpp) in the service loop; what reaches
// the bus is decided per object by the transmit policy set up here.

#include "knx_service_internal.hpp"

#include "control_core.hpp"
#include "control_service.hpp"
#include "hvac_control.hpp"
#include "knx_product.hpp"
#include "sensor_data.h"

#include "esp_random.h"
#include "esp_timer.h"

#include "knx/util/log.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>

using namespace knx;
using namespace knx::application;

namespace habinari_knx::service {

namespace {

namespace hvac_ns = habinari::hvac;
namespace control = habinari::control;
using control::LockGuard;
using Settings = control::Settings;
using Outputs = control::Outputs;

// --- Bus-citizenship: boot-storm mitigation -------------------------------
// On a bus-wide power-up (line powered on) every device would otherwise dump
// its full object set at t=0, swamping the shared ~50 telegram/s TP1 medium —
// worse the more of these boards share a line. Two independent guards:
//   1. A per-device random startup delay before the first full publish, so
//      devices on the same line desynchronise instead of all firing at once.
//   2. A global outbound telegram rate limit. The stack defers over-budget
//      sends (coalescing each object to its latest value) and drains them from
//      loop(), so even the initial burst and later change storms are spread
//      over time with nothing dropped. Requires a monotonic time source, which
//      also enables the cyclic status heartbeats.
constexpr uint32_t kStartupPublishMaxJitterMs = 8000;
constexpr uint32_t kMaxUnsolicitedTelegramsPerWindow = 5;
constexpr uint32_t kTelegramRateWindowMs = 1000;

// The per-port "has this changed?" bookkeeping this snapshot used to carry is
// gone: the stack's GroupObjectTransmitPolicy already decides Send / Defer /
// Suppress per object from the COV delta, the minimum repetition time and the
// heartbeat (see applyTransmitPolicies), which is the same model the KNX sensor
// FBs specify. So the loop publishes every port every tick and lets the policy
// layer drop what has not moved — one place deciding, instead of two that could
// disagree.
struct PublishSnapshot {
    sensor_data_t sensorData{};
    Settings settings{};
    Outputs out{};
    hvac_ns::OperatingPreset hvacOperatingMode{hvac_ns::OperatingPreset::Comfort};
    hvac_ns::ControllerMode controllerMode{hvac_ns::ControllerMode::Auto};
    hvac_ns::VentilationMode ventilationMode{hvac_ns::VentilationMode::Auto};
    bool controllerOnOff{true};
};

PublishSnapshot takePublishSnapshot()
{
    PublishSnapshot snapshot;
    LockGuard lock(g_state.mutex);
    snapshot.sensorData = g_state.latestSensorData;
    snapshot.settings = g_state.hvac;
    snapshot.out = g_state.out;
    snapshot.hvacOperatingMode = g_state.in.hvacOperatingMode;
    snapshot.controllerMode = g_state.in.controllerMode;
    snapshot.ventilationMode = g_state.in.ventilationMode;
    snapshot.controllerOnOff = g_state.in.controllerOnOff;
    return snapshot;
}

// The value types the product's ports carry. Listing them keeps a call site
// from reaching app.publish() with a type it was never meant to (an int, a
// double) through template deduction; convert explicitly at the call instead.
//
// A port the ETS project left unlinked publishes as a successful no-op, so
// nothing here fires for unused datapoints; see reportUnlinkedPorts for the
// one-shot inventory of those. Anything that does reach this log is a genuine
// send failure, printed by name because the bare enum value was unreadable.
template <typename T>
void publishPort(HabinariApp &app, HabinariPort port, T value, const char *label)
{
    static_assert(std::is_same_v<T, float> || std::is_same_v<T, bool> ||
                      std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t> ||
                      std::is_same_v<T, Dpt20Mode>,
                  "publishPort: unsupported port value type");
    const auto result = app.publish(port, value);
    if (result.isError()) {
        KNX_LOGW(TAG, "%s publish failed: %s", label, util::errorCodeToString(result.error()));
    }
}

// Push the send-on-change / minimum-repetition / cyclic-heartbeat transmit
// policy (see knx/application/group_object.hpp) to every transmitting port.
//
// This is the device's implementation of the KNX sensor Functional Block
// sending model (Vol 7/19/20 clause 3.x: Heartbeat, MinRepTime, COVCondition):
// the ETS parameters name those three quantities directly, and the stack turns
// them into Send / Defer / Suppress decisions. The publish path therefore does
// not filter anything itself.
//
// Cheap and idempotent, so it is simply re-applied every control tick (~1 Hz)
// rather than only on onParameterChanged, which keeps it in step with the
// "reconfigure-from-settings-every-tick" pattern used for the controllers.
// Write-only CommandPorts never transmit and are intentionally absent.
void applyTransmitPolicies(HabinariApp &app, const Settings &settings)
{
    const uint32_t heartbeatMs = settings.heartbeatSeconds * 1000u;
    const uint32_t minRepMs = settings.minRepTimeSeconds * 1000u;

    const auto covPolicy = [&](float threshold) {
        GroupObjectTransmitPolicy policy{};
        policy.onChangeEnabled = true;
        policy.changeThreshold = static_cast<double>(threshold);
        policy.cyclicIntervalMs = heartbeatMs;
        policy.minIntervalMs = minRepMs;
        return policy;
    };

    // Analogue measurements and derived values: a per-measurand COV delta, since
    // 0.2 is a sensible step in K and a meaningless one in ppm.
    struct CovPort {
        HabinariPort port;
        float threshold;
    };
    const CovPort covPorts[] = {
        {HabinariPort::RoomTemperature, settings.roomTemperatureCovK},
        {HabinariPort::RoomHumidity, settings.roomHumidityCovPct},
        {HabinariPort::RoomCo2, settings.co2CovPpm},
        {HabinariPort::RoomAirPressure, settings.pressureCovPa},
        {HabinariPort::RoomAirPressureSeaLevel, settings.pressureCovPa},
        {HabinariPort::RoomAirQualityIndex, settings.airQualityCovIndex},
        {HabinariPort::RoomCo2Equivalent, settings.co2CovPpm},
        {HabinariPort::RoomVocEquivalent, settings.co2CovPpm},
        {HabinariPort::RoomDewPoint, settings.derivedCov},
        {HabinariPort::RoomAbsoluteHumidity, settings.derivedCov},
        {HabinariPort::FloorTemperature, settings.floorTemperatureCovK},
        {HabinariPort::FloorHumidity, settings.floorHumidityCovPct},
        {HabinariPort::FloorAbsoluteHumidity, settings.derivedCov},
        {HabinariPort::DewPointMargin, settings.derivedCov},
        {HabinariPort::SetpointBase, settings.roomTemperatureCovK},
        {HabinariPort::SetpointStatus, settings.roomTemperatureCovK},
        {HabinariPort::SetpointHeatingStatus, settings.roomTemperatureCovK},
        {HabinariPort::SetpointCoolingStatus, settings.roomTemperatureCovK},
        {HabinariPort::SetpointShiftStatus, settings.roomTemperatureCovK},
        {HabinariPort::Co2Setpoint, settings.co2CovPpm},
        // The trend is published in K/h, so its COV threshold is the room
        // temperature COV per hour rather than per degree.
        {HabinariPort::TemperatureTrend, settings.roomTemperatureCovK},
    };
    for (const auto &entry : covPorts) {
        (void)app.setTransmitPolicy(entry.port, covPolicy(entry.threshold));
    }

    // Discrete states, enumerations and status words: any change is meaningful,
    // so only the heartbeat and the minimum repetition time apply.
    GroupObjectTransmitPolicy discrete{};
    discrete.onChangeEnabled = true;
    discrete.changeThreshold = 0.0;  // <= 0 with onChange means "any change"
    discrete.cyclicIntervalMs = heartbeatMs;
    discrete.minIntervalMs = minRepMs;

    static constexpr HabinariPort kDiscretePorts[] = {
        HabinariPort::AirQualityAccuracy,
        HabinariPort::FloorMoistureAlarm,
        HabinariPort::FloorLimitActive,
        HabinariPort::FloorComfortActive,
        HabinariPort::DewPointAlarm,
        HabinariPort::FreeCoolingAvailable,
        HabinariPort::FreeDryingAvailable,
        HabinariPort::ControllerOnOff,
        HabinariPort::HvacModeStatus,
        HabinariPort::ContrModeStatus,
        HabinariPort::ContrModeSecondary,
        HabinariPort::HeatingControlValue,
        HabinariPort::CoolingControlValue,
        HabinariPort::HeatingRequest,
        HabinariPort::CoolingRequest,
        HabinariPort::HeatCoolModeStatus,
        HabinariPort::EnableHeatStatus,
        HabinariPort::EnableCoolStatus,
        HabinariPort::ControllerStatus,
        HabinariPort::VentilationDemand,
        HabinariPort::VentilationStage,
        HabinariPort::VentilationMode,
        HabinariPort::VentilationBoostRequest,
        HabinariPort::DehumidifyRequest,
        HabinariPort::AirQualityStatus,
        HabinariPort::RoomSensorStatus,
        HabinariPort::FloorProbeStatus,
        HabinariPort::AirQualitySensorStatus,
        HabinariPort::SensorHealthMask,
        HabinariPort::SensorDisagreementAlarm,
        HabinariPort::OccupancyDetected,
        HabinariPort::EstimatedOccupants,
        HabinariPort::WindowOpenDetected,
    };
    for (const auto port : kDiscretePorts) {
        (void)app.setTransmitPolicy(port, discrete);
    }

    // Alarms are exempt from the minimum repetition time. That floor exists to
    // keep a noisy measurement from flooding the bus, and applying it to a fire
    // alarm would delay it by up to that many seconds for no benefit — an alarm
    // that has changed state is exactly the telegram worth sending at once.
    GroupObjectTransmitPolicy alarm = discrete;
    alarm.minIntervalMs = 0;
    static constexpr HabinariPort kAlarmPorts[] = {
        HabinariPort::FireAlarm,
        HabinariPort::FirePreAlarm,
        HabinariPort::DeviceFault,
    };
    for (const auto port : kAlarmPorts) {
        (void)app.setTransmitPolicy(port, alarm);
    }
}

// Publish every transmitting port. The transmit policy decides what actually
// reaches the bus (see applyTransmitPolicies), so an unchanged value costs a
// compare and nothing else; a port the ETS project left unlinked is a no-op.
void publishSnapshot(HabinariApp &app, const PublishSnapshot &snapshot)
{
    const auto &out = snapshot.out;
    const auto &sensors = snapshot.sensorData;
    const control::CorrectedReadings readings = control::correctReadings(sensors, snapshot.settings);

    // --- Room air measurements (FB RTS / RRHS / RAQS) ---
    if (readings.roomTemperatureValid) {
        publishPort(app, HabinariPort::RoomTemperature, readings.roomTemperatureC,
                    "room temperature");
    }
    if (readings.roomHumidityValid) {
        publishPort(app, HabinariPort::RoomHumidity, readings.roomHumidityPct, "room humidity");
    }
    if (readings.co2Valid) {
        publishPort(app, HabinariPort::RoomCo2, readings.co2Ppm, "room co2");
    }
    if (readings.pressureValid) {
        publishPort(app, HabinariPort::RoomAirPressure, readings.pressurePa, "air pressure");
        publishPort(app, HabinariPort::RoomAirPressureSeaLevel, out.seaLevelPressurePa,
                    "sea-level pressure");
    }
    if (readings.iaqValid) {
        publishPort(app, HabinariPort::RoomAirQualityIndex,
                    static_cast<uint16_t>(readings.iaqIndex + 0.5f), "air quality index");
        publishPort(app, HabinariPort::AirQualityAccuracy, sensors.air_quality_accuracy,
                    "air quality accuracy");
    }
    if (sensors.co2_equivalent.valid) {
        publishPort(app, HabinariPort::RoomCo2Equivalent, sensors.co2_equivalent.value,
                    "co2 equivalent");
    }
    if (sensors.voc_equivalent.valid) {
        publishPort(app, HabinariPort::RoomVocEquivalent, sensors.voc_equivalent.value,
                    "voc equivalent");
    }

    // --- Derived room air values ---
    if (readings.roomAirValid()) {
        publishPort(app, HabinariPort::RoomDewPoint, out.roomDewPointC, "room dew point");
        publishPort(app, HabinariPort::RoomAbsoluteHumidity, out.roomAbsoluteHumidityGm3,
                    "room absolute humidity");
    }

    // --- Floor probe (FB FTS + slab moisture) ---
    if (readings.floorTemperatureValid) {
        publishPort(app, HabinariPort::FloorTemperature, readings.floorTemperatureC,
                    "floor temperature");
    }
    if (readings.floorHumidityValid) {
        publishPort(app, HabinariPort::FloorHumidity, readings.floorHumidityPct,
                    "floor humidity");
    }
    if (readings.floorProbeValid()) {
        publishPort(app, HabinariPort::FloorAbsoluteHumidity, out.floorAbsoluteHumidityGm3,
                    "floor absolute humidity");
    }
    publishPort(app, HabinariPort::FloorMoistureAlarm, out.floorMoistureAlarm,
                "floor moisture alarm");
    publishPort(app, HabinariPort::FloorLimitActive, out.floorLimitActive, "floor limit active");
    publishPort(app, HabinariPort::FloorComfortActive, out.floorComfortActive,
                "floor comfort active");

    // --- Condensation protection (FB DPS) ---
    publishPort(app, HabinariPort::DewPointAlarm, out.dewPointAlarm, "dew point alarm");
    publishPort(app, HabinariPort::DewPointMargin, out.dewPointMarginK, "dew point margin");
    publishPort(app, HabinariPort::FreeCoolingAvailable, out.freeCoolingAvailable,
                "free cooling available");
    publishPort(app, HabinariPort::FreeDryingAvailable, out.freeDryingAvailable,
                "free drying available");

    // --- Mode and setpoints (FB RTSM) ---
    publishPort(app, HabinariPort::ControllerOnOff, snapshot.controllerOnOff,
                "controller on/off");
    // HvacModeStatus reports the mode the controller resolved to, which
    // window/presence handling can move away from the requested one.
    publishPort(app, HabinariPort::HvacModeStatus, static_cast<Dpt20Mode>(out.activePreset),
                "hvac mode status");
    publishPort(app, HabinariPort::ContrModeStatus,
                hvac_ns::controllerModeToContrMode(out.activeControllerMode), "contr mode status");
    publishPort(app, HabinariPort::ContrModeSecondary,
                hvac_ns::controllerModeToContrMode(out.activeControllerMode),
                "contr mode secondary");
    publishPort(app, HabinariPort::SetpointBase, snapshot.settings.comfortHeatingSetpointC,
                "base setpoint");
    publishPort(app, HabinariPort::SetpointShiftStatus, out.setpointShiftFeedbackK,
                "setpoint shift status");
    publishPort(app, HabinariPort::SetpointStatus, out.activeSetpointC, "active setpoint");
    publishPort(app, HabinariPort::SetpointHeatingStatus, out.heatingSetpointC,
                "heating setpoint");
    publishPort(app, HabinariPort::SetpointCoolingStatus, out.coolingSetpointC,
                "cooling setpoint");

    // --- Controller outputs (FB RTC) ---
    publishPort(app, HabinariPort::HeatingControlValue, out.heatingControlPercent,
                "heating control value");
    publishPort(app, HabinariPort::CoolingControlValue, out.coolingControlPercent,
                "cooling control value");
    publishPort(app, HabinariPort::HeatingRequest, out.heatingRequest, "heating request");
    publishPort(app, HabinariPort::CoolingRequest, out.coolingRequest, "cooling request");
    publishPort(app, HabinariPort::HeatCoolModeStatus, out.heatCoolModeHeating,
                "heat/cool mode status");
    publishPort(app, HabinariPort::EnableHeatStatus, out.enableHeat, "enable heat status");
    publishPort(app, HabinariPort::EnableCoolStatus, out.enableCool, "enable cool status");
    publishPort(app, HabinariPort::ControllerStatus, out.controllerStatus, "controller status");

    // --- Ventilation / air quality ---
    publishPort(app, HabinariPort::Co2Setpoint, snapshot.settings.ventilationSetpointPpm,
                "co2 setpoint");
    publishPort(app, HabinariPort::VentilationDemand, out.ventilationDemandPercent,
                "ventilation demand");
    publishPort(app, HabinariPort::VentilationStage, static_cast<uint8_t>(out.ventilationLevel),
                "ventilation stage");
    publishPort(app, HabinariPort::VentilationMode,
                static_cast<uint8_t>(snapshot.ventilationMode), "ventilation mode");
    publishPort(app, HabinariPort::VentilationBoostRequest, out.ventilationBoostRequest,
                "ventilation boost request");
    publishPort(app, HabinariPort::DehumidifyRequest, out.dehumidifyRequest,
                "dehumidify request");
    publishPort(app, HabinariPort::AirQualityStatus, out.airQualityStatus,
                "air quality status");

    // --- Device diagnostics ---
    publishPort(app, HabinariPort::DeviceFault, out.deviceFault, "device fault");
    publishPort(app, HabinariPort::RoomSensorStatus, out.roomSensorStatus, "room sensor status");
    publishPort(app, HabinariPort::FloorProbeStatus, out.floorProbeStatus, "floor probe status");
    publishPort(app, HabinariPort::AirQualitySensorStatus, out.airQualitySensorStatus,
                "air quality sensor status");
    publishPort(app, HabinariPort::SensorHealthMask, out.sensorHealthMask, "sensor health mask");
    publishPort(app, HabinariPort::SensorDisagreementAlarm, out.sensorDisagreement,
                "sensor disagreement");

    // --- Derived events ---
    // The alarms are published unconditionally so a cleared alarm is reported
    // too; the measured trend only once its window has filled, because a
    // half-filled regression is not a trend.
    publishPort(app, HabinariPort::FireAlarm, sensors.events.fire_alarm, "fire alarm");
    publishPort(app, HabinariPort::FirePreAlarm, sensors.events.fire_pre_alarm,
                "fire pre-alarm");
    if (sensors.trends.temperature.valid) {
        publishPort(app, HabinariPort::TemperatureTrend,
                    sensors.trends.temperature.per_minute * 60.0f, "temperature trend");
    }
    publishPort(app, HabinariPort::OccupancyDetected, sensors.events.occupancy_detected,
                "occupancy detected");
    publishPort(app, HabinariPort::EstimatedOccupants, sensors.events.estimated_occupants,
                "estimated occupants");
    publishPort(app, HabinariPort::WindowOpenDetected, sensors.events.window_open_detected,
                "window open detected");
}

}  // namespace

// One-shot inventory of transmitting communication objects that the ETS project
// linked no group address to. Publishing on them is a silent no-op, so without
// this the integrator has no way to tell "configured off" from "broken". Logged
// once per transition into Operational (i.e. after every download).
void reportUnlinkedPorts(HabinariApp &app)
{
    const auto &compiled = app.compiledEndpoint();
    const auto &runtimeObjects = compiled.runtime.communicationObjects;
    const auto &exportObjects = compiled.exportDescriptor.communicationObjects;

    unsigned unlinked = 0;
    for (size_t slot = 0; slot < runtimeObjects.size(); ++slot) {
        const auto &descriptor = runtimeObjects[slot];
        if (!descriptor.transmit || app.isPortLinked(descriptor.logicalId)) {
            continue;
        }
        ++unlinked;
        KNX_LOGI(TAG,
                 "CO #%u \"%.*s\" has no group address - not transmitted",
                 static_cast<unsigned>(exportObjects[slot].exportNumber),
                 static_cast<int>(exportObjects[slot].displayName.size()),
                 exportObjects[slot].displayName.data());
    }

    if (unlinked == 0) {
        KNX_LOGI(TAG, "All transmitting communication objects are linked");
    } else {
        KNX_LOGI(TAG, "%u transmitting communication object(s) unlinked in this project", unlinked);
    }
}

void configureTransmitShaping(HabinariApp &app)
{
    // Outbound transmit shaping. A monotonic millisecond clock is required for
    // the global rate limiter, per-object cyclic heartbeats and min-interval
    // floors to take effect (see applyTransmitPolicies + setTelegramRateLimit).
    app.setTimeSource([]() -> uint32_t {
        return static_cast<uint32_t>(esp_timer_get_time() / 1000);
    });
    app.setTelegramRateLimit(application::TelegramRateLimitConfig{
        .maxTelegrams = kMaxUnsolicitedTelegramsPerWindow,
        .perWindowMs = kTelegramRateWindowMs,
        .minGapMs = 0,
    });
}

void refreshTransmitPolicies(HabinariApp &app)
{
    // The transmit policy is derived from the configured COV deltas and
    // repetition times, so it is refreshed when a tick may have changed
    // them — same cadence as before, just driven from the other side.
    Settings settings;
    {
        LockGuard lock(g_state.mutex);
        settings = g_state.hvac;
    }
    applyTransmitPolicies(app, settings);
}

void publishAllState(HabinariApp &app)
{
    // The snapshot is taken only here: it copies the whole state under the
    // control mutex, and the service loop also runs on every inbound frame.
    publishSnapshot(app, takePublishSnapshot());
}

uint32_t startupPublishJitterMs()
{
    return esp_random() % (kStartupPublishMaxJitterMs + 1u);
}

}  // namespace habinari_knx::service
