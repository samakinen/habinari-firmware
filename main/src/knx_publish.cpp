// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// Outbound publication: what the device sends spontaneously, and the shaping
// that keeps it a good bus citizen. When it is offered is decided by
// PublishSchedule (knx_publish_schedule.hpp) in the service loop; what reaches
// the bus is decided per object by the transmit policy set up here.

#include "knx_service_internal.hpp"

#include "control_core.hpp"
#include "control_service.hpp"
#include "knx_port_values.hpp"
#include "knx_product.hpp"

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

namespace control = habinari::control;
using control::LockGuard;
using Settings = control::Settings;

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

// The per-port "has this changed?" bookkeeping a publish snapshot used to
// carry is gone: the stack's GroupObjectTransmitPolicy already decides Send /
// Defer / Suppress per object from the COV delta, the minimum repetition time
// and the heartbeat (see applyTransmitPolicies), which is the same model the KNX
// sensor FBs specify. So the loop publishes every port every tick and lets the
// policy layer drop what has not moved — one place deciding, instead of two
// that could disagree.

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

// Offer every transmitting port its value (knx_port_values.hpp). The transmit
// policy decides what actually reaches the bus (see applyTransmitPolicies), so
// an unchanged value costs a compare and nothing else; a port the ETS project
// left unlinked is a no-op; a measurement with no valid reading is not offered
// at all, so a dead sensor's last value never goes out.
void publishValues(HabinariApp &app, const PortValues &values)
{
    forEachTransmitPort([&]<HabinariPort Port>(const char *label) {
        if (const auto value = values.template get<Port>()) {
            publishPort(app, Port, *value, label);
        }
    });
}

}  // namespace

// One-shot inventory of transmitting communication objects that the ETS project
// linked no group address to. Publishing on them is a silent no-op, so without
// this the integrator has no way to tell "configured off" from "broken". Logged
// on every transition into Operational (PublishSchedule): each boot of a
// commissioned device, and each return from programming mode or a download.
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

PortValues currentPortValues()
{
    PortState state;
    {
        LockGuard lock(g_state.mutex);
        state.sensorData = g_state.latestSensorData;
        state.settings = g_state.hvac;
        state.out = g_state.out;
        state.ventilationMode = g_state.in.ventilationMode;
        state.controllerOnOff = g_state.in.controllerOnOff;
    }
    return PortValues(state);
}

void publishAllState(HabinariApp &app)
{
    // The state is copied only here, once per control tick: it is the whole
    // state under the control mutex, and the service loop also runs on every
    // inbound frame.
    publishValues(app, currentPortValues());
}

uint32_t startupPublishJitterMs()
{
    return esp_random() % (kStartupPublishMaxJitterMs + 1u);
}

}  // namespace habinari_knx::service
