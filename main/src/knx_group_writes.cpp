// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// ---------------------------------------------------------------------------
// Inbound group writes.
//
// A write that has a control_state_write() equivalent goes through it, so a KNX
// telegram meets the same gate a Modbus or MQTT write does — finite, in range,
// a defined code point — and a refusal is logged instead of cast into the
// controller. Which command a port stands for, and how its bus value maps, is
// knx_group_writes.hpp (host-tested). What has no command (the neighbour-device
// measurements, the changeover inputs) is written directly.
// ---------------------------------------------------------------------------

#include "knx_service_internal.hpp"

#include "control_service.hpp"
#include "control_state.h"
#include "knx_group_writes.hpp"
#include "knx_product.hpp"

#include "esp_err.h"
#include "esp_log.h"

#include <cmath>
#include <cstdint>

using namespace knx::application;

namespace habinari_knx::service {

namespace {

using habinari::control::LockGuard;

void logInvalidData(const char *label)
{
    ESP_LOGW(TAG, "%s: sender reports invalid data (DPT 9 7FFFh), ignored", label);
}

// Hand a mapped write to control_state_write(). True if it was applied.
bool submit(const GroupCommand &command, const char *label)
{
    switch (command.kind) {
    case GroupCommand::Kind::InvalidData:
        // A DPT 9 group write carrying 7FFFh is the sender saying it has no
        // value — never a value to act on.
        logInvalidData(label);
        return false;
    case GroupCommand::Kind::Ignore:
        return false;
    case GroupCommand::Kind::Apply:
        break;
    }

    const esp_err_t err = control_state_write(command.command, command.value);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "%s write %.2f rejected: %s", label, static_cast<double>(command.value),
                 esp_err_to_name(err));
        return false;
    }
    return true;
}

template <HabinariPort Port, typename T>
bool writeCommand(T value)
{
    return submit(groupCommand<Port>(value), CommandPort<Port>::label);
}

// A command whose effect is worth a log line: the value the control state
// settled on, which the configured limits may have clamped.
template <HabinariPort Port, typename ReadT, typename LogT>
void writeCommandAndReport(float value, ReadT current, float minChange, LogT report)
{
    const auto read = [&current] {
        LockGuard lock(g_state.mutex);
        return current();
    };
    const GroupCommand command = groupCommand<Port>(value);
    if (command.kind != GroupCommand::Kind::Apply) {
        (void)submit(command, CommandPort<Port>::label);
        return;
    }
    const float before = read();
    if (submit(command, CommandPort<Port>::label)) {
        const float after = read();
        if (std::fabs(after - before) >= minChange) {
            report(after);
        }
    }
}

template <HabinariPort Port>
void writeNeighbourReading(float value, const char *label)
{
    bool usable = false;
    {
        LockGuard lock(g_state.mutex);
        usable = storeNeighbourReading<Port>(g_state.in, value);
    }
    if (!usable) {
        logInvalidData(label);
    }
}

}  // namespace

void bindGroupObjectWrites(HabinariBindings &bindings)
{
    bindings
        // ---- Condensation protection (FB DPS) ----
        .onStateWrite<HabinariPort::DewPointStatusInput>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.externalDewPointAlarm = value;
        })

        // ---- Neighbour-device inputs ----
        // A neighbour that reports invalid data has no reading, so its last
        // one stops being used rather than being trusted indefinitely.
        .onStateWrite<HabinariPort::OutsideTemperature>([](float value) {
            writeNeighbourReading<HabinariPort::OutsideTemperature>(value, "outside temperature");
        })
        .onStateWrite<HabinariPort::OutsideHumidity>([](float value) {
            writeNeighbourReading<HabinariPort::OutsideHumidity>(value, "outside humidity");
        })
        .onStateWrite<HabinariPort::FlowTemperature>([](float value) {
            writeNeighbourReading<HabinariPort::FlowTemperature>(value, "flow temperature");
        })

        // ---- Mode and setpoints (FB RTSM) ----
        .onStateWrite<HabinariPort::ControllerOnOff>([](bool value) {
            writeCommand<HabinariPort::ControllerOnOff>(value);
        })
        .onStateWrite<HabinariPort::HvacMode>([](Dpt20Mode value) {
            writeCommand<HabinariPort::HvacMode>(value);
        })
        .onStateWrite<HabinariPort::ContrMode>([](uint8_t value) {
            writeCommand<HabinariPort::ContrMode>(value);
        })
        // Base setpoint write from an HMI or Home Assistant's
        // target_temperature: this is the comfort heating anchor of the KNX
        // setpoint ladder, so writing it moves standby/economy/cooling with it.
        .onStateWrite<HabinariPort::SetpointBase>([](float value) {
            writeCommandAndReport<HabinariPort::SetpointBase>(
                value, [] { return g_state.hvac.comfortHeatingSetpointC; }, 0.01f,
                [](float after) {
                    ESP_LOGI(TAG, "Base (comfort heating) setpoint updated to %.2f C", after);
                });
        })
        .onStateWrite<HabinariPort::SetpointShift>([](float value) {
            writeCommand<HabinariPort::SetpointShift>(value);
        })

        // ---- Room inputs (FB WOS / PRD / WCOS) ----
        .onStateWrite<HabinariPort::WindowStatus>([](bool value) {
            writeCommand<HabinariPort::WindowStatus>(value);
        })
        .onStateWrite<HabinariPort::PresenceStatus>([](bool value) {
            writeCommand<HabinariPort::PresenceStatus>(value);
        })
        .onStateWrite<HabinariPort::SwitchHeat>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.switchHeat = value;
        })
        .onStateWrite<HabinariPort::SwitchCool>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.switchCool = value;
        })
        .onStateWrite<HabinariPort::ChangeOverStatus>([](bool value) {
            LockGuard lock(g_state.mutex);
            g_state.in.changeOverStatus = value;
        })

        // ---- Ventilation / air quality ----
        .onStateWrite<HabinariPort::Co2Setpoint>([](float value) {
            writeCommandAndReport<HabinariPort::Co2Setpoint>(
                value, [] { return g_state.hvac.ventilationSetpointPpm; }, 0.5f,
                [](float after) { ESP_LOGI(TAG, "CO2 setpoint updated to %.0f ppm", after); });
        })
        .onStateWrite<HabinariPort::VentilationMode>([](uint8_t value) {
            writeCommand<HabinariPort::VentilationMode>(value);
        })

        // ---- Derived events ----
        // Acknowledging clears the latched fire alarm. Handled on the control
        // tick rather than here so the acknowledge and the detector state
        // change happen in one place, in the task that owns the detector.
        .onStateWrite<HabinariPort::AlarmAcknowledge>([](bool value) {
            writeCommand<HabinariPort::AlarmAcknowledge>(value);
        });
}

}  // namespace habinari_knx::service
