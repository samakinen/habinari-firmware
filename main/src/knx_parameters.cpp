// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// ETS parameters into the control state: the live onParameterChanged bindings
// and the one-time load at start-up, both driven by kParameterTable
// (knx_parameter_table.hpp), so the two can never map a parameter differently.

#include "knx_service_internal.hpp"

#include "control_service.hpp"
#include "knx_parameter_table.hpp"
#include "sensor_fusion_service.h"

#include <tuple>

namespace habinari_knx::service {

namespace {

using habinari::control::LockGuard;

template <HabinariParameter Id, typename Apply>
void bindParameter(HabinariBindings &bindings, const ParameterRow<Id, Apply> &)
{
    bindings.template onParameterChanged<Id>([](ParameterValue<Id> value) {
        LockGuard lock(g_state.mutex);
        Apply{}(g_state, value);
    });
}

}  // namespace

void bindParameters(HabinariBindings &bindings)
{
    std::apply([&bindings](const auto &...rows) { (bindParameter(bindings, rows), ...); },
               kParameterTable);
}

void seedStateFromParameters(HabinariApp &app)
{
    LockGuard lock(g_state.mutex);
    // Seed engineering values from the ETS parameters: the same table the
    // onParameterChanged bindings use, applied once. Fusion tuning starts
    // from the compiled-in defaults so the fields the ETS product does not
    // expose (staleness, inter-sensor offsets) are populated, and the
    // table then overlays the ones it does.
    sensor_fusion_default_config(&g_state.fusion);
    loadParameters(app.parameters(), g_state);
    g_state.fusionConfigDirty = true;
    auto &s = g_state.hvac;
    // Seed the runtime mode/state inputs from their ETS defaults.
    g_state.in.controllerOnOff = s.controllerDefaultEnable;
    g_state.in.hvacOperatingMode = s.defaultHvacOperatingMode;
    g_state.in.controllerMode = s.defaultControllerMode;
}

}  // namespace habinari_knx::service
