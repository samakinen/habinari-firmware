// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// ETS parameters into the control state: the load at start-up and the reload
// after an ETS download, both driven by kParameterTable
// (knx_parameter_table.hpp), so the two can never map a parameter differently.

#include "knx_service_internal.hpp"

#include "control_service.hpp"
#include "knx_parameter_table.hpp"
#include "sensor_fusion_service.h"

namespace habinari_knx::service {

namespace {

using habinari::control::LockGuard;

}  // namespace

void bindParameters(HabinariBindings &bindings)
{
    // After a completed ETS download every row is applied again, with the
    // control mutex held, exactly as at start-up.
    bindings.onParametersChanged([](knx::product::params::ViewFor<kHabinariParameters> parameters) {
        LockGuard lock(g_state.mutex);
        loadParameters(parameters.block<HabinariParameter>(), g_state);
    });
}

void seedStateFromParameters(HabinariApp &app)
{
    LockGuard lock(g_state.mutex);
    // Seed engineering values from the ETS parameters: the same table the
    // download callback uses, applied once. Fusion tuning starts
    // from the compiled-in defaults so the fields the ETS product does not
    // expose (staleness, inter-sensor offsets) are populated, and the
    // table then overlays the ones it does.
    sensor_fusion_default_config(&g_state.fusion);
    loadParameters(app.parameters().block<HabinariParameter>(), g_state);
    g_state.fusionConfigDirty = true;
    auto &s = g_state.hvac;
    // Seed the runtime mode/state inputs from their ETS defaults.
    g_state.in.controllerOnOff = s.controllerDefaultEnable;
    g_state.in.hvacOperatingMode = s.defaultHvacOperatingMode;
    g_state.in.controllerMode = s.defaultControllerMode;
}

}  // namespace habinari_knx::service
