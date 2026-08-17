// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include "control_core.hpp"
#include "control_service.h"
#include "sensor_data.h"
#include "sensor_fusion_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @file control_service.hpp
 * @brief In-tree C++ view of the control service: the shared state and its lock.
 *
 * control_state.h is the narrow, flattened C contract every adapter *could*
 * use. This is the wide one, for adapters written in C++ that live in this
 * repository and want the typed Settings/Inputs/Outputs structs rather than a
 * copy flattened into floats — the KNX adapter has ~100 parameter callbacks,
 * and routing each through a float-valued command API would be neither faster
 * nor clearer.
 *
 * Threading contract, which is the whole point of this header:
 *
 *   - Every field of ServiceState is guarded by ServiceState::mutex.
 *   - Take it with LockGuard, never by hand.
 *   - Hold it for assignments and snapshot copies only. Never call into a
 *     protocol stack, the fusion layer, NVS or a control loop while holding it.
 *   - `out` is written only by the control task, and only at the end of a tick,
 *     so a reader under the lock always sees one coherent cycle's results —
 *     never a half-updated mixture of this tick and the last.
 */

namespace habinari {
namespace control {

/// Everything the device shares between the control task and its adapters.
/// One instance, reached through state().
struct ServiceState {
    SemaphoreHandle_t mutex{nullptr};

    // --- Measurements in ----------------------------------------------------
    sensor_data_t latestSensorData{};
    bool hasSensorData{false};  ///< at least one sampling cycle has completed

    // --- The control model --------------------------------------------------
    Settings hvac{};  ///< configured tuning, whatever front end configured it
    Inputs in{};      ///< what the installation is telling us
    Outputs out{};    ///< what the last tick decided

    // Tuning for the acquisition-side fusion layer. Held here so configuration
    // front ends have one place to write to; pushed into sensor_fusion_configure()
    // by the control task when it changes, because that call takes its own lock
    // and must not happen under ours.
    sensor_fusion_config_t fusion{};
    bool fusionConfigDirty{true};
    bool acknowledgeAlarmsRequested{false};

    // --- Programming mode -----------------------------------------------------
    // The board's one "this device is selected" state, and the only gate any
    // commissioning path has: KNX individual addressing, BLE pairing and Modbus
    // re-addressing all wait on it. The PROG button raises the request; the KNX
    // adapter claims it if present (its stack owns the flag and only its task
    // may touch it), and otherwise the control task acts on it directly — which
    // is why a Modbus-only build blinks at all.
    bool programmingModeActive{false};
    bool programmingModeToggleRequested{false};
    /// Tick count at which programming mode lapses, or 0 for no deadline. A
    /// device left selected is a device anybody can re-address or pair with, so
    /// it is not left selected.
    TickType_t programmingModeDeadline{0};
    control_programming_mode_callback_t programmingModeCallback{nullptr};

    /// Incremented once per completed control tick.
    uint32_t generation{0};
};

/// The one instance. Valid before control_service_start(); `mutex` is null until
/// then, which every accessor here treats as "not running yet".
ServiceState &state();

/// RAII lock. Null-safe so pre-start callers degrade to a no-op rather than
/// crashing during early init.
class LockGuard {
public:
    explicit LockGuard(SemaphoreHandle_t mutex) : mutex_(mutex)
    {
        if (mutex_ != nullptr) {
            xSemaphoreTake(mutex_, portMAX_DELAY);
        }
    }

    ~LockGuard()
    {
        if (mutex_ != nullptr) {
            xSemaphoreGive(mutex_);
        }
    }

    LockGuard(const LockGuard &) = delete;
    LockGuard &operator=(const LockGuard &) = delete;

private:
    SemaphoreHandle_t mutex_;
};

/// Consume a pending programming-mode toggle request, if any. Adapters call
/// this from their own loop; it clears the flag, so exactly one adapter acts
/// on a press.
bool takeProgrammingModeToggleRequest();

}  // namespace control
}  // namespace habinari
