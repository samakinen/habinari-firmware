// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <cstdint>

/**
 * @file knx_publish_schedule.hpp
 * @brief When the KNX service loop offers the device's state to the bus.
 *
 * Three rules, decided here and acted on by the service loop:
 *
 *   - Publishing is offered once per control tick, not once per loop pass: the
 *     loop also spins at a few milliseconds while the stack has in-flight work,
 *     and re-offering every object that often would burn CPU on suppression
 *     decisions that cannot have changed.
 *   - Boot-storm mitigation: nothing is published until a per-device random
 *     release tick, so a bus-wide power-up does not make every device transmit
 *     at t=0. Sensor updates keep accumulating meanwhile and go out together
 *     once the gate opens.
 *   - Becoming Operational means the association table is now the project's,
 *     so that is when "which objects did ETS actually link?" is answered: on
 *     every boot of a commissioned device, and on every return to Operational
 *     — whether programming mode was left from ETS or with the button.
 *
 * Nothing is published, and nothing reported, outside the Operational state.
 *
 * Platform-free so the gating can be driven tick by tick on the host. Ticks are
 * the FreeRTOS tick count, compared with wrap-around arithmetic.
 */

namespace habinari_knx {

class PublishSchedule {
public:
    /// What the loop should do on this pass, in this order.
    struct Pass {
        bool reportUnlinkedPorts{false};  ///< the device has just become Operational
        bool announceHold{false};         ///< first pass held back by the startup gate
        bool publish{false};              ///< offer every transmitting port now
    };

    /// @p releaseTick: the first tick at which publishing may start.
    constexpr explicit PublishSchedule(uint32_t releaseTick) : releaseTick_(releaseTick) {}

    /// A control tick has completed: the next open pass publishes.
    constexpr void controlTick() { due_ = true; }

    /// Decide this pass, given the tick count and whether the device is
    /// Operational — read once, at the top of the pass, before the loop acts
    /// on a programming-mode button press. Every transition is then seen by
    /// the pass after it, whatever caused it: ETS over the bus, the button, a
    /// download. A toggle the loop makes this pass is the next pass's input.
    constexpr Pass begin(uint32_t nowTick, bool operational)
    {
        Pass pass{};
        pass.reportUnlinkedPorts = operational && !wasOperational_;
        wasOperational_ = operational;
        if (!operational) {
            return pass;
        }
        if (!released(nowTick, releaseTick_)) {
            if (!holdAnnounced_) {
                pass.announceHold = true;
                holdAnnounced_ = true;
            }
        } else if (due_) {
            pass.publish = true;
            due_ = false;
        }
        return pass;
    }

    /// True once @p nowTick has reached @p releaseTick, across tick wrap-around.
    static constexpr bool released(uint32_t nowTick, uint32_t releaseTick)
    {
        return static_cast<int32_t>(nowTick - releaseTick) >= 0;
    }

private:
    uint32_t releaseTick_;
    // The lifecycle the previous pass observed. Nothing has been observed
    // before the first pass, which therefore sees a commissioned device's
    // boot as the transition into Operational that it is.
    bool wasOperational_{false};
    bool due_{true};
    bool holdAnnounced_{false};
};

}  // namespace habinari_knx
