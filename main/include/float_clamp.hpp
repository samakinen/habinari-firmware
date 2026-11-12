// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <cmath>

namespace habinari {

// NaN fails both comparisons, so without the first test it would come straight
// back out and into a PID integrator, which never recovers from it, or into a
// float-to-integer cast, which is undefined for NaN. The low bound is the safe
// answer for every caller here: no demand, lowest setpoint, no occupants.
// control_state_write() refuses non-finite commands; this is the last line of
// defence behind it, not the gate.
inline float clampf(float value, float lo, float hi)
{
    if (std::isnan(value)) {
        return lo;
    }
    return value < lo ? lo : (value > hi ? hi : value);
}

} // namespace habinari
