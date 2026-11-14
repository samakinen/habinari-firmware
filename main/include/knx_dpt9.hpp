// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

#include <limits>

/**
 * @file knx_dpt9.hpp
 * @brief The DPT 9 "invalid data" code, as it reaches the KNX adapter.
 *
 * 03_07_02 §3.10: "For all Datapoint Types 9.xxx, the encoded value 7FFFh shall
 * always be used to denote invalid data", and the valid range ends at
 * 670 433,28. A neighbouring device whose sensor has failed therefore sends
 * 7FFFh. The stack's codec maps that code to a quiet NaN in both directions:
 * 7FFFh decodes to NaN and NaN encodes to 7FFFh, so a value can never be
 * mistaken for the code (taken at face value, 7FFFh would be a CO2 setpoint of
 * 670 760 ppm or an outdoor temperature hotter than the sun).
 *
 * Kept apart from knx_service.cpp, which needs the stack and FreeRTOS, so the
 * host tests can pin it.
 */

namespace habinari_knx {

/// Top of the DPT 9 range (03_07_02 §3.10), encoded 7FFEh.
inline constexpr float kDpt9MaxValid = 670433.28f;
/// The stack's form of 7FFFh: what it decodes to, and what encodes to it.
inline constexpr float kDpt9InvalidDecoded = std::numeric_limits<float>::quiet_NaN();

/// True if @p decoded is the 7FFFh "invalid data" code rather than a value.
/// NaN fails every comparison, so it lands here. So does anything nearer to
/// 7FFFh read as a number (670 760.96) than to the top of the range, so a
/// decoder that does not map the code to NaN is still caught; the midpoint
/// keeps rounding from putting 7FFEh on the wrong side.
constexpr bool dpt9IsInvalid(float decoded)
{
    return !(decoded <= (kDpt9MaxValid + 670760.96f) / 2.0f);
}

}  // namespace habinari_knx
