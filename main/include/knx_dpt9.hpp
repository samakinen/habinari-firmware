// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

/**
 * @file knx_dpt9.hpp
 * @brief The DPT 9 "invalid data" code, as it reaches the KNX adapter.
 *
 * 03_07_02 §3.10: "For all Datapoint Types 9.xxx, the encoded value 7FFFh shall
 * always be used to denote invalid data", and the valid range ends at
 * 670 433,28. A neighbouring device whose sensor has failed therefore sends
 * 7FFFh — and the stack's codec decodes it like any other code, to 670 760.96.
 * Taken at face value that is a CO2 setpoint of 670 760 ppm or an outdoor
 * temperature hotter than the sun.
 *
 * Kept apart from knx_service.cpp, which needs the stack and FreeRTOS, so the
 * host tests can pin it.
 */

namespace habinari_knx {

/// Top of the DPT 9 range (03_07_02 §3.10), encoded 7FFEh.
inline constexpr float kDpt9MaxValid = 670433.28f;
/// What 7FFFh decodes to: (0.01 × 2047) × 2^15.
inline constexpr float kDpt9InvalidDecoded = 670760.96f;

/// True if @p decoded is the 7FFFh "invalid data" code rather than a value.
/// Compared against the midpoint so that neither the decode's rounding nor
/// these constants' can put a value on the wrong side. NaN, which no DPT 9 code
/// decodes to, counts as invalid too.
constexpr bool dpt9IsInvalid(float decoded)
{
    return !(decoded <= (kDpt9MaxValid + kDpt9InvalidDecoded) / 2.0f);
}

}  // namespace habinari_knx
