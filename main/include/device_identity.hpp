// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

/**
 * @file device_identity.hpp
 * @brief Rendering of the device's KNX identity: hex dumps and the ETS device
 *        certificate (serial number + FDSK, base32 with a CRC-4).
 *
 * Split out of knx_service.cpp so the root-secret dry run can print exactly
 * the certificate the device will present once its eFuse is burned, and so the
 * encoding is covered by the host tests in main/test.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace habinari::identity {

/// Characters in a formatted certificate, excluding the five separators.
inline constexpr size_t kCertificateChars = 36;
/// Buffer size that always holds a formatted certificate plus its NUL.
inline constexpr size_t kCertificateBufferSize = kCertificateChars + 5 + 1;

/// Render bytes as uppercase hex into caller storage, optionally separated.
void toHex(std::span<const uint8_t> bytes, char sep, char *out, size_t outLen);

/**
 * @brief CRC-4 (generator x^4 + x + 1, "10011") over a byte span.
 *
 * Folded one nibble at a time. This is the check value ETS expects in the
 * trailing bits of a printed device certificate; without it the certificate is
 * rejected as mistyped.
 */
uint8_t crc4(std::span<const uint8_t> bytes);

/// Render bytes as RFC 4648 base32 (uppercase A-Z2-7, unpadded). Returns the
/// number of characters written.
size_t toBase32(std::span<const uint8_t> bytes, char *out, size_t outLen);

/**
 * @brief Format the ETS device certificate: base32(serial[6] || key[16] || crc4).
 *
 * Printed as the six dash-separated groups of six that ETS's "Add device
 * certificate" dialog expects. ETS decodes it back into the serial number and
 * the FDSK, which it then uses as the initial tool key.
 *
 * The 176 payload bits do not fill a whole number of base32 characters: the
 * 36th character carries the last payload bit plus the four CRC-4 bits over
 * the 22 payload bytes. Encoding the CRC as a 23rd byte (high nibble) lines
 * those bits up, and the 37th character it produces is pad-only and dropped.
 */
void formatDeviceCertificate(std::span<const uint8_t, 6> serial,
                             const std::array<uint8_t, 16> &key,
                             char *out,
                             size_t outLen);

} // namespace habinari::identity
