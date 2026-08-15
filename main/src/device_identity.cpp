// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * @file device_identity.cpp
 * @brief Hex / base32 / CRC-4 rendering of the device's KNX identity.
 *
 * Moved verbatim out of knx_service.cpp; see device_identity.hpp for the
 * encoding rationale. Kept free of ESP-IDF dependencies so main/test can pin
 * the certificate encoding to fixed vectors.
 */

#include "device_identity.hpp"

#include <algorithm>
#include <cstdio>

namespace habinari::identity {

void toHex(std::span<const uint8_t> bytes, char sep, char *out, size_t outLen)
{
    size_t pos = 0;
    for (size_t i = 0; i < bytes.size() && pos + 3 < outLen; ++i) {
        if (i != 0 && sep != '\0' && pos + 1 < outLen) {
            out[pos++] = sep;
        }
        pos += static_cast<size_t>(std::snprintf(out + pos, outLen - pos, "%02X", bytes[i]));
    }
    if (outLen > 0) {
        out[(pos < outLen) ? pos : (outLen - 1)] = '\0';
    }
}

uint8_t crc4(std::span<const uint8_t> bytes)
{
    static constexpr uint8_t kTable[16] = {0x0, 0x3, 0x6, 0x5, 0xC, 0xF, 0xA, 0x9,
                                           0xB, 0x8, 0xD, 0xE, 0x7, 0x4, 0x1, 0x2};
    uint8_t crc = 0;
    for (const uint8_t byte : bytes) {
        crc = kTable[crc ^ (byte >> 4)];
        crc = kTable[crc ^ (byte & 0x0Fu)];
    }
    return crc;
}

size_t toBase32(std::span<const uint8_t> bytes, char *out, size_t outLen)
{
    static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ234567";
    uint32_t buffer = 0;
    int bits = 0;
    size_t pos = 0;
    for (const uint8_t byte : bytes) {
        buffer = (buffer << 8) | byte;
        bits += 8;
        while (bits >= 5 && pos + 1 < outLen) {
            bits -= 5;
            out[pos++] = kAlphabet[(buffer >> bits) & 0x1Fu];
        }
    }
    // Trailing bits are left-aligned into a final character (23 bytes -> 37
    // characters, so this always fires for a device certificate).
    if (bits > 0 && pos + 1 < outLen) {
        out[pos++] = kAlphabet[(buffer << (5 - bits)) & 0x1Fu];
    }
    if (outLen > 0) {
        out[pos] = '\0';
    }
    return pos;
}

void formatDeviceCertificate(std::span<const uint8_t, 6> serial,
                             const std::array<uint8_t, 16> &key,
                             char *out,
                             size_t outLen)
{
    std::array<uint8_t, 23> payload{};
    std::copy(serial.begin(), serial.end(), payload.begin());
    std::copy(key.begin(), key.end(), payload.begin() + serial.size());
    payload.back() = static_cast<uint8_t>(
        crc4(std::span<const uint8_t>(payload.data(), payload.size() - 1)) << 4);

    char raw[40] = {};
    size_t rawLen = toBase32(std::span<const uint8_t>(payload.data(), payload.size()),
                             raw, sizeof(raw));
    if (rawLen > kCertificateChars) {
        rawLen = kCertificateChars;
    }

    size_t pos = 0;
    for (size_t i = 0; i < rawLen && pos + 1 < outLen; ++i) {
        if (i != 0 && (i % 6) == 0 && pos + 1 < outLen) {
            out[pos++] = '-';
        }
        out[pos++] = raw[i];
    }
    if (outLen > 0) {
        out[pos] = '\0';
    }
}

} // namespace habinari::identity
