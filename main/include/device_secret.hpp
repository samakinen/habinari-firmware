// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

/**
 * @file device_secret.hpp
 * @brief Per-device root secret in eFuse, and the KNX FDSK derived from it.
 *
 * The KNX Data Secure FDSK must be unique per device, unpredictable, and
 * survive a factory reset — while the same firmware image ships to every
 * board. Storing it in NVS satisfies none of the first three (a factory reset
 * erases it, so the printed device certificate stops matching the device), and
 * deriving it from the MAC/serial satisfies uniqueness only: both are public,
 * so anyone holding the firmware could recompute the fleet's keys.
 *
 * So the device keeps a 256-bit root secret in a read-protected eFuse key
 * block with purpose HMAC_UP. Software can never read it back; it can only ask
 * the HMAC peripheral to compute over it. The FDSK is derived on every boot:
 *
 *     FDSK = HMAC-SHA256(root, "KNstaX/KNX-FDSK/v1" || serial[6])[0..15]
 *
 * That survives nvs_flash_erase(), a full `erase-flash`, a reflash and OTA, so
 * a certificate label printed at manufacture stays valid for the life of the
 * chip. The label prefix is domain separation: the same root can derive
 * unrelated future secrets without exposing this one.
 *
 * This header holds the portable half — the KDF and the entropy gate — so both
 * are host-testable (see main/test). The eFuse/HMAC-peripheral half is
 * declared at the bottom and implemented in device_secret_esp.cpp.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace habinari::secret {

inline constexpr size_t kRootSecretBytes = 32;
inline constexpr size_t kFdskBytes = 16;
inline constexpr size_t kSerialBytes = 6;

using RootSecret = std::array<uint8_t, kRootSecretBytes>;
using Fdsk = std::array<uint8_t, kFdskBytes>;
using Digest = std::array<uint8_t, 32>;
using Serial = std::span<const uint8_t, kSerialBytes>;

// Domain-separation label, without its NUL. Changing this string changes every
// device's FDSK and invalidates every printed certificate — hence the version
// suffix rather than an edit in place.
inline constexpr char kFdskLabel[] = "KNstaX/KNX-FDSK/v1";
inline constexpr size_t kFdskLabelLen = sizeof(kFdskLabel) - 1;
inline constexpr size_t kFdskMessageBytes = kFdskLabelLen + kSerialBytes;

// The BLE out-of-band channel's pairing passkey, derived from the same root
// under a different label. That is the whole point of domain separation: the
// device needs a second printable per-device credential with exactly the
// properties the FDSK has — unique, unpredictable from public data, surviving a
// factory reset — and the label makes it impossible for one to reveal the
// other. A device that speaks both would print two independent values.
inline constexpr char kBlePasskeyLabel[] = "habinari/BLE-passkey/v1";
inline constexpr size_t kBlePasskeyLabelLen = sizeof(kBlePasskeyLabel) - 1;
inline constexpr size_t kBlePasskeyMessageBytes = kBlePasskeyLabelLen + kSerialBytes;
/// Bluetooth passkey entry is a six-digit decimal number, 000000..999999.
inline constexpr uint32_t kBlePasskeyModulus = 1000000;

// --- Entropy gate ---------------------------------------------------------
// Burning an eFuse is irreversible, so the sample the key is folded from is
// measured first and the burn is refused if the source looks dead. These are
// coarse liveness gates, not a randomness test suite: each threshold sits many
// standard deviations away from what a healthy source produces, so a false
// refusal is far less likely than the hardware fault it is looking for.
struct EntropyQuality {
    uint32_t sampledBytes{0};       ///< Bytes measured (context bytes excluded).
    uint32_t sampledWords{0};       ///< Whole 32-bit words within those bytes.
    uint32_t repeatedWords{0};      ///< Words identical to their predecessor.
    uint32_t oneBits{0};            ///< Population count over the sample.
    uint32_t distinctByteValues{0}; ///< Distinct byte values seen, 0..256.
    uint32_t orAll{0};              ///< OR of every word: 0 means stuck low.
    uint32_t andAll{0xFFFFFFFFu};   ///< AND of every word: ~0 means stuck high.
    bool constantWords{false};      ///< Every word carried the same value.

    /// Minimum sample the gates are calibrated for (8 x 128 B in practice).
    static constexpr uint32_t kMinSampleBytes = 1024;
    /// A healthy 1 KB sample shows ~250 of the 256 byte values.
    static constexpr uint32_t kMinDistinctByteValues = 200;
    /// Adjacent-word repeats are a 2^-32 coincidence; allow a wide margin and
    /// still catch a source that mostly repeats itself.
    static constexpr uint32_t kMaxRepeatedWordsNumerator = 1;
    static constexpr uint32_t kMaxRepeatedWordsDenominator = 8;
    /// Bit balance must be within 40..60 % — about 18 sigma on 8192 bits.
    static constexpr uint32_t kMinOneBitsPercent = 40;
    static constexpr uint32_t kMaxOneBitsPercent = 60;

    /// True when the sample is large enough and shows no sign of a dead source.
    bool sane() const;
};

/**
 * @brief Accumulates entropy into a 256-bit root secret and measures it.
 *
 * Everything added is folded through SHA-256, so the result is no weaker than
 * the strongest input. Two kinds of input are distinguished:
 *
 *  - addEntropy(): bytes from the hardware RNG. Hashed *and* measured.
 *  - addContext(): non-secret device data (MAC, timestamps). Hashed only, and
 *    deliberately not counted as entropy — the MAC is public and contributes
 *    no unpredictability. It is folded in purely so that a catastrophic RNG
 *    failure yields distinct-per-device keys rather than one fleet-wide key.
 */
class EntropyPool {
public:
    EntropyPool();
    ~EntropyPool();

    EntropyPool(const EntropyPool &) = delete;
    EntropyPool &operator=(const EntropyPool &) = delete;

    /// Fold in hardware-RNG bytes and update the quality statistics.
    void addEntropy(std::span<const uint8_t> bytes);
    /// Fold in non-secret device data; contributes uniqueness, never entropy.
    void addContext(std::span<const uint8_t> bytes);

    const EntropyQuality &quality() const { return _quality; }

    /// Produce the root secret. Returns false if the digest fails or if the
    /// pool was never fed. Callers must still check quality().sane().
    bool finish(RootSecret &out);

private:
    void *_ctx{nullptr};                   ///< Streaming SHA-256, type-erased.
    EntropyQuality _quality{};
    bool _wordCarry{false};                ///< A partial word straddling two adds.
    uint32_t _partialWord{0};
    uint32_t _partialBytes{0};
    uint32_t _previousWord{0};
    bool _havePreviousWord{false};
    bool _firstWordSeen{false};
    uint32_t _firstWord{0};
    bool _byteSeen[256]{};
    bool _finished{false};
};

// --- Key derivation -------------------------------------------------------

/// Standard RFC 2104 HMAC-SHA256, the same function the HMAC peripheral
/// implements in upstream mode. Exposed so tests can pin it to RFC 4231.
bool hmacSha256(std::span<const uint8_t> key, std::span<const uint8_t> message, Digest &out);

/// Build the KDF message: label || serial. Shared by the software and
/// peripheral paths so the two cannot drift apart.
void buildFdskMessage(Serial serial, std::array<uint8_t, kFdskMessageBytes> &out);

/// Truncate an HMAC-SHA256 digest to the 128-bit FDSK.
void fdskFromDigest(const Digest &digest, Fdsk &out);

/// Reference (software) derivation, used for the dry run and by the burn-time
/// cross-check against the peripheral.
bool deriveFdsk(const RootSecret &root, Serial serial, Fdsk &out);

/// Build the BLE passkey KDF message: label || serial.
void buildBlePasskeyMessage(Serial serial, std::array<uint8_t, kBlePasskeyMessageBytes> &out);

/**
 * @brief Fold an HMAC-SHA256 digest into a six-digit passkey.
 *
 * The first four bytes, big-endian, modulo 10^6. The modulo bias is about one
 * part in 4300 across the range — far below anything that matters against a
 * space of a million that an attacker must guess online, one interactive
 * pairing attempt at a time, inside a commissioning window.
 */
uint32_t blePasskeyFromDigest(const Digest &digest);

/// Reference (software) derivation of the BLE pairing passkey.
bool deriveBlePasskey(const RootSecret &root, Serial serial, uint32_t &out);

// --- Platform layer (device_secret_esp.cpp) -------------------------------

enum class RootSecretState {
    DerivedFromEfuse,   ///< The block holds an HMAC_UP key; FDSK is authoritative.
    DryRunCandidate,    ///< Burning disabled: a candidate was generated and logged only.
    BurnFailed,         ///< Provisioning was attempted and the eFuse write failed.
    EntropyRejected,    ///< The RNG sample failed the sanity gate; nothing was burned.
    BlockUnavailable,   ///< The configured key block is used for another purpose.
    DerivationFailed,   ///< The block is provisioned but the HMAC call failed.
};

struct RootSecretResult {
    RootSecretState state{RootSecretState::DerivationFailed};
    Fdsk fdsk{};
    /// True only when `fdsk` is the value this device will keep using, i.e.
    /// when it came from the eFuse-backed HMAC peripheral.
    bool fdskValid{false};
};

struct BlePasskeyResult {
    uint32_t passkey{0};
    /// True when the passkey came from the eFuse-backed HMAC peripheral, i.e.
    /// when it is the value printed on this device's label. False means the
    /// root secret has not been provisioned and the caller is looking at a
    /// build-time fallback that is identical on every unprovisioned board.
    bool fromEfuse{false};
};

/**
 * @brief Resolve the BLE pairing passkey for this device.
 *
 * Reads the eFuse key block and derives through the HMAC peripheral, exactly as
 * resolveFdsk() does, but never provisions: burning a fuse is a manufacturing
 * step and must not be a side effect of bringing a service channel up. On an
 * unprovisioned device this reports fromEfuse = false and a fixed development
 * passkey, which the service channel logs loudly.
 */
BlePasskeyResult resolveBlePasskey(Serial serial);

/**
 * @brief Resolve the device FDSK, provisioning the root secret if needed.
 *
 * Reads the configured eFuse key block. If it already carries an HMAC_UP key,
 * the FDSK is derived through the peripheral and returned. If the block is
 * unused, a candidate root secret is gathered and gated; it is then burned
 * (and the derivation cross-checked against software HMAC) when
 * CONFIG_HABINARI_ROOT_SECRET_BURN is set, or merely logged when it is
 * not. Logs its own detail; the caller only needs the result.
 */
RootSecretResult resolveFdsk(Serial serial);

} // namespace habinari::secret
