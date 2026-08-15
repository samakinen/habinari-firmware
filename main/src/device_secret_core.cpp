// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * @file device_secret_core.cpp
 * @brief Portable half of the device root secret: entropy pooling and the KDF.
 *
 * Deliberately free of ESP-IDF dependencies so the host tests in main/test can
 * exercise the entropy gate and pin the KDF to fixed vectors. The eFuse and
 * HMAC-peripheral half lives in device_secret_esp.cpp.
 */

#include "device_secret.hpp"

#include "mbedtls/md.h"
#include "mbedtls/platform_util.h"

#include <bit>
#include <cstring>

namespace habinari::secret {

namespace {

constexpr size_t kSha256BlockBytes = 64;

/**
 * @brief Streaming SHA-256 over the generic message-digest API.
 *
 * mbedtls_sha256_* changed its signatures between mbedTLS 2.x (host) and 3.x,
 * and mbedTLS 4.x (ESP-IDF) moved the MD-level HMAC helpers behind
 * MBEDTLS_DECLARE_PRIVATE_IDENTIFIERS. What is public and identical in all
 * three is plain digest streaming, so that is all this file uses — HMAC is
 * composed from it below.
 */
class Sha256Stream {
public:
    Sha256Stream()
    {
        const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        if (info == nullptr) {
            return;
        }
        mbedtls_md_init(&_ctx);
        _allocated = true;
        _ok = mbedtls_md_setup(&_ctx, info, 0) == 0 && mbedtls_md_starts(&_ctx) == 0;
    }

    ~Sha256Stream()
    {
        if (_allocated) {
            mbedtls_md_free(&_ctx);
        }
    }

    Sha256Stream(const Sha256Stream &) = delete;
    Sha256Stream &operator=(const Sha256Stream &) = delete;

    bool valid() const { return _ok; }

    bool update(std::span<const uint8_t> bytes)
    {
        if (!_ok) {
            return false;
        }
        if (bytes.empty()) {
            return true;
        }
        _ok = mbedtls_md_update(&_ctx, bytes.data(), bytes.size()) == 0;
        return _ok;
    }

    bool finish(Digest &out)
    {
        if (!_ok) {
            return false;
        }
        _ok = false; // single shot
        return mbedtls_md_finish(&_ctx, out.data()) == 0;
    }

private:
    mbedtls_md_context_t _ctx{};
    bool _allocated{false};
    bool _ok{false};
};

Sha256Stream *asStream(void *ctx)
{
    return static_cast<Sha256Stream *>(ctx);
}

/// SHA-256 over up to two concatenated spans.
bool sha256(std::span<const uint8_t> first, std::span<const uint8_t> second, Digest &out)
{
    Sha256Stream stream;
    return stream.update(first) && stream.update(second) && stream.finish(out);
}

} // namespace

bool EntropyQuality::sane() const
{
    if (sampledBytes < kMinSampleBytes || sampledWords == 0) {
        return false;
    }
    if (orAll == 0 || andAll == 0xFFFFFFFFu || constantWords) {
        return false;
    }
    if (repeatedWords * kMaxRepeatedWordsDenominator >
        sampledWords * kMaxRepeatedWordsNumerator) {
        return false;
    }
    if (distinctByteValues < kMinDistinctByteValues) {
        return false;
    }
    const uint64_t totalBits = static_cast<uint64_t>(sampledBytes) * 8u;
    const uint64_t ones = static_cast<uint64_t>(oneBits) * 100u;
    if (ones < totalBits * kMinOneBitsPercent || ones > totalBits * kMaxOneBitsPercent) {
        return false;
    }
    return true;
}

EntropyPool::EntropyPool()
{
    auto *stream = new Sha256Stream();
    if (!stream->valid()) {
        delete stream;
        return;
    }
    _ctx = stream;
}

EntropyPool::~EntropyPool()
{
    delete asStream(_ctx);
    _ctx = nullptr;
    mbedtls_platform_zeroize(&_partialWord, sizeof(_partialWord));
    mbedtls_platform_zeroize(&_previousWord, sizeof(_previousWord));
    mbedtls_platform_zeroize(&_firstWord, sizeof(_firstWord));
}

void EntropyPool::addContext(std::span<const uint8_t> bytes)
{
    if (_ctx == nullptr || _finished || bytes.empty()) {
        return;
    }
    asStream(_ctx)->update(bytes);
}

void EntropyPool::addEntropy(std::span<const uint8_t> bytes)
{
    if (_ctx == nullptr || _finished || bytes.empty()) {
        return;
    }
    asStream(_ctx)->update(bytes);

    for (const uint8_t byte : bytes) {
        _quality.sampledBytes++;
        _quality.oneBits += static_cast<uint32_t>(std::popcount(byte));
        if (!_byteSeen[byte]) {
            _byteSeen[byte] = true;
            _quality.distinctByteValues++;
        }

        // Word statistics carry across calls, so a sample split into chunks
        // measures the same as one taken in a single read.
        _partialWord = (_partialWord << 8) | byte;
        if (++_partialBytes < 4) {
            continue;
        }
        const uint32_t word = _partialWord;
        _partialWord = 0;
        _partialBytes = 0;

        _quality.sampledWords++;
        _quality.orAll |= word;
        _quality.andAll &= word;
        if (_havePreviousWord && word == _previousWord) {
            _quality.repeatedWords++;
        }
        _previousWord = word;
        _havePreviousWord = true;
        if (!_firstWordSeen) {
            _firstWordSeen = true;
            _firstWord = word;
            _quality.constantWords = true;
        } else if (word != _firstWord) {
            _quality.constantWords = false;
        }
    }
}

bool EntropyPool::finish(RootSecret &out)
{
    if (_ctx == nullptr || _finished || _quality.sampledBytes == 0) {
        return false;
    }
    _finished = true;

    Digest digest{};
    if (!asStream(_ctx)->finish(digest)) {
        return false;
    }
    static_assert(kRootSecretBytes == std::tuple_size_v<Digest>,
                  "The root secret is the full SHA-256 digest");
    std::memcpy(out.data(), digest.data(), out.size());
    mbedtls_platform_zeroize(digest.data(), digest.size());
    return true;
}

// RFC 2104 HMAC-SHA256, composed from plain digest streaming. This is the
// function the HMAC peripheral implements in upstream mode, so the software
// derivation used by the dry run and the hardware one used afterwards agree;
// test_device_secret.cpp pins it to the RFC 4231 vectors.
bool hmacSha256(std::span<const uint8_t> key, std::span<const uint8_t> message, Digest &out)
{
    std::array<uint8_t, kSha256BlockBytes> paddedKey{};
    if (key.size() > kSha256BlockBytes) {
        Digest hashedKey{};
        if (!sha256(key, {}, hashedKey)) {
            return false;
        }
        std::memcpy(paddedKey.data(), hashedKey.data(), hashedKey.size());
        mbedtls_platform_zeroize(hashedKey.data(), hashedKey.size());
    } else if (!key.empty()) {
        std::memcpy(paddedKey.data(), key.data(), key.size());
    }

    std::array<uint8_t, kSha256BlockBytes> ipad{};
    std::array<uint8_t, kSha256BlockBytes> opad{};
    for (size_t i = 0; i < kSha256BlockBytes; ++i) {
        ipad[i] = static_cast<uint8_t>(paddedKey[i] ^ 0x36u);
        opad[i] = static_cast<uint8_t>(paddedKey[i] ^ 0x5Cu);
    }
    mbedtls_platform_zeroize(paddedKey.data(), paddedKey.size());

    Digest inner{};
    bool ok = sha256(std::span<const uint8_t>(ipad.data(), ipad.size()), message, inner);
    if (ok) {
        ok = sha256(std::span<const uint8_t>(opad.data(), opad.size()),
                    std::span<const uint8_t>(inner.data(), inner.size()), out);
    }

    mbedtls_platform_zeroize(ipad.data(), ipad.size());
    mbedtls_platform_zeroize(opad.data(), opad.size());
    mbedtls_platform_zeroize(inner.data(), inner.size());
    return ok;
}

void buildFdskMessage(Serial serial, std::array<uint8_t, kFdskMessageBytes> &out)
{
    std::memcpy(out.data(), kFdskLabel, kFdskLabelLen);
    std::memcpy(out.data() + kFdskLabelLen, serial.data(), kSerialBytes);
}

void fdskFromDigest(const Digest &digest, Fdsk &out)
{
    std::memcpy(out.data(), digest.data(), out.size());
}

bool deriveFdsk(const RootSecret &root, Serial serial, Fdsk &out)
{
    std::array<uint8_t, kFdskMessageBytes> message{};
    buildFdskMessage(serial, message);

    Digest digest{};
    if (!hmacSha256(std::span<const uint8_t>(root.data(), root.size()),
                    std::span<const uint8_t>(message.data(), message.size()), digest)) {
        return false;
    }
    fdskFromDigest(digest, out);
    mbedtls_platform_zeroize(digest.data(), digest.size());
    return true;
}

void buildBlePasskeyMessage(Serial serial, std::array<uint8_t, kBlePasskeyMessageBytes> &out)
{
    std::memcpy(out.data(), kBlePasskeyLabel, kBlePasskeyLabelLen);
    std::memcpy(out.data() + kBlePasskeyLabelLen, serial.data(), kSerialBytes);
}

uint32_t blePasskeyFromDigest(const Digest &digest)
{
    const uint32_t head = (static_cast<uint32_t>(digest[0]) << 24)
                          | (static_cast<uint32_t>(digest[1]) << 16)
                          | (static_cast<uint32_t>(digest[2]) << 8)
                          | static_cast<uint32_t>(digest[3]);
    return head % kBlePasskeyModulus;
}

bool deriveBlePasskey(const RootSecret &root, Serial serial, uint32_t &out)
{
    std::array<uint8_t, kBlePasskeyMessageBytes> message{};
    buildBlePasskeyMessage(serial, message);

    Digest digest{};
    if (!hmacSha256(std::span<const uint8_t>(root.data(), root.size()),
                    std::span<const uint8_t>(message.data(), message.size()), digest)) {
        return false;
    }
    out = blePasskeyFromDigest(digest);
    mbedtls_platform_zeroize(digest.data(), digest.size());
    return true;
}

} // namespace habinari::secret
