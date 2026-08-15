// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

/**
 * @file test_device_secret.cpp
 * @brief Host tests for the portable half of the device root secret.
 *
 * Two things need pinning here, because the eFuse burn that depends on them is
 * irreversible and the certificate they produce gets printed on a label:
 *
 *  1. The KDF. The FDSK is computed in software during the dry run and by the
 *     HMAC peripheral afterwards, and the two must agree byte for byte. The
 *     peripheral implements plain RFC 2104 HMAC-SHA256, so the software side
 *     is pinned to the RFC 4231 vectors, and the whole derivation is pinned to
 *     a golden computed independently (Python hmac/hashlib).
 *
 *  2. The entropy gate, which is the only thing standing between a dead RNG
 *     and a permanently fused dud key.
 */

#include "unity.h"

#include "device_identity.hpp"
#include "device_secret.hpp"

#include <array>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace habinari;
using secret::Digest;
using secret::EntropyPool;
using secret::EntropyQuality;
using secret::Fdsk;
using secret::RootSecret;

void setUp() {}
void tearDown() {}

namespace {

std::span<const uint8_t> bytesOf(const std::vector<uint8_t> &v)
{
    return std::span<const uint8_t>(v.data(), v.size());
}

std::span<const uint8_t> bytesOf(const std::string &s)
{
    return std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(s.data()), s.size());
}

std::string hex(std::span<const uint8_t> bytes)
{
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string out;
    for (const uint8_t b : bytes) {
        out.push_back(kDigits[b >> 4]);
        out.push_back(kDigits[b & 0x0F]);
    }
    return out;
}

// A fixed root secret and serial number, so the derivation has a stable golden.
const RootSecret kTestRoot = [] {
    RootSecret r{};
    for (size_t i = 0; i < r.size(); ++i) {
        r[i] = static_cast<uint8_t>(i);
    }
    return r;
}();

constexpr uint8_t kTestSerial[6] = {0x24, 0x6F, 0x28, 0x11, 0x22, 0x33};

secret::Serial testSerial()
{
    return std::span<const uint8_t, 6>(kTestSerial, 6);
}

// Fill a buffer from a deterministic PRNG — a stand-in for a healthy hardware
// RNG sample, so the "accepts a good sample" case is reproducible.
std::vector<uint8_t> pseudoRandomSample(size_t bytes, uint32_t seed)
{
    std::mt19937 rng(seed);
    std::vector<uint8_t> out(bytes);
    for (auto &byte : out) {
        byte = static_cast<uint8_t>(rng() & 0xFFu);
    }
    return out;
}

EntropyQuality measure(std::span<const uint8_t> sample)
{
    EntropyPool pool;
    pool.addEntropy(sample);
    return pool.quality();
}

} // namespace

// --- HMAC primitive -------------------------------------------------------

// RFC 4231 test case 1. The dry-run certificate is only trustworthy if the
// software HMAC matches the HMAC peripheral, and the peripheral implements the
// standard function — so pin the software side to the standard's own vectors.
void test_hmac_sha256_rfc4231_case1()
{
    const std::vector<uint8_t> key(20, 0x0b);
    const std::string data = "Hi There";

    Digest out{};
    TEST_ASSERT_TRUE(secret::hmacSha256(bytesOf(key), bytesOf(data), out));
    TEST_ASSERT_EQUAL_STRING("b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7",
                             hex(out).c_str());
}

// RFC 4231 test case 2 (short key, so the key padding path differs).
void test_hmac_sha256_rfc4231_case2()
{
    const std::string key = "Jefe";
    const std::string data = "what do ya want for nothing?";

    Digest out{};
    TEST_ASSERT_TRUE(secret::hmacSha256(bytesOf(key), bytesOf(data), out));
    TEST_ASSERT_EQUAL_STRING("5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843",
                             hex(out).c_str());
}

// RFC 4231 test case 6: a 131-byte key, which exercises the "hash the key
// first" branch. The device's own key is always 32 bytes, but the branch is
// there and untested code in a KDF is how goldens quietly change.
void test_hmac_sha256_rfc4231_case6()
{
    const std::vector<uint8_t> key(131, 0xaa);
    const std::string data = "Test Using Larger Than Block-Size Key - Hash Key First";

    Digest out{};
    TEST_ASSERT_TRUE(secret::hmacSha256(bytesOf(key), bytesOf(data), out));
    TEST_ASSERT_EQUAL_STRING("60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54",
                             hex(out).c_str());
}

// --- FDSK derivation ------------------------------------------------------

// The message the HMAC runs over is built once and used by both the software
// and the peripheral path; if its layout drifted the two would silently
// disagree, so pin it exactly.
void test_fdsk_message_layout()
{
    std::array<uint8_t, secret::kFdskMessageBytes> message{};
    secret::buildFdskMessage(testSerial(), message);

    TEST_ASSERT_EQUAL_UINT32(24u, static_cast<uint32_t>(message.size()));
    TEST_ASSERT_EQUAL_MEMORY("KNstaX/KNX-FDSK/v1", message.data(), secret::kFdskLabelLen);
    TEST_ASSERT_EQUAL_MEMORY(kTestSerial, message.data() + secret::kFdskLabelLen, 6);
}

// Golden computed independently (python: hmac.new(root, b"KNstaX/KNX-FDSK/v1"
// + serial, sha256).digest()[:16]). A change here means every already-printed
// device certificate has been invalidated.
void test_fdsk_golden()
{
    Fdsk fdsk{};
    TEST_ASSERT_TRUE(secret::deriveFdsk(kTestRoot, testSerial(), fdsk));
    TEST_ASSERT_EQUAL_STRING("0c5555406b87fea1d845819aab253531", hex(fdsk).c_str());
}

void test_fdsk_is_deterministic()
{
    Fdsk first{};
    Fdsk second{};
    TEST_ASSERT_TRUE(secret::deriveFdsk(kTestRoot, testSerial(), first));
    TEST_ASSERT_TRUE(secret::deriveFdsk(kTestRoot, testSerial(), second));
    TEST_ASSERT_EQUAL_MEMORY(first.data(), second.data(), first.size());
}

// One bit of serial or of root secret must change the key completely —
// otherwise two boards off the same reel could share an FDSK.
void test_fdsk_binds_serial_and_root()
{
    Fdsk baseline{};
    TEST_ASSERT_TRUE(secret::deriveFdsk(kTestRoot, testSerial(), baseline));

    uint8_t otherSerial[6] = {};
    std::memcpy(otherSerial, kTestSerial, sizeof(otherSerial));
    otherSerial[5] ^= 0x01;
    Fdsk bySerial{};
    TEST_ASSERT_TRUE(
        secret::deriveFdsk(kTestRoot, std::span<const uint8_t, 6>(otherSerial, 6), bySerial));
    TEST_ASSERT_NOT_EQUAL(0, std::memcmp(baseline.data(), bySerial.data(), baseline.size()));

    RootSecret otherRoot = kTestRoot;
    otherRoot[0] ^= 0x01;
    Fdsk byRoot{};
    TEST_ASSERT_TRUE(secret::deriveFdsk(otherRoot, testSerial(), byRoot));
    TEST_ASSERT_NOT_EQUAL(0, std::memcmp(baseline.data(), byRoot.data(), baseline.size()));
}

// The FDSK is the leading half of the digest; the peripheral path truncates
// the same way.
void test_fdsk_is_digest_prefix()
{
    std::array<uint8_t, secret::kFdskMessageBytes> message{};
    secret::buildFdskMessage(testSerial(), message);

    Digest digest{};
    TEST_ASSERT_TRUE(secret::hmacSha256(
        std::span<const uint8_t>(kTestRoot.data(), kTestRoot.size()),
        std::span<const uint8_t>(message.data(), message.size()), digest));

    Fdsk fromDigest{};
    secret::fdskFromDigest(digest, fromDigest);

    Fdsk derived{};
    TEST_ASSERT_TRUE(secret::deriveFdsk(kTestRoot, testSerial(), derived));
    TEST_ASSERT_EQUAL_MEMORY(fromDigest.data(), derived.data(), derived.size());
    TEST_ASSERT_EQUAL_MEMORY(digest.data(), derived.data(), derived.size());
}

// --- Entropy pool ---------------------------------------------------------

void test_pool_accepts_healthy_sample()
{
    const auto sample = pseudoRandomSample(1024, 12345);
    const EntropyQuality quality = measure(bytesOf(sample));

    TEST_ASSERT_TRUE(quality.sane());
    TEST_ASSERT_EQUAL_UINT32(1024u, quality.sampledBytes);
    TEST_ASSERT_EQUAL_UINT32(256u, quality.sampledWords);
    TEST_ASSERT_EQUAL_UINT32(0u, quality.repeatedWords);
    TEST_ASSERT_FALSE(quality.constantWords);
    TEST_ASSERT_GREATER_OR_EQUAL(EntropyQuality::kMinDistinctByteValues,
                                 quality.distinctByteValues);
}

// A sample below the calibrated size is refused even when it looks random:
// the thresholds are only meaningful over 1 KB.
void test_pool_rejects_short_sample()
{
    const auto sample = pseudoRandomSample(512, 777);
    TEST_ASSERT_FALSE(measure(bytesOf(sample)).sane());
}

// The three dead-source signatures: stuck at 0, stuck at 1, and a register
// that returns the same word forever.
void test_pool_rejects_stuck_low()
{
    const std::vector<uint8_t> sample(2048, 0x00);
    const EntropyQuality quality = measure(bytesOf(sample));
    TEST_ASSERT_FALSE(quality.sane());
    TEST_ASSERT_EQUAL_UINT32(0u, quality.orAll);
}

void test_pool_rejects_stuck_high()
{
    const std::vector<uint8_t> sample(2048, 0xFF);
    const EntropyQuality quality = measure(bytesOf(sample));
    TEST_ASSERT_FALSE(quality.sane());
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, quality.andAll);
}

void test_pool_rejects_constant_words()
{
    std::vector<uint8_t> sample;
    // 0x5A3C repeated: perfectly balanced bit count, so only the constant and
    // repeated-word gates can catch it.
    for (size_t i = 0; i < 512; ++i) {
        sample.insert(sample.end(), {0x5A, 0x3C, 0x5A, 0x3C});
    }
    const EntropyQuality quality = measure(bytesOf(sample));
    TEST_ASSERT_TRUE(quality.constantWords);
    TEST_ASSERT_FALSE(quality.sane());
}

// A biased source: every byte has exactly one bit set, so 12.5 % ones.
void test_pool_rejects_biased_sample()
{
    std::mt19937 rng(99);
    std::vector<uint8_t> sample(2048);
    for (auto &byte : sample) {
        byte = static_cast<uint8_t>(1u << (rng() % 8));
    }
    const EntropyQuality quality = measure(bytesOf(sample));
    TEST_ASSERT_FALSE(quality.sane());
}

// A source stuck in a narrow alphabet still fails, even with plausible bit
// balance and no adjacent repeats.
void test_pool_rejects_narrow_alphabet()
{
    std::mt19937 rng(4242);
    std::vector<uint8_t> sample(2048);
    for (auto &byte : sample) {
        // 16 values spread across the byte range, balanced around 4 bits set.
        static constexpr uint8_t kAlphabet[16] = {0x0F, 0xF0, 0x33, 0xCC, 0x55, 0xAA, 0x3C, 0xC3,
                                                  0x69, 0x96, 0x5A, 0xA5, 0x66, 0x99, 0x3A, 0xC5};
        byte = kAlphabet[rng() % 16];
    }
    const EntropyQuality quality = measure(bytesOf(sample));
    TEST_ASSERT_LESS_THAN(EntropyQuality::kMinDistinctByteValues, quality.distinctByteValues);
    TEST_ASSERT_FALSE(quality.sane());
}

// The real sample arrives in chunks with context folded in between, so word
// statistics must carry across calls rather than restart at every chunk.
void test_pool_word_stats_span_chunk_boundaries()
{
    const auto sample = pseudoRandomSample(1024, 2024);

    EntropyPool whole;
    whole.addEntropy(bytesOf(sample));

    EntropyPool chunked;
    for (size_t offset = 0; offset < sample.size(); offset += 3) {
        const size_t len = std::min<size_t>(3, sample.size() - offset);
        chunked.addEntropy(std::span<const uint8_t>(sample.data() + offset, len));
    }

    TEST_ASSERT_EQUAL_UINT32(whole.quality().sampledWords, chunked.quality().sampledWords);
    TEST_ASSERT_EQUAL_UINT32(whole.quality().orAll, chunked.quality().orAll);
    TEST_ASSERT_EQUAL_UINT32(whole.quality().andAll, chunked.quality().andAll);
    TEST_ASSERT_EQUAL_UINT32(whole.quality().oneBits, chunked.quality().oneBits);
    TEST_ASSERT_EQUAL_UINT32(whole.quality().distinctByteValues,
                             chunked.quality().distinctByteValues);
}

// Context bytes (MAC, timestamps) must reach the digest without being counted
// as entropy — otherwise a dead RNG could be masked by public data.
void test_pool_context_is_hashed_but_not_measured()
{
    const auto sample = pseudoRandomSample(1024, 5150);
    const std::vector<uint8_t> context = {0x24, 0x6F, 0x28, 0x11, 0x22, 0x33};

    EntropyPool withContext;
    withContext.addEntropy(bytesOf(sample));
    withContext.addContext(bytesOf(context));
    RootSecret a{};
    TEST_ASSERT_TRUE(withContext.finish(a));

    EntropyPool withoutContext;
    withoutContext.addEntropy(bytesOf(sample));
    RootSecret b{};
    TEST_ASSERT_TRUE(withoutContext.finish(b));

    TEST_ASSERT_EQUAL_UINT32(withoutContext.quality().sampledBytes,
                             withContext.quality().sampledBytes);
    TEST_ASSERT_NOT_EQUAL(0, std::memcmp(a.data(), b.data(), a.size()));
}

// Same inputs in the same order produce the same secret, and it is the SHA-256
// of the concatenation rather than of the last chunk alone.
void test_pool_output_is_deterministic()
{
    const auto sample = pseudoRandomSample(1024, 31337);

    RootSecret first{};
    RootSecret second{};
    EntropyPool a;
    a.addEntropy(bytesOf(sample));
    TEST_ASSERT_TRUE(a.finish(first));
    EntropyPool b;
    b.addEntropy(bytesOf(sample));
    TEST_ASSERT_TRUE(b.finish(second));
    TEST_ASSERT_EQUAL_MEMORY(first.data(), second.data(), first.size());

    // Independently: SHA-256 of the same bytes. (python: hashlib.sha256 over
    // the same mt19937 stream is awkward, so compare against a second pool fed
    // one byte at a time — same bytes, different call pattern.)
    EntropyPool drip;
    for (const uint8_t byte : sample) {
        drip.addEntropy(std::span<const uint8_t>(&byte, 1));
    }
    RootSecret third{};
    TEST_ASSERT_TRUE(drip.finish(third));
    TEST_ASSERT_EQUAL_MEMORY(first.data(), third.data(), first.size());
}

void test_pool_finish_requires_entropy()
{
    EntropyPool empty;
    RootSecret out{};
    TEST_ASSERT_FALSE(empty.finish(out));

    // Context alone is not entropy and must not unlock a secret either.
    EntropyPool contextOnly;
    const std::vector<uint8_t> context(64, 0xA5);
    contextOnly.addContext(bytesOf(context));
    TEST_ASSERT_FALSE(contextOnly.finish(out));
}

void test_pool_finish_is_single_shot()
{
    const auto sample = pseudoRandomSample(1024, 8);
    EntropyPool pool;
    pool.addEntropy(bytesOf(sample));

    RootSecret out{};
    TEST_ASSERT_TRUE(pool.finish(out));
    TEST_ASSERT_FALSE(pool.finish(out));
}

// --- Certificate encoding -------------------------------------------------

// Golden computed independently in Python (base32 of serial || fdsk || crc4,
// truncated to 36 characters and grouped in sixes). This is the string an
// installer types into ETS, so it is the contract the dry run prints against.
void test_certificate_golden()
{
    const Fdsk fdsk = {0x0C, 0x55, 0x55, 0x40, 0x6B, 0x87, 0xFE, 0xA1,
                       0xD8, 0x45, 0x81, 0x9A, 0xAB, 0x25, 0x35, 0x31};
    char certificate[identity::kCertificateBufferSize] = {};
    identity::formatDeviceCertificate(testSerial(), fdsk, certificate, sizeof(certificate));

    TEST_ASSERT_EQUAL_STRING("ERXSQE-JCGMGF-KVKANO-D75IOY-IWAZVK-ZFGUYX", certificate);
    TEST_ASSERT_EQUAL_UINT32(identity::kCertificateChars + 5,
                             static_cast<uint32_t>(std::strlen(certificate)));
}

// The certificate the dry run prints must be the one the device derives, so
// the whole chain root -> FDSK -> certificate is pinned end to end.
void test_certificate_matches_derived_fdsk()
{
    Fdsk fdsk{};
    TEST_ASSERT_TRUE(secret::deriveFdsk(kTestRoot, testSerial(), fdsk));

    char certificate[identity::kCertificateBufferSize] = {};
    identity::formatDeviceCertificate(testSerial(), fdsk, certificate, sizeof(certificate));
    TEST_ASSERT_EQUAL_STRING("ERXSQE-JCGMGF-KVKANO-D75IOY-IWAZVK-ZFGUYX", certificate);
}

void test_certificate_uses_base32_alphabet_and_grouping()
{
    const Fdsk fdsk = {};
    char certificate[identity::kCertificateBufferSize] = {};
    identity::formatDeviceCertificate(testSerial(), fdsk, certificate, sizeof(certificate));

    size_t chars = 0;
    for (size_t i = 0; certificate[i] != '\0'; ++i) {
        const char c = certificate[i];
        if (c == '-') {
            TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(chars % 6));
            continue;
        }
        TEST_ASSERT_TRUE((c >= 'A' && c <= 'Z') || (c >= '2' && c <= '7'));
        chars++;
    }
    TEST_ASSERT_EQUAL_UINT32(identity::kCertificateChars, static_cast<uint32_t>(chars));
}

void test_crc4_golden()
{
    TEST_ASSERT_EQUAL_HEX8(0x0D, identity::crc4(std::span<const uint8_t>(kTestSerial, 6)));
    const uint8_t zeros[4] = {};
    TEST_ASSERT_EQUAL_HEX8(0x00, identity::crc4(std::span<const uint8_t>(zeros, 4)));
}

void test_tohex_separator_and_truncation()
{
    const uint8_t bytes[3] = {0x00, 0xAB, 0xFF};

    // The bound is `pos + 3 < outLen`, i.e. one byte more headroom than the
    // 2n+1 the output strictly needs. Every caller sizes its buffer 3n, so the
    // margin never bites; pin it so a future tightening is a deliberate change.
    char plain[8] = {};
    identity::toHex(std::span<const uint8_t>(bytes, 3), '\0', plain, sizeof(plain));
    TEST_ASSERT_EQUAL_STRING("00ABFF", plain);

    char exact[7] = {};
    identity::toHex(std::span<const uint8_t>(bytes, 3), '\0', exact, sizeof(exact));
    TEST_ASSERT_EQUAL_STRING("00AB", exact);

    char spaced[9] = {};
    identity::toHex(std::span<const uint8_t>(bytes, 3), ' ', spaced, sizeof(spaced));
    TEST_ASSERT_EQUAL_STRING("00 AB FF", spaced);

    // A short buffer must truncate and still terminate rather than overrun.
    char tight[4] = {};
    identity::toHex(std::span<const uint8_t>(bytes, 3), '\0', tight, sizeof(tight));
    TEST_ASSERT_EQUAL_STRING("00", tight);
}

int main()
{
    UNITY_BEGIN();

    RUN_TEST(test_hmac_sha256_rfc4231_case1);
    RUN_TEST(test_hmac_sha256_rfc4231_case2);
    RUN_TEST(test_hmac_sha256_rfc4231_case6);

    RUN_TEST(test_fdsk_message_layout);
    RUN_TEST(test_fdsk_golden);
    RUN_TEST(test_fdsk_is_deterministic);
    RUN_TEST(test_fdsk_binds_serial_and_root);
    RUN_TEST(test_fdsk_is_digest_prefix);

    RUN_TEST(test_pool_accepts_healthy_sample);
    RUN_TEST(test_pool_rejects_short_sample);
    RUN_TEST(test_pool_rejects_stuck_low);
    RUN_TEST(test_pool_rejects_stuck_high);
    RUN_TEST(test_pool_rejects_constant_words);
    RUN_TEST(test_pool_rejects_biased_sample);
    RUN_TEST(test_pool_rejects_narrow_alphabet);
    RUN_TEST(test_pool_word_stats_span_chunk_boundaries);
    RUN_TEST(test_pool_context_is_hashed_but_not_measured);
    RUN_TEST(test_pool_output_is_deterministic);
    RUN_TEST(test_pool_finish_requires_entropy);
    RUN_TEST(test_pool_finish_is_single_shot);

    RUN_TEST(test_certificate_golden);
    RUN_TEST(test_certificate_matches_derived_fdsk);
    RUN_TEST(test_certificate_uses_base32_alphabet_and_grouping);
    RUN_TEST(test_crc4_golden);
    RUN_TEST(test_tohex_separator_and_truncation);

    return UNITY_END();
}
