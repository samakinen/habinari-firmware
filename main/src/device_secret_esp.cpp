/**
 * @file device_secret_esp.cpp
 * @brief eFuse-backed device root secret: provisioning and FDSK derivation.
 *
 * See device_secret.hpp for why the FDSK is derived rather than stored.
 *
 * Provisioning is a once-per-chip, irreversible operation, so it is gated
 * three ways: the target block must be unused, the RNG sample must pass the
 * entropy gate, and CONFIG_SENSOR_BOARD_ROOT_SECRET_BURN must be set. With
 * that last one off (the default) the device runs the whole path and logs the
 * candidate secret, the FDSK it would produce and the resulting ETS
 * certificate, without touching a fuse — which is how the values get eyeballed
 * on real hardware before anything becomes permanent.
 */

#include "device_secret.hpp"

#include "device_identity.hpp"

#include "bootloader_random.h"
#include "esp_efuse.h"
#include "esp_hmac.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/platform_util.h"
#include "sdkconfig.h"

#include <cinttypes>
#include <cstring>

namespace sensor_board::secret {

namespace {

constexpr const char *TAG = "device_secret";

constexpr int kRootSecretBlockIndex = CONFIG_SENSOR_BOARD_ROOT_SECRET_EFUSE_BLOCK;
static_assert(kRootSecretBlockIndex >= 0 && kRootSecretBlockIndex <= 5,
              "The HMAC peripheral can only use eFuse key blocks 0..5");

constexpr esp_efuse_block_t kRootSecretBlock =
    static_cast<esp_efuse_block_t>(EFUSE_BLK_KEY0 + kRootSecretBlockIndex);
constexpr hmac_key_id_t kRootSecretHmacKey =
    static_cast<hmac_key_id_t>(HMAC_KEY0 + kRootSecretBlockIndex);

// Entropy gathering. The sample is taken in chunks spread over time rather
// than in one burst: successive SAR-ADC samples taken milliseconds apart are
// far less correlated than back-to-back reads. 8 x 128 B = 1024 B, the size
// EntropyQuality's thresholds are calibrated for.
constexpr size_t kEntropyChunks = 8;
constexpr size_t kEntropyChunkBytes = 128;
// bootloader_random_enable() has just powered up the SAR ADC through regi2c
// and enabled its trigger; the analog front end needs to settle before the
// first samples mean anything.
constexpr uint32_t kEntropySettleMs = 20;
constexpr uint32_t kEntropyChunkGapMs = 10;

/// Fold a rate-limited, time-spread RNG sample into a candidate root secret.
/// Returns false when the sample fails the sanity gate (nothing is burned).
bool gatherRootSecret(RootSecret &out, EntropyQuality &quality)
{
    EntropyPool pool;
    uint8_t chunk[kEntropyChunkBytes];

    // No RF is ever enabled on this board, so the SAR-ADC noise source has to
    // be switched on explicitly for esp_random() to be a true RNG. Nothing
    // else here uses the ADC; it is switched back off immediately.
    bootloader_random_enable();
    vTaskDelay(pdMS_TO_TICKS(kEntropySettleMs));
    for (size_t c = 0; c < kEntropyChunks; ++c) {
        esp_fill_random(chunk, sizeof(chunk));
        pool.addEntropy(std::span<const uint8_t>(chunk, sizeof(chunk)));

        // The RTC/systimer counter runs off a different clock domain than the
        // CPU, so the exact arrival time of each chunk carries a little
        // genuine jitter. Folded in as context, never counted as entropy.
        const int64_t now = esp_timer_get_time();
        pool.addContext(std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(&now),
                                                 sizeof(now)));
        if (c + 1 < kEntropyChunks) {
            vTaskDelay(pdMS_TO_TICKS(kEntropyChunkGapMs));
        }
    }
    bootloader_random_disable();
    mbedtls_platform_zeroize(chunk, sizeof(chunk));

    // Public, so it adds no unpredictability. It is folded in only so that a
    // total RNG failure would still yield a distinct key per device rather
    // than one shared by the whole fleet.
    uint8_t mac[6] = {};
    if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
        pool.addContext(std::span<const uint8_t>(mac, sizeof(mac)));
    }

    quality = pool.quality();
    ESP_LOGI(TAG,
             "RNG sample: %" PRIu32 " B, %" PRIu32 "/256 byte values, %" PRIu32
             " %% one bits, %" PRIu32 " repeated words, OR=%08" PRIx32 " AND=%08" PRIx32,
             quality.sampledBytes, quality.distinctByteValues,
             quality.sampledBytes ? (quality.oneBits * 100u) / (quality.sampledBytes * 8u) : 0u,
             quality.repeatedWords, quality.orAll, quality.andAll);

    if (!quality.sane()) {
        return false;
    }
    return pool.finish(out);
}

#if !CONFIG_SENSOR_BOARD_ROOT_SECRET_BURN
/// Log what a candidate root secret would produce, without burning anything.
void logCandidate(const RootSecret &root, Serial serial, const Fdsk &fdsk)
{
#if CONFIG_SENSOR_BOARD_ROOT_SECRET_LOG_DRY_RUN_SECRET
    char rootHex[3 * kRootSecretBytes] = {};
    identity::toHex(std::span<const uint8_t>(root.data(), root.size()), ' ', rootHex,
                    sizeof(rootHex));
    ESP_LOGW(TAG, "DRY RUN root secret (would be burned into BLOCK_KEY%d): %s",
             kRootSecretBlockIndex, rootHex);
#else
    (void)root;
#endif

    char fdskHex[3 * kFdskBytes] = {};
    identity::toHex(std::span<const uint8_t>(fdsk.data(), fdsk.size()), ' ', fdskHex,
                    sizeof(fdskHex));
    ESP_LOGW(TAG, "DRY RUN FDSK that this root would derive: %s", fdskHex);

    char certificate[identity::kCertificateBufferSize] = {};
    identity::formatDeviceCertificate(serial, fdsk, certificate, sizeof(certificate));
    ESP_LOGW(TAG, "DRY RUN device certificate: %s", certificate);
    ESP_LOGW(TAG,
             "DRY RUN: no eFuse was written. This candidate is discarded; a different one is "
             "generated on every boot until CONFIG_SENSOR_BOARD_ROOT_SECRET_BURN is enabled.");
}
#endif // !CONFIG_SENSOR_BOARD_ROOT_SECRET_BURN

/// Derive through the HMAC peripheral, i.e. from the key the device actually
/// holds. Never sees the root secret itself.
bool deriveFdskFromEfuse(Serial serial, Fdsk &out)
{
    std::array<uint8_t, kFdskMessageBytes> message{};
    buildFdskMessage(serial, message);

    Digest digest{};
    const esp_err_t err =
        esp_hmac_calculate(kRootSecretHmacKey, message.data(), message.size(), digest.data());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_hmac_calculate failed: %s", esp_err_to_name(err));
        return false;
    }
    fdskFromDigest(digest, out);
    mbedtls_platform_zeroize(digest.data(), digest.size());
    return true;
}

} // namespace

RootSecretResult resolveFdsk(Serial serial)
{
    RootSecretResult result{};

    const esp_efuse_purpose_t purpose = esp_efuse_get_key_purpose(kRootSecretBlock);
    if (purpose == ESP_EFUSE_KEY_PURPOSE_HMAC_UP) {
        if (!deriveFdskFromEfuse(serial, result.fdsk)) {
            result.state = RootSecretState::DerivationFailed;
            return result;
        }
        result.state = RootSecretState::DerivedFromEfuse;
        result.fdskValid = true;
        ESP_LOGI(TAG, "Device root secret present in BLOCK_KEY%d; FDSK derived from eFuse",
                 kRootSecretBlockIndex);
        return result;
    }

    if (!esp_efuse_key_block_unused(kRootSecretBlock)) {
        ESP_LOGE(TAG,
                 "eFuse BLOCK_KEY%d is in use for purpose %d, not HMAC_UP — cannot provision a "
                 "device root secret there. Pick a free block with "
                 "CONFIG_SENSOR_BOARD_ROOT_SECRET_EFUSE_BLOCK.",
                 kRootSecretBlockIndex, static_cast<int>(purpose));
        result.state = RootSecretState::BlockUnavailable;
        return result;
    }

    RootSecret root{};
    EntropyQuality quality{};
    if (!gatherRootSecret(root, quality)) {
        ESP_LOGE(TAG,
                 "RNG sanity check failed — refusing to provision a device root secret from a "
                 "sample this poor.");
        mbedtls_platform_zeroize(root.data(), root.size());
        result.state = RootSecretState::EntropyRejected;
        return result;
    }

    // The software derivation is the reference: in a dry run it is what gets
    // printed, and after a burn it is what the peripheral is checked against.
    Fdsk softwareFdsk{};
    const bool derived = deriveFdsk(root, serial, softwareFdsk);

#if CONFIG_SENSOR_BOARD_ROOT_SECRET_BURN
    // esp_efuse_write_key() sets the key purpose and then read- and
    // write-protects the block, so this is the last moment the secret exists
    // in a readable form anywhere.
    const esp_err_t err = esp_efuse_write_key(kRootSecretBlock, ESP_EFUSE_KEY_PURPOSE_HMAC_UP,
                                              root.data(), root.size());
    mbedtls_platform_zeroize(root.data(), root.size());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_efuse_write_key(BLOCK_KEY%d) failed: %s", kRootSecretBlockIndex,
                 esp_err_to_name(err));
        result.state = RootSecretState::BurnFailed;
        return result;
    }
    ESP_LOGW(TAG, "Burned a device root secret into eFuse BLOCK_KEY%d (irreversible)",
             kRootSecretBlockIndex);

    if (!deriveFdskFromEfuse(serial, result.fdsk)) {
        result.state = RootSecretState::DerivationFailed;
        return result;
    }
    // If the peripheral disagreed with RFC 2104 HMAC-SHA256 the certificate
    // printed during the dry run would have been a lie, so say so loudly
    // rather than quietly shipping a different key.
    if (derived && result.fdsk != softwareFdsk) {
        ESP_LOGE(TAG,
                 "HMAC peripheral output differs from the software derivation — the dry-run "
                 "certificate for this device is NOT the one it will present.");
    }
    result.state = RootSecretState::DerivedFromEfuse;
    result.fdskValid = true;
    return result;
#else
    if (derived) {
        logCandidate(root, serial, softwareFdsk);
    } else {
        ESP_LOGE(TAG, "DRY RUN: FDSK derivation failed for the candidate root secret");
    }
    mbedtls_platform_zeroize(root.data(), root.size());
    result.state = RootSecretState::DryRunCandidate;
    return result;
#endif
}

} // namespace sensor_board::secret
