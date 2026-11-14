// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// The device's KNX identity: its serial number, its KNX Data Secure factory
// tool key (FDSK) and the ETS device certificate that pairs the two.

#include "knx_service_internal.hpp"

#include "device_identity.hpp"
#include "device_secret.hpp"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "nvs.h"

#include "knx/util/log.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <span>

using namespace knx;

// The product declares itself Data Secure capable on both media
// (SecurityPolicy::dataSecureCapable in knx_product.hpp, SecureEnabled in the
// .knxprod), and everything below assumes the stack can honour that. A build
// without CONFIG_KNX_ENABLE_SECURITY links no Security Interface Object and no
// CCM transforms, so the factory Tool Key would be installed into nothing and
// the certificate printed at boot would describe a key the device cannot use.
// That is how the TP1 image shipped until sdkconfig.defaults set the option;
// refuse to build it again.
static_assert(KNX_SECURE_ENABLED,
              "knx_product.hpp declares dataSecureCapable = true: build with "
              "CONFIG_KNX_ENABLE_SECURITY=y (sdkconfig.defaults sets it)");

namespace habinari_knx::service {

namespace {

// --- KNX Data Secure ------------------------------------------------------
// The device's FDSK / initial tool key is derived from a 256-bit root secret
// held in a read-protected eFuse key block (see device_secret.hpp), so it
// survives a factory reset and the same firmware image still gives every board
// a different key. It is logged every boot, together with the serial number
// and the ETS device certificate, so an installer can capture it locally for
// secure commissioning.
//
// Until that eFuse block is provisioned — which is deliberately off by default
// while the dry run is being checked on hardware — the device falls back to
// the legacy behaviour: a random 16-byte key generated on first boot and kept
// in NVS. That key does NOT survive knx_service_reset_nvm(), which erases the
// whole default NVS partition, so a factory reset regenerates it and the
// previously printed certificate stops matching the device.
//
// Data Secure is built into every KNX image, TP1 and IP alike; the ETS
// catalogue entry says so too (SecurityPolicy in knx_product.hpp), and the two
// must agree (see the static_assert above).
constexpr const char *kSecureNvsNamespace = "knx_secure";
constexpr const char *kSecureNvsToolKeyBlob = "tool_key";

// Hex, base32, CRC-4 and ETS device-certificate rendering now live in
// device_identity.hpp so the root-secret dry run can print the same
// certificate and the encoding is covered by host tests.
using habinari::identity::formatDeviceCertificate;
using habinari::identity::toHex;

// Load the persisted KNX Data Secure tool key, generating and storing a fresh
// random one on first boot. Returns true on success; `created` reports whether
// a new key was generated this call.
bool loadOrCreateToolKey(std::array<uint8_t, 16> &key, bool &created)
{
    created = false;
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kSecureNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Data Secure: nvs_open failed: %s", esp_err_to_name(err));
        return false;
    }

    size_t len = key.size();
    err = nvs_get_blob(handle, kSecureNvsToolKeyBlob, key.data(), &len);
    if (err == ESP_ERR_NVS_NOT_FOUND || (err == ESP_OK && len != key.size())) {
        esp_fill_random(key.data(), key.size());
        err = nvs_set_blob(handle, kSecureNvsToolKeyBlob, key.data(), key.size());
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        created = true;
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Data Secure: tool key %s failed: %s",
                 created ? "generation" : "load", esp_err_to_name(err));
        return false;
    }
    return true;
}

// Resolve the device's factory tool key, preferring the FDSK derived from the
// eFuse root secret and falling back to the legacy NVS key while that block is
// still unprovisioned. `serial` is null when the MAC read failed, which rules
// out the derived path (the serial is part of the KDF message).
bool resolveFactoryToolKey(const uint8_t *serial,
                           std::array<uint8_t, 16> &key,
                           ToolKeySource &source,
                           bool &created)
{
    created = false;
    if (serial != nullptr) {
        const auto resolved =
            habinari::secret::resolveFdsk(std::span<const uint8_t, 6>(serial, 6));
        if (resolved.fdskValid) {
            key = resolved.fdsk;
            source = ToolKeySource::EfuseDerived;
            return true;
        }
    }
    source = ToolKeySource::NvsFallback;
    return loadOrCreateToolKey(key, created);
}

}  // namespace

KnxSerialNumber readSerialNumber()
{
    KnxSerialNumber serial;
    // KNX serial number (device object PID 11): derive from the
    // factory-programmed base MAC so every board reports a unique,
    // stable identity instead of all zeroes. The MAC is globally unique,
    // so this doubles as the device's manufacturing serial. It is logged
    // here so it can be paired with the Data Secure key during ETS
    // commissioning.
    uint8_t mac[6] = {};
    if (esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK) {
        std::copy(std::begin(mac), std::end(mac), std::begin(serial.bytes));
        serial.valid = true;
        char serialHex[3 * 6] = {};
        toHex(std::span<const uint8_t>(mac, sizeof(mac)), ':', serialHex, sizeof(serialHex));
        KNX_LOGI(TAG, "KNX serial number: %s", serialHex);
    } else {
        KNX_LOGW(TAG, "KNX serial number unavailable: esp_read_mac failed");
    }
    return serial;
}

void assignSerialNumber(HabinariApp &app, const KnxSerialNumber &serial)
{
    if (serial.valid) {
        app.deviceObject().setSerialNumber(std::span<const uint8_t>(serial.bytes, sizeof(serial.bytes)));
    }
}

DataSecureFactoryKey resolveDataSecureFactoryKey(const KnxSerialNumber &serial)
{
    // This key is the device's FDSK / initial tool key: derived from the
    // eFuse root secret once that is provisioned, otherwise the legacy
    // NVS-stored random key.
    DataSecureFactoryKey factoryKey;
    factoryKey.valid = resolveFactoryToolKey(serial.valid ? serial.bytes : nullptr, factoryKey.key,
                                             factoryKey.source, factoryKey.created);
    return factoryKey;
}

product::SecureCommissioningOptions dataSecureFactoryDefaults(const DataSecureFactoryKey &factoryKey)
{
    // The FDSK is the factory default, set before the stack starts (the
    // pre-init rule on BusAccessUnit::init). ETS installs its own Tool Key
    // during secure commissioning and then restarts the device; the stack
    // restores that key from its persistence over this default, so the
    // device is never put back on its FDSK behind ETS's back (which made
    // ETS's first S-A_Sync_Request after the restart fail: "no SyncResponse
    // was received; probably the key did not match"). A Factory Reset
    // returns to it.
    //
    // Security Mode stays off ex-factory. It is ETS's property, it is
    // persisted, and ETS turns it on when it finishes a secure download.
    // Forcing it on made the device refuse every unsecured management
    // service, so a project that does not use security could not
    // commission it at all.
    product::SecureCommissioningOptions secure{};
    if (factoryKey.valid) {
        secure.factoryToolKey = factoryKey.key;
    }
    secure.securityModeAtFactoryReset = false;
    return secure;
}

void reportDataSecure(HabinariApp &app, const KnxSerialNumber &serial,
                      const DataSecureFactoryKey &factoryKey)
{
    if (!factoryKey.valid) {
        // resolveFactoryToolKey() has logged why.
        return;
    }

    // Log the key in hex so an installer can capture it locally and enter
    // it, together with the serial number, into ETS for secure
    // commissioning, while it is the key in force. hasEtsToolKey() is only
    // "some tool key is set"; what decides is whether the key in force IS
    // the factory key.
    const bool factoryKeyInForce = app.toolKeyInForceIs(factoryKey.key);
    const auto decision = habinari::identity::certificateDecision(serial.valid, app.hasEtsToolKey(),
                                                                  factoryKeyInForce);

    if (decision == habinari::identity::CertificateDecision::EtsKeyInForce) {
        KNX_LOGW(TAG,
                 "KNX Data Secure available (mode %s, set by ETS). Keeping the "
                 "tool key already on the device; the factory key is only valid "
                 "again after a factory reset.",
                 app.isSecurityEnabled() ? "ON" : "off");
    } else {
        char keyHex[3 * 16] = {};
        toHex(std::span<const uint8_t>(factoryKey.key.data(), factoryKey.key.size()), ' ', keyHex,
              sizeof(keyHex));
        KNX_LOGW(TAG,
                 "KNX Data Secure available (mode %s, set by ETS). "
                 "Tool key (%s): %s",
                 app.isSecurityEnabled() ? "ON" : "off",
                 factoryKey.source == ToolKeySource::EfuseDerived
                     ? "derived from the eFuse root secret"
                     : (factoryKey.created ? "NVS, generated" : "NVS, stored"),
                 keyHex);
    }

    if (factoryKey.source == ToolKeySource::NvsFallback) {
        KNX_LOGW(TAG,
                 "This device has no eFuse root secret yet, so the key above lives "
                 "in NVS and a factory reset will replace it — the certificate "
                 "below is only valid until then.");
    }

    // The certificate is only usable while the factory key is the one the
    // device would actually accept. Printing it once ETS has installed a
    // Tool Key of its own is worse than printing nothing: ETS answers an
    // FDSK that is no longer in force with "Device is secured with a key
    // not known within this project", and re-entering the same certificate
    // can never change that.
    switch (decision) {
        case habinari::identity::CertificateDecision::NoSerial:
            KNX_LOGW(TAG, "KNX Data Secure device certificate unavailable: no serial number");
            break;
        case habinari::identity::CertificateDecision::EtsKeyInForce:
            KNX_LOGW(TAG,
                     "KNX Data Secure device certificate NOT printed: the Tool Key in "
                     "force was installed by ETS, so the factory certificate would not "
                     "be accepted. Use the project that commissioned this device, or "
                     "factory-reset it and use the certificate printed on the next boot.");
            break;
        case habinari::identity::CertificateDecision::Print: {
            char certificate[habinari::identity::kCertificateBufferSize] = {};
            formatDeviceCertificate(std::span<const uint8_t, 6>(serial.bytes, 6), factoryKey.key,
                                    certificate, sizeof(certificate));
            KNX_LOGW(TAG, "KNX Data Secure device certificate (enter in ETS): %s", certificate);
            break;
        }
    }
}

}  // namespace habinari_knx::service
