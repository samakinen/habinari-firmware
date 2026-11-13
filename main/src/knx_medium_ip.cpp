// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// ---------------------------------------------------------------------------
// The transport, and the only thing about this device that differs by medium.
//
// Everything else in the adapter — every group object, every parameter
// callback, the publish policy, programming mode — is the same code on TP1 and
// on KNXnet/IP. This file and its sibling are the seam: each implements the
// medium hooks of knx_service_internal.hpp, and startProduct() brings its own
// transport up and hands back the identical started product handle, so nothing
// downstream knows which medium it is running on. Exactly one of the two is
// compiled (main/CMakeLists.txt).
// ---------------------------------------------------------------------------
//
// This is the KNXnet/IP side: routing over the Wi-Fi station, the routing
// group as an out-of-band setting, and the identity ETS reads back from the
// KNXnet/IP Parameter Object. knx_medium_tp1.cpp is the other.

#include "knx_service_internal.hpp"

#include "device_config.h"
#include "device_default_name.h"
#include "knx_ip_addr.h"
#include "wifi_service.h"

#include "esp_err.h"
#include "esp_netif.h"
#include "nvs.h"

#include "knx/netip/netip_parameter_object.hpp"
#include "knx/util/log.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

using namespace knx;
using namespace knx::product;

namespace habinari_knx::service {

namespace {

// --- KNXnet/IP settings ----------------------------------------------------
//
// The routing group is the one KNXnet/IP setting an installation genuinely
// changes: two KNX projects sharing an IP network have to be told apart, and
// the group is how. It is a device_config item rather than a Kconfig-only
// value because that change happens on site, and the service channel is how a
// device that has not joined the right group yet can still be reached.
//
// ETS can set it too (PID_ROUTING_MULTICAST_ADDRESS, or the Domain Address
// written by serial number), and the stack persists what ETS wrote. The two
// are reconciled as "latest change wins": a service-channel write also leaves
// kKnxIpNvsMulticastPending behind, the next start hands the group to the
// stack as overriding what it has stored, and the flag is cleared once the
// stack has taken it. Without a pending change the stored group stands.
//
// Everything else about the device — the individual address, the group object
// bindings, the parameters — is ETS's, and does not belong here.
constexpr const char *kKnxIpNvsNamespace = "knxip";
constexpr const char *kKnxIpNvsMulticast = "mcast";
constexpr const char *kKnxIpNvsMulticastPending = "mcast_new";

esp_err_t knxIpConfigGet(const device_config_item_t *item, device_config_value_t *out)
{
    out->str[0] = '\0';

    nvs_handle_t handle = 0;
    if (nvs_open(kKnxIpNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
        // Nothing stored yet is a valid answer: the compiled-in default is in
        // force, and resolveMulticastGroup() reports which that is.
        return ESP_OK;
    }
    size_t len = sizeof(out->str);
    if (nvs_get_str(handle, static_cast<const char *>(item->ctx), out->str, &len) != ESP_OK) {
        out->str[0] = '\0';
    }
    nvs_close(handle);
    return ESP_OK;
}

esp_err_t knxIpConfigSet(const device_config_item_t *item, const device_config_value_t *value)
{
    // Refuse a group the device could not join rather than storing it and
    // failing to come up after the reboot this setting asks for: the group is
    // joined at start-up, so a bad one is a device that never appears again.
    // An empty value is not a bad one — it means "use the compiled-in default".
    if (value->str[0] != '\0' && !knx_ip_is_multicast_text(value->str)) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kKnxIpNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(handle, static_cast<const char *>(item->ctx), value->str);
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, kKnxIpNvsMulticastPending, 1u);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

// Whether the service channel changed the group since the stack last took it.
bool multicastChangePending()
{
    nvs_handle_t handle = 0;
    if (nvs_open(kKnxIpNvsNamespace, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    uint8_t pending = 0;
    const bool set = nvs_get_u8(handle, kKnxIpNvsMulticastPending, &pending) == ESP_OK && pending != 0u;
    nvs_close(handle);
    return set;
}

void clearMulticastChangePending()
{
    nvs_handle_t handle = 0;
    if (nvs_open(kKnxIpNvsNamespace, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    if (nvs_erase_key(handle, kKnxIpNvsMulticastPending) == ESP_OK) {
        (void)nvs_commit(handle);
    }
    nvs_close(handle);
}

const device_config_item_t kKnxIpConfigItems[] = {
    {
        .key = "knx.mcast",
        .label = "KNX routing multicast group",
        .unit = nullptr,
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_REBOOT,
        .min = 0.0f,
        .max = 15.0f,  // "255.255.255.255"
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = knxIpConfigGet,
        .set = knxIpConfigSet,
        .ctx = const_cast<char *>(kKnxIpNvsMulticast),
    },
};

// A registered item's current string value, empty when unset.
bool readConfigString(const char *key, char *out, size_t outLen)
{
    out[0] = '\0';
    const device_config_item_t *item = device_config_find(key);
    if (item == nullptr) {
        return false;
    }
    device_config_value_t value{};
    if (device_config_get(item, &value) != ESP_OK || value.str[0] == '\0') {
        return false;
    }
    snprintf(out, outLen, "%s", value.str);
    return true;
}

// The device name an installer set, or the board's default. It is what ETS
// shows in a discovery list, so it should name this unit rather than the model.
std::string resolveFriendlyName()
{
    char name[DEVICE_CONFIG_STRING_MAX] = {0};
    if (readConfigString("dev.name", name, sizeof(name))) {
        return std::string(name);
    }

    // The same fallback every other channel shows: "<prefix> XXYYZZ" from the
    // last three MAC bytes, so a board nobody has named still appears in ETS
    // under something that tells two of them apart.
    uint8_t mac[6] = {};
    if (wifi_service_mac(mac) != ESP_OK) {
        return std::string();
    }
    // 30 octets is the KNXnet/IP friendly-name field; the encoder pads and the
    // BAU trims, so this only has to be big enough not to truncate first.
    char fallback[32] = {0};
    device_default_name(&mac[3], fallback, sizeof(fallback));
    return std::string(fallback);
}

// The product's default group: the service-channel setting, else Kconfig. The
// stack uses it when it has none stored, or when multicastChangePending().
IpAddress resolveMulticastGroup()
{
    char configured[DEVICE_CONFIG_STRING_MAX] = {0};
    if (readConfigString("knx.mcast", configured, sizeof(configured))) {
        const IpAddress parsed = IpAddress::fromString(configured);
        if (!parsed.isZero()) {
            return parsed;
        }
        KNX_LOGW(TAG, "Ignoring unparseable routing group '%s'", configured);
    }
    return IpAddress::fromString(CONFIG_HABINARI_KNXIP_MULTICAST_ADDR);
}

// Publish what the device currently answers on into the KNXnet/IP Parameter
// Object. ETS reads its address and its name from there, and none of it is
// knowable before the interface has a lease — so this runs after start and
// again whenever the address changes. The routing group is not part of it:
// the stack owns PID_ROUTING_MULTICAST_ADDRESS (see the settings above).
void publishIpIdentity(HabinariApp &app)
{
    esp_netif_ip_info_t info{};
    if (wifi_service_ip_info(&info) != ESP_OK || info.ip.addr == 0) {
        KNX_LOGD(TAG, "No address yet — KNXnet/IP parameters not published");
        return;
    }

    netip::KnxNetIpConfiguration config{};
    config.currentIpAddress = IpAddress(info.ip.addr);
    config.currentSubnetMask = IpAddress(info.netmask.addr);
    config.currentDefaultGateway = IpAddress(info.gw.addr);
    config.timeToLive = static_cast<uint8_t>(CONFIG_HABINARI_KNXIP_TTL);
    config.ipAssignmentMethod = netip::ip_assignment::kDhcp;
    // Only what this build actually contains. Claiming Tunnelling here would
    // have ETS offer the device as an interface and then fail to connect to it.
    config.deviceCapabilities = netip::device_capability::kDhcp;
    config.friendlyName = resolveFriendlyName();
    (void)wifi_service_mac(config.macAddress.data());

    const auto result = app.applyKnxNetIpConfiguration(config);
    if (result.isError()) {
        KNX_LOGW(TAG, "KNXnet/IP parameter publish failed: %d", static_cast<int>(result.error()));
        return;
    }

    char text[16] = {0};
    config.currentIpAddress.toString(text);
    KNX_LOGI(TAG, "KNXnet/IP parameters published: %s", text);
}

#if defined(CONFIG_HABINARI_KNXIP_LINK_DIAGNOSTICS)
// What the link counters stood at when last logged. Only the service task
// reads or writes them (serviceMedium).
uint64_t lastLinkReceived = 0;
uint64_t lastLinkDropped = 0;
uint32_t lastLinkPoolDropped = 0;
TickType_t lastLinkLogTick = 0;
#endif

}  // namespace

void prepareMedium()
{
    // Registered before association is attempted, so a device with the wrong
    // credentials — or on the wrong routing group — is still reachable from the
    // service channel to be corrected. That is the whole point of the registry.
    const esp_err_t wifiCfg = wifi_service_register_config();
    if (wifiCfg != ESP_OK) {
        KNX_LOGW(TAG, "Wi-Fi settings not exposed out of band: %s", esp_err_to_name(wifiCfg));
    }
    const esp_err_t knxCfg = device_config_register(
        kKnxIpConfigItems, sizeof(kKnxIpConfigItems) / sizeof(*kKnxIpConfigItems));
    if (knxCfg != ESP_OK) {
        KNX_LOGW(TAG, "KNXnet/IP settings not exposed out of band: %s", esp_err_to_name(knxCfg));
    }

    // Not fatal. An unprovisioned device still boots, still senses, still
    // controls, and can still be given credentials over BLE — which is exactly
    // the situation a device out of the box is in.
    const esp_err_t wifiErr = wifi_service_start();
    if (wifiErr != ESP_OK) {
        KNX_LOGW(TAG, "Wi-Fi not started (%s) — KNXnet/IP will be unreachable until "
                      "credentials are provisioned", esp_err_to_name(wifiErr));
    }
}

util::Result<HabinariProductHandle> startProduct(platform::Esp32Platform &platform,
                                                 HabinariBindings &&bindings)
{
    // The stack needs an interface to join the multicast group on. Waiting is
    // not the same as requiring: the routing endpoint recovers when the
    // interface turns up, so running out of patience costs a slower start
    // rather than a dead device.
    if (CONFIG_HABINARI_KNXIP_WIFI_WAIT_S > 0 && !wifi_service_is_connected()) {
        KNX_LOGI(TAG, "Waiting up to %d s for Wi-Fi", CONFIG_HABINARI_KNXIP_WIFI_WAIT_S);
        g_knx.waitingForNetwork.store(true);
        const esp_err_t waited = wifi_service_wait_connected(CONFIG_HABINARI_KNXIP_WIFI_WAIT_S * 1000u);
        g_knx.waitingForNetwork.store(false);
        if (waited != ESP_OK) {
            KNX_LOGW(TAG, "Starting KNXnet/IP without a network — the device will "
                          "join the routing group once Wi-Fi associates");
        }
    }
    g_knx.startedWithoutNetwork = !wifi_service_is_connected();

    IpRoutingOptions options{};
    options.multicastGroup = resolveMulticastGroup();
    options.multicastGroupOverridesStored = multicastChangePending();
    options.port = NetIpPort(CONFIG_HABINARI_KNXIP_PORT);
    options.timeToLive = static_cast<uint8_t>(CONFIG_HABINARI_KNXIP_TTL);
    options.friendlyName = resolveFriendlyName();

    char groupText[16] = {0};
    options.multicastGroup.toString(groupText);
    KNX_LOGI(TAG, "KNXnet/IP routing, default group %s%s, port %d (TTL %d), announced as \"%s\"",
             groupText, options.multicastGroupOverridesStored ? " (set on the service channel)" : "",
             CONFIG_HABINARI_KNXIP_PORT, CONFIG_HABINARI_KNXIP_TTL,
             options.friendlyName.c_str());

    return startCommissionedProduct(platform, kHabinariProduct, std::move(bindings),
                                    std::move(options));
}

void mediumStarted(HabinariApp &app)
{
    // The stack has stored the service channel's group by now (it saves it
    // before start returns), so the request has been carried out.
    if (multicastChangePending()) {
        clearMulticastChangePending();
    }
    {
        char groupText[16] = {0};
        app.routingMulticastAddress().toString(groupText);
        KNX_LOGI(TAG, "Routing group in force: %s", groupText);
    }

    // The address is published now and again on every change: a DHCP renewal
    // that moves the device would otherwise leave ETS reading back an address
    // it no longer answers on.
    publishIpIdentity(app);
    wifi_service_set_event_callback(
        [](bool connected, void *ctx) {
            if (!connected) {
                return;
            }
            // Runs on the default event loop, which must not block. Waking the
            // service task is enough; it republishes on its next pass.
            auto *state = static_cast<KnxAdapterState *>(ctx);
            if (state == nullptr) {
                return;
            }
            state->ipIdentityDirty.store(true);
            const TaskHandle_t handle = state->taskHandle.load();
            if (handle != nullptr) {
                xTaskNotifyGive(handle);
            }
        },
        &g_knx);

    // The association that beat the callback registration: it raised no event
    // anyone was listening for, and the socket's own join attempt had already
    // failed. Nothing else would ever ask for the membership again.
    if (g_knx.startedWithoutNetwork && wifi_service_is_connected()) {
        g_knx.ipIdentityDirty.store(true);
    }

    KNX_LOGI(TAG, "ETS-commissionable KNXnet/IP sensor bridge started");
}

void serviceMedium(HabinariApp &app)
{
    if (g_knx.ipIdentityDirty.exchange(false)) {
        // The membership first, the identity second. The join is what makes
        // the device hear the group at all — it is established against
        // whichever interface exists when the routing socket opens, and on
        // a device that booted before the access point answered, that was
        // no interface. Publishing an address nobody can reach us on would
        // be the more visible half of the same event and the less useful.
        const auto rejoin = app.refreshIpMulticastMembership();
        if (rejoin.isError()) {
            KNX_LOGW(TAG, "Routing group rejoin failed: %d",
                     static_cast<int>(rejoin.error()));
        } else {
            KNX_LOGI(TAG, "Rejoined the routing group on the current interface");
        }
        publishIpIdentity(app);
    }

#if defined(CONFIG_HABINARI_KNXIP_LINK_DIAGNOSTICS)
    // The question a capture taken on the network cannot answer: when the
    // device stops answering a client, did the telegram reach it at all?
    // A datagram count standing still while ETS is plainly transmitting is
    // a delivery problem below the stack; a rising drop count is this
    // device discarding what it did receive. Rate-limited to a second, and
    // silent while nothing moves, so it can be left on for a download.
    {
        const TickType_t nowTick = xTaskGetTickCount();
        if ((nowTick - lastLinkLogTick) >= pdMS_TO_TICKS(kControlIntervalMs)) {
            const auto link = app.linkDiagnostics();
            if (link.mediumDatagramsReceived != lastLinkReceived
                || link.mediumDatagramsDropped != lastLinkDropped
                || link.rxPoolDropped != lastLinkPoolDropped) {
                lastLinkReceived = link.mediumDatagramsReceived;
                lastLinkDropped = link.mediumDatagramsDropped;
                lastLinkPoolDropped = link.rxPoolDropped;
                lastLinkLogTick = nowTick;
                KNX_LOGI(TAG,
                         "Routing link: %llu datagrams in (%llu dropped, %u ms since "
                         "the last), data link: %u frames, %u filtered, %u pool-dropped, "
                         "%u undecodable",
                         static_cast<unsigned long long>(link.mediumDatagramsReceived),
                         static_cast<unsigned long long>(link.mediumDatagramsDropped),
                         static_cast<unsigned>(link.msSinceLastInbound),
                         static_cast<unsigned>(link.rxFrames),
                         static_cast<unsigned>(link.rxFilterDropped),
                         static_cast<unsigned>(link.rxPoolDropped),
                         static_cast<unsigned>(link.rxDecodeFailed));
            }
        }
    }
#endif
}

}  // namespace habinari_knx::service
