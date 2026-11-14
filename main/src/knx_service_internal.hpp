// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#pragma once

/**
 * @file knx_service_internal.hpp
 * @brief The pieces of the KNX adapter, and what they share.
 *
 * The adapter is one FreeRTOS task built from these parts, each in its own
 * translation unit:
 *
 *   knx_service.cpp         the adapter descriptor, start-up handshake and task
 *   knx_group_reads.cpp     group-object read responses (provideState)
 *                           — the same values publication sends
 *   knx_group_writes.cpp    inbound group writes -> control_state_write()
 *   knx_parameters.cpp      ETS parameters -> control state (knx_parameter_table.hpp)
 *   knx_publish.cpp         outbound publication and transmit shaping
 *   knx_data_secure.cpp     serial number, factory tool key, device certificate
 *   knx_stack_watch.cpp     stack headroom of the KNX tasks
 *   knx_medium_tp1.cpp /
 *   knx_medium_ip.cpp       the transport — the only thing that differs by medium
 *
 * Private to the adapter: nothing outside these files includes it.
 */

#include "control_service.hpp"
#include "knx_port_values.hpp"
#include "knx_product.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "knx/platform/esp32_platform.hpp"
#include "knx/product/commissioned_product.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace habinari_knx::service {

inline constexpr const char *TAG = "knx_service";

// The publish cadence ceiling, and the link-diagnostics log interval. The
// control task owns the real cadence; see knxServiceTask.
inline constexpr uint32_t kControlIntervalMs = 1000;

// The service task's stack. Start-up, not the service loop, sets its peak; see
// knxServiceTask for what is on it, and knx_stack_watch.cpp for how it is
// measured. Retune from the logged worst case, not from guesswork.
inline constexpr uint32_t kKnxServiceTaskStackSize = 32 * 1024;

using HabinariProductHandle =
    knx::product::CommissionedProductHandle<std::remove_cvref_t<decltype(kHabinariProduct)>,
                                            knx::product::kDefaultBindingCapacity>;
using HabinariBindings =
    knx::product::CommissionedBindingsBuilder<std::remove_cvref_t<decltype(kHabinariProduct)>,
                                              knx::product::kDefaultBindingCapacity>;
using HabinariApp = HabinariProductHandle::element_type;

// The device's own state — settings, installation inputs, control outputs —
// lives in control_service.cpp, not here. This adapter reads and writes it
// through the same handle every other adapter uses; the only thing it still
// owns privately is its task.
//
// Binding a reference is safe before control_service_start(): state() returns a
// constant-initialised namespace-scope object, and `mutex` being null is the
// documented "service not running yet" case that LockGuard tolerates.
extern habinari::control::ServiceState &g_state;  // knx_service.cpp

// What only the KNX personality has.
struct KnxAdapterState {
    // Written by the service task as its first act, so that anything able to
    // notify it — the control tick, the stack's work callback, the Wi-Fi event
    // — finds the handle already there. Read from those other tasks, hence
    // atomic.
    std::atomic<TaskHandle_t> taskHandle{nullptr};
    bool started{false};
    // The task waiting in knxAdapterStart() for the start-up result, or null
    // once nobody is. Taken with an exchange by whichever side gets there
    // first: the service task to report, or knxAdapterStart() giving up. A
    // report can then never be sent to an app_main that has already returned
    // and been deleted.
    std::atomic<TaskHandle_t> startWaiter{nullptr};
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
    // True while startProduct() waits for Wi-Fi. knxAdapterStart() stops
    // waiting for a start-up that is only waiting for the network.
    std::atomic<bool> waitingForNetwork{false};
#endif
    // Raised by the control task through the adapter's on_control_tick hook,
    // consumed by the service loop with an exchange, so a tick raised between
    // the read and the clear is never lost.
    std::atomic<bool> controlTickPending{true};
    // Raised on a lifecycle change (from whichever task reports it), consumed
    // by the service loop, which logs the KNX tasks' stack headroom.
    std::atomic<bool> stackCheckPending{false};
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
    // Raised from the Wi-Fi event loop when the interface gains an address, so
    // the service task republishes what ETS reads back. Consumed the same way
    // as controlTickPending above.
    std::atomic<bool> ipIdentityDirty{false};
    // True when the routing socket opened with no interface to join the group
    // on. Set before the socket exists and read once the event callback is
    // installed, to close the gap between the two: an interface that arrived in
    // between leaves no event to fire and would strand the device off the group.
    bool startedWithoutNetwork{false};
#endif
};

extern KnxAdapterState g_knx;

// --- knx_group_reads.cpp / knx_group_writes.cpp ------------------------------

/// Read requests: every transmitting port answers from the control state.
void bindGroupObjectReads(HabinariBindings &bindings);
/// Inbound group writes: commands, neighbour inputs and changeover inputs.
void bindGroupObjectWrites(HabinariBindings &bindings);

// --- knx_parameters.cpp ------------------------------------------------------

/// Live onParameterChanged bindings, one per kParameterTable row.
void bindParameters(HabinariBindings &bindings);
/// Seed the control state from the parameters the product loaded at start-up.
void seedStateFromParameters(HabinariApp &app);

// --- knx_publish.cpp ---------------------------------------------------------

/// Monotonic time source and global outbound rate limit.
void configureTransmitShaping(HabinariApp &app);
/// Re-derive every transmitting port's transmit policy from the settings.
void refreshTransmitPolicies(HabinariApp &app);
/// Every transmitting port's value, from one copy of the state taken under the
/// control lock. Both the publish path and the read responses use it, so a
/// read answers exactly what publication sends (knx_port_values.hpp).
PortValues currentPortValues();
/// Offer every transmitting port; the transmit policy decides what is sent.
void publishAllState(HabinariApp &app);
/// One-shot inventory of transmitting objects ETS linked no group address to.
void reportUnlinkedPorts(HabinariApp &app);
/// The per-device random delay before the first full publish, in ms.
uint32_t startupPublishJitterMs();

// --- knx_data_secure.cpp -----------------------------------------------------

/// The KNX serial number (device object PID 11), from the base MAC.
struct KnxSerialNumber {
    uint8_t bytes[6]{};
    bool valid{false};
};

/// Where the factory tool key came from. The two differ in shelf life: the
/// eFuse-derived FDSK is fixed for the life of the chip, while the NVS key is
/// erased by a factory reset.
enum class ToolKeySource : uint8_t { EfuseDerived, NvsFallback };

/// The device's KNX Data Secure factory tool key (FDSK), resolved before the
/// stack starts.
struct DataSecureFactoryKey {
    std::array<uint8_t, 16> key{};
    ToolKeySource source{ToolKeySource::NvsFallback};
    /// The NVS fallback key was generated on this boot.
    bool created{false};
    bool valid{false};
};

/// Read the serial number from the base MAC and log it. Before start.
KnxSerialNumber readSerialNumber();
/// Resolve the FDSK: eFuse-derived, else the NVS fallback. Before start.
DataSecureFactoryKey resolveDataSecureFactoryKey(const KnxSerialNumber &serial);
/// The product's ex-factory Data Secure state: the FDSK as factory Tool Key,
/// Security Mode off until ETS turns it on. A Tool Key ETS stored wins over
/// it at every start, and a Factory Reset returns to it.
knx::product::SecureCommissioningOptions dataSecureFactoryDefaults(const DataSecureFactoryKey &factoryKey);
/// Set the serial number on the started device object.
void assignSerialNumber(HabinariApp &app, const KnxSerialNumber &serial);
/// Log the Data Secure state, the factory key and the ETS device certificate
/// while they are still usable.
void reportDataSecure(HabinariApp &app, const KnxSerialNumber &serial,
                      const DataSecureFactoryKey &factoryKey);

// --- knx_stack_watch.cpp -----------------------------------------------------

/// The read-only "knx.stack" item: least free stack seen, per KNX task.
void registerStackDiagnostics();
/// Log each KNX task's stack high-water mark and the worst seen; warn under 1 KB.
void logStackHeadroom(const char *why);
/// Once per loop pass: log on a lifecycle change and every 10 minutes.
void serviceStackWatch();

// --- knx_medium_tp1.cpp / knx_medium_ip.cpp ----------------------------------

/// Before the service task exists: anything the medium needs from app_main's
/// context. On IP, the out-of-band settings and the Wi-Fi station.
void prepareMedium();
/// Bring the transport up and start the product on it.
knx::util::Result<HabinariProductHandle> startProduct(knx::platform::Esp32Platform &platform,
                                                      HabinariBindings &&bindings,
                                                      const knx::product::SecureCommissioningOptions &secure);
/// After start, before the loop: anything the medium publishes or subscribes to.
void mediumStarted(HabinariApp &app);
/// Once per loop pass, after the stack's own loop().
void serviceMedium(HabinariApp &app);

}  // namespace habinari_knx::service
