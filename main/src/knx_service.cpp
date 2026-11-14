// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// The KNX adapter: its descriptor, the start-up handshake with app_main, and
// the service task that runs the stack. The task is a sequence of calls into
// the pieces listed in knx_service_internal.hpp; what each does lives there.

#include "knx_service_internal.hpp"

#include "control_service.h"
#include "control_service.hpp"
#include "knx_product.hpp"
#include "knx_publish_schedule.hpp"
#include "protocol_adapter.h"

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "knx/platform/esp32_platform.hpp"
#include "knx/product/commissioned_product.hpp"
#include "knx/util/log.hpp"

#include <cstdint>
#include <type_traits>
#include <utility>

using namespace knx;
using namespace knx::application;
using namespace knx::product;

namespace habinari_knx::service {

habinari::control::ServiceState &g_state = habinari::control::state();
KnxAdapterState g_knx;

namespace {

namespace control = habinari::control;
using control::LockGuard;

// PublishSchedule compares ticks with 32-bit wrap-around arithmetic.
static_assert(std::is_same_v<TickType_t, uint32_t>, "TickType_t must be 32 bits");

// kKnxServiceTaskStackSize (knx_service_internal.hpp) is 32 KB because of
// start-up, not the service loop. The commissioned runtime is ~53 KB for this
// product and is built directly on the heap by startCommissionedProduct(), so
// it never crosses this stack; what remains here is the bindings builder (the
// buildAndStartProduct() frame: 10,240 B on TP1, 13,696 B on knxip by
// -fstack-usage, November 2026) and the BAU init call tree with
// NVS/crypto/logging under it.
//
// Everything on that path that is sized by the port and parameter counts —
// the builder, the endpoint bindings, the parameter callbacks — is now handed
// down by rvalue reference rather than by value, so it exists once instead of
// once per frame it passes through. Measured frames before that change, at 61
// ports / 73 parameters: knxServiceTask 19,040 B + startCommissionedProduct
// 96 B + make_unique 9,008 B + the runtime constructor 5,920 B = 34,064 B,
// which overflowed a 32 KB stack during boot (the C6 hardware stack guard
// trapped it inside the constructor prologue).
//
// The builder now lives in buildAndStartProduct()'s frame rather than the
// task's, so the task's own frame no longer carries it through the service
// loop. The high-water mark is logged at the end of start-up, on every
// lifecycle change and every 10 minutes (knx_stack_watch.cpp).

// How long knxAdapterStart() waits for the service task to report whether the
// product started. Start-up itself (NVS load, BAU init) takes well under a
// second. The KNXnet/IP wait for Wi-Fi is deliberately not part of the
// allowance: app_main starts the BLE service channel after the adapters, and
// that channel is how a device with wrong Wi-Fi credentials gets corrected.
// Holding it back for the whole Wi-Fi wait would delay the repair by exactly
// the outage it is meant to repair.
constexpr uint32_t kKnxStartTimeoutMs = 5000u;

// Tells knxAdapterStart() how start-up went: ESP_OK once the product is
// running, or the error that stopped it — if it is still waiting. A result
// nobody waits for any more is only logged, and only when it is a failure:
// that is the case of a KNXnet/IP start that outlasted the boot sequence
// waiting for Wi-Fi (see kKnxStartTimeoutMs).
void reportStartResult(esp_err_t result)
{
    const TaskHandle_t waiter = g_knx.startWaiter.exchange(nullptr);
    if (waiter != nullptr) {
        (void)xTaskNotify(waiter, static_cast<uint32_t>(result), eSetValueWithOverwrite);
    } else if (result != ESP_OK) {
        KNX_LOGE(TAG, "KNX failed to start after boot had moved on: %s", esp_err_to_name(result));
    }
}

esp_err_t toEspErr(util::ErrorCode error)
{
    // The detail is in the log line above this; the code only has to be
    // something esp_err_to_name() can say and the registry can refuse on.
    switch (error) {
    case util::ErrorCode::Timeout:
        return ESP_ERR_TIMEOUT;
    case util::ErrorCode::OperationNotSupported:
        return ESP_ERR_NOT_SUPPORTED;
    default:
        return ESP_FAIL;
    }
}

// Assemble the bindings and start the product on this build's medium.
//
// A function of its own so that its frame — the bindings builder, 10–14 KB for
// a product this size — is gone before the service loop runs. noinline keeps the
// compiler from folding it back into the task, where the builder would occupy
// the task's frame for good.
[[gnu::noinline]] util::Result<HabinariProductHandle>
buildAndStartProduct(platform::Esp32Platform &platform)
{
    // Declared first, then registered onto as an lvalue. Writing this as
    // `auto bindings = makeCommissionedBindings(...).provideState(...)...`
    // would put two builders in this frame — the chain's temporary and the
    // named object initialised from it.
    auto bindings = makeCommissionedBindings(kHabinariProduct);
    bindGroupObjectReads(bindings);
    bindGroupObjectWrites(bindings);
    bindParameters(bindings);
    bindings
        .onProgrammingModeChanged([](bool enabled) {
            KNX_LOGI(TAG, "Programming mode: %s", enabled ? "ON" : "OFF");
            // ETS can enter programming mode over the bus, not only via the
            // button, so the service hears about it either way and the board LED
            // has one thing to follow.
            control_service_set_programming_mode(enabled);
        })
        .onLifecycleChanged([](DeviceLifecycleState state) {
            KNX_LOGI(TAG,
                     "Lifecycle: %s",
                     state == DeviceLifecycleState::Operational   ? "Operational" :
                     state == DeviceLifecycleState::Commissioning ? "Commissioning" :
                                                                  "Uncommissioned");
            // A download or a reset is where the stacks peak; the service
            // loop logs them on its next pass rather than from this context.
            g_knx.stackCheckPending.store(true);
            const TaskHandle_t handle = g_knx.taskHandle.load();
            if (handle != nullptr) {
                xTaskNotifyGive(handle);
            }
        })
        .onFault([](FaultInfo info) {
            KNX_LOGE(TAG,
                     "Fault: code=%d detail=%s",
                     static_cast<int>(info.code),
                     info.detail != nullptr ? info.detail : "");
        });

    // startProduct() builds the runtime on the heap and hands back an owning
    // handle: at ~34 KB it cannot travel through a stack frame.
    return startProduct(platform, std::move(bindings));
}

void logIndividualAddress(HabinariApp &app)
{
    const auto ownAddress = app.individualAddress();
    KNX_LOGI(TAG,
             "KNX own individual address: 0x%04X (%s)",
             ownAddress.raw,
             isInitialIndividualAddress(ownAddress) ? "initial/uncommissioned" :
             isIndividualBroadcastAddress(ownAddress) ? "broadcast" :
             isOperationalIndividualAddress(ownAddress) ? "operational" :
             "unknown");
}

bool isOperational(const HabinariApp &app)
{
    return app.lifecycleState() == DeviceLifecycleState::Operational;
}

// One pass of the service loop after the stack's own loop(): the medium's
// housekeeping, then the transmit policy and publication.
void servicePass(HabinariApp &app, PublishSchedule &schedule, uint32_t startupJitterMs)
{
    serviceMedium(app);
    serviceStackWatch();

    if (g_knx.controlTickPending.exchange(false)) {
        refreshTransmitPolicies(app);
        schedule.controlTick();
    }

    // The lifecycle as it stands before the button is acted on below; see
    // PublishSchedule::begin for why that is the reading that counts.
    const auto pass = schedule.begin(xTaskGetTickCount(), isOperational(app));
    if (pass.reportUnlinkedPorts) {
        reportUnlinkedPorts(app);
    }
    if (pass.announceHold) {
        KNX_LOGI(TAG, "Holding initial KNX publish for %u ms (boot-storm mitigation)",
                 static_cast<unsigned>(startupJitterMs));
    }
    // A programming-mode toggle (button) is honoured on every pass, held
    // publish or not, Operational or not.
    if (control::takeProgrammingModeToggleRequest()) {
        app.toggleProgrammingMode();
    }
    // Offered once per control tick; the per-object transmit policy decides
    // what is actually worth a telegram.
    if (pass.publish) {
        publishAllState(app);
    }
}

void knxServiceTask(void *arg)
{
    (void)arg;
    // First, before anything that could lead to a notification: the control
    // tick and the stack's work callback both look this up to wake us.
    g_knx.taskHandle.store(xTaskGetCurrentTaskHandle());

    // DEBUG shows per-frame bus traffic (RX/TX dumps, DL-ACK diagnostics,
    // connection events) — the development default while the stack is being
    // brought up. Info for production builds: INFO carries only state changes
    // (address, lifecycle, load state) and WARN/ERROR faults. The esp_log
    // levels this routes through are set for the whole image in main.c.
    knx::log::setLevel(knx::log::Level::Info);

    platform::Esp32Platform platform;

    auto appResult = buildAndStartProduct(platform);
    if (appResult.isError()) {
        KNX_LOGE(TAG, "KNX start failed: %s", util::errorCodeToString(appResult.error()));
        g_knx.taskHandle.store(nullptr);
        reportStartResult(toEspErr(appResult.error()));
        vTaskDelete(nullptr);
        return;
    }
    // Already heap-constructed by startCommissionedProduct — just take ownership.
    const HabinariProductHandle appPtr = std::move(appResult.value());
    auto &app = *appPtr;

    // The ETS device certificate encodes the serial number together with the
    // key, so the serial comes first.
    const KnxSerialNumber serial = assignSerialNumber(app);
    configureTransmitShaping(app);
    setUpDataSecure(app, serial);
    logIndividualAddress(app);
    seedStateFromParameters(app);
    mediumStarted(app);

    // The control loops do not run here any more — control_service.cpp owns
    // them, and this task is told when a cycle has completed (see
    // knxAdapterOnControlTick). That is what lets a Modbus-only or MQTT-only
    // image control the room with no KNX task in the picture at all.
    constexpr TickType_t kControlIntervalTicks = pdMS_TO_TICKS(kControlIntervalMs);

    // Event-driven wake: the KNX stack signals when new work appears (an inbound
    // frame decoded by the RX worker task, a queued group auto-response, a
    // pending publish action). We turn that signal into a task notification so
    // this loop can park instead of busy-polling a mostly-empty queue every few
    // ms — that idle spinning wasted CPU and, more importantly, stole cycles
    // from the RX worker whose blocking management-response sends run there.
    // The callback fires from the RX worker or this task's context (never an
    // ISR), so xTaskNotifyGive is correct; the notification latches, so a signal
    // raised while we are mid-loop is coalesced into the next wake, not lost.
    app.setWorkAvailableCallback([]() {
        const TaskHandle_t handle = g_knx.taskHandle.load();  // set once at startup
        if (handle != nullptr) {
            xTaskNotifyGive(handle);
        }
    });

    // While the stack still has in-flight polled work (e.g. a group auto-response
    // progressing through its send state machine) we wake at this short cadence
    // to drive it to completion; when fully idle we instead sleep until the next
    // control tick or the next work notification, whichever comes first.
    const TickType_t kActiveWorkPollTicks =
        (pdMS_TO_TICKS(2) > 0) ? pdMS_TO_TICKS(2) : 1;

    // Boot-storm mitigation: hold the first full publish for a per-device random
    // delay so a bus-wide power-up does not make every device transmit at t=0.
    // The rate limiter (configureTransmitShaping) additionally spreads whatever
    // burst does occur.
    const uint32_t startupJitterMs = startupPublishJitterMs();
    PublishSchedule schedule(xTaskGetTickCount() + pdMS_TO_TICKS(startupJitterMs));

    // Startup is this task's stack peak, and it is now behind us.
    logStackHeadroom("start-up");

    reportStartResult(ESP_OK);

    for (;;) {
        app.loop();
        servicePass(app, schedule, startupJitterMs);

        // Park until the stack signals new work or the control task announces
        // the next cycle — no fixed busy-poll. If polled work is still in
        // flight, wake at the short active cadence to service it to completion;
        // otherwise sleep for a whole control interval. Either a work
        // notification (an inbound frame) or the tick notification cuts the
        // sleep short, and both latch, so neither is ever missed. The interval
        // is a ceiling now rather than a schedule — the control task owns the
        // cadence, and this only bounds how long we sleep if it stops.
        const TickType_t blockTicks = app.ownerWorkHint().hasImmediateWork()
            ? kActiveWorkPollTicks
            : kControlIntervalTicks;
        (void)ulTaskNotifyTake(pdTRUE, blockTicks);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// The adapter descriptor — the KNX personality's entire public surface.
//
// NVS initialisation and factory reset belong to main.c and the control
// service, so KNX is one optional adapter among several; what is here is the
// mapping and nothing else.
// ---------------------------------------------------------------------------

namespace {

esp_err_t knxAdapterStart()
{
    if (g_knx.started) {
        return ESP_OK;
    }
    // NVS and the control state are already up: control_service_start() ran
    // first, which is the ordering protocol_adapters_start_all() guarantees.
    if (g_state.mutex == nullptr) {
        KNX_LOGE(TAG, "Control service is not running");
        return ESP_ERR_INVALID_STATE;
    }

    // Anything the medium needs before the task exists. On KNXnet/IP that is
    // the Wi-Fi station and its out-of-band settings, registered before
    // association is attempted so a device with the wrong credentials is still
    // reachable from the service channel to be corrected.
    prepareMedium();
    registerStackDiagnostics();

    // Wait for the task to say whether the product actually started. Creating
    // the task used to count as success, so a stack that then failed to come
    // up deleted its task in silence and the required-adapter check never saw
    // it. The task reports through this task's notification value.
    (void)xTaskNotifyStateClear(nullptr);
    (void)ulTaskNotifyValueClear(nullptr, UINT32_MAX);
    g_knx.startWaiter.store(xTaskGetCurrentTaskHandle());
    const BaseType_t created = xTaskCreate(knxServiceTask, "knx_service",
                                           kKnxServiceTaskStackSize,
                                           nullptr, 8, nullptr);
    if (created != pdPASS) {
        g_knx.startWaiter.store(nullptr);
        return ESP_ERR_NO_MEM;
    }

    uint32_t result = 0;
    TickType_t wait = pdMS_TO_TICKS(kKnxStartTimeoutMs);
    while (xTaskNotifyWait(0, UINT32_MAX, &result, wait) != pdTRUE) {
        if (g_knx.startWaiter.exchange(nullptr) == nullptr) {
            // The task took the waiter just before we gave up; its report is
            // on its way, so take it rather than return without it.
            wait = portMAX_DELAY;
            continue;
        }
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
        if (g_knx.waitingForNetwork.load()) {
            KNX_LOGW(TAG, "KNX start-up is waiting for Wi-Fi; carrying on with boot. "
                          "A start-up failure from here on is only logged");
            g_knx.started = true;
            return ESP_OK;
        }
#endif
        KNX_LOGE(TAG, "KNX did not report start-up within %u ms",
                 static_cast<unsigned>(kKnxStartTimeoutMs));
        return ESP_ERR_TIMEOUT;
    }
    if (static_cast<esp_err_t>(result) != ESP_OK) {
        return static_cast<esp_err_t>(result);
    }

    g_knx.started = true;
    return ESP_OK;
}

// Called from the control task. Sets a flag and wakes the service loop; it must
// not do any KNX work itself, because the control task has no business blocking
// on a bus with a 104 us bit cell.
void knxAdapterOnControlTick(uint32_t)
{
    g_knx.controlTickPending.store(true);
    const TaskHandle_t handle = g_knx.taskHandle.load();
    if (handle != nullptr) {
        xTaskNotifyGive(handle);
    }
}

bool knxAdapterIdentifyActive()
{
    if (g_state.mutex == nullptr) {
        return false;
    }
    LockGuard lock(g_state.mutex);
    return g_state.programmingModeActive;
}

}  // namespace
}  // namespace habinari_knx::service

using namespace habinari_knx::service;

extern "C" const protocol_adapter_t knx_protocol_adapter = {
#if defined(CONFIG_HABINARI_KNX_MEDIUM_IP)
    .name = "knx-ip",
#else
    .name = "knx-tp1",
#endif
    .start = knxAdapterStart,
    .on_control_tick = knxAdapterOnControlTick,
    // No on_sensor_data hook: measurements are published from the control tick
    // under the transmit policy, which is what decides whether a new reading is
    // worth a telegram. Publishing straight off the sensor task would bypass it.
    .on_sensor_data = nullptr,
    .identify_active = knxAdapterIdentifyActive,
    // KNX owns programming mode wherever it is compiled in. The flag lives in
    // the stack's device object, ETS can set it over the bus, and the stack is
    // only safe to touch from the KNX task — so the control service must not
    // toggle it behind the stack's back. This adapter consumes the button
    // request on its own task and reports the result back through
    // control_service_set_programming_mode().
    .owns_programming_mode = true,
    // Required: on a board wired to a TP1 bus, a device that silently fails to
    // join it is worse than one that refuses to boot and says why.
    //
    // The same holds on IP, and for the same reason — but note that "started"
    // there means the service task was created, not that the network exists.
    // A KNXnet/IP device that boots before its access point is a normal state,
    // not a failure, so it is not one this flag should turn into a dead boot.
    .required = true,
};
