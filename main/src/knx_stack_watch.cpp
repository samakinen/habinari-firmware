// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

// Stack headroom of the KNX tasks: the service task and the stack's receive
// task. The receive task is "knx_tp1_rx" on both media — KNXnet/IP routing
// runs through the same data link layer, whose task waits in the socket — and
// it unwraps KNX Secure, dispatches management and sends the response, so it
// is the one most likely to run short. Sizing rule and bench procedure:
// docs/code-review-2026-10-fix-design.md, section 9, "3.8h".

#include "knx_service_internal.hpp"

#include "device_config.h"

#include "esp_err.h"

#include "knx/config.hpp"
#include "knx/util/log.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>

namespace habinari_knx::service {

namespace {

constexpr uint32_t kStackWarnBytes = 1024;
constexpr TickType_t kStackLogInterval = pdMS_TO_TICKS(10u * 60u * 1000u);
constexpr uint32_t kUnseen = UINT32_MAX;

struct WatchedTask {
    const char *name;
    uint32_t size;
    // The lowest high-water mark seen, across restarts of the task. Written by
    // the service task, read by the service channel.
    std::atomic<uint32_t> minFree{kUnseen};
};

WatchedTask g_watched[] = {
    {"knx_service", kKnxServiceTaskStackSize},
    {"knx_tp1_rx", knx::config::RX_TASK_STACK_SIZE},
};

// Only the service task reads or writes it.
TickType_t g_lastLogTick = 0;

// "knx.stack" on the service channel: the worst headroom seen, per task.
esp_err_t stackConfigGet(const device_config_item_t *, device_config_value_t *out)
{
    const uint32_t service = g_watched[0].minFree.load();
    const uint32_t rx = g_watched[1].minFree.load();
    (void)snprintf(out->str, sizeof(out->str), "svc %ld rx %ld bytes free",
                   service == kUnseen ? -1L : static_cast<long>(service),
                   rx == kUnseen ? -1L : static_cast<long>(rx));
    return ESP_OK;
}

const device_config_item_t kStackConfigItems[] = {
    {
        .key = "knx.stack",
        .label = "KNX task stack, least free (-1 = not seen yet)",
        .unit = nullptr,
        .type = DEVICE_CONFIG_TYPE_STRING,
        .flags = DEVICE_CONFIG_FLAG_READ_ONLY,
        .min = 0.0f,
        .max = 0.0f,
        .enum_labels = nullptr,
        .enum_count = 0,
        .get = stackConfigGet,
        .set = nullptr,
        .ctx = nullptr,
    },
};

}  // namespace

// Looked up by name every time rather than cached: the stack deletes and
// recreates its receive task when the data link is closed and reopened.
void logStackHeadroom(const char *why)
{
    g_lastLogTick = xTaskGetTickCount();
    for (WatchedTask &task : g_watched) {
        const TaskHandle_t handle = xTaskGetHandle(task.name);
        if (handle == nullptr) {
            continue;
        }
        // Bytes on ESP-IDF, and already the minimum since the task started.
        const uint32_t free = static_cast<uint32_t>(uxTaskGetStackHighWaterMark(handle));
        if (free < task.minFree.load()) {
            task.minFree.store(free);
        }
        const uint32_t worst = task.minFree.load();
        if (worst < kStackWarnBytes) {
            KNX_LOGW(TAG, "Stack %s (%s): %u of %u bytes free, worst %u, under 1 KB",
                     task.name, why, static_cast<unsigned>(free),
                     static_cast<unsigned>(task.size), static_cast<unsigned>(worst));
        } else {
            KNX_LOGI(TAG, "Stack %s (%s): %u of %u bytes free, worst %u",
                     task.name, why, static_cast<unsigned>(free),
                     static_cast<unsigned>(task.size), static_cast<unsigned>(worst));
        }
    }
}

void registerStackDiagnostics()
{
    const esp_err_t err = device_config_register(
        kStackConfigItems, sizeof(kStackConfigItems) / sizeof(*kStackConfigItems));
    if (err != ESP_OK) {
        KNX_LOGW(TAG, "Stack diagnostics not exposed out of band: %s", esp_err_to_name(err));
    }
}

void serviceStackWatch()
{
    if (g_knx.stackCheckPending.exchange(false)) {
        logStackHeadroom("lifecycle change");
    } else if ((xTaskGetTickCount() - g_lastLogTick) >= kStackLogInterval) {
        logStackHeadroom("periodic");
    }
}

}  // namespace habinari_knx::service
