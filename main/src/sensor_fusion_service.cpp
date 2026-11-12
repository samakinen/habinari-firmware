// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025-2026 Sami Mäkinen

#include "sensor_fusion_service.h"

#include "sensor_fusion_pipeline.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// The C entry points: one fusion pipeline (sensor_fusion_pipeline.cpp) and the
// lock every call into it holds. The fusion itself is platform-free and
// host-tested; what is left here is the locking and the logging.

namespace {

static const char *TAG = "sensor_fusion";

/// Guards g_pipeline. Held for a whole update, not just the config copy:
/// configure() rewrites the channel and detector configurations the update is
/// reading, so a configure from the control task in the middle of a cycle
/// would otherwise be a data race on half of the pipeline.
SemaphoreHandle_t g_mutex = nullptr;
habinari::fusion::Pipeline g_pipeline;

/// RAII hold of the fusion lock. The lock exists from sensor_fusion_init() on;
/// entry points refuse to run before then rather than run unlocked.
class FusionLock {
public:
    FusionLock()
        : held_(g_mutex != nullptr && xSemaphoreTake(g_mutex, portMAX_DELAY) == pdTRUE)
    {
    }
    ~FusionLock()
    {
        if (held_) {
            xSemaphoreGive(g_mutex);
        }
    }
    FusionLock(const FusionLock &) = delete;
    FusionLock &operator=(const FusionLock &) = delete;

    explicit operator bool() const { return held_; }

private:
    bool held_;
};

} // namespace

extern "C" esp_err_t sensor_fusion_init(void)
{
    if (g_mutex != nullptr) {
        return ESP_OK;
    }
    g_mutex = xSemaphoreCreateMutex();
    if (g_mutex == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    sensor_fusion_config_t defaults;
    sensor_fusion_default_config(&defaults);
    sensor_fusion_configure(&defaults);
    return ESP_OK;
}

extern "C" void sensor_fusion_configure(const sensor_fusion_config_t *config)
{
    if (config == nullptr) {
        return;
    }
    FusionLock lock;
    if (!lock) {
        ESP_LOGE(TAG, "sensor_fusion_configure before sensor_fusion_init; ignored");
        return;
    }
    g_pipeline.configure(*config);
}

extern "C" void sensor_fusion_acknowledge_alarms(void)
{
    // Called from the control task; the fire detector's latch is mid-update on
    // the sensor task often enough that this needs the lock like everything else.
    FusionLock lock;
    if (!lock) {
        return;
    }
    g_pipeline.acknowledgeAlarms();
}

extern "C" void sensor_fusion_update(const sensor_bus_results_t *bus,
                                     const ext_probe_results_t *probe,
                                     float dt_seconds,
                                     sensor_data_t *out)
{
    if (out == nullptr) {
        return;
    }
    // The whole cycle under the lock (see g_mutex). Pure arithmetic on a few
    // dozen floats, so the control task waits on it for well under a
    // millisecond at most.
    FusionLock lock;
    if (!lock) {
        // Not initialised: report nothing rather than fuse unlocked.
        *out = sensor_data_t{};
        return;
    }
    g_pipeline.update(bus, probe, dt_seconds, out);

    // One line per cycle at Debug; the fault transitions that matter are logged
    // by the KNX diagnostics objects, not here.
    ESP_LOGD(TAG,
             "T=%.2f(src%u%s) RH=%.2f(src%u) CO2=%.0f dT=%.2fK/min healthy=0x%02x suspect=0x%02x",
             out->temperature.value, out->temperature.source,
             out->temperature.disagreement ? " !" : "", out->humidity.value, out->humidity.source,
             out->co2.value, out->trends.temperature.per_minute, out->health.healthy_mask,
             out->health.suspect_mask);
}
