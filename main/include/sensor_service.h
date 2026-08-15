#pragma once

#include <esp_err.h>

#include "ext_probe.h"
#include "sensor_bus.h"
#include "sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file sensor_service.h
 * @brief Periodic acquisition task: samples every sensor, runs the readings
 *        through the fusion layer, and hands out one sensor_data_t per cycle.
 *
 * Sampling rate versus publish rate are deliberately different problems here.
 * The task samples fast (SENSOR_SERVICE_SAMPLE_INTERVAL_MS) because that is
 * what makes averaging, rate-of-change and fault detection work at all; what
 * reaches the KNX bus is decided much later by the per-object transmit policy
 * (send-on-change plus heartbeat, see knx_service.cpp). Sampling faster
 * therefore costs I2C traffic and nothing on the bus.
 */

/// Base sampling period. 5 s gives the trend monitors a 100 s regression window
/// and the smoothing filter enough samples to actually average, while staying
/// far below the SCD4x low-power cadence (~30 s) it has to interleave with.
#define SENSOR_SERVICE_SAMPLE_INTERVAL_MS 5000

/// How often to re-probe for an external probe that was absent at startup, in
/// sampling cycles. The probe is a field-installed accessory that can be
/// plugged in after commissioning, and requiring a reboot for that was a
/// needless service call.
#define SENSOR_SERVICE_PROBE_RETRY_CYCLES 60

typedef void (*sensor_data_updated_callback_t)(const sensor_data_t *data);

typedef struct {
    sensor_bus_t sensor_bus;                 // internal I2C sensor package
    ext_probe_t ext_probe;                   // optional external probe
    sensor_data_updated_callback_t callback; // invoked once per sampling cycle
    TaskHandle_t task_handle;
    volatile bool stop_requested;            // cooperative stop; see sensor_service_stop
} sensor_service_t;

typedef sensor_service_t *sensor_service_handle_t;

esp_err_t sensor_service_init(sensor_service_handle_t handle, sensor_data_updated_callback_t callback);
esp_err_t sensor_service_start(sensor_service_handle_t service_handle);
/// Asks the task to finish its current cycle and exit, then releases the bus.
/// Cooperative on purpose: deleting the task outright could strand the BME688
/// mutex it shares with the BSEC task.
esp_err_t sensor_service_stop(sensor_service_handle_t service_handle);

#ifdef __cplusplus
}
#endif
