#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "bme68x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Air-quality outputs produced by the Bosch BSEC fusion library from the BME688
// raw gas signal (plus temperature/humidity/pressure for compensation). These
// are the *value-add* over the raw gas resistance, which on its own is an
// uncompensated, drifting ohms reading. The board's SCD4x already provides true
// CO2, so `co2_equivalent` here is complementary VOC-derived data.
//
// This whole component is only meaningful when built with CONFIG_BME688_USE_BSEC
// and the proprietary Bosch BSEC library present (see README.md). Otherwise the
// functions below are inert stubs and no air-quality data is produced.
typedef struct {
    float   iaq;              // Indoor air-quality index, 0..500 (lower = cleaner)
    uint8_t iaq_accuracy;     // BSEC calibration status: 0=unreliable .. 3=high
    float   co2_equivalent;   // Estimated CO2-equivalent, ppm
    float   voc_equivalent;   // Estimated breath-VOC-equivalent, ppm
    float   comp_temperature; // Heat-compensated temperature, degC
    float   comp_humidity;    // Heat-compensated relative humidity, %RH
    float   pressure;         // Raw BME688 pressure, Pa (BSEC has no pressure output)
    bool    valid;            // true once BSEC has produced a first result
} bsec_air_quality_t;

// True only in builds compiled against the proprietary BSEC library
// (CONFIG_BME688_USE_BSEC). When false every function below is an inert stub.
bool bsec_integration_available(void);

// Initialise BSEC and subscribe to the IAQ/CO2/VOC virtual sensors. `dev` must
// already be initialised (bme68x_init). `bus_mutex` serialises BME688 access
// against the periodic sensor-bus reader (pass NULL to skip locking). No-op in
// stub builds.
esp_err_t bsec_integration_init(struct bme68x_dev *dev, SemaphoreHandle_t bus_mutex);

// Start the background task that drives the BME688 on BSEC's low-power (~3 s) or
// ultra-low-power (~300 s) schedule and keeps the latest outputs. No-op in stub
// builds.
esp_err_t bsec_integration_start(void);

// Copy the most recent air-quality outputs. Returns false (leaving *out
// untouched) if none are available yet or BSEC is not compiled in.
bool bsec_integration_get_latest(bsec_air_quality_t *out);

#ifdef __cplusplus
}
#endif
