#pragma once
// Minimal Arduino compatibility shim for ESP-IDF
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Arduino-style typedefs
typedef unsigned int uint;

static inline uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static inline uint32_t micros(void) {
    return (uint32_t)(esp_timer_get_time());
}

static inline void delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// Stubs for Arduino digital APIs if ever included indirectly
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif

// No-op stubs; not used by TPUART stack directly
static inline void pinMode(int, int) {}
static inline void digitalWrite(int, int) {}
