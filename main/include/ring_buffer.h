#pragma once
#include "esp_err.h"

typedef struct {
    uint8_t size_mask; // Size of the buffer
    uint8_t head; // Head index
    uint8_t tail; // Tail index
    uint8_t *buffer; // Ring buffer for received data
} ring_buffer_t;

static esp_err_t ring_buffer_init(ring_buffer_t *ring_buffer, uint8_t size) {
    ring_buffer->size_mask = size - 1; // Size of the buffer must be a power of 2
    if ((size & (size - 1)) != 0) {
        return ESP_ERR_INVALID_ARG; // Size must be a power of 2
    }
    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    ring_buffer->buffer = (uint8_t *)malloc(size * sizeof(uint8_t));
    if (ring_buffer->buffer == NULL) {
        return ESP_ERR_NO_MEM; // Memory allocation failed
    }
}

static inline bool ring_buffer_is_empty(ring_buffer_t *ring_buffer) {
    return ring_buffer->head == ring_buffer->tail;
}

static inline bool ring_buffer_is_full(ring_buffer_t *ring_buffer) {
    return ((ring_buffer->head + 1) & ring_buffer->size_mask) == ring_buffer->tail;
}

static inline esp_err_t ring_buffer_push(ring_buffer_t *ring_buffer, uint8_t value) {
    if (ring_buffer_is_full(ring_buffer)) {
        return ESP_ERR_NO_MEM; // Buffer is full
    }
    ring_buffer->buffer[ring_buffer->head] = value;
    ring_buffer->head = (ring_buffer->head + 1) & ring_buffer->size_mask;
    return ESP_OK;
}

static inline esp_err_t ring_buffer_pop(ring_buffer_t *ring_buffer, uint8_t *value) {
    if (ring_buffer_is_empty(ring_buffer)) {
        return ESP_ERR_INVALID_STATE; // Buffer is empty
    }
    *value = ring_buffer->buffer[ring_buffer->tail];
    ring_buffer->tail = (ring_buffer->tail + 1) & ring_buffer->size_mask;
    return ESP_OK;
}

static inline esp_err_t ring_buffer_peek(ring_buffer_t *ring_buffer, uint8_t *value) {
    if (ring_buffer_is_empty(ring_buffer)) {
        return ESP_ERR_INVALID_STATE; // Buffer is empty
    }
    *value = ring_buffer->buffer[ring_buffer->tail];
    return ESP_OK;
}

static void ring_buffer_free(ring_buffer_t *ring_buffer) {
    free(ring_buffer->buffer);
    ring_buffer->buffer = NULL;
    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    ring_buffer->size_mask = 0;
}