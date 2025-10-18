/**
 * @file knx_ring_buffer.c
 * @brief Implementation of lock-free ring buffer for KNX telegram storage
 */

#include "knx_ring_buffer.h"
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#else
#define IRAM_ATTR
#endif

void knx_ring_buffer_init(knx_ring_buffer_t *rb)
{
    if (rb == NULL) {
        return;
    }
    
    memset(rb, 0, sizeof(knx_ring_buffer_t));
    rb->head = 0;
    rb->tail = 0;
    rb->dropped_count = 0;
}

bool IRAM_ATTR knx_ring_buffer_push(knx_ring_buffer_t *rb, 
                                    const uint8_t *data, 
                                    uint8_t length,
                                    uint8_t errors,
                                    uint32_t parity_bits,
                                    uint64_t timestamp)
{
    if (rb == NULL || data == NULL || length == 0 || length > KNX_RING_BUFFER_TELEGRAM_MAX_SIZE) {
        return false;
    }
    
    // Calculate next head position
    const uint8_t next_head = (rb->head + 1) & (KNX_RING_BUFFER_SIZE - 1);
    
    // Check if buffer is full (head would catch up to tail)
    if (next_head == rb->tail) {
        // Buffer full - drop telegram and increment counter
        rb->dropped_count++;
        return false;
    }
    
    // Get pointer to current slot
    knx_ring_buffer_entry_t *entry = &rb->buffer[rb->head];
    
    // Copy telegram data efficiently
    for (uint8_t i = 0; i < length; i++) {
        entry->data[i] = data[i];
    }
    
    // Store metadata
    entry->length = length;
    entry->errors = errors;
    entry->parity_bits = parity_bits;
    entry->timestamp = timestamp;
    
    // Memory barrier to ensure all writes complete before updating head
    // This prevents the consumer from seeing partially written data
    __asm__ __volatile__("" ::: "memory");
    
    // Update head index (makes telegram visible to consumer)
    rb->head = next_head;
    
    return true;
}

bool knx_ring_buffer_pop(knx_ring_buffer_t *rb, knx_ring_buffer_entry_t *entry)
{
    if (rb == NULL || entry == NULL) {
        return false;
    }
    
    // Check if buffer is empty
    if (rb->tail == rb->head) {
        return false; // No telegrams available
    }
    
    // Get pointer to current slot
    const knx_ring_buffer_entry_t *src = &rb->buffer[rb->tail];
    
    // Copy telegram data efficiently
    for (uint8_t i = 0; i < src->length; i++) {
        entry->data[i] = src->data[i];
    }
    
    // Copy metadata
    entry->length = src->length;
    entry->errors = src->errors;
    entry->parity_bits = src->parity_bits;
    entry->timestamp = src->timestamp;
    
    // Memory barrier to ensure read completes before updating tail
    // This prevents the producer from overwriting data we're still reading
    __asm__ __volatile__("" ::: "memory");
    
    // Update tail index (frees slot for reuse)
    rb->tail = (rb->tail + 1) & (KNX_RING_BUFFER_SIZE - 1);
    
    return true;
}
