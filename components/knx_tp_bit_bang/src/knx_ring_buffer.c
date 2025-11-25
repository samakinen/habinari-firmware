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

bool IRAM_ATTR knx_ring_buffer_push_msg(knx_ring_buffer_t *rb, knx_ring_buffer_data_t data)
{
    if (rb == NULL) {
        return false;
    }
    
    // Calculate next head position
    const uint8_t next_head = (rb->head + 1) & (KNX_RING_BUFFER_SIZE - 1);
    
    // Check if buffer is full (head would catch up to tail)
    if (next_head == rb->tail) {
        // Buffer full - drop byte and increment counter
        rb->dropped_count++;
        return false;
    }
    
    // Store the byte
    rb->buffer[rb->head] = data;
    
    // Memory barrier to ensure all writes complete before updating head
    // This prevents the consumer from seeing partially written data
    __asm__ __volatile__("" ::: "memory");
    
    // Update head index (makes byte visible to consumer)
    rb->head = next_head;
    
    return true;
}

bool knx_ring_buffer_pop_msg(knx_ring_buffer_t *rb, knx_ring_buffer_data_t *out)
{
    if (rb == NULL || out == NULL) {
        return false;
    }
    
    // Check if buffer is empty
    if (rb->tail == rb->head) {
        return false; // No bytes available
    }
    
    // Read the byte
    *out = rb->buffer[rb->tail];
    
    // Memory barrier to ensure read completes before updating tail
    // This prevents the producer from overwriting data we're still reading
    __asm__ __volatile__("" ::: "memory");
    
    // Update tail index (frees slot for reuse)
    rb->tail = (rb->tail + 1) & (KNX_RING_BUFFER_SIZE - 1);
    
    return true;
}

bool knx_ring_buffer_peek_msg(const knx_ring_buffer_t *rb, uint8_t offset, knx_ring_buffer_data_t *out)
{
    if (rb == NULL || out == NULL) {
        return false;
    }
    
    // Check if buffer has enough messages for the requested offset
    const uint8_t available = (rb->head - rb->tail) & (KNX_RING_BUFFER_SIZE - 1);
    if (offset >= available) {
        return false; // Not enough messages in buffer
    }
    
    // Calculate the index to peek at
    const uint8_t peek_index = (rb->tail + offset) & (KNX_RING_BUFFER_SIZE - 1);
    
    // Read the message without modifying the tail
    *out = rb->buffer[peek_index];
    
    // Memory barrier to ensure consistent read
    __asm__ __volatile__("" ::: "memory");
    
    return true;
}
