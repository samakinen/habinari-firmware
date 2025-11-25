/**
 * @file knx_ring_buffer.h
 * @brief High-performance lock-free ring buffer for KNX telegram storage
 * 
 * This ring buffer implements a single-producer, single-consumer (SPSC) pattern
 * optimized for ISR-to-task communication. The producer (ISR) writes BYTES
 * as they arrive (byte-by-byte streaming), and the consumer (task) reads them
 * at its own pace.
 * 
 * Key features:
 * - Lock-free operation using memory barriers
 * - O(1) constant-time push/pop operations
 * - Power-of-2 size for fast index wrapping
 * - Overflow detection and statistics
 * - Precise timestamp per telegram
 */

#ifndef KNX_RING_BUFFER_H
#define KNX_RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Ring buffer size (must be power of 2 for efficient modulo operation)
 * 
 * Recommendations:
 * - 4: Minimum for basic applications
 * - 8: Recommended for most applications (default)
 * - 16: High-traffic KNX networks
 * - 32: Maximum recommended (memory constrained)
 */
#ifndef KNX_RING_BUFFER_SIZE
#define KNX_RING_BUFFER_SIZE 64
#endif

typedef struct {
    uint8_t type;
    uint8_t data;
} knx_ring_buffer_data_t;

// Compile-time check: ensure size is power of 2
_Static_assert((KNX_RING_BUFFER_SIZE & (KNX_RING_BUFFER_SIZE - 1)) == 0, 
               "KNX_RING_BUFFER_SIZE must be a power of 2");

/**
 * @brief Message ring buffer structure for communication with ISR
 * 
 * This structure maintains a simple SPSC ring buffer of KNX messages.
 * It is intentionally minimal for ISR safety and high throughput.
 */

typedef struct {
    knx_ring_buffer_data_t buffer[KNX_RING_BUFFER_SIZE];   ///< Raw message storage
    volatile uint8_t head;                  ///< Write index (ISR updates)
    volatile uint8_t tail;                  ///< Read index (task updates)
    volatile uint32_t dropped_count;        ///< Overflow counter
} knx_ring_buffer_t;

/**
 * @brief Initialize a ring buffer
 * 
 * @param rb Pointer to ring buffer structure
 */
void knx_ring_buffer_init(knx_ring_buffer_t *rb);

/**
 * @brief Push a single message into the ring buffer (ISR-safe)
 * 
 * Called from ISR context to store a received message. If the buffer is full,
 * the message is dropped and the dropped counter is incremented.
 * 
 * @param rb Pointer to ring buffer
 * @param msg Message to push
 * @return true if message was stored, false if buffer was full
 */
bool knx_ring_buffer_push_msg(knx_ring_buffer_t *rb, knx_ring_buffer_data_t data);

/**
 * @brief Pop a single message from the ring buffer (task-safe)
 * 
 * Called from task context to retrieve the oldest message. If the buffer is
 * empty, the function returns false.
 * 
 * @param rb Pointer to ring buffer
 * @param out Pointer to message to receive the value
 * @return true if a message was retrieved, false if buffer is empty
 */
bool knx_ring_buffer_pop_msg(knx_ring_buffer_t *rb, knx_ring_buffer_data_t *out);

/**
 * @brief Peek at a message in the ring buffer without removing it (task-safe)
 * 
 * Called from task context to read a message at a given offset from the tail
 * without removing it from the buffer. Offset 0 is the oldest message (same as pop),
 * offset 1 is the second oldest, etc.
 * 
 * @param rb Pointer to ring buffer
 * @param offset Offset from tail (0 = oldest message)
 * @param out Pointer to message to receive the value
 * @return true if a message was retrieved, false if buffer doesn't have enough messages
 */
bool knx_ring_buffer_peek_msg(const knx_ring_buffer_t *rb, uint8_t offset, knx_ring_buffer_data_t *out);

/**
 * @brief Check if ring buffer is empty
 * 
 * @param rb Pointer to ring buffer
 * @return true if buffer is empty, false otherwise
 */
static inline bool knx_ring_buffer_is_empty(const knx_ring_buffer_t *rb)
{
    return rb->head == rb->tail;
}

/**
 * @brief Check if ring buffer is full
 * 
 * @param rb Pointer to ring buffer
 * @return true if buffer is full, false otherwise
 */
static inline bool knx_ring_buffer_is_full(const knx_ring_buffer_t *rb)
{
    return ((rb->head + 1) & (KNX_RING_BUFFER_SIZE - 1)) == rb->tail;
}

/**
 * @brief Get number of messages available in the ring buffer
 * 
 * @param rb Pointer to ring buffer
 * @return Number of messages waiting (0 to KNX_RING_BUFFER_SIZE-1)
 */
static inline uint8_t knx_ring_buffer_available(const knx_ring_buffer_t *rb)
{
    return (rb->head - rb->tail) & (KNX_RING_BUFFER_SIZE - 1);
}

/**
 * @brief Get number of free slots in the ring buffer
 * 
 * @param rb Pointer to ring buffer
 * @return Number of free slots (0 to KNX_RING_BUFFER_SIZE-1)
 */
static inline uint8_t knx_ring_buffer_free_slots(const knx_ring_buffer_t *rb)
{
    return (KNX_RING_BUFFER_SIZE - 1) - knx_ring_buffer_available(rb);
}

/**
 * @brief Get total number of messages dropped due to overflow
 * 
 * @param rb Pointer to ring buffer
 * @return Total dropped message count since initialization
 */
static inline uint32_t knx_ring_buffer_get_dropped_count(const knx_ring_buffer_t *rb)
{
    return rb->dropped_count;
}

/**
 * @brief Reset the dropped counter
 * 
 * @param rb Pointer to ring buffer
 */
static inline void knx_ring_buffer_reset_dropped_count(knx_ring_buffer_t *rb)
{
    rb->dropped_count = 0;
}

#ifdef __cplusplus
}
#endif

#endif // KNX_RING_BUFFER_H
