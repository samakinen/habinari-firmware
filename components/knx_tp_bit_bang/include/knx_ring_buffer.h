/**
 * @file knx_ring_buffer.h
 * @brief High-performance lock-free ring buffer for KNX telegram storage
 * 
 * This ring buffer implements a single-producer, single-consumer (SPSC) pattern
 * optimized for ISR-to-task communication. The producer (ISR) writes telegrams
 * when reception completes, and the consumer (application task) reads at its own pace.
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
#define KNX_RING_BUFFER_SIZE 8
#endif

// Compile-time check: ensure size is power of 2
_Static_assert((KNX_RING_BUFFER_SIZE & (KNX_RING_BUFFER_SIZE - 1)) == 0, 
               "KNX_RING_BUFFER_SIZE must be a power of 2");

/**
 * @brief Maximum telegram data size in bytes (KNX TP1 standard)
 */
#define KNX_RING_BUFFER_TELEGRAM_MAX_SIZE 23

/**
 * @brief Ring buffer entry containing a received KNX telegram
 */
typedef struct {
    uint8_t data[KNX_RING_BUFFER_TELEGRAM_MAX_SIZE]; ///< Telegram data bytes
    uint8_t length;                                   ///< Actual telegram length (1-23)
    uint8_t errors;                                   ///< Error flags for this telegram
    uint32_t parity_bits;                            ///< Parity bits received
    uint64_t timestamp;                              ///< Reception timestamp (microseconds)
} knx_ring_buffer_entry_t;

/**
 * @brief Ring buffer structure for KNX telegram storage
 * 
 * This structure maintains the ring buffer state including head/tail indices
 * and statistics. It's designed for single-producer, single-consumer access.
 */
typedef struct {
    knx_ring_buffer_entry_t buffer[KNX_RING_BUFFER_SIZE]; ///< Ring buffer array
    volatile uint8_t head;                                  ///< Write index (ISR updates)
    volatile uint8_t tail;                                  ///< Read index (task updates)
    volatile uint32_t dropped_count;                       ///< Overflow counter
} knx_ring_buffer_t;

/**
 * @brief Initialize a ring buffer
 * 
 * @param rb Pointer to ring buffer structure
 */
void knx_ring_buffer_init(knx_ring_buffer_t *rb);

/**
 * @brief Push a telegram entry into the ring buffer (ISR-safe)
 * 
 * This function is called from ISR context to store a received telegram.
 * If the buffer is full, the telegram is dropped and the dropped counter is incremented.
 * 
 * @param rb Pointer to ring buffer
 * @param data Pointer to telegram data bytes
 * @param length Telegram length in bytes (1-23)
 * @param errors Error flags for this telegram
 * @param parity_bits Parity bits received
 * @param timestamp Reception timestamp in microseconds
 * @return true if telegram was stored, false if buffer was full
 * 
 * @note This function must be called from the same context (ISR) to maintain thread safety
 */
bool knx_ring_buffer_push(knx_ring_buffer_t *rb, 
                          const uint8_t *data, 
                          uint8_t length,
                          uint8_t errors,
                          uint32_t parity_bits,
                          uint64_t timestamp);

/**
 * @brief Pop a telegram entry from the ring buffer (task-safe)
 * 
 * This function is called from task context to retrieve the oldest telegram.
 * If the buffer is empty, the function returns false.
 * 
 * @param rb Pointer to ring buffer
 * @param entry Pointer to entry structure to receive the telegram
 * @return true if telegram was retrieved, false if buffer is empty
 * 
 * @note This function must be called from the same context (task) to maintain thread safety
 */
bool knx_ring_buffer_pop(knx_ring_buffer_t *rb, knx_ring_buffer_entry_t *entry);

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
 * @brief Get number of telegrams available in the ring buffer
 * 
 * @param rb Pointer to ring buffer
 * @return Number of telegrams waiting (0 to KNX_RING_BUFFER_SIZE-1)
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
 * @brief Get total number of telegrams dropped due to overflow
 * 
 * @param rb Pointer to ring buffer
 * @return Total dropped telegram count since initialization
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
