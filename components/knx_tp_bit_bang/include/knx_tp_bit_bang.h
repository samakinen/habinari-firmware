#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "knx_ring_buffer.h"

//#include "knx_tp_bit_bang_events.h"

static const int32_t KNX_BIT_TIME_US = 104; // Time for one bit in microseconds
static const int32_t KNX_ZERO_ACTIVE_TIME_US = 35; // Time for the zero active state in microseconds
static const int32_t KNX_ZERO_EQUALIZATION_TIME_US = 69; // Time for the zero equalization state in microseconds
static const int32_t KNX_INTER_BYTE_TIME_US = (KNX_BIT_TIME_US * 2); // Time between bytes in microseconds (3 bits)
static const int32_t KNX_ACK_TIMEOUT_US = ((15+10+10)*KNX_BIT_TIME_US); // Timeout for ACK in microseconds (15 bits wait + 10 bits for the ACK + 10 bits for processing)
static const int32_t KNX_MAX_TELEGRAM_SIZE = 1+5+16+1; // Maximum telegram size in bytes (Control + Address + Data + Checksum)
static const int32_t KNX_WAIT_MORE_DATA_TIMEOUT_US = ((2+4)*KNX_BIT_TIME_US); // Timeout for waiting for more data in microseconds (2 pause bits + 4 extra bits)
static const int32_t KNX_BIT_SAMPLING_OFFSET_US = (KNX_BIT_TIME_US/4*3); // Offset for bit sampling in microseconds (3/4 of the bit time)
static const int32_t KNX_TELEGRAM_SPACING = (KNX_BIT_TIME_US*16); // Delay after the previous start bit in microseconds before new frame can be transmitted
static const int32_t KNX_SYNC_TO_ACK_DELAY_US = ((11+15)*KNX_BIT_TIME_US); // Delay before sending ACK in microseconds from last start bit (11 bits for byte + 15 bits idle time)

// ACK byte constants using const uint8_t for type safety while maintaining optimization
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_NONE = 0b11111111; // ACK not received
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_ACK  = 0b11001100; // ACK byte for KNX TP
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_NACK = 0b00001100; // NACK byte for KNX TP
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_BUSY = 0b11000000; // BUSY byte for KNX TP
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_NACK_BUSY = 0b00000000; // NACK + BUSY byte for KNX TP

typedef uint8_t knx_tp_bit_bang_rx_state_t; // State type for the bit-bang RX operation
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_ERROR = 0; // Error state
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_IDLE = 1; // No data to receive
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_RECEIVE = 5; // Receiving data
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_WAIT_MORE_DATA = 6; // Waiting for more data
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_SEND_ACK = 7; // Sending ACK

typedef uint8_t knx_tp_bit_bang_tx_state_t; // State type for the bit-bang TX operation
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_ERROR = 0; // No data to transmit
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_IDLE = 1; // No data to transmit
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT = 2; // Waiting free slot
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_ACTIVE = 3; // TX in progress
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_EQUALIZATION = 4; // TX in progress
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE = 5; // TX in progress
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_COLLISION = 6; // Collision detected
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_WAITING_ACK = 7; // Waiting for ACK
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_ACK_RECEIVED = 8; // ACK received
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_ACK_NOT_RECEIVED = 9; // ACK not received
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_ACK_BUSY = 10; // ACK busy received
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_ACK_NACK = 11; // ACK NACK received
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_ACK_NACK_BUSY = 12; // ACK NACK + busy received

static const uint8_t KNX_TP_BIT_BANG_COLLISION = (1 << 0); // Bitmask for collision detection
static const uint8_t KNX_TP_BIT_BANG_RX_BUFFER_OVERFLOW = (1 << 1); // Bitmask for RX buffer overflow
static const uint8_t KNX_TP_BIT_BANG_PARITY_ERROR = (1 << 2); // Bitmask for parity bit error
static const uint8_t KNX_TP_BIT_BANG_FRAMING_ERROR = (1 << 3); // Bitmask for invalid received data

static const uint8_t KNX_TP_BIT_BANG_FLAG_NEXT_EVENT_TX = (1 << 0); // Next timer event will trigger TX routines
static const uint8_t KNX_TP_BIT_BANG_FLAG_NO_COLLISION_DETECT = (1 << 1); // Disable collision detection on TX
static const uint8_t KNX_TP_BIT_BANG_FLAG_COLLISION_OK = (1 << 2); // Collision detected, stop TX, but no error
static const uint8_t KNX_TP_BIT_BANG_FLAG_AUTO_ACK = (1 << 3); // Acknowledge received telegrams
static const uint8_t KNX_TP_BIT_BANG_FLAG_BUSY = (1 << 4); // Acknowledge with busy status
static const uint8_t KNX_TP_BIT_BANG_FLAG_PROMISCUOUS = (1 << 5); // Receive all telegrams, only ACK if addressed to us
static const uint8_t KNX_TP_BIT_BANG_FLAG_SENDING_ACK = (1 << 6); // Currently sending ACK

#define KNX_TP_BIT_BANG_TX_BUFFER_SIZE 23 // Size of the TX buffer in bytes (max telegram size)
#define KNX_TP_BIT_BANG_RX_BUFFER_SIZE 23 // Size of the RX buffer in bytes (max telegram size)
#define KNX_TP_BIT_BANG_MAX_GROUP_ADDRESSES 16 // Maximum number of group addresses to listen to


typedef enum {
    KNX_EVENT_BUS_IDLE,
    KNX_EVENT_COLLISION,
    KNX_EVENT_BYTE_RECEIVED,
    KNX_EVENT_TELEGRAM_RECEIVED,
    KNX_EVENT_ACK_RECEIVED,
    KNX_EVENT_ACK_NOT_RECEIVED,
} knx_event_t;

typedef struct {
    gpio_num_t tx_pin;          // KNX TP TX pin
    gpio_num_t rx_pin;          // KNX TP RX pin
    gpio_num_t prog_btn_pin;    // Program button pin (optional)
    gpio_num_t led_pin;         // Status LED pin (optional)
} knx_tp_pin_config_t;

typedef uint8_t knx_priority_t; // Priority type for KNX telegrams

#define TIMER_MARGINES_SIZE 64

/**
 * @brief KNX TP bit-bang driver state structure
 * 
 * Optimized layout for cache performance and memory efficiency:
 * - Hot fields (accessed in every ISR) at the beginning
 * - Fields grouped by size for optimal packing
 * - 32-byte aligned for cache line optimization
 * - Total size: ~1.2KB
 */
typedef struct {
    // ========================================================================
    // HOT FIELDS - Accessed in every ISR call (first cache line)
    // ========================================================================
    
    // 64-bit fields (8-byte alignment)
    uint64_t tx_alarm_value;        // Timer alarm value for TX operations
    uint64_t rx_alarm_value;        // Timer alarm value for RX operations

    
    // Handles/pointers (4-byte on ESP32-C6)
    gptimer_handle_t timer;         // Timer handle for bit-bang operation

    
    // 32-bit fields
    uint32_t flags;                 // Status flags (NEXT_EVENT_TX, COLLISION_DETECT, etc.)
    uint32_t rx_parity_bits;        // Parity bits for received telegram
    
    // 16-bit fields
    uint16_t device_address;        // Device address for KNX TP device
    
    // 8-bit fields - frequently accessed state
    knx_tp_bit_bang_tx_state_t tx_state;  // Current TX state
    knx_tp_bit_bang_rx_state_t rx_state;  // Current RX state
    uint8_t tx_current_byte;        // Current byte being transmitted (data or ACK)
    uint8_t tx_byte_position;       // Current byte position in TX buffer
    int8_t tx_bit_position;         // Current bit position in TX byte (-1=start, 0-7=data, 8=parity, 9=stop)
    uint8_t rx_byte_position;       // Current byte position in RX buffer
    int8_t rx_bit_position;         // Current bit position in RX byte (-1=start, 0-7=data, 8=parity, 9=stop)
    uint8_t tx_telegram_length;     // Length of telegram to send
    uint8_t rx_telegram_length;     // Length of telegram received
    uint8_t errors;                 // Error flags (COLLISION, OVERFLOW, PARITY, FRAMING)
    uint8_t rx_zero_detected;       // Flag indicating zero bit detected
    
    // ========================================================================
    // WARM FIELDS - Accessed occasionally (subsequent cache lines)
    // ========================================================================
    uint64_t sync_point;            // Timer count at the latest start bit
    TaskHandle_t xTaskToNotify;     // Task handle for notifications
    // Buffers - aligned to 4-byte boundary for efficient access
    uint8_t rx_buffer[KNX_TP_BIT_BANG_RX_BUFFER_SIZE] __attribute__((aligned(4)));  // RX telegram buffer
    uint8_t tx_buffer[KNX_TP_BIT_BANG_TX_BUFFER_SIZE] __attribute__((aligned(4)));  // TX telegram buffer
    
    // Ring buffer for received telegrams (accessed after ISR)
    knx_ring_buffer_t rx_ring_buffer;  // High-performance lock-free ring buffer
    
    // ========================================================================
    // COLD FIELDS - Rarely accessed (end of struct)
    // ========================================================================
    
    // Address filtering (configured once during init or runtime)
    uint16_t group_addresses[KNX_TP_BIT_BANG_MAX_GROUP_ADDRESSES];  // Group addresses to listen to
    uint8_t group_address_count;    // Number of valid group addresses in the list
    
    // ========================================================================
    // DEBUG FIELDS - Only accessed for diagnostics
    // ========================================================================
    
    uint32_t tx_timer_count;        // Count of TX timer events
    uint32_t rx_timer_count;        // Count of RX timer events
    
    // Debug arrays - keep at end to avoid cache pollution during normal operation
    int32_t tx_timer_delay[TIMER_MARGINES_SIZE];
    int32_t tx_timer_durations[TIMER_MARGINES_SIZE];
    int32_t rx_timer_delays[TIMER_MARGINES_SIZE];
    int32_t rx_timer_durations[TIMER_MARGINES_SIZE];
} __attribute__((aligned(32))) knx_tp_bit_bang_t;

typedef knx_tp_bit_bang_t *knx_tp_bit_bang_handle_t;

// ISR functions (defined in knx_tp_bit_bang_isr.c)
bool IRAM_ATTR knx_tp_bit_bang_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg);
void IRAM_ATTR knx_tp_bit_bang_rx_pin_isr(void *arg);
void IRAM_ATTR rearm_timer(knx_tp_bit_bang_t *bit_bang);

// New optimized initialization using compile-time pin configuration
esp_err_t knx_tp_bit_bang_init(knx_tp_bit_bang_t *bit_bang);

// Backward compatibility wrapper (deprecated)
__attribute__((deprecated("Use knx_tp_bit_bang_init() with Kconfig pin settings instead")))
esp_err_t knx_tp_bit_bang_init_legacy(knx_tp_bit_bang_t *bit_bang, gpio_num_t tx_pin, gpio_num_t rx_pin);

esp_err_t knx_tp_bit_bang_send(knx_tp_bit_bang_handle_t bit_bang, uint8_t *data, uint16_t length);
//esp_err_t knx_tp_bit_bang_receive(knx_tp_bit_bang_handle_t bit_bang, uint8_t *data, uint16_t length);
esp_err_t knx_tp_bit_bang_tx_enable(knx_tp_bit_bang_t *bit_bang);

// Rearm timer (for external use)
esp_err_t knx_tp_bit_bang_rearm_timer(knx_tp_bit_bang_t *bit_bang);

// Reset TX state machine
esp_err_t knx_tp_bit_bang_reset_tx(knx_tp_bit_bang_t *bit_bang);


// Ring buffer API for receiving telegrams
/**
 * @brief Pop a received telegram from the ring buffer
 * 
 * @param bit_bang Pointer to the bit-bang instance
 * @param entry Pointer to ring buffer entry structure to store the received data
 * @return true if a telegram was retrieved, false if buffer is empty
 */
bool knx_tp_bit_bang_pop_telegram(knx_tp_bit_bang_t *bit_bang, knx_ring_buffer_entry_t *entry);

/**
 * @brief Check if there are telegrams available in the ring buffer
 * 
 * @param bit_bang Pointer to the bit-bang instance
 * @return Number of telegrams available in the buffer
 */
uint8_t knx_tp_bit_bang_telegrams_available(knx_tp_bit_bang_t *bit_bang);

/**
 * @brief Get statistics about dropped telegrams
 * 
 * @param bit_bang Pointer to the bit-bang instance
 * @return Number of telegrams dropped due to buffer overflow
 */
uint32_t knx_tp_bit_bang_get_dropped_count(knx_tp_bit_bang_t *bit_bang);

// Performance monitoring functions
void knx_tp_bit_bang_get_performance_stats(knx_tp_bit_bang_t *bit_bang, 
                                           uint32_t *tx_timer_count, 
                                           uint32_t *rx_timer_count);

// Address filtering API
/**
 * @brief Set the physical/individual address for this device
 * 
 * @param bit_bang Pointer to the bit-bang instance
 * @param address Individual address in KNX format (Area.Line.Device)
 * @return ESP_OK on success
 */
esp_err_t knx_tp_bit_bang_set_device_address(knx_tp_bit_bang_t *bit_bang, uint16_t address);

/**
 * @brief Add a group address to the listen list
 * 
 * When AUTO_ACK is enabled, the device will acknowledge telegrams sent to any
 * of the group addresses in this list, or to its individual address.
 * 
 * @param bit_bang Pointer to the bit-bang instance
 * @param group_address Group address to listen to
 * @return ESP_OK on success, ESP_ERR_NO_MEM if list is full
 */
esp_err_t knx_tp_bit_bang_add_group_address(knx_tp_bit_bang_t *bit_bang, uint16_t group_address);

/**
 * @brief Remove a group address from the listen list
 * 
 * @param bit_bang Pointer to the bit-bang instance
 * @param group_address Group address to remove
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if address not in list
 */
esp_err_t knx_tp_bit_bang_remove_group_address(knx_tp_bit_bang_t *bit_bang, uint16_t group_address);

/**
 * @brief Clear all group addresses from the listen list
 * 
 * @param bit_bang Pointer to the bit-bang instance
 * @return ESP_OK on success
 */
esp_err_t knx_tp_bit_bang_clear_group_addresses(knx_tp_bit_bang_t *bit_bang);

void knx_format_individual_address(uint16_t addr, char* out, size_t out_size);
void knx_format_group_address(uint16_t addr, char* out, size_t out_size);
uint16_t knx_tp_bit_bang_triplet_to_address(uint8_t main, uint8_t middle, uint8_t sub);