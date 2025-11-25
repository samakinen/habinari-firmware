#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "knx_ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

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
static const int32_t KNX_SYNC_TO_ACK_DELAY_BIT_TIMES = (11+15); // Delay before sending ACK in bit times from last start bit (11 bits for byte + 15 bits idle time)

// ACK byte constants using const uint8_t for type safety while maintaining optimization
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_NONE = 0b11111111; // ACK not received
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_ACK  = 0b11001100; // ACK byte for KNX TP
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_NACK = 0b00001100; // NACK byte for KNX TP
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_BUSY = 0b11000000; // BUSY byte for KNX TP
static const uint8_t KNX_TP_BIT_BANG_ACK_BYTE_NACK_BUSY = 0b00000000; // NACK + BUSY byte for KNX TP

typedef uint8_t knx_tp_bit_bang_rx_state_t; // State type for the bit-bang RX operation
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_ERROR = 0; // Error state
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_IDLE = 1; // Waiting for new telegram
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_RECEIVE = 5; // Receiving telegram data
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_WAIT_MORE_DATA = 6; // Waiting for new byte
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_WAIT_ACK = 7; // Waiting for ACK
static const knx_tp_bit_bang_rx_state_t KNX_TP_BIT_BANG_RX_STATE_RECEIVING_ACK = 8; // Receiving ACK

typedef uint8_t knx_tp_bit_bang_tx_state_t; // State type for the bit-bang TX operation
// Idle states 0 - 7
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_IDLE = 0; // No data to transmit
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_LAST_IDLE = 7; // Placeholder for last idle state
// Busy states 8 - 15
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT = 8; // Waiting free slot
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_ACTIVE = 9; // TX in progress
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_EQUALIZATION = 10; // TX in progress
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE = 11; // TX in progress
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_COLLISION = 12; // Collision detected
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_WAITING_ACK = 13; // Waiting for ACK
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_LAST_BUSY = 15; // Placeholder for last busy state
//  Error states 16+
static const knx_tp_bit_bang_tx_state_t KNX_TP_BIT_BANG_TX_STATE_ERROR = 16; // Error occurred

static const uint8_t KNX_TP_BIT_BANG_RX_ERROR_PARITY = (1 << 0); // Bitmask for parity bit error
static const uint8_t KNX_TP_BIT_BANG_RX_ERROR_FRAMING = (1 << 1); // Bitmask for invalid received data
static const uint8_t KNX_TP_BIT_BANG_RX_ERROR_CRC = (1 << 2); // Bitmask for CRC error

static const uint8_t KNX_TP_BIT_BANG_FLAG_NEXT_EVENT_TX = (1 << 0); // Next timer event will trigger TX routines
static const uint8_t KNX_TP_BIT_BANG_FLAG_NO_COLLISION_DETECT = (1 << 1); // Disable collision detection on TX
static const uint8_t KNX_TP_BIT_BANG_FLAG_RECEIVING_ACK = (1 << 2); // Currently receiving ACK
static const uint8_t KNX_TP_BIT_BANG_FLAG_SENDING_ACK = (1 << 3); // Currently sending ACK

static const uint8_t KNX_TP_BIT_BANG_MSG_TYPE_START = 1; // Start receiving telegram
static const uint8_t KNX_TP_BIT_BANG_MSG_TYPE_DATA = 2; // Data byte type
static const uint8_t KNX_TP_BIT_BANG_MSG_TYPE_PARITY_ERROR = 3; // Parity error byte type
static const uint8_t KNX_TP_BIT_BANG_MSG_TYPE_FRAMING_ERROR = 4; // Framing error byte type
static const uint8_t KNX_TP_BIT_BANG_MSG_TYPE_END = 5; // End of telegram. (no data)
static const uint8_t KNX_TP_BIT_BANG_MSG_TYPE_ACK = 6; // ACK byte type
static const uint8_t KNX_TP_BIT_BANG_MSG_TYPE_TX_ACK_RESPONSE = 7; // ACK response for transmitted telegram

// =====================
// TX result types (defined early because used in driver state struct)
// =====================

typedef enum {
    KNX_TX_ACK_NONE = 0,
    KNX_TX_ACK_ACK = 1,
    KNX_TX_ACK_BUSY = 2,
    KNX_TX_ACK_NACK = 3,
    KNX_TX_ACK_NACK_BUSY = 4,
} knx_tx_ack_t;

typedef struct {
    knx_tx_ack_t ack;       // Final ACK outcome
    uint8_t errors;         // Error flags at completion (collision/parity/framing)
    uint8_t length;         // TX length
    uint64_t timestamp;     // Timer count at completion
} knx_tx_result_t;

#define KNX_TP_BIT_BANG_RX_BUFFER_SIZE 23 // Size of the RX buffer in bytes (max telegram size)
#define KNX_MAX_TELEGRAM_SIZE 23 // Maximum telegram size in bytes (Control + Address + Data + Checksum)

typedef enum {
    KNX_EVENT_NONE = 0,
    KNX_EVENT_BUS_IDLE,
    KNX_EVENT_COLLISION,
    KNX_EVENT_BYTE_RECEIVED,
    KNX_EVENT_TELEGRAM_RECEIVED,
    KNX_EVENT_ACK_RECEIVED,
    KNX_EVENT_ACK_NOT_RECEIVED,
    KNX_EVENT_INVALID_STATE
} knx_event_t;

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
    
    // 8-bit fields - frequently accessed state
    uint8_t rx_zero_detected;       // Flag indicating zero bit detected
    knx_tp_bit_bang_tx_state_t tx_state;  // Current TX state
    knx_tp_bit_bang_rx_state_t rx_state;  // Current RX state
    uint8_t tx_current_byte;        // Current byte being transmitted (data or ACK)
    uint8_t pending_ack_byte;       // ACK byte requested by host via U_ACK_REQ (0 = none)
    uint8_t tx_buffer[KNX_MAX_TELEGRAM_SIZE]; // Buffer for TX telegram
    uint8_t tx_byte_position;       // Current byte position in TX buffer
    uint8_t tx_telegram_length;     // Length of telegram to send
    int8_t tx_bit_position;         // Current bit position in TX byte (-1=start, 0-7=data, 8=parity, 9=stop)
    uint8_t rx_byte;                // Currently received byte
    int8_t rx_bit_position;         // Current bit position in RX byte (-1=start, 0-7=data, 8=parity, 9=stop)
    uint8_t rx_errors;              // RX error flags (PARITY, FRAMING, CRC)
    uint8_t tx_wait_time;           // Wait time since last sync point (in bit times)
    
    // ========================================================================
    // WARM FIELDS - Accessed occasionally (subsequent cache lines)
    // ========================================================================

    uint64_t sync_point;            // Timer count at the latest start bit
    TaskHandle_t xTaskToNotify;     // Task handle for notifications
    
    // Ring buffer for received telegrams (accessed after ISR)
    knx_ring_buffer_t rx_ring_buffer;  // High-performance lock-free ring buffer
    
    // ========================================================================
    // COLD FIELDS - Rarely accessed (end of struct)
    // ========================================================================
    
    // Device address for TPUART hardware filtering (individual address only)
    uint16_t device_address;        // Individual device address for auto-ACK
    
    // TX result tracking
    knx_tx_result_t last_tx_result; // Last transmission result
    
} __attribute__((aligned(32))) knx_tp_bit_bang_t;

typedef knx_tp_bit_bang_t *knx_tp_bit_bang_handle_t;

// ISR functions (defined in knx_tp_bit_bang_isr.c)
bool IRAM_ATTR knx_tp_bit_bang_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg);
void IRAM_ATTR knx_tp_bit_bang_rx_pin_isr(void *arg);
void IRAM_ATTR rearm_timer(knx_tp_bit_bang_t *bit_bang);

// New optimized initialization using compile-time pin configuration
esp_err_t knx_tp_bit_bang_init(knx_tp_bit_bang_t *bit_bang);

esp_err_t knx_tp_bit_bang_send(knx_tp_bit_bang_handle_t bit_bang, uint8_t *data, uint16_t length);

// Rearm timer (for external use)
esp_err_t knx_tp_bit_bang_rearm_timer(knx_tp_bit_bang_t *bit_bang);

// Reset TX state machine
esp_err_t knx_tp_bit_bang_reset_tx(knx_tp_bit_bang_t *bit_bang);

// Core bit-bang functions (time-critical only)
bool knx_tp_bit_bang_pop_data(knx_tp_bit_bang_t *bit_bang, uint8_t *out);

// Deinitialize and release resources (timer, ISR). Safe to call multiple times.
esp_err_t knx_tp_bit_bang_deinit(knx_tp_bit_bang_t *bit_bang);

// Device address management (TPUART supports individual address filtering)
esp_err_t knx_tp_bit_bang_set_device_address(knx_tp_bit_bang_t *bit_bang, uint16_t address);

// Address formatting utilities
void knx_format_individual_address(uint16_t address, char *buf, size_t buf_size);
void knx_format_group_address(uint16_t address, char *buf, size_t buf_size);

#ifdef __cplusplus
}
#endif