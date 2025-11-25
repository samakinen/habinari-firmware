#include "knx_tp_bit_bang.h"
#include <stdbool.h>
//#include "esp_attr.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"

// Configuration validation - ensure all required KNX configuration is defined
#ifndef CONFIG_KNX_TP_TX_PIN
#error "CONFIG_KNX_TP_TX_PIN must be defined in sdkconfig. Please run 'idf.py menuconfig' and configure KNX TP settings."
#endif

#ifndef CONFIG_KNX_TP_RX_PIN
#error "CONFIG_KNX_TP_RX_PIN must be defined in sdkconfig. Please run 'idf.py menuconfig' and configure KNX TP settings."
#endif

#ifndef CONFIG_KNX_USE_DIRECT_GPIO
#error "CONFIG_KNX_USE_DIRECT_GPIO must be defined in sdkconfig. Please configure GPIO access method in menuconfig."
#endif

#ifdef CONFIG_KNX_USE_DIRECT_GPIO
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"

// Compile-time GPIO optimizations using Kconfig values
#define KNX_TX_PIN CONFIG_KNX_TP_TX_PIN
#define KNX_RX_PIN CONFIG_KNX_TP_RX_PIN

// Pre-calculated bit masks for maximum performance
#define KNX_TX_PIN_MASK (1UL << KNX_TX_PIN)

// Ultra-fast GPIO macros using direct register access
// Note: On ESP32, out_w1ts/out_w1tc are registers, not nested structs.
// Use single-level field access for correct code generation.
#if defined(CONFIG_IDF_TARGET_ESP32)
#define KNX_GPIO_SET_HIGH_FAST(mask) do { GPIO.out_w1ts = (uint32_t)(mask); } while(0)
#define KNX_GPIO_SET_LOW_FAST(mask)  do { GPIO.out_w1tc = (uint32_t)(mask); } while(0)
#else
// For other targets (ESP32-C6 and friends) the GPIO register layout differs.
// Fall back to the public GPIO API for portability. It's slightly slower
// but keeps the code building across targets. If timing needs tightening
// for a specific target, add a target-specific fast-path here.
#define KNX_GPIO_SET_HIGH_FAST(mask) do { gpio_set_level(KNX_TX_PIN, 1); } while(0)
#define KNX_GPIO_SET_LOW_FAST(mask)  do { gpio_set_level(KNX_TX_PIN, 0); } while(0)
#endif

// Optimized single-pin operations
#define KNX_TX_HIGH() KNX_GPIO_SET_HIGH_FAST(KNX_TX_PIN_MASK)
#define KNX_TX_LOW() KNX_GPIO_SET_LOW_FAST(KNX_TX_PIN_MASK)


#else
// Fallback to standard GPIO functions
#define KNX_TX_HIGH() gpio_set_level(CONFIG_KNX_TP_TX_PIN, 1)
#define KNX_TX_LOW() gpio_set_level(CONFIG_KNX_TP_TX_PIN, 0)

#endif // CONFIG_KNX_USE_DIRECT_GPIO

//#define IS_DATA_BIT(pos) ((pos) >= 0 && (pos) <= 7)
// Use unsigned comparison to eliminate negative check
#define IS_DATA_BIT(pos) ((unsigned)(pos) <= 7u)

// ******* Common helper functions ********

/**
 * @brief Calculate the parity bit for a given byte
 * 
 * Uses the built-in population count function for optimal performance on modern
 * architectures. Returns 1 for odd parity, 0 for even parity.
 * 
 * @param byte The input byte to calculate parity for
 * @return uint8_t 1 if odd number of 1-bits, 0 if even number of 1-bits
 */
static inline uint8_t IRAM_ATTR calculate_parity_bit(uint8_t byte) {
    // Use built-in population count for better performance on modern architectures
    return __builtin_popcount(byte) & 1;
}

/**
 * @brief Push a received message into the ring buffer and notify the host task
 * 
 * This function adds a message to the RX ring buffer and wakes up the stack task
 * so it can process partial frames immediately. Used from ISR context.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @param data Message data to push to the ring buffer
 */
static void IRAM_ATTR push_msg(knx_tp_bit_bang_t * bb, knx_ring_buffer_data_t data)
{
    knx_ring_buffer_push_msg(&bb->rx_ring_buffer, data);
    // Wake the stack task so it can feed partial frames to the DLL immediately
    if (bb->xTaskToNotify != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(bb->xTaskToNotify, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Rearm the timer to the next scheduled event (TX or RX)
 * 
 * Determines which timer event should fire next (RX or TX) and configures the
 * hardware timer accordingly. RX events are prioritized as they are more common.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 */
void rearm_timer(knx_tp_bit_bang_t *bb) {
    const uint64_t tx_alarm = bb->tx_alarm_value;
    const uint64_t rx_alarm = bb->rx_alarm_value;
    
    // Fast path: RX events are much more common
    if (__builtin_expect(rx_alarm != 0, 1)) {
        if (tx_alarm == 0 || rx_alarm <= tx_alarm) {
            // RX is earliest or only alarm - hot path
            bb->flags &= ~KNX_TP_BIT_BANG_FLAG_NEXT_EVENT_TX;
            gptimer_set_alarm_action(bb->timer, &(gptimer_alarm_config_t){
                .alarm_count = rx_alarm,
                .flags.auto_reload_on_alarm = false
            });
            return;
        }
    }
    
    // TX is earliest or both zeroes - less common case
    if (__builtin_expect(tx_alarm != 0, 1)) {
        bb->flags |= KNX_TP_BIT_BANG_FLAG_NEXT_EVENT_TX;
        gptimer_set_alarm_action(bb->timer, &(gptimer_alarm_config_t){
            .alarm_count = tx_alarm,
            .flags.auto_reload_on_alarm = false
        });
        return;
    }
    // No alarms active - stop timer (rare case)
    // TODO: shut down timer to save power
}


// ******* RX processing helpers ********

/**
 * @brief Handle start bit reception
 * 
 * Initializes the receive state for a new byte. Clears the receive byte buffer
 * and resets the bit position counter.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @param bit The received start bit value (should be 0)
 * @return uint64_t Time in microseconds until the next bit should be sampled
 */
static inline uint64_t IRAM_ATTR rx_handle_start(knx_tp_bit_bang_t *bb, uint8_t bit)
{
    bb->rx_byte = 0; // Clear receive byte
    bb->rx_bit_position = 0;
    return KNX_BIT_TIME_US;
}

/**
 * @brief Handle parity bit reception
 * 
 * Validates the received parity bit against the calculated parity of the
 * received byte. Sets error flag if parity mismatch is detected.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @param bit The received parity bit value
 * @return uint64_t Time in microseconds until the next bit should be sampled
 */
static inline uint64_t IRAM_ATTR rx_handle_parity(knx_tp_bit_bang_t *bb, uint8_t bit)
{
    if (bit != calculate_parity_bit(bb->rx_byte)) {
        bb->rx_errors |= KNX_TP_BIT_BANG_RX_ERROR_PARITY;
    }
    bb->rx_bit_position = 9;
    return KNX_BIT_TIME_US;
}

/**
 * @brief Handle ACK stop bit reception
 * 
 * Processes the stop bit of an acknowledgment byte. If TX was waiting for this ACK,
 * it transitions TX to idle state. Otherwise, the ACK is reported as a standalone message.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @param bit The received stop bit value (should be 1)
 * @return uint64_t Time in microseconds until next expected event (typically 0)
 */
static inline uint64_t IRAM_ATTR rx_handle_ack_stop(knx_tp_bit_bang_t *bb, uint8_t bit)
{
    bb->flags &= ~KNX_TP_BIT_BANG_FLAG_RECEIVING_ACK;
    bb->rx_state = KNX_TP_BIT_BANG_RX_STATE_IDLE;
    if (bb->tx_state == KNX_TP_BIT_BANG_TX_STATE_WAITING_ACK) 
    {
        // TX was waiting for this ACK
        bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_IDLE;
        bb->tx_alarm_value = 0; // Clear TX alarm
        knx_ring_buffer_data_t msg = {.type = KNX_TP_BIT_BANG_MSG_TYPE_TX_ACK_RESPONSE, .data = bb->rx_byte};
        push_msg(bb, msg);
        return 0; // No further RX expected
    }
    // ACK received outside of TX context
    knx_ring_buffer_data_t ack_msg = {.type = KNX_TP_BIT_BANG_MSG_TYPE_ACK, .data = bb->rx_byte};
    push_msg(bb, ack_msg);
    return 0; // No further RX expected
    
}

/**
 * @brief Handle stop bit reception
 * 
 * Processes the stop bit of a received byte. Validates the stop bit value and
 * handles both ACK bytes and normal data bytes. Pushes the received data to
 * the ring buffer and sets up timing for potential additional bytes.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @param bit The received stop bit value (should be 1)
 * @return uint64_t Time in microseconds until next expected event
 */
static uint64_t IRAM_ATTR rx_handle_stop(knx_tp_bit_bang_t *bb, uint8_t bit)
{
    if (__builtin_expect(bit != 1, 0)) {
        bb->rx_errors |= KNX_TP_BIT_BANG_RX_ERROR_FRAMING;
        knx_ring_buffer_data_t err_msg = {.type = KNX_TP_BIT_BANG_MSG_TYPE_FRAMING_ERROR, .data = 0};
        push_msg(bb, err_msg);
    }

    if (__builtin_expect(bb->flags & KNX_TP_BIT_BANG_FLAG_RECEIVING_ACK, 0)) {
        // ACK byte received
        return rx_handle_ack_stop(bb, bit);
    } else {
        // Normal data byte received
        knx_ring_buffer_data_t data_msg = {.type = KNX_TP_BIT_BANG_MSG_TYPE_DATA, .data = bb->rx_byte};
        push_msg(bb, data_msg);
    }

    bb->rx_state = KNX_TP_BIT_BANG_RX_STATE_WAIT_MORE_DATA;
    return KNX_WAIT_MORE_DATA_TIMEOUT_US; // Wait for gap between bytes
}

/**
 * @brief Process a received bit during RX operation
 * 
 * Main bit processing function that handles the current bit based on position.
 * Optimized for data bits (hot path) with special handling for start, parity,
 * and stop bits.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @return uint64_t Time in microseconds until the next bit should be sampled
 */
static inline uint64_t IRAM_ATTR process_receive_bit(knx_tp_bit_bang_t *bb) {
    // Read current bit and clear flag asap
    const uint8_t bit = bb->rx_zero_detected ? 0 : 1;
    bb->rx_zero_detected = 0;

    int8_t pos = bb->rx_bit_position;

    if (__builtin_expect(IS_DATA_BIT(pos), 1)) {
        // Data bits 1..8 (hot path)
        bb->rx_byte |= (uint8_t)(bit << pos);
        bb->rx_bit_position = pos + 1;
        return KNX_BIT_TIME_US;
    }
    // Start bit (rare)
    switch(pos) {
        case -1:
            return rx_handle_start(bb, bit);
        case 8:
            return rx_handle_parity(bb, bit);
        case 9:
            return rx_handle_stop(bb, bit);
        default:
            // Should never happen - crash if we reach here
            abort();
    }
}

/**
 * @brief Telegram timeout/gap handler - runs when inter-byte gap indicates frame end
 * 
 * This is a cold path function that executes when the inter-byte gap timeout expires,
 * indicating the end of a telegram. Handles ACK transmission if requested by the host.
 * 
 * @param edata Timer event data from the hardware timer
 * @param bb Pointer to the KNX bit-bang instance
 */
static uint64_t IRAM_ATTR rx_handle_telegram_complete(
    const gptimer_alarm_event_data_t *edata,
    knx_tp_bit_bang_t *bb)
{
    uint8_t ack_byte = bb->pending_ack_byte;
    knx_tp_bit_bang_tx_state_t tx_state = bb->tx_state;
    uint8_t rx_errors = bb->rx_errors;
    bb->rx_errors = 0;
    // This timeout just indicates frame end
    knx_ring_buffer_data_t end_msg = {
        .type = KNX_TP_BIT_BANG_MSG_TYPE_END,
        .data = rx_errors
    };
    push_msg(bb, end_msg);
    if (ack_byte != 0) {
        if (tx_state != KNX_TP_BIT_BANG_TX_STATE_IDLE &&
            tx_state != KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT) {
            // Cannot send ACK now - TX in progress
            return 0; // ACK not sent
        }
        if (__builtin_expect(rx_errors != 0, 0)) {
            // Errors detected - add NACK to ACK byte
            bb->tx_current_byte= ack_byte & (~KNX_TP_BIT_BANG_ACK_BYTE_NACK);
        } else {
            // No errors - send requested ACK
            bb->tx_current_byte = ack_byte;
        }
        // Host requested an ACK via U_ACK_REQ
        bb->flags |= KNX_TP_BIT_BANG_FLAG_SENDING_ACK;
        bb->pending_ack_byte = 0;
        bb->tx_bit_position = -1;
        bb->tx_byte_position = 0;
        bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT;
        // Delay before sending ACK relative to last start bit: 11 bits (byte) + 15 bits idle
        bb->tx_alarm_value = bb->sync_point + KNX_SYNC_TO_ACK_DELAY_BIT_TIMES * KNX_BIT_TIME_US;
        return 0; // Timer will be rearmed by caller
    }
    return 0; // No further RX timer events needed
}


// ******* TX processing helpers ********

/**
 * @brief Send a single bit and return the time until next event
 * 
 * Transmits either a '1' bit (idle line) or '0' bit (active low with proper timing).
 * For zero bits, sets the TX line high and transitions to active transmission state.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @param bit The bit value to transmit (0 or 1)
 * @return uint64_t Time in microseconds until the next transmission event
 */
static inline IRAM_ATTR uint64_t send_bit(knx_tp_bit_bang_t *bb, uint8_t bit)
{
    if (bit != 0) {
        bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE;
        return KNX_BIT_TIME_US; // Send a one bit (leave line idle)
    }
    
    // Send a zero bit (active low)
    bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_ACTIVE;
    KNX_TX_HIGH();
    
    return KNX_ZERO_ACTIVE_TIME_US;
}

/**
 * @brief Handle start bit transmission
 * 
 * Transmits the start bit (always 0) and advances the bit position to the first data bit.
 * 
 * @param edata Timer event data from the hardware timer
 * @param bb Pointer to the KNX bit-bang instance
 * @return uint64_t Time in microseconds until the next transmission event
 */
static inline uint64_t IRAM_ATTR tx_handle_start_bit(const gptimer_alarm_event_data_t *edata, knx_tp_bit_bang_t *bb) {
    bb->tx_bit_position = 0; // Move to first data bit
    return send_bit(bb, 0); // Start bit is always 0
}

/**
 * @brief Handle parity bit transmission
 * 
 * Calculates and transmits the parity bit for the current byte and advances
 * the bit position to the stop bit.
 * 
 * @param edata Timer event data from the hardware timer
 * @param bb Pointer to the KNX bit-bang instance
 * @return uint64_t Time in microseconds until the next transmission event
 */
static inline uint64_t IRAM_ATTR tx_handle_parity_bit(const gptimer_alarm_event_data_t *edata, knx_tp_bit_bang_t *bb) {
    const uint8_t parity_bit = calculate_parity_bit(bb->tx_current_byte);
    bb->tx_bit_position = 9; // Move to stop bit
    return send_bit(bb, parity_bit);
}

/**
 * @brief Handle stop bit transmission
 * 
 * Transmits the stop bit and handles end-of-byte logic. For ACK bytes, transitions
 * to idle or resumes normal TX. For normal bytes, advances to next byte or waits for ACK.
 * 
 * @param edata Timer event data from the hardware timer
 * @param bb Pointer to the KNX bit-bang instance
 * @return uint64_t Time in microseconds until the next transmission event
 */
static inline uint64_t IRAM_ATTR tx_handle_stop_bit(const gptimer_alarm_event_data_t *edata, knx_tp_bit_bang_t *bb) {
    if (__builtin_expect((bb->flags & KNX_TP_BIT_BANG_FLAG_SENDING_ACK), 0)) {
        // Finished sending ACK
        bb->flags &= ~KNX_TP_BIT_BANG_FLAG_SENDING_ACK;
        if (bb->tx_telegram_length > 0) {
            // Resume normal TX if pending
            bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT;
            return ((bb->tx_wait_time + 1) * KNX_BIT_TIME_US); // Wait stop bit and wait delay before sending next telegram;
        }
        bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_IDLE;
        return 0; // No further action
    }
    // Move to next byte or wait for ACK
    bb->tx_byte_position++;
    bb->tx_bit_position = -1;
    if (__builtin_expect(bb->tx_byte_position >= bb->tx_telegram_length, 0)) {
        // No more bytes. End of telegram
        bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_WAITING_ACK;
        return KNX_BIT_TIME_US + KNX_ACK_TIMEOUT_US; // Send stop bit, wait for ACK
    }
    bb->tx_current_byte = bb->tx_buffer[bb->tx_byte_position];
    return KNX_BIT_TIME_US + KNX_INTER_BYTE_TIME_US; // Send stop bit, wait inter byte gap and start bit of next byte
}

/**
 * @brief Handle end of TX bit event and schedule next bit or state
 * 
 * Main transmission bit processing function. Optimized for data bits (hot path)
 * with special handling for start, parity, and stop bits. Extracts and transmits
 * the next bit from the current byte.
 * 
 * @param edata Timer event data from the hardware timer
 * @param bb Pointer to the KNX bit-bang instance
 * @return uint64_t Time in microseconds until the next transmission event
 */
static inline uint64_t IRAM_ATTR process_end_of_tx_bit(const gptimer_alarm_event_data_t *edata, knx_tp_bit_bang_t *bb) {
    const int8_t bit_pos = bb->tx_bit_position;

    if (__builtin_expect(IS_DATA_BIT(bit_pos), 1)) {
        // Hot path: data bits (1-8) - ~85% of execution time
        bb->tx_bit_position = bit_pos + 1;
        return send_bit(bb, (bb->tx_current_byte >> bit_pos) & 0x1); // Extract and send bit
    } 
    // Cold path: special bits (start, parity, stop)
    switch (bit_pos) {
        case -1:
            // Start bit (0)
            return tx_handle_start_bit(edata, bb);
        case 8:
            // Parity bit - direct access optimal for single use
            return tx_handle_parity_bit(edata, bb);
        case 9:
            // Stop bit (1)
            return tx_handle_stop_bit(edata, bb);
        default:
            // Should never happen - crash if we reach here
            abort();
    }
}


/**
 * @brief Handle ACK timeout during TX
 * 
 * Called when no ACK is received within the expected timeout period after
 * transmitting a telegram. Reports the timeout to the application layer.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @return uint64_t Time in microseconds until next event (typically 0)
 */
static uint64_t IRAM_ATTR tx_handle_waiting_ack(knx_tp_bit_bang_t *bb)
{
    // No ACK received before timeout
    // Normally ACK should be handled by rx_handle_ack_stop()
    knx_ring_buffer_data_t timeout_msg = {.type = KNX_TP_BIT_BANG_MSG_TYPE_TX_ACK_RESPONSE, .data = KNX_TP_BIT_BANG_ACK_BYTE_NONE};
    push_msg(bb, timeout_msg);
    bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_IDLE;
    return 0; // No further TX
}

/**
 * @brief Handle potential expiration of waiting slot at the beginning of TX
 * 
 * Implements the KNX medium access control by waiting for the appropriate slot time
 * before beginning transmission. Handles both ACK and normal telegram transmission.
 * 
 * @param edata Timer event data from the hardware timer
 * @param bb Pointer to the KNX bit-bang instance
 * @return uint64_t Time in microseconds until the next transmission event
 */
static uint64_t IRAM_ATTR tx_handle_waiting_slot(
    const gptimer_alarm_event_data_t *edata,
    knx_tp_bit_bang_t *bb)
{
    // Sync point is time of last start bit on bus
    uint64_t sync_point = bb->sync_point;
    bool sending_ack = (bb->flags & KNX_TP_BIT_BANG_FLAG_SENDING_ACK) != 0;
    // Wait time depends on whether sending ACK or normal telegram
    uint64_t wait_time = (KNX_BIT_TIME_US * bb->tx_wait_time);
    if (__builtin_expect(sending_ack || (edata->count_value - sync_point >= wait_time), 1)) {
        // Slot time expired, bus is idle (likely case)
        // Start sending the telegram, initialize positions
        bb->tx_bit_position = -1;
        bb->tx_byte_position = 0;
        if (bb->flags & KNX_TP_BIT_BANG_FLAG_SENDING_ACK) {
            // Sending ACK
            // ACK byte is already in tx_current_byte
        } else {
            // Sending normal telegram
            bb->tx_current_byte = bb->tx_buffer[0];
        }
        return tx_handle_start_bit(edata, bb); // Send start bit
    }
    // Slot time not expired, set the timer for the next slot
    return sync_point + wait_time - edata->alarm_value;
}

/**
 * @brief Process TX timer events
 * 
 * Main TX state machine that handles all transmission-related timer events.
 * Dispatches to appropriate handlers based on current TX state and manages
 * GPIO control and timing.
 * 
 * @param timer Handle to the hardware timer
 * @param edata Timer event data from the hardware timer
 * @param bb Pointer to the KNX bit-bang instance
 * @return bool Always returns false (no task yielding required)
 */
static inline bool IRAM_ATTR process_tx_timer(
                                gptimer_handle_t timer,    
                                const gptimer_alarm_event_data_t *edata, 
                                knx_tp_bit_bang_t *bb) {
    uint64_t alarm_value;
    
    switch (bb->tx_state) {
        // Most common cases first for better branch prediction
        case KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE:
        case KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_EQUALIZATION:
            // Set TX GPIO to idle state
            KNX_TX_LOW();
            
            // End of bit, move to the next one
            alarm_value = process_end_of_tx_bit(edata, bb);
            break;
            
        case KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_ACTIVE:
            // Single register write to set TX (and debug) GPIO to idle state
            KNX_TX_LOW();
            
            // End of zero active time - transition to equalization
            bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_EQUALIZATION;
            alarm_value = KNX_ZERO_EQUALIZATION_TIME_US;
            break;

        case KNX_TP_BIT_BANG_TX_STATE_WAITING_ACK:
            alarm_value = tx_handle_waiting_ack(bb);
            break;

        case KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT:
            alarm_value = tx_handle_waiting_slot(edata, bb);
            break;
        default:
            // Unknown state - should never happen
            abort();
    }
    if (__builtin_expect(alarm_value > 0, 1)) {
        bb->tx_alarm_value = edata->alarm_value + alarm_value;
    } else {
        bb->tx_alarm_value = 0;
    }
    return false;
}

/**
 * @brief Process RX timer events
 * 
 * Main RX state machine that handles all reception-related timer events.
 * Dispatches to appropriate handlers based on current RX state for bit sampling
 * and telegram completion detection.
 * 
 * @param edata Timer event data from the hardware timer
 * @param bb Pointer to the KNX bit-bang instance
 * @return bool Always returns false (no task yielding required)
 */
static inline bool IRAM_ATTR process_rx_timer(const gptimer_alarm_event_data_t *edata, knx_tp_bit_bang_t *bb) {

    const uint8_t state = bb->rx_state;
    uint64_t alarm_value;
    switch(state) {
        case KNX_TP_BIT_BANG_RX_STATE_IDLE:
            // No RX in progress - nothing to do
            alarm_value = 0;
            break;
        case KNX_TP_BIT_BANG_RX_STATE_RECEIVE:
            // Process received bit
            alarm_value = process_receive_bit(bb);
            break;
        case KNX_TP_BIT_BANG_RX_STATE_WAIT_MORE_DATA:
            // Inter-byte gap timeout - end of telegram
            alarm_value = rx_handle_telegram_complete(edata, bb);
            break;
        case KNX_TP_BIT_BANG_RX_STATE_WAIT_ACK:
            // Waiting for host to request ACK - handled below
            alarm_value = 0;
            break;
        default:
            // Unknown state - should never happen
            abort();
    }
    if (__builtin_expect(alarm_value > 0, 1)) {
        bb->rx_alarm_value = edata->alarm_value + alarm_value;
    } else {
        bb->rx_alarm_value = 0;
    }

    return false;
}


// ******* Interrupt Service Routines ********

/**
 * @brief Timer ISR - handles both TX and RX events
 * 
 * Main timer interrupt service routine that dispatches to TX or RX processing
 * based on the next scheduled event type. Re-arms the timer for the next event.
 * 
 * @param timer Handle to the hardware timer that generated the interrupt
 * @param edata Timer event data containing alarm count and other timing info
 * @param arg User argument pointer (knx_tp_bit_bang_t instance)
 * @return bool Always returns false (task yielding handled internally)
 */
bool knx_tp_bit_bang_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg) {
    knx_tp_bit_bang_t *bb = (knx_tp_bit_bang_t *)arg;
    
    if (__builtin_expect(bb->flags & KNX_TP_BIT_BANG_FLAG_NEXT_EVENT_TX, 0)) {
        // Process TX timer event (less common case)
        bb->tx_alarm_value = 0;
        process_tx_timer(timer, edata, bb);
    }
    else {
        // Process RX timer event (more common case)
        bb->rx_alarm_value = 0;
        process_rx_timer(edata, bb);
    }
    rearm_timer(bb);
    return false; // taskYIELD_FROM_ISR() has been handled already
}


// ******* RX Pin ISR - Zero Bit Detection ********

/**
 * @brief Reset TX state to idle and ensure TX line is idle
 * 
 * Called when a collision is detected during transmission. Resets the TX state
 * machine to wait for the next available slot and ensures the TX line is in idle state.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 */
static void reset_sending_state(knx_tp_bit_bang_t *bb) {
    KNX_TX_LOW(); // Ensure TX line is idle
    bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT;
    bb->tx_byte_position = 0;
    bb->tx_bit_position = -1;
    uint64_t current_time;
    gptimer_get_raw_count(bb->timer, &current_time);
    bb->tx_alarm_value = current_time + (KNX_BIT_TIME_US * bb->tx_wait_time);
}

/**
 * @brief Detect collision during zero bit transmission
 * 
 * Checks if a collision occurred during transmission of a zero bit. A collision
 * happens when another device transmits while we are sending a zero bit. If detected,
 * resets the transmission state to retry later.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @return bool True if collision was detected and handled, false otherwise
 */
static inline bool IRAM_ATTR collision_detect(knx_tp_bit_bang_t *bb) {
    if (__builtin_expect(bb->tx_state != KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_ACTIVE
        || bb->flags & (KNX_TP_BIT_BANG_FLAG_SENDING_ACK | KNX_TP_BIT_BANG_FLAG_NO_COLLISION_DETECT), 1)) {
        // Not sending a zero bit - no collision possible
        // Or collision detection disabled / sending ACK
        return false;
    }
    reset_sending_state(bb);
    return true;
}

/**
 * @brief RX pin ISR - handles zero bit detection
 * 
 * GPIO interrupt service routine triggered on rising edges of the RX pin,
 * indicating the end of a zero bit. Handles collision detection and synchronizes
 * RX timing for new telegrams or continuing reception.
 * 
 * @param arg User argument pointer (knx_tp_bit_bang_t instance)
 */
void knx_tp_bit_bang_rx_pin_isr(void *arg) {
    knx_tp_bit_bang_t *bb = (knx_tp_bit_bang_t *)arg;
    
    // Set zero detected flag using atomic write (single instruction)
    bb->rx_zero_detected = 1;

    collision_detect(bb);

    // Most interrupts happen mid-telegram (RECEIVE state) when a '0' bit ends (rising edge).
    // Starting a new telegram is comparatively rare -> mark as unlikely.
    if (__builtin_expect(bb->rx_state == KNX_TP_BIT_BANG_RX_STATE_IDLE, 0)) {
        // Start of new telegram
        bb->rx_bit_position = -1;
        bb->pending_ack_byte = 0;
        bb->rx_state = KNX_TP_BIT_BANG_RX_STATE_RECEIVE;
        
        // Synchronize RX timer and the sync_point
        gptimer_get_raw_count(bb->timer, &bb->sync_point);
        bb->rx_alarm_value = bb->sync_point + KNX_BIT_SAMPLING_OFFSET_US;
        rearm_timer(bb);
    } else if (bb->rx_state == KNX_TP_BIT_BANG_RX_STATE_WAIT_MORE_DATA) {
        // Receive new byte of telegram
        bb->rx_bit_position = -1;
        bb->rx_state = KNX_TP_BIT_BANG_RX_STATE_RECEIVE;

        // Synchronize RX timer and the sync_point
        gptimer_get_raw_count(bb->timer, &bb->sync_point);
        bb->rx_alarm_value = bb->sync_point + KNX_BIT_SAMPLING_OFFSET_US;
        rearm_timer(bb);
    }
    // For other states, do nothing (error states, etc.)
}


// ******* Public API functions ********

/**
 * @brief Rearm the timer for the next scheduled event
 * 
 * Public API function to manually rearm the timer. Typically called after
 * configuration changes that might affect timing.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if bb is NULL
 */
esp_err_t knx_tp_bit_bang_rearm_timer(knx_tp_bit_bang_t *bb) {
    if (bb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }    
    rearm_timer(bb);
    return ESP_OK;
}

/**
 * @brief Reset the TX state machine to idle
 * 
 * Public API function to reset the transmission state machine. Clears all TX
 * state variables and ensures the TX line is in idle state. Used for error
 * recovery or initialization.
 * 
 * @param bb Pointer to the KNX bit-bang instance
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if bb is NULL
 */
esp_err_t knx_tp_bit_bang_reset_tx(knx_tp_bit_bang_t *bb) {
    KNX_TX_LOW(); // Ensure TX line is idle
    if (bb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }    
    bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_IDLE;
    bb->tx_alarm_value = 0;
    bb->tx_bit_position = -1;
    bb->tx_byte_position = 0;
    bb->tx_telegram_length = 0;
    return ESP_OK;
}
