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
#define KNX_GPIO_SET_HIGH_FAST(mask) do { GPIO.out_w1ts.out_w1ts = (uint32_t)(mask); } while(0)
#define KNX_GPIO_SET_LOW_FAST(mask) do { GPIO.out_w1tc.out_w1tc = (uint32_t)(mask); } while(0)

// Optimized single-pin operations
#define KNX_TX_HIGH() KNX_GPIO_SET_HIGH_FAST(KNX_TX_PIN_MASK)
#define KNX_TX_LOW() KNX_GPIO_SET_LOW_FAST(KNX_TX_PIN_MASK)


#else
// Fallback to standard GPIO functions
#define KNX_TX_HIGH() gpio_set_level(CONFIG_KNX_TP_TX_PIN, 1)
#define KNX_TX_LOW() gpio_set_level(CONFIG_KNX_TP_TX_PIN, 0)

#endif // CONFIG_KNX_USE_DIRECT_GPIO

static inline uint8_t IRAM_ATTR calculate_parity_bit(uint8_t byte) {
    // Use built-in population count for better performance on modern architectures
    return __builtin_popcount(byte) & 1;
}

// Runs at most once per telegram; keep in IRAM but out-of-line to reduce hot ISR text
static uint8_t IRAM_ATTR __attribute__((noinline, cold)) calculate_checksum(const uint8_t *data, uint8_t length) {
    uint8_t calculated_checksum = 0xFF;
    if (length <= 1) return calculated_checksum;
    const uint8_t *p = data, *end = data + (size_t)length - 1;
    for (; p < end; ++p) {
        calculated_checksum ^= *p;
    }
    return calculated_checksum;
}

static inline IRAM_ATTR uint64_t send_bit(knx_tp_bit_bang_t *bit_bang, uint8_t bit)
{
    if (bit != 0) {
        bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE;
        return KNX_BIT_TIME_US; // Send a one bit (leave line idle)
    }
    
    // Send a zero bit (active low)
    bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_ACTIVE;
    KNX_TX_HIGH();
    
    return KNX_ZERO_ACTIVE_TIME_US;
}

// Runs once per telegram; keep out-of-line to shrink hot path
static bool IRAM_ATTR __attribute__((noinline, cold)) is_addressed_to_us(knx_tp_bit_bang_t *bit_bang)
{
    if (__builtin_expect(bit_bang->rx_telegram_length < 6, 0)) {
        return false;  // Invalid telegram
    }
    
    const uint8_t * const telegram = bit_bang->rx_buffer;
    const uint16_t dst_address = ((uint16_t)telegram[4] << 8) | telegram[5];
    const uint8_t dst_is_group = telegram[1] & 0x80;
    
    if (__builtin_expect(dst_is_group, 1)) {
        // Hot path: group addresses are more common than individual addressing
        // Linear search is optimal for small lists (n <= 16)
        // Cache-friendly: group_addresses array is in COLD section but small (32 bytes)
        const uint8_t count = bit_bang->group_address_count;
        const uint16_t * const addresses = bit_bang->group_addresses;
        for (uint8_t i = 0; i < count; i++) {
            if (addresses[i] == dst_address) {
                return true;
            }
        }
        return false;
    } else {
        // Individual address check (single comparison - very fast)
        return (dst_address == bit_bang->device_address);
    }
}

// Split rare RX branches into cold helpers to keep hot path small
static uint64_t IRAM_ATTR __attribute__((noinline, cold)) rx_handle_start(knx_tp_bit_bang_t *bb, uint8_t bit)
{
    if (bit != 0) {
        bb->errors |= KNX_TP_BIT_BANG_FRAMING_ERROR;
        bb->rx_state = KNX_TP_BIT_BANG_RX_STATE_IDLE;
        return 0;
    }
    bb->rx_buffer[bb->rx_byte_position] = 0; // Clear receive byte
    bb->rx_bit_position = 0;
    return KNX_BIT_TIME_US;
}

static uint64_t IRAM_ATTR __attribute__((noinline, cold)) rx_handle_parity(knx_tp_bit_bang_t *bb, uint8_t bit)
{
    if (bit != calculate_parity_bit(bb->rx_buffer[bb->rx_byte_position])) {
        bb->errors |= KNX_TP_BIT_BANG_PARITY_ERROR;
    }
    bb->rx_parity_bits |= (1u << bb->rx_byte_position);
    bb->rx_bit_position = 9;
    return KNX_BIT_TIME_US;
}

static uint64_t IRAM_ATTR __attribute__((noinline, cold)) rx_handle_stop(knx_tp_bit_bang_t *bb, uint8_t bit)
{
    if (__builtin_expect(bit != 1, 0)) {
        bb->errors |= KNX_TP_BIT_BANG_FRAMING_ERROR;
    }
    uint8_t next = (uint8_t)(bb->rx_byte_position + 1);
    bb->rx_byte_position = next;

    if (next >= KNX_TP_BIT_BANG_RX_BUFFER_SIZE) {
        bb->errors |= KNX_TP_BIT_BANG_RX_BUFFER_OVERFLOW;
        bb->rx_state = KNX_TP_BIT_BANG_RX_STATE_ERROR;
        return 0;
    }

    bb->rx_state = KNX_TP_BIT_BANG_RX_STATE_WAIT_MORE_DATA;
    return KNX_WAIT_MORE_DATA_TIMEOUT_US; // Wait for gap between bytes
}

//#define IS_DATA_BIT(pos) ((pos) >= 0 && (pos) <= 7)
// Use unsigned comparison to eliminate negative check
#define IS_DATA_BIT(pos) ((unsigned)(pos) <= 7u)

static inline uint64_t IRAM_ATTR process_end_of_tx_bit(const gptimer_alarm_event_data_t *edata, knx_tp_bit_bang_t *bit_bang) {
    const int8_t bit_pos = bit_bang->tx_bit_position;
    uint8_t bit = 0;

    if (__builtin_expect(IS_DATA_BIT(bit_pos), 1)) {
        // Hot path: data bits (1-8) - ~85% of execution time
        bit = ((bit_bang->tx_current_byte >> bit_pos) & 0x1); // Extract bit
    } else {
        // Cold path: special bits (start, parity, stop)
        switch (bit_pos) {
            case -1:
                // Start bit (0)
                bit_bang->sync_point = edata->count_value;
                bit = 0;
                break;
            case 8:
                // Parity bit - direct access optimal for single use
                bit = calculate_parity_bit(bit_bang->tx_current_byte);
                break;
            case 9:
                // Stop bit (1)
                if (__builtin_expect((bit_bang->flags & KNX_TP_BIT_BANG_FLAG_SENDING_ACK), 0)) {
                    // Finished sending ACK
                    bit_bang->flags &= ~KNX_TP_BIT_BANG_FLAG_SENDING_ACK;
                    if (__builtin_expect(bit_bang->tx_telegram_length > 0, 0)) {
                        // More data to send
                        bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT;
                        return KNX_INTER_BYTE_TIME_US; // Wait inter byte gap before sending next byte
                    }
                    bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_IDLE;
                    return 0; // No further action
                }
                // Move to next byte or wait for ACK
                bit_bang->tx_byte_position++;
                bit_bang->tx_bit_position = -1;
                if (__builtin_expect(bit_bang->tx_byte_position >= bit_bang->tx_telegram_length, 0)) {
                    // No more bytes. End of telegram
                    bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_WAITING_ACK;
                    return KNX_BIT_TIME_US + KNX_ACK_TIMEOUT_US; // Send stop bit, wait for ACK
                }
                bit_bang->tx_current_byte = bit_bang->tx_buffer[bit_bang->tx_byte_position];
                return KNX_BIT_TIME_US + KNX_INTER_BYTE_TIME_US; // Send stop bit, wait inter byte gap and start bit of next byte
            default:
                // Should never happen - crash if we reach here
                abort();
        }
    }
    bit_bang->tx_bit_position++;
    return send_bit(bit_bang, bit);
}

static inline bool IRAM_ATTR signal_task_with_event(knx_tp_bit_bang_t *bit_bang, knx_event_t event) {
#ifdef CONFIG_KNX_ENABLE_TASK_NOTIFICATIONS
    // Only compile task notification code if enabled
    if (__builtin_expect(bit_bang->xTaskToNotify != NULL, 0)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        bit_bang->last_event = event;
        vTaskNotifyGiveFromISR(bit_bang->xTaskToNotify, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
#else
    // Compile to nothing if task notifications are disabled
    (void)bit_bang;
    (void)event;
#endif
    return false;
}


static inline uint64_t IRAM_ATTR process_receive_bit(knx_tp_bit_bang_t *bit_bang) {
    // Read current bit and clear flag asap
    const uint8_t bit = bit_bang->rx_zero_detected ? 0 : 1;
    bit_bang->rx_zero_detected = 0;

    int8_t pos = bit_bang->rx_bit_position;
    uint8_t byte_pos = bit_bang->rx_byte_position;
    uint8_t *rx_buf = bit_bang->rx_buffer;

    if (__builtin_expect(IS_DATA_BIT(pos), 1)) {
        // Data bits 1..8 (hot path)
        rx_buf[byte_pos] |= (uint8_t)(bit << pos);
        bit_bang->rx_bit_position = pos + 1;
        return KNX_BIT_TIME_US;
    }
    // Start bit (rare)
    switch(pos) {
        case -1:
            return rx_handle_start(bit_bang, bit);
        case 8:
            return rx_handle_parity(bit_bang, bit);
        case 9:
            return rx_handle_stop(bit_bang, bit);
        default:
            // Should never happen, but handle gracefully
            bit_bang->rx_state = KNX_TP_BIT_BANG_RX_STATE_ERROR;
            return 0;
    }
}

void rearm_timer(knx_tp_bit_bang_t *bit_bang) {
    // Optimized: eliminate extra loads and branches
    const uint64_t tx_alarm = bit_bang->tx_alarm_value;
    const uint64_t rx_alarm = bit_bang->rx_alarm_value;
    
    // Fast path: RX events are much more common
    if (__builtin_expect(rx_alarm != 0, 1)) {
        if (tx_alarm == 0 || rx_alarm <= tx_alarm) {
            // RX is earliest or only alarm - hot path
            bit_bang->flags &= ~KNX_TP_BIT_BANG_FLAG_NEXT_EVENT_TX;
            gptimer_set_alarm_action(bit_bang->timer, &(gptimer_alarm_config_t){
                .alarm_count = rx_alarm,
                .flags.auto_reload_on_alarm = false
            });
            return;
        }
    }
    
    // TX is earliest or both zeroes - less common case
    if (__builtin_expect(tx_alarm != 0, 1)) {
        bit_bang->flags |= KNX_TP_BIT_BANG_FLAG_NEXT_EVENT_TX;
        gptimer_set_alarm_action(bit_bang->timer, &(gptimer_alarm_config_t){
            .alarm_count = tx_alarm,
            .flags.auto_reload_on_alarm = false
        });
        return;
    }
    // No alarms active - stop timer (rare case)
    // TODO: shut down timer to save power
}

esp_err_t knx_tp_bit_bang_rearm_timer(knx_tp_bit_bang_t *bit_bang) {
    if (bit_bang == NULL) {
        return ESP_ERR_INVALID_ARG;
    }    
    rearm_timer(bit_bang);
    return ESP_OK;
}

esp_err_t knx_tp_bit_bang_reset_tx(knx_tp_bit_bang_t *bit_bang) {
    KNX_TX_LOW(); // Ensure TX line is idle
    if (bit_bang == NULL) {
        return ESP_ERR_INVALID_ARG;
    }    
    bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_IDLE;
    bit_bang->tx_alarm_value = 0;
    bit_bang->tx_bit_position = -1;
    bit_bang->tx_byte_position = 0;
    bit_bang->tx_telegram_length = 0;
    return ESP_OK;
}

// Split rare TX states into cold helpers to keep hot path small
static bool IRAM_ATTR __attribute__((noinline, cold)) tx_handle_waiting_ack(knx_tp_bit_bang_t *bb)
{
    // ACK wait timeout expired
    bb->tx_alarm_value = 0;
    bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_ACK_NOT_RECEIVED;
    return signal_task_with_event(bb, KNX_EVENT_ACK_NOT_RECEIVED);
}

static bool IRAM_ATTR __attribute__((noinline, cold)) tx_handle_waiting_slot(
    const gptimer_alarm_event_data_t *edata,
    knx_tp_bit_bang_t *bb)
{
    uint64_t sync_point = bb->sync_point;
    if (__builtin_expect(edata->count_value - sync_point >= KNX_TELEGRAM_SPACING, 1)) {
        // Slot time expired, bus is idle (likely case)
        // Start sending the telegram, initialize positions
        bb->tx_bit_position = -1;
        bb->tx_byte_position = 0;
        bb->tx_current_byte = bb->tx_buffer[0];
        bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE;
        uint64_t alarm_value = process_end_of_tx_bit(edata, bb);
        if (__builtin_expect(alarm_value > 0, 1)) {
            bb->tx_alarm_value = edata->alarm_value + alarm_value;
        }
    } else {
        // Slot time not expired, set the timer for the next slot
        bb->tx_alarm_value = sync_point + KNX_TELEGRAM_SPACING;
    }
    return false;
}

static bool IRAM_ATTR __attribute__((noinline, cold)) tx_handle_error(knx_tp_bit_bang_t *bb)
{
    // Unexpected state, stop the timer
    bb->tx_alarm_value = 0;
    bb->tx_state = KNX_TP_BIT_BANG_TX_STATE_ERROR;
    return false;
}

static inline bool IRAM_ATTR process_tx_timer(
                                gptimer_handle_t timer,    
                                const gptimer_alarm_event_data_t *edata, 
                                knx_tp_bit_bang_t *bit_bang) {
    uint64_t alarm_value;
    
    // Ultra-fast GPIO operations using compile-time optimized direct register access
    switch (bit_bang->tx_state) {
        // Most common cases first for better branch prediction
        case KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE:
        case KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_EQUALIZATION:
            // Single register write to set TX (and debug) GPIO to idle state
            KNX_TX_LOW();
            
            // End of bit, move to the next one
            alarm_value = process_end_of_tx_bit(edata, bit_bang);
            if (__builtin_expect(alarm_value > 0, 1)) {
                bit_bang->tx_alarm_value = edata->alarm_value + alarm_value;
            }
            return false;
            
        case KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_ACTIVE:
            // Single register write to set TX (and debug) GPIO to idle state
            KNX_TX_LOW();
            
            // End of zero active time - transition to equalization
            bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_EQUALIZATION;
            bit_bang->tx_alarm_value = edata->alarm_value + KNX_ZERO_EQUALIZATION_TIME_US;
            return false;

        case KNX_TP_BIT_BANG_TX_STATE_WAITING_ACK:
            return tx_handle_waiting_ack(bit_bang);
            
        case KNX_TP_BIT_BANG_TX_STATE_WAITING_SLOT:
            return tx_handle_waiting_slot(edata, bit_bang);
            
        default:
            return tx_handle_error(bit_bang);
    }
}

static inline void IRAM_ATTR collision_detect(knx_tp_bit_bang_t *bit_bang) {
    if (__builtin_expect((bit_bang->flags & (KNX_TP_BIT_BANG_FLAG_NO_COLLISION_DETECT | KNX_TP_BIT_BANG_FLAG_SENDING_ACK))
        || ((bit_bang->tx_state != KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE) && (bit_bang->tx_state != KNX_TP_BIT_BANG_TX_STATE_SENDING_ZERO_EQUALIZATION))
        , 1)) {
        // No collision detection during ACK sending, or when disabled, or when sending not sending passive bits
        return;
    }

    // Collision detected
    // Reset TX state
    bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_COLLISION;
    bit_bang->tx_alarm_value = 0; // Stop the TX timer
    signal_task_with_event(bit_bang, KNX_EVENT_COLLISION);
}

// Telegram completion handler - runs once per telegram (cold path)
static bool IRAM_ATTR __attribute__((noinline, cold)) rx_handle_telegram_complete(
    const gptimer_alarm_event_data_t *edata,
    knx_tp_bit_bang_t *bit_bang)
{
    // Telegram is complete - finalize RX state quickly
    bit_bang->rx_state = KNX_TP_BIT_BANG_RX_STATE_IDLE;
    bit_bang->rx_telegram_length = bit_bang->rx_byte_position;
    bit_bang->rx_alarm_value = 0;

    // Cache hot fields locally to minimize repeated loads
    const uint32_t flags = bit_bang->flags;
    const uint8_t * const buf = bit_bang->rx_buffer;
    const uint8_t len = bit_bang->rx_telegram_length;
    const uint8_t errors = bit_bang->errors;

    bool is_for_us = is_addressed_to_us(bit_bang);

    // Early exit: if not addressed to us and not in promiscuous mode, nothing to do
    if (__builtin_expect(!is_for_us && !(flags & KNX_TP_BIT_BANG_FLAG_PROMISCUOUS), 1)) {
        return false;
    }

    bool is_valid = true;

    // If it's for us, determine validity (errors + checksum)
    if (is_for_us) {
        const uint8_t err_mask = (KNX_TP_BIT_BANG_FRAMING_ERROR | KNX_TP_BIT_BANG_PARITY_ERROR | KNX_TP_BIT_BANG_RX_BUFFER_OVERFLOW);
        if (__builtin_expect((errors & err_mask) == 0, 1)) {
            const uint8_t calc = calculate_checksum(buf, len);
            const uint8_t recv = buf[len - 1];
            is_valid = (calc == recv);
        } else {
            is_valid = false;
        }

        // Send ACK/NACK/BUSY when enabled
        if (flags & KNX_TP_BIT_BANG_FLAG_AUTO_ACK) {
            uint8_t ack_byte = is_valid ? KNX_TP_BIT_BANG_ACK_BYTE_ACK : KNX_TP_BIT_BANG_ACK_BYTE_NACK;
            if (flags & KNX_TP_BIT_BANG_FLAG_BUSY) {
                ack_byte = KNX_TP_BIT_BANG_ACK_BYTE_BUSY;
            }
            bit_bang->flags |= KNX_TP_BIT_BANG_FLAG_SENDING_ACK;
            bit_bang->tx_current_byte = ack_byte;
            bit_bang->tx_bit_position = -1;
            bit_bang->tx_byte_position = 0;
            bit_bang->tx_state = KNX_TP_BIT_BANG_TX_STATE_SENDING_ONE;
            // Delay before sending ACK relative to last start bit: 11 bits (byte) + 15 bits idle
            bit_bang->tx_alarm_value = bit_bang->sync_point + (uint64_t)((11 + 15) * KNX_BIT_TIME_US);
        }
    }

    // Decide if we should store to ring buffer
    if (__builtin_expect(((is_for_us && is_valid) || (flags & KNX_TP_BIT_BANG_FLAG_PROMISCUOUS)), 1)) {
        knx_ring_buffer_push(&bit_bang->rx_ring_buffer,
                             buf,
                             len,
                             errors,
                             bit_bang->rx_parity_bits,
                             edata->count_value);
    }

    // Optionally signal task for backward compatibility (if enabled)
    #ifdef CONFIG_KNX_ENABLE_TASK_NOTIFICATIONS
    if (bit_bang->xTaskToNotify != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        bit_bang->last_event = KNX_EVENT_TELEGRAM_RECEIVED;
        vTaskNotifyGiveFromISR(bit_bang->xTaskToNotify, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }
    #endif
    return false;
}

static inline bool IRAM_ATTR process_rx_timer(const gptimer_alarm_event_data_t *edata, knx_tp_bit_bang_t *bit_bang) {

    const uint8_t state = bit_bang->rx_state;
    if (__builtin_expect(state == KNX_TP_BIT_BANG_RX_STATE_RECEIVE, 1)) {
        const uint64_t delta = process_receive_bit(bit_bang);
        if (delta > 0) {
            bit_bang->rx_alarm_value = edata->alarm_value + delta;
        }
        return false;
    }

    if (__builtin_expect(state == KNX_TP_BIT_BANG_RX_STATE_WAIT_MORE_DATA, 0)) {
        return rx_handle_telegram_complete(edata, bit_bang);
    }

    // Other states: nothing to do
    return false;
}


bool knx_tp_bit_bang_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg) {
    knx_tp_bit_bang_t *bit_bang = (knx_tp_bit_bang_t *)arg;
    bool ret;
    uint64_t current_timer;
    int32_t *current_duration = NULL;

    if (__builtin_expect(bit_bang->flags & KNX_TP_BIT_BANG_FLAG_NEXT_EVENT_TX, 0)) {
        // Process TX timer event (less common case)
        bit_bang->tx_timer_count++;
        bit_bang->tx_alarm_value = 0;
        ret = process_tx_timer(timer, edata, bit_bang);
        // Store timing margin for diagnostics (in microseconds)
        current_duration = &bit_bang->tx_timer_durations[bit_bang->tx_timer_count % TIMER_MARGINES_SIZE];
        bit_bang->tx_timer_delay[bit_bang->tx_timer_count % TIMER_MARGINES_SIZE] = (int32_t)(edata->count_value - edata->alarm_value);
    }
    else {
        // Process RX timer event (more common case)
        bit_bang->rx_timer_count++;
        bit_bang->rx_alarm_value = 0;
        ret = process_rx_timer(edata, bit_bang);
        current_duration = &bit_bang->rx_timer_durations[bit_bang->rx_timer_count % TIMER_MARGINES_SIZE];
        bit_bang->rx_timer_delays[bit_bang->rx_timer_count % TIMER_MARGINES_SIZE] = (int32_t)(edata->count_value - edata->alarm_value);
    }
    rearm_timer(bit_bang);
    gptimer_get_raw_count(bit_bang->timer, &current_timer);
    *current_duration = (int32_t)(current_timer - edata->count_value);
    return ret;
}

void knx_tp_bit_bang_rx_pin_isr(void *arg) {
    knx_tp_bit_bang_t *bit_bang = (knx_tp_bit_bang_t *)arg;
    
    // Set zero detected flag using atomic write (single instruction)
    bit_bang->rx_zero_detected = 1;
    
    collision_detect(bit_bang);


    // Most interrupts happen mid-telegram (RECEIVE state) when a '0' bit ends (rising edge).
    // Starting a new telegram is comparatively rare -> mark as unlikely.
    if (__builtin_expect(bit_bang->rx_state == KNX_TP_BIT_BANG_RX_STATE_IDLE, 0)) {
        // Start of new telegram
        bit_bang->rx_byte_position = 0;
        bit_bang->rx_parity_bits = 0;
        bit_bang->rx_bit_position = -1;
        bit_bang->rx_state = KNX_TP_BIT_BANG_RX_STATE_RECEIVE;
        
        // Synchronize RX timer and the sync_point
        gptimer_get_raw_count(bit_bang->timer, &bit_bang->sync_point);
        bit_bang->rx_alarm_value = bit_bang->sync_point + KNX_BIT_SAMPLING_OFFSET_US;
        rearm_timer(bit_bang);
    } else if (bit_bang->rx_state == KNX_TP_BIT_BANG_RX_STATE_WAIT_MORE_DATA) {
        // Receive new byte of telegram
        bit_bang->rx_bit_position = -1;
        bit_bang->rx_state = KNX_TP_BIT_BANG_RX_STATE_RECEIVE;
        
        // Synchronize RX timer and the sync_point
        gptimer_get_raw_count(bit_bang->timer, &bit_bang->sync_point);
        bit_bang->rx_alarm_value = bit_bang->sync_point + KNX_BIT_SAMPLING_OFFSET_US;
        rearm_timer(bit_bang);
    }
    // For other states, do nothing (error states, etc.)
}
