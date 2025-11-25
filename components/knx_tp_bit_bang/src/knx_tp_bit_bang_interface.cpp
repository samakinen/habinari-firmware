#include "knx_tp_bit_bang_interface.h"
#include "knx_tp_bit_bang.h"
#include "knx_ring_buffer.h"
#include <deque>
#include <vector>
#include <cstring>
#include <esp_log.h>
#include "TPUart/Types.h"

static const char *TAG = "KnxTpBitBangIF";

namespace TPUart {
namespace Interface {

    // Use TPUart Types.h macros directly; avoid redefining names

    /**
     * @brief Implementation class for KNX TP Bit-Bang Interface
     * 
     * Handles TPUART emulation on top of the ISR-based bit-bang driver.
     * The driver handles timing-critical operations, this class handles protocol emulation.
     */

    class KnxTpBitBangInterfaceImpl {
      public:
        knx_tp_bit_bang_t _bitbang;
        bool _initialized = false;
        std::deque<uint8_t> _rx_queue; // bytes available to read by host
        std::vector<uint8_t> _tx_buffer; // building outgoing telegram from host
        bool _receiving = false;
        bool _expect_data = false;
        bool _last_was_end = false;
        int _expect_addr_bytes = 0;
        uint8_t _addr_bytes[3];
        bool _monitoring = false;
        bool _stopped = false;
        
        std::function<bool()> _cb;
        TaskHandle_t _task_handle = nullptr; // Task handle for direct notifications from ISR

        // Command state machine for multi-byte commands
        enum CommandState {
            CMD_IDLE
        };
        CommandState _cmd_state = CMD_IDLE;
        int _cmd_bytes_remaining = 0;
        uint8_t _cmd_buffer[2]; // For multi-byte commands (max 2 bytes for TPUART)
        uint8_t _cmd_opcode = 0; // Store command opcode for multi-step commands

        KnxTpBitBangInterfaceImpl() {
            memset(&_bitbang, 0, sizeof(_bitbang));
            _tx_buffer.reserve(64);
        }

        ~KnxTpBitBangInterfaceImpl() {}

        void init() {
            if (!_initialized) {
                esp_err_t err = knx_tp_bit_bang_init(&_bitbang);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "knx_tp_bit_bang_init failed: %s", esp_err_to_name(err));
                } else {
                    // Register this interface as the ISR notification target
                    _bitbang.xTaskToNotify = _task_handle;
                    ESP_LOGI(TAG, "Initialized bit-bang driver with impl %p, stored task handle %p, bitbang.task=%p",
                             this, _task_handle, _bitbang.xTaskToNotify);
                    _initialized = true;
                }
            }
        }

        /**
         * @brief Check if there are more incoming messages to process
         * 
         * @return true if there are more messages to process
         * @return false if no more messages are pending
         */
        bool hasMoreToProcess() const {
            return !knx_ring_buffer_is_empty(&_bitbang.rx_ring_buffer);
        }

        /**
         * @brief Pump incoming messages from the driver ring buffer to host queue
         * 
         * Processes messages from the ISR ring buffer and converts them to TPUART
         * protocol format. Handles telegram assembly and CRC calculation.
         */
        void pollIncoming() {
            if (!_initialized) return;

            knx_ring_buffer_data_t msg;
            int msg_count = 0;
            
            while (knx_ring_buffer_pop_msg(&_bitbang.rx_ring_buffer, &msg)) {
                // Debug: log each message
                const char *type_str = "UNKNOWN";
                switch (msg.type) {
                    case KNX_TP_BIT_BANG_MSG_TYPE_START: type_str = "START"; break;
                    case KNX_TP_BIT_BANG_MSG_TYPE_DATA: type_str = "DATA"; break;
                    case KNX_TP_BIT_BANG_MSG_TYPE_END: type_str = "END"; break;
                    case KNX_TP_BIT_BANG_MSG_TYPE_ACK: type_str = "ACK"; break;
                    case KNX_TP_BIT_BANG_MSG_TYPE_TX_ACK_RESPONSE: type_str = "TX_ACK"; break;
                    case KNX_TP_BIT_BANG_MSG_TYPE_PARITY_ERROR: type_str = "PARITY_ERR"; break;
                    case KNX_TP_BIT_BANG_MSG_TYPE_FRAMING_ERROR: type_str = "FRAMING_ERR"; break;
                    default: break;
                }
                // ESP_LOGI(TAG, "RX msg: 0x%02X (type: %s)", msg.data, type_str);

                switch (msg.type) {
                    case KNX_TP_BIT_BANG_MSG_TYPE_START:
                        // First byte of telegram - start new frame
                        
                        // Determine frame type and send appropriate indication
                        if ((msg.data & 0x53) == 0x10) {
                            // Standard frame (control byte pattern)
                            _rx_queue.push_back(L_DATA_STANDARD_IND);
                        } else {
                            // Extended frame
                            _rx_queue.push_back(L_DATA_EXTENDED_IND);
                        }
                        _rx_queue.push_back(msg.data);
                        break;

                    case KNX_TP_BIT_BANG_MSG_TYPE_DATA:
                        // Data byte received
                        _rx_queue.push_back(msg.data);
                        break;

                    case KNX_TP_BIT_BANG_MSG_TYPE_END:
                        // End of telegram - send end marker
                        _rx_queue.push_back(0x00); // End marker
                        // Send frame end indication for complete TPUART emulation
                        _rx_queue.push_back(U_FRAME_END_IND);
                        break;

                    case KNX_TP_BIT_BANG_MSG_TYPE_TX_ACK_RESPONSE:
                        // ACK response for transmitted telegram
                        {
                            bool success = (msg.data == KNX_TP_BIT_BANG_ACK_BYTE_ACK);
                            uint8_t response = success ? 
                                (uint8_t)(L_DATA_CON | 0x80) : (uint8_t)L_DATA_CON;  // 0x80 = SUCCESS bit
                            _rx_queue.push_back(response);
                            ESP_LOGI(TAG, "TX result: %s (ACK=0x%02X)", 
                                     success ? "SUCCESS" : "FAILED", msg.data);
                            
                            // If failed, check for collision and send frame state if needed
                            if (!success && msg.data == KNX_TP_BIT_BANG_ACK_BYTE_NACK) {
                                _rx_queue.push_back(U_FRAME_STATE_IND);
                                _rx_queue.push_back(TRANSMIT_ERROR);
                            }
                        }
                        break;

                    case KNX_TP_BIT_BANG_MSG_TYPE_ACK:
                        // Standalone ACK byte (not related to our transmission)
                        // In NCN5120 mode, this would typically not be forwarded to host
                        ESP_LOGI(TAG, "Standalone ACK received: 0x%02X", msg.data);
                        break;

                    case KNX_TP_BIT_BANG_MSG_TYPE_PARITY_ERROR:
                        ESP_LOGW(TAG, "RX error: %s", type_str);
                        // Send frame state indication with parity error
                        _rx_queue.push_back(U_FRAME_STATE_IND);
                        _rx_queue.push_back(PARITY_BIT_ERROR);
                        break;
                    case KNX_TP_BIT_BANG_MSG_TYPE_FRAMING_ERROR:
                        ESP_LOGW(TAG, "RX error: %s", type_str);
                        // Send frame state indication with timing error
                        _rx_queue.push_back(U_FRAME_STATE_IND);
                        _rx_queue.push_back(TIMING_ERROR);
                        break;

                    default:
                        ESP_LOGW(TAG, "Unknown message type: %d", msg.type);
                        break;
                }
                
                msg_count++;

                // Notify via callback if registered
                if (_cb) {
                    _cb();
                }
            }
            
            // Debug logging: show message flow
            if (msg_count > 0) {
                // ESP_LOGI(TAG, "Processed %d messages", msg_count);
            }
        }
    };

    // Small pimpl to keep header clean
    static KnxTpBitBangInterfaceImpl *g_impl = nullptr;

    void KnxTpBitBangInterface::flush()
    {
        // nothing to flush for bit-bang
    }

    void KnxTpBitBangInterface::begin(int /*baud*/)
    {
        if (!g_impl) {
            g_impl = new KnxTpBitBangInterfaceImpl();
            ESP_LOGI(TAG, "Created KnxTpBitBangInterfaceImpl %p in begin()", g_impl);
        }
        // Initialize the bit-bang driver once; it will use whatever
        // task handle was previously set via setTaskHandle (if any).
        g_impl->init();
    }

    void KnxTpBitBangInterface::end()
    {
        if (g_impl) {
            // Deinitialize bit-bang resources to avoid dangling ISRs/timers
            knx_tp_bit_bang_deinit(&g_impl->_bitbang);
            delete g_impl;
            g_impl = nullptr;
        }
    }

    bool KnxTpBitBangInterface::available()
    {
        if (!g_impl) return false;
        // Poll driver for new telegrams
        g_impl->pollIncoming();
        return !g_impl->_rx_queue.empty();
    }

    bool KnxTpBitBangInterface::availableForWrite()
    {
        // Always allow write; transmission is queued to driver
        return true;
    }

    bool KnxTpBitBangInterface::write(char value)
    {
        if (!g_impl) return false;
        uint8_t byte = (uint8_t)value;

        // Handle SET ADDRESS request which expects following bytes
        if (g_impl->_expect_addr_bytes > 0) {
            // For TPUART2, two bytes (high, low) 
            int idx = (g_impl->_expect_addr_bytes == 2) ? 0 : 1;
            g_impl->_addr_bytes[idx] = byte;
            g_impl->_expect_addr_bytes--;
            if (g_impl->_expect_addr_bytes == 0) {
                uint16_t addr = ((uint16_t)g_impl->_addr_bytes[0] << 8) | g_impl->_addr_bytes[1];
                knx_tp_bit_bang_set_device_address(&g_impl->_bitbang, addr);
                ESP_LOGI(TAG, "TPUART: Set device address to 0x%04x", addr);
                // Note: ACK behavior is now handled automatically by ISR
            }
            return true;
        }

        // Data byte following a position command or repetition setting
        if (g_impl->_expect_data) {
            // Check if this is for telegram transmission or configuration
            if (g_impl->_receiving) {
                // Add to telegram buffer
                g_impl->_tx_buffer.push_back(byte);
                ESP_LOGI(TAG, "Added data byte 0x%02X to buffer (size: %d)", byte, g_impl->_tx_buffer.size());
                
                if (g_impl->_last_was_end) {
                    // This was the last byte - process telegram
                    ESP_LOGI(TAG, "Received complete telegram (%d bytes)", g_impl->_tx_buffer.size());
                    if (!g_impl->_tx_buffer.empty()) {
                        bool should_send = !g_impl->_monitoring;
                        if (should_send) {
                            ESP_LOGI(TAG, "Sending telegram via driver...");
                            knx_tp_bit_bang_send(&g_impl->_bitbang, g_impl->_tx_buffer.data(), g_impl->_tx_buffer.size());
                        } else {
                            ESP_LOGI(TAG, "Monitor mode: not sending telegram to bus");
                        }
                    }
                    g_impl->_tx_buffer.clear();
                    g_impl->_receiving = false;
                    g_impl->_last_was_end = false;
                } else {
                    // Expect more data
                    g_impl->_expect_data = false;
                }
            } else {
                // This is configuration data (e.g., repetition count)
                ESP_LOGI(TAG, "TPUART: Configuration data received: 0x%02X", byte);
                g_impl->_expect_data = false;
            }
            
            return true;
        }

        if (byte == U_RESET_REQ) {
            // Reset behavior: leave monitor mode
            g_impl->_monitoring = false;
            // Note: New implementation handles mode changes differently
            // Respond with reset indication
            g_impl->_rx_queue.push_back(U_RESET_IND);
            // Don't trigger callback here - responses are read via polling
            return true;
        }

        if (byte == U_STATE_REQ) {
            // Send state indication with status flags
            g_impl->_rx_queue.push_back(U_STATE_IND);
            // Add status byte: combine various state flags
            uint8_t status = 0x00; // Start with no errors
            if (g_impl->_stopped) status |= 0x04; // Custom stopped flag
            if (g_impl->_monitoring) status |= 0x02; // Custom monitoring flag
            g_impl->_rx_queue.push_back(status);
            return true;
        }

        if (byte == U_SYSTEM_STATE_REQ) {
            // Return system state indication with detailed status flags
            g_impl->_rx_queue.push_back(U_SYSTEM_STAT_IND);
            // System status byte: no errors, normal operation
            uint8_t system_status = 0x00; // No temperature warning, protocol error, etc.
            g_impl->_rx_queue.push_back(system_status);
            return true;
        }

        if (byte == U_POLLING_STATE_REQ) {
            // Polling state response - indicate ready for communication
            g_impl->_rx_queue.push_back(U_STATE_IND);
            return true;
        }

        // TPUART SET ADDRESS (2 bytes: hi, lo)
        if (byte == U_TPUART2_SET_ADDRESS_REQ) {
            // Expect two address bytes next (high, low)
            g_impl->_expect_addr_bytes = 2;
            return true;
        }

        // TPUART SET REPETITION (1 byte follows: repetitions)
        if (byte == U_TPUART2_SET_REPETITION_REQ) {
            g_impl->_expect_data = true; // Expect 1 byte for repetition count
            ESP_LOGI(TAG, "TPUART: Set repetition request received");
            return true;
        }

        // TPUART CRC activation (acknowledge but don't actually implement)
        if (byte == U_TPUART2_ACTIVATECRC_REQ) {
            ESP_LOGI(TAG, "TPUART: CRC activation request (not implemented in bit-bang)");
            // Send configuration indication to acknowledge
            g_impl->_rx_queue.push_back(U_CONFIGURE_IND);
            g_impl->_rx_queue.push_back(0x00); // Configuration status: CRC not active
            return true;
        }



        // Position/offset commands
        if ((byte & 0xF8) == U_L_DATA_OFFSET_REQ) {
            // ignore for now (small frames)
            return true;
        }

        // U_L_DataStart: exactly 0x80 (10000000)
        if (byte == U_L_DATA_START_REQ) {
            g_impl->_tx_buffer.clear();
            g_impl->_receiving = true;
            g_impl->_expect_data = true;
            g_impl->_last_was_end = false;
            ESP_LOGI(TAG, "U_L_DATA_START_REQ: Starting new telegram");
            return true;
        }
        
        // U_L_DataContinue: 0x81-0xBF (but spec says 0x81-0xBE, so use 0xBE as max)
        // Pattern: 10iiiiii where i = index (1-62)
        if ((byte & 0xC0) == 0x80 && byte >= 0x81 && byte <= 0xBE) {
            if (!g_impl->_receiving) {
                ESP_LOGE(TAG, "U_L_DATA_CONT_REQ without START");
                return false;
            }
            g_impl->_expect_data = true;
            g_impl->_last_was_end = false;
            uint8_t index = byte & 0x3F;
            ESP_LOGI(TAG, "U_L_DATA_CONT_REQ: index=%d", index);
            return true;
        }

        // U_L_DataEnd: 0x47-0x7F (01llllll where l = length)
        // Pattern: 01llllll where l = length (7-63)
        if ((byte & 0xC0) == 0x40 && byte >= 0x47 && byte <= 0x7F) {
            if (!g_impl->_receiving) {
                ESP_LOGE(TAG, "U_L_DATA_END_REQ without START");
                return false;
            }
            g_impl->_expect_data = true;
            g_impl->_last_was_end = true;
            uint8_t length = byte & 0x3F;
            ESP_LOGI(TAG, "U_L_DATA_END_REQ: length=%d (last byte follows)", length);
            return true;
        }

        // Busy mode (TPUART)
        if (byte == U_TPUART2_SET_BUSY_REQ) {
            ESP_LOGI(TAG, "TPUART: Setting busy mode");
            // Note: Busy mode handling moved to ISR implementation
            return true;
        }
        if (byte == U_TPUART2_QUIT_BUSY_REQ) {
            ESP_LOGI(TAG, "TPUART: Quitting busy mode");
            // Note: Busy mode handling moved to ISR implementation
            return true;
        }

        if (byte == U_BUSMON_REQ) {
            g_impl->_monitoring = true;
            // Note: New implementation might handle promiscuous mode differently
            ESP_LOGI(TAG, "Entering bus monitor mode");
            return true;
        }
        if (byte == U_STOP_MODE_REQ) {
            g_impl->_stopped = true;
            ESP_LOGI(TAG, "TPUART: Entering stop mode");
            // Send stop mode indication
            g_impl->_rx_queue.push_back(U_STOP_MODE_IND);
            return true;
        }
        if (byte == U_EXIT_STOP_MODE_REQ) {
            g_impl->_stopped = false;
            ESP_LOGI(TAG, "TPUART: Exiting stop mode");
            // Send state indication to confirm exit from stop mode
            g_impl->_rx_queue.push_back(U_STATE_IND);
            return true;
        }

        // Acknowledge request (NCN5120 behavior: queue ACK for current frame)
        if ((byte & 0xF8) == U_ACK_REQ) {
            // Map U_ACK_REQ flags to ACK byte
            const uint8_t flags = (byte & 0x07);
            uint8_t ack_byte = KNX_TP_BIT_BANG_ACK_BYTE_NONE;
            
            if (flags & U_ACK_REQ_ADDRESSED) {
                // Addressed ACK requested
                if (flags & U_ACK_REQ_BUSY) {
                    ack_byte = (flags & U_ACK_REQ_NACK) ? 
                        KNX_TP_BIT_BANG_ACK_BYTE_NACK_BUSY : KNX_TP_BIT_BANG_ACK_BYTE_BUSY;
                } else if (flags & U_ACK_REQ_NACK) {
                    ack_byte = KNX_TP_BIT_BANG_ACK_BYTE_NACK;
                } else {
                    ack_byte = KNX_TP_BIT_BANG_ACK_BYTE_ACK;
                }
                g_impl->_bitbang.pending_ack_byte = ack_byte;
            }
            return true;
        }

        // Unknown/control bytes: ignore for now
        return true;
    }

    int KnxTpBitBangInterface::read()
    {
        if (!g_impl) return -1;
        if (g_impl->_rx_queue.empty()) {
            // Poll driver just in case
            g_impl->pollIncoming();
            if (g_impl->_rx_queue.empty()) return -1;
        }
        uint8_t v = g_impl->_rx_queue.front();
        g_impl->_rx_queue.pop_front();
        return (int)v;
    }

    bool KnxTpBitBangInterface::overflow()
    {
        return false;
    }

    bool KnxTpBitBangInterface::hasCallback()
    {
        return true;
    }

    bool KnxTpBitBangInterface::hasMoreToProcess() const
    {
        if (!g_impl) return false;
        return g_impl->hasMoreToProcess();
    }

    void KnxTpBitBangInterface::registerCallback(std::function<bool()> callback)
    {
        if (!g_impl) g_impl = new KnxTpBitBangInterfaceImpl();
        g_impl->_cb = callback;
    }

    void KnxTpBitBangInterface::setTaskHandle(TaskHandle_t handle)
    {
        ESP_LOGI(TAG, "Setting task handle %p for ISR notifications for %p", handle, g_impl);
        if (!g_impl) {
            // This means begin() has not been called yet. Just remember the
            // desired handle; begin()/init() will wire it into the driver.
            // We log this to make the sequence explicit.
            ESP_LOGW(TAG, "setTaskHandle called before begin(); handle will be applied on init");
            // Allocate the implementation but do NOT call init() here; that
            // is owned by begin() so we don't create multiple instances.
            g_impl = new KnxTpBitBangInterfaceImpl();
        }

        g_impl->_task_handle = handle;

        // If already initialized, update the driver's task handle too.
        if (g_impl->_initialized) {
            g_impl->_bitbang.xTaskToNotify = handle;
            ESP_LOGI(TAG, "Updated task handle %p for ISR notifications (already initialized)", handle);
        }

        ESP_LOGI(TAG, "Final task handle %p for ISR notifications for impl %p, bitbang.task=%p", handle, g_impl, g_impl->_bitbang.xTaskToNotify);
    }

    TaskHandle_t KnxTpBitBangInterface::getTaskHandle() const
    {
        if (!g_impl) return nullptr;
        return g_impl->_bitbang.xTaskToNotify;
    }

    knx_event_t KnxTpBitBangInterface::getLastEvent()
    {
        // Last event tracking removed in new ISR implementation
        if (!g_impl) return KNX_EVENT_INVALID_STATE;
        // Return a default state since last_event is no longer tracked
        return KNX_EVENT_NONE;
    }

} // namespace Interface
} // namespace TPUart

// C linkage helper to access driver handle for test/monitoring purposes
extern "C" knx_tp_bit_bang_t* knx_tp_bit_bang_get_global_handle() {
    if (TPUart::Interface::g_impl && TPUart::Interface::g_impl->_initialized) {
        return &TPUart::Interface::g_impl->_bitbang;
    }
    return nullptr;
}
