#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "knx_tp_bit_bang_interface.h"
#include "TPUart/DataLinkLayer.h"
#include "TPUart/Frame.h"
#define MASK_VERSION 0x07B0
#include "TPUart/Types.h"
#include "cemi_frame.h"
#include "knx_types.h"

static const char* TAG = "knx_stack_test";

static TPUart::Interface::KnxTpBitBangInterface s_if;
static TPUart::DataLinkLayer s_dll;
static constexpr uint16_t GA_2_1_0 = 0x1100; // Group address 2/1/0

static void log_message(const char* msg, bool error) {
    if (error) {
        ESP_LOGE(TAG, "%s", msg);
    } else {
        ESP_LOGI(TAG, "%s", msg);
    }
}

// Simple probe state to correlate TX ack for a destination
struct ProbeState {
    volatile bool pending = false;
    volatile bool acked = false;
    volatile uint16_t dest = 0;
};
static ProbeState s_probe;

static void on_frame_received(TPUart::Frame& frame) {
    // Distinguish between frames we transmitted vs. frames received from the bus
    const bool was_tx = frame.isTransmitted();
    std::string info = frame.printFrame();
    ESP_LOGI(TAG, "%s %s", was_tx ? "TX" : "RX", info.c_str());

    // Highlight when we receive our subscribed GA (2/1/0)
    if (!was_tx && frame.isGroupAddress() && frame.destination() == GA_2_1_0) {
        ESP_LOGI(TAG, "RX subscribed GA 2/1/0; ACK should be sent by DLL");
    }

    // Correlate TX ack for probe destination
    if (was_tx && s_probe.pending) {
        if (frame.destination() == s_probe.dest) {
            s_probe.acked = frame.isAck() && !frame.isNack();
            s_probe.pending = false;
        }
    }

    // For transmitted frames, report the acknowledge outcome using abstract flags
    if (was_tx) {
        if (frame.isAck()) {
            if (frame.isNack() && frame.isBusy()) {
                ESP_LOGW(TAG, "TX confirmed: NACK+BUSY received");
            } else if (frame.isNack()) {
                ESP_LOGW(TAG, "TX confirmed: NACK received");
            } else if (frame.isBusy()) {
                ESP_LOGW(TAG, "TX confirmed: ACK BUSY received");
            } else {
                ESP_LOGI(TAG, "TX confirmed: ACK received");
            }
        } else {
            // No ACK received within timeout window
            ESP_LOGE(TAG, "TX confirmed: No ACK received (timeout)");
        }
    }
}

// Build and send a valid standard TP frame via cEMI builder
static bool send_tp_frame_individual(uint16_t src, uint16_t dst, ApduType apci, const uint8_t* payload, uint8_t payload_len) {
    const uint8_t apdu_octets = (uint8_t)(1 + payload_len); // 1 byte APCI/TPCI + payload
    CemiFrame cemi(apdu_octets);
    cemi.messageCode(L_data_req);
    cemi.frameType(StandardFrame);
    cemi.repetition(RepetitionAllowed);
    cemi.priority(NormalPriority);
    cemi.ack(AckRequested);
    cemi.confirm(ConfirmNoError);
    cemi.addressType(IndividualAddress);
    cemi.hopCount(6);
    cemi.sourceAddress(src);
    cemi.destinationAddress(dst);
    cemi.tpdu().type(DataInduvidual);
    cemi.apdu().type(apci);
    if (payload_len && payload) {
        uint8_t* p = cemi.apdu().data();
        for (uint8_t i = 0; i < payload_len; ++i) p[i] = payload[i];
    }

    const uint16_t len = cemi.telegramLengthtTP();
    std::vector<uint8_t> tp(len);
    cemi.fillTelegramTP(tp.data());

    // Create a Frame object (copies the data) and push to transmit queue
    TPUart::Frame *txframe = new TPUart::Frame((char*)tp.data(), len);
    if (!s_dll.pushTransmitQueue(txframe)) {
        delete txframe;
        ESP_LOGW(TAG, "TX queue full, dropped frame to %u.%u.%u", (dst >> 12) & 0xF, (dst >> 8) & 0xF, dst & 0xFF);
        return false;
    }
    return true;
}

// Probe for presence of an individual address by sending DeviceDescriptorRead(0)
static bool probe_device_fffa(uint32_t timeout_ms = 300) {
    const uint16_t dst = 0xFFFA; // 15.15.250
    const uint16_t src = 0x1101; // must match s_dll.setOwnAddress()
    const uint8_t dd_index = 0x00; // Device Descriptor Type 0

    // Arm probe state
    s_probe.dest = dst;
    s_probe.acked = false;
    s_probe.pending = true;

    // Send DeviceDescriptorRead (point-to-point)
    if (!send_tp_frame_individual(src, dst, DeviceDescriptorRead, &dd_index, 1)) {
        s_probe.pending = false;
        return false;
    }

    // Wait for TX result to surface in callback
    const unsigned long start = millis();
    while (s_probe.pending && (millis() - start) < timeout_ms) {
        s_dll.process();
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (s_probe.pending) {
        // timeout waiting for TX completion
        s_probe.pending = false;
        return false;
    }

    return s_probe.acked;
}

static bool send_test_telegram_to_ga_2_1_0() {
    // Build a small standard frame with 1-byte APDU (Group Write) using cEMI would be better;
    // keep existing demo for now.
    const size_t frame_size = 9;
    char buf[frame_size];
    
    buf[0] = (char)0x9C; // control 1
    buf[1] = (char)0xFF; // source high (standard frame layout)
    buf[2] = (char)0xFA; // source low
    buf[3] = (char)0x11; // dest high (2/1/0)
    buf[4] = (char)0x11; // dest low
    buf[5] = (char)0xE1; // length + flags (group)
    buf[6] = (char)0x00; // APDU byte
    // CRC8 over 0..6
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < frame_size - 1; ++i) crc ^= (uint8_t)buf[i];
    buf[7] = (char)crc; // checksum
    buf[8] = 0x00; // padding (ignored)

    TPUart::Frame *txframe = new TPUart::Frame(buf, (unsigned short)8); // use first 8 bytes
    if (!s_dll.pushTransmitQueue(txframe)) {
        delete txframe;
        ESP_LOGW(TAG, "TX queue full, dropped test frame");
        return false;
    }
    ESP_LOGI(TAG, "Queued test frame to GA 2/1/0 (demo)");
    return true;
}

static void knx_stack_task(void* arg) {
    (void)arg;

    // Register this task handle for direct ISR notifications BEFORE calling begin()
    TaskHandle_t task_handle = xTaskGetCurrentTaskHandle();
    ESP_LOGI(TAG, "KNX stack task handle: %p", task_handle);
    s_if.setTaskHandle(task_handle);

    // Begin with NCN5120 mode using our bit-bang interface
    s_dll.begin(TPUart::BCU_NCN5120, &s_if);

    // Set our own address for auto-ack by device; 1.1.1 example
    s_dll.setOwnAddress(0x1101);

    // Register simple logging callbacks
    s_dll.registerMessage(log_message);
    s_dll.registerReceivedFrame(on_frame_received);
    // Acknowledge group frames addressed to GA 2/1/0 (0x1100). Others: no ACK from us.
    s_dll.registerCheckAcknowledge([](unsigned short dest, bool isGroup){
        if (isGroup && dest == GA_2_1_0) return TPUart::ACK_Addressed;
        return TPUart::ACK_None;
    });

    ESP_LOGI(TAG, "KNX TPUART DLL started");

    // One-shot: probe device 15.15.250 after link up delay
    vTaskDelay(pdMS_TO_TICKS(500));
    bool present = probe_device_fffa();
    ESP_LOGI(TAG, "Probe 15.15.250: %s", present ? "PRESENT (ACK)" : "NO RESPONSE");

    // Main processing loop: block on ISR notification for immediate RX processing
    unsigned long last_probe = millis();
    s_if.setTaskHandle(task_handle);
    while (true) {
        // Block waiting for notification from ISR (RX complete or TX complete)
        // with a 2000ms timeout so periodic tasks still run
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "KNX stack task woke up. Last event: %lu task %p", s_if.getLastEvent(), s_if.getTaskHandle());

        // Process DLL immediately on wake (processes RX frames, sends U_ACK_REQ if needed)
        s_dll.process();

        // Periodic re-probe every 60s
        unsigned long now = millis();
        if (now - last_probe >= 60000UL) {
            last_probe = now;
            bool ok = probe_device_fffa();
            ESP_LOGI(TAG, "Probe 15.15.250: %s", ok ? "PRESENT (ACK)" : "NO RESPONSE");
        }
    }
}

extern "C" void knx_stack_test_start(void) {
    static bool started = false;
    if (started) return;
    started = true;
    TaskHandle_t task_handle = nullptr;
    xTaskCreate(knx_stack_task, "knx_stack", 4096, nullptr, 8, &task_handle);
    //s_if.setTaskHandle(task_handle);
}
