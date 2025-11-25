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

// Add near the top, after TAG / globals
static constexpr uint16_t GA_2_1_3 = 0x1103; // Group address 2/1/3

static TPUart::AcknowledgeType check_ack(unsigned short destination, bool isGroupAddress)
{
    if (isGroupAddress && destination == GA_2_1_3) {
        ESP_LOGI(TAG, "ACK requested for group address 2/1/3 (0x%04x)", destination);
        return TPUart::ACK_Addressed;
    }
    return TPUart::ACK_None;
}

// Telegram logging callback
static void on_frame_received(TPUart::Frame& frame) {
    // Get frame information
    const bool is_extended = frame.isExtended();
    const bool is_group_addr = frame.isGroupAddress();
    const uint16_t src = frame.source();
    const uint16_t dst = frame.destination();
    const uint16_t size = frame.size();
    
    // Format addresses
    char src_str[16], dst_str[16];
    snprintf(src_str, sizeof(src_str), "%u.%u.%u", 
             (src >> 12) & 0xF, (src >> 8) & 0xF, src & 0xFF);
    
    if (is_group_addr) {
        // Group address format: main/middle/sub
        snprintf(dst_str, sizeof(dst_str), "%u/%u/%u", 
                 (dst >> 11) & 0x1F, (dst >> 8) & 0x7, dst & 0xFF);
    } else {
        // Individual address format: area.line.device
        snprintf(dst_str, sizeof(dst_str), "%u.%u.%u", 
                 (dst >> 12) & 0xF, (dst >> 8) & 0xF, dst & 0xFF);
    }
    
    // Log the telegram
    ESP_LOGI(TAG, "RX Telegram: %s frame, src=%s, dst=%s (%s), size=%u bytes", 
             is_extended ? "Extended" : "Standard",
             src_str, dst_str, 
             is_group_addr ? "Group" : "Individual",
             size);
    
    // Log raw frame data in hex for debugging
    const char* data = frame.data();
    if (data && size > 0) {
        ESP_LOGI(TAG, "Frame data: %s", frame.printFrame().c_str());
    }
}

static void knx_stack_task(void* arg) {
    (void)arg;


    // Begin with BCU_TPUART2 mode using our bit-bang interface
    s_dll.begin(TPUart::BCU_TPUART2, &s_if);

    // Register this task handle for direct ISR notifications BEFORE calling begin()
    TaskHandle_t task_handle = xTaskGetCurrentTaskHandle();
    s_if.setTaskHandle(task_handle);
    ESP_LOGI(TAG, "KNX stack task handle: %p", task_handle);

    // Set our own address for auto-ack by device; 1.1.1 example
    s_dll.setOwnAddress(0x1101);

    // Register callback for received telegrams
    s_dll.registerReceivedFrame(on_frame_received);
    
    // Register check acknowledge callback
    s_dll.registerCheckAcknowledge(check_ack);
    
    ESP_LOGI(TAG, "KNX stack initialized with device address 1.1.1, monitoring bus...");

    while (true) {
        // Block waiting for notification from ISR (RX complete or TX complete)
        // with a 10000ms timeout so periodic tasks still run
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
        //ESP_LOGI(TAG, "KNX stack task woke up. task %p", s_if.getTaskHandle());

        // Process DLL immediately on wake (processes RX frames, sends U_ACK_REQ if needed)
        do {
            s_dll.process();
        } while (s_if.hasMoreToProcess());
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

