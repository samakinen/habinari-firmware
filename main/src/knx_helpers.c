/* KNX helper function implementations moved from main.c */
#include "knx_helpers.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "knx_helpers";

void byte_to_binary(uint8_t b, char out[9])
{
    for (int i = 0; i < 8; i++) {
        out[i] = (b & (1 << (7 - i))) ? '1' : '0';
    }
    out[8] = '\0';
}

const char* knx_priority_str(uint8_t ctrl1)
{
    switch ((ctrl1 >> 2) & 0x3) {
        case 0: return "system";
        case 1: return "urgent";
        case 2: return "normal";
        case 3: return "low";
        default: return "?";
    }
}

const char* knx_apci_str(uint8_t apci4)
{
    switch (apci4) {
        case 0x0: return "Group Read";
        case 0x1: return "Group Response";
        case 0x2: return "Group Write";
        default:  return "APCI Other";
    }
}

void log_knx_tp1_telegram(const uint8_t* buf, size_t len)
{
    if (len < 7) {
        ESP_LOGI(TAG, "KNX TP1: short frame (%u bytes)", (unsigned)len);
        return;
    }
    uint8_t ctrl1 = buf[0];
    uint8_t ctrl2 = buf[1];
    uint16_t src = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t dst = ((uint16_t)buf[4] << 8) | buf[5];
    uint8_t npdu_len = buf[6];
    bool dst_is_group = (ctrl2 & 0x80) != 0;

    char src_str[16], dst_str[16];
    knx_format_individual_address(src, src_str, sizeof(src_str));
    if (dst_is_group) knx_format_group_address(dst, dst_str, sizeof(dst_str));
    else knx_format_individual_address(dst, dst_str, sizeof(dst_str));

    uint8_t calculated_checksum = 0xFF;
    for (size_t i = 0; i < len - 1; ++i) {
        calculated_checksum ^= buf[i];
    }
    uint8_t received_checksum = (len > 0) ? buf[len - 1] : 0;
    bool checksum_valid = (calculated_checksum == received_checksum);

    const uint8_t* apdu = (len > 7) ? &buf[7] : NULL;
    size_t apdu_len = (len > 8) ? (len - 8) : 0;
    uint8_t apci4 = 0xF;
    uint8_t tpci2 = 0;
    uint8_t dpt1 = 0;
    if (apdu && len >= 9) {
        tpci2 = (apdu[0] >> 6) & 0x03;
        apci4 = ((apdu[0] & 0x03) << 2) | ((apdu[1] >> 6) & 0x03);
        dpt1 = (apdu[1] & 0x3F);
    }

    ESP_LOGI(TAG, "KNX TP1: pri=%s, ctrl1=0x%02x ctrl2=0x%02x, src=%s, dst=%s (%s), len=%u",
             knx_priority_str(ctrl1), ctrl1, ctrl2, src_str, dst_str,
             dst_is_group ? "GA" : "IA", (unsigned)npdu_len);

    ESP_LOGI(TAG, "  Checksum: rx=0x%02x calc=0x%02x %s",
             received_checksum, calculated_checksum,
             checksum_valid ? "VALID" : "INVALID");

    if (apdu && len >= 9) {
        ESP_LOGI(TAG, "  TPDU: TPCI=%u, APCI=%s (0x%x)", tpci2, knx_apci_str(apci4), apci4);
        if (apci4 <= 0x2) {
            ESP_LOGI(TAG, "  APDU: %s%s%u", (apci4 == 0x2) ? "write " : ((apci4 == 0x1) ? "response " : "read "),
                     (apdu_len >= 1) ? "DPT1=" : "", (apdu_len >= 1) ? (dpt1 & 0x01) : 0);
        } else {
            char apdu_hex[64];
            apdu_hex[0] = '\0';
            size_t pos = 0;
            for (size_t i = 0; i < apdu_len && pos < sizeof(apdu_hex); ++i) {
                int w = snprintf(apdu_hex + pos, sizeof(apdu_hex) - pos, "%s%02X", (i ? " " : ""), apdu[i]);
                if (w <= 0 || (size_t)w >= (sizeof(apdu_hex) - pos)) { pos = sizeof(apdu_hex) - 1; apdu_hex[pos] = '\0'; break; }
                pos += (size_t)w;
            }
            ESP_LOGI(TAG, "  APDU: %s", apdu_hex);
        }
    }
}

size_t buffer_to_binary_string(const uint8_t *buf, size_t len, char *out, size_t out_size)
{
    if (out_size == 0) return 0;
    size_t pos = 0;
    out[0] = '\0';
    for (size_t i = 0; i < len; ++i) {
        char tmp[9];
        byte_to_binary(buf[i], tmp);
        const char *sep = (i == 0) ? "" : " ";
        int written = snprintf(out + pos, (pos < out_size) ? (out_size - pos) : 0, "%s%s", sep, tmp);
        if (written < 0) break;
        if ((size_t)written >= (out_size - pos)) { pos = out_size - 1; out[pos] = '\0'; break; }
        pos += (size_t)written;
    }
    return pos;
}

void print_data(knx_tp_bit_bang_t *knx_bit_bang)
{
    static char tx_timer_delay_line[512];
    static char tx_timer_duration_line[512];
    static char rx_timer_delay_line[512];
    static char rx_timer_duration_line[512];
    tx_timer_delay_line[0] = '\0';
    tx_timer_duration_line[0] = '\0';
    rx_timer_delay_line[0] = '\0';
    rx_timer_duration_line[0] = '\0';
    size_t pos = 0;
    for (int i = 0; i < TIMER_MARGINES_SIZE && pos < sizeof(tx_timer_delay_line); i++) {
        pos += snprintf(tx_timer_delay_line + pos, sizeof(tx_timer_delay_line) - pos, "%s%" PRId32,
                        (i == 0) ? "" : " ", knx_bit_bang->tx_timer_delay[i]);
    }
    pos = 0;
    for (int i = 0; i < TIMER_MARGINES_SIZE && pos < sizeof(tx_timer_duration_line); i++) {
        pos += snprintf(tx_timer_duration_line + pos, sizeof(tx_timer_duration_line) - pos, "%s%" PRId32,
                        (i == 0) ? "" : " ", knx_bit_bang->tx_timer_durations[i]);
    }
    pos = 0;
    for (int i = 0; i < TIMER_MARGINES_SIZE && pos < sizeof(rx_timer_delay_line); i++) {
        pos += snprintf(rx_timer_delay_line + pos, sizeof(rx_timer_delay_line) - pos, "%s%" PRId32,
                        (i == 0) ? "" : " ", knx_bit_bang->rx_timer_delays[i]);
    }
    pos = 0;
    for (int i = 0; i < TIMER_MARGINES_SIZE && pos < sizeof(rx_timer_duration_line); i++) {
        pos += snprintf(rx_timer_duration_line + pos, sizeof(rx_timer_duration_line) - pos, "%s%" PRId32,
                        (i == 0) ? "" : " ", knx_bit_bang->rx_timer_durations[i]);
    }
    ESP_LOGI(TAG, "KNX Timer delays - TX: %s", tx_timer_delay_line);
    ESP_LOGI(TAG, "KNX Timer durations - TX: %s", tx_timer_duration_line);
    ESP_LOGI(TAG, "KNX Timer delays - RX: %s", rx_timer_delay_line);
    ESP_LOGI(TAG, "KNX Timer durations - RX: %s", rx_timer_duration_line);
}
