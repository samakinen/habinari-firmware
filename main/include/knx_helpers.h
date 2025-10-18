/* KNX helper functions header
 * Moved out from main.c for better organization
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "knx_tp_bit_bang.h"

void byte_to_binary(uint8_t b, char out[9]);

const char* knx_priority_str(uint8_t ctrl1);
const char* knx_apci_str(uint8_t apci4);
void log_knx_tp1_telegram(const uint8_t* buf, size_t len);
size_t buffer_to_binary_string(const uint8_t *buf, size_t len, char *out, size_t out_size);
void print_data(knx_tp_bit_bang_t *knx_bit_bang);
