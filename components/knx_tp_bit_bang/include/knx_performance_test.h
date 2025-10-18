/**
 * @file knx_performance_test.h
 * @brief Performance testing functions for KNX TP bit-bang optimizations
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Run comprehensive performance test of the optimized KNX bit-bang implementation
 * 
 * This function tests and reports on:
 * - Initialization performance
 * - GPIO operation speed
 * - ISR execution metrics
 * - Memory usage optimization
 */
void knx_performance_test(void);

#ifdef __cplusplus
}
#endif