/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file sram_startup_entropy.h
 * @brief SRAM startup sampling and entropy buffer declarations.
 */
#ifndef __SRAM_STARTUP_ENTROPY_H__
#define __SRAM_STARTUP_ENTROPY_H__
#include "stm32f4xx.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* SRAM sampling window used during startup/runtime entropy collection. */
#define SRAM_START_ADDR     0x20000000
#define SRAM_SIZE           (128 * 1024)
#define SRAM_SAMPLE_OFFSET  (32 * 1024)
#define SRAM_SAMPLE_SIZE    (4 * 1024)
/* Raw SRAM entropy buffer and sampling status values. */
#define SRAM_ENTROPY_BUFFER_SIZE  256
#define SRAM_SAMPLE_SUCCESS           0
#define SRAM_SAMPLE_ALREADY_DONE      1
#define SRAM_SAMPLE_NULL_BUFFER     -502
#define SRAM_SAMPLE_ADDR_OVERFLOW   -503
/* Startup SRAM sample buffer and completion flag. */
extern volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE];
/* Keep the sampled flag in .data so startup code does not place it in .bss. */
extern volatile uint8_t sram_sampled __attribute__ ((section(".data")));
/**
 * @brief Capture the configured SRAM startup sample.
 * @return SRAM sampling status.
 */
int sram_sample_startup_state(void);
/**
 * @brief Capture a runtime SRAM sample.
 * @return SRAM sampling status.
 */
int sram_sample_runtime(void);
/**
 * @brief Calculate simple byte/bit statistics for the SRAM sample.
 * @param zero_bytes Zero-byte count output.
 * @param ff_bytes 0xFF-byte count output.
 * @param bit_count Set-bit count output.
 */
void sram_get_statistics(uint32_t *zero_bytes, uint32_t *ff_bytes, uint32_t *bit_count);
#endif
