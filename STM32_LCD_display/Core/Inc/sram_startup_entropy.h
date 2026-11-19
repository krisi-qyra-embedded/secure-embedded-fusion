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
/* SRAM sampling window used by the startup entropy path. */
#define SRAM_START_ADDR     0x20000000
#define SRAM_SIZE           (128 * 1024)
#define SRAM_SAMPLE_OFFSET  (32 * 1024)
#define SRAM_SAMPLE_SIZE    (4 * 1024)
#define SRAM_ENTROPY_BUFFER_SIZE  256
/* SRAM sampling status values. */
#define SRAM_SAMPLE_SUCCESS           0
#define SRAM_SAMPLE_ALREADY_DONE      1
#define SRAM_SAMPLE_NULL_BUFFER     -502
#define SRAM_SAMPLE_ADDR_OVERFLOW   -503
/* Startup buffer and cold-boot sampling flag are kept in .data. */
extern volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE];
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
 * @brief Calculate basic statistics for the SRAM entropy buffer.
 * @param zero_bytes Number of zero-valued bytes.
 * @param ff_bytes Number of 0xFF-valued bytes.
 * @param bit_count Number of set bits.
 */
void sram_get_statistics(uint32_t *zero_bytes, uint32_t *ff_bytes, uint32_t *bit_count);
#endif
