/* Author: i fiksuari pas prog | Documentation date: 14 May 2026 */
#ifndef __SRAM_STARTUP_ENTROPY_H__
#define __SRAM_STARTUP_ENTROPY_H__

#include "stm32f4xx.h"
#include "uart.h"  /* log_printf() */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define SRAM_START_ADDR     0x20000000
#define SRAM_SIZE           (128 * 1024)  /* 128KB */
/* Sample from middle of SRAM to avoid stack/heap */
#define SRAM_SAMPLE_OFFSET  (32 * 1024)   /* Start at 32KB offset */
#define SRAM_SAMPLE_SIZE    (4 * 1024)    /* Read 4KB chunk */

/* Buffer to store SRAM startup state (must be in .data, not .bss!) */
#define SRAM_ENTROPY_BUFFER_SIZE  256  /* Store 256 bytes of raw SRAM state */
/* SRAM runtime sample return macros */
#define SRAM_SAMPLE_SUCCESS           0
#define SRAM_SAMPLE_ALREADY_DONE      1
#define SRAM_SAMPLE_NULL_BUFFER     -502
#define SRAM_SAMPLE_ADDR_OVERFLOW   -503
/* Use __attribute__((section(\".data\"))) to ensure buffer is in .data section
   and gets properly initialized (not zero'd like .bss) */

/* Use __attribute__((section(\".data\"))) to ensure buffer is in .data section
   and gets properly initialized (not zero'd like .bss) */
extern volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE];/* Use __attribute__((section(".data"))) to ensure flag is initialized
   by the reset handler on cold boot */
extern volatile uint8_t sram_sampled __attribute__ ((section(".data")));

int sram_sample_startup_state(void);
int sram_sample_runtime(void);
void sram_get_statistics(uint32_t *zero_bytes, uint32_t *ff_bytes, uint32_t *bit_count);


#endif
