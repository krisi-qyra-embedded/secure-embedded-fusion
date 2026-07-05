/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include <sram_startup_entropy.h>
#include "uart.h"  /* log_printf() */


/* ========== Configuration ========== */

/* ========== Early Boot SRAM Sampling ========== */

/**
 * Sample uninitialized SRAM state.
 *
 * MUST be called from startup assembly BEFORE C runtime init!
 *
 * Safe calling location: Reset_Handler, before:
 * - SystemInit()
 * - __libc_init_array()
 * - main()
 *
 * Assembly integration example:
 *
 *   Reset_Handler:
 *     bl  sram_sample_startup_state    ; Call BEFORE anything else
 *     bl  SystemInit
 *     bl  __libc_init_array
 *     bl  main
 */

volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE] __attribute__ ((section(".noinit"), aligned(32)));
volatile uint8_t sram_sampled __attribute__ ((section(".data"))) = 0;

/* Captures bytes from the configured SRAM sampling region. */
int sram_sample_startup_state(void)
{
    /* Reject a repeated sample or address wrap before copying SRAM. */
    if (sram_sampled != 0) {
        return SRAM_SAMPLE_ALREADY_DONE;
    }

    if (sram_entropy_buffer == NULL) {
        log_printf("sram_sample_startup_safe: sram_entropy_buffer is NULL\r\n");
        return SRAM_SAMPLE_NULL_BUFFER;
    }

    uintptr_t sample_addr = (uintptr_t)SRAM_START_ADDR;
    sample_addr += (uintptr_t)SRAM_SAMPLE_OFFSET;

    if (sample_addr + (uintptr_t)SRAM_ENTROPY_BUFFER_SIZE < sample_addr) {
        log_printf("sram_sample_startup_safe: address overflow detected\r\n");
        return SRAM_SAMPLE_ADDR_OVERFLOW;
    }

    uint8_t *sram_ptr = (uint8_t *)sample_addr;

    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        sram_entropy_buffer[i] = sram_ptr[i];
    }

    sram_sampled = 1;

    return SRAM_SAMPLE_SUCCESS;
}

/* ========== Safe C-Runtime Access Functions ========== */

/* Captures bytes from the runtime SRAM sampling region. */
int sram_sample_runtime(void)
{
    /* Offset the region by 8 KiB and check address overflow before reading. */
    if (sram_sampled != 0) {
        return SRAM_SAMPLE_ALREADY_DONE;
    }

    if (sram_entropy_buffer == NULL) {
        log_printf("sram_sample_runtime_safe: sram_entropy_buffer is NULL\r\n");
        return SRAM_SAMPLE_NULL_BUFFER;
    }

    uintptr_t sample_addr = (uintptr_t)SRAM_START_ADDR;
    sample_addr += (uintptr_t)SRAM_SAMPLE_OFFSET;
    sample_addr += (uintptr_t)(8 * 1024U);  

    if (sample_addr + (uintptr_t)SRAM_ENTROPY_BUFFER_SIZE < sample_addr) {
        log_printf("sram_sample_runtime_safe: address overflow detected\r\n");
        return SRAM_SAMPLE_ADDR_OVERFLOW;
    }

    uint8_t *sram_ptr = (uint8_t *)sample_addr;

    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        sram_entropy_buffer[i] = sram_ptr[i];
    }

    sram_sampled = 2;   

    return SRAM_SAMPLE_SUCCESS;
}

/* Returns the retained SRAM sample buffer. */
const uint8_t* sram_get_entropy_buffer(void)
{
    /* The buffer resides in .noinit rather than cleared startup storage. */
    return sram_entropy_buffer;
}

/* Returns the SRAM sample buffer capacity. */
uint32_t sram_get_entropy_size(void)
{
    /* Report the configured byte count used by the sampling routines. */
    return SRAM_ENTROPY_BUFFER_SIZE;
}

/**
 * Check if SRAM has been sampled
 * Returns: 0 = not sampled, 1 = early boot sample, 2 = runtime sample
 */
/* Returns the SRAM sampling state. */
uint8_t sram_is_sampled(void)
{
    /* The state distinguishes an unsampled buffer from startup or runtime samples. */
    return sram_sampled;
}

/* Copies a bounded slice of the SRAM sample. */
uint32_t sram_get_bytes(uint8_t *out_buffer, uint32_t out_len, uint32_t offset)
{
    /* Reject an unsampled buffer or an offset beyond its capacity. */
    if (out_buffer == NULL || sram_sampled == 0) {
        return 0;
    }

    if (offset >= SRAM_ENTROPY_BUFFER_SIZE) {
        return 0;
    }

    uint32_t available = SRAM_ENTROPY_BUFFER_SIZE - offset;
    uint32_t to_copy = (out_len <= available) ? out_len : available;

    for (uint32_t i = 0; i < to_copy; i++) {
        out_buffer[i] = sram_entropy_buffer[offset + i];
    }

    return to_copy;
}

/* Counts erased bytes, zero bytes and set bits in SRAM samples. */
void sram_get_statistics(uint32_t *zero_bytes, uint32_t *ff_bytes, uint32_t *bit_count)
{
    /* Skip unsampled storage and update only requested counters. */
    if (zero_bytes) *zero_bytes = 0;
    if (ff_bytes) *ff_bytes = 0;
    if (bit_count) *bit_count = 0;

    if (sram_sampled == 0) return;

    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        uint8_t byte = sram_entropy_buffer[i];

        if (byte == 0x00 && zero_bytes) (*zero_bytes)++;
        if (byte == 0xFF && ff_bytes) (*ff_bytes)++;

        if (bit_count) {
            uint8_t b = byte;
            while (b) {
                *bit_count += (b & 1);
                b >>= 1;
            }
        }
    }
}

/**
 * Print SRAM entropy buffer (for debugging)
 *
 * max_bytes: Maximum number of bytes to print (0 = all)
 */
/* Logs a selected prefix of the SRAM sample. */
void sram_print_buffer(uint32_t max_bytes)
{
    /* Clamp the requested length to the captured buffer size. */
    extern int printf(const char *fmt, ...);   

    if (sram_sampled == 0) {
        log_printf("SRAM not yet sampled!\r\n");
        return;
    }

    uint32_t count = (max_bytes == 0 || max_bytes > SRAM_ENTROPY_BUFFER_SIZE)
                     ? SRAM_ENTROPY_BUFFER_SIZE : max_bytes;

    log_printf("SRAM Startup State (first %lu bytes):\r\n", count);

    for (uint32_t i = 0; i < count; i++) {
        if (i % 16 == 0) {
            log_printf("\r\n%04lX: ", i);
        }
        log_printf("%02X ", sram_entropy_buffer[i]);
    }
    log_printf("\r\n");
}

/* ========== IMPORTANT INTEGRATION NOTES ========== */

/*
 * For BEST entropy quality, integrate into startup assembly:
 *
 * File: startup_stm32f411xe.s (or similar)
 *
 * Reset_Handler:
 *     ldr   r0, =_estack
 *     mov   sp, r0          ; Set stack pointer
 *
 *     ; === ADD THIS BEFORE SystemInit ===
 *     bl    sram_sample_startup_state
 *     ; ==================================
 *
 *     bl    SystemInit
 *     bl    __libc_init_array
 *     bl    main
 *     bx    lr
 *
 *
 * Alternative (if startup.s modification not possible):
 * Call sram_sample_runtime() as FIRST LINE in main():
 *
 * int main(void)
 * {
 *     sram_sample_runtime();  // << Call immediately
 *
 *     SystemCoreClockUpdate();
 *     // ... rest of init
 * }
 *
 * Note: Runtime sampling has lower quality than early boot sampling,
 * but still provides some entropy.
 */

