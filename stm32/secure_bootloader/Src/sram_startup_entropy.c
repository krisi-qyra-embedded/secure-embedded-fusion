/* Author: i fiksuari pas prog | Documentation date: 14 May 2026 */
#include <sram_startup_entropy.h>
#include "uart.h"  /* log_printf() */


/* sram_startup_entropy.c - SRAM Startup State Entropy Collection
 *
 * Collects randomness from uninitialized SRAM at boot time.
 *
 * Theory:
 * - SRAM cells power up in unpredictable states (0 or 1)
 * - State depends on manufacturing variations, temperature, voltage
 * - Must be read BEFORE any initialization (.bss clearing, stack setup)
 *
 * Hardware: STM32F411 (128KB SRAM at 0x20000000)
 *
 * CRITICAL: This must be called VERY EARLY in startup, before:
 * - .bss section is cleared
 * - Stack is initialized
 * - Any C runtime initialization
 *
 * Typical integration: Call from Reset_Handler before SystemInit()
 */

/* ========== Configuration ========== */

/* STM32F411 SRAM layout:
   - Start: 0x20000000
   - Size: 128KB (0x20000)
   - Safe region to read: Middle section to avoid stack/heap collisions
*/

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



/* Handles the sram sample startup state operation. */
int sram_sample_startup_state(void)
{
    /* Prevent re-sampling (only valid once after power-on) */
    if (sram_sampled != 0) {
        return SRAM_SAMPLE_ALREADY_DONE;
    }

    /* Defensive: ensure destination buffer exists */
    if (sram_entropy_buffer == NULL) {
        log_printf("sram_sample_startup_safe: sram_entropy_buffer is NULL\r\n");
        return SRAM_SAMPLE_NULL_BUFFER;
    }

    /* Compute sample address safely */
    uintptr_t sample_addr = (uintptr_t)SRAM_START_ADDR;
    sample_addr += (uintptr_t)SRAM_SAMPLE_OFFSET;

    /* Check for overflow for sample_addr + size */
    if (sample_addr + (uintptr_t)SRAM_ENTROPY_BUFFER_SIZE < sample_addr) {
        log_printf("sram_sample_startup_safe: address overflow detected\r\n");
        return SRAM_SAMPLE_ADDR_OVERFLOW;
    }

    uint8_t *sram_ptr = (uint8_t *)sample_addr;

    /* Copy uninitialized SRAM bytes to our buffer (byte-by-byte) */
    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        sram_entropy_buffer[i] = sram_ptr[i];
    }

    /* Mark as sampled (startup) */
    sram_sampled = 1;

    return SRAM_SAMPLE_SUCCESS;
}

/* ========== Safe C-Runtime Access Functions ========== */

/*
 * sram_sample_runtime_safe
 *
 * Runtime-safe version of sram_sample_runtime() that returns explicit
 * status codes instead of being void. Does basic defensive checks.
 */
int sram_sample_runtime(void)
{
    /* If early sampling was done, don't overwrite it */
    if (sram_sampled != 0) {
        return SRAM_SAMPLE_ALREADY_DONE;
    }

    /* Defensive: ensure destination buffer exists */
    if (sram_entropy_buffer == NULL) {
        log_printf("sram_sample_runtime_safe: sram_entropy_buffer is NULL\r\n");
        return SRAM_SAMPLE_NULL_BUFFER;
    }

    /* Compute sample address (keep arithmetic in uintptr_t to avoid overflow) */
    uintptr_t sample_addr = (uintptr_t)SRAM_START_ADDR;
    sample_addr += (uintptr_t)SRAM_SAMPLE_OFFSET;
    sample_addr += (uintptr_t)(8 * 1024U); /* original + (8 * 1024) */

    /* Check for overflow of the sample address + size */
    if (sample_addr + (uintptr_t)SRAM_ENTROPY_BUFFER_SIZE < sample_addr) {
        log_printf("sram_sample_runtime_safe: address overflow detected\r\n");
        return SRAM_SAMPLE_ADDR_OVERFLOW;
    }

    uint8_t *sram_ptr = (uint8_t *)sample_addr;

    /* Copy bytes */
    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        sram_entropy_buffer[i] = sram_ptr[i];
    }

    sram_sampled = 2;  /* Mark as runtime-sampled (lower quality) */

    return SRAM_SAMPLE_SUCCESS;
}

/**
 * Get pointer to SRAM entropy buffer
 * Returns: Pointer to buffer containing SRAM startup state
 */
const uint8_t* sram_get_entropy_buffer(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return sram_entropy_buffer;
}

/**
 * Get size of SRAM entropy buffer
 * Returns: Buffer size in bytes
 */
uint32_t sram_get_entropy_size(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return SRAM_ENTROPY_BUFFER_SIZE;
}

/**
 * Check if SRAM has been sampled
 * Returns: 0 = not sampled, 1 = early boot sample, 2 = runtime sample
 */
uint8_t sram_is_sampled(void)
{
    /* Check the return status before continuing to the next step. */
    return sram_sampled;
}

/**
 * Get raw SRAM bytes from specified offset in sampled buffer
 *
 * out_buffer: Output buffer
 * out_len: Number of bytes to copy
 * offset: Offset in SRAM entropy buffer
 *
 * Returns: Number of bytes copied
 */

uint32_t sram_get_bytes(uint8_t *out_buffer, uint32_t out_len, uint32_t offset)
{
    if (out_buffer == NULL || sram_sampled == 0) {
        return 0;
    }

    /* Bounds check */
    if (offset >= SRAM_ENTROPY_BUFFER_SIZE) {
        return 0;
    }

    /* Adjust length if it exceeds buffer */
    uint32_t available = SRAM_ENTROPY_BUFFER_SIZE - offset;
    uint32_t to_copy = (out_len <= available) ? out_len : available;

    /* Copy bytes */
    for (uint32_t i = 0; i < to_copy; i++) {
        out_buffer[i] = sram_entropy_buffer[offset + i];
    }

    return to_copy;
}

/**
 * Calculate basic statistics on SRAM startup state
 *
 * Useful for debugging and quality assessment:
 * - Zero bytes: Number of 0x00 bytes
 * - FF bytes: Number of 0xFF bytes
 * - Bit count: Total number of '1' bits
 *
 * Good SRAM entropy shows:
 * - Roughly equal mix of 0x00 and 0xFF
 * - Bit count near 50% (random distribution)
 * - Few repeated patterns
 */

void sram_get_statistics(uint32_t *zero_bytes, uint32_t *ff_bytes, uint32_t *bit_count)
{
    if (zero_bytes) *zero_bytes = 0;
    if (ff_bytes) *ff_bytes = 0;
    if (bit_count) *bit_count = 0;

    if (sram_sampled == 0) return;

    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        uint8_t byte = sram_entropy_buffer[i];

        /* Count specific patterns */
        if (byte == 0x00 && zero_bytes) (*zero_bytes)++;
        if (byte == 0xFF && ff_bytes) (*ff_bytes)++;

        /* Count set bits */
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
void sram_print_buffer(uint32_t max_bytes)
{
    extern int printf(const char *fmt, ...);  /* Assume printf available */

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


