/**
 * @file sram_startup_entropy.c
 * @brief SRAM state sampling and diagnostic helpers for entropy collection.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include <sram_startup_entropy.h>
#include "uart.h"
/* Captured SRAM bytes are kept in .noinit so startup data is not cleared. */
volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE] __attribute__ ((section(".noinit"), aligned(32)));
/* 0 = not sampled, 1 = startup sample, 2 = runtime sample. */
volatile uint8_t sram_sampled __attribute__ ((section(".data"))) = 0;
/* Startup SRAM sampling. */
/**
 * @brief Capture SRAM state from the configured startup sampling region.
 * @return SRAM sampling status.
 */
int sram_sample_startup_state(void)
{
    /* Preserve the first valid SRAM sample. */
    if (sram_sampled != 0) {
        return SRAM_SAMPLE_ALREADY_DONE;
    }
    if (sram_entropy_buffer == NULL) {
        log_printf("sram_sample_startup_safe: sram_entropy_buffer is NULL\r\n");
        return SRAM_SAMPLE_NULL_BUFFER;
    }
    /* Keep address arithmetic in uintptr_t and reject wraparound. */
    uintptr_t sample_addr = (uintptr_t)SRAM_START_ADDR;
    sample_addr += (uintptr_t)SRAM_SAMPLE_OFFSET;
    if (sample_addr + (uintptr_t)SRAM_ENTROPY_BUFFER_SIZE < sample_addr) {
        log_printf("sram_sample_startup_safe: address overflow detected\r\n");
        return SRAM_SAMPLE_ADDR_OVERFLOW;
    }
    uint8_t *sram_ptr = (uint8_t *)sample_addr;
    /* Copy raw SRAM bytes into the protected capture buffer. */
    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        sram_entropy_buffer[i] = sram_ptr[i];
    }
    sram_sampled = 1;
    return SRAM_SAMPLE_SUCCESS;
}
/* Runtime fallback sampling and accessors. */
/**
 * @brief Capture SRAM state from the runtime fallback sampling region.
 * @return SRAM sampling status.
 */
int sram_sample_runtime(void)
{
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
    /* Runtime sampling is tracked separately from early-startup sampling. */
    sram_sampled = 2;
    return SRAM_SAMPLE_SUCCESS;
}
/**
 * @brief Get the captured SRAM entropy buffer.
 * @return Pointer to the captured SRAM bytes.
 */
const uint8_t* sram_get_entropy_buffer(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return sram_entropy_buffer;
}
/**
 * @brief Get the configured SRAM entropy buffer size.
 * @return SRAM entropy buffer size in bytes.
 */
uint32_t sram_get_entropy_size(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return SRAM_ENTROPY_BUFFER_SIZE;
}
/**
 * @brief Return the current SRAM sampling state.
 * @return 0 if not sampled, 1 for startup sampling, or 2 for runtime sampling.
 */
uint8_t sram_is_sampled(void)
{
    /* Check the return status before continuing to the next step. */
    return sram_sampled;
}
/**
 * @brief Copy bytes from the captured SRAM entropy buffer.
 * @param out_buffer Destination buffer.
 * @param out_len Maximum number of bytes to copy.
 * @param offset Offset inside the SRAM entropy buffer.
 * @return Number of bytes copied.
 */
uint32_t sram_get_bytes(uint8_t *out_buffer, uint32_t out_len, uint32_t offset)
{
    if (out_buffer == NULL || sram_sampled == 0) {
        return 0;
    }
    if (offset >= SRAM_ENTROPY_BUFFER_SIZE) {
        return 0;
    }
    /* Clamp the requested copy to the captured buffer boundary. */
    uint32_t available = SRAM_ENTROPY_BUFFER_SIZE - offset;
    uint32_t to_copy = (out_len <= available) ? out_len : available;
    for (uint32_t i = 0; i < to_copy; i++) {
        out_buffer[i] = sram_entropy_buffer[offset + i];
    }
    return to_copy;
}
/* Diagnostic statistics and output. */
/**
 * @brief Collect basic byte and bit statistics from the captured SRAM buffer.
 * @param zero_bytes Receives the number of 0x00 bytes when non-NULL.
 * @param ff_bytes Receives the number of 0xFF bytes when non-NULL.
 * @param bit_count Receives the total number of set bits when non-NULL.
 */
void sram_get_statistics(uint32_t *zero_bytes, uint32_t *ff_bytes, uint32_t *bit_count)
{
    if (zero_bytes) *zero_bytes = 0;
    if (ff_bytes) *ff_bytes = 0;
    if (bit_count) *bit_count = 0;
    if (sram_sampled == 0) return;
    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        /* Count obvious byte patterns and the total number of set bits. */
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
 * @brief Print the captured SRAM bytes for diagnostics.
 * @param max_bytes Maximum number of bytes to print, or 0 for the full buffer.
 */
void sram_print_buffer(uint32_t max_bytes)
{
    /* Keep lengths within the allocated buffer capacity. */
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
