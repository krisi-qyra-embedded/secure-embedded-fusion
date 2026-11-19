/**
 * @file sram_startup_entropy.c
 * @brief SRAM startup sampling and entropy buffer helpers.
 */
#include <sram_startup_entropy.h>
#include "uart.h"
/* .noinit keeps this buffer outside normal startup clearing. */
volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE] __attribute__ ((section(".noinit"), aligned(32)));
/* Tracks whether startup or runtime sampling already ran. */
volatile uint8_t sram_sampled __attribute__ ((section(".data"))) = 0;
/**
 * @brief Sample the configured SRAM region during early startup.
 * @return SRAM sampling status.
 */
int sram_sample_startup_state(void)
{
    /* Startup SRAM should only be captured once. */
    if (sram_sampled != 0) {
        return SRAM_SAMPLE_ALREADY_DONE;
    }
    if (sram_entropy_buffer == NULL) {
        log_printf("sram_sample_startup_safe: sram_entropy_buffer is NULL\r\n");
        return SRAM_SAMPLE_NULL_BUFFER;
    }
    /* Build the configured sample address with uintptr_t arithmetic. */
    uintptr_t sample_addr = (uintptr_t)SRAM_START_ADDR;
    sample_addr += (uintptr_t)SRAM_SAMPLE_OFFSET;
    /* Reject address wrap before reading the region. */
    if (sample_addr + (uintptr_t)SRAM_ENTROPY_BUFFER_SIZE < sample_addr) {
        log_printf("sram_sample_startup_safe: address overflow detected\r\n");
        return SRAM_SAMPLE_ADDR_OVERFLOW;
    }
    /* Read the selected SRAM region byte by byte. */
    uint8_t *sram_ptr = (uint8_t *)sample_addr;
    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        sram_entropy_buffer[i] = sram_ptr[i];
    }
    /* State 1 marks an early-startup sample. */
    sram_sampled = 1;
    return SRAM_SAMPLE_SUCCESS;
}
/**
 * @brief Sample the fallback SRAM region at runtime.
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
    /* Runtime fallback uses a later SRAM region. */
    sample_addr += (uintptr_t)(8 * 1024U);
    if (sample_addr + (uintptr_t)SRAM_ENTROPY_BUFFER_SIZE < sample_addr) {
        log_printf("sram_sample_runtime_safe: address overflow detected\r\n");
        return SRAM_SAMPLE_ADDR_OVERFLOW;
    }
    uint8_t *sram_ptr = (uint8_t *)sample_addr;
    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        sram_entropy_buffer[i] = sram_ptr[i];
    }
    /* State 2 marks the lower-quality runtime fallback sample. */
    sram_sampled = 2;
    return SRAM_SAMPLE_SUCCESS;
}
/**
 * @brief Get the sampled SRAM entropy buffer.
 * @return Pointer to the entropy buffer.
 */
const uint8_t* sram_get_entropy_buffer(void)
{
    return sram_entropy_buffer;
}
/**
 * @brief Get the SRAM entropy buffer size.
 * @return Buffer size in bytes.
 */
uint32_t sram_get_entropy_size(void)
{
    return SRAM_ENTROPY_BUFFER_SIZE;
}
/**
 * @brief Get the current SRAM sampling state.
 * @return 0 if not sampled, 1 for startup sample, 2 for runtime sample.
 */
uint8_t sram_is_sampled(void)
{
    return sram_sampled;
}
/**
 * @brief Copy bytes from the sampled SRAM buffer.
 * @param out_buffer Destination buffer.
 * @param out_len Requested byte count.
 * @param offset Offset inside the entropy buffer.
 * @return Number of bytes copied.
 */
uint32_t sram_get_bytes(uint8_t *out_buffer, uint32_t out_len, uint32_t offset)
{
    /* Do not expose data before a successful sample exists. */
    if (out_buffer == NULL || sram_sampled == 0) {
        return 0;
    }
    if (offset >= SRAM_ENTROPY_BUFFER_SIZE) {
        return 0;
    }
    /* Clamp the copy to the remaining buffer space. */
    uint32_t available = SRAM_ENTROPY_BUFFER_SIZE - offset;
    uint32_t to_copy = (out_len <= available) ? out_len : available;
    for (uint32_t i = 0; i < to_copy; i++) {
        out_buffer[i] = sram_entropy_buffer[offset + i];
    }
    return to_copy;
}
/**
 * @brief Calculate basic statistics for the sampled SRAM data.
 * @param zero_bytes Number of 0x00 bytes.
 * @param ff_bytes Number of 0xFF bytes.
 * @param bit_count Total number of set bits.
 */
void sram_get_statistics(uint32_t *zero_bytes, uint32_t *ff_bytes, uint32_t *bit_count)
{
    /* Reset only the statistic outputs supplied by the caller. */
    if (zero_bytes) *zero_bytes = 0;
    if (ff_bytes) *ff_bytes = 0;
    if (bit_count) *bit_count = 0;
    if (sram_sampled == 0) return;
    for (uint32_t i = 0; i < SRAM_ENTROPY_BUFFER_SIZE; i++) {
        uint8_t byte = sram_entropy_buffer[i];
        /* Track common byte patterns and total set-bit count. */
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
 * @brief Print sampled SRAM bytes for debug.
 * @param max_bytes Maximum bytes to print, or 0 for the full buffer.
 */
void sram_print_buffer(uint32_t max_bytes)
{
    extern int printf(const char *fmt, ...);
    if (sram_sampled == 0) {
        log_printf("SRAM not yet sampled!\r\n");
        return;
    }
    /* Limit debug output without changing the stored sample. */
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
