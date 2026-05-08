/* Author: i fiksuari pas prog | Documentation date: 8 May 2026 */
#include "device_uid_entropy.h"


/* device_uid_entropy.c - STM32 Device Unique ID Collection
 *
 * Reads the factory-programmed 96-bit Unique Device ID (UID).
 *
 * Properties:
 * - 96 bits (12 bytes) unique per device
 * - Programmed at factory (ST Microelectronics)
 * - Never changes (read-only)
 * - Located at fixed address 0x1FFF7A10 (STM32F411)
 *
 * Use Cases:
 * - Device fingerprinting
 * - Personalization string for DRBG
 * - Device-specific key derivation
 * - Anti-cloning protection
 *
 * NOT suitable as primary entropy source (static/predictable)
 * BUT excellent for adding uniqueness to entropy pool
 */


/**
 * Read Device Unique ID from factory-programmed registers
 *
 * Reads 96-bit UID and stores in internal buffer.
 * Can be called multiple times (idempotent).
 */

uint8_t device_uid_buffer[UID_SIZE_BYTES] = {0};
volatile uint8_t uid_read = 0;
uint32_t uid_hash;

/*
 * device_uid_read_safe
 *
 * Reads the 96-bit unique device ID into device_uid_buffer.
 * Performs basic validation and returns explicit status codes.
 */
int device_uid_read(void)
{
    /* Defensive: ensure buffer exists */
    if (device_uid_buffer == NULL) {
        log_printf("device_uid_read_safe: device_uid_buffer is NULL\n");
        return DEVICE_UID_READ_NULL_BUFFER;
    }

    /* Read three 32-bit words from UID registers */
    uint32_t uid0 = *(volatile uint32_t *)UID_WORD0_ADDR;
    uint32_t uid1 = *(volatile uint32_t *)UID_WORD1_ADDR;
    uint32_t uid2 = *(volatile uint32_t *)UID_WORD2_ADDR;

    /* Defensive: reject all-zero UID (should never happen on real silicon) */
    if ((uid0 == 0U) && (uid1 == 0U) && (uid2 == 0U)) {
        log_printf("device_uid_read_safe: UID registers returned all zeros\n");
        return DEVICE_UID_READ_INVALID_UID;
    }

    /* Store as bytes (little-endian) */
    device_uid_buffer[0]  = (uint8_t)((uid0 >> 0)  & 0xFF);
    device_uid_buffer[1]  = (uint8_t)((uid0 >> 8)  & 0xFF);
    device_uid_buffer[2]  = (uint8_t)((uid0 >> 16) & 0xFF);
    device_uid_buffer[3]  = (uint8_t)((uid0 >> 24) & 0xFF);

    device_uid_buffer[4]  = (uint8_t)((uid1 >> 0)  & 0xFF);
    device_uid_buffer[5]  = (uint8_t)((uid1 >> 8)  & 0xFF);
    device_uid_buffer[6]  = (uint8_t)((uid1 >> 16) & 0xFF);
    device_uid_buffer[7]  = (uint8_t)((uid1 >> 24) & 0xFF);

    device_uid_buffer[8]  = (uint8_t)((uid2 >> 0)  & 0xFF);
    device_uid_buffer[9]  = (uint8_t)((uid2 >> 8)  & 0xFF);
    device_uid_buffer[10] = (uint8_t)((uid2 >> 16) & 0xFF);
    device_uid_buffer[11] = (uint8_t)((uid2 >> 24) & 0xFF);

    uid_read = 1;

    return DEVICE_UID_READ_SUCCESS;
}

/**
 * Get pointer to UID buffer
 * Automatically reads UID if not already done
 *
 * Returns: Pointer to 12-byte UID buffer
 */

const uint8_t* device_uid_get_buffer(void)
{
    /* Keep lengths within the allocated buffer capacity. */
    if (!uid_read) {
        device_uid_read();
    }
    return device_uid_buffer;
}

/**
 * Get UID size in bytes
 * Returns: 12 (96 bits)
 */
uint32_t device_uid_get_size(void)
{
    /* Check the return status before continuing to the next step. */
    return UID_SIZE_BYTES;
}

/**
 * Copy UID to external buffer
 *
 * out_buffer: Destination buffer (must be at least 12 bytes)
 * out_len: Size of output buffer
 *
 * Returns: Number of bytes copied (0-12)
 */

uint32_t device_uid_get_bytes(uint8_t *out_buffer, uint32_t out_len)
{
    /* Check the return status before continuing to the next step. */
    if (out_buffer == NULL || out_len == 0) {
        return 0;
    }

    if (!uid_read) {
        device_uid_read();
    }

    uint32_t to_copy = (out_len < UID_SIZE_BYTES) ? out_len : UID_SIZE_BYTES;

    for (uint32_t i = 0; i < to_copy; i++) {
        out_buffer[i] = device_uid_buffer[i];
    }

    return to_copy;
}

/**
 * Get UID as three 32-bit words (native format)
 *
 * uid0, uid1, uid2: Pointers to receive 32-bit words
 */

void device_uid_get_words(uint32_t *uid0, uint32_t *uid1, uint32_t *uid2)
{
    /* Check the return status before continuing to the next step. */
    if (!uid_read) {
        device_uid_read();
    }

    if (uid0) {
        *uid0 = (device_uid_buffer[3] << 24) |
                (device_uid_buffer[2] << 16) |
                (device_uid_buffer[1] << 8)  |
                (device_uid_buffer[0] << 0);
    }

    if (uid1) {
        *uid1 = (device_uid_buffer[7] << 24) |
                (device_uid_buffer[6] << 16) |
                (device_uid_buffer[5] << 8)  |
                (device_uid_buffer[4] << 0);
    }

    if (uid2) {
        *uid2 = (device_uid_buffer[11] << 24) |
                (device_uid_buffer[10] << 16) |
                (device_uid_buffer[9] << 8)   |
                (device_uid_buffer[8] << 0);
    }
}

/**
 * Compare UID with another UID
 *
 * other_uid: Buffer containing another UID (12 bytes)
 *
 * Returns: 1 if match, 0 if different
 */

uint8_t device_uid_compare(const uint8_t *other_uid)
{
    /* Check the return status before continuing to the next step. */
    if (other_uid == NULL) {
        return 0;
    }

    if (!uid_read) {
        device_uid_read();
    }

    for (uint32_t i = 0; i < UID_SIZE_BYTES; i++) {
        if (device_uid_buffer[i] != other_uid[i]) {
            return 0;
        }
    }

    return 1;
}

/**
 * Calculate simple hash of UID (for compact representation)
 *
 * Uses simple XOR folding to produce 32-bit hash
 *
 * Returns: 32-bit hash value
 */

uint32_t device_uid_hash(void)
{
    if (!uid_read) {
        device_uid_read();
    }

    uint32_t uid0, uid1, uid2;
    device_uid_get_words(&uid0, &uid1, &uid2);

    /* XOR fold to 32 bits */
    return uid0 ^ uid1 ^ uid2;
}


/**
 * Print UID in human-readable format
 *
 * Formats as: XXXXXXXX-XXXXXXXX-XXXXXXXX (hex, dash-separated)
 */

void device_uid_print(void)
{
    extern int printf(const char *fmt, ...);  /* Assume printf available */

    if (!uid_read) {
        device_uid_read();
    }

    uint32_t uid0, uid1, uid2;
    device_uid_get_words(&uid0, &uid1, &uid2);

    log_printf("Device UID: %08lX-%08lX-%08lX\r\n", uid0, uid1, uid2);
}

/**
 * Print UID as byte array (for copying to code)
 */
void device_uid_print_bytes(void)
{
    /* Check the return status before continuing to the next step. */
    extern int printf(const char *fmt, ...);

    if (!uid_read) {
        device_uid_read();
    }

    log_printf("Device UID (bytes): ");
    for (uint32_t i = 0; i < UID_SIZE_BYTES; i++) {
    	log_printf("0x%02X", device_uid_buffer[i]);
        if (i < UID_SIZE_BYTES - 1) {
        	log_printf(", ");
        }
    }
    log_printf("\r\n");
}

/**
 * Check if UID appears valid (not all zeros or all FFs)
 *
 * Returns: 1 if valid, 0 if suspicious
 */
uint8_t device_uid_is_valid(void)
{
    if (!uid_read) {
        device_uid_read();
    }

    uint8_t all_zero = 1;
    uint8_t all_ff = 1;

    for (uint32_t i = 0; i < UID_SIZE_BYTES; i++) {
        if (device_uid_buffer[i] != 0x00) all_zero = 0;
        if (device_uid_buffer[i] != 0xFF) all_ff = 0;
    }

    /* Valid if NOT all zeros and NOT all FFs */
    return (!all_zero && !all_ff) ? 1 : 0;
}

/* ========== Advanced: UID-based Device Fingerprint ========== */

/**
 * Generate device fingerprint by mixing UID with other device parameters
 *
 * Includes:
 * - Unique ID (96 bits)
 * - Flash size (16 bits)
 * - Device ID (16 bits)
 * - Revision ID (16 bits)
 *
 * out_buffer: Output buffer (minimum 16 bytes)
 * out_len: Size of output buffer
 *
 * Returns: Number of bytes written
 */

uint32_t device_generate_fingerprint(uint8_t *out_buffer, uint32_t out_len)
{
    if (out_buffer == NULL || out_len < 16) {
        return 0;
    }

    if (!uid_read) {
        device_uid_read();
    }

    /* Start with UID (12 bytes) */
    uint32_t offset = 0;
    for (uint32_t i = 0; i < UID_SIZE_BYTES && offset < out_len; i++) {
        out_buffer[offset++] = device_uid_buffer[i];
    }

    /* Add Flash size (F_SIZE register at 0x1FFF7A22) */
    if (offset + 2 <= out_len) {
        uint16_t flash_size = *(volatile uint16_t *)0x1FFF7A22;
        out_buffer[offset++] = (flash_size >> 0) & 0xFF;
        out_buffer[offset++] = (flash_size >> 8) & 0xFF;
    }

    /* Add Device ID and Revision ID from DBGMCU */
    if (offset + 4 <= out_len) {
        uint32_t dbgmcu_idcode = DBGMCU->IDCODE;
        out_buffer[offset++] = (dbgmcu_idcode >> 0)  & 0xFF;  /* DEV_ID low */
        out_buffer[offset++] = (dbgmcu_idcode >> 8)  & 0xFF;  /* DEV_ID high */
        out_buffer[offset++] = (dbgmcu_idcode >> 16) & 0xFF;  /* REV_ID low */
        out_buffer[offset++] = (dbgmcu_idcode >> 24) & 0xFF;  /* REV_ID high */
    }

    return offset;
}

/* ========== Usage Notes ========== */

/*
 * UID as Personalization String:
 *
 * const uint8_t *uid = device_uid_get_buffer();
 * cmox_drbg_init(drbg_ctx,
 *                entropy, 32,
 *                uid, 12,        // Use UID as personalization
 *                nonce, 16);
 *
 *
 * UID for Device-Specific Keys:
 *
 * uint8_t fingerprint[16];
 * device_generate_fingerprint(fingerprint, sizeof(fingerprint));
 *
 * // Mix with DRBG output for device-specific key
 * for (int i = 0; i < 16; i++) {
 *     derived_key[i] = drbg_key[i] ^ fingerprint[i];
 * }
 *
 *
 * UID for Anti-Cloning:
 *
 * // Store expected UID in flash (encrypted)
 * const uint8_t expected_uid[12] = { ... };
 *
 * // At runtime, verify
 * if (!device_uid_compare(expected_uid)) {
 *     // Device cloned or tampered!
 *     halt_system();
 * }
 */


