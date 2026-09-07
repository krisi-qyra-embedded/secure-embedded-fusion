/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "device_uid_entropy.h"


uint8_t device_uid_buffer[UID_SIZE_BYTES] = {0};
volatile uint8_t uid_read = 0;
uint32_t uid_hash;

/* Reads and caches the STM32 factory UID. */
int device_uid_read(void)
{
    /* Reject an all-zero UID before publishing the cached bytes. */
    if (device_uid_buffer == NULL) {
        log_printf("device_uid_read_safe: device_uid_buffer is NULL\n");
        return DEVICE_UID_READ_NULL_BUFFER;
    }

    uint32_t uid0 = *(volatile uint32_t *)UID_WORD0_ADDR;
    uint32_t uid1 = *(volatile uint32_t *)UID_WORD1_ADDR;
    uint32_t uid2 = *(volatile uint32_t *)UID_WORD2_ADDR;

    if ((uid0 == 0U) && (uid1 == 0U) && (uid2 == 0U)) {
        log_printf("device_uid_read_safe: UID registers returned all zeros\n");
        return DEVICE_UID_READ_INVALID_UID;
    }

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

/* Returns the cached device UID bytes. */
const uint8_t *device_uid_get_buffer(void)
{
    /* Read the UID first if the cache has not been populated. */
    if (!uid_read) {
        device_uid_read();
    }

    return device_uid_buffer;
}

/* Returns the device UID byte count. */
uint32_t device_uid_get_size(void)
{
    /* Use the same size as the cached 96-bit UID buffer. */
    return UID_SIZE_BYTES;
}

/* Copies UID bytes into the caller's buffer. */
uint32_t device_uid_get_bytes(uint8_t *out_buffer, uint32_t out_len)
{
    /* Limit the copy to the smaller of the UID and output sizes. */
    if (out_buffer == NULL || out_len == 0) {
        return 0;
    }

    if (!uid_read) {
        device_uid_read();
    }

    uint32_t to_copy =
        (out_len < UID_SIZE_BYTES) ? out_len : UID_SIZE_BYTES;

    for (uint32_t i = 0; i < to_copy; i++) {
        out_buffer[i] = device_uid_buffer[i];
    }

    return to_copy;
}

/* Reconstructs the three cached UID words. */
void device_uid_get_words(uint32_t *uid0,
                          uint32_t *uid1,
                          uint32_t *uid2)
{
    /* Fill only the output word pointers supplied by the caller. */
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

/* Compares a supplied UID with this device's UID. */
uint8_t device_uid_compare(const uint8_t *other_uid)
{
    /* Reject a null input and stop at the first differing byte. */
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

/* Forms a compact XOR identifier from the UID. */
uint32_t device_uid_hash(void)
{
    /* Combine the three UID words; this is not a cryptographic hash. */
    if (!uid_read) {
        device_uid_read();
    }

    uint32_t uid0, uid1, uid2;

    device_uid_get_words(&uid0, &uid1, &uid2);

    return uid0 ^ uid1 ^ uid2;
}

/* Logs the UID as three hexadecimal words. */
void device_uid_print(void)
{
    /* Ensure the UID cache is populated before formatting it. */
    extern int printf(const char *fmt, ...);   

    if (!uid_read) {
        device_uid_read();
    }

    uint32_t uid0, uid1, uid2;

    device_uid_get_words(&uid0, &uid1, &uid2);

    log_printf("Device UID: %08lX-%08lX-%08lX\r\n",
               uid0,
               uid1,
               uid2);
}

/* Logs the cached UID byte sequence. */
void device_uid_print_bytes(void)
{
    /* Separate the bytes with commas for diagnostic output. */
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

/* Checks that the cached UID is neither zero nor erased. */
uint8_t device_uid_is_valid(void)
{
    /* Track all-zero and all-ones patterns across the full UID. */
    if (!uid_read) {
        device_uid_read();
    }

    uint8_t all_zero = 1;
    uint8_t all_ff = 1;

    for (uint32_t i = 0; i < UID_SIZE_BYTES; i++) {
        if (device_uid_buffer[i] != 0x00) {
            all_zero = 0;
        }

        if (device_uid_buffer[i] != 0xFF) {
            all_ff = 0;
        }
    }

    return (!all_zero && !all_ff) ? 1 : 0;
}

/* ========== Advanced: UID-based Device Fingerprint ========== */

/* Builds a device fingerprint from UID and chip metadata. */
uint32_t device_generate_fingerprint(uint8_t *out_buffer,
                                     uint32_t out_len)
{
    /* Append flash size and debug identification only when space remains. */
    if (out_buffer == NULL || out_len < 16) {
        return 0;
    }

    if (!uid_read) {
        device_uid_read();
    }

    uint32_t offset = 0;

    for (uint32_t i = 0;
         i < UID_SIZE_BYTES && offset < out_len;
         i++) {

        out_buffer[offset++] = device_uid_buffer[i];
    }

    if (offset + 2 <= out_len) {

        uint16_t flash_size =
            *(volatile uint16_t *)0x1FFF7A22;

        out_buffer[offset++] =
            (flash_size >> 0) & 0xFF;

        out_buffer[offset++] =
            (flash_size >> 8) & 0xFF;
    }

    if (offset + 4 <= out_len) {

        uint32_t dbgmcu_idcode = DBGMCU->IDCODE;

        out_buffer[offset++] =
            (dbgmcu_idcode >> 0) & 0xFF;    

        out_buffer[offset++] =
            (dbgmcu_idcode >> 8) & 0xFF;    

        out_buffer[offset++] =
            (dbgmcu_idcode >> 16) & 0xFF;   

        out_buffer[offset++] =
            (dbgmcu_idcode >> 24) & 0xFF;   
    }

    return offset;
}

/* ========== Usage Notes ========== */

/*
 * UID as Personalization String:
 *
 * const uint8_t *uid = device_uid_get_buffer();
 *
 * cmox_drbg_init(drbg_ctx,
 *                entropy, 32,
 *                uid, 12,
 *                nonce, 16);
 *
 *
 * UID for Device-Specific Keys:
 *
 * uint8_t fingerprint[16];
 *
 * device_generate_fingerprint(
 *     fingerprint,
 *     sizeof(fingerprint));
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
 *     // Device cloned or tampered
 *     halt_system();
 * }
 */
