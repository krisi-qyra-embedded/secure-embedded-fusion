/**
 * @file device_uid_entropy.c
 * @brief STM32F411 UID collection and device-specific personalization helpers.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include "device_uid_entropy.h"
/* Cached 96-bit device UID and read state. */
uint8_t device_uid_buffer[UID_SIZE_BYTES] = {0};
volatile uint8_t uid_read = 0;
uint32_t uid_hash;
/**
 * @brief Read and validate the factory-programmed STM32 device UID.
 * @return Device UID read status.
 */
int device_uid_read(void)
{
    if (device_uid_buffer == NULL) {
        log_printf("device_uid_read_safe: device_uid_buffer is NULL\n");
        return DEVICE_UID_READ_NULL_BUFFER;
    }
    /* Read the three factory UID words directly from the system-memory registers. */
    uint32_t uid0 = *(volatile uint32_t *)UID_WORD0_ADDR;
    uint32_t uid1 = *(volatile uint32_t *)UID_WORD1_ADDR;
    uint32_t uid2 = *(volatile uint32_t *)UID_WORD2_ADDR;
    /* Reject an impossible all-zero device identity. */
    if ((uid0 == 0U) && (uid1 == 0U) && (uid2 == 0U)) {
        log_printf("device_uid_read_safe: UID registers returned all zeros\n");
        return DEVICE_UID_READ_INVALID_UID;
    }
    /* Store the UID in little-endian byte order for the entropy/personalization path. */
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
 * @brief Get the cached 12-byte device UID.
 * @return Pointer to the UID buffer.
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
 * @brief Get the STM32 device UID size.
 * @return UID size in bytes.
 */
uint32_t device_uid_get_size(void)
{
    /* Check the return status before continuing to the next step. */
    return UID_SIZE_BYTES;
}
/**
 * @brief Copy the device UID into a caller-provided buffer.
 * @param out_buffer Destination buffer.
 * @param out_len Destination buffer size.
 * @return Number of UID bytes copied.
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
 * @brief Return the device UID as three 32-bit words.
 * @param uid0 First UID word output.
 * @param uid1 Second UID word output.
 * @param uid2 Third UID word output.
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
 * @brief Compare the local device UID with another 12-byte UID.
 * @param other_uid UID to compare against.
 * @return 1 on match, otherwise 0.
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
 * @brief Fold the three UID words into a compact 32-bit diagnostic value.
 * @return 32-bit XOR-folded UID value.
 */
uint32_t device_uid_hash(void)
{
    if (!uid_read) {
        device_uid_read();
    }
    uint32_t uid0, uid1, uid2;
    device_uid_get_words(&uid0, &uid1, &uid2);
    /* Compact diagnostic value only; this is not a cryptographic hash. */
    return uid0 ^ uid1 ^ uid2;
}
/**
 * @brief Print the device UID as three hexadecimal words.
 */
void device_uid_print(void)
{
    /* Check the return status before continuing to the next step. */
    extern int printf(const char *fmt, ...);
    if (!uid_read) {
        device_uid_read();
    }
    uint32_t uid0, uid1, uid2;
    device_uid_get_words(&uid0, &uid1, &uid2);
    log_printf("Device UID: %08lX-%08lX-%08lX\r\n", uid0, uid1, uid2);
}
/**
 * @brief Print the device UID as a byte array.
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
 * @brief Check that the cached UID is not all-zero or all-0xFF.
 * @return 1 when the UID looks valid, otherwise 0.
 */
uint8_t device_uid_is_valid(void)
{
    /* Return a failure status if any validation check fails. */
    if (!uid_read) {
        device_uid_read();
    }
    uint8_t all_zero = 1;
    uint8_t all_ff = 1;
    for (uint32_t i = 0; i < UID_SIZE_BYTES; i++) {
        if (device_uid_buffer[i] != 0x00) all_zero = 0;
        if (device_uid_buffer[i] != 0xFF) all_ff = 0;
    }
    return (!all_zero && !all_ff) ? 1 : 0;
}
/**
 * @brief Build a device fingerprint from UID, Flash size and DBGMCU identification.
 * @param out_buffer Fingerprint output buffer.
 * @param out_len Output buffer size.
 * @return Number of bytes written.
 */
uint32_t device_generate_fingerprint(uint8_t *out_buffer, uint32_t out_len)
{
    if (out_buffer == NULL || out_len < 16) {
        return 0;
    }
    if (!uid_read) {
        device_uid_read();
    }
    /* Fingerprint layout starts with the 12-byte UID. */
    uint32_t offset = 0;
    for (uint32_t i = 0; i < UID_SIZE_BYTES && offset < out_len; i++) {
        out_buffer[offset++] = device_uid_buffer[i];
    }
    if (offset + 2 <= out_len) {
        /* Append the factory Flash-size register when space is available. */
        uint16_t flash_size = *(volatile uint16_t *)0x1FFF7A22;
        out_buffer[offset++] = (flash_size >> 0) & 0xFF;
        out_buffer[offset++] = (flash_size >> 8) & 0xFF;
    }
    if (offset + 4 <= out_len) {
        /* Finish with device and revision identification from DBGMCU. */
        uint32_t dbgmcu_idcode = DBGMCU->IDCODE;
        out_buffer[offset++] = (dbgmcu_idcode >> 0)  & 0xFF;
        out_buffer[offset++] = (dbgmcu_idcode >> 8)  & 0xFF;
        out_buffer[offset++] = (dbgmcu_idcode >> 16) & 0xFF;
        out_buffer[offset++] = (dbgmcu_idcode >> 24) & 0xFF;
    }
    return offset;
}
