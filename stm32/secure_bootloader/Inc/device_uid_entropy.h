/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file device_uid_entropy.h
 * @brief STM32F411 device UID access and fingerprint helper declarations.
 */
#ifndef __DEVICE_UID_ENTROPY__
#define __DEVICE_UID_ENTROPY__
#include "stm32f4xx.h"
#include "device_uid_entropy.h"
#include "DMA.h"
#include "pwm_dma.h"
#include "sram_startup_entropy.h"
#include "nonce.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* STM32F411 96-bit unique device identifier. */
#define UID_BASE_ADDR       0x1FFF7A10
#define UID_WORD0_ADDR      (UID_BASE_ADDR + 0x00)
#define UID_WORD1_ADDR      (UID_BASE_ADDR + 0x04)
#define UID_WORD2_ADDR      (UID_BASE_ADDR + 0x08)
/* UID read status values. */
#define DEVICE_UID_READ_SUCCESS        0
#define DEVICE_UID_READ_NULL_BUFFER   -401
#define DEVICE_UID_READ_INVALID_UID   -402
#define UID_SIZE_BYTES      12
/* Cached UID state. */
extern uint8_t device_uid_buffer[UID_SIZE_BYTES];
extern volatile uint8_t uid_read;
extern uint32_t uid_hash;
/**
 * @brief Read the STM32 device UID into the local buffer.
 * @return UID read status.
 */
int device_uid_read(void);
/**
 * @brief Get the cached UID buffer.
 * @return Pointer to the 12-byte UID buffer.
 */
const uint8_t* device_uid_get_buffer(void);
/**
 * @brief Get the UID size.
 * @return UID size in bytes.
 */
uint32_t device_uid_get_size(void);
/**
 * @brief Copy UID bytes to an external buffer.
 * @param out_buffer Destination buffer.
 * @param out_len Destination buffer size.
 * @return Number of bytes copied.
 */
uint32_t device_uid_get_bytes(uint8_t *out_buffer, uint32_t out_len);
/**
 * @brief Read the UID as three 32-bit words.
 * @param uid0 First UID word.
 * @param uid1 Second UID word.
 * @param uid2 Third UID word.
 */
void device_uid_get_words(uint32_t *uid0, uint32_t *uid1, uint32_t *uid2);
/**
 * @brief Compare another UID against the cached device UID.
 * @param other_uid UID buffer to compare.
 * @return Comparison result.
 */
uint8_t device_uid_compare(const uint8_t *other_uid);
/**
 * @brief Calculate the project UID hash.
 * @return UID hash value.
 */
uint32_t device_uid_hash(void);
/**
 * @brief Print the device UID.
 */
void device_uid_print(void);
/**
 * @brief Print the UID as individual bytes.
 */
void device_uid_print_bytes(void);
/**
 * @brief Check whether the cached UID is valid.
 * @return Non-zero when the UID is valid.
 */
uint8_t device_uid_is_valid(void);
/**
 * @brief Generate a device fingerprint into an output buffer.
 * @param out_buffer Destination buffer.
 * @param out_len Destination buffer size.
 * @return Number of bytes written.
 */
uint32_t device_generate_fingerprint(uint8_t *out_buffer, uint32_t out_len);
#endif
