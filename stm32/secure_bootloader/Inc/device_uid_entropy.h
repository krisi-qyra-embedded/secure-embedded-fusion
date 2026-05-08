/* Author: i fiksuari pas prog | Documentation date: 8 May 2026 */
#ifndef __DEVICE_UID_ENTROPY__
#define __DEVICE_UID_ENTROPY__

#include "stm32f4xx.h"
#include "device_uid_entropy.h"
#include "DMA.h"
#include "pwm_dma.h"
#include "sram_startup_entropy.h"
#include "nonce.h"
#include "uart.h"  /* log_printf() */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ========== STM32F411 UID Register Addresses ========== */

/* Unique ID base address (96 bits total, stored as 3x 32-bit words) */
#define UID_BASE_ADDR       0x1FFF7A10
/* Individual 32-bit word addresses */
#define UID_WORD0_ADDR      (UID_BASE_ADDR + 0x00)  /* Bits [31:0] */
#define UID_WORD1_ADDR      (UID_BASE_ADDR + 0x04)  /* Bits [63:32] */
#define UID_WORD2_ADDR      (UID_BASE_ADDR + 0x08)  /* Bits [95:64] */
/* Device UID read return macros */
#define DEVICE_UID_READ_SUCCESS        0
#define DEVICE_UID_READ_NULL_BUFFER   -401
#define DEVICE_UID_READ_INVALID_UID   -402
/* UID size in bytes */
#define UID_SIZE_BYTES      12

/* Buffer to store UID (12 bytes) */
extern uint8_t device_uid_buffer[UID_SIZE_BYTES];
extern volatile uint8_t uid_read;
extern uint32_t uid_hash;

int device_uid_read(void);
const uint8_t* device_uid_get_buffer(void);
uint32_t device_uid_get_size(void);
uint32_t device_uid_get_bytes(uint8_t *out_buffer, uint32_t out_len);
void device_uid_get_words(uint32_t *uid0, uint32_t *uid1, uint32_t *uid2);
uint8_t device_uid_compare(const uint8_t *other_uid);
uint32_t device_uid_hash(void);
void device_uid_print(void);
void device_uid_print_bytes(void);
uint8_t device_uid_is_valid(void);
uint32_t device_generate_fingerprint(uint8_t *out_buffer, uint32_t out_len);


#endif
