/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
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
#define UID_BASE_ADDR       0x1FFF7A10
#define UID_WORD0_ADDR      (UID_BASE_ADDR + 0x00)
#define UID_WORD1_ADDR      (UID_BASE_ADDR + 0x04)
#define UID_WORD2_ADDR      (UID_BASE_ADDR + 0x08)
#define DEVICE_UID_READ_SUCCESS        0
#define DEVICE_UID_READ_NULL_BUFFER   -401
#define DEVICE_UID_READ_INVALID_UID   -402
#define UID_SIZE_BYTES      12
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
