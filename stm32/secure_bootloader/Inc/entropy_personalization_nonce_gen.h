/* Author: i fiksuari pas prog | Documentation date: 11 May 2026 */

#ifndef ENTROPY_PERSONALIZATION_NONCE_GEN_H_
#define ENTROPY_PERSONALIZATION_NONCE_GEN_H_

#include "stm32f4xx.h"
#include "pwm_dma.h"
#include "DMA.h"
#include <sram_startup_entropy.h>
#include <device_uid_entropy.h>
#include <nonce.h>
#include "uart.h"  /* log_printf() */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "cmox_crypto.h"

/* Hash length for SHA-256 */
#define SRAM_ENTROPY_BUFFER_SIZE  		 256
/* Return codes */
#define ENTROPY_HASH_SUCCESS             0
#define ENTROPY_HASH_ERR_NULL_INPUT     -1
#define ENTROPY_HASH_ERR_BAD_LENGTH     -2
#define ENTROPY_HASH_ERR_HASH_FAILED    -3
#define ENTROPY_HASH_ERR_BUFFER_EMPTY   -4
#define ENTROPY_HASH_ERR_COMBINE_FAILED -5

/* Return macros */
#define ADC_HASH_ALL_SUCCESS        0
#define ADC_PC4_HASH_FAILED       -101
#define ADC_PC5_HASH_FAILED       -102
#define ADC_PB0_HASH_FAILED       -103
#define ADC_HASH_RETRY_COUNT        2  /* number of attempts per hash */
#define DEVICE_UID_HASH_FAILED        -204

/* Entropy creation return macros */
#define ENTROPY_CREATION_SUCCESS        0
#define JITTER_HASH_FAILED            -201
#define SRAM_HASH_FAILED              -202
#define COMBINED_HASH_FAILED          -203
#define ENTROPY_CREATION_RETRY_COUNT     2  /* attempts per step */

/* hash_device_uid macros */
#define DEVICE_UID_HASH_LEN                32U
#define ADC_HASH_LEN                   	   32U
#define JITTER_HASHED                      32U
#define SRAM_ENTROPY_HASH_LEN              32U
#define ENTROPY_HASH_LEN                   32U
#define NONCE_HASH_LEN                     16U

#define UID_SIZE_BYTES      		       12
/* UID size in bytes */
#define DEVICE_UID_HASH_SUCCESS             0
#define DEVICE_UID_HASH_ERR_BAD_INPUT      -2
#define DEVICE_UID_HASH_ERR_HASH_FAILED    -1

/* ===== entropy_personalisation_HW_init result codes ===== */
#define ENTROPY_HW_INIT_SUCCESS            0
#define ENTROPY_HW_INIT_ADC_FAILED        -7001
#define ENTROPY_HW_INIT_JITTER_FAILED     -7002
#define ENTROPY_HW_INIT_SRAM_FAILED       -7003
#define ENTROPY_HW_INIT_UID_FAILED        -7004

/* ===== Nonce generation return codes ===== */
#define NONCE_GEN_SUCCESS           0
#define NONCE_GEN_NULL_NONCE       -601
#define NONCE_GEN_FAILED           -602
#define NONCE_GEN_RETRY_COUNT        2

/* convenience test */
#define DEVICE_UID_HASH_OK(rc) ((rc) == DEVICE_UID_HASH_SUCCESS)

extern uint8_t device_uid_buffer_hashed[DEVICE_UID_HASH_LEN ];
extern uint8_t buf_pc4_hashed[ENTROPY_HASH_LEN];
extern uint8_t buf_pc5_hashed[ENTROPY_HASH_LEN];
extern uint8_t buf_pb0_hashed[ENTROPY_HASH_LEN];
extern uint8_t jitter_accum_hashed[ENTROPY_HASH_LEN];
extern uint8_t sram_entropy_buffer_hashed[ENTROPY_HASH_LEN];
extern uint8_t entropy_hashed[ENTROPY_HASH_LEN];
extern uint8_t nonce_hashed[NONCE_HASH_LEN];

int compute_device_uid_hashed_global(void);
int entropy_personalisation_hash(void);
int hash_adc_pc4(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len);
int hash_adc_pc5(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len);
int hash_adc_pb0(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len);
int hash_jitter_accum(const uint32_t *in_jitter, uint8_t *out_hash, size_t out_len);
int hash_sram_entropy(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len);
int combine_all_hashed_entropy(uint8_t *out_hash, size_t out_len);
int entropy_personalisation_HW_init(void);
int nonce_generation(void);


#endif /* ENTROPY_PERSONALIZATION_NONCE_GEN_H_ */
