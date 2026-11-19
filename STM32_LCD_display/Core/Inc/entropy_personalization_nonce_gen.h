/**
 * @file entropy_gen.h
 * @brief Entropy hashing, UID personalisation and nonce generation declarations.
 */
#ifndef ENTROPY_PERSONALIZATION_NONCE_GEN_H_
#define ENTROPY_PERSONALIZATION_NONCE_GEN_H_
#include "stm32f4xx.h"
#include "pwm_dma.h"
#include "DMA.h"
#include <sram_startup_entropy.h>
#include <device_uid_entropy.h>
#include <nonce.h>
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "cmox_crypto.h"
/* Entropy buffer and hash status values. */
#define SRAM_ENTROPY_BUFFER_SIZE  		 256
#define ENTROPY_HASH_SUCCESS             0
#define ENTROPY_HASH_ERR_NULL_INPUT     -1
#define ENTROPY_HASH_ERR_BAD_LENGTH     -2
#define ENTROPY_HASH_ERR_HASH_FAILED    -3
#define ENTROPY_HASH_ERR_BUFFER_EMPTY   -4
#define ENTROPY_HASH_ERR_COMBINE_FAILED -5
/* ADC and UID hash status values. */
#define ADC_HASH_ALL_SUCCESS        0
#define ADC_PC4_HASH_FAILED       -101
#define ADC_PC5_HASH_FAILED       -102
#define ADC_PB0_HASH_FAILED       -103
#define ADC_HASH_RETRY_COUNT        2
#define DEVICE_UID_HASH_FAILED        -204
/* Entropy collection and combination status values. */
#define ENTROPY_CREATION_SUCCESS        0
#define JITTER_HASH_FAILED            -201
#define SRAM_HASH_FAILED              -202
#define COMBINED_HASH_FAILED          -203
#define ENTROPY_CREATION_RETRY_COUNT     2
/* Hash lengths used by each entropy source. */
#define DEVICE_UID_HASH_LEN                32U
#define ADC_HASH_LEN                   	   32U
#define JITTER_HASHED                      32U
#define SRAM_ENTROPY_HASH_LEN              32U
#define ENTROPY_HASH_LEN                   32U
#define NONCE_HASH_LEN                     16U
#define UID_SIZE_BYTES      		       12
/* Device UID hash status values. */
#define DEVICE_UID_HASH_SUCCESS             0
#define DEVICE_UID_HASH_ERR_BAD_INPUT      -2
#define DEVICE_UID_HASH_ERR_HASH_FAILED    -1
/* Hardware entropy initialization status values. */
#define ENTROPY_HW_INIT_SUCCESS            0
#define ENTROPY_HW_INIT_ADC_FAILED        -7001
#define ENTROPY_HW_INIT_JITTER_FAILED     -7002
#define ENTROPY_HW_INIT_SRAM_FAILED       -7003
#define ENTROPY_HW_INIT_UID_FAILED        -7004
/* Nonce generation status values. */
#define NONCE_GEN_SUCCESS           0
#define NONCE_GEN_NULL_NONCE       -601
#define NONCE_GEN_FAILED           -602
#define NONCE_GEN_RETRY_COUNT        2
/* Helper for UID hash success checks. */
#define DEVICE_UID_HASH_OK(rc) ((rc) == DEVICE_UID_HASH_SUCCESS)
/* Hashed entropy material shared with the crypto layer. */
extern uint8_t device_uid_buffer_hashed[DEVICE_UID_HASH_LEN ];
extern uint8_t buf_pc4_hashed[ENTROPY_HASH_LEN];
extern uint8_t buf_pc5_hashed[ENTROPY_HASH_LEN];
extern uint8_t buf_pb0_hashed[ENTROPY_HASH_LEN];
extern uint8_t jitter_accum_hashed[ENTROPY_HASH_LEN];
extern uint8_t sram_entropy_buffer_hashed[ENTROPY_HASH_LEN];
extern uint8_t entropy_hashed[ENTROPY_HASH_LEN];
extern uint8_t nonce_hashed[NONCE_HASH_LEN];
/**
 * @brief Hash the STM32 UID into the global UID hash buffer.
 * @return Device UID hash status.
 */
int compute_device_uid_hashed_global(void);
/**
 * @brief Hash and combine all configured entropy sources.
 * @return Entropy hash status.
 */
int entropy_personalisation_hash(void);
/**
 * @brief Hash the PC4 ADC entropy buffer.
 * @param in_buffer ADC input buffer.
 * @param out_hash Hash output buffer.
 * @param out_len Output buffer size.
 * @return ADC hash status.
 */
int hash_adc_pc4(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len);
/**
 * @brief Hash the PC5 ADC entropy buffer.
 * @param in_buffer ADC input buffer.
 * @param out_hash Hash output buffer.
 * @param out_len Output buffer size.
 * @return ADC hash status.
 */
int hash_adc_pc5(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len);
/**
 * @brief Hash the PB0 ADC entropy buffer.
 * @param in_buffer ADC input buffer.
 * @param out_hash Hash output buffer.
 * @param out_len Output buffer size.
 * @return ADC hash status.
 */
int hash_adc_pb0(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len);
/**
 * @brief Hash the captured timer-jitter samples.
 * @param in_jitter Jitter sample buffer.
 * @param out_hash Hash output buffer.
 * @param out_len Output buffer size.
 * @return Jitter hash status.
 */
int hash_jitter_accum(const uint32_t *in_jitter, uint8_t *out_hash, size_t out_len);
/**
 * @brief Hash the sampled SRAM startup entropy.
 * @param in_buffer SRAM entropy buffer.
 * @param out_hash Hash output buffer.
 * @param out_len Output buffer size.
 * @return SRAM hash status.
 */
int hash_sram_entropy(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len);
/**
 * @brief Combine the individual entropy hashes into one final hash.
 * @param out_hash Final entropy hash buffer.
 * @param out_len Output buffer size.
 * @return Entropy combination status.
 */
int combine_all_hashed_entropy(uint8_t *out_hash, size_t out_len);
/**
 * @brief Initialize the hardware entropy sources used by the crypto layer.
 * @return Hardware entropy initialization status.
 */
int entropy_personalisation_HW_init(void);
/**
 * @brief Generate and hash the nonce input material.
 * @return Nonce generation status.
 */
int nonce_generation(void);
#endif /* ENTROPY_PERSONALIZATION_NONCE_GEN_H_ */
