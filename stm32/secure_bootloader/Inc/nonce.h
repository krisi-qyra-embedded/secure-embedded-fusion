/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file nonce.h
 * @brief Nonce generation, entropy mixing and secure-clear declarations.
 */
#ifndef __NONCE_H__
#define __NONCE_H__
#include "stm32f4xx.h"
#include "pwm_dma.h"
#include "DMA.h"
#include <entropy_personalization_nonce_gen.h>
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* Nonce, static entropy and counter-string buffer sizes. */
#define NONCE_SIZE_BYTES  16
#define STATIC_ENTROPY_SIZE  32
#define COUNTER_STRING_SIZE  21
/* Runtime nonce state. */
extern volatile uint32_t message_counter;
//extern const uint8_t *nonce;
extern uint8_t nonce_buffer[NONCE_SIZE_BYTES];
extern  uint8_t static_entropy[8];
extern uint8_t static_entropy_initialized;
/** @brief Nonce subsystem status. */
typedef enum {
   NONCE_OK                    =  0,
   NONCE_ERR_ALREADY_INIT      = -1,
   NONCE_ERR_INVALID_CONSTANTS = -2,
   NONCE_ERR_ENTROPY_INIT_FAIL = -3,
   NONCE_ERR_BAD_INPUT         = -4
} nonce_status_t;
/* Optional weak platform entropy hook. */
#ifdef __GNUC__
#define WEAK_ATTR __attribute__((weak))
#else
#define WEAK_ATTR
#endif
#ifndef NONCE_SIZE_BYTES
#error "NONCE_SIZE_BYTES must be defined"
#endif
#ifndef STATIC_ENTROPY_SIZE
#error "STATIC_ENTROPY_SIZE must be defined"
#endif
#ifndef COUNTER_STRING_SIZE
#error "COUNTER_STRING_SIZE must be defined"
#endif
/**
 * @brief Fill a buffer from an optional platform entropy source.
 * @param buf Destination buffer.
 * @param len Requested byte count.
 * @return Platform result.
 */
int platform_get_random_bytes(uint8_t *buf, size_t len) WEAK_ATTR;
/**
 * @brief Initialize the nonce subsystem.
 * @return Nonce status.
 */
nonce_status_t nonce_simple_init(void);
/**
 * @brief Increment the per-message nonce counter.
 */
void nonce_increment_message_counter(void);
/**
 * @brief Get the current message counter.
 * @return Current counter value.
 */
uint32_t nonce_get_message_counter(void);
/**
 * @brief Get the current counter string.
 * @return Pointer to the counter string.
 */
const char* nonce_get_counter_string(void);
/**
 * @brief Generate the next nonce into the shared buffer.
 * @return Pointer to the generated nonce.
 */
uint8_t* nonce_simple_generate(void);
/**
 * @brief Generate a nonce and copy it to a caller buffer.
 * @param out_buffer Destination buffer.
 * @return Number of bytes written.
 */
uint32_t nonce_simple_generate_to_buffer(uint8_t *out_buffer);
/**
 * @brief Get the most recently generated nonce.
 * @return Pointer to the shared nonce buffer.
 */
const uint8_t* nonce_simple_get_last(void);
/**
 * @brief Print one nonce in hexadecimal.
 * @param nonce Nonce buffer.
 */
void nonce_simple_print(const uint8_t *nonce);
/**
 * @brief Print the current nonce breakdown.
 */
void nonce_simple_print_detailed(void);
/**
 * @brief Compare the current nonce against the previous value.
 * @return Non-zero when unique.
 */
uint8_t nonce_simple_verify_unique(void);
/**
 * @brief Mix the ADC and jitter hashes into static nonce entropy.
 * @param adc_pc4_hash PC4 ADC hash.
 * @param adc_pc5_hash PC5 ADC hash.
 * @param adc_pb0_hash PB0 ADC hash.
 * @param jitter_hash Timer-jitter hash.
 * @return Entropy-mixing result.
 */
int nonce_compute_static_entropy(const uint8_t *adc_pc4_hash,
                                 const uint8_t *adc_pc5_hash,
                                 const uint8_t *adc_pb0_hash,
                                 const uint8_t *jitter_hash);
/**
 * @brief Clear a sensitive buffer without allowing optimization away.
 * @param ptr Buffer to clear.
 * @param len Buffer size.
 */
void secure_zero(void *ptr, size_t len);
#endif
