/**
 * @file nonce.h
 * @brief Nonce generation, counter tracking and entropy-mixing declarations.
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
/* Nonce, entropy and counter-string buffer sizes. */
#define NONCE_SIZE_BYTES  16
#define STATIC_ENTROPY_SIZE  32
#define COUNTER_STRING_SIZE  21
/* Monotonic message counter and nonce working state. */
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
/* Optional weak platform randomness hook. */
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
 * @brief Fill a buffer from an optional platform randomness source.
 * @param buf Destination buffer.
 * @param len Number of bytes requested.
 * @return 0 on success, non-zero on failure.
 */
int platform_get_random_bytes(uint8_t *buf, size_t len) WEAK_ATTR;
/**
 * @brief Initialize the nonce subsystem.
 * @return Nonce status.
 */
nonce_status_t nonce_simple_init(void);
/**
 * @brief Increment the message counter after a transmission.
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
 * @brief Generate the next nonce into the shared nonce buffer.
 * @return Pointer to the generated nonce.
 */
 uint8_t* nonce_simple_generate(void);
/**
 * @brief Generate a nonce and copy it to an external buffer.
 * @param out_buffer Destination buffer.
 * @return Number of bytes written.
 */
uint32_t nonce_simple_generate_to_buffer(uint8_t *out_buffer);
/**
 * @brief Get the last generated nonce.
 * @return Pointer to the nonce buffer.
 */
const uint8_t* nonce_simple_get_last(void);
/**
 * @brief Print a nonce in hexadecimal form.
 * @param nonce Nonce buffer.
 */
void nonce_simple_print(const uint8_t *nonce);
/**
 * @brief Print the current nonce with its internal fields.
 */
void nonce_simple_print_detailed(void);
/**
 * @brief Check the current nonce against the previous value.
 * @return 1 when unique, otherwise 0.
 */
uint8_t nonce_simple_verify_unique(void);
/**
 * @brief Build the static entropy value from the hashed hardware sources.
 * @param adc_pc4_hash PC4 ADC hash.
 * @param adc_pc5_hash PC5 ADC hash.
 * @param adc_pb0_hash PB0 ADC hash.
 * @param jitter_hash Timer-jitter hash.
 * @return Static entropy generation status.
 */
int nonce_compute_static_entropy(const uint8_t *adc_pc4_hash,
                                 const uint8_t *adc_pc5_hash,
                                 const uint8_t *adc_pb0_hash,
                                 const uint8_t *jitter_hash);
/**
 * @brief Securely clear a memory region.
 * @param ptr Buffer to clear.
 * @param len Number of bytes to clear.
 */
void secure_zero(void *ptr, size_t len);
#endif
