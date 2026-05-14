/* Author: i fiksuari pas prog | Documentation date: 14 May 2026 */
#ifndef __NONCE_H__
#define __NONCE_H__

#include "stm32f4xx.h"
#include "pwm_dma.h"
#include "DMA.h"
#include <entropy_personalization_nonce_gen.h>
#include "uart.h"  /* log_printf() */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Nonce size (8 bytes: 4 counter + 4 timer) */
#define NONCE_SIZE_BYTES  16
#define STATIC_ENTROPY_SIZE  32
#define COUNTER_STRING_SIZE  21
/* ========== Function Prototypes ========== */
/* Message counter (increments with each data transmission) */
extern volatile uint32_t message_counter;
/* Generate nonce */
//extern const uint8_t *nonce;
extern uint8_t nonce_buffer[NONCE_SIZE_BYTES];

extern  uint8_t static_entropy[8];
extern uint8_t static_entropy_initialized;

/* Status return codes for nonce subsystem */
typedef enum {
    NONCE_OK                    =  0, /* success */
    NONCE_ERR_ALREADY_INIT      = -1, /* already initialised (use reseed if needed) */
    NONCE_ERR_INVALID_CONSTANTS = -2, /* compile-time sizes are invalid */
    NONCE_ERR_ENTROPY_INIT_FAIL = -3, /* failed to seed static entropy */
    NONCE_ERR_BAD_INPUT         = -4  /* general / unexpected error */
} nonce_status_t;

/* Optional platform hook: if defined by platform, used to seed static_entropy.
 * Prototype expected to return 0 on success, non-zero on failure.
 * Provide your own implementation in platform code and define PLATFORM_HAS_HW_RNG.
 */
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



int platform_get_random_bytes(uint8_t *buf, size_t len) WEAK_ATTR;

/**
 * Initialize nonce system
 * Call once at startup
 */
nonce_status_t nonce_simple_init(void);

/**
 * Increment message counter
 * Call EVERY TIME data is sent
 */
void nonce_increment_message_counter(void);
/**
 * Get current message counter value
 * Returns: Message count (e.g., 1, 2, 3...)
 */
uint32_t nonce_get_message_counter(void);
/**
 * Get counter as string format
 * Returns: String like \"message1_sent\", \"message2_sent\"
 */
const char* nonce_get_counter_string(void);
/**
 * Generate 8-byte nonce
 *
 * [0-3]: Counter (from \"messageN_sent\" string)
 * [4-7]: TIM1 timer snapshot (continuous counter)
 *
 * Returns: Pointer to 8-byte nonce
 */
 uint8_t* nonce_simple_generate(void);
/**
 * Generate nonce and copy to buffer
 *
 * out_buffer: Must be at least 8 bytes
 * Returns: Bytes written (8)
 */
uint32_t nonce_simple_generate_to_buffer(uint8_t *out_buffer);
/**
 * Get last generated nonce
 * Returns: Pointer to 8-byte buffer
 */
const uint8_t* nonce_simple_get_last(void);
/**
 * Print nonce in hex
 */
void nonce_simple_print(const uint8_t *nonce);
/**
 * Print nonce with detailed breakdown
 */
void nonce_simple_print_detailed(void);
/**
 * Verify nonce is unique (different from previous)
 * Returns: 1 if unique, 0 if collision
 */
uint8_t nonce_simple_verify_unique(void);

int nonce_compute_static_entropy(const uint8_t *adc_pc4_hash,
                                  const uint8_t *adc_pc5_hash,
                                  const uint8_t *adc_pb0_hash,
                                  const uint8_t *jitter_hash);

void secure_zero(void *ptr, size_t len);


#endif
