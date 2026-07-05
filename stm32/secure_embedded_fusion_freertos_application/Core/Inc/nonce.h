/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

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

#define NONCE_SIZE_BYTES  16
#define STATIC_ENTROPY_SIZE  32
#define COUNTER_STRING_SIZE  21
/* ========== Function Prototypes ========== */
extern volatile uint32_t message_counter;
//extern const uint8_t *nonce;
extern uint8_t nonce_buffer[NONCE_SIZE_BYTES];

extern  uint8_t static_entropy[8];
extern uint8_t static_entropy_initialized;

typedef enum {
    NONCE_OK                    =  0,  
    NONCE_ERR_ALREADY_INIT      = -1,  
    NONCE_ERR_INVALID_CONSTANTS = -2,  
    NONCE_ERR_ENTROPY_INIT_FAIL = -3,  
    NONCE_ERR_BAD_INPUT         = -4   
} nonce_status_t;

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

nonce_status_t nonce_simple_init(void);

void nonce_increment_message_counter(void);
uint32_t nonce_get_message_counter(void);
const char* nonce_get_counter_string(void);
 uint8_t* nonce_simple_generate(void);
uint32_t nonce_simple_generate_to_buffer(uint8_t *out_buffer);
const uint8_t* nonce_simple_get_last(void);
void nonce_simple_print(const uint8_t *nonce);
void nonce_simple_print_detailed(void);
uint8_t nonce_simple_verify_unique(void);

int nonce_compute_static_entropy(const uint8_t *adc_pc4_hash,
                                  const uint8_t *adc_pc5_hash,
                                  const uint8_t *adc_pb0_hash,
                                  const uint8_t *jitter_hash);

void secure_zero(void *ptr, size_t len);

#endif
