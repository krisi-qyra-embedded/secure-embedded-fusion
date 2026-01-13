/* systick.h | Author: i fiksuari mbas prog | Date: 13 January 2026 */
#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    SYSTICK_OK = 0,
    SYSTICK_ERR_INVALID_ARG = -1,
    SYSTICK_ERR_RELOAD_OVERFLOW = -2,
    SYSTICK_ERR_ALREADY_INITIALIZED = -3,
    SYSTICK_ERR_NOT_INITIALIZED = -4
} systick_status_t;

extern volatile uint32_t g_ms_ticks;
extern volatile int systick_inited;
extern volatile uint32_t supervisor_last_tick;
extern volatile uint8_t  supervisor_due;


void SysTick_Handler(void);

/* Initialize SysTick to generate a 1 ms tick. */
systick_status_t systick_init(uint32_t sysclk_hz, uint32_t priority);

/* Disable SysTick and de-initialize the driver. */
void systick_deinit(void);

/* Return monotonic millisecond tick. */
uint32_t ms_get(void);


void systick_delay_ms(uint32_t ms);

/* A small self-test: verifies ms tick advances within timeout_ms. */
systick_status_t systick_selftest(uint32_t timeout_ms);

/* Optional user hook for reporting errors / diagnostics. */
void systick_error_hook(const char *msg, int code);

#ifdef __cplusplus
}
#endif

#endif 
