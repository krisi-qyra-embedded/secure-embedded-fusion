/* systick.h | Author: i fiksuari pas prog | Date: 22 February 2026 */
#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Status codes */
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

/* Initialize SysTick to generate a 1 ms tick.
 * - sysclk_hz: CPU clock frequency in Hz (e.g. 16000000U).
 * - priority: SysTick IRQ priority (0 = highest). Use value appropriate for your system.
 *
 * Returns SYSTICK_OK on success or a negative systick_status_t on error.
 *
 * Notes:
 * - Call once early in startup (after SystemCoreClock is valid).
 * - This routine is defensive: it validates reload fits in 24-bit LOAD and protects
 *   against double-initialization.
 */
systick_status_t systick_init(uint32_t sysclk_hz, uint32_t priority);

/* Disable SysTick and de-initialize the driver. Safe to call before re-init. */
void systick_deinit(void);

/* Return monotonic millisecond tick. Wraps every ~49 days.
 * Reading a 32-bit volatile on 32-bit Cortex is atomic; no IRQ masking required.
 */
uint32_t ms_get(void);

/* Blocking millisecond delay using the millisecond tick.
 * Implemented as a low-power-friendly busy-wait (uses __WFI).
 */
void systick_delay_ms(uint32_t ms);

/* A small self-test: verifies ms tick advances within timeout_ms.
 * Returns SYSTICK_OK if tick advanced, otherwise negative error.
 */
systick_status_t systick_selftest(uint32_t timeout_ms);

/* Optional user hook for reporting errors / diagnostics.
 * Weak default provided; application may override to capture faults.
 */
void systick_error_hook(const char *msg, int code);

#ifdef __cplusplus
}
#endif

#endif /* SYSTICK_H */
