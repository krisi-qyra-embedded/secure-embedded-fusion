/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

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
systick_status_t systick_init(uint32_t sysclk_hz, uint32_t priority);
void systick_deinit(void);
uint32_t ms_get(void);
void systick_delay_ms(uint32_t ms);
systick_status_t systick_selftest(uint32_t timeout_ms);
void systick_error_hook(const char *msg, int code);
#ifdef __cplusplus
}
#endif
#endif /* SYSTICK_H */
