/**
 * @file systick.h
 * @brief SysTick 1 ms timebase, delay and self-test declarations.
 */
#ifndef SYSTICK_H
#define SYSTICK_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
/** @brief SysTick driver status. */
typedef enum {
   SYSTICK_OK = 0,
   SYSTICK_ERR_INVALID_ARG = -1,
   SYSTICK_ERR_RELOAD_OVERFLOW = -2,
   SYSTICK_ERR_ALREADY_INITIALIZED = -3,
   SYSTICK_ERR_NOT_INITIALIZED = -4
} systick_status_t;
/* Millisecond timebase and supervisor timing state. */
extern volatile uint32_t g_ms_ticks;
extern volatile int systick_inited;
extern volatile uint32_t supervisor_last_tick;
extern volatile uint8_t  supervisor_due;
/**
 * @brief Handle the 1 ms SysTick interrupt.
 */
void SysTick_Handler(void);
/**
 * @brief Initialize SysTick for a 1 ms system tick.
 * @param sysclk_hz CPU clock frequency in Hz.
 * @param priority SysTick interrupt priority.
 * @return SysTick status.
 */
systick_status_t systick_init(uint32_t sysclk_hz, uint32_t priority);
/**
 * @brief Disable and deinitialize SysTick.
 */
void systick_deinit(void);
/**
 * @brief Get the monotonic millisecond tick.
 * @return Current millisecond count.
 */
uint32_t ms_get(void);
/**
 * @brief Delay using the SysTick millisecond timebase.
 * @param ms Delay in milliseconds.
 */
void systick_delay_ms(uint32_t ms);
/**
 * @brief Verify that the SysTick counter advances.
 * @param timeout_ms Maximum test time.
 * @return SysTick status.
 */
systick_status_t systick_selftest(uint32_t timeout_ms);
/**
 * @brief Application hook for SysTick diagnostics.
 * @param msg Diagnostic message.
 * @param code Error code.
 */
void systick_error_hook(const char *msg, int code);
#ifdef __cplusplus
}
#endif
#endif /* SYSTICK_H */
