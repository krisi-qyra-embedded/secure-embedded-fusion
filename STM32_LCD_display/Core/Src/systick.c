/**
 * @file systick.c
 * @brief SysTick 1 ms timebase, delay and self-test helpers.
 */
#include "systick.h"
#include "iwdg.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "lvgl.h"


extern volatile uint8_t g_lvgl_tick_enabled;
volatile uint32_t g_ms_ticks;
volatile int systick_inited = 0;
volatile uint32_t supervisor_last_tick = 0;
volatile uint8_t  supervisor_due = 0;
/* Weak hook for SysTick diagnostics. */
__attribute__((weak)) void systick_error_hook(const char *msg, int code)
{
   (void) msg;
   (void) code;
}
extern void xPortSysTickHandler(void);
/* Update HAL/project ticks and forward to FreeRTOS. */
void SysTick_Handler(void)
    {
        g_ms_ticks++;
        HAL_IncTick();
        if (g_lvgl_tick_enabled != 0U) {
            lv_tick_inc(1U);
        }
    }


/* Calculate the 24-bit reload value for a 1 ms tick. */
static int compute_reload(uint32_t sysclk_hz, uint32_t *out_reload)
{
   /* Reject invalid input before calculating the reload value. */
   if (sysclk_hz == 0U || out_reload == NULL) return -1;
   uint32_t reload = sysclk_hz / 1000U;
   if (reload == 0U) return -1;
   /* SysTick LOAD stores ticks-per-period minus one. */
   reload -= 1U;
   /* LOAD is limited to 24 bits on Cortex-M. */
   if (reload > 0x00FFFFFFU) return -1;
   *out_reload = reload;
   return 0;
}
/**
 * @brief Initialize SysTick as a 1 ms system timebase.
 * @param sysclk_hz System clock frequency.
 * @param priority SysTick interrupt priority.
 * @return SysTick status.
 */
systick_status_t systick_init(uint32_t sysclk_hz, uint32_t priority)
{
   /* Do not silently reconfigure an active system timebase. */
   if (systick_inited) {
       systick_error_hook("systick_init: already initialized", SYSTICK_ERR_ALREADY_INITIALIZED);
       return SYSTICK_ERR_ALREADY_INITIALIZED;
   }
   uint32_t reload;
   if (compute_reload(sysclk_hz, &reload) != 0) {
       systick_error_hook("systick_init: invalid sysclk or reload overflow", SYSTICK_ERR_RELOAD_OVERFLOW);
       return SYSTICK_ERR_RELOAD_OVERFLOW;
   }
   /* Stop SysTick before touching LOAD/VAL. */
   SysTick->CTRL = 0U;
   /* Program one interrupt every millisecond. */
   SysTick->LOAD = reload;
   SysTick->VAL  = 0U;
   /* Complete register writes before enabling the timebase. */
   __DSB(); __ISB();
#if defined(NVIC_SetPriority) && (defined(SysTick_IRQn))
   /* Use requested priority when valid, otherwise clamp it. */
   if (priority <= (uint32_t)((1U << __NVIC_PRIO_BITS) - 1U)) {
       NVIC_SetPriority(SysTick_IRQn, (uint32_t)priority);
   } else {
       NVIC_SetPriority(SysTick_IRQn, (uint32_t)((1U << __NVIC_PRIO_BITS) - 1U));
   }
#endif
   /* Enable processor clock source, interrupt and counter. */
   SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                 | SysTick_CTRL_TICKINT_Msk
                 | SysTick_CTRL_ENABLE_Msk;
   /* Read back LOAD as a lightweight configuration check. */
   if ((SysTick->LOAD & 0x00FFFFFFU) != (reload & 0x00FFFFFFU)) {
       systick_error_hook("systick_init: LOAD readback mismatch", SYSTICK_ERR_RELOAD_OVERFLOW);
   }
   /* Publish initialized state only after setup is complete. */
   systick_inited = 1;
   g_ms_ticks = 0U;
   return SYSTICK_OK;
}
/**
 * @brief Disable SysTick and clear local tick state.
 */
void systick_deinit(void)
{
   SysTick->CTRL = 0U;
   SysTick->LOAD = 0U;
   SysTick->VAL  = 0U;
   systick_inited = 0;
   g_ms_ticks = 0U;
}
/**
 * @brief Get the current millisecond tick.
 * @return Milliseconds since SysTick initialization.
 */
uint32_t ms_get(void)
{
   return g_ms_ticks;
}
/**
 * @brief Wait for a bounded number of milliseconds.
 * @param ms Delay in milliseconds.
 */
void systick_delay_ms(uint32_t ms)
{
   /* Zero delay is a fast no-op. */
   if (ms == 0U) return;
   if (!systick_inited) {
       systick_error_hook("systick_delay_ms: not initialized", SYSTICK_ERR_NOT_INITIALIZED);
       return;
   }
   /* Unsigned subtraction keeps the delay safe across tick wrap. */
   uint32_t start = ms_get();
   while ((ms_get() - start) < ms) {
       /* Sleep between tick interrupts instead of busy-spinning. */
   	__DSB();
   	__ISB();
   	__WFI();
   }
}
/**
 * @brief Check that the SysTick counter is advancing.
 * @param timeout_ms Maximum wait time.
 * @return SysTick status.
 */
systick_status_t systick_selftest(uint32_t timeout_ms)
{
   /* Self-test requires an active SysTick timebase. */
   if (!systick_inited) return SYSTICK_ERR_NOT_INITIALIZED;
   /* One tick advance is enough to confirm the timebase is running. */
   uint32_t t0 = ms_get();
   uint32_t deadline = t0 + timeout_ms;
   while (ms_get() == t0) {
       if ((ms_get() - t0) >= timeout_ms) {
           systick_error_hook("systick_selftest: tick did not advance", SYSTICK_ERR_NOT_INITIALIZED);
           return SYSTICK_ERR_NOT_INITIALIZED;
       }
       __WFI();
   }
   return SYSTICK_OK;
}
