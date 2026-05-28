/* Author: i fiksuari pas prog
 * Documentation date: 2026-05-28
 */

/*
 * systick.c — robust SysTick 1 ms timebase helper for STM32/CMSIS
 *
 * Features:
 *  - Defensive init with argument validation and reload overflow check.
 *  - Init idempotence detection.
 *  - Optional IRQ priority set for SysTick.
 *  - Small self-test helper to confirm tick advances.
 *  - Blocking delay using ms_get() with __WFI to reduce power while waiting.
 *  - Weak error hook for application diagnostics.
 *
 * Usage:
 *  - Call systick_init(SystemCoreClock, some_priority) early in main()
 *  - Use ms_get() to timestamp events (hb_spi_ts = ms_get()).
 *  - Use systick_delay_ms() for bounded waits (not for long-running tasks).
 */

#include "systick.h"
#include "iwdg.h"
#include "stm32f4xx.h"   /* CMSIS device header */
#include <stdint.h>

volatile uint32_t g_ms_ticks;
volatile int systick_inited = 0;
volatile uint32_t supervisor_last_tick = 0;
volatile uint8_t  supervisor_due = 0;
/* Provides the overridable SysTick diagnostic hook. */
__attribute__((weak)) void systick_error_hook(const char *msg, int code)
{
    /* The default weak hook discards the diagnostic arguments. */
    (void) msg;
    (void) code;
}

extern void xPortSysTickHandler(void);

/* Services the millisecond timebase interrupt. */
void SysTick_Handler(void)
{
    /* Advance the software tick used by peripheral timeout checks. */
    g_ms_ticks++;

    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

/* Calculates a one-millisecond SysTick reload value. */
static int compute_reload(uint32_t sysclk_hz, uint32_t *out_reload)
{
    /* Reject a zero clock and reload values outside the 24-bit range. */
    if (sysclk_hz == 0U || out_reload == NULL) return -1;
    uint32_t reload = sysclk_hz / 1000U;
    if (reload == 0U) return -1;
    reload -= 1U; /* SysTick LOAD = ticks-1 */
    if (reload > 0x00FFFFFFU) return -1;
    *out_reload = reload;
    return 0;
}

/* Starts the millisecond SysTick timebase. */
systick_status_t systick_init(uint32_t sysclk_hz, uint32_t priority)
{
    /* Clear the counter before enabling its clock and interrupt. */
    if (systick_inited) {
        systick_error_hook("systick_init: already initialized", SYSTICK_ERR_ALREADY_INITIALIZED);
        return SYSTICK_ERR_ALREADY_INITIALIZED;
    }

    uint32_t reload;
    if (compute_reload(sysclk_hz, &reload) != 0) {
        systick_error_hook("systick_init: invalid sysclk or reload overflow", SYSTICK_ERR_RELOAD_OVERFLOW);
        return SYSTICK_ERR_RELOAD_OVERFLOW;
    }

    SysTick->CTRL = 0U;

    SysTick->LOAD = reload;
    SysTick->VAL  = 0U;
    __DSB(); __ISB();

    /* Set SysTick priority if possible (CMSIS helper). Validate priority */
#if defined(NVIC_SetPriority) && (defined(SysTick_IRQn))
    /* NVIC_SetPriority accepts 0..(1<<__NVIC_PRIO_BITS)-1 ideally; we do not validate range here
       but we catch obviously large values. */
    if (priority <= (uint32_t)((1U << __NVIC_PRIO_BITS) - 1U)) {
        NVIC_SetPriority(SysTick_IRQn, (uint32_t)priority);
    } else {
        NVIC_SetPriority(SysTick_IRQn, (uint32_t)((1U << __NVIC_PRIO_BITS) - 1U));
    }
#endif

    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;

    if ((SysTick->LOAD & 0x00FFFFFFU) != (reload & 0x00FFFFFFU)) {
        systick_error_hook("systick_init: LOAD readback mismatch", SYSTICK_ERR_RELOAD_OVERFLOW);
    }

    systick_inited = 1;
    g_ms_ticks = 0U;  

    return SYSTICK_OK;
}

/* Stops SysTick and clears its initialization state. */
void systick_deinit(void)
{
    /* Reset the reload and current-count registers with the software tick. */
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;
    systick_inited = 0;
    g_ms_ticks = 0U;
}

/* Returns the software millisecond counter. */
uint32_t ms_get(void)
{
    /* Read the counter updated by the SysTick interrupt. */
    /* 32-bit read on Cortex-M is atomic; no irq masking required.
       If you run on a platform where this isn't true, protect externally. */
    return g_ms_ticks;
}

/* Waits for a duration using SysTick milliseconds. */
void systick_delay_ms(uint32_t ms)
{
    /* Subtract the start tick so ordinary counter wrap is handled. */
    if (ms == 0U) return;
    if (!systick_inited) {
        systick_error_hook("systick_delay_ms: not initialized", SYSTICK_ERR_NOT_INITIALIZED);
        return;
    }

    uint32_t start = ms_get();
    while ((ms_get() - start) < ms) {
    	__DSB();
    	__ISB();
    	__WFI();
    }
}

/* Checks that the SysTick counter advances. */
systick_status_t systick_selftest(uint32_t timeout_ms)
{
    /* Wait for a changed tick using the supplied timeout check. */
    if (!systick_inited) return SYSTICK_ERR_NOT_INITIALIZED;

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

