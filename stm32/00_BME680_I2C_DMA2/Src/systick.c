/* systick.c | Author: i fiksuari mbas prog | Date: 13 January 2026 */
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

/* Handles the systick error hook operation and returns the resulting status. */
__attribute__((weak)) void systick_error_hook(const char *msg, int code)
{
    /* Check the current status before continuing with the operation. */
    (void) msg;
    (void) code;
    /* Default: no-op. Application may implement printing/logging here. */
}


/* Handles the interrupt events associated with SysTick Handler. */
void SysTick_Handler(void)
{
    /* Clear handled flags and update the transfer state. */
    g_ms_ticks++;
    if ((g_ms_ticks - supervisor_last_tick) >= SUPERVISOR_PERIOD_MS) {
        supervisor_last_tick = g_ms_ticks;
        supervisor_due = 1;
    }
}


/* Handles the compute reload operation and returns the resulting status. */
static int compute_reload(uint32_t sysclk_hz, uint32_t *out_reload)
{
    /* Check the current status before continuing with the operation. */
    if (sysclk_hz == 0U || out_reload == NULL) return -1;
    uint32_t reload = sysclk_hz / 1000U;
    if (reload == 0U) return -1;
    reload -= 1U; 
    if (reload > 0x00FFFFFFU) return -1;
    *out_reload = reload;
    return 0;
}

/* Configures systick init for normal operation. */
systick_status_t systick_init(uint32_t sysclk_hz, uint32_t priority)
{
    /* Configure only the resources required by this module. */
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
    
    if (priority <= (uint32_t)((1U << __NVIC_PRIO_BITS) - 1U)) {
        NVIC_SetPriority(SysTick_IRQn, (uint32_t)priority);
    } else {
        
        NVIC_SetPriority(SysTick_IRQn, (uint32_t)((1U << __NVIC_PRIO_BITS) - 1U));
    }
#endif

    /* Enable SysTick: use processor clock, enable IRQ, enable counter */
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

/* Configures systick deinit for normal operation. */
void systick_deinit(void)
{
    /* Configure only the resources required by this module. */
    
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;
    systick_inited = 0;
    g_ms_ticks = 0U;
}


/* Reads ms get and returns the resulting status. */
uint32_t ms_get(void)
{
    /* 32-bit read on Cortex-M is atomic; no irq masking required. */
    return g_ms_ticks;
}

/* Blocking millisecond delay using ms_get(). */
void systick_delay_ms(uint32_t ms)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    if (ms == 0U) return;
    if (!systick_inited) {
        systick_error_hook("systick_delay_ms: not initialized", SYSTICK_ERR_NOT_INITIALIZED);
        return;
    }

    uint32_t start = ms_get();
    while ((ms_get() - start) < ms) {
        /* Wait for interrupt to reduce power / busy-looping */
    	__DSB();
    	__ISB();
    	__WFI();
    }
}

/* Simple self-test: make sure tick advances within timeout_ms. */
systick_status_t systick_selftest(uint32_t timeout_ms)
{
    /* Check the current status before continuing with the operation. */
    if (!systick_inited) return SYSTICK_ERR_NOT_INITIALIZED;

    uint32_t t0 = ms_get();
    uint32_t deadline = t0 + timeout_ms;
    /* wait until tick increases at least 1 ms or timeout */
    while (ms_get() == t0) {
        if ((ms_get() - t0) >= timeout_ms) {
            systick_error_hook("systick_selftest: tick did not advance", SYSTICK_ERR_NOT_INITIALIZED);
            return SYSTICK_ERR_NOT_INITIALIZED;
        }
        __WFI();
    }
    return SYSTICK_OK;
}

