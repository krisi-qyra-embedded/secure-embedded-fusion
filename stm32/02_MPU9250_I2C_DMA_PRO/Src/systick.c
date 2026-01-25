/* systick.c | Author: i fiksuari pas prog | Date: 25 January 2026 */
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
 
/* Handles the systick error hook operation. */
__attribute__((weak)) void systick_error_hook(const char *msg, int code)
{
    /* Check the operation status before continuing. */
    (void) msg;
    (void) code;
     
}

 
/* Handles the sys tick handler operation. */
void SysTick_Handler(void)
{
    /* Check the operation status before continuing. */
    g_ms_ticks++;
    if ((g_ms_ticks - supervisor_last_tick) >= SUPERVISOR_PERIOD_MS) {
        supervisor_last_tick = g_ms_ticks;
        supervisor_due = 1;
    }
}

 
/* Handles the compute reload operation. */
static int compute_reload(uint32_t sysclk_hz, uint32_t *out_reload)
{
    /* Check the operation status before continuing. */
    if (sysclk_hz == 0U || out_reload == NULL) return -1;
    uint32_t reload = sysclk_hz / 1000U;
    if (reload == 0U) return -1;
    reload -= 1U; /* SysTick LOAD = ticks-1 */
    if (reload > 0x00FFFFFFU) return -1;
    *out_reload = reload;
    return 0;
}

/* Configures SysTick to provide the application millisecond counter. */
systick_status_t systick_init(uint32_t sysclk_hz, uint32_t priority)
{
    /* Validate the reload value before enabling the tick. */
    if (systick_inited) {
        systick_error_hook("systick_init: already initialized", SYSTICK_ERR_ALREADY_INITIALIZED);
        return SYSTICK_ERR_ALREADY_INITIALIZED;
    }

    uint32_t reload;
    if (compute_reload(sysclk_hz, &reload) != 0) {
        systick_error_hook("systick_init: invalid sysclk or reload overflow", SYSTICK_ERR_RELOAD_OVERFLOW);
        return SYSTICK_ERR_RELOAD_OVERFLOW;
    }

    /* Disable SysTick while configuring */
    SysTick->CTRL = 0U;

     
    SysTick->LOAD = reload;
    SysTick->VAL  = 0U;
    __DSB(); __ISB();

    /* Set SysTick priority if possible (CMSIS helper). Validate priority */
#if defined(NVIC_SetPriority) && (defined(SysTick_IRQn))
    /* NVIC_SetPriority accepts 0..(1<<__NVIC_PRIO_BITS)-1 ideally; we do not validate range here but we catch */
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

/* Initializes systick deinit for the requested operation. */
void systick_deinit(void)
{
    /* Validate required parameters before applying the setup. */
    /* disable SysTick */
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;
    systick_inited = 0;
    g_ms_ticks = 0U;
}

 
/* Handles the ms get operation. */
uint32_t ms_get(void)
{
    /* Check the operation status before continuing. */
    /* 32-bit read on Cortex-M is atomic; no irq masking required. */
    return g_ms_ticks;
}

 


/* Provides the delay required by systick delay ms. */
void systick_delay_ms(uint32_t ms)
{
    /* Keep the delay within the configured timing interval. */
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

 

/* Checks that the SysTick counter progresses as expected. */
systick_status_t systick_selftest(uint32_t timeout_ms)
{
    /* Use a bounded interval for the timer self-test. */
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

