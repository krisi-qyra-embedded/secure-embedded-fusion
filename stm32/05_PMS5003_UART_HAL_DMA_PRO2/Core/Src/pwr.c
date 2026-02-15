/* pwr.c | Author: i fiksuari pas prog | Date: 15 February 2026 */

#include "pwr.h"

/* Clear a safe number of NVIC pending registers (covers typical STM32F4 IRQs) */
void PWR_ClearAllPendingIRQs(void)
{
    /* 8 words -> covers up to 256 IRQs, safe for STM32F4 */
    for (uint32_t i = 0u; i < 8u; ++i) {
        NVIC->ICPR[i] = 0xFFFFFFFFu;
        (void)NVIC->ICPR[i]; /* read-back to ensure write completes */
    }
}

/* Clear all EXTI pending flags (PR register) */
void PWR_ClearAllEXTIPending(void)
{
    /* Write 1 to pending bits to clear them */
    EXTI->PR = 0xFFFFFFFFu;
    (void)EXTI->PR;
}

/* Prepare: atomically clear pending interrupts/events and ensure Sleep (not DeepSleep) */
void PWR_PrepareForSleep(void)
{
    /* Make clearing atomic against interrupts */
    __disable_irq();

    /* Clear pending NVIC IRQs and EXTI pending flags */
    PWR_ClearAllPendingIRQs();
    PWR_ClearAllEXTIPending();

    /* Ensure SLEEPDEEP = 0 -> Sleep mode (not deep sleep/Stop) */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Ensure SEVONPEND = 0 -> only enabled interrupts/events wake on WFE */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

    /* Re-enable interrupts so that new IRQs/events can wake the CPU */
    __enable_irq();
}

/* configure Sleep behaviour once at system init or when changing power policy */
void sleep_mode_configure_once(void)
{
    /* ensure we use Sleep (not DeepSleep/STOP) */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* only enabled interrupts/events wake on WFE */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}


/* Enter Sleep using WFI */
void PWR_EnterSleep_WFI(void)
{
    /* Enter the wait state only after the pending work is handled. */
    PWR_PrepareForSleep();

    /* Ensure memory accesses complete and instruction pipeline flushed */
    __DSB();
    __ISB();

    __WFI(); /* Sleep until an enabled interrupt occurs */
    /* Execution resumes immediately after wake */
}


/* Enter Sleep using WFE (safe pattern) */
void PWR_EnterSleep_WFE(void)
{
    /* Enter the wait state only after the pending work is handled. */
    PWR_PrepareForSleep();

    __DSB();
    __ISB();

    /* Typical SEV/WFE handshake to avoid missing events */
    __SEV();
    __WFE(); /* consumes SEV */
    __WFE(); /* waits for next event or interrupt */

    /* Execution resumes here after event/interrupt */
}


