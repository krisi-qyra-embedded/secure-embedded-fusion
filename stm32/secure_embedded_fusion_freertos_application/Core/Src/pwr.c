/* Author: i fiksuari pas prog
 * Documentation date: 2026-08-18
 */

#include "pwr.h"

/* Clears pending interrupt requests before sleep. */
void PWR_ClearAllPendingIRQs(void)
{
    /* Acknowledge every NVIC pending-clear bank. */
    /* 8 words -> covers up to 256 IRQs, safe for STM32F4 */
    for (uint32_t i = 0u; i < 8u; ++i) {
        NVIC->ICPR[i] = 0xFFFFFFFFu;
        (void)NVIC->ICPR[i];  
    }
}

/* Clears pending external interrupt events. */
void PWR_ClearAllEXTIPending(void)
{
    /* Write the EXTI pending bits and read back the register. */
    EXTI->PR = 0xFFFFFFFFu;
    (void)EXTI->PR;
}

/* Clears wakeup state and selects ordinary Cortex-M sleep. */
void PWR_PrepareForSleep(void)
{
    /* Mask interrupts while clearing pending NVIC and EXTI events. */
    __disable_irq();

    PWR_ClearAllPendingIRQs();
    PWR_ClearAllEXTIPending();

    /* Ensure SLEEPDEEP = 0 -> Sleep mode (not deep sleep/Stop) */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Ensure SEVONPEND = 0 -> only enabled interrupts/events wake on WFE */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

    __enable_irq();
}

/* Selects ordinary sleep without event-on-pending behavior. */
void sleep_mode_configure_once(void)
{
    /* Clear SLEEPDEEP and SEVONPEND in the system control register. */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}

/* Enters sleep until an interrupt wakes the core. */
void PWR_EnterSleep_WFI(void)
{
    /* Complete peripheral writes before executing WFI. */
    PWR_PrepareForSleep();

    __DSB();
    __ISB();

    __WFI();  
}

/* Enters sleep after clearing the stale event latch. */
void PWR_EnterSleep_WFE(void)
{
    /* Use SEV and the first WFE to consume an existing event. */
    PWR_PrepareForSleep();

    __DSB();
    __ISB();

    __SEV();
    __WFE();  
    __WFE();  

}

