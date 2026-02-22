/* pwr.c | Author: i fiksuari pas prog | Date: 22 February 2026 */
#include "pwr.h"

/* Handles the PWR ClearAllPendingIRQs interrupt path. */
void PWR_ClearAllPendingIRQs(void)
{
    /* Clear pending interrupt state before entering sleep. */
    for (uint32_t i = 0u; i < 8u; ++i) {
        NVIC->ICPR[i] = 0xFFFFFFFFu;
        (void)NVIC->ICPR[i];
    }
}

/* Implements the PWR ClearAllEXTIPending operation. */
void PWR_ClearAllEXTIPending(void)
{
    /* Clear EXTI pending flags that could wake the MCU early. */
    EXTI->PR = 0xFFFFFFFFu;
    (void)EXTI->PR;
}

/* Implements the PWR PrepareForSleep operation. */
void PWR_PrepareForSleep(void)
{
    /* Prepare the interrupt and power state before CPU sleep. */
    __disable_irq();

    PWR_ClearAllPendingIRQs();
    PWR_ClearAllEXTIPending();

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

    __enable_irq();
}

/* Initializes the sleep mode configure once resources. */
void sleep_mode_configure_once(void)
{
    /* Initialize the low-power configuration once. */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}

/* Implements the PWR EnterSleep WFI operation. */
void PWR_EnterSleep_WFI(void)
{
    /* Enter sleep until the next interrupt. */
    PWR_PrepareForSleep();
    __DSB();
    __ISB();
    __WFI();
}

/* Implements the PWR EnterSleep WFE operation. */
void PWR_EnterSleep_WFE(void)
{
    /* Enter sleep until an event wakes the core. */
    PWR_PrepareForSleep();
    __DSB();
    __ISB();
    __SEV();
    __WFE();
    __WFE();
}
