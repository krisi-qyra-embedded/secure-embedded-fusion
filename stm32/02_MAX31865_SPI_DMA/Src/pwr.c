 

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

 
/* Handles the PWR PrepareForSleep operation. */
void PWR_PrepareForSleep(void)
{
    /* Check the current status before proceeding. */
     
    __disable_irq();

    /* Clear pending NVIC IRQs and EXTI pending flags */
    PWR_ClearAllPendingIRQs();
    PWR_ClearAllEXTIPending();

     
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

     
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

    /* Re-enable interrupts so that new IRQs/events can wake the CPU */
    __enable_irq();
}

 
/* Initializes the resources used by sleep mode configure once. */
void sleep_mode_configure_once(void)
{
    /* Set up the required peripheral configuration. */
     
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

     
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}


 
/* Handles the PWR EnterSleep WFI operation. */
void PWR_EnterSleep_WFI(void)
{
    /* Check the current status before proceeding. */
    PWR_PrepareForSleep();

     
    __DSB();
    __ISB();

    __WFI();  
     
}


 
/* Handles the PWR EnterSleep WFE operation. */
void PWR_EnterSleep_WFE(void)
{
    /* Check the current status before proceeding. */
    PWR_PrepareForSleep();

    __DSB();
    __ISB();

     
    __SEV();
    __WFE();  
    __WFE(); /* waits for next event or interrupt */

     
}


