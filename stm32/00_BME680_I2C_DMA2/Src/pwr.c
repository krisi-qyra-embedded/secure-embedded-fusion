/* pwr.c | Author: i fiksuari mbas prog | Date: 13 January 2026 */

#include "pwr.h"

/* Clear a safe number of NVIC pending registers (covers typical STM32F4 IRQs) */
void PWR_ClearAllPendingIRQs(void)
{
    /* 8 words -> covers up to 256 IRQs, safe for STM32F4 */
    for (uint32_t i = 0u; i < 8u; ++i) {
        NVIC->ICPR[i] = 0xFFFFFFFFu;
        (void)NVIC->ICPR[i]; 
    }
}


/* Handles the PWR ClearAllEXTIPending operation and returns the resulting status. */
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

    
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Ensure SEVONPEND = 0 -> only enabled interrupts/events wake on WFE */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

    
    __enable_irq();
}

/* configure Sleep behaviour once at system init or when changing power policy */
void sleep_mode_configure_once(void)
{
    /* Configure only the resources required by this module. */
    
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* only enabled interrupts/events wake on WFE */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}



/* Handles the PWR EnterSleep WFI operation and returns the resulting status. */
void PWR_EnterSleep_WFI(void)
{
    /* Check the current status before continuing with the operation. */
    PWR_PrepareForSleep();

    
    __DSB();
    __ISB();

    __WFI(); 
    
}



/* Handles the PWR EnterSleep WFE operation and returns the resulting status. */
void PWR_EnterSleep_WFE(void)
{
    /* Check the current status before continuing with the operation. */
    PWR_PrepareForSleep();

    __DSB();
    __ISB();

    
    __SEV();
    __WFE(); 
    __WFE(); 

    /* Execution resumes here after event/interrupt */
}


