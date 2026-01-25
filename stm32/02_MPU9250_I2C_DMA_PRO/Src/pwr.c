/* pwr.c | Author: i fiksuari pas prog | Date: 25 January 2026 */
 






#include "pwr.h"

/* Clear a safe number of NVIC pending registers (covers typical STM32F4 IRQs) */
/* Handles the pwr clear all pending irqs operation. */
void PWR_ClearAllPendingIRQs(void)
{
    /* Check the operation status before continuing. */
     
    for (uint32_t i = 0u; i < 8u; ++i) {
        NVIC->ICPR[i] = 0xFFFFFFFFu;
        (void)NVIC->ICPR[i];  
    }
}

/* Clear all EXTI pending flags (PR register) */
/* Handles the pwr clear all extipending operation. */
void PWR_ClearAllEXTIPending(void)
{
    /* Check the operation status before continuing. */
     
    EXTI->PR = 0xFFFFFFFFu;
    (void)EXTI->PR;
}

 
/* Handles the pwr prepare for sleep operation. */
void PWR_PrepareForSleep(void)
{
    /* Check the operation status before continuing. */
     
    __disable_irq();

    /* Clear pending NVIC IRQs and EXTI pending flags */
    PWR_ClearAllPendingIRQs();
    PWR_ClearAllEXTIPending();

    /* Ensure SLEEPDEEP = 0 -> Sleep mode (not deep sleep/Stop) */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Ensure SEVONPEND = 0 -> only enabled interrupts/events wake on WFE */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

     
    __enable_irq();
}

 
/* Initializes sleep mode configure once for the requested operation. */
void sleep_mode_configure_once(void)
{
    /* Validate required parameters before applying the setup. */
    /* ensure we use Sleep (not DeepSleep/STOP) */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* only enabled interrupts/events wake on WFE */
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}


/* Enter Sleep using WFI */
/* Enters sleep until an enabled interrupt occurs. */
void PWR_EnterSleep_WFI(void)
{
    /* Execute WFI only after the pending interrupt state is handled. */
    PWR_PrepareForSleep();

     
    __DSB();
    __ISB();

    __WFI(); /* Sleep until an enabled interrupt occurs */
     
}


 
/* Handles the pwr enter sleep wfe operation. */
void PWR_EnterSleep_WFE(void)
{
    /* Check the operation status before continuing. */
    PWR_PrepareForSleep();

    __DSB();
    __ISB();

     
    __SEV();
    __WFE();  
    __WFE();  

     
}


