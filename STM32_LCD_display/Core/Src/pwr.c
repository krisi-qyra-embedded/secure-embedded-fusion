/**
 * @file pwr.c
 * @brief Cortex-M sleep and pending interrupt helpers.
 */
#include "pwr.h"
/**
 * @brief Clear pending NVIC interrupts.
 */
void PWR_ClearAllPendingIRQs(void)
{
   /* Eight ICPR words cover the STM32F4 interrupt range. */
   for (uint32_t i = 0u; i < 8u; ++i) {
       NVIC->ICPR[i] = 0xFFFFFFFFu;
       (void)NVIC->ICPR[i]; /* Force the write to complete. */
   }
}
/**
 * @brief Clear pending EXTI flags.
 */
void PWR_ClearAllEXTIPending(void)
{
   /* EXTI pending bits are cleared by writing 1. */
   EXTI->PR = 0xFFFFFFFFu;
   (void)EXTI->PR;
}
/**
 * @brief Prepare the CPU for normal Sleep mode.
 */
void PWR_PrepareForSleep(void)
{
   /* Keep pending-state cleanup atomic. */
   __disable_irq();
   PWR_ClearAllPendingIRQs();
   PWR_ClearAllEXTIPending();
   /* Use Sleep mode, not DeepSleep/STOP. */
   SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
   /* Pending interrupts alone should not generate WFE events. */
   SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
   /* New interrupts can now wake the CPU. */
   __enable_irq();
}
/**
 * @brief Configure the default Sleep policy.
 */
void sleep_mode_configure_once(void)
{
   /* Stay in normal Sleep mode. */
   SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
   /* WFE wakes from real enabled events/interrupts. */
   SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;
}
/**
 * @brief Enter Sleep mode using WFI.
 */
void PWR_EnterSleep_WFI(void)
{
   PWR_PrepareForSleep();
   /* Complete memory/register writes before sleeping. */
   __DSB();
   __ISB();
   __WFI(); /* Resume on the next enabled interrupt. */
}
/**
 * @brief Enter Sleep mode using WFE.
 */
void PWR_EnterSleep_WFE(void)
{
   PWR_PrepareForSleep();
   __DSB();
   __ISB();
   /* Clear any stale event, then wait for the next one. */
   __SEV();
   __WFE(); /* Consume the local SEV event. */
   __WFE(); /* Wait for the next event or interrupt. */
}
