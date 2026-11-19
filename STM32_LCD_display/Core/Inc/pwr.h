/**
 * @file pwr.h
 * @brief Cortex-M sleep and pending-interrupt helper declarations.
 */
#ifndef PWR_H_
#define PWR_H_
#include "stm32f4xx.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief Prepare the MCU state before entering sleep.
 */
void PWR_PrepareForSleep(void);
/**
 * @brief Enter sleep using WFI.
 */
void PWR_EnterSleep_WFI(void);
/**
 * @brief Enter sleep using WFE.
 */
void PWR_EnterSleep_WFE(void);
/**
 * @brief Clear pending NVIC interrupts.
 */
void PWR_ClearAllPendingIRQs(void);
/**
 * @brief Clear pending EXTI lines.
 */
void PWR_ClearAllEXTIPending(void);
/**
 * @brief Configure the project sleep mode once during startup.
 */
void sleep_mode_configure_once(void);
/* Lightweight wait used when only the next enabled IRQ should wake the CPU. */
static inline void cpu_sleep_until_irq(void) {
   __DSB();
   __ISB();
   __WFI();
}
#ifdef __cplusplus
}
#endif
#endif /* PWR_H_ */
