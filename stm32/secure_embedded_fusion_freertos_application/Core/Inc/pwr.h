/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#ifndef PWR_H_
#define PWR_H_

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public API */
void PWR_PrepareForSleep(void);
void PWR_EnterSleep_WFI(void);
void PWR_EnterSleep_WFE(void);

void PWR_ClearAllPendingIRQs(void);
void PWR_ClearAllEXTIPending(void);
void sleep_mode_configure_once(void);

/* Sleep until next enabled IRQ/event. Must be extremely lightweight:
   Typically implemented as: __DSB(); __ISB(); __WFI(); */
/* Waits for an interrupt during SPI polling. */
static inline void cpu_sleep_until_irq(void) {
    /* Order register accesses before sleeping the core. */
    __DSB();
    __ISB();
    __WFI();
}

#ifdef __cplusplus
}
#endif

#endif /* PWR_H_ */
