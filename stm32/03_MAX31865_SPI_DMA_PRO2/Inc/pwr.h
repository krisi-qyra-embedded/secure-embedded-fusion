
#ifndef PWR_H_
#define PWR_H_

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

 
void PWR_PrepareForSleep(void);
void PWR_EnterSleep_WFI(void);
void PWR_EnterSleep_WFE(void);

 
void PWR_ClearAllPendingIRQs(void);
void PWR_ClearAllEXTIPending(void);
void sleep_mode_configure_once(void);


/* Sleep until next enabled IRQ/event. */
static inline void cpu_sleep_until_irq(void) {
    __DSB();
    __ISB();
    __WFI();
}


#ifdef __cplusplus
}
#endif

#endif /* PWR_H_ */
