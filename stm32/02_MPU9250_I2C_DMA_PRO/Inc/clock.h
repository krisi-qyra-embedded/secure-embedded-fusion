/* clock.h | Author: i fiksuari pas prog | Date: 25 January 2026 */
/* clock.h 96 MHz clock configuration for STM32F4xx using 8 MHz HSE */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_

#include "stm32f4xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PLL setup for 96 MHz SYSCLK from 8 MHz HSE: VCO_IN = HSE / PLL_M = 8 / 8 = 1 MHz VCO_OUT = VCO_IN * PLL_N = */
#define HSE_CLOCK_HZ    (8000000UL)
#define SYSCLK_96MHZ    (96000000UL)

#define PLL_M           (8U)
#define PLL_N           (192U)
#define PLL_P           (2U)
#define PLL_Q           (4U)

 
void clock_96_mhz_init(void);

/* Backward-compatible wrapper for existing code paths */
void clock_100_mhz_init(void);

#ifdef __cplusplus
}
#endif

#endif  
