/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_
#include "stm32f4xx.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* 8 MHz HSE -> 192 MHz PLL VCO -> 96 MHz SYSCLK and 48 MHz PLLQ clock. */
#define HSE_CLOCK_HZ    (8000000UL)
#define SYSCLK_96MHZ    (96000000UL)
#define PLL_M           (8U)
#define PLL_N           (192U)
#define PLL_P           (2U)
#define PLL_Q           (4U)
void clock_96_mhz_init(void);
void clock_100_mhz_init(void);
#ifdef __cplusplus
}
#endif
#endif /* INC_CLOCK_H_ */
