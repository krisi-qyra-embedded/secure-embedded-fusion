/* clock.h | Author: i fiksuari pas prog | Date: 11 February 2026 */

#ifndef INC_CLOCK_H_
#define INC_CLOCK_H_

#include"stm32f4xx.h"

void clock_100_mhz_init(void);

#define PLL_M   4
#define PLL_N   192
#define PLL_P   4
#define PLL_Q   8

#endif /* INC_CLOCK_H_ */
