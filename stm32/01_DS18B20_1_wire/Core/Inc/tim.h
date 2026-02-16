/* tim.h | Author: i fiksuari pas prog | Date: 17 February 2026 */

#ifndef INC_TIM_H_
#define INC_TIM_H_
#include"stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include<stdio.h>
#include<stdint.h>
#include<string.h>

void MX_TIM2_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* INC_TIM_H_ */
