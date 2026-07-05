/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#ifndef INC_TIM3_H_
#define INC_TIM3_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif


extern TIM_HandleTypeDef htim3;

tim_status_t MX_TIM3_Init(void);

void delay_us(uint16_t us);
//void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIM_H_ */
