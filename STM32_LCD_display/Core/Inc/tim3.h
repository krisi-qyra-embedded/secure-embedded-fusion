/**
 * @file tim3.h
 * @brief TIM3 microsecond timebase and blocking delay declarations.
 */
#ifndef INC_TIM3_H_
#define INC_TIM3_H_
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "tim.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Shared TIM3 HAL handle. */
extern TIM_HandleTypeDef htim3;
/**
 * @brief Initialize TIM3 for the microsecond delay path.
 * @return Timer status.
 */
tim_status_t MX_TIM3_Init(void);
/**
 * @brief Apply a blocking microsecond delay.
 * @param us Delay in microseconds.
 */
void delay_us(uint16_t us);
//void delay_ms(uint32_t ms);
#ifdef __cplusplus
}
#endif
#endif /* INC_TIM_H_ */
