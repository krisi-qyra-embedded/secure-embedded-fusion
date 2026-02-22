/* tim.h | Author: i fiksuari pas prog | Date: 22 February 2026 */
#ifndef INC_TIM_H_
#define INC_TIM_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TIM_STATUS_OK = 0,
    TIM_STATUS_BAD_PARAM,
    TIM_STATUS_INIT_FAIL,
    TIM_STATUS_START_FAIL,
    TIM_STATUS_STOP_FAIL
} tim_status_t;

extern TIM_HandleTypeDef htim3;

tim_status_t MX_TIM3_Init(void);

void delay_us(uint16_t us);
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIM_H_ */
