/* tim.h | Author: i fiksuari pas prog | Date: 15 February 2026 */
#ifndef INC_TIM_H_
#define INC_TIM_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;

typedef enum
{
    TIM_STATUS_OK = 0,
    TIM_STATUS_BAD_PARAM,
    TIM_STATUS_INIT_FAIL,
    TIM_STATUS_START_FAIL,
    TIM_STATUS_STOP_FAIL
} tim_status_t;

tim_status_t MX_TIM2_Init(void);
tim_status_t tim2_start_oneshot_us(uint16_t delay_us);
tim_status_t tim2_stop_oneshot(void);
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_TIM_H_ */
