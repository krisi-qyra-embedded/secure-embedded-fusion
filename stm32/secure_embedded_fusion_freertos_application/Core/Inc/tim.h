/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef INC_TIM_H_
#define INC_TIM_H_
#include"stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
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
void tim2_1hz_init(void);
#define SR_UIF     (1U << 0)
void tim2_1hz_interrupt_init(void);
void tim2_3ms_delay(void);
void tim2_delay_ms(uint32_t ms);
void tim2_69ms_delay(void);
void tim2_1s_delay(void);
void tim2_1hz_interrupt_init(void);
uint8_t tim2_1s_check_flag(void);
void gpioa5_init(void);
uint8_t tim3_3ms_check_flag(void);
extern volatile uint8_t tim2_1s_flag;  
extern volatile uint8_t tim3_3ms_flag;
#endif /* INC_TIM_H_ */
