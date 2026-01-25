/* tim.h | Author: i fiksuari pas prog | Date: 25 January 2026 */

#ifndef INC_TIM_H_
#define INC_TIM_H_
#include"stm32f4xx.h"

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
extern volatile uint8_t tim2_1s_flag; // Flag set by TIM2 ISR
extern volatile uint8_t tim3_3ms_flag;
#endif  
