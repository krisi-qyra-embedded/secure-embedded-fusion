/**
 * @file tim.h
 * @brief TIM2 delay, one-shot and interrupt helper declarations.
 */
#ifndef INC_TIM_H_
#define INC_TIM_H_
#include"stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
/* Shared TIM2 HAL handle. */
extern TIM_HandleTypeDef htim2;
/** @brief TIM2 helper status. */
typedef enum
{
   TIM_STATUS_OK = 0,
   TIM_STATUS_BAD_PARAM,
   TIM_STATUS_INIT_FAIL,
   TIM_STATUS_START_FAIL,
   TIM_STATUS_STOP_FAIL
} tim_status_t;
/**
 * @brief Initialize TIM2.
 * @return Timer status.
 */
tim_status_t MX_TIM2_Init(void);
/**
 * @brief Start a TIM2 one-shot delay in microseconds.
 * @param delay_us Delay in microseconds.
 * @return Timer status.
 */
tim_status_t tim2_start_oneshot_us(uint16_t delay_us);
/**
 * @brief Stop the active TIM2 one-shot delay.
 * @return Timer status.
 */
tim_status_t tim2_stop_oneshot(void);
/**
 * @brief Apply a millisecond delay.
 * @param ms Delay in milliseconds.
 */
void delay_ms(uint32_t ms);
/**
 * @brief Configure TIM2 for the one-hertz timebase.
 */
void tim2_1hz_init(void);
#define SR_UIF     (1U << 0)
/**
 * @brief Enable the TIM2 one-hertz interrupt path.
 */
void tim2_1hz_interrupt_init(void);
/**
 * @brief Apply the fixed 3 ms TIM2 delay.
 */
void tim2_3ms_delay(void);
/**
 * @brief Apply a TIM2 millisecond delay.
 * @param ms Delay in milliseconds.
 */
void tim2_delay_ms(uint32_t ms);
/**
 * @brief Apply the fixed 69 ms TIM2 delay.
 */
void tim2_69ms_delay(void);
/**
 * @brief Apply the fixed 1 s TIM2 delay.
 */
void tim2_1s_delay(void);
void tim2_1hz_interrupt_init(void);
/**
 * @brief Check and clear the TIM2 one-second flag.
 * @return Non-zero when the flag was set.
 */
uint8_t tim2_1s_check_flag(void);
/**
 * @brief Initialize PA5 for the timer test path.
 */
void gpioa5_init(void);
/**
 * @brief Check the TIM3 3 ms flag.
 * @return Current flag state.
 */
uint8_t tim3_3ms_check_flag(void);
extern volatile uint8_t tim2_1s_flag; // Flag set by TIM2 ISR
extern volatile uint8_t tim3_3ms_flag;
#endif /* INC_TIM_H_ */
