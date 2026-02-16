/* tim.c | Author: i fiksuari pas prog | Date: 17 February 2026 */
#include "tim.h"

#define TIM2EN (1U << 0)
#define GPIOAEN     (1U << 0)
TIM_HandleTypeDef htim2;

/* Initializes the MX TIM2 Init resources. */
void MX_TIM2_Init(void){
    __HAL_RCC_TIM2_CLK_ENABLE();  // Enable clock for TIM2

    htim2.Instance = TIM2;                          // Use TIM2
    htim2.Init.Prescaler = 96 - 1;                  // Set prescaler to get a 1 MHz timer (assuming a 48 MHz clock)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;    // Count up
    htim2.Init.Period = 0xFFFFFFFF;                 // Max period (so it can run indefinitely)
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        // Initialization Error
        //Error_Handler();
    	return -1;
    }
    HAL_TIM_Base_Start(&htim2);  // Start TIM2
}

/* Implements the delay us operation. */
void delay_us(uint32_t us){
    // Microsecond delay using a timer
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Assuming htim2 is your initialized timer
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

/* Implements the delay ms operation. */
void delay_ms(uint32_t ms)
{
    /* Wait the requested number of milliseconds. */
	HAL_Delay(ms);
}

