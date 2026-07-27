/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include "tim3.h"

TIM_HandleTypeDef htim3;

/* Starts TIM3 as the microsecond OneWire delay timer. */
tim_status_t MX_TIM3_Init(void)
{
    /* Double the APB1 timer clock when the peripheral bus is prescaled. */
    uint32_t tim_clk_hz;
    uint32_t pclk1_hz;

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;

    pclk1_hz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_HCLK_DIV1)
    {
        tim_clk_hz = pclk1_hz;
    }
    else
    {
        tim_clk_hz = pclk1_hz * 2U;
    }

    /* 1 MHz timer tick: 1 counter count = 1 us. */
    htim3.Init.Prescaler         = (uint32_t)((tim_clk_hz / 1000000U) - 1U);
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 0xFFFFU;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        return TIM_STATUS_INIT_FAIL;
    }

    if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
    {
        return TIM_STATUS_START_FAIL;
    }

    return TIM_STATUS_OK;
}

/* Waits for the requested TIM3 microsecond interval. */
void delay_us(uint16_t us)
{
    /* Reset the counter before polling the requested count. */
    if (us == 0U)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) < us)
    {
    }
}

//void delay_ms(uint32_t ms)
//{
//    HAL_Delay(ms);
//}

