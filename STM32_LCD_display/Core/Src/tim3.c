/**
 * @file tim3.c
 * @brief TIM3 microsecond timebase and blocking delay helper.
 */
#include "tim3.h"
#include "lvgl.h"

TIM_HandleTypeDef htim3;
/**
 * @brief Initialize TIM3 as a free-running 1 MHz timer.
 * @return TIM3 status.
 */
tim_status_t MX_TIM3_Init(void)
{
   uint32_t tim_clk_hz;
   uint32_t pclk1_hz;
   __HAL_RCC_TIM3_CLK_ENABLE();
   htim3.Instance = TIM3;
   pclk1_hz = HAL_RCC_GetPCLK1Freq();
   /* APB timers run at 2x PCLK when the APB prescaler is not 1. */
   if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_HCLK_DIV1)
   {
       tim_clk_hz = pclk1_hz;
   }
   else
   {
       tim_clk_hz = pclk1_hz * 2U;
   }
   /* 1 MHz timer clock gives one counter step per microsecond. */
   htim3.Init.Prescaler         = (uint32_t)((tim_clk_hz / 1000000U) - 1U);
   htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
   htim3.Init.Period            = 0xFFFFU;
   htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
   {
       return TIM_STATUS_INIT_FAIL;
   }
   /* Keep TIM3 running so delay_us() only has to reset CNT. */
   if (HAL_TIM_Base_Start(&htim3) != HAL_OK)
   {
       return TIM_STATUS_START_FAIL;
   }
   return TIM_STATUS_OK;
}
/**
 * @brief Run a blocking microsecond delay using TIM3.
 * @param us Delay in microseconds.
 */
void delay_us(uint16_t us)
{
   if (us == 0U)
   {
       return;
   }
   /* Restart timing from zero for this delay. */
   __HAL_TIM_SET_COUNTER(&htim3, 0U);
   while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim3) < us)
   {
       /* Wait until the 1 MHz counter reaches the requested delay. */
   }
}
//void delay_ms(uint32_t ms)
//{
//    HAL_Delay(ms);
//}
