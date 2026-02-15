/* tim.c | Author: i fiksuari pas prog | Date: 15 February 2026 */
#include "tim.h"

TIM_HandleTypeDef htim2;

/* Initialize the required peripheral resources in MX_TIM2_Init. */
tim_status_t MX_TIM2_Init(void)
{
    /* Set up the required registers before starting the peripheral. */
    uint32_t tim_clk_hz;
    uint32_t pclk1_hz;

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;

    pclk1_hz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_HCLK_DIV1)
    {
        tim_clk_hz = pclk1_hz;
    }
    else
    {
        tim_clk_hz = pclk1_hz * 2U;
    }

    htim2.Init.Prescaler         = (uint32_t)((tim_clk_hz / 1000000U) - 1U);
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 100U - 1U;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        return TIM_STATUS_INIT_FAIL;
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    return TIM_STATUS_OK;
}

/* Process the tim2_start_oneshot_us operation. */
tim_status_t tim2_start_oneshot_us(uint16_t delay_us)
{
    /* Validate the current state before continuing. */
    if (delay_us == 0U)
    {
        return TIM_STATUS_BAD_PARAM;
    }

    if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
    {
        return TIM_STATUS_STOP_FAIL;
    }

    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_AUTORELOAD(&htim2, (uint32_t)delay_us - 1U);

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        return TIM_STATUS_START_FAIL;
    }

    return TIM_STATUS_OK;
}

/* Process the tim2_stop_oneshot operation. */
tim_status_t tim2_stop_oneshot(void)
{
    /* Validate the current state before continuing. */
    if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
    {
        return TIM_STATUS_STOP_FAIL;
    }

    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    return TIM_STATUS_OK;
}

/* Wait for the required timing in delay_ms. */
void delay_ms(uint32_t ms)
{
    /* Keep this wait bounded to avoid blocking indefinitely. */
    HAL_Delay(ms);
}

/* Handle interrupt events in TIM2_IRQHandler. */
void TIM2_IRQHandler(void)
{
    /* Clear the handled event and update the transfer state. */
    HAL_TIM_IRQHandler(&htim2);
}
