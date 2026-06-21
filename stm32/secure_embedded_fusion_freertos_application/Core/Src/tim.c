/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#include "tim.h"

#define TIM2EN                          (1U << 0)
#define TIM2_TIMCLK_HZ                 (96000000UL)
#define TIM2_3MS_TICK_HZ               (10000UL)
#define TIM2_3MS_TICKS                 (30UL)
#define TIM2_1S_BASE_TICK_HZ           (1000UL)
#define TIM2_1S_TICKS                  (1000UL)
#define TIM2_1HZ_INT_TICK_HZ           (10000UL)
#define TIM2_1HZ_INT_TICKS             (10000UL)

volatile uint8_t tim2_1s_flag = 0;
TIM_HandleTypeDef htim2;

/* Waits for a configured three-millisecond TIM2 interval. */
void tim2_3ms_delay(void)
{
    /* Clear the update flag before starting the blocking timer wait. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     
    (void)RCC->APB1ENR;

    /* TIM2 input clock = 96 MHz -> 10 kHz timer tick (0.1 ms) */
    TIM2->PSC = (uint32_t)((TIM2_TIMCLK_HZ / TIM2_3MS_TICK_HZ) - 1UL);  
    TIM2->ARR = (uint32_t)(TIM2_3MS_TICKS - 1UL);                       /* 30 ticks = 3 ms */
    TIM2->CNT = 0U;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->CR1 |= TIM_CR1_CEN;

    while ((TIM2->SR & TIM_SR_UIF) == 0U) {
        __asm__ volatile ("nop");
    }

    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->CR1 &= ~TIM_CR1_CEN;
}

/**
 * Blocking delay using TIM2.
 * @param ms  Number of milliseconds to delay (0 -> returns immediately)
 *
 * Notes:
 * - Assumes TIM2 timer input clock is 96 MHz.
 * - Chooses PSC so PSC <= 0xFFFF.
 * - ARR is treated as 32-bit (TIM2 on STM32F4).
 * - This is a blocking busy-wait that waits for the update event (UIF).
 */
/* Uses TIM2 to wait for a millisecond duration. */
void tim2_delay_ms(uint32_t ms)
{
    /* Split intervals that exceed the timer's auto-reload capacity. */
    uint64_t base_presc;
    uint32_t presc_reg;
    uint64_t presc_plus1;
    uint64_t tick_freq;
    uint64_t ticks_needed;
    const uint64_t ARR_MAX = 0xFFFFFFFFULL;
    uint64_t remaining_ticks;

    if (ms == 0U) {
        return;
    }

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;

    base_presc = (uint64_t)TIM2_TIMCLK_HZ / 1000ULL;
    if (base_presc < 1ULL) {
        base_presc = 1ULL;
    }

    if (base_presc <= 0xFFFFULL) {
        presc_plus1 = base_presc;
        presc_reg = (uint32_t)(presc_plus1 - 1ULL);
    } else {
        uint64_t factor = (base_presc + 0xFFFFULL - 1ULL) / 0xFFFFULL;
        presc_plus1 = (base_presc + factor - 1ULL) / factor;

        if (presc_plus1 < 1ULL) {
            presc_plus1 = 1ULL;
        }
        if (presc_plus1 > 0xFFFFULL) {
            presc_plus1 = 0xFFFFULL;
        }

        presc_reg = (uint32_t)(presc_plus1 - 1ULL);
    }

    tick_freq = (uint64_t)TIM2_TIMCLK_HZ / presc_plus1;
    ticks_needed = (tick_freq * (uint64_t)ms + 999ULL) / 1000ULL;
    remaining_ticks = ticks_needed;

    while (remaining_ticks > 0ULL) {
        uint64_t this_chunk = (remaining_ticks <= ARR_MAX) ? remaining_ticks : ARR_MAX;

        TIM2->PSC = presc_reg & 0xFFFFU;
        TIM2->ARR = (uint32_t)(this_chunk - 1ULL);
        TIM2->CNT = 0U;
        TIM2->SR &= ~TIM_SR_UIF;
        __DSB();
        __ISB();
        TIM2->CR1 |= TIM_CR1_CEN;

        while ((TIM2->SR & TIM_SR_UIF) == 0U) {
            __asm__ volatile ("nop");
        }

        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        remaining_ticks -= this_chunk;
    }
}

/* Configures TIM2 for a one-second interval. */
void tim2_1s_init(void)
{
    /* Leave update interrupts disabled until the start routine arms them. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;

    /* 96 MHz / 96000 = 1 kHz tick, 1000 ticks = 1 second */
    TIM2->PSC = (uint32_t)((TIM2_TIMCLK_HZ / TIM2_1S_BASE_TICK_HZ) - 1UL);  
    TIM2->ARR = (uint32_t)(TIM2_1S_TICKS - 1UL);                              
    TIM2->CNT = 0U;

    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER &= ~TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM2_IRQn);
}

/* start timer for one-shot 1s; safe to call repeatedly */
/* Starts the one-second TIM2 interval. */
void tim2_1s_start(void)
{
    /* Clear the software flag and pending update before enabling the timer. */
    tim2_1s_flag = 0U;
    TIM2->CNT = 0U;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* Stops TIM2 and its update interrupt. */
void tim2_1s_stop(void)
{
    /* Clear any pending update after disabling counting. */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM2->SR &= ~TIM_SR_UIF;
}

/* Configures PA5 as a push-pull output. */
void gpioa5_init(void)
{
    /* Select output mode and remove the pin's pull resistors. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(3U << (5U * 2U));
    GPIOA->MODER |=  (1U << (5U * 2U));

    GPIOA->OTYPER &= ~(1U << 5U);
    GPIOA->OSPEEDR |= (3U << (5U * 2U));
    GPIOA->PUPDR &= ~(3U << (5U * 2U));
}

/* Starts periodic one-hertz TIM2 interrupts. */
void tim2_1hz_interrupt_init(void)
{
    /* Program the prescaler and period before enabling update interrupts. */
    RCC->APB1ENR |= TIM2EN;
    (void)RCC->APB1ENR;

    /* 96 MHz / 9600 = 10 kHz tick, 10000 ticks = 1 second */
    TIM2->PSC = (uint32_t)((TIM2_TIMCLK_HZ / TIM2_1HZ_INT_TICK_HZ) - 1UL);  
    TIM2->ARR = (uint32_t)(TIM2_1HZ_INT_TICKS - 1UL);                        
    TIM2->CNT = 0U;
    TIM2->CR1 |= TIM_CR1_CEN;

    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
}

/* Configures the HAL TIM2 handle with a microsecond counter. */
tim_status_t MX_TIM2_Init(void)
{
    /* Account for the doubled timer clock when APB1 is prescaled. */
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

/* Arms a TIM2 update interrupt for the requested interval. */
tim_status_t tim2_start_oneshot_us(uint16_t delay_us)
{
    /* Stop the prior timer and clear its flag before changing ARR. */
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

/* Stops the TIM2 interval and clears its update flag. */
tim_status_t tim2_stop_oneshot(void)
{
    /* Return a stop failure before clearing the interrupt state. */
    if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
    {
        return TIM_STATUS_STOP_FAIL;
    }

    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    return TIM_STATUS_OK;
}

//void delay_ms(uint32_t ms)
//{
//    HAL_Delay(ms);
//}

//void TIM2_IRQHandler(void)
//{
//    HAL_TIM_IRQHandler(&htim2);
//}

