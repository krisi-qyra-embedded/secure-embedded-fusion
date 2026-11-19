/**
 * @file tim.c
 * @brief TIM2 delay, one-shot and interrupt helpers.
 */
#include "tim.h"
#include "lvgl.h"

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
/**
 * @brief Run a blocking 3 ms TIM2 delay.
 */
void tim2_3ms_delay(void)
{
   /* Build a 10 kHz timer tick, then wait for 30 counts. */
   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
   (void)RCC->APB1ENR;
   TIM2->PSC = (uint32_t)((TIM2_TIMCLK_HZ / TIM2_3MS_TICK_HZ) - 1UL);
   TIM2->ARR = (uint32_t)(TIM2_3MS_TICKS - 1UL);
   TIM2->CNT = 0U;
   TIM2->SR &= ~TIM_SR_UIF;
   TIM2->CR1 |= TIM_CR1_CEN;
   /* Poll the update flag for the end of the delay. */
   while ((TIM2->SR & TIM_SR_UIF) == 0U) {
       __asm__ volatile ("nop");
   }
   TIM2->SR &= ~TIM_SR_UIF;
   TIM2->CR1 &= ~TIM_CR1_CEN;
}
/**
 * @brief Run a blocking TIM2 millisecond delay.
 * @param ms Delay in milliseconds.
 */
void tim2_delay_ms(uint32_t ms)
{
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
   /* Start from a 1 ms timer base and keep PSC inside 16 bits. */
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
   /* Convert the requested delay to timer ticks. */
   tick_freq = (uint64_t)TIM2_TIMCLK_HZ / presc_plus1;
   ticks_needed = (tick_freq * (uint64_t)ms + 999ULL) / 1000ULL;
   remaining_ticks = ticks_needed;
   /* Split very long delays across the 32-bit ARR range. */
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
/**
 * @brief Configure TIM2 for a one-second update period.
 */
void tim2_1s_init(void)
{
   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
   (void)RCC->APB1ENR;
   /* 1 kHz timer tick with 1000 counts gives one second. */
   TIM2->PSC = (uint32_t)((TIM2_TIMCLK_HZ / TIM2_1S_BASE_TICK_HZ) - 1UL);
   TIM2->ARR = (uint32_t)(TIM2_1S_TICKS - 1UL);
   TIM2->CNT = 0U;
   TIM2->SR &= ~TIM_SR_UIF;
   TIM2->DIER &= ~TIM_DIER_UIE;
   NVIC_EnableIRQ(TIM2_IRQn);
}
/**
 * @brief Start the one-second TIM2 one-shot.
 */
void tim2_1s_start(void)
{
   /* Restart from a clean counter and update flag. */
   tim2_1s_flag = 0U;
   TIM2->CNT = 0U;
   TIM2->SR &= ~TIM_SR_UIF;
   TIM2->DIER |= TIM_DIER_UIE;
   TIM2->CR1 |= TIM_CR1_CEN;
}
/**
 * @brief Stop the one-second TIM2 one-shot.
 */
void tim2_1s_stop(void)
{
   TIM2->CR1 &= ~TIM_CR1_CEN;
   TIM2->DIER &= ~TIM_DIER_UIE;
   TIM2->SR &= ~TIM_SR_UIF;
}
/**
 * @brief Configure PA5 as a GPIO output.
 */
void gpioa5_init(void)
{
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
   GPIOA->MODER &= ~(3U << (5U * 2U));
   GPIOA->MODER |=  (1U << (5U * 2U));
   GPIOA->OTYPER &= ~(1U << 5U);
   GPIOA->OSPEEDR |= (3U << (5U * 2U));
   GPIOA->PUPDR &= ~(3U << (5U * 2U));
}
/**
 * @brief Configure TIM2 for a continuous 1 Hz update interrupt.
 */
void tim2_1hz_interrupt_init(void)
{
   RCC->APB1ENR |= TIM2EN;
   (void)RCC->APB1ENR;
   /* 10 kHz timer tick with 10000 counts gives one second. */
   TIM2->PSC = (uint32_t)((TIM2_TIMCLK_HZ / TIM2_1HZ_INT_TICK_HZ) - 1UL);
   TIM2->ARR = (uint32_t)(TIM2_1HZ_INT_TICKS - 1UL);
   TIM2->CNT = 0U;
   TIM2->CR1 |= TIM_CR1_CEN;
   TIM2->DIER |= TIM_DIER_UIE;
   NVIC_EnableIRQ(TIM2_IRQn);
}
/**
 * @brief Initialize the HAL TIM2 base for microsecond one-shots.
 * @return TIM2 status.
 */
tim_status_t MX_TIM2_Init(void)
{
   uint32_t tim_clk_hz;
   uint32_t pclk1_hz;
   __HAL_RCC_TIM2_CLK_ENABLE();
   htim2.Instance = TIM2;
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
   /* 1 MHz timer base gives one count per microsecond. */
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
/**
 * @brief Start a TIM2 one-shot in microseconds.
 * @param delay_us One-shot duration in microseconds.
 * @return TIM2 status.
 */
tim_status_t tim2_start_oneshot_us(uint16_t delay_us)
{
   if (delay_us == 0U)
   {
       return TIM_STATUS_BAD_PARAM;
   }
   /* Stop and clear the previous one-shot before loading a new period. */
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
/**
 * @brief Stop the active TIM2 one-shot.
 * @return TIM2 status.
 */
tim_status_t tim2_stop_oneshot(void)
{
   if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
   {
       return TIM_STATUS_STOP_FAIL;
   }
   __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
   __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
   return TIM_STATUS_OK;
}
/**
 * @brief Delay through the HAL millisecond timebase.
 * @param ms Delay in milliseconds.
 */
void delay_ms(uint32_t ms)
{
   HAL_Delay(ms);
}
//void TIM2_IRQHandler(void)
//{
//    HAL_TIM_IRQHandler(&htim2);
//}
