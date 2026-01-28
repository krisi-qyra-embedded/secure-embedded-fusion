/* adc_dma.c | Author: i fiksuari mbas prog | Date: 28 January 2026 */
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

/* Provides the short delay required by the driver timing. */
void tim2_3ms_delay(void)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    /* Enable TIM2 clock */
    (void)RCC->APB1ENR;

    /* TIM2 input clock = 96 MHz -> 10 kHz timer tick (0.1 ms) */
    TIM2->PSC = (uint32_t)((TIM2_TIMCLK_HZ / TIM2_3MS_TICK_HZ) - 1UL); /* 9600 - 1 */
    TIM2->ARR = (uint32_t)(TIM2_3MS_TICKS - 1UL);                       
    TIM2->CNT = 0U;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->CR1 |= TIM_CR1_CEN;

    while ((TIM2->SR & TIM_SR_UIF) == 0U) {
        __asm__ volatile ("nop");
    }

    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->CR1 &= ~TIM_CR1_CEN;
}

/* Blocking delay using TIM2. */
void tim2_delay_ms(uint32_t ms)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
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

    /* Target a 1 ms time base from a 96 MHz TIM2 input clock. */
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

/* Configures tim2 1s init for normal operation. */
void tim2_1s_init(void)
{
    /* Configure only the resources required by this module. */
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
void tim2_1s_start(void)
{
    /* Check the current status before continuing with the operation. */
    tim2_1s_flag = 0U;
    TIM2->CNT = 0U;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* stop timer and disable interrupt (safe if timer already stopped by IRQ) */
void tim2_1s_stop(void)
{
    /* Check the current status before continuing with the operation. */
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->DIER &= ~TIM_DIER_UIE;
    TIM2->SR &= ~TIM_SR_UIF;
}

/* Configures gpioa5 init for normal operation. */
void gpioa5_init(void)
{
    /* Configure only the resources required by this module. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(3U << (5U * 2U));
    GPIOA->MODER |=  (1U << (5U * 2U));

    GPIOA->OTYPER &= ~(1U << 5U);
    GPIOA->OSPEEDR |= (3U << (5U * 2U));
    GPIOA->PUPDR &= ~(3U << (5U * 2U));
}

/* Configures tim2 1hz interrupt init for normal operation. */
void tim2_1hz_interrupt_init(void)
{
    /* Configure only the resources required by this module. */
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
