/* tim.c | Author: i fiksuari mbas prog | Date: 13 January 2026 */
#include"tim.h"

#define TIM2EN (1U << 0)
volatile uint8_t tim2_1s_flag = 0;

/* Provides the short delay required by the driver timing. */
void tim2_3ms_delay(void)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    /* Enable TIM2 clock */

    TIM2->PSC = 1600 - 1;                   
    TIM2->ARR = 30;                         
    TIM2->CNT = 0;                          /* Reset counter */
    TIM2->SR &= ~TIM_SR_UIF;                /* Clear update interrupt flag */
    TIM2->CR1 |= TIM_CR1_CEN;               
    while(!(TIM2->SR & TIM_SR_UIF));       /* Wait for update event (timer overflow) */
    TIM2->SR &= ~TIM_SR_UIF;                
    TIM2->CR1 &= ~TIM_CR1_CEN;              
}

/**
 * Blocking delay using TIM2.
 * @param ms  Number of milliseconds to delay (0 -> returns immediately)
 *
 * Notes:
 * - Uses SystemCoreClock (CMSIS) to derive prescaler. Ensure SystemCoreClock is set.
 * - Chooses PSC so PSC <= 0xFFFF. ARR is 32-bit (TIM2 is 32-bit on many STM32 parts).
 * - This is a blocking busy-wait that waits for the update event (UIF).
 */
void tim2_delay_ms(uint32_t ms)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    if (ms == 0U) return;
    /* Enable TIM2 clock (idempotent) */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR; /* small readback to ensure clock write completes */
    
    uint64_t base_presc = (uint64_t)16000000 / 1000U; 
    
    if (base_presc < 1U) base_presc = 1U;

    uint32_t presc_reg;      /* value to write into PSC (PSC = presc_reg) */
    uint64_t presc_plus1;    
    if (base_presc <= 0xFFFFU) {
        
        presc_plus1 = base_presc;
        presc_reg = (uint32_t)(presc_plus1 - 1U);
    } else {
        
        
        uint64_t factor = (base_presc + 0xFFFFULL - 1ULL) / 0xFFFFULL;
        
        presc_plus1 = (base_presc + factor - 1ULL) / factor;
        if (presc_plus1 < 1ULL) presc_plus1 = 1ULL;
        if (presc_plus1 > 0xFFFFULL) presc_plus1 = 0xFFFFULL; 
        presc_reg = (uint32_t)(presc_plus1 - 1U);
    }
    /* Compute timer tick frequency (Hz) = SystemCoreClock / presc_plus1 */
    uint64_t tick_freq = (uint64_t)16000000 / presc_plus1;
    
    uint64_t ticks_needed = (tick_freq * (uint64_t)ms + 999ULL) / 1000ULL;
    /* ARR must fit into a 32-bit register for TIM2 (on many STM32 families TIM2 is 32-bit). */
    const uint64_t ARR_MAX = 0xFFFFFFFFULL;
    /* If ticks_needed fits in ARR, do single-shot; otherwise split into chunks */
    uint64_t remaining_ticks = ticks_needed;
    while (remaining_ticks > 0ULL) {
        uint64_t this_chunk = (remaining_ticks <= ARR_MAX) ? remaining_ticks : ARR_MAX;
        
        TIM2->PSC = presc_reg & 0xFFFFU;         
        TIM2->ARR = (uint32_t)this_chunk;       
        TIM2->CNT = 0U;                         /* reset counter */
        /* Clear UIF flag and start timer */
        TIM2->SR &= ~TIM_SR_UIF;
        __DSB(); __ISB();
        TIM2->CR1 |= TIM_CR1_CEN;
        
        while (!(TIM2->SR & TIM_SR_UIF)) {
            __asm volatile ("nop");
        }
        /* Stop timer and clear flag */
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        remaining_ticks -= this_chunk;
    }
    
}


/* Configures tim2 1s init for normal operation. */
void tim2_1s_init(void)
{
    /* Enable TIM2 clock once */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    
    TIM2->PSC = 16000 - 1;    
    TIM2->ARR = 1000 - 1;     
    TIM2->CNT = 0;

    /* Clear any pending flag and disable update interrupt for now */
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER &= ~TIM_DIER_UIE;

    /* Enable NVIC IRQ once (idempotent) */
    NVIC_EnableIRQ(TIM2_IRQn);

    
}

/* start timer for one-shot 1s; safe to call repeatedly */
 void tim2_1s_start(void)
{
    /* Check the current status before continuing with the operation. */
    tim2_1s_flag = 0;            
    TIM2->CNT = 0;               /* reset counter */
    TIM2->SR &= ~TIM_SR_UIF;     
    TIM2->DIER |= TIM_DIER_UIE;  /* enable update interrupt */
    TIM2->CR1 |= TIM_CR1_CEN;    /* start timer */
}

/* stop timer and disable interrupt (safe if timer already stopped by IRQ) */
 void tim2_1s_stop(void)
{
    /* Check the current status before continuing with the operation. */
    TIM2->CR1 &= ~TIM_CR1_CEN;   /* stop timer */
    TIM2->DIER &= ~TIM_DIER_UIE; /* disable update interrupt */
    TIM2->SR &= ~TIM_SR_UIF;     
}

/* Configures gpioa5 init for normal operation. */
void gpioa5_init(void)
{
    /* Configure only the resources required by this module. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   

    GPIOA->MODER &= ~(3 << (5 * 2));       
    GPIOA->MODER |= (1 << (5 * 2));        

    GPIOA->OTYPER &= ~(1 << 5);             
    GPIOA->OSPEEDR |= (3 << (5 * 2));      
    GPIOA->PUPDR &= ~(3 << (5 * 2));       
}

/* Configures tim2 1hz interrupt init for normal operation. */
void tim2_1hz_interrupt_init(void)
{
	/* Configure only the resources required by this module. */
	RCC->APB1ENR |= TIM2EN;
	TIM2->PSC = 9600-1;
	TIM2->ARR = 10000;
	TIM2->CNT = 0;/* the count has stopped after psc and arr has reached it's climax (Clear the value) */
	TIM2->CR1 |= TIM2EN;

	
	TIM2->DIER |= (1U << 0);
	
	NVIC_EnableIRQ(TIM2_IRQn);
}




