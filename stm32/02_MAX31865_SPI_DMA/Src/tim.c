#include"tim.h"

#define TIM2EN (1U << 0)
volatile uint8_t tim2_1s_flag = 0;

/* Handles the tim2 3ms delay operation. */
void tim2_3ms_delay(void)
{
    /* Check the current status before proceeding. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     

    TIM2->PSC = 1600 - 1;                    
    TIM2->ARR = 30;                          
    TIM2->CNT = 0;                           
    TIM2->SR &= ~TIM_SR_UIF;                /* Clear update interrupt flag */
    TIM2->CR1 |= TIM_CR1_CEN;                
    while(!(TIM2->SR & TIM_SR_UIF));       /* Wait for update event (timer overflow) */
    TIM2->SR &= ~TIM_SR_UIF;                /* Clear flag */
    TIM2->CR1 &= ~TIM_CR1_CEN;               
}

/* Blocking delay using TIM2. */
void tim2_delay_ms(uint32_t ms)
{
    /* Check the current status before proceeding. */
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
        /* Wait for update event (UIF) */
        while (!(TIM2->SR & TIM_SR_UIF)) {
            __asm volatile ("nop");
        }
        /* Stop timer and clear flag */
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        remaining_ticks -= this_chunk;
    }
     
}


/* Initializes the resources used by tim2 1s init. */
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

 
/* Handles the tim2 1s start operation. */
 void tim2_1s_start(void)
{
    /* Check the current status before proceeding. */
    tim2_1s_flag = 0;            /* clear user-visible flag */
    TIM2->CNT = 0;               /* reset counter */
    TIM2->SR &= ~TIM_SR_UIF;     /* clear pending update flag */
    TIM2->DIER |= TIM_DIER_UIE;   
    TIM2->CR1 |= TIM_CR1_CEN;     
}

/* stop timer and disable interrupt (safe if timer already stopped by IRQ) */
 void tim2_1s_stop(void)
{
    /* Check the current status before proceeding. */
    TIM2->CR1 &= ~TIM_CR1_CEN;    
    TIM2->DIER &= ~TIM_DIER_UIE;  
    TIM2->SR &= ~TIM_SR_UIF;     /* clear pending flag */
}

/* Initializes the resources used by gpioa5 init. */
void gpioa5_init(void)
{
    /* Set up the required peripheral configuration. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   /* Enable GPIOA clock */

    GPIOA->MODER &= ~(3 << (5 * 2));        
    GPIOA->MODER |= (1 << (5 * 2));         

    GPIOA->OTYPER &= ~(1 << 5);              
    GPIOA->OSPEEDR |= (3 << (5 * 2));       
    GPIOA->PUPDR &= ~(3 << (5 * 2));        
}

/* Handles the interrupt state for tim2 1hz interrupt init. */
void tim2_1hz_interrupt_init(void)
{
    /* Set up the required peripheral configuration. */
	RCC->APB1ENR |= TIM2EN;
	TIM2->PSC = 9600-1; 
	TIM2->ARR = 10000; 
	TIM2->CNT = 0; 
	TIM2->CR1 |= TIM2EN; 

	 
	TIM2->DIER |= (1U << 0);
	 
	NVIC_EnableIRQ(TIM2_IRQn);
}




