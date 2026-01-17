/* tim.c | Author: i fiksuari pas prog | Date: 17 January 2026 */
#include"tim.h"

#define TIM2EN (1U << 0)
volatile uint8_t tim2_1s_flag = 0;

/* Provides the delay required by tim2 3ms delay. */
void tim2_3ms_delay(void)
{
    /* Keep the delay within the configured timing interval. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    // Enable TIM2 clock

    TIM2->PSC = 1600 - 1;                   // Prescaler: 16MHz/1600 = 10kHz (0.1 ms tick)
    TIM2->ARR = 30;                         // Auto-reload: 30 ticks = 3 ms
    TIM2->CNT = 0;                          // Reset counter
    TIM2->SR &= ~TIM_SR_UIF;                // Clear update interrupt flag
    TIM2->CR1 |= TIM_CR1_CEN;               // Enable timer
    while(!(TIM2->SR & TIM_SR_UIF));       // Wait for update event (timer overflow)
    TIM2->SR &= ~TIM_SR_UIF;                // Clear flag
    TIM2->CR1 &= ~TIM_CR1_CEN;              // Disable timer (optional)
}

/* Initializes tim2 1s init for the requested operation. */
void tim2_1s_init(void)
{
    /* Validate required parameters before applying the setup. */
    /* Enable TIM2 clock once */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Configure prescaler/arr for ~1s */
    TIM2->PSC = 16000 - 1;     
    TIM2->ARR = 1000 - 1;      
    TIM2->CNT = 0;

     
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER &= ~TIM_DIER_UIE;

    /* Enable NVIC IRQ once (idempotent) */
    NVIC_EnableIRQ(TIM2_IRQn);

     
}

 
/* Handles the tim2 1s start operation. */
 void tim2_1s_start(void)
{
    /* Check the operation status before continuing. */
    tim2_1s_flag = 0;             
    TIM2->CNT = 0;               /* reset counter */
    TIM2->SR &= ~TIM_SR_UIF;      
    TIM2->DIER |= TIM_DIER_UIE;   
    TIM2->CR1 |= TIM_CR1_CEN;     
}

/* stop timer and disable interrupt (safe if timer already stopped by IRQ) */
/* Handles the tim2 1s stop operation. */
 void tim2_1s_stop(void)
{
    /* Check the operation status before continuing. */
    TIM2->CR1 &= ~TIM_CR1_CEN;   /* stop timer */
    TIM2->DIER &= ~TIM_DIER_UIE;  
    TIM2->SR &= ~TIM_SR_UIF;      
}

/* Initializes gpioa5 init for the requested operation. */
void gpioa5_init(void)
{
    /* Validate required parameters before applying the setup. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock

    GPIOA->MODER &= ~(3 << (5 * 2));       // Clear mode bits for PA5
    GPIOA->MODER |= (1 << (5 * 2));        // Set PA5 as general purpose output

    GPIOA->OTYPER &= ~(1 << 5);             // Push-pull output
    GPIOA->OSPEEDR |= (3 << (5 * 2));      // High speed
    GPIOA->PUPDR &= ~(3 << (5 * 2));       // No pull-up, no pull-down
}

/* Handles the tim2 1hz interrupt init interrupt or callback. */
void tim2_1hz_interrupt_init(void)
{
	/* Record the event so waiting code can finish safely. */
	RCC->APB1ENR |= TIM2EN;
	TIM2->PSC = 9600-1;//Divide 16000000/1600 = 10000
	TIM2->ARR = 10000;//Divide 10000/10000 = 1 second or 1 hz frequency
	TIM2->CNT = 0;//the count has stopped after psc and arr has reached it's climax (Clear the value)
	TIM2->CR1 |= TIM2EN;//after every configurations comes the enable of the counter

	//enable the interrupt
	TIM2->DIER |= (1U << 0);
	//enable the interrupt in the NVIC.
	NVIC_EnableIRQ(TIM2_IRQn);
}




