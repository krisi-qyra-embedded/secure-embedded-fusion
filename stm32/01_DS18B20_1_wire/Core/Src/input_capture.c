/* input_capture.c | Author: i fiksuari pas prog | Date: 22 February 2026 */
#include "input_capture.h"

/* Implements the tim2 output compare PA5 operation. */
void tim2_output_compare_PA5(void)//00: CC1 channel is configured as output  by deafoult
{
    /* Handle the operation and its status locally. */
RCC->AHB1ENR |= (1U<<0);
GPIOA->MODER &=~(1U<<10);
GPIOA->MODER |=(1U<<11);
GPIOA->AFR[0] |=(1U<<20);
RCC->APB1ENR |= (1U<<0);
TIM2->PSC = 4800-1;
TIM2->ARR = 10000-1;
TIM2->CCMR1 = ((1U<<4)|(1U<<5));
TIM2->CCER |= (1U<<0);
TIM2->CNT = 0;
TIM2->CR1 = (1U<<0);
}

/* Implements the tim3 input capture PA6 operation. */
void tim3_input_capture_PA6(void)//01: CC1 channel is configured as input, IC1 is mapped on TI1.TIM3->CCMR1 |= (1U<<0); enable the input
{

	    // Enable GPIO clock for Port A
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	    // Configure PA6 as alternate function mode
	    GPIOA->MODER |= 0x2000;
	    // Select AF2 (TIM3) for PA6
	    GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_1;
	    // Enable TIM3 clock
	    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	    // Set prescaler to scale down the clock frequency (optional, depending on input frequency range)
	    // For example, if we expect a 1 Hz signal, the prescaler can be set to 96000-1 to make each timer tick 1 ms.
	    TIM3->PSC = 48000-1;  // Prescaler for 96 MHz system clock, resulting in a 1 kHz timer clock
	    // Set auto-reload register to maximum value (optional, not necessary for input capture)
	    //TIM3->ARR = 0xFFFF;  // Max value to allow capturing full timer range
	    // Configure TIM3 in input capture mode
	    TIM3->CCMR1 = 0x41;  // CC1 channel is configured as input, IC1 is mapped on TI1
	    // Capture on rising edge
	    TIM3->CCER = 0x0B;
	    // Enable TIM3 counter
	    TIM3->CR1 |= TIM_CR1_CEN;
}
