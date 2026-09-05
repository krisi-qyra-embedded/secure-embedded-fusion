/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "adc.h"
#include "stm32f4xx.h"

#define GPIOAEN		(1U<<0)
#define ADC1EN		(1U<<8)
#define ADC_CH1		(1U<<0)
#define AD_SEQ_LEN_1  0x00
#define CR2_ADCON	 (1U<<0)
#define CR2_CONT	 (1U<<1)
#define CR2_SWSTART	  (1U<<30)
#define SR_EOC		  (1U<<1)

/* Configures PA1 as the ADC1 input. */
void pa1_adc_init(void)
{
	/* Select channel 1 as the single regular conversion. */
	RCC->AHB1ENR |= GPIOAEN;

	GPIOA->MODER |=(1U<<2);
	GPIOA->MODER |=(1U<<3);

	RCC->APB2ENR |=ADC1EN;

	ADC1->SQR3 = ADC_CH1;

	ADC1->SQR1 = AD_SEQ_LEN_1;

	ADC1->CR2 |=CR2_ADCON;

}

/* Starts continuous ADC1 conversions. */
void start_conversion(void)
{
	/* Software-trigger the first conversion after enabling continuous mode. */
	ADC1->CR2 |=CR2_CONT;

	ADC1->CR2 |=CR2_SWSTART;
}

/* Reads the next ADC1 conversion result. */
uint32_t adc_read(void)
{
	/* Wait for EOC before reading the ADC data register. */
	while(!(ADC1->SR & SR_EOC)){}

	return (ADC1->DR);
}
