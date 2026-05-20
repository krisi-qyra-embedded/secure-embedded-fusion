/**
 * @file adc.c
 * @brief ADC1 setup and blocking conversion helpers for PA1.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include "adc.h"
#include "stm32f4xx.h"

/* ADC1 clock, channel and control-bit definitions. */
#define GPIOAEN		(1U<<0)
#define ADC1EN		(1U<<8)
#define ADC_CH1		(1U<<0)
#define AD_SEQ_LEN_1  0x00
#define CR2_ADCON	 (1U<<0)
#define CR2_CONT	 (1U<<1)
#define CR2_SWSTART	  (1U<<30)
#define SR_EOC		  (1U<<1)

/**
 * @brief Configure PA1 as ADC1 channel 1.
 */
void pa1_adc_init(void)
{
	/* Enable GPIOA and switch PA1 to analog mode. */
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |=(1U<<2);
	GPIOA->MODER |=(1U<<3);

	/* Configure ADC1 for a single-channel conversion sequence. */
	RCC->APB2ENR |=ADC1EN;
	ADC1->SQR3 = ADC_CH1;
	ADC1->SQR1 = AD_SEQ_LEN_1;
	ADC1->CR2 |=CR2_ADCON;
}

/**
 * @brief Start continuous ADC1 conversions.
 */
void start_conversion(void)
{
    /* Check the return status before continuing to the next step. */
	ADC1->CR2 |=CR2_CONT;
	ADC1->CR2 |=CR2_SWSTART;
}

/**
 * @brief Wait for the current ADC1 conversion and return the sample.
 * @return Raw ADC conversion value.
 */
uint32_t adc_read(void)
{
    /* Check the read status before using the returned data. */
	while(!(ADC1->SR & SR_EOC)){}
	return (ADC1->DR);
}
