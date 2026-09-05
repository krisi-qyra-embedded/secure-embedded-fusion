/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "adc_dma.h"

#define GPIOAEN			(1U<<0)
#define ADC1EN			(1U<<8)
#define CR1_SCAN		(1U<<8)
#define CR2_DMA			(1U<<8)
#define CR2_DDS			(1U<<9)
#define CR2_CONT		(1U<<1)
#define CR2_ADCON		(1U<<0)
#define CR2_SWSTART		(1U<<30)

#define DMA2EN				(1U<<22)
#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_CIRC		(1U<<8)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)


#define TIM2EN				(1U<<0)
#define CR1_CEN				(1U<<0)


uint16_t adc_raw_data[NUM_OF_SAMPLES];

/* Configures timer-triggered ADC1 sampling through DMA. */
void adc_tim_dma_init(void)
{
	/* Stop Stream0 before configuring circular transfers from ADC1 DR. */
	RCC->AHB1ENR |= GPIOAEN;

	GPIOA->MODER |= (1U<<0);
	GPIOA->MODER |= (1U<<1);

	RCC->APB2ENR |= ADC1EN;

	ADC1->CR2 |= CR2_DMA|CR2_DDS;

	ADC1->CR2 |= (1U<<28);
	ADC1->CR2 &= ~(1U<<29);

	ADC1->CR2 &= ~(1U<<24);
	ADC1->CR2 |= (1U<<25);
	ADC1->CR2 |= (1U<<26);
	ADC1->CR2 &= ~(1U<<27);

	RCC->AHB1ENR |=DMA2EN;

	DMA2_Stream0->CR &=~DMA_SCR_EN;

	while((DMA2_Stream0->CR & DMA_SCR_EN)){}

	DMA2_Stream0->CR |=DMA_SCR_CIRC;

	DMA2_Stream0->CR |= (1U<<13);
	DMA2_Stream0->CR &= ~(1U<<14);

	DMA2_Stream0->CR |= (1U<<11);
	DMA2_Stream0->CR &= ~(1U<<12);

	DMA2_Stream0->CR |=DMA_SCR_MINC;

	DMA2_Stream0->CR |= DMA_SCR_TCIE;

	DMA2_Stream0->PAR = (uint32_t)(&(ADC1->DR));
	DMA2_Stream0->M0AR = (uint32_t)(&adc_raw_data);

	DMA2_Stream0->NDTR = (uint16_t)NUM_OF_SAMPLES;

	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	RCC->APB1ENR |= TIM2EN;

	TIM2->PSC =  16000 - 1; //  16 000 000 / 16 000 = 1 000

	TIM2->ARR = 10 - 1; // 1 000 /10 =  100Hz

	TIM2->CR2 &= ~(1U<<4);
	TIM2->CR2 |= (1U<<5);
	TIM2->CR2 &= ~(1U<<6);
	ADC1->CR2 |=CR2_ADCON;

	DMA2_Stream0->CR |= DMA_SCR_EN;

	TIM2->CR1 |=CR1_CEN;

}

/* Starts continuous two-channel ADC1 sampling through DMA. */
void adc_dma_init(void)
{
	/* Scan channels 0 and 1 into the circular sample buffer. */
	RCC->AHB1ENR |= GPIOAEN;

	GPIOA->MODER |= (1U<<0);
	GPIOA->MODER |= (1U<<1);

	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	RCC->APB2ENR |= ADC1EN;

	ADC1->SQR1 |= (1U<<20);
	ADC1->SQR1 &= ~(1U<<21);
	ADC1->SQR1 &= ~(1U<<22);
	ADC1->SQR1 &= ~(1U<<23);

	ADC1->SQR3 = (0U<<0) | (1U<<5);

	ADC1->CR1 = CR1_SCAN;

	ADC1->CR2 |=CR2_CONT |CR2_DMA|CR2_DDS;

	RCC->AHB1ENR |=DMA2EN;

	DMA2_Stream0->CR &=~DMA_SCR_EN;

	while((DMA2_Stream0->CR & DMA_SCR_EN)){}

	DMA2_Stream0->CR |=DMA_SCR_CIRC;

	DMA2_Stream0->CR |= (1U<<13);
	DMA2_Stream0->CR &= ~(1U<<14);

	DMA2_Stream0->CR |= (1U<<11);
	DMA2_Stream0->CR &= ~(1U<<12);

	DMA2_Stream0->CR |=DMA_SCR_MINC;

	DMA2_Stream0->PAR = (uint32_t)(&(ADC1->DR));
	DMA2_Stream0->M0AR = (uint32_t)(&adc_raw_data);

	DMA2_Stream0->NDTR = (uint16_t)NUM_OF_CHANNELS;

	DMA2_Stream0->CR |= DMA_SCR_EN;

	ADC1->CR2 |=CR2_ADCON;

	ADC1->CR2 |=CR2_SWSTART;

}
