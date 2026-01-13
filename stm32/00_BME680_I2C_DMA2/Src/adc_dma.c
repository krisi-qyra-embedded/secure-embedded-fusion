/* adc_dma.c | Author: i fiksuari mbas prog | Date: 13 January 2026 */
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


/* Configures tim dma init for normal operation. */
void adc_tim_dma_init(void)
{
	/* GPIO Configuration */
	/* Enable clock access to ADC GPIO Pin's Port */
	RCC->AHB1ENR |= GPIOAEN;

	
	GPIOA->MODER |= (1U<<0);
	GPIOA->MODER |= (1U<<1);

	/* ADC Configuration */
	/* Enable clock access to ADC */
	RCC->APB2ENR |= ADC1EN;

	/* Select to use DMA */
	ADC1->CR2 |= CR2_DMA|CR2_DDS;

	
	ADC1->CR2 |= (1U<<28);
	ADC1->CR2 &= ~(1U<<29);

	
	ADC1->CR2 &= ~(1U<<24);
	ADC1->CR2 |= (1U<<25);
	ADC1->CR2 |= (1U<<26);
	ADC1->CR2 &= ~(1U<<27);


	/* DMA Configuration */
	/* Enable clock access to DMA */
	RCC->AHB1ENR |=DMA2EN;


	/* Disable DMA stream */
	DMA2_Stream0->CR &=~DMA_SCR_EN;

	/* Wait till DMA is disabled */
	while((DMA2_Stream0->CR & DMA_SCR_EN)){}

	
	DMA2_Stream0->CR |=DMA_SCR_CIRC;

	

	DMA2_Stream0->CR |= (1U<<13);
	DMA2_Stream0->CR &= ~(1U<<14);

	
	DMA2_Stream0->CR |= (1U<<11);
	DMA2_Stream0->CR &= ~(1U<<12);

	
	DMA2_Stream0->CR |=DMA_SCR_MINC;

	/* Enable transfer complete interrupt */
	DMA2_Stream0->CR |= DMA_SCR_TCIE;

	
	DMA2_Stream0->PAR = (uint32_t)(&(ADC1->DR));
	
	DMA2_Stream0->M0AR = (uint32_t)(&adc_raw_data);

	/* Set number of transfer */
	DMA2_Stream0->NDTR = (uint16_t)NUM_OF_SAMPLES;

	/* Enable DMA interrupt in NVIC */
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	

	/* Configure timer period to 100Hz i.e, ADC is going to be sampling at a 100Hz rate, every 10ms */

    /* Enable clock access tO TIM2 */
	RCC->APB1ENR |= TIM2EN;

	
	TIM2->PSC =  16000 - 1; 

	
	TIM2->ARR = 10 - 1; 

	
	TIM2->CR2 &= ~(1U<<4);
	TIM2->CR2 |= (1U<<5);
	TIM2->CR2 &= ~(1U<<6);
	
	/* Enable ADC */
	ADC1->CR2 |=CR2_ADCON;

	/* Enable DMA stream */
	DMA2_Stream0->CR |= DMA_SCR_EN;

	
	TIM2->CR1 |=CR1_CEN;

}

/* Configures dma init for normal operation. */
void adc_dma_init(void)
{
	/* GPIO Configuration */
	/* Enable clock access to ADC GPIO Pin's Port */
	RCC->AHB1ENR |= GPIOAEN;

	
	GPIOA->MODER |= (1U<<0);
	GPIOA->MODER |= (1U<<1);

	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	/* ADC Configuration */
	
	RCC->APB2ENR |= ADC1EN;

	
	ADC1->SQR1 |= (1U<<20);
	ADC1->SQR1 &= ~(1U<<21);
	ADC1->SQR1 &= ~(1U<<22);
	ADC1->SQR1 &= ~(1U<<23);

	
	ADC1->SQR3 = (0U<<0) | (1U<<5);

	
	ADC1->CR1 = CR1_SCAN;

	
	ADC1->CR2 |=CR2_CONT |CR2_DMA|CR2_DDS;


	/* DMA Configuration */
	
	RCC->AHB1ENR |=DMA2EN;


	
	DMA2_Stream0->CR &=~DMA_SCR_EN;

	/* Wait till DMA is disabled */
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

	/* ADC Configuration */
	
	ADC1->CR2 |=CR2_ADCON;

	
	ADC1->CR2 |=CR2_SWSTART;


}
