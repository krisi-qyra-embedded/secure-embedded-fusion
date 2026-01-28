/* adc_dma.c | Author: i fiksuari mbas prog | Date: 28 January 2026 */
#include "dma.h"

#define DMA2EN				(1U<<22)
#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)

/* Configures dma2 mem2mem config for normal operation. */
void dma2_mem2mem_config(void)
{
	/* Enable clock access to the dma module */
	RCC->AHB1ENR |= DMA2EN;

	/* Disable dma stream */
	DMA2_Stream0->CR = 0;

	/* Wait until stream is disabled */
	while((DMA2_Stream0->CR & DMA_SCR_EN)){}

	/* Configure dma parameters */

	
	DMA2_Stream0->CR |= (1U<<13);
	DMA2_Stream0->CR &= ~(1U<<14);

	
	DMA2_Stream0->CR |= (1U<<11);
	DMA2_Stream0->CR &= ~(1U<<12);

	
	DMA2_Stream0->CR |=DMA_SCR_MINC;

	
	DMA2_Stream0->CR |=DMA_SCR_PINC;

	/* Select mem-to-mem transfer */
	DMA2_Stream0->CR &= ~(1U<<6);
	DMA2_Stream0->CR |= (1U<<7);

	/* Enable transfer complete interrupt */
	DMA2_Stream0->CR |= DMA_SCR_TCIE;

	/* Enable transfer error interrupt */
	DMA2_Stream0->CR |= DMA_SCR_TEIE;

	
	DMA2_Stream0->FCR |=DMA_SFCR_DMDIS;

	/* Set DMA FIFO threshold */
	DMA2_Stream0->FCR |=(1U<<0);
	DMA2_Stream0->FCR |=(1U<<1);

	/* Enable DMA interrupt in NVIC */
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}



/* Handles the transfer start operation and returns the resulting status. */
void dma_transfer_start(uint32_t src_buff, uint32_t dest_buff, uint32_t len)
{
	/* Check the current status before continuing with the operation. */
	
	DMA2_Stream0->PAR = src_buff;

	
	DMA2_Stream0->M0AR = dest_buff;

	/* Set transfer length */
	DMA2_Stream0->NDTR = len;

	/* Enable dma stream */
	DMA2_Stream0->CR |= DMA_SCR_EN;

}







