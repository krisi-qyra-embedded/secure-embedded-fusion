/* dma.c | Author: i fiksuari pas prog | Date: 17 January 2026 */
#include "dma.h"

#define DMA2EN				(1U<<22)
#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)

/* Initializes dma2 mem2mem config for the requested operation. */
void dma2_mem2mem_config(void)
{
	/* Validate required parameters before applying the setup. */
	/* Enable clock access to the dma module */
	RCC->AHB1ENR |= DMA2EN;

	/* Disable dma stream */
	DMA2_Stream0->CR = 0;

	 
	while((DMA2_Stream0->CR & DMA_SCR_EN)){}

	/* Configure dma parameters */

	 
	DMA2_Stream0->CR |= (1U<<13);
	DMA2_Stream0->CR &= ~(1U<<14);

	 
	DMA2_Stream0->CR |= (1U<<11);
	DMA2_Stream0->CR &= ~(1U<<12);

	 
	DMA2_Stream0->CR |=DMA_SCR_MINC;

	 
	DMA2_Stream0->CR |=DMA_SCR_PINC;

	 
	DMA2_Stream0->CR &= ~(1U<<6);
	DMA2_Stream0->CR |= (1U<<7);

	 
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



/* Handles the dma transfer start operation. */
void dma_transfer_start(uint32_t src_buff, uint32_t dest_buff, uint32_t len)
{
	/* Check the operation status before continuing. */
	 
	DMA2_Stream0->PAR = src_buff;

	 
	DMA2_Stream0->M0AR = dest_buff;

	 
	DMA2_Stream0->NDTR = len;

	 
	DMA2_Stream0->CR |= DMA_SCR_EN;

}







