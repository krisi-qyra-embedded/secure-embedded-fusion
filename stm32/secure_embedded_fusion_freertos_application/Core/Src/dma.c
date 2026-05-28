/* Author: i fiksuari pas prog
 * Documentation date: 2026-05-28
 */

#include "dma.h"

#define DMA2EN				(1U<<22)
#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)

/* Configures DMA2 Stream0 for memory-to-memory copies. */
void dma2_mem2mem_config(void)
{
	/* Enable address increment and transfer-complete and error interrupts. */
	RCC->AHB1ENR |= DMA2EN;

	DMA2_Stream0->CR = 0;

	while((DMA2_Stream0->CR & DMA_SCR_EN)){}

	DMA2_Stream0->CR |= (1U<<13);
	DMA2_Stream0->CR &= ~(1U<<14);

	DMA2_Stream0->CR |= (1U<<11);
	DMA2_Stream0->CR &= ~(1U<<12);

	DMA2_Stream0->CR |=DMA_SCR_MINC;

	DMA2_Stream0->CR |=DMA_SCR_PINC;

	DMA2_Stream0->CR &= ~(1U<<6);
	DMA2_Stream0->CR |= (1U<<7);

	DMA2_Stream0->CR |= DMA_SCR_TCIE;

	DMA2_Stream0->CR |= DMA_SCR_TEIE;

	DMA2_Stream0->FCR |=DMA_SFCR_DMDIS;

	DMA2_Stream0->FCR |=(1U<<0);
	DMA2_Stream0->FCR |=(1U<<1);

	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/* Starts a DMA2 Stream0 memory copy. */
void dma_transfer_start(uint32_t src_buff, uint32_t dest_buff, uint32_t len)
{
	/* Load both addresses and the transfer count before enabling the stream. */
	DMA2_Stream0->PAR = src_buff;

	DMA2_Stream0->M0AR = dest_buff;

	DMA2_Stream0->NDTR = len;

	DMA2_Stream0->CR |= DMA_SCR_EN;

}

