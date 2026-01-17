/* spi_dma.c | Author: i fiksuari pas prog | Date: 17 January 2026 */
#include "spi_dma.h"


 





#define GPIOAEN			(1U<<0)
#define SPI1EN			(1U<<12)

#define CR1_SSM			(1U<<9)
#define CR1_SSI			(1U<<8)
#define CR1_MSTR		(1U<<2)
#define CR1_CPOL		(1U<<1)
#define CR1_PHA			(1U<<0)
#define CR2_TXDMAEN		(1U<<1)
#define CR2_RXDMAEN		(1U<<0)
#define CR1_SPE			(1U<<6)

#define DMA2EN				(1U<<22)

#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_CIRC		(1U<<8)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)

#define HIFCR_CDMEIF5		(1U<<8)
#define HIFCR_CTEIF5		(1U<<9)
#define HIFCR_CTCIF5		(1U<<11)

#define HIFCR_CDMEIF6		(1U<<18)
#define HIFCR_CTEIF6		(1U<<19)
#define HIFCR_CTCIF6		(1U<<21)

#define HIFSR_TCIF5		(1U<<11)
#define HIFSR_TCIF6		(1U<<21)


/* Initializes spi1 dma init for the requested operation. */
void spi1_dma_init(void)
{
	/* Validate required parameters before applying the setup. */

	/* GPIO Configuration */
	/* Enable clock to PORT of SPI pins */
	RCC->AHB1ENR |= GPIOAEN;

	 
	 
	GPIOA->MODER &=~(1U<<10);
	GPIOA->MODER |=(1U<<11);

	 
	GPIOA->MODER &=~(1U<<12);
	GPIOA->MODER |=(1U<<13);

	 
	GPIOA->MODER &=~(1U<<14);
	GPIOA->MODER |=(1U<<15);

	 
	 
	GPIOA->AFR[0]  |= (1U<<20);
	GPIOA->AFR[0]  &=~(1U<<21);
	GPIOA->AFR[0]  |= (1U<<22);
	GPIOA->AFR[0]  &=~(1U<<23);

	 
	GPIOA->AFR[0]  |= (1U<<24);
	GPIOA->AFR[0]  &=~(1U<<25);
	GPIOA->AFR[0]  |= (1U<<26);
	GPIOA->AFR[0]  &=~(1U<<27);

	 
	GPIOA->AFR[0]  |= (1U<<28);
	GPIOA->AFR[0]  &=~(1U<<29);
	GPIOA->AFR[0]  |= (1U<<30);
	GPIOA->AFR[0]  &=~(1U<<31);

	 

	/* Enable clock access to SPI1 module */
	RCC->APB2ENR |=SPI1EN;

	 
	SPI1->CR1 |= CR1_SSM|CR1_SSI;

	 
	SPI1->CR1 |=CR1_MSTR;

	 
	SPI1->CR1 |=SPI_CR1_BR_0|CR1_PHA|CR1_CPOL;

	/* Set clock divider : PCLCK/4 */
	SPI1->CR1 |=(1U<<3);
	SPI1->CR1 &=~(1U<<4);
	SPI1->CR1 &=~(1U<<5);

	/* Select to use DMA */
	SPI1->CR2 |=CR2_RXDMAEN|CR2_TXDMAEN;

     
	SPI1->CR1 |=CR1_SPE;
}

/* Initializes dma2 stream3 spi tx init for the requested operation. */
void dma2_stream3_spi_tx_init(void)
{
	/* Validate required parameters before applying the setup. */
	/* DMA Configuration */
	/* Enable clock access to DMA */
	RCC->AHB1ENR |= DMA2EN;

	/* Disable DMA stream */
	DMA2_Stream3->CR = 0;

	/* Wait till DMA Stream is disabled */
	while((DMA2_Stream3->CR & DMA_SCR_EN)){}

    /* Configure DMA Stream parameters */
	 
	DMA2_Stream3->CR |= DMA_SCR_MINC;

	 
	DMA2_Stream3->CR |= (1U<<25);
	DMA2_Stream3->CR |= (1U<<26);
	DMA2_Stream3->CR &= ~(1U<<27);

	 
	DMA2_Stream3->CR |=(1U<<6);
	DMA2_Stream3->CR &=~(1U<<7);

	 
	DMA2_Stream3->CR |= DMA_SCR_TCIE;

	/* Enable transfer error interrupt */
	DMA2_Stream3->CR |= DMA_SCR_TEIE;

	 
	DMA2_Stream3->FCR |=DMA_SFCR_DMDIS;

	/* Set DMA FIFO threshold */
	DMA2_Stream3->FCR |=(1U<<0);
	DMA2_Stream3->FCR |=(1U<<1);

	/* Enable DMA Stream interrupt in the NVIC */
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}


/* Initializes dma2 stream2 spi rx init for the requested operation. */
void dma2_stream2_spi_rx_init(void)
{
	/* Validate required parameters before applying the setup. */
	/* DMA Configuration */
	/* Enable clock access to DMA */
	RCC->AHB1ENR |= DMA2EN;

	/* Disable DMA stream */
	DMA2_Stream2->CR = 0;

	/* Wait till DMA Stream is disabled */
	while((DMA2_Stream2->CR & DMA_SCR_EN)){}

    /* Configure DMA Stream parameters */
	 
	DMA2_Stream2->CR |= DMA_SCR_MINC;

	 
	DMA2_Stream2->CR |= (1U<<25);
	DMA2_Stream2->CR |= (1U<<26);
	DMA2_Stream2->CR &= ~(1U<<27);

	 
	DMA2_Stream2->CR &=~(1U<<6);
	DMA2_Stream2->CR &=~(1U<<7);

	 
	DMA2_Stream2->CR |= DMA_SCR_TCIE;

	/* Enable transfer error interrupt */
	DMA2_Stream2->CR |= DMA_SCR_TEIE;

	 
	DMA2_Stream2->FCR |=DMA_SFCR_DMDIS;

	 
	DMA2_Stream2->FCR |=(1U<<0);
	DMA2_Stream2->FCR |=(1U<<1);

	/* Enable DMA Stream interrupt in the NVIC */
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}


/* Handles the dma2 stream3 spi transfer operation. */
void dma2_stream3_spi_transfer(uint32_t msg_to_send, uint32_t msg_len)
{
	/* Check the operation status before continuing. */
	 
	DMA2->LIFCR = LIFCR_CTCIF3 |LIFCR_CTEIF3;

	 
	DMA2_Stream3->PAR =	(uint32_t)(&(SPI1->DR));

	 
	DMA2_Stream3->M0AR = msg_to_send;

	 
	DMA2_Stream3->NDTR = msg_len;

	 
	DMA2_Stream3->CR |= DMA_SCR_EN;
}

/* Handles the dma2 stream2 spi receive operation. */
void dma2_stream2_spi_receive(uint32_t received_msg, uint32_t msg_len)
{
	/* Check the operation status before continuing. */
	 
	DMA2->LIFCR = LIFCR_CTCIF2 |LIFCR_CTEIF2;

	 
	DMA2_Stream2->PAR =	(uint32_t)(&(SPI1->DR));

	 
	DMA2_Stream2->M0AR = received_msg;

	 
	DMA2_Stream2->NDTR = msg_len;

	 
	DMA2_Stream2->CR |= DMA_SCR_EN;

}

