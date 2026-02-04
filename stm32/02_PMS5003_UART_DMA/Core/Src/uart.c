/* uart.c | Author: i fiksuari pas prog | Date: 04 February 2026 */
#include "uart.h"
#include "DMA.h"
#define periphclck 96000000//max clock speed
#define baudrate 9600
static void uart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t BaudRate);
static uint16_t compute_uart_div(uint32_t PeriphClk,uint32_t BaudRate);
#define DMA2EN (1U << 22)


/* Configure the UART interface in uart2_tx_rx_init. */
void uart2_tx_rx_init(void)
{
		/* Set the baud rate and peripheral registers for this interface. */
		RCC->AHB1ENR |= (1U << 0);
		// Set PA9 as TX (alternate function mode)
		GPIOA->MODER &= ~(1U << 18);  // Clear mode bits for PA9
		GPIOA->MODER |= (1U << 19);   // Set mode bits to alternate function (10)
		// Set PA10 as RX (alternate function mode)
		GPIOA->MODER &= ~(1U << 20);  // Clear mode bits for PA10
		GPIOA->MODER |= (1U << 21);   // Set mode bits to alternate function (10)
		// Set alternate function to AF7 (USART1) for PA9 and PA10
		GPIOA->AFR[1] |= (1U << 4) | (1U << 5) | (1U << 6);  // AF7 for PA9
		GPIOA->AFR[1] &= ~(1U << 7);
		GPIOA->AFR[1] |= (1U << 8) | (1U << 9) | (1U << 10); // AF7 for PA10
		GPIOA->AFR[1] &= ~(1U << 11);
		// Enable USART1 clock on APB2
		RCC->APB2ENR |= (1U << 4);
		// Configure baud rate
		uart_set_baudrate(USART1,periphclck,baudrate);
		// Enable DMA for USART1 (optional)
		USART1->CR3 |= ((1U << 6) | (1U << 7));
		// Enable TX and RX in USART1 control register
		USART1->CR1 |= ((1U << 3) | (1U << 2));
		// Clear TXE flag
		USART1->SR &= ~(1U << 6);
		// Enable TXE, RXNE, and USART1
		USART1->CR1 |= (1U << 7) | (1U << 6) | (1U << 5);
		USART1->CR1 |= (1U << 13);
		// Enable USART1 interrupt in NVIC (optional)
		//NVIC_EnableIRQ(USART2_IRQn);
}

/* Initialize the required peripheral resources in DMA1_Init. */
void DMA1_Init(void)
{
	/* Set up the required registers before starting the peripheral. */
	RCC->AHB1ENR |= DMA2EN;
}

/* Configure the UART interface in DMA_channel4_Stream5_UART_RX_config. */
void DMA_channel4_Stream5_UART_RX_config(uint8_t *data, uint16_t length)
{
    // Step 1: Ensure DMA stream is disabled
    DMA2_Stream5->CR = 0x00; // Reset the control register to disable
    while (DMA2_Stream5->CR & (1U << 0)) {} // Wait until the stream is disabled
    // Step 2: Clear any existing interrupt flags for Stream 5
    DMA2->HIFCR |= (1U << 8) | (1U << 9) | (1U << 11);
    // Step 3: Set the memory address
    DMA2_Stream5->M0AR = (uint32_t)data;
    // Step 4: Set the peripheral address (USART data register)
    DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);
    // Step 5: Set the number of data items to transfer
    DMA2_Stream5->NDTR = length;
    // Step 6: Set the channel to channel 4
    DMA2_Stream5->CR &= ~(7U << 25); // Clear channel selection bits
    DMA2_Stream5->CR |= (4U << 25);  // Set channel to 4
    // Step 7: Enable memory increment mode (increment address after each transfer)
    DMA2_Stream5->CR |= (1U << 10);
    // Step 8: Enable transfer complete interrupt
    DMA2_Stream5->CR |= (1U << 4);
    // Step 9: Enable transfer error interrupt
    DMA2_Stream5->CR |= (1U << 2);
    // Step 10: Set priority level (choose medium or high as required by system)
    DMA2_Stream5->CR |= (2U << 16); // High priority
    // Step 11: Set FIFO mode for stability (optional)
    DMA2_Stream5->FCR = (1U << 2); // Enable FIFO mode with half threshold
    // Step 12: Set transfer direction to Peripheral-to-Memory (required for RX)
    DMA2_Stream5->CR &= ~(3U << 6);
    // Step 13: Enable the DMA stream
    DMA2_Stream5->CR |= (1U << 0);
    // Step 14: Enable DMA interrupt in the NVIC
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}


/* Configure the UART interface in DMA_channel4_Stream6_UART_TX_config. */
void DMA_channel4_Stream6_UART_TX_config(uint8_t *data,uint16_t length)
{
	//disable DMA stream by setting all the register values equal to 0
	DMA2_Stream7->CR = 0x00;
	//wait until stream is disabled
	while(DMA2_Stream7->CR & (1U << 0)){}
	//clear the interrupt flags for stream 6
	DMA2->HIFCR |= (1U << 21) | (1U << 19) | (1U << 18);
	//set peripheral address
	DMA2_Stream7->PAR = (uint32_t)(&(USART1->DR));
	//set memory address
	DMA2_Stream7->M0AR = (uint32_t)data;
	/*set transfer length*/
	DMA2_Stream7->NDTR = length;//len of data to be transfered
	//set channel to channel 4.
	DMA2_Stream7->CR &= ~(1U << 25);
	DMA2_Stream7->CR &= ~(1U << 26);
	DMA2_Stream7->CR |= (1U << 27);
	//increment memory
	DMA2_Stream7->CR |= (1U << 10);
	//enable circular mode
	DMA2_Stream7->CR |= (1U << 8);
	//set the direction peripheral to memory
	DMA2_Stream7->CR |= (1U << 6);
	DMA2_Stream7->CR &= ~(1U << 7);
	//transfer complete interrupt enable
	DMA2_Stream7->CR |= (1U << 4);
	// Transfer error interrupt enable
	DMA2_Stream7->CR |= (1U << 2);
	//enable DMA stream
	DMA2_Stream7->CR |= (1U << 0);
	//enable DMA in NVIC
	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/* Calculate or program the UART timing in uart_set_baudrate. */
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Use the configured peripheral clock and requested baud rate. */
    USARTx->BRR = compute_uart_div(PeriphClk, BaudRate);
}

/* Calculate or program the UART timing in compute_uart_div. */
static uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Use the configured peripheral clock and requested baud rate. */
    return ((PeriphClk + (BaudRate / 2U)) / BaudRate);
}


