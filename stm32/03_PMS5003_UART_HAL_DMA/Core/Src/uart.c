/* uart.c | Author: i fiksuari pas prog | Date: 08 February 2026 */
#include "uart.h"

// Function to initialize UART1
UART_HandleTypeDef huart1; // UART handle
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Configure USART1 for PMS5003 sensor receiving. */
void UART2_Init(void)
{
	// Enable the clocks for GPIOA and USART1
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();

	// Configure PA9 as TX and PA10 as RX for USART1
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// PA9 -> USART1_TX
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate function push-pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // Alternate function for USART1
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// PA10 -> USART1_RX
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate function push-pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // Alternate function for USART1
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	__HAL_RCC_DMA2_CLK_DISABLE();

	// USART1 configuration
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600; // Baud rate set to 9600
	huart1.Init.WordLength = UART_WORDLENGTH_8B; // 8 data bits
	huart1.Init.StopBits = UART_STOPBITS_1; // 1 stop bit
	huart1.Init.Parity = UART_PARITY_NONE; // No parity
	huart1.Init.Mode = UART_MODE_TX_RX; // Enable transmission and reception
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE; // No hardware flow control
	huart1.Init.OverSampling = UART_OVERSAMPLING_16; // Default oversampling
	// Initialize USART1 with the configured settings
	HAL_UART_Init(&huart1);
	//configure USART1_RX DMA
	hdma_usart1_rx.Instance = DMA2_Stream2;
	hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_usart1_rx);
	//link usart1 rx to dma
	huart1.hdmarx = &hdma_usart1_rx;

	hdma_usart1_tx.Instance = DMA2_Stream7;
	hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_tx.Init.Mode = DMA_NORMAL;
	hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_usart1_tx);
	//link usart1 rx to dma
	huart1.hdmatx = &hdma_usart1_tx;
///////////////////////////////////////////////////////
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn,0,0);
///////////////////////////////////////////////////////
//	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
//	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn,0,0);
}

/* Handle interrupt events in DMA2_Stream2_IRQHandler. */
void DMA2_Stream2_IRQHandler(void)
{
	/* Clear the handled event and update the transfer state. */
	HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/* Handle interrupt events in DMA2_Stream7_IRQHandler. */
void DMA2_Stream7_IRQHandler(void)
{
	/* Clear the handled event and update the transfer state. */
	HAL_DMA_IRQHandler(&hdma_usart1_tx);
}
