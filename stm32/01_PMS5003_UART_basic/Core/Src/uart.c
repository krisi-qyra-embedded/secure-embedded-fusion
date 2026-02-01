/* uart.c | Author: i fiksuari pas prog | Date: 01 February 2026 */
#include "uart.h"

// Function to initialize UART1
UART_HandleTypeDef huart1; // UART handle

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
	if (HAL_UART_Init(&huart1) != HAL_OK)
	  {
	   // Initialization error handling
	   Error_Handler();
	  }
}

// Error Handler
void Error_Handler(void)
{
    /* Clear the handled event and update the transfer state. */
    while(1)
    {
        // Error loop
    }
}
