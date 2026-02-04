/* uart.h | Author: i fiksuari pas prog | Date: 04 February 2026 */
#ifndef INC_UART_H_
#define INC_UART_H_
#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include <stdbool.h>
#include "stm32f411xe.h"
#include "stm32f4xx_hal.h"

void uart2_tx_rx_init(void);
#define UART_DATA_BUFF_SIZE 15
#define UART_DATA_BUFF_SIZE2 1
void DMA1_Init(void);
void DMA_channel4_Stream6_UART_TX_config(uint8_t *data,uint16_t length);
void DMA_channel4_Stream5_UART_RX_config(uint8_t *data,uint16_t length);


#endif /* INC_UART_H_ */
