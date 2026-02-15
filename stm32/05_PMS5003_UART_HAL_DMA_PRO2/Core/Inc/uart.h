/* uart.h | Author: i fiksuari pas prog | Date: 15 February 2026 */
#ifndef UART_H
#define UART_H

#include "stm32f4xx_hal.h"
#include "pm2_5.h"
#include <stdint.h>

typedef enum
{
    UART_STATUS_OK = 0U,
    UART_STATUS_INVALID_PARAM,
    UART_STATUS_INIT_ERROR,
    UART_STATUS_BUSY,
    UART_STATUS_DMA_ERROR,
    UART_STATUS_TIMEOUT,
    UART_STATUS_ERROR
} UART_Status_t;

extern UART_HandleTypeDef huart1;

UART_Status_t UART1_PM25_Init(void);
UART_Status_t UART1_PM25_Recover(PMS25_App_t *app);
UART_Status_t UART1_RX_DMA_Receive(uint8_t *p_data, uint16_t length, uint32_t timeout_ms);
uint32_t UART1_PM25_GetLastRxActivityTick(void);

#endif /* UART_H */
