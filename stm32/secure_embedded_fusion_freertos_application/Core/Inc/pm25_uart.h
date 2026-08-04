/* Author: i fiksuari pas prog
 * Documentation date: 2026-08-18
 */

#ifndef INC_PM25_UART_H_
#define INC_PM25_UART_H_

#include "stm32f4xx_hal.h"
#include "pm2_5.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    UART1_RX_DMA_EVENT_IDLE = 0U,
    UART1_RX_DMA_EVENT_DONE,
    UART1_RX_DMA_EVENT_ERROR
} UART1_RxDmaEvent_t;

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

/*
 * FreeRTOS binary semaphore used by USART1 RX DMA completion path.
 *
 * Ownership/mapping:
 *   PM2.5 USART1_RX -> PA10 -> DMA2_Stream5 -> pm25_uart_rx_sem
 *
 * Given from:
 *   HAL_UART_RxCpltCallback()
 *   HAL_UART_ErrorCallback()
 */
extern SemaphoreHandle_t pm25_uart_rx_sem;
extern volatile UART1_RxDmaEvent_t g_uart1_rx_dma_event;
/* ----------------------------- Public API ----------------------------- */

UART_Status_t UART1_PM25_Init(void);
UART_Status_t UART1_PM25_Recover(PMS25_App_t *app);

UART_Status_t UART1_RX_DMA_Receive(uint8_t *p_data,
                                   uint16_t length,
                                   uint32_t timeout_ms);

uint32_t UART1_PM25_GetLastRxActivityTick(void);

/* ----------------------------- RTOS sync API ----------------------------- */

int UART1_PM25_RtosSyncInit(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_PM25_UART_H_ */
