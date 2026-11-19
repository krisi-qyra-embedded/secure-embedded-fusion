/**
 * @file pm25_uart.h
 * @brief USART1 DMA receive path for the PMS5003 sensor.
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
/** @brief USART1 RX DMA completion event. */
typedef enum
{
   UART1_RX_DMA_EVENT_IDLE = 0U,
   UART1_RX_DMA_EVENT_DONE,
   UART1_RX_DMA_EVENT_ERROR
} UART1_RxDmaEvent_t;
/** @brief PM2.5 UART driver status. */
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
/* USART1 handle and DMA completion synchronization. */
extern UART_HandleTypeDef huart1;
extern SemaphoreHandle_t pm25_uart_rx_sem;
extern volatile UART1_RxDmaEvent_t g_uart1_rx_dma_event;
/**
 * @brief Initialize USART1 and DMA for PM2.5 reception.
 * @return UART status.
 */
UART_Status_t UART1_PM25_Init(void);
/**
 * @brief Recover the PM2.5 UART/DMA receive path.
 * @param app PM2.5 application context.
 * @return UART status.
 */
UART_Status_t UART1_PM25_Recover(PMS25_App_t *app);
/**
 * @brief Receive one PM2.5 frame block using DMA.
 * @param p_data Destination buffer.
 * @param length Number of bytes to receive.
 * @param timeout_ms Maximum wait time.
 * @return UART status.
 */
UART_Status_t UART1_RX_DMA_Receive(uint8_t *p_data,
                                  uint16_t length,
                                  uint32_t timeout_ms);
/**
 * @brief Get the tick of the most recent RX activity.
 * @return Last RX activity tick.
 */
uint32_t UART1_PM25_GetLastRxActivityTick(void);
/**
 * @brief Create the PM2.5 UART RTOS synchronization objects.
 * @return Initialization result.
 */
int UART1_PM25_RtosSyncInit(void);
#ifdef __cplusplus
}
#endif
#endif /* INC_PM25_UART_H_ */
