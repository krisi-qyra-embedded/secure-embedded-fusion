/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#ifndef ESP32_UART_H_
#define ESP32_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f4xx.h"

/*
 * STM32F411 Nucleo:
 * USART6_TX = PC6, AF8
 * USART6_RX = PC7, AF8
 *
 * Same typical baudrate as PMS/PM2.5 UART sensors.
 * Change this if your PM2.5 driver uses another baudrate.
 */

//#define ESP32_UART_PROVIDE_IRQ_HANDLER 	0U

#ifndef ESP32_UART_BAUDRATE
#define ESP32_UART_BAUDRATE              9600U
#endif

#ifndef ESP32_UART_RX_BUFFER_SIZE
#define ESP32_UART_RX_BUFFER_SIZE        512U
#endif

#ifndef ESP32_UART_TX_BUFFER_SIZE
#define ESP32_UART_TX_BUFFER_SIZE        512U
#endif

#ifndef ESP32_UART_IRQ_PRIORITY
#define ESP32_UART_IRQ_PRIORITY          6U
#endif

#ifndef ESP32_UART_PROVIDE_IRQ_HANDLER
#define ESP32_UART_PROVIDE_IRQ_HANDLER   1U
#endif

typedef enum {
    ESP32_UART_STATUS_OK = 0,
    ESP32_UART_STATUS_ERROR,
    ESP32_UART_STATUS_TIMEOUT,
    ESP32_UART_STATUS_BUSY,
    ESP32_UART_STATUS_INVALID_PARAM,
    ESP32_UART_STATUS_NOT_INITIALIZED,
    ESP32_UART_STATUS_OVERFLOW
} ESP32_UART_Status_t;

typedef struct {
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t rx_overflows;
    uint32_t tx_overflows;
    uint32_t parity_errors;
    uint32_t framing_errors;
    uint32_t noise_errors;
    uint32_t overrun_errors;
} ESP32_UART_Stats_t;

ESP32_UART_Status_t ESP32_UART_Init(void);
void ESP32_UART_DeInit(void);

ESP32_UART_Status_t ESP32_UART_Write(const uint8_t *data,
                                     uint16_t len,
                                     uint32_t timeout_ms);

ESP32_UART_Status_t ESP32_UART_Read(uint8_t *data,
                                    uint16_t len,
                                    uint32_t timeout_ms,
                                    uint16_t *bytes_read);

ESP32_UART_Status_t ESP32_UART_ReadByte(uint8_t *byte,
                                        uint32_t timeout_ms);

uint16_t ESP32_UART_RxAvailable(void);
uint16_t ESP32_UART_TxFreeSpace(void);

void ESP32_UART_FlushRx(void);
void ESP32_UART_FlushTx(void);

bool ESP32_UART_IsTxBusy(void);

void ESP32_UART_GetStats(ESP32_UART_Stats_t *stats);
void ESP32_UART_ClearStats(void);

/* ------------------------------------------------------------
 * Bare-metal ESP32 UART link test API
 * ------------------------------------------------------------ */
ESP32_UART_Status_t ESP32_UART_LinkTest_Init(void);
void ESP32_UART_LinkTest_RunOnce(void);
void ESP32_UART_LinkTest_DelayMs(uint32_t delay_ms);
void ESP32_UART_LinkTest_PrintHealth(void);

/*
 * Call this from USART6_IRQHandler().
 * If ESP32_UART_PROVIDE_IRQ_HANDLER = 1, the .c file already provides USART6_IRQHandler().
 */
void ESP32_UART_IRQHandler(void);

void ESP32_UART_RxOverflowCallback(void);
void ESP32_UART_ErrorCallback(uint32_t usart_sr);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_UART_H_ */
