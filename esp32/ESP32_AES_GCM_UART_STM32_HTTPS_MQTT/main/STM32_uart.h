/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef STM32_UART_H_
#define STM32_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define STM32_GCM_WIRE_PACKET_SIZE    197U

#ifndef STM32_UART_PORT
#define STM32_UART_PORT                 UART_NUM_2
#endif

#ifndef STM32_UART_TX_GPIO
#define STM32_UART_TX_GPIO              GPIO_NUM_17
#endif

#ifndef STM32_UART_RX_GPIO
#define STM32_UART_RX_GPIO              GPIO_NUM_16
#endif

#ifndef STM32_UART_BAUDRATE
#define STM32_UART_BAUDRATE             9600
#endif

#ifndef STM32_UART_RX_BUF_SIZE
#define STM32_UART_RX_BUF_SIZE          1024
#endif

#ifndef STM32_UART_TX_BUF_SIZE
#define STM32_UART_TX_BUF_SIZE          1024
#endif

#ifndef STM32_UART_EVENT_QUEUE_SIZE
#define STM32_UART_EVENT_QUEUE_SIZE     20
#endif

#ifndef STM32_UART_EVENT_TASK_STACK
#define STM32_UART_EVENT_TASK_STACK     4096
#endif

#ifndef STM32_UART_EVENT_TASK_PRIORITY
#define STM32_UART_EVENT_TASK_PRIORITY  12
#endif

#ifndef STM32_UART_TEST_MAX_FRAME_SIZE
#define STM32_UART_TEST_MAX_FRAME_SIZE  256U
#endif

#if STM32_UART_TEST_MAX_FRAME_SIZE < STM32_GCM_WIRE_PACKET_SIZE
#error "STM32_UART_TEST_MAX_FRAME_SIZE must be at least 197 bytes"
#endif

typedef enum
{
    STM32_UART_STATUS_OK = 0,  
    STM32_UART_STATUS_ERROR,  
    STM32_UART_STATUS_INVALID_PARAM,  
    STM32_UART_STATUS_NOT_INITIALIZED,  
    STM32_UART_STATUS_TIMEOUT,  
    STM32_UART_STATUS_NO_DATA  

} STM32_UART_Status_t;

typedef struct
{
    uint32_t rx_bytes;  
    uint32_t tx_bytes;  
    uint32_t rx_frames;  
    uint32_t tx_frames;  
    uint32_t checksum_ok;  
    uint32_t checksum_bad;  
    uint32_t fifo_overflows;  
    uint32_t buffer_full;  
    uint32_t parity_errors;  
    uint32_t frame_errors;  
    uint32_t pattern_detected;  
    uint32_t unknown_events;  

} STM32_UART_Stats_t;

STM32_UART_Status_t STM32_UART_Init(void);

void STM32_UART_DeInit(void);

STM32_UART_Status_t STM32_UART_Write(
    const uint8_t *data,
    size_t len,
    uint32_t timeout_ms);

STM32_UART_Status_t STM32_UART_Read(
    uint8_t *data,
    size_t max_len,
    uint32_t timeout_ms,
    size_t *bytes_read);

int STM32_UART_RxAvailable(void);

void STM32_UART_FlushRx(void);

void STM32_UART_ResyncRx(void);

void STM32_UART_GetStats(
    STM32_UART_Stats_t *stats);

void STM32_UART_ClearStats(void);

STM32_UART_Status_t STM32_UART_LinkTest_Start(void);

void STM32_UART_LinkTest_PrintHealth(void);

bool STM32_UART_ReceiveEncryptedPacket(
    uint8_t *packet,
    size_t packet_capacity,
    size_t *packet_len,
    uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* STM32_UART_H_ */
