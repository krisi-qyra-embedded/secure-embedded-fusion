/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef ESP32_UART_H_
#define ESP32_UART_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "telemetry.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#ifndef ESP32_UART_BAUDRATE
#define ESP32_UART_BAUDRATE                     9600U
#endif
#ifndef ESP32_UART_RX_BUFFER_SIZE
#define ESP32_UART_RX_BUFFER_SIZE               512U
#endif
#ifndef ESP32_UART_TX_BUFFER_SIZE
#define ESP32_UART_TX_BUFFER_SIZE               512U
#endif
#ifndef ESP32_UART_IRQ_PRIORITY
#define ESP32_UART_IRQ_PRIORITY                 6U
#endif
#ifndef ESP32_UART_PROVIDE_IRQ_HANDLER
#define ESP32_UART_PROVIDE_IRQ_HANDLER          1U
#endif
#ifndef ESP32_UART_DEBUG_PRINT_TX_ENCRYPTED_PACKET
#define ESP32_UART_DEBUG_PRINT_TX_ENCRYPTED_PACKET  0U
#endif
#ifndef ESP32_UART_DEBUG_PRINT_RX_ENCRYPTED_PACKET
#define ESP32_UART_DEBUG_PRINT_RX_ENCRYPTED_PACKET  0U
#endif
#ifndef ESP32_UART_RUNTIME_MAX_ATTEMPTS
#define ESP32_UART_RUNTIME_MAX_ATTEMPTS         3U
#endif
#ifndef ESP32_UART_RUNTIME_RETRY_DELAY_MS
#define ESP32_UART_RUNTIME_RETRY_DELAY_MS       100U
#endif
#ifndef ESP32_UART_COMPACT_LOGS
#define ESP32_UART_COMPACT_LOGS                  1U
#endif
#ifndef ESP32_UART_HEALTH_EVERY_N_OK
#define ESP32_UART_HEALTH_EVERY_N_OK            10U
#endif
#ifndef SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM
#define SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM   6U
#endif
#ifndef SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM
#define SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM    (1UL << 5)
#endif
typedef enum
{
   ESP32_UART_STATUS_OK = 0,
   ESP32_UART_STATUS_ERROR,
   ESP32_UART_STATUS_TIMEOUT,
   ESP32_UART_STATUS_BUSY,
   ESP32_UART_STATUS_INVALID_PARAM,
   ESP32_UART_STATUS_NOT_INITIALIZED,
   ESP32_UART_STATUS_OVERFLOW
} ESP32_UART_Status_t;
typedef enum
{
   ESP32_UART_TRANSFER_IDLE = 0,
   ESP32_UART_TRANSFER_COMPLETE,
   ESP32_UART_TRANSFER_ERROR
} ESP32_UART_TransferState_t;
typedef enum
{
   ESP32_UART_RUNTIME_OK = 0,
   ESP32_UART_RUNTIME_FATAL,
   ESP32_UART_RUNTIME_BAD_PARAM
} ESP32_UART_RuntimeStatus_t;
typedef struct
{
   uint32_t rx_bytes;
   uint32_t tx_bytes;
   uint32_t rx_overflows;
   uint32_t tx_overflows;
   uint32_t parity_errors;
   uint32_t framing_errors;
   uint32_t noise_errors;
   uint32_t overrun_errors;
} ESP32_UART_Stats_t;
extern SemaphoreHandle_t esp32_uart_rx_sem;
extern SemaphoreHandle_t esp32_uart_tx_sem;
extern volatile ESP32_UART_TransferState_t esp32_uart_rx_cmplt;
extern volatile ESP32_UART_TransferState_t esp32_uart_tx_cmplt;
extern SemaphoreHandle_t xSerialSemaphore;
#define ESP32_LOG_PRINTF(...)                                                \
   do {                                                                     \
       if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&           \
           (xSerialSemaphore != NULL)) {                                    \
           if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) ==     \
               pdTRUE) {                                                    \
               log_printf(__VA_ARGS__);                                     \
               (void)xSemaphoreGive(xSerialSemaphore);                      \
           }                                                                \
       } else {                                                             \
           log_printf(__VA_ARGS__);                                         \
       }                                                                    \
   } while (0)
ESP32_UART_Status_t ESP32_UART_Init(void);
void ESP32_UART_DeInit(void);
ESP32_UART_Status_t ESP32_UART_Write(
   const uint8_t *data,
   uint16_t len,
   uint32_t timeout_ms);
ESP32_UART_Status_t ESP32_UART_WaitTxComplete(uint32_t timeout_ms);
ESP32_UART_Status_t ESP32_UART_Read(
   uint8_t *data,
   uint16_t len,
   uint32_t timeout_ms,
   uint16_t *bytes_read);
ESP32_UART_Status_t ESP32_UART_ReadByte(
   uint8_t *byte,
   uint32_t timeout_ms);
uint16_t ESP32_UART_RxAvailable(void);
uint16_t ESP32_UART_TxFreeSpace(void);
void ESP32_UART_FlushRx(void);
void ESP32_UART_FlushTx(void);
bool ESP32_UART_IsTxBusy(void);
void ESP32_UART_GetStats(ESP32_UART_Stats_t *stats);
void ESP32_UART_ClearStats(void);
ESP32_UART_Status_t ESP32_UART_LinkTest_Init(void);
ESP32_UART_RuntimeStatus_t ESP32_UART_LinkTest_RunOnce(const telemetry_snapshot_t *snapshot);
void ESP32_UART_LinkTest_DelayMs(uint32_t delay_ms);
void ESP32_UART_LinkTest_PrintHealth(void);
void ESP32_UART_IRQHandler(void);
void ESP32_UART_RxOverflowCallback(void);
void ESP32_UART_ErrorCallback(uint32_t usart_sr);
#ifdef __cplusplus
}
#endif
#endif /* ESP32_UART_H_ */
