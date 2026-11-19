/**
 * @file ESP32_uart.h
 * @brief USART6 transport, runtime state and encrypted ESP32 link declarations.
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
/* USART6 uses PC6 TX and PC7 RX on AF8. */
#ifndef ESP32_UART_BAUDRATE
#define ESP32_UART_BAUDRATE                     9600U
#endif
#ifndef ESP32_UART_RX_BUFFER_SIZE
#define ESP32_UART_RX_BUFFER_SIZE               512U
#endif
#ifndef ESP32_UART_TX_BUFFER_SIZE
#define ESP32_UART_TX_BUFFER_SIZE               512U
#endif
/* IRQ priority must remain FreeRTOS FromISR-safe. */
#ifndef ESP32_UART_IRQ_PRIORITY
#define ESP32_UART_IRQ_PRIORITY                 6U
#endif
#ifndef ESP32_UART_PROVIDE_IRQ_HANDLER
#define ESP32_UART_PROVIDE_IRQ_HANDLER          1U
#endif
/* Development packet dumps stay disabled by default. */
#ifndef ESP32_UART_DEBUG_PRINT_TX_ENCRYPTED_PACKET
#define ESP32_UART_DEBUG_PRINT_TX_ENCRYPTED_PACKET  0U
#endif
#ifndef ESP32_UART_DEBUG_PRINT_RX_ENCRYPTED_PACKET
#define ESP32_UART_DEBUG_PRINT_RX_ENCRYPTED_PACKET  0U
#endif
/* Bounded runtime retry policy for the encrypted link. */
#ifndef ESP32_UART_RUNTIME_MAX_ATTEMPTS
#define ESP32_UART_RUNTIME_MAX_ATTEMPTS         3U
#endif
#ifndef ESP32_UART_RUNTIME_RETRY_DELAY_MS
#define ESP32_UART_RUNTIME_RETRY_DELAY_MS       100U
#endif
/* Compact success logging without raw crypto dumps. */
#ifndef ESP32_UART_COMPACT_LOGS
#define ESP32_UART_COMPACT_LOGS                  1U
#endif
#ifndef ESP32_UART_HEALTH_EVERY_N_OK
#define ESP32_UART_HEALTH_EVERY_N_OK            10U
#endif
/* Shared fault slot assigned to the encrypted ESP32 UART link. */
#ifndef SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM
#define SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM   6U
#endif
#ifndef SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM
#define SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM    (1UL << 5)
#endif
/** @brief USART6 driver return status. */
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
/** @brief Interrupt-driven transfer completion state. */
typedef enum
{
   ESP32_UART_TRANSFER_IDLE = 0,
   ESP32_UART_TRANSFER_COMPLETE,
   ESP32_UART_TRANSFER_ERROR
} ESP32_UART_TransferState_t;
/** @brief Encrypted link runtime status. */
typedef enum
{
   ESP32_UART_RUNTIME_OK = 0,
   ESP32_UART_RUNTIME_FATAL,
   ESP32_UART_RUNTIME_BAD_PARAM
} ESP32_UART_RuntimeStatus_t;
/** @brief USART6 traffic and hardware error counters. */
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
/* Completion objects are created before the scheduler starts. */
extern SemaphoreHandle_t esp32_uart_rx_sem;
extern SemaphoreHandle_t esp32_uart_tx_sem;
extern volatile ESP32_UART_TransferState_t esp32_uart_rx_cmplt;
extern volatile ESP32_UART_TransferState_t esp32_uart_tx_cmplt;
/* Shared USART2 logging mutex. */
extern SemaphoreHandle_t xSerialSemaphore;
/* Serialize ESP32 UART logs without touching crypto/link state. */
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
/**
 * @brief Initialize the interrupt-driven USART6 link.
 * @return UART driver status.
 */
ESP32_UART_Status_t ESP32_UART_Init(void);
/**
 * @brief Deinitialize the USART6 link.
 */
void ESP32_UART_DeInit(void);
/**
 * @brief Queue data for USART6 transmission.
 * @param data Source buffer.
 * @param len Number of bytes to send.
 * @param timeout_ms Maximum wait time.
 * @return UART driver status.
 */
ESP32_UART_Status_t ESP32_UART_Write(
   const uint8_t *data,
   uint16_t len,
   uint32_t timeout_ms);
/**
 * @brief Wait for the current USART6 transmission to finish.
 * @param timeout_ms Maximum wait time.
 * @return UART driver status.
 */
ESP32_UART_Status_t ESP32_UART_WaitTxComplete(uint32_t timeout_ms);
/**
 * @brief Read bytes from the USART6 receive buffer.
 * @param data Destination buffer.
 * @param len Requested byte count.
 * @param timeout_ms Maximum wait time.
 * @param bytes_read Number of bytes returned.
 * @return UART driver status.
 */
ESP32_UART_Status_t ESP32_UART_Read(
   uint8_t *data,
   uint16_t len,
   uint32_t timeout_ms,
   uint16_t *bytes_read);
/**
 * @brief Read one byte from USART6.
 * @param byte Destination byte.
 * @param timeout_ms Maximum wait time.
 * @return UART driver status.
 */
ESP32_UART_Status_t ESP32_UART_ReadByte(
   uint8_t *byte,
   uint32_t timeout_ms);
/**
 * @brief Get the number of buffered RX bytes.
 * @return Available byte count.
 */
uint16_t ESP32_UART_RxAvailable(void);
/**
 * @brief Get the remaining TX buffer space.
 * @return Free byte count.
 */
uint16_t ESP32_UART_TxFreeSpace(void);
/**
 * @brief Clear the USART6 receive buffer.
 */
void ESP32_UART_FlushRx(void);
/**
 * @brief Clear the USART6 transmit buffer.
 */
void ESP32_UART_FlushTx(void);
/**
 * @brief Check whether USART6 is still transmitting.
 * @return True while transmission is active.
 */
bool ESP32_UART_IsTxBusy(void);
/**
 * @brief Copy the current USART6 statistics.
 * @param stats Statistics output.
 */
void ESP32_UART_GetStats(ESP32_UART_Stats_t *stats);
/**
 * @brief Clear the USART6 statistics counters.
 */
void ESP32_UART_ClearStats(void);
/**
 * @brief Initialize the encrypted full-duplex ESP32 link.
 * @return UART driver status.
 */
ESP32_UART_Status_t ESP32_UART_LinkTest_Init(void);
/**
 * @brief Run one authenticated telemetry transaction with the ESP32.
 * @param snapshot Fresh telemetry snapshot.
 * @return Encrypted-link runtime status.
 */
ESP32_UART_RuntimeStatus_t ESP32_UART_LinkTest_RunOnce(const telemetry_snapshot_t *snapshot);
/**
 * @brief Delay the ESP32 link task without busy-waiting.
 * @param delay_ms Delay in milliseconds.
 */
void ESP32_UART_LinkTest_DelayMs(uint32_t delay_ms);
/**
 * @brief Print the compact encrypted-link health summary.
 */
void ESP32_UART_LinkTest_PrintHealth(void);
/**
 * @brief Handle USART6 RX/TX and error interrupts.
 */
void ESP32_UART_IRQHandler(void);
/**
 * @brief Optional callback for RX buffer overflow.
 */
void ESP32_UART_RxOverflowCallback(void);
/**
 * @brief Optional callback for USART6 hardware errors.
 * @param usart_sr Captured USART status register.
 */
void ESP32_UART_ErrorCallback(uint32_t usart_sr);
#ifdef __cplusplus
}
#endif
#endif /* ESP32_UART_H_ */
