/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef PM2_5_H
#define PM2_5_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#ifdef __cplusplus
extern "C" {
#endif
extern SemaphoreHandle_t xSerialSemaphore;
#ifndef PM25_LOG_PRINTF
#define PM25_LOG_PRINTF(...)                                                       \
   do {                                                                           \
       if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&                 \
           (xSerialSemaphore != NULL)) {                                          \
           if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) == pdTRUE) {  \
               log_printf(__VA_ARGS__);                                           \
               xSemaphoreGive(xSerialSemaphore);                                  \
           }                                                                      \
       } else {                                                                   \
           log_printf(__VA_ARGS__);                                               \
       }                                                                          \
   } while (0)
#endif
#define PM25_SYSTICK_CORE_HZ             96000000U
#define PM25_SYSTICK_DIV                 6U
#define PM25_FRAME_POLL_TIMEOUT_MS       1200U
#define PM25_NO_FRAME_FAULT_MS           2000U
#define PM25_SAMPLE_PERIOD_MS            1000U
#define PM25_RECOVERY_DELAY_MS           100U
#define PM25_MAX_RECOVERY_ATTEMPTS       3U
#define PM25_LOG_FLUSH_DELAY_MS          100U
#define PM25_IWDG_RELOAD_VALUE           2499U
typedef enum
{
   PMS25_STATUS_OK = 0U,
   PMS25_STATUS_INVALID_PARAM,
   PMS25_STATUS_TIMEOUT,
   PMS25_STATUS_UART_ERROR,
   PMS25_STATUS_SYNC_ERROR,
   PMS25_STATUS_LENGTH_ERROR,
   PMS25_STATUS_CHECKSUM_ERROR
} PMS25_Status_t;
typedef struct
{
   uint16_t frame_len;
   uint16_t pm1_0_standard;
   uint16_t pm2_5_standard;
   uint16_t pm10_standard;
   uint16_t pm1_0_atmospheric;
   uint16_t pm2_5_atmospheric;
   uint16_t pm10_atmospheric;
   uint16_t particles_0_3um;
   uint16_t particles_0_5um;
   uint16_t particles_1_0um;
   uint16_t particles_2_5um;
   uint16_t particles_5_0um;
   uint16_t particles_10um;
   uint16_t reserved;
   uint16_t checksum;
} PMS25_Data_t;
typedef enum
{
   PMS25_APP_RUNTIME_OK = 0U,
   PMS25_APP_RUNTIME_BAD_PARAM,
   PMS25_APP_RUNTIME_READ_FAIL,
   PMS25_APP_RUNTIME_FATAL
} PMS25_AppRuntimeStatus_t;
typedef struct
{
   PMS25_Data_t Data;
   uint32_t FramePollTimeoutMs;
   uint32_t NoFrameFaultMs;
   uint32_t SamplePeriodMs;
   uint32_t RecoveryDelayMs;
   uint8_t  MaxRecoveryAttempts;
   uint8_t  RecoveryAttempt;
   uint32_t LastFrameAvailableTick;
} PMS25_App_t;
PMS25_Status_t PMS25_Read(PMS25_Data_t *out, uint32_t timeout_ms);
const char *PMS25_StatusStr(PMS25_Status_t st);
PMS25_AppRuntimeStatus_t PMS25_AppInit(PMS25_App_t *app,
                                      uint32_t frame_poll_timeout_ms,
                                      uint32_t no_frame_fault_ms,
                                      uint32_t sample_period_ms,
                                      uint32_t recovery_delay_ms,
                                      uint8_t max_recovery_attempts);
PMS25_AppRuntimeStatus_t PMS25_AppRun(PMS25_App_t *app);
#ifdef __cplusplus
}
#endif
#endif /* PM2_5_H */
