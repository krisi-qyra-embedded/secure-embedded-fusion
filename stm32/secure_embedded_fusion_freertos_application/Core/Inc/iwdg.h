/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef __IWDG_H__
#define __IWDG_H__
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c_dma.h"
#include "semphr.h"
#include "cmsis_os.h"
#ifdef __cplusplus
extern "C" {
#endif
extern SemaphoreHandle_t xSerialSemaphore;
#define SYSTEM_LOG_PRINTF(...)                                                     \
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
#define IWDG_KEY_ENABLE             0x0000CCCCU
#define IWDG_KEY_WR_ACCESS_ENABLE   0x00005555U
#define IWDG_KEY_RELOAD             0x0000AAAAU
#define SUPERVISOR_PERIOD_MS        50U
#define HEARTBEAT_WINDOW_MS         10000U
#define INTENTIONAL_RESET_TAG       0xDEADBEEFU
#define IWDG_PRESCALER_4            0x00000000U
#define IWDG_PRESCALER_8            0x00000001U
#define IWDG_PRESCALER_16           0x00000002U
#define IWDG_PRESCALER_32           0x00000003U
#define IWDG_PRESCALER_64           0x00000004U
#define IWDG_PRESCALER_128          0x00000005U
#define IWDG_PRESCALER_256          0x00000006U
#define IWDG_RELOAD_MAX             0x0FFFU
#define IWDG_READY_POLL_MAX         1000000U
#define FAULT_CAPTURE_GRACE_MS          (6000U)
#define FAULT_CAPTURE_ABSOLUTE_MAX_MS   (10000U)
#ifndef FAULT_CAPTURE_MIN_GRACE_MS
#define FAULT_CAPTURE_MIN_GRACE_MS      (2000U)
#endif
#ifndef FAULT_CAPTURE_ENROLL_MS
#define FAULT_CAPTURE_ENROLL_MS       (8000U)
#endif
#ifndef FAULT_CAPTURE_HARD_MAX_MS
#define FAULT_CAPTURE_HARD_MAX_MS     (60000U)
#endif
typedef enum
{
   IWDG_OK = 0,
   IWDG_ERR_TIMEOUT,
   IWDG_ERR_INVALID_PARAM,
   IWDG_ERR_ALREADY_INIT,
   IWDG_ERR_HW,
} iwdg_status_t;
extern volatile uint32_t hb_spi_ts;
extern volatile uint32_t hb_main_ts;
extern volatile uint32_t hb_bme680_ts;
extern volatile uint32_t hb_bme680main_ts;
extern volatile uint32_t hb_mpu9250_ts;
extern volatile uint32_t hb_mpu9250main_ts;
extern volatile uint8_t  iwdg_reset_requested;
extern volatile uint32_t iwdg_reset_request_ms;
iwdg_status_t iwdg_init(uint32_t prescaler,
                       uint32_t reload,
                       uint32_t poll_budget);
iwdg_status_t iwdg_refresh(void);
bool iwdg_is_ready(void);
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code);
void check_reset_source(void);
void supervisor_heartbeats_init(void);
void supervisor_check_and_kick(void);
void request_iwdg_reset(const char *reason);
void escalate_to_iwdg_reset(const char *reason);
void on_fatal_init_error(const char *msg);
uint8_t system_fault_capture_or_critical_fault_active(void);
void heartbeat_signal_spi(void);
void heartbeat_signal_main(void);
void heartbeat_signal_bme680(void);
void heartbeat_signal_bme680main(void);
void heartbeat_signal_mpu9250(void);
void heartbeat_signal_mpu9250main(void);
#ifdef __cplusplus
}
#endif
#endif /* __IWDG_H__ */
