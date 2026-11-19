/**
 * @file iwdg.h
 * @brief IWDG supervision, heartbeat tracking and fault-reset declarations.
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
/* Shared USART2 logging mutex. */
extern SemaphoreHandle_t xSerialSemaphore;
/* Serialize supervisor logs once the scheduler is running. */
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
/* STM32 IWDG access and reload keys. */
#define IWDG_KEY_ENABLE             0x0000CCCCU
#define IWDG_KEY_WR_ACCESS_ENABLE   0x00005555U
#define IWDG_KEY_RELOAD             0x0000AAAAU
/* Supervisor timing and intentional-reset marker. */
#define SUPERVISOR_PERIOD_MS        50U
#define HEARTBEAT_WINDOW_MS         10000U
#define INTENTIONAL_RESET_TAG       0xDEADBEEFU
/* IWDG prescaler selections. */
#define IWDG_PRESCALER_4            0x00000000U
#define IWDG_PRESCALER_8            0x00000001U
#define IWDG_PRESCALER_16           0x00000002U
#define IWDG_PRESCALER_32           0x00000003U
#define IWDG_PRESCALER_64           0x00000004U
#define IWDG_PRESCALER_128          0x00000005U
#define IWDG_PRESCALER_256          0x00000006U
#define IWDG_RELOAD_MAX             0x0FFFU
#define IWDG_READY_POLL_MAX         1000000U
/* Fault-capture timing windows before watchdog escalation. */
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
/** @brief IWDG driver status. */
typedef enum
{
   IWDG_OK = 0,
   IWDG_ERR_TIMEOUT,
   IWDG_ERR_INVALID_PARAM,
   IWDG_ERR_ALREADY_INIT,
   IWDG_ERR_HW,
} iwdg_status_t;
/* Heartbeat timestamps monitored by the supervisor. */
extern volatile uint32_t hb_spi_ts;
extern volatile uint32_t hb_main_ts;
extern volatile uint32_t hb_bme680_ts;
extern volatile uint32_t hb_bme680main_ts;
extern volatile uint32_t hb_mpu9250_ts;
extern volatile uint32_t hb_mpu9250main_ts;
/* Pending watchdog-reset request state. */
extern volatile uint8_t  iwdg_reset_requested;
extern volatile uint32_t iwdg_reset_request_ms;
/**
 * @brief Initialize the independent watchdog.
 * @param prescaler IWDG prescaler selection.
 * @param reload Reload value up to IWDG_RELOAD_MAX.
 * @param poll_budget Maximum PVU/RVU wait budget, or 0 for default.
 * @return IWDG status.
 */
iwdg_status_t iwdg_init(uint32_t prescaler,
                       uint32_t reload,
                       uint32_t poll_budget);
/**
 * @brief Reload the independent watchdog.
 * @return IWDG status.
 */
iwdg_status_t iwdg_refresh(void);
/**
 * @brief Check whether IWDG register updates are complete.
 * @return True when the watchdog is ready.
 */
bool iwdg_is_ready(void);
/**
 * @brief Handle an IWDG-related application error.
 * @param msg Error message.
 * @param code IWDG status code.
 */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code);
/**
 * @brief Check and report the MCU reset source.
 */
void check_reset_source(void);
/**
 * @brief Initialize supervisor heartbeat timestamps.
 */
void supervisor_heartbeats_init(void);
/**
 * @brief Validate heartbeats and refresh the watchdog when healthy.
 */
void supervisor_check_and_kick(void);
/**
 * @brief Request a controlled watchdog reset.
 * @param reason Reset reason.
 */
void request_iwdg_reset(const char *reason);
/**
 * @brief Escalate a persistent fault to an IWDG reset.
 * @param reason Reset reason.
 */
void escalate_to_iwdg_reset(const char *reason);
/**
 * @brief Handle a fatal startup failure through the watchdog path.
 * @param msg Failure description.
 */
void on_fatal_init_error(const char *msg);
/**
 * @brief Check whether fault capture or a critical fault is active.
 * @return Non-zero while the system is in a protected fault window.
 */
uint8_t system_fault_capture_or_critical_fault_active(void);
/**
 * @brief Signal SPI/MAX31865 DMA activity to the supervisor.
 */
void heartbeat_signal_spi(void);
/**
 * @brief Signal a valid MAX31865 application sample.
 */
void heartbeat_signal_main(void);
/**
 * @brief Signal BME680 driver activity.
 */
void heartbeat_signal_bme680(void);
/**
 * @brief Signal a valid BME680 application sample.
 */
void heartbeat_signal_bme680main(void);
/**
 * @brief Signal MPU9250 driver activity.
 */
void heartbeat_signal_mpu9250(void);
/**
 * @brief Signal a valid MPU9250 application sample.
 */
void heartbeat_signal_mpu9250main(void);
#ifdef __cplusplus
}
#endif
#endif /* __IWDG_H__ */
