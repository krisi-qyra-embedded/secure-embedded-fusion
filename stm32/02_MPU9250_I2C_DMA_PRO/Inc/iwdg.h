/* iwdg.h | Author: i fiksuari pas prog | Date: 25 January 2026 */
#ifndef __IWDG_H__
#define __IWDG_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "systick.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Keys (common STM32 values) */
#define IWDG_KEY_ENABLE             0x0000CCCCU
#define IWDG_KEY_WR_ACCESS_ENABLE   0x00005555U
#define IWDG_KEY_RELOAD             0x0000AAAAU

#define SUPERVISOR_PERIOD_MS   50U
#define HEARTBEAT_WINDOW_MS    3000U
#define INTENTIONAL_RESET_TAG  0xDEADBEEFU

/* Prescaler choices (user should pass one of these) */
#define IWDG_PRESCALER_4            0x00000000U
#define IWDG_PRESCALER_8            0x00000001U
#define IWDG_PRESCALER_16           0x00000002U
#define IWDG_PRESCALER_32           0x00000003U
#define IWDG_PRESCALER_64           0x00000004U
#define IWDG_PRESCALER_128          0x00000005U
#define IWDG_PRESCALER_256          0x00000006U

/* Maximum reload for typical STM32 IWDG */
#define IWDG_RELOAD_MAX             0x0FFFU

/* Poll loop budget -- tune per system (higher => more tolerant of slow bus) */
#define IWDG_READY_POLL_MAX         1000000U

typedef enum {
    IWDG_OK = 0,
    IWDG_ERR_TIMEOUT,
    IWDG_ERR_INVALID_PARAM,
    IWDG_ERR_ALREADY_INIT,
    IWDG_ERR_HW,
} iwdg_status_t;

extern volatile uint32_t hb_mpu9250_ts;
extern volatile uint32_t hb_mpu9250main_ts;
 



iwdg_status_t iwdg_init(uint32_t prescaler, uint32_t reload, uint32_t poll_budget);

/* Kick the watchdog. Returns IWDG_OK on success */
iwdg_status_t iwdg_refresh(void);

 
bool iwdg_is_ready(void);

/* Optional: application can override this weak hook to handle/report errors.
   Signature: void IWDG_ErrorHandler(const char *msg, iwdg_status_t code) */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code);

void check_reset_source(void);

void supervisor_check_and_kick(void);

 inline void heartbeat_signal_bme680(void);
 inline void heartbeat_signal_bme680main(void);


#ifdef __cplusplus
}
#endif

#endif
