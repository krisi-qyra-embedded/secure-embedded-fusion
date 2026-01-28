/* adc_dma.c | Author: i fiksuari mbas prog | Date: 28 January 2026 */
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


#define IWDG_KEY_ENABLE             0x0000CCCCU
#define IWDG_KEY_WR_ACCESS_ENABLE   0x00005555U
#define IWDG_KEY_RELOAD             0x0000AAAAU

#define BOOT_SUPERVISOR_GRACE_MS    12000U
#define HEARTBEAT_WINDOW_BME680_MS   3000U
#define HEARTBEAT_WINDOW_MAIN_MS     5000U

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


#define IWDG_RELOAD_MAX             0x0FFFU


#define IWDG_READY_POLL_MAX         1000000U

typedef enum {
    IWDG_OK = 0,
    IWDG_ERR_TIMEOUT,
    IWDG_ERR_INVALID_PARAM,
    IWDG_ERR_ALREADY_INIT,
    IWDG_ERR_HW,
} iwdg_status_t;

extern uint32_t g_boot_ms;
extern volatile uint8_t g_bme680_supervisor_armed;
extern volatile uint32_t hb_bme680_ts;
extern volatile uint32_t hb_bme680main_ts;
/* Initialize IWDG. Safe to call multiple times; subsequent calls return IWDG_ERR_ALREADY_INIT. prescaler. */
iwdg_status_t iwdg_init(uint32_t prescaler, uint32_t reload, uint32_t poll_budget);

/* Kick the watchdog. Returns IWDG_OK on success. */
iwdg_status_t iwdg_refresh(void);

/* Query readiness (non-blocking). */
bool iwdg_is_ready(void);

/* Optional: application can override this weak hook to handle/report errors. */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code);

void check_reset_source(void);

void supervisor_check_and_kick(void);

 inline void heartbeat_signal_bme680(void);
 inline void heartbeat_signal_bme680main(void);


#ifdef __cplusplus
}
#endif

#endif
