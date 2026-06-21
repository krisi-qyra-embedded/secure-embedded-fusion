/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

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

#define SUPERVISOR_PERIOD_MS        50U
#define HEARTBEAT_WINDOW_MS         7000U
#define INTENTIONAL_RESET_TAG       0xDEADBEEFU

#define IWDG_PRESCALER_4            0x00000000U
#define IWDG_PRESCALER_8            0x00000001U
#define IWDG_PRESCALER_16           0x00000002U
#define IWDG_PRESCALER_32           0x00000003U
#define IWDG_PRESCALER_64           0x00000004U
#define IWDG_PRESCALER_128          0x00000005U
#define IWDG_PRESCALER_256          0x00000006U

#define IWDG_RELOAD_MAX             0x0FFFU

/* Poll loop budget -- tune per system */
#define IWDG_READY_POLL_MAX         1000000U

typedef enum
{
    IWDG_OK = 0,
    IWDG_ERR_TIMEOUT,
    IWDG_ERR_INVALID_PARAM,
    IWDG_ERR_ALREADY_INIT,
    IWDG_ERR_HW,
} iwdg_status_t;

/* ----------------------------- Heartbeat globals ----------------------------- */

extern volatile uint32_t hb_spi_ts;
extern volatile uint32_t hb_main_ts;

extern volatile uint32_t hb_bme680_ts;
extern volatile uint32_t hb_bme680main_ts;

extern volatile uint32_t hb_mpu9250_ts;
extern volatile uint32_t hb_mpu9250main_ts;

/* ----------------------------- IWDG API ----------------------------- */

/*
 * Initialize IWDG.
 * Safe to call multiple times; subsequent calls return IWDG_ERR_ALREADY_INIT.
 *
 * prescaler: one of the IWDG_PRESCALER_* macros.
 * reload:    0..IWDG_RELOAD_MAX.
 * poll_budget: max wait budget for PVU/RVU to clear.
 *              Pass 0 to use default.
 */
iwdg_status_t iwdg_init(uint32_t prescaler,
                        uint32_t reload,
                        uint32_t poll_budget);

iwdg_status_t iwdg_refresh(void);

bool iwdg_is_ready(void);

/*
 * Optional application error handler.
 * Signature:
 *   void IWDG_ErrorHandler(const char *msg, iwdg_status_t code);
 */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code);

/* ----------------------------- Supervisor / reset API ----------------------------- */

void check_reset_source(void);

void supervisor_heartbeats_init(void);

void supervisor_check_and_kick(void);

void escalate_to_iwdg_reset(const char *reason);

void on_fatal_init_error(const char *msg);

/* ----------------------------- Heartbeat signal API ----------------------------- */

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
