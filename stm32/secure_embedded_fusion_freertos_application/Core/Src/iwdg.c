/* Author: i fiksuari pas prog
 * Documentation date: 2026-05-28
 */

#include "iwdg.h"
#include <stdio.h>   /* only for optional debug message in default handler */
#include "spi_dma.h"
#include "bme680_defs.h"
#include "tim.h"


#ifndef READ_BIT
#define READ_BIT(REG, BIT)    ((((REG) & (BIT)) != 0U) ? (BIT) : 0U)
#endif

#ifndef IWDG_SR_PVU
#define IWDG_SR_PVU   (1U << 0)   /* PR update ongoing */
#endif
#ifndef IWDG_SR_RVU
#define IWDG_SR_RVU   (1U << 1)   /* RLR update ongoing */
#endif

static volatile bool iwdg_inited = false;
static uint32_t iwdg_poll_budget = IWDG_READY_POLL_MAX;
volatile uint32_t system_fault_last_progress_ms = 0U;

/* Stores the intentional-reset marker in the backup register. */
static inline void backup_write_marker(uint32_t tag)
{
    /* Use BKP0R when the target exposes that register. */
#ifdef RTC_BKP0R
    RTC->BKP0R = tag;
#else
    (void)tag;  
#endif
}

/* Reads the retained intentional-reset marker. */
static inline uint32_t backup_read_marker(void)
{
    /* Return zero when the backup register is unavailable. */
#ifdef RTC_BKP0R
    return RTC->BKP0R;
#else
    return 0U;  
#endif
}

/* escalation helper: persist marker then stop kicking -> IWDG will reset */
/* Stops normal execution so the watchdog can reset the device. */
void escalate_to_iwdg_reset(const char *reason)
{
    /* Store the intentional-reset tag before entering the reset wait. */
    backup_write_marker(INTENTIONAL_RESET_TAG);

    log_printf("ESCALATE -> IWDG reset: %s\r\n", reason ? reason : "(no reason)");
    tim2_delay_ms(2);

    __DSB();
    __ISB();

    for (;;) {
        __WFI();
    }
}

/* ----------------------------- Heartbeat state ----------------------------- */

volatile uint32_t hb_bme680_ts;
volatile uint32_t hb_bme680main_ts;
volatile uint8_t iwdg_reset_requested = 0U;
volatile uint32_t hb_mpu9250_ts;
volatile uint32_t hb_mpu9250main_ts;
volatile uint8_t  system_fault_pending_mask = 0U;

volatile uint8_t  system_fault_detected_mask = 0U;
volatile uint8_t  system_fault_completed_mask = 0U;
volatile uint8_t  system_fault_capture_active = 0U;
volatile uint32_t system_fault_capture_start_ms = 0U;

/* Seeds the supervised heartbeat timestamps. */
void supervisor_heartbeats_init(void)
{
    /* Use one millisecond snapshot for the initial heartbeat ages. */
    uint32_t now = ms_get();

    hb_bme680_ts       = now;
    hb_bme680main_ts   = now;
    hb_mpu9250_ts      = now;
    hb_mpu9250main_ts  = now;
}

/* Records BME680 peripheral progress. */
void heartbeat_signal_bme680(void)
{
    /* Refresh the bus heartbeat when BME680 activity is reported. */
    hb_bme680_ts = ms_get();
}

/* Records BME680 application progress. */
void heartbeat_signal_bme680main(void)
{
    /* Keep the task heartbeat distinct from its I2C heartbeat. */
    hb_bme680main_ts = ms_get();
}

/* Records MPU9250 peripheral progress. */
void heartbeat_signal_mpu9250(void)
{
    /* Timestamp progress in the sensor's I2C path. */
    hb_mpu9250_ts = ms_get();
}

/* Records MPU9250 application progress. */
void heartbeat_signal_mpu9250main(void)
{
    /* Refresh the task heartbeat independently of DMA completion. */
    hb_mpu9250main_ts = ms_get();
}

/* Starts or extends capture of an early sensor fault. */
void sensor_fault_detected_early(uint8_t fault_bit, const char *reason)
{
    /* Advance the progress timestamp only for newly detected faults. */
    uint8_t already_detected;

    already_detected = ((system_fault_detected_mask & fault_bit) != 0U) ? 1U : 0U;

    system_fault_detected_mask |= fault_bit;

    if (system_fault_capture_active == 0U) {
        system_fault_capture_active = 1U;
        system_fault_capture_start_ms = ms_get();
        system_fault_last_progress_ms = system_fault_capture_start_ms;

        log_printf("[FAULT_CAPTURE] early start: %s\r\n",
                   reason ? reason : "(no reason)");
        tim2_delay_ms(20U);

        log_printf("[FAULT_CAPTURE] detected_mask=0x%02X completed_mask=0x%02X\r\n",
                   (unsigned)system_fault_detected_mask,
                   (unsigned)system_fault_completed_mask);
        tim2_delay_ms(20U);

        return;
    }

    if (already_detected == 0U) {
        system_fault_last_progress_ms = ms_get();

        log_printf("[FAULT_CAPTURE] early additional: %s\r\n",
                   reason ? reason : "(no reason)");
        tim2_delay_ms(20U);

        log_printf("[FAULT_CAPTURE] detected_mask=0x%02X completed_mask=0x%02X\r\n",
                   (unsigned)system_fault_detected_mask,
                   (unsigned)system_fault_completed_mask);
        tim2_delay_ms(20U);
    }
}

/* Marks a sensor's fault diagnostics as complete. */
void sensor_fault_log_completed(uint8_t fault_bit, const char *reason)
{
    /* Record the intentional-reset tag before publishing completion. */
    uint8_t already_completed;

    backup_write_marker(INTENTIONAL_RESET_TAG);

    already_completed = ((system_fault_completed_mask & fault_bit) != 0U) ? 1U : 0U;

    system_fault_completed_mask |= fault_bit;

    if (already_completed == 0U) {
        system_fault_last_progress_ms = ms_get();

        log_printf("[FAULT_CAPTURE] completed: %s\r\n",
                   reason ? reason : "(no reason)");
        tim2_delay_ms(20U);

        log_printf("[FAULT_CAPTURE] detected_mask=0x%02X completed_mask=0x%02X\r\n",
                   (unsigned)system_fault_detected_mask,
                   (unsigned)system_fault_completed_mask);
        tim2_delay_ms(20U);
    }
}

/* Reports whether the watchdog reset was intentionally requested. */
void check_reset_source(void)
{
    /* Compare the retained marker with the IWDG reset flag. */
    if ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U) {
        RCC->CSR |= RCC_CSR_RMVF;

        if (backup_read_marker() == INTENTIONAL_RESET_TAG) {
            log_printf("Boot: intentional supervisor reset (IWDG)\r\n");
            tim2_delay_ms(2);
            backup_write_marker(0U);  
        } else {
            log_printf("Boot: unexpected watchdog reset (crash)\r\n");
            tim2_delay_ms(2);
        }
    }
}

/* Clears an early fault that recovered before completion. */
void sensor_fault_recovered_clear(uint8_t fault_bit, const char *reason)
{
    /* Leave faults with completed fatal diagnostics latched. */
    uint8_t was_detected;
    uint8_t was_completed;

    was_detected  = ((system_fault_detected_mask & fault_bit) != 0U) ? 1U : 0U;
    was_completed = ((system_fault_completed_mask & fault_bit) != 0U) ? 1U : 0U;

    if ((was_detected == 0U) || (was_completed != 0U)) {
        return;
    }

    system_fault_detected_mask &= (uint8_t)(~fault_bit);
    system_fault_completed_mask &= (uint8_t)(~fault_bit);

    log_printf("[FAULT_CAPTURE] recovered/cleared: %s\r\n",
               reason ? reason : "(no reason)");
    tim2_delay_ms(20U);

    log_printf("[FAULT_CAPTURE] detected_mask=0x%02X completed_mask=0x%02X\r\n",
               (unsigned)system_fault_detected_mask,
               (unsigned)system_fault_completed_mask);
    tim2_delay_ms(20U);

    if ((system_fault_detected_mask == 0U) &&
        (system_fault_completed_mask == 0U)) {

        system_fault_capture_active = 0U;
        system_fault_capture_start_ms = 0U;
        system_fault_last_progress_ms = 0U;

        log_printf("[FAULT_CAPTURE] cleared: no active sensor faults\r\n");
        tim2_delay_ms(20U);
    } else {
        system_fault_last_progress_ms = ms_get();
    }
}

/* Queues a sensor fault for watchdog reset handling. */
void request_sensor_fault_reset(uint8_t fault_bit, const char *reason)
{
    /* Start the capture window when the first pending fault is recorded. */
    backup_write_marker(INTENTIONAL_RESET_TAG);

    system_fault_pending_mask |= fault_bit;

    if (system_fault_capture_active == 0U) {
        system_fault_capture_active = 1U;
        system_fault_capture_start_ms = ms_get();

        SYSTEM_LOG_PRINTF("[FAULT_CAPTURE] started: %s\r\n",
                          reason ? reason : "(no reason)");
        tim2_delay_ms(20U);
    } else {
        SYSTEM_LOG_PRINTF("[FAULT_CAPTURE] additional fault: %s\r\n",
                          reason ? reason : "(no reason)");
        tim2_delay_ms(20U);
    }

    SYSTEM_LOG_PRINTF("[FAULT_CAPTURE] mask=0x%02X\r\n",
                      (unsigned)system_fault_pending_mask);
    tim2_delay_ms(20U);
}
/* Checks system progress before refreshing the watchdog. */
void supervisor_check_and_kick(void)
{
    /* Escalate when a supervised heartbeat exceeds its allowed age. */
    uint32_t now = ms_get();

    if (system_fault_capture_active != 0U) {
        uint32_t no_progress_age;

        no_progress_age = now - system_fault_last_progress_ms;

        if ((system_fault_detected_mask & SYSTEM_FAULT_ALL_SENSOR_BITS) ==
             SYSTEM_FAULT_ALL_SENSOR_BITS) {

            if ((system_fault_completed_mask & SYSTEM_FAULT_ALL_SENSOR_BITS) ==
                 SYSTEM_FAULT_ALL_SENSOR_BITS) {

                log_printf("[FAULT_CAPTURE] both detected and completed, detected=0x%02X completed=0x%02X\r\n",
                           (unsigned)system_fault_detected_mask,
                           (unsigned)system_fault_completed_mask);
                tim2_delay_ms(20U);

                escalate_to_iwdg_reset("both sensor fault logs completed");
            }
        }

        if (no_progress_age >= FAULT_CAPTURE_WINDOW_MS) {
            log_printf("[FAULT_CAPTURE] window expired, detected=0x%02X completed=0x%02X no_progress_age=%u ms\r\n",
                       (unsigned)system_fault_detected_mask,
                       (unsigned)system_fault_completed_mask,
                       (unsigned)no_progress_age);
            tim2_delay_ms(20U);

            escalate_to_iwdg_reset("fault capture window expired");
        }

        if (iwdg_refresh() != IWDG_OK) {
            IWDG_ErrorHandler("iwdg_refresh failed during fault capture", IWDG_ERR_HW);
            escalate_to_iwdg_reset("iwdg_refresh failed during fault capture");
        }

        return;
    }

    {
        uint32_t age_bme680      = now - hb_bme680_ts;
        uint32_t age_bme680main  = now - hb_bme680main_ts;
        uint32_t age_mpu9250     = now - hb_mpu9250_ts;
        uint32_t age_mpu9250main = now - hb_mpu9250main_ts;

        if (age_bme680 > HEARTBEAT_WINDOW_MS) {
            log_printf("SUPERVISOR: BME680 I2C heartbeat missed (age=%u ms)\r\n",
                       (unsigned)age_bme680);
            tim2_delay_ms(2U);
            escalate_to_iwdg_reset("BME680 I2C heartbeat missed");
        }

        if (age_bme680main > HEARTBEAT_WINDOW_MS) {
            log_printf("SUPERVISOR: BME680 main/task heartbeat missed (age=%u ms)\r\n",
                       (unsigned)age_bme680main);
            tim2_delay_ms(2U);
            escalate_to_iwdg_reset("BME680 main/task heartbeat missed");
        }

        if (age_mpu9250 > HEARTBEAT_WINDOW_MS) {
            log_printf("SUPERVISOR: MPU9250 I2C heartbeat missed (age=%u ms)\r\n",
                       (unsigned)age_mpu9250);
            tim2_delay_ms(2U);
            escalate_to_iwdg_reset("MPU9250 I2C heartbeat missed");
        }

        if (age_mpu9250main > HEARTBEAT_WINDOW_MS) {
            log_printf("SUPERVISOR: MPU9250 main/task heartbeat missed (age=%u ms)\r\n",
                       (unsigned)age_mpu9250main);
            tim2_delay_ms(2U);
            escalate_to_iwdg_reset("MPU9250 main/task heartbeat missed");
        }
    }

    /* Everything OK -> kick watchdog */
    if (iwdg_refresh() != IWDG_OK) {
        IWDG_ErrorHandler("iwdg_refresh failed", IWDG_ERR_HW);
        escalate_to_iwdg_reset("iwdg_refresh failed");
    }
}
/* Halts after an unrecoverable watchdog setup or refresh error. */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code)
{
    /* Order memory accesses before waiting for the watchdog reset. */
    log_printf("IWDG Error: %s (code=%d)\r\n", msg ? msg : "NULL", (int)code);
    tim2_delay_ms(2);

    __DSB();
    __ISB();

    while (1) {
        __WFI();
    }
}

/* Reports a fatal application initialization error. */
void on_fatal_init_error(const char *msg)
{
    /* Pass the failure reason to the watchdog escalation path. */
    log_printf("FATAL: %s -> escalating\r\n", msg ? msg : "(no reason)");
    tim2_delay_ms(2);
    escalate_to_iwdg_reset(msg);
}

/* Internal helper: wait for PVU/RVU == 0 with budget iterations */
/* Waits for watchdog prescaler and reload updates. */
static iwdg_status_t wait_iwdg_ready(uint32_t budget)
{
    /* Use elapsed milliseconds to bound the pending-update wait. */
    const uint32_t timeout_ms = (budget == 0U) ? iwdg_poll_budget : budget;
    const uint32_t start = ms_get();

    while (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) != 0U) {
        if ((ms_get() - start) >= timeout_ms) {
            return IWDG_ERR_TIMEOUT;
        }

        __DSB();
        __ISB();
        __WFI();
    }

    return IWDG_OK;
}

/* Public API */
/* Starts the independent watchdog with the requested settings. */
iwdg_status_t iwdg_init(uint32_t prescaler, uint32_t reload, uint32_t poll_budget)
{
    /* Wait for PR and RLR updates before issuing the first reload. */
    if (reload > IWDG_RELOAD_MAX) {
        IWDG_ErrorHandler("IWDG: invalid reload", IWDG_ERR_INVALID_PARAM);
        return IWDG_ERR_INVALID_PARAM;
    }

    if (iwdg_inited) {
        return IWDG_ERR_ALREADY_INIT;
    }

    if (poll_budget != 0U) {
        iwdg_poll_budget = poll_budget;
    }

    IWDG->KR = IWDG_KEY_ENABLE;

    __asm volatile ("nop");

    IWDG->KR = IWDG_KEY_WR_ACCESS_ENABLE;

    IWDG->PR  = prescaler;
    IWDG->RLR = reload;

    if (wait_iwdg_ready(0U) != IWDG_OK) {
        IWDG_ErrorHandler("IWDG: timeout waiting for PR/RLR update", IWDG_ERR_TIMEOUT);
        return IWDG_ERR_TIMEOUT;
    }

    IWDG->KR = IWDG_KEY_RELOAD;

#if defined(IWDG_RLR)
    if ((IWDG->RLR & IWDG_RELOAD_MAX) != (reload & IWDG_RELOAD_MAX)) {
        IWDG_ErrorHandler("IWDG: RLR readback mismatch", IWDG_ERR_HW);
    }
#endif

    iwdg_inited = true;
    return IWDG_OK;
}

/* Reloads an initialized independent watchdog. */
iwdg_status_t iwdg_refresh(void)
{
    /* Reject refresh before initialization and check pending register updates. */
    if (!iwdg_inited) {
        IWDG_ErrorHandler("IWDG: refresh called before init", IWDG_ERR_INVALID_PARAM);
        return IWDG_ERR_INVALID_PARAM;
    }

    IWDG->KR = IWDG_KEY_RELOAD;

    if (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) != 0U) {
        if (wait_iwdg_ready(iwdg_poll_budget / 10U) != IWDG_OK) {
            IWDG_ErrorHandler("IWDG: refresh caused SR pending", IWDG_ERR_TIMEOUT);
            return IWDG_ERR_TIMEOUT;
        }
    }

    return IWDG_OK;
}

/* Checks whether watchdog register updates have completed. */
bool iwdg_is_ready(void)
{
    /* Require both prescaler and reload update flags to be clear. */
    return (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) == 0U) ? true : false;
}
