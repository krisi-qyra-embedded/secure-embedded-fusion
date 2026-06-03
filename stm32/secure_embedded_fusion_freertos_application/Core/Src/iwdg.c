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

/* ----------------------------- Heartbeat state ----------------------------- */

volatile uint32_t hb_bme680_ts;
volatile uint32_t hb_bme680main_ts;
volatile uint8_t iwdg_reset_requested = 0U;
volatile uint32_t hb_mpu9250_ts;
volatile uint32_t hb_mpu9250main_ts;

/* escalation helper: persist marker then stop kicking -> IWDG will reset */
/* Stops normal execution so the watchdog can reset the device. */
void escalate_to_iwdg_reset(const char *reason)
{
    /* Store the intentional-reset tag before entering the reset wait. */
    backup_write_marker(INTENTIONAL_RESET_TAG);

    iwdg_reset_requested = 1U;

    SYSTEM_LOG_PRINTF("ESCALATE -> IWDG reset armed: %s\r\n",
                      reason ? reason : "(no reason)");
    tim2_delay_ms(20U);

    __DSB();
    __ISB();

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskSuspend(NULL);

        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }

    for (;;) {
        __WFI();
    }
}

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

/* Reports whether the watchdog reset was intentionally requested. */
void check_reset_source(void)
{
    /* Compare the retained marker with the IWDG reset flag. */
    const uint32_t csr = RCC->CSR;
    const uint32_t marker = backup_read_marker();

    if ((csr & RCC_CSR_IWDGRSTF) != 0U) {
        if (marker == INTENTIONAL_RESET_TAG) {
            SYSTEM_LOG_PRINTF("Boot: intentional supervisor reset via IWDG\r\n");
            tim2_delay_ms(20U);
            backup_write_marker(0U);
        } else {
            SYSTEM_LOG_PRINTF("Boot: unexpected watchdog reset (crash)\r\n");
            tim2_delay_ms(20U);
        }
    }

    if ((csr & RCC_CSR_SFTRSTF) != 0U) {
        if (marker == INTENTIONAL_RESET_TAG) {
            SYSTEM_LOG_PRINTF("Boot: intentional supervisor software reset\r\n");
            tim2_delay_ms(20U);
            backup_write_marker(0U);
        } else {
            SYSTEM_LOG_PRINTF("Boot: unexpected software reset\r\n");
            tim2_delay_ms(20U);
        }
    }

    RCC->CSR |= RCC_CSR_RMVF;
}

/* Requests a supervised watchdog reset. */
void request_iwdg_reset(const char *reason)
{
    /* Preserve the first reset-request timestamp for the capture deadline. */
    backup_write_marker(INTENTIONAL_RESET_TAG);

    iwdg_reset_requested = 1U;

    SYSTEM_LOG_PRINTF("ESCALATE -> IWDG reset requested: %s\r\n",
                      reason ? reason : "(no reason)");
    tim2_delay_ms(20);
}

/* Checks system progress before refreshing the watchdog. */
void supervisor_check_and_kick(void)
{
    /* Escalate when a supervised heartbeat exceeds its allowed age. */
    uint32_t now;
    uint32_t age_bme680;
    uint32_t age_bme680main;
    uint32_t age_mpu9250;
    uint32_t age_mpu9250main;

    if (iwdg_reset_requested != 0U) {
        escalate_to_iwdg_reset("reset already requested");
        return;
    }

    now = ms_get();

    age_bme680      = now - hb_bme680_ts;
    age_bme680main  = now - hb_bme680main_ts;
    age_mpu9250     = now - hb_mpu9250_ts;
    age_mpu9250main = now - hb_mpu9250main_ts;

    if (age_bme680 > HEARTBEAT_WINDOW_MS) {
        SYSTEM_LOG_PRINTF("SUPERVISOR: BME680 I2C heartbeat missed (age=%u ms)\r\n",
                          (unsigned)age_bme680);
        tim2_delay_ms(20U);
        escalate_to_iwdg_reset("BME680 I2C heartbeat missed");
    }

    if (age_bme680main > HEARTBEAT_WINDOW_MS) {
        SYSTEM_LOG_PRINTF("SUPERVISOR: BME680 main/task heartbeat missed (age=%u ms)\r\n",
                          (unsigned)age_bme680main);
        tim2_delay_ms(20U);
        escalate_to_iwdg_reset("BME680 main/task heartbeat missed");
    }

    if (age_mpu9250 > HEARTBEAT_WINDOW_MS) {
        SYSTEM_LOG_PRINTF("SUPERVISOR: MPU9250 I2C heartbeat missed (age=%u ms)\r\n",
                          (unsigned)age_mpu9250);
        tim2_delay_ms(20U);
        escalate_to_iwdg_reset("MPU9250 I2C heartbeat missed");
    }

    if (age_mpu9250main > HEARTBEAT_WINDOW_MS) {
        SYSTEM_LOG_PRINTF("SUPERVISOR: MPU9250 main/task heartbeat missed (age=%u ms)\r\n",
                          (unsigned)age_mpu9250main);
        tim2_delay_ms(20U);
        escalate_to_iwdg_reset("MPU9250 main/task heartbeat missed");
    }

    if (iwdg_refresh() != IWDG_OK) {
        IWDG_ErrorHandler("iwdg_refresh failed", IWDG_ERR_HW);
        escalate_to_iwdg_reset("iwdg_refresh failed");
    }
}

/* Halts after an unrecoverable watchdog setup or refresh error. */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code)
{
	/* Order memory accesses before waiting for the watchdog reset. */
	SYSTEM_LOG_PRINTF("IWDG Error: %s (code=%d)\r\n", msg ? msg : "NULL", (int)code);
	tim2_delay_ms(20);

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
	SYSTEM_LOG_PRINTF("FATAL: %s -> escalating\r\n", msg ? msg : "(no reason)");
	tim2_delay_ms(20);
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
