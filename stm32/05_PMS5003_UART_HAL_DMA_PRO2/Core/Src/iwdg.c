/* iwdg.c | Author: i fiksuari pas prog | Date: 15 February 2026 */
#include "iwdg.h"
#include <stdio.h>   /* only for optional debug message in default handler */
/*#include "spi_dma.h"*/
/*#include "bme680_defs.h"*/
#include "systick.h"
#include "tim.h"

/* If READ_BIT macro unavailable, provide fallback */
#ifndef READ_BIT
#define READ_BIT(REG, BIT)    ((((REG) & (BIT)) != 0U) ? (BIT) : 0U)
#endif

/* SR flags (common names) */
#ifndef IWDG_SR_PVU
#define IWDG_SR_PVU   (1U << 0)   /* PR update ongoing */
#endif
#ifndef IWDG_SR_RVU
#define IWDG_SR_RVU   (1U << 1)   /* RLR update ongoing */
#endif

/* Internal state */
static volatile bool iwdg_inited = false;
static uint32_t iwdg_poll_budget = IWDG_READY_POLL_MAX;

/* persistent marker helpers - implement for your MCU (RTC BKP, backup SRAM, flash) */
static inline void backup_write_marker(uint32_t tag)
{
    /* Write the reset marker only when backup storage is available. */
#ifdef RTC_BKP0R
    /* Example for MCUs with RTC backup registers - replace with correct register names */
    RTC->BKP0R = tag;
#else
    (void)tag; /* TODO: implement persistent storage for production */
#endif
}

/* Read the stored reset marker when the target supports it. */
static inline uint32_t backup_read_marker(void)
{
    /* Return a default value if backup registers are unavailable. */
#ifdef RTC_BKP0R
    return RTC->BKP0R;
#else
    return 0U; /* TODO: implement persistent storage for production */
#endif
}

/* escalation helper: persist marker then stop kicking -> IWDG will reset */
void escalate_to_iwdg_reset(const char *reason)
{
    /* Persist intent so next boot can classify the reset */
    backup_write_marker(INTENTIONAL_RESET_TAG);

    /* Best-effort log */
    log_printf("ESCALATE -> IWDG reset: %s\r\n", reason ? reason : "(no reason)");
    delay_ms(2);
    /* Ensure memory/log side effects are committed before sleeping */
    __DSB();
    __ISB();

    /* Do NOT refresh IWDG anymore.
       Sleep between interrupts until watchdog resets the MCU. */
    for (;;) {
        __WFI();
    }
}

/* Global and heartbeat vars */
volatile uint32_t hb_bme680_ts;
volatile uint32_t hb_bme680main_ts;

/* Drivers must call this in DMA TC (place in your DMA completion callback) */
void heartbeat_signal_bme680(void)  {
    /* Validate the current state before continuing. */ hb_bme680_ts  = ms_get(); }
/* Process the heartbeat_signal_bme680main operation. */
void heartbeat_signal_bme680main(void) {
    /* Validate the current state before continuing. */ hb_bme680main_ts = ms_get(); }

/* YOUR original check_reset_source (kept, but clears persistent marker when intentional) */
void check_reset_source(void)
{
    /* Clear stale status before restarting the operation. */
    if ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U) {
        RCC->CSR |= RCC_CSR_RMVF;

        if (backup_read_marker() == INTENTIONAL_RESET_TAG) {
            log_printf("Boot: intentional supervisor reset (IWDG)\r\n");
            delay_ms(2);
            backup_write_marker(0U); /* clear */
        } else {
            log_printf("Boot: unexpected watchdog reset (crash)\r\n");
            delay_ms(2);
        }
    }
}

/* Minimal supervisor: only place that refreshes IWDG */
void supervisor_check_and_kick(void)
{
    /* Validate the current state before continuing. */
    uint32_t now      = ms_get();
    uint32_t age_bme680  = now - hb_bme680_ts;   /* wrap-safe unsigned subtraction */
    uint32_t age_bme680main = now - hb_bme680main_ts;

    /* Check SPI heartbeat (DMA completion must call heartbeat_signal_spi()) */
    if (age_bme680 > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: BME680 I2C heartbeat missed (age=%u ms)\r\n", (unsigned)age_bme680);
        delay_ms(2);
        /* we expect the driver's own recovery attempts to have been done already and returned error to main.
           If driver cannot recover, escalate. */
        escalate_to_iwdg_reset("BME680 I2C heartbeat missed");
    }

    /* Check main heartbeat */
    if (age_bme680main > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: main heartbeat missed (age=%u ms)\r\n", (unsigned)age_bme680main);
        delay_ms(2);
        escalate_to_iwdg_reset("main heartbeat missed");
    }

    /* Everything OK -> kick watchdog */
    if (iwdg_refresh() != IWDG_OK) {
        /* treat driver-level failure as fatal */
        IWDG_ErrorHandler("iwdg_refresh failed", IWDG_ERR_HW);
        escalate_to_iwdg_reset("iwdg_refresh failed");
    }
}

/* Minimal supervisor: only place that refreshes IWDG */
void supervisor_check_and_kick_ds18b20(void)
{
	/* Validate the current state before continuing. */

	if (iwdg_refresh() != IWDG_OK) {
	        log_printf("SUPERVISOR: iwdg_refresh failed\r\n");
	        delay_ms(2U);
	        escalate_to_iwdg_reset("iwdg_refresh failed");
	    }

}


/* Minimal supervisor: only place that refreshes IWDG */
void supervisor_check_and_kick_pm2_5(void)
{
	/* Validate the current state before continuing. */

	if (iwdg_refresh() != IWDG_OK) {
	        log_printf("SUPERVISOR: iwdg_refresh failed\r\n");
	        delay_ms(2U);
	        escalate_to_iwdg_reset("iwdg_refresh failed");
	    }

}



/* Override weak error hook from iwdg driver (application-specific handling) */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code)
{
    /* Clear the handled event and update the transfer state. */
    log_printf("IWDG Error: %s (code=%d)\r\n", msg ? msg : "NULL", (int)code);
    delay_ms(2U);
    __DSB();
    __ISB();

    while (1) {
        __WFI();
    }
}

/* Centralized error handling in main: replace all busy-wait error traps with this simple wrapper */
void on_fatal_init_error(const char *msg)
{
    /* Set up the required registers before starting the peripheral. */
    log_printf("FATAL: %s -> escalating\r\n", msg ? msg : "(no reason)");
    delay_ms(2);
    escalate_to_iwdg_reset(msg);
}

/* Internal helper: wait for PVU/RVU == 0 with budget iterations */
static iwdg_status_t wait_iwdg_ready(uint32_t budget)
{
    /* Refresh the watchdog only when the required checks pass. */
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
iwdg_status_t iwdg_init(uint32_t prescaler, uint32_t reload, uint32_t poll_budget)
{
    /* Parameter validation */
    if (reload > IWDG_RELOAD_MAX) {
        IWDG_ErrorHandler("IWDG: invalid reload", IWDG_ERR_INVALID_PARAM);

        return IWDG_ERR_INVALID_PARAM;
    }

    /* If already initialized, do not reconfigure (watchdog is typically one-shot) */
    if (iwdg_inited) {
        return IWDG_ERR_ALREADY_INIT;
    }

    if (poll_budget != 0U) {
        iwdg_poll_budget = poll_budget;
    }

    /* Enable IWDG (this will start the watchdog). On many STM32s this cannot be reversed. */
    IWDG->KR = IWDG_KEY_ENABLE;

    /* Give hardware a moment to take the enable */
    __asm volatile ("nop");

    /* Enable write access to PR and RLR */
    IWDG->KR = IWDG_KEY_WR_ACCESS_ENABLE;

    /* Program prescaler */
    IWDG->PR = prescaler;

    /* Program reload */
    IWDG->RLR = reload;

    /* Wait for hardware to accept new PR/RLR */
    if (wait_iwdg_ready(0U) != IWDG_OK) {
        IWDG_ErrorHandler("IWDG: timeout waiting for PR/RLR update", IWDG_ERR_TIMEOUT);
        return IWDG_ERR_TIMEOUT;
    }

    /* Final refresh to load counter with new reload value */
    IWDG->KR = IWDG_KEY_RELOAD;

    /* Quick verify: read back RLR if readable on this device (some parts may not allow readback) */
#if defined(IWDG_RLR)
    if ((IWDG->RLR & IWDG_RELOAD_MAX) != (reload & IWDG_RELOAD_MAX)) {
        /* Not all MCUs permit immediate readback; treat mismatch as warning but continue. */
        IWDG_ErrorHandler("IWDG: RLR readback mismatch", IWDG_ERR_HW);
        /* continue nonetheless; watchdog is running */
    }
#endif

    /* Mark as initialized so subsequent calls do not attempt reconfiguration */
    iwdg_inited = true;
    return IWDG_OK;
}

/* Handle watchdog supervision in iwdg_refresh. */
iwdg_status_t iwdg_refresh(void)
{
    /* Refresh is simply writing the reload key; ensure initialized first */
    if (!iwdg_inited) {
        IWDG_ErrorHandler("IWDG: refresh called before init", IWDG_ERR_INVALID_PARAM);
        return IWDG_ERR_INVALID_PARAM;
    }

    /* Write reload key (kick) */
    IWDG->KR = IWDG_KEY_RELOAD;

    /* Optional: quick check that SR does not indicate update pending */
    if (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) != 0U) {
        /* If pending, give short budget to complete; consider it non-fatal */
        if (wait_iwdg_ready(iwdg_poll_budget / 10U) != IWDG_OK) {
            IWDG_ErrorHandler("IWDG: refresh caused SR pending", IWDG_ERR_TIMEOUT);
            return IWDG_ERR_TIMEOUT;
        }
    }

    return IWDG_OK;
}

/* Handle watchdog supervision in iwdg_is_ready. */
bool iwdg_is_ready(void)
{
    /* Refresh the watchdog only when the required checks pass. */
    return (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) == 0U) ? true : false;
}
