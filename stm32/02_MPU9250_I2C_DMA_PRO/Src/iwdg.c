/* iwdg.c | Author: i fiksuari pas prog | Date: 25 January 2026 */
#include "iwdg.h"
#include <stdio.h>   /* only for optional debug message in default handler */
#include "spi_dma.h"
#include "tim.h"

 
#ifndef READ_BIT
#define READ_BIT(REG, BIT)    ((((REG) & (BIT)) != 0U) ? (BIT) : 0U)
#endif

 
#ifndef IWDG_SR_PVU
#define IWDG_SR_PVU   (1U << 0)    
#endif
#ifndef IWDG_SR_RVU
#define IWDG_SR_RVU   (1U << 1)    
#endif

 
static volatile bool iwdg_inited = false;
static uint32_t iwdg_poll_budget = IWDG_READY_POLL_MAX;

 
/* Writes backup write marker to the selected interface. */
static inline void backup_write_marker(uint32_t tag)
{
    /* Check status before reporting a completed transfer. */
#ifdef RTC_BKP0R
    /* Example for MCUs with RTC backup registers - replace with correct register names */
    RTC->BKP0R = tag;
#else
    (void)tag;  
#endif
}

/* Reads backup read marker from the device or configured buffer. */
static inline uint32_t backup_read_marker(void)
{
    /* Check the transfer result before returning the data. */
#ifdef RTC_BKP0R
    return RTC->BKP0R;
#else
    return 0U;  
#endif
}

/* escalation helper: persist marker then stop kicking -> IWDG will reset */
/* Stops normal execution and allows the watchdog to reset the board. */
void escalate_to_iwdg_reset(const char *reason)
{
    /* Avoid another watchdog refresh after unrecoverable failure. */
    /* Persist intent so next boot can classify the reset */
    backup_write_marker(INTENTIONAL_RESET_TAG);

     
    log_printf("ESCALATE -> IWDG reset: %s\r\n", reason ? reason : "(no reason)");
    tim2_delay_ms(2);
    /* Ensure memory/log side effects are committed before sleeping */
    __DSB();
    __ISB();

    /* Do NOT refresh IWDG anymore. Sleep between interrupts until watchdog resets the MCU */
    for (;;) {
        __WFI();
    }
}

 
volatile uint32_t hb_mpu9250_ts;
volatile uint32_t hb_mpu9250main_ts;

/* Drivers must call this in DMA TC (place in your DMA completion callback) */
/* Updates the heartbeat timestamp for heartbeat signal mpu9250. */
void heartbeat_signal_mpu9250(void)  {
    /* Record the current tick to indicate task progress. */ hb_mpu9250_ts  = ms_get(); }
/* Updates the heartbeat timestamp for heartbeat signal mpu9250main. */
void heartbeat_signal_mpu9250main(void) {
    /* Record the current tick to indicate task progress. */ hb_mpu9250main_ts = ms_get(); }

 
/* Checks the MCU reset flags for watchdog and other reset causes. */
void check_reset_source(void)
{
    /* Read and clear reset flags after the cause is recorded. */
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

/* Minimal supervisor: only place that refreshes IWDG */
/* Checks the runtime heartbeat ages and refreshes the watchdog when healthy. */
void supervisor_check_and_kick(void)
{
    /* Allow a watchdog refresh only while the required activity is valid. */
    uint32_t now      = ms_get();
    uint32_t age_mpu9250  = now - hb_mpu9250_ts;    
    uint32_t age_mpu9250main = now - hb_mpu9250main_ts;

    /* Check SPI heartbeat (DMA completion must call heartbeat_signal_spi()) */
    if (age_mpu9250 > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: BME680 I2C heartbeat missed (age=%u ms)\r\n", (unsigned)age_mpu9250);
        tim2_delay_ms(2);
        /* we expect the driver's own recovery attempts to have been done already and returned error to main. */
        escalate_to_iwdg_reset("BME680 I2C heartbeat missed");
    }

     
    if (age_mpu9250main > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: main heartbeat missed (age=%u ms)\r\n", (unsigned)age_mpu9250main);
        tim2_delay_ms(2);
        escalate_to_iwdg_reset("main heartbeat missed");
    }

    /* Everything OK -> kick watchdog */
    if (iwdg_refresh() != IWDG_OK) {
        /* treat driver-level failure as fatal */
        IWDG_ErrorHandler("iwdg_refresh failed", IWDG_ERR_HW);
        escalate_to_iwdg_reset("iwdg_refresh failed");
    }
}

/* Override weak error hook from iwdg driver (application-specific handling) */
/* Handles the iwdg error handler operation. */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code)
{
    /* Check the operation status before continuing. */
     
    log_printf("IWDG Error: %s (code=%d)\r\n", msg ? msg : "NULL", (int)code);
    tim2_delay_ms(2);
    __DSB();
    __ISB();

    while (1) {
        __WFI();
    }
}

/* Centralized error handling in main: replace all busy-wait error traps with this simple wrapper */
/* Initializes on fatal init error for the requested operation. */
void on_fatal_init_error(const char *msg)
{
    /* Validate required parameters before applying the setup. */
    log_printf("FATAL: %s -> escalating\r\n", msg ? msg : "(no reason)");
    tim2_delay_ms(2);
    escalate_to_iwdg_reset(msg);
}

 
/* Waits for completion of wait iwdg ready with a timeout. */
static iwdg_status_t wait_iwdg_ready(uint32_t budget)
{
    /* Return a timeout rather than waiting indefinitely. */
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

 
/* Initializes the independent watchdog timeout and prescaler. */
iwdg_status_t iwdg_init(uint32_t prescaler, uint32_t reload, uint32_t poll_budget)
{
    /* Wait for watchdog registers to be ready before continuing. */
     
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

    /* Enable IWDG (this will start the watchdog). On many STM32s this cannot be reversed */
    IWDG->KR = IWDG_KEY_ENABLE;

     
    __asm volatile ("nop");

     
    IWDG->KR = IWDG_KEY_WR_ACCESS_ENABLE;

     
    IWDG->PR = prescaler;

     
    IWDG->RLR = reload;

     
    if (wait_iwdg_ready(0U) != IWDG_OK) {
        IWDG_ErrorHandler("IWDG: timeout waiting for PR/RLR update", IWDG_ERR_TIMEOUT);
        return IWDG_ERR_TIMEOUT;
    }

     
    IWDG->KR = IWDG_KEY_RELOAD;

     
#if defined(IWDG_RLR)
    if ((IWDG->RLR & IWDG_RELOAD_MAX) != (reload & IWDG_RELOAD_MAX)) {
         
        IWDG_ErrorHandler("IWDG: RLR readback mismatch", IWDG_ERR_HW);
        /* continue nonetheless; watchdog is running */
    }
#endif

     
    iwdg_inited = true;
    return IWDG_OK;
}

/* Refreshes the independent watchdog counter. */
iwdg_status_t iwdg_refresh(void)
{
    /* Refresh only after the application supervision checks. */
     
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

/* Reads iwdg is ready from the device or configured buffer. */
bool iwdg_is_ready(void)
{
    /* Check the transfer result before returning the data. */
    return (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) == 0U) ? true : false;
}
