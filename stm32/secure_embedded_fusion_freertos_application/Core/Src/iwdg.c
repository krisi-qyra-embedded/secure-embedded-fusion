/* Author: i fiksuari pas prog
 * Documentation date: 2026-08-18
 */

#include "iwdg.h"
#include <stdio.h>   /* only for optional debug message in default handler */
#include "spi_dma.h"
#include "bme680_defs.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"


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

static volatile uint32_t fault_capture_mask = 0U;
static volatile uint8_t  fault_capture_locked = 0U;

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

/* ----------------------------- Fault state externs ----------------------------- */

extern volatile uint32_t system_fault_active_mask;
extern volatile uint32_t system_fault_completed_mask;
extern volatile uint8_t  system_fault_owner;
extern volatile uint32_t system_fault_in_progress_mask;

#ifndef SYSTEM_FAULT_MASK_BME680
#define SYSTEM_FAULT_MASK_BME680      (1UL << 0)
#endif

#ifndef SYSTEM_FAULT_MASK_MPU9250
#define SYSTEM_FAULT_MASK_MPU9250     (1UL << 1)
#endif

#ifndef SYSTEM_FAULT_MASK_MAX31865
#define SYSTEM_FAULT_MASK_MAX31865    (1UL << 2)
#endif

#ifndef SYSTEM_FAULT_MASK_PM25
#define SYSTEM_FAULT_MASK_PM25        (1UL << 3)
#endif

#ifndef SYSTEM_FAULT_MASK_DS18B20
#define SYSTEM_FAULT_MASK_DS18B20     (1UL << 4)
#endif

#ifndef SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM
#define SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM  (1UL << 5)
#endif

#ifndef SYSTEM_FAULT_MASK_CRITICAL
#define SYSTEM_FAULT_MASK_CRITICAL  \
    (SYSTEM_FAULT_MASK_BME680 | \
     SYSTEM_FAULT_MASK_MPU9250 | \
     SYSTEM_FAULT_MASK_MAX31865 | \
     SYSTEM_FAULT_MASK_PM25 | \
     SYSTEM_FAULT_MASK_DS18B20 | \
     SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM)
#endif


#ifndef FAULT_CAPTURE_GRACE_MS
#define FAULT_CAPTURE_GRACE_MS              (6000U)
#endif

#ifndef FAULT_CAPTURE_ABSOLUTE_MAX_MS
#define FAULT_CAPTURE_ABSOLUTE_MAX_MS       (12000U)
#endif

#ifndef FAULT_CAPTURE_ENROLL_MS
#define FAULT_CAPTURE_ENROLL_MS             FAULT_CAPTURE_GRACE_MS
#endif

#ifndef FAULT_CAPTURE_HARD_MAX_MS
#define FAULT_CAPTURE_HARD_MAX_MS           FAULT_CAPTURE_ABSOLUTE_MAX_MS
#endif

#ifndef IWDG_RESET_FALLBACK_MS
#define IWDG_RESET_FALLBACK_MS              (3000U)
#endif


/* Checks whether critical fault handling is active. */
uint8_t system_fault_capture_or_critical_fault_active(void)
{
    /* Combine active, in-progress and completed masks with the reset request. */
    uint32_t critical_faults;

    critical_faults =
        (system_fault_active_mask |
         system_fault_in_progress_mask |
         system_fault_completed_mask) & SYSTEM_FAULT_MASK_CRITICAL;

    if (iwdg_reset_requested != 0U) {
        return 1U;
    }

    if (critical_faults != 0U) {
        return 1U;
    }

    return 0U;
}

/* escalation helper: persist marker then stop kicking -> IWDG will reset */
/* Stops normal execution so the watchdog can reset the device. */
void escalate_to_iwdg_reset(const char *reason)
{
    /* Store the intentional-reset tag before entering the reset wait. */
    uint32_t start_ms;

    backup_write_marker(INTENTIONAL_RESET_TAG);

    log_printf("ESCALATE -> IWDG reset: %s\r\n", reason ? reason : "(no reason)");
    app_fault_delay_ms(2U);

    start_ms = ms_get();

    __DSB();
    __ISB();

    for (;;) {
        if ((ms_get() - start_ms) >= IWDG_RESET_FALLBACK_MS) {
            log_printf("[FATAL] IWDG reset fallback expired -> forcing NVIC_SystemReset()\r\n");
            app_fault_delay_ms(2U);

            __DSB();
            __ISB();
            NVIC_SystemReset();
        }

        __WFI();
    }
}
/* ----------------------------- Heartbeat state ----------------------------- */

volatile uint32_t hb_spi_ts;
volatile uint32_t hb_main_ts;

volatile uint32_t hb_bme680_ts;
volatile uint32_t hb_bme680main_ts;

volatile uint32_t hb_mpu9250_ts;
volatile uint32_t hb_mpu9250main_ts;

volatile uint32_t hb_esp32_uart_aes_gcm_ts;

volatile uint8_t  iwdg_reset_requested = 0U;
volatile uint32_t iwdg_reset_request_ms = 0U;

/* Seeds the supervised heartbeat timestamps. */
void supervisor_heartbeats_init(void)
{
    /* Use one millisecond snapshot for the initial heartbeat ages. */
    uint32_t now = ms_get();

    hb_spi_ts          = now;
    hb_main_ts         = now;

    hb_bme680_ts       = now;
    hb_bme680main_ts   = now;

    hb_mpu9250_ts      = now;
    hb_mpu9250main_ts  = now;

    hb_esp32_uart_aes_gcm_ts = now;
}

/*
 * MAX31865/SPI:
 *   heartbeat_signal_spi()  -> call from SPI DMA RX complete IRQ/path
 *   heartbeat_signal_main() -> call after a valid MAX31865 temperature sample
 */
/* Records progress in the MAX31865 SPI path. */
void heartbeat_signal_spi(void)
{
    /* Update the SPI heartbeat using the shared millisecond timebase. */
    hb_spi_ts = ms_get();
}

/* Records progress in the MAX31865 application path. */
void heartbeat_signal_main(void)
{
    /* Timestamp the application heartbeat separately from SPI activity. */
    hb_main_ts = ms_get();
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

/* Records progress in the encrypted ESP32 link task. */
void heartbeat_signal_esp32_uart_aes_gcm(void)
{
    /* Update the link heartbeat even when the task is waiting for data. */
    hb_esp32_uart_aes_gcm_ts = ms_get();
}

/* Reports whether the watchdog reset was intentionally requested. */
void check_reset_source(void)
{
    /* Compare the retained marker with the IWDG reset flag. */
    if ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U) {
        RCC->CSR |= RCC_CSR_RMVF;

        if (backup_read_marker() == INTENTIONAL_RESET_TAG) {
            log_printf("Boot: intentional supervisor reset (IWDG)\r\n");
            app_fault_delay_ms(2);
            backup_write_marker(0U);  
        } else {
            log_printf("Boot: unexpected watchdog reset (crash)\r\n");
            app_fault_delay_ms(2);
        }
    }
}

/* Requests a supervised watchdog reset. */
void request_iwdg_reset(const char *reason)
{
    /* Preserve the first reset-request timestamp for the capture deadline. */
    backup_write_marker(INTENTIONAL_RESET_TAG);

    if (iwdg_reset_requested == 0U) {
        iwdg_reset_request_ms = ms_get();

        fault_capture_mask = 0U;
        fault_capture_locked = 0U;
    }

    iwdg_reset_requested = 1U;

    SYSTEM_LOG_PRINTF("ESCALATE -> IWDG reset requested: %s\r\n",
                      reason ? reason : "(no reason)");
    app_fault_delay_ms(20U);
}

/* Counts the set bits in a fault mask. */
static uint32_t fault_popcount32(uint32_t x)
{
    /* Shift through the mask while accumulating its low bit. */
    uint32_t count = 0U;

    while (x != 0U) {
        count += (x & 1U);
        x >>= 1U;
    }

    return count;
}

/* Checks system progress before refreshing the watchdog. */
void supervisor_check_and_kick(void)
{
    /* Escalate when a supervised heartbeat exceeds its allowed age. */
    uint32_t now = ms_get();

    if (iwdg_reset_requested != 0U) {
        uint32_t age = now - iwdg_reset_request_ms;

        uint32_t active;
        uint32_t completed;
        uint32_t in_progress;
        uint32_t capture_completed;
        uint32_t capture_in_progress;
        uint32_t capture_count;

        taskENTER_CRITICAL();

        active =
            system_fault_active_mask & SYSTEM_FAULT_MASK_CRITICAL;

        completed =
            system_fault_completed_mask & SYSTEM_FAULT_MASK_CRITICAL;

        in_progress =
            system_fault_in_progress_mask & SYSTEM_FAULT_MASK_CRITICAL;

        if (fault_capture_locked == 0U) {
            fault_capture_mask |= active;

            if (age >= FAULT_CAPTURE_ENROLL_MS) {
                fault_capture_locked = 1U;
            }
        } else {
            fault_capture_mask |= active;
        }

        taskEXIT_CRITICAL();

        capture_completed =
            completed & fault_capture_mask;

        capture_in_progress =
            in_progress & fault_capture_mask;

        capture_count = fault_popcount32(fault_capture_mask);

        /*
         * Normal completion path:
         * single fault -> reset after that one completes
         * multi fault  -> reset after all captured sensors complete
         */
        if ((fault_capture_mask != 0U) &&
            ((capture_completed & fault_capture_mask) == fault_capture_mask) &&
            (capture_in_progress == 0U) &&
            (age >= FAULT_CAPTURE_ENROLL_MS)) {

            SYSTEM_LOG_PRINTF("[FAULT][COMPLETE] capture_mask=0x%08lX active=0x%08lX in_progress=0x%08lX completed=0x%08lX count=%lu age=%lu last_owner=%u\r\n",
                              (unsigned long)fault_capture_mask,
                              (unsigned long)active,
                              (unsigned long)in_progress,
                              (unsigned long)completed,
                              (unsigned long)capture_count,
                              (unsigned long)age,
                              (unsigned)system_fault_owner);
            app_fault_delay_ms(20U);

            if (capture_count > 1U) {
                escalate_to_iwdg_reset("multi fault capture complete");
            } else {
                escalate_to_iwdg_reset("single fault capture complete");
            }
        }

        if (age >= FAULT_CAPTURE_HARD_MAX_MS) {
            SYSTEM_LOG_PRINTF("[FAULT][HARD_TIMEOUT] capture_mask=0x%08lX active=0x%08lX in_progress=0x%08lX completed=0x%08lX count=%lu age=%lu last_owner=%u\r\n",
                              (unsigned long)fault_capture_mask,
                              (unsigned long)active,
                              (unsigned long)in_progress,
                              (unsigned long)completed,
                              (unsigned long)capture_count,
                              (unsigned long)age,
                              (unsigned)system_fault_owner);
            app_fault_delay_ms(20U);

            taskENTER_CRITICAL();
            system_fault_completed_mask |= fault_capture_mask;
            system_fault_in_progress_mask &= ~fault_capture_mask;
            taskEXIT_CRITICAL();

            escalate_to_iwdg_reset("fault capture hard timeout");
        }

        if (iwdg_refresh() != IWDG_OK) {
            IWDG_ErrorHandler("iwdg_refresh failed during fault capture", IWDG_ERR_HW);
            escalate_to_iwdg_reset("iwdg_refresh failed during fault capture");
        }

        return;
    }

    /* ----------------------------- Normal heartbeat supervision ----------------------------- */

    uint32_t age_spi         = now - hb_spi_ts;
    uint32_t age_main        = now - hb_main_ts;

    uint32_t age_bme680      = now - hb_bme680_ts;        
    uint32_t age_bme680main  = now - hb_bme680main_ts;

    uint32_t age_mpu9250     = now - hb_mpu9250_ts;
    uint32_t age_mpu9250main = now - hb_mpu9250main_ts;

    uint32_t age_esp32_uart_aes_gcm = now - hb_esp32_uart_aes_gcm_ts;

    if (age_spi > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: MAX31865 SPI/DMA heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_spi);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("MAX31865 SPI/DMA heartbeat missed");
    }

    if (age_main > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: MAX31865 main/task heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_main);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("MAX31865 main/task heartbeat missed");
    }

    if (age_bme680 > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: BME680 I2C heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_bme680);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("BME680 I2C heartbeat missed");
    }

    if (age_bme680main > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: BME680 main/task heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_bme680main);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("BME680 main/task heartbeat missed");
    }

    if (age_mpu9250 > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: MPU9250 I2C heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_mpu9250);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("MPU9250 I2C heartbeat missed");
    }

    if (age_mpu9250main > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: MPU9250 main/task heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_mpu9250main);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("MPU9250 main/task heartbeat missed");
    }

if (age_esp32_uart_aes_gcm > HEARTBEAT_WINDOW_MS) {
    log_printf(
        "SUPERVISOR: ESP32 UART AES-GCM heartbeat missed "
        "(age=%u ms)\r\n",
        (unsigned)age_esp32_uart_aes_gcm);

    app_fault_delay_ms(2U);

    escalate_to_iwdg_reset(
        "ESP32 UART AES-GCM heartbeat missed");
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
    app_fault_delay_ms(2);

    __DSB();
    __ISB();

    while (1) {
        __WFI();
    }
}

//void on_fatal_init_error(const char *msg)
//{
//    log_printf("FATAL: %s -> escalating\r\n", msg ? msg : "(no reason)");
//    app_fault_delay_ms(2);
//    escalate_to_iwdg_reset(msg);
//
//}

/* Reports a fatal application initialization error. */
void on_fatal_init_error(const char *msg)
{
    /* Pass the failure reason to the watchdog escalation path. */
    log_printf(
        "FATAL: %s -> NVIC system reset\r\n",
        msg ? msg : "(no reason)");

    app_fault_delay_ms(2U);

    /*
     * CMSIS software system reset through the Cortex-M AIRCR register.
     */
    NVIC_SystemReset();

    for (;;)
    {
        __NOP();
    }
}

 /* Resets the device after a failure before watchdog startup. */
 void app_pre_iwdg_fatal_reset(const char *reason)
{
    /* Drain the diagnostic message before requesting the system reset. */
    log_printf(
        "[FATAL][PRE-IWDG] %s -> NVIC system reset\r\n",
        (reason != NULL) ? reason : "(no reason)");

    app_fault_delay_ms(20U);

    __DSB();
    __ISB();
    NVIC_SystemReset();

    for (;;)
    {
        __NOP();
    }
}

///* Centralized error handling in main: replace all busy-wait error traps with this wrapper */
//void on_fatal_init_error(const char *msg)
//{
//    log_printf("FATAL: %s -> escalating\r\n", msg ? msg : "(no reason)");
//    app_fault_delay_ms(2);
//    escalate_to_iwdg_reset(msg);
//}

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
