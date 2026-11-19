/**
 * @file iwdg.c
 * @brief IWDG supervision, heartbeat checks and fault reset handling.
 */
#include "iwdg.h"
#include <stdio.h>   /* debug logging */
#include "spi_dma.h"
#include "bme680_defs.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * Combined watchdog / supervisor module for:
 *   - MAX31865 SPI/DMA heartbeat
 *   - MAX31865 main/task heartbeat
 *   - BME680  I2C heartbeat
 *   - MPU9250 I2C heartbeat
 *   - BME680 main/task heartbeat
 *   - MPU9250 main/task heartbeat
 *   - PM2.5 fault-capture participation (no heartbeat supervision)
 *   - DS18B20 fault-capture participation (no heartbeat supervision)
 *   - ESP32 UART AES-GCM fault-capture participation
 *     (progress timestamp recorded, no normal heartbeat enforcement)
 *
 * Design rule:
 *   supervisor_check_and_kick() is the only place that refreshes the IWDG.
 *   If any monitored heartbeat becomes stale, the supervisor stops refreshing
 *   the watchdog and lets the IWDG reset the MCU.
 */
/* READ_BIT fallback. */
#ifndef READ_BIT
#define READ_BIT(REG, BIT)    ((((REG) & (BIT)) != 0U) ? (BIT) : 0U)
#endif

/* IWDG status flags. */
#ifndef IWDG_SR_PVU
#define IWDG_SR_PVU   (1U << 0)   /* PR update ongoing */
#endif
#ifndef IWDG_SR_RVU
#define IWDG_SR_RVU   (1U << 1)   /* RLR update ongoing */
#endif

/* Internal state */
static volatile bool iwdg_inited = false;
static uint32_t iwdg_poll_budget = IWDG_READY_POLL_MAX;
static volatile uint32_t fault_capture_mask = 0U;
static volatile uint8_t  fault_capture_locked = 0U;

/* Save intentional-reset marker when backup register is available. */
static inline void backup_write_marker(uint32_t tag)
{
#ifdef RTC_BKP0R
    /* RTC backup register path. */
    RTC->BKP0R = tag;
#else
    (void)tag; /* no persistent marker on this target */
#endif
}

/* Read intentional-reset marker. */
static inline uint32_t backup_read_marker(void)
{
#ifdef RTC_BKP0R
    return RTC->BKP0R;
#else
    return 0U; /* no persistent marker on this target */
#endif
}

/* Shared fault state */
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
#define SYSTEM_FAULT_MASK_CRITICAL  \\
    (SYSTEM_FAULT_MASK_BME680 | \\
     SYSTEM_FAULT_MASK_MPU9250 | \\
     SYSTEM_FAULT_MASK_MAX31865 | \\
     SYSTEM_FAULT_MASK_PM25 | \\
     SYSTEM_FAULT_MASK_DS18B20 | \\
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

/**
 * @brief Check if critical fault capture is active.
 *
 * @return 1 when active, otherwise 0.
 */
uint8_t system_fault_capture_or_critical_fault_active(void)
{
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

/**
 * @brief Stop normal execution and force watchdog reset.
 *
 * @param reason Reset reason used in logs.
 */
void escalate_to_iwdg_reset(const char *reason)
{
    uint32_t start_ms;
    backup_write_marker(INTENTIONAL_RESET_TAG);
    log_printf("ESCALATE -> IWDG reset: %s\r\n", reason ? reason : "(no reason)");
    app_fault_delay_ms(2U);
    start_ms = ms_get();
    __DSB();
    __ISB();
    /*
     * Preferred path:
     * Stop refreshing IWDG and let the hardware watchdog reset the MCU.
     */
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

/* Heartbeat state */

/* MAX31865 */
volatile uint32_t hb_spi_ts;
volatile uint32_t hb_main_ts;

/* BME680 */
volatile uint32_t hb_bme680_ts;
volatile uint32_t hb_bme680main_ts;

/* MPU9250 */
volatile uint32_t hb_mpu9250_ts;
volatile uint32_t hb_mpu9250main_ts;

/*
 * Encrypted UART progress timestamp.
 *
 * This is intentionally diagnostic only. Transport failures use the bounded
 * three-attempt recovery path and coordinated fault capture instead of an
 * independent heartbeat reset race.
 */
volatile uint32_t hb_esp32_uart_aes_gcm_ts;

/* Fatal reset request */
volatile uint8_t  iwdg_reset_requested = 0U;
volatile uint32_t iwdg_reset_request_ms = 0U;

/**
 * @brief Initialize all heartbeat timestamps.
 */
void supervisor_heartbeats_init(void)
{
    uint32_t now = ms_get();
    hb_spi_ts          = now;
    hb_main_ts         = now;
    hb_bme680_ts       = now;
    hb_bme680main_ts   = now;
    hb_mpu9250_ts      = now;
    hb_mpu9250main_ts  = now;
    hb_esp32_uart_aes_gcm_ts = now;
}

/* Heartbeat update helpers. */

/* Update MAX31865 SPI heartbeat. */
void heartbeat_signal_spi(void)
{
    hb_spi_ts = ms_get();
}

/* Update MAX31865 task heartbeat. */
void heartbeat_signal_main(void)
{
    hb_main_ts = ms_get();
}

/* Update BME680 driver heartbeat. */
void heartbeat_signal_bme680(void)
{
    hb_bme680_ts = ms_get();
}

/* Update BME680 task heartbeat. */
void heartbeat_signal_bme680main(void)
{
    hb_bme680main_ts = ms_get();
}

/* Update MPU9250 driver heartbeat. */
void heartbeat_signal_mpu9250(void)
{
    hb_mpu9250_ts = ms_get();
}

/* Update MPU9250 task heartbeat. */
void heartbeat_signal_mpu9250main(void)
{
    hb_mpu9250main_ts = ms_get();
}

/* Update ESP32 encrypted-link heartbeat. */
void heartbeat_signal_esp32_uart_aes_gcm(void)
{
    hb_esp32_uart_aes_gcm_ts = ms_get();
}

/**
 * @brief Check if the last reset came from IWDG.
 */
void check_reset_source(void)
{
    if ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U) {
        RCC->CSR |= RCC_CSR_RMVF;
        if (backup_read_marker() == INTENTIONAL_RESET_TAG) {
            log_printf("Boot: intentional supervisor reset (IWDG)\r\n");
            app_fault_delay_ms(2);
            backup_write_marker(0U); /* clear marker */
        } else {
            log_printf("Boot: unexpected watchdog reset (crash)\r\n");
            app_fault_delay_ms(2);
        }
    }
}

/**
 * @brief Request coordinated IWDG reset after fault capture.
 *
 * @param reason Reset reason used in logs.
 */
void request_iwdg_reset(const char *reason)
{
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

/* Count active bits in a fault mask. */
static uint32_t fault_popcount32(uint32_t x)
{
    uint32_t count = 0U;
    while (x != 0U) {
        count += (x & 1U);
        x >>= 1U;
    }
    return count;
}

/**
 * @brief Run fault capture and heartbeat supervision.
 *
 * This is the normal IWDG refresh point.
 */
void supervisor_check_and_kick(void)
{
    uint32_t now = ms_get();
    /*
     * Fatal reset was requested by one sensor task.
     *
     * no reset immediately. Keeping the IWDG alive for a short bounded grace
     * window so the other critical sensor task can also finish and print its
     * 3/3 fault chain. Reset immediately if all currently active critical
     * sensors have completed their fatal reporting.
     */
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
        /*
         * During enrollment, keep adding newly detected faulty sensors.
         * This fixes the MPU-first/BME-second race.
         */
        if (fault_capture_locked == 0U) {
            fault_capture_mask |= active;
            if (age >= FAULT_CAPTURE_ENROLL_MS) {
                fault_capture_locked = 1U;
            }
        } else {
            /*
             * Even after lock, still allow already-faulting sensors to be included.
             * This protects against task scheduling latency.
             */
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
        /*
         * Hard safety path:
         * Do not let the system freeze forever if one task never clears in_progress.
         */
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
            /*
             * Safety cleanup before reset: if one captured task got trapped in
             * in_progress, do not keep waiting. We are already in hard-timeout.
             */
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
    /* Normal heartbeat checks */
    uint32_t age_spi         = now - hb_spi_ts;
    uint32_t age_main        = now - hb_main_ts;
    uint32_t age_bme680      = now - hb_bme680_ts;       /* wrap-safe */
    uint32_t age_bme680main  = now - hb_bme680main_ts;
    uint32_t age_mpu9250     = now - hb_mpu9250_ts;
    uint32_t age_mpu9250main = now - hb_mpu9250main_ts;
    uint32_t age_esp32_uart_aes_gcm = now - hb_esp32_uart_aes_gcm_ts;
    /* MAX31865 SPI/DMA */
    if (age_spi > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: MAX31865 SPI/DMA heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_spi);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("MAX31865 SPI/DMA heartbeat missed");
    }
    /* MAX31865 task */
    if (age_main > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: MAX31865 main/task heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_main);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("MAX31865 main/task heartbeat missed");
    }
    /* BME680 driver */
    if (age_bme680 > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: BME680 I2C heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_bme680);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("BME680 I2C heartbeat missed");
    }
    /* BME680 task */
    if (age_bme680main > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: BME680 main/task heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_bme680main);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("BME680 main/task heartbeat missed");
    }
    /* MPU9250 driver */
    if (age_mpu9250 > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: MPU9250 I2C heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_mpu9250);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("MPU9250 I2C heartbeat missed");
    }
    /* MPU9250 task */
    if (age_mpu9250main > HEARTBEAT_WINDOW_MS) {
        log_printf("SUPERVISOR: MPU9250 main/task heartbeat missed (age=%u ms)\r\n",
                   (unsigned)age_mpu9250main);
        app_fault_delay_ms(2);
        escalate_to_iwdg_reset("MPU9250 main/task heartbeat missed");
    }
    /* ESP32 encrypted link */
if (age_esp32_uart_aes_gcm > HEARTBEAT_WINDOW_MS) {
    log_printf(
        "SUPERVISOR: ESP32 UART AES-GCM heartbeat missed "
        "(age=%u ms)\r\n",
        (unsigned)age_esp32_uart_aes_gcm);
    app_fault_delay_ms(2U);
    escalate_to_iwdg_reset(
        "ESP32 UART AES-GCM heartbeat missed");
    }
    /* All checks passed, refresh watchdog. */
    if (iwdg_refresh() != IWDG_OK) {
        IWDG_ErrorHandler("iwdg_refresh failed", IWDG_ERR_HW);
        escalate_to_iwdg_reset("iwdg_refresh failed");
    }
}

/**
 * @brief Handle IWDG driver errors.
 *
 * @param msg Error message.
 * @param code IWDG status code.
 */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code)
{
    log_printf("IWDG Error: %s (code=%d)\r\n", msg ? msg : "NULL", (int)code);
    app_fault_delay_ms(2);
    __DSB();
    __ISB();
    while (1) {
        __WFI();
    }
}

/* Fatal init reset helper. */
//void on_fatal_init_error(const char *msg)
//{
//    log_printf("FATAL: %s -> escalating\r\n", msg ? msg : "(no reason)");
//    app_fault_delay_ms(2);
//    escalate_to_iwdg_reset(msg);
//
//}

/**
 * @brief Reset MCU after a fatal initialization error.
 *
 * @param msg Error message.
 */
void on_fatal_init_error(const char *msg)
{
    log_printf(
        "FATAL: %s -> NVIC system reset\r\n",
        msg ? msg : "(no reason)");
    /*
     * Give the UART logger a very short window to finish transmitting
     * the fatal message before resetting the MCU.
     */
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

/**
 * @brief Reset MCU after a fatal error before IWDG startup.
 *
 * @param reason Reset reason.
 */
 void app_pre_iwdg_fatal_reset(const char *reason)
{
    /*
     *
     * app_crypto_and_esp32_link_init() runs before iwdg_init() and before
     * vTaskStartScheduler(), so the watchdog/supervisor recovery path does
     * not exist yet.
     */
    log_printf(
        "[FATAL][PRE-IWDG] %s -> NVIC system reset\r\n",
        (reason != NULL) ? reason : "(no reason)");
    app_fault_delay_ms(20U);
    __DSB();
    __ISB();
    NVIC_SystemReset();
    /* Defensive fallback. */
    for (;;)
    {
        __NOP();
    }
}

///* Fatal init reset helper. */
//void on_fatal_init_error(const char *msg)
//{
//    log_printf("FATAL: %s -> escalating\r\n", msg ? msg : "(no reason)");
//    app_fault_delay_ms(2);
//    escalate_to_iwdg_reset(msg);
//}

/* Wait until IWDG update bits clear. */
static iwdg_status_t wait_iwdg_ready(uint32_t budget)
{
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

/**
 * @brief Initialize the independent watchdog.
 *
 * @param prescaler IWDG prescaler value.
 * @param reload IWDG reload value.
 * @param poll_budget Ready-wait timeout.
 * @return IWDG status.
 */
iwdg_status_t iwdg_init(uint32_t prescaler, uint32_t reload, uint32_t poll_budget)
{
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
    /* Enable IWDG. */
    IWDG->KR = IWDG_KEY_ENABLE;
    __asm volatile ("nop");
    /* Enable PR/RLR write access. */
    IWDG->KR = IWDG_KEY_WR_ACCESS_ENABLE;
    IWDG->PR  = prescaler;
    IWDG->RLR = reload;
    if (wait_iwdg_ready(0U) != IWDG_OK) {
        IWDG_ErrorHandler("IWDG: timeout waiting for PR/RLR update", IWDG_ERR_TIMEOUT);
        return IWDG_ERR_TIMEOUT;
    }
    /* Load new reload value. */
    IWDG->KR = IWDG_KEY_RELOAD;
#if defined(IWDG_RLR)
    if ((IWDG->RLR & IWDG_RELOAD_MAX) != (reload & IWDG_RELOAD_MAX)) {
        IWDG_ErrorHandler("IWDG: RLR readback mismatch", IWDG_ERR_HW);
    }
#endif
    iwdg_inited = true;
    return IWDG_OK;
}

/**
 * @brief Refresh the independent watchdog.
 *
 * @return IWDG status.
 */
iwdg_status_t iwdg_refresh(void)
{
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

/**
 * @brief Check if IWDG register updates are complete.
 *
 * @return true when ready.
 */
bool iwdg_is_ready(void)
{
    return (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) == 0U) ? true : false;
}
