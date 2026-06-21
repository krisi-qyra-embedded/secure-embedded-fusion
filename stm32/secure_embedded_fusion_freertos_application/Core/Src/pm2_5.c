/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#include "pm2_5.h"
#include "uart.h"
#include "pm25_uart.h"
#include "stm32f4xx_hal.h"
#include "pwr.h"
#include "tim.h"

#include <string.h>

#define PMS25_FRAME_SIZE               32U
#define PMS25_HEADER_BYTE_0            0x42U
#define PMS25_HEADER_BYTE_1            0x4DU
#define PMS25_EXPECTED_FRAME_LEN       28U

#define PMS25_PREFAULT_BACKOFF_MS      20U
#define PMS25_FATAL_LOG_FLUSH_MS       100U

static uint16_t PMS25_ReadBE16(const uint8_t *p);
static PMS25_Status_t PMS25_MapUartStatus(UART_Status_t uart_st);
static const char *PMS25_UartStatusStr(UART_Status_t st);
static void PMS25_AppSleepMs(uint32_t ms);
static void PMS25_LogSample(const PMS25_Data_t *d);
static void PMS25_ClearData(PMS25_App_t *app);
static void PMS25_LogRuntimeReadFailure(PMS25_Status_t pm_st,
                                        uint32_t no_frame_elapsed_ms,
                                        uint32_t no_frame_fault_ms,
                                        uint8_t recovery_attempt,
                                        uint8_t max_recovery_attempts);

static PMS25_Status_t PMS25_ReadInternal(PMS25_Data_t *out,
                                         uint32_t timeout_ms,
                                         uint8_t *frame_available_out);

/* Decodes a big-endian PM2.5 frame field. */
static uint16_t PMS25_ReadBE16(const uint8_t *p)
{
    /* Place the first byte in the high half of the 16-bit value. */
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* Maps UART receive failures to PM2.5 statuses. */
static PMS25_Status_t PMS25_MapUartStatus(UART_Status_t uart_st)
{
    /* Preserve the receive-timeout distinction from other UART errors. */
    switch (uart_st)
    {
        case UART_STATUS_OK:
            return PMS25_STATUS_OK;

        case UART_STATUS_TIMEOUT:
            return PMS25_STATUS_TIMEOUT;

        default:
            return PMS25_STATUS_UART_ERROR;
    }
}

/* Returns a diagnostic label for a PM2.5 UART status. */
static const char *PMS25_UartStatusStr(UART_Status_t st)
{
    /* Use the fallback label for an unsupported UART status. */
    switch (st)
    {
        case UART_STATUS_OK:            return "OK";
        case UART_STATUS_INVALID_PARAM: return "INVALID_PARAM";
        case UART_STATUS_INIT_ERROR:    return "INIT_ERROR";
        case UART_STATUS_BUSY:          return "BUSY";
        case UART_STATUS_DMA_ERROR:     return "DMA_ERROR";
        case UART_STATUS_TIMEOUT:       return "TIMEOUT";
        case UART_STATUS_ERROR:         return "ERROR";
        default:                        return "UNKNOWN";
    }
}

/* Returns a diagnostic label for a PM2.5 parser status. */
const char *PMS25_StatusStr(PMS25_Status_t st)
{
    /* Distinguish frame, checksum and transport failures. */
    switch (st)
    {
        case PMS25_STATUS_OK:             return "OK";
        case PMS25_STATUS_INVALID_PARAM:  return "INVALID_PARAM";
        case PMS25_STATUS_TIMEOUT:        return "TIMEOUT";
        case PMS25_STATUS_UART_ERROR:     return "UART_ERROR";
        case PMS25_STATUS_SYNC_ERROR:     return "SYNC_ERROR";
        case PMS25_STATUS_LENGTH_ERROR:   return "LENGTH_ERROR";
        case PMS25_STATUS_CHECKSUM_ERROR: return "CHECKSUM_ERROR";
        default:                          return "UNKNOWN";
    }
}

/* Waits between PM2.5 application operations. */
static void PMS25_AppSleepMs(uint32_t ms)
{
    /* Use the application delay path for sampling and recovery intervals. */
    const uint32_t deadline = HAL_GetTick() + ms;

    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
        cpu_sleep_until_irq();
    }
}

/* Reports the PM2.5 mass and particle-count sample. */
static void PMS25_LogSample(const PMS25_Data_t *d)
{
    /* Reject a null sample before reading its decoded fields. */
    if (d == NULL) {
        log_printf("[PM25][ERROR] PMS25_LogSample: NULL data pointer\r\n");
        return;
    }

    log_printf("[PM25] STD: PM1.0=%u PM2.5=%u PM10=%u | ATM: PM1.0=%u PM2.5=%u PM10=%u\r\n",
               (unsigned)d->pm1_0_standard,
               (unsigned)d->pm2_5_standard,
               (unsigned)d->pm10_standard,
               (unsigned)d->pm1_0_atmospheric,
               (unsigned)d->pm2_5_atmospheric,
               (unsigned)d->pm10_atmospheric);

    log_printf("[PM25] PARTICLES: >0.3um=%u >0.5um=%u >1.0um=%u >2.5um=%u >5.0um=%u >10um=%u\r\n",
               (unsigned)d->particles_0_3um,
               (unsigned)d->particles_0_5um,
               (unsigned)d->particles_1_0um,
               (unsigned)d->particles_2_5um,
               (unsigned)d->particles_5_0um,
               (unsigned)d->particles_10um);
}

/* Clears the cached PM2.5 measurement. */
static void PMS25_ClearData(PMS25_App_t *app)
{
    /* Discard prior data when entering receive recovery. */
    if (app != NULL)
    {
        memset(&app->Data, 0, sizeof(app->Data));
    }
}

/* Reports PM2.5 read failure and recovery timing. */
static void PMS25_LogRuntimeReadFailure(PMS25_Status_t pm_st,
                                        uint32_t no_frame_elapsed_ms,
                                        uint32_t no_frame_fault_ms,
                                        uint8_t recovery_attempt,
                                        uint8_t max_recovery_attempts)
{
    /* Include the no-frame age and current recovery attempt. */
    log_printf("[WARN][PM25] read failed, st=%s, no_frame_elapsed=%lu ms, threshold=%lu ms, recovery attempt=%u/%u\r\n",
               PMS25_StatusStr(pm_st),
               (unsigned long)no_frame_elapsed_ms,
               (unsigned long)no_frame_fault_ms,
               (unsigned)recovery_attempt,
               (unsigned)max_recovery_attempts);
}

/* Receives, validates and decodes one PM2.5 frame. */
static PMS25_Status_t PMS25_ReadInternal(PMS25_Data_t *out,
                                         uint32_t timeout_ms,
                                         uint8_t *frame_available_out)
{
    /* Search for the sync bytes before checking frame length and checksum. */
    uint8_t frame[PMS25_FRAME_SIZE];
    uint8_t first_byte;
    uint16_t checksum_calc;
    uint16_t checksum_rx;
    uint32_t start_tick;
    uint32_t elapsed;
    uint32_t remaining;
    UART_Status_t uart_st;
    PMS25_Status_t last_nonfatal = PMS25_STATUS_TIMEOUT;
    uint8_t i;

    if ((out == NULL) || (timeout_ms == 0U))
    {
        return PMS25_STATUS_INVALID_PARAM;
    }

    if (frame_available_out != NULL)
    {
        *frame_available_out = 0U;
    }

    start_tick = HAL_GetTick();

    while (1)
    {
        elapsed = HAL_GetTick() - start_tick;
        if (elapsed >= timeout_ms)
        {
            return last_nonfatal;
        }

        remaining = timeout_ms - elapsed;

        uart_st = UART1_RX_DMA_Receive(&first_byte, 1U, remaining);
        if (uart_st != UART_STATUS_OK)
        {
            return PMS25_MapUartStatus(uart_st);
        }

        if (first_byte != PMS25_HEADER_BYTE_0)
        {
            continue;
        }

        frame[0] = first_byte;

        elapsed = HAL_GetTick() - start_tick;
        if (elapsed >= timeout_ms)
        {
            return last_nonfatal;
        }

        remaining = timeout_ms - elapsed;

        uart_st = UART1_RX_DMA_Receive(&frame[1], 31U, remaining);
        if (uart_st != UART_STATUS_OK)
        {
            return PMS25_MapUartStatus(uart_st);
        }

        if (frame[1] != PMS25_HEADER_BYTE_1)
        {
            last_nonfatal = PMS25_STATUS_SYNC_ERROR;
            continue;
        }

        if (frame_available_out != NULL)
        {
            *frame_available_out = 1U;
        }

        checksum_calc = 0U;
        for (i = 0U; i < 30U; i++)
        {
            checksum_calc = (uint16_t)(checksum_calc + frame[i]);
        }

        checksum_rx = PMS25_ReadBE16(&frame[30]);

        if (checksum_calc != checksum_rx)
        {
            last_nonfatal = PMS25_STATUS_CHECKSUM_ERROR;
            continue;
        }

        out->frame_len = PMS25_ReadBE16(&frame[2]);
        if (out->frame_len != PMS25_EXPECTED_FRAME_LEN)
        {
            last_nonfatal = PMS25_STATUS_LENGTH_ERROR;
            continue;
        }

        out->pm1_0_standard    = PMS25_ReadBE16(&frame[4]);
        out->pm2_5_standard    = PMS25_ReadBE16(&frame[6]);
        out->pm10_standard     = PMS25_ReadBE16(&frame[8]);

        out->pm1_0_atmospheric = PMS25_ReadBE16(&frame[10]);
        out->pm2_5_atmospheric = PMS25_ReadBE16(&frame[12]);
        out->pm10_atmospheric  = PMS25_ReadBE16(&frame[14]);

        out->particles_0_3um   = PMS25_ReadBE16(&frame[16]);
        out->particles_0_5um   = PMS25_ReadBE16(&frame[18]);
        out->particles_1_0um   = PMS25_ReadBE16(&frame[20]);
        out->particles_2_5um   = PMS25_ReadBE16(&frame[22]);
        out->particles_5_0um   = PMS25_ReadBE16(&frame[24]);
        out->particles_10um    = PMS25_ReadBE16(&frame[26]);

        out->reserved          = PMS25_ReadBE16(&frame[28]);
        out->checksum          = checksum_rx;

        return PMS25_STATUS_OK;
    }
}

/* Reads one PM2.5 sample within the supplied timeout. */
PMS25_Status_t PMS25_Read(PMS25_Data_t *out, uint32_t timeout_ms)
{
    /* Use the parser without requesting its frame-availability indicator. */
    return PMS25_ReadInternal(out, timeout_ms, NULL);
}

/* Initializes PM2.5 sampling and recovery limits. */
PMS25_AppRuntimeStatus_t PMS25_AppInit(PMS25_App_t *app,
                                       uint32_t frame_poll_timeout_ms,
                                       uint32_t no_frame_fault_ms,
                                       uint32_t sample_period_ms,
                                       uint32_t recovery_delay_ms,
                                       uint8_t max_recovery_attempts)
{
    /* Reject zero timing budgets and a zero retry limit. */
    if ((app == NULL) ||
        (frame_poll_timeout_ms == 0U) ||
        (no_frame_fault_ms == 0U) ||
        (sample_period_ms == 0U) ||
        (max_recovery_attempts == 0U))
    {
        return PMS25_APP_RUNTIME_BAD_PARAM;
    }

    memset(app, 0, sizeof(*app));

    app->FramePollTimeoutMs     = frame_poll_timeout_ms;
    app->NoFrameFaultMs         = no_frame_fault_ms;
    app->SamplePeriodMs         = sample_period_ms;
    app->RecoveryDelayMs        = recovery_delay_ms;
    app->MaxRecoveryAttempts    = max_recovery_attempts;
    app->RecoveryAttempt        = 0U;
    app->LastFrameAvailableTick = HAL_GetTick();

    return PMS25_APP_RUNTIME_OK;
}

/* Runs PM2.5 sampling and bounded UART recovery. */
PMS25_AppRuntimeStatus_t PMS25_AppRun(PMS25_App_t *app)
{
    /* Delay escalation until the configured no-frame interval has elapsed. */
    PMS25_Status_t pm_st;
    UART_Status_t uart_st;
    uint8_t frame_available = 0U;
    uint32_t now;
    uint32_t no_frame_elapsed;

    if (app == NULL)
    {
        return PMS25_APP_RUNTIME_BAD_PARAM;
    }

    pm_st = PMS25_ReadInternal(&app->Data, app->FramePollTimeoutMs, &frame_available);
    now = HAL_GetTick();

    if (frame_available != 0U)
    {
        app->LastFrameAvailableTick = now;
    }

    if (pm_st == PMS25_STATUS_OK)
    {
        app->RecoveryAttempt = 0U;
        PMS25_LogSample(&app->Data);
        PMS25_AppSleepMs(app->SamplePeriodMs);
        return PMS25_APP_RUNTIME_OK;
    }

    no_frame_elapsed = now - app->LastFrameAvailableTick;

    if (no_frame_elapsed < app->NoFrameFaultMs)
    {
        PMS25_AppSleepMs(PMS25_PREFAULT_BACKOFF_MS);
        return PMS25_APP_RUNTIME_OK;
    }

    app->RecoveryAttempt++;

    PMS25_LogRuntimeReadFailure(pm_st,
                                no_frame_elapsed,
                                app->NoFrameFaultMs,
                                app->RecoveryAttempt,
                                app->MaxRecoveryAttempts);

    PMS25_ClearData(app);

    uart_st = UART1_PM25_Recover(app);
    if (uart_st != UART_STATUS_OK)
    {
        log_printf("[ERR][PM25] UART1_PM25_Recover failed, st=%s\r\n",
                   PMS25_UartStatusStr(uart_st));
    }

    app->LastFrameAvailableTick = HAL_GetTick();

    if (app->RecoveryAttempt >= app->MaxRecoveryAttempts)
    {
        log_printf("[FATAL][PM25] recovery attempts exhausted (%u/%u)\r\n",
                   (unsigned)app->RecoveryAttempt,
                   (unsigned)app->MaxRecoveryAttempts);
        delay_ms(PMS25_FATAL_LOG_FLUSH_MS);
        return PMS25_APP_RUNTIME_FATAL;
    }

    if (app->RecoveryDelayMs != 0U)
    {
        delay_ms(app->RecoveryDelayMs);
    }

    return PMS25_APP_RUNTIME_OK;
}
