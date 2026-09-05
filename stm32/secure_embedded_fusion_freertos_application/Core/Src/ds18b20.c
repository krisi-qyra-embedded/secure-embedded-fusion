/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "ds18b20.h"
#include "uart.h"
#include "pwr.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include "tim.h"
#include "tim3.h"
#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "i2c_dma.h"
#include "telemetry.h"


static void DS18B20_LogGroupSeparator(void);
static void DS18B20_LogTemperatureSample(uint8_t sensor_index, float temperature_c);
static void DS18B20_AppResetRuntimeState(DS18B20_App_t *App);
static DS18B20_Status_t DS18B20_MapOwStatus(OneWire_Status_t st);
static const char *DS18B20_StatusStr(DS18B20_Status_t st);
static void DS18B20_AppGpioInit(GPIO_TypeDef *Port, uint16_t Pin);

extern void system_fault_mark_in_progress(uint8_t owner);
extern void system_fault_mark_completed(uint8_t owner);
extern void system_fault_clear_active(uint8_t owner);
extern uint8_t system_fault_capture_or_critical_fault_active(void);

static void DS18B20_AppMarkFaultInProgress(void);
static void DS18B20_AppMarkFaultCompleted(void);
static void DS18B20_AppClearFaultActive(void);

extern QueueHandle_t q_ds18b20;
extern uint32_t ds18b20_generation;

/* Checks whether normal DS18B20 output is allowed. */
static uint8_t DS18B20_NormalLoggingAllowed(void)
{
    /* Mute routine temperature logs during critical fault capture. */
    return (system_fault_capture_or_critical_fault_active() == 0U) ? 1U : 0U;
}

/* Waits between DS18B20 application operations. */
static void DS18B20_AppSleepMs(uint32_t ms)
{
    /* Use the application delay path while conversion or recovery settles. */
    if (ms == 0U) {
        return;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        TickType_t ticks = pdMS_TO_TICKS(ms);

        if (ticks == 0U) {
            ticks = 1U;
        }

        vTaskDelay(ticks);
        return;
    }

    delay_ms(ms);
}

/* Starts DS18B20 fault capture under FreeRTOS. */
static void DS18B20_AppMarkFaultInProgress(void)
{
    /* Publish the sensor's in-progress bit only after scheduler startup. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_DS18B20);
    }
}

/* Marks the DS18B20 fault report complete. */
static void DS18B20_AppMarkFaultCompleted(void)
{
    /* Use the shared completion mask when the scheduler is running. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        system_fault_mark_completed(SYSTEM_FAULT_OWNER_DS18B20);
    }
}

/* Clears recovered DS18B20 fault state. */
static void DS18B20_AppClearFaultActive(void)
{
    /* Release the sensor's shared fault bits from task context. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        system_fault_clear_active(SYSTEM_FAULT_OWNER_DS18B20);
    }
}

/* Formats a DS18B20 fault message and allows it to drain. */
static void DS18B20_AppLogErrorDelay(const char *fmt, ...)
{
    /* Clamp the formatted text to the diagnostic buffer capacity. */
    char tmp[160];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    if (n > 0)
    {
        if (n > (int)(sizeof(tmp) - 1)) {
            n = (int)(sizeof(tmp) - 1);
        }

        DS18B20_LOG_PRINTF("%s", tmp);
    }

    /* user requested 10 ms delay after error print; yield if FreeRTOS is running */
    DS18B20_AppSleepMs(10U);
}

/* Clears retained DS18B20 discovery and sample data. */
static void DS18B20_AppClearDiscoveryState(DS18B20_App_t *App)
{
    /* Reset the ROM count with device, alarm and temperature arrays. */
    if (App == NULL) {
        return;
    }

    App->OW.RomCnt = 0U;
    memset(App->DS.DevAddr, 0, sizeof(App->DS.DevAddr));
    memset(App->DS.AlmAddr, 0, sizeof(App->DS.AlmAddr));
    memset(App->DS.Temperature, 0, sizeof(App->DS.Temperature));
}

/* Checks that all discovered DS18B20 devices can convert and read. */
static DS18B20_Status_t DS18B20_AppVerifyAllSensorsReady(DS18B20_App_t *App)
{
    /* Verify the expected sensor count before starting the group conversion. */
    uint8_t i;
    DS18B20_Status_t st;
    float temp;

    if (App == NULL) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    if (App->OW.RomCnt == 0U) {
        return DS18B20_STATUS_NO_DEVICE;
    }

    if ((App->ExpectedSensorCount != 0U) &&
        (App->OW.RomCnt != App->ExpectedSensorCount))
    {
        return DS18B20_STATUS_NOT_FOUND;
    }

    st = DS18B20_StartAll(&App->OW);
    if (st != DS18B20_STATUS_OK) {
        return st;
    }

    DS18B20_AppSleepMs(App->DS.ConversionTimeoutMs);

    for (i = 0U; i < App->OW.RomCnt; i++)
    {
        st = DS18B20_Read(&App->OW, App->DS.DevAddr[i], &temp);
        if (st != DS18B20_STATUS_OK) {
            return st;
        }

        App->DS.Temperature[i] = temp;
    }

    return DS18B20_STATUS_OK;
}

/* Applies one stage of DS18B20 bus recovery. */
static DS18B20_Status_t DS18B20_AppApplyRecoveryPhase(DS18B20_App_t *App,
                                                      uint8_t phase)
{
    /* Escalate from a bus reset to rescan and GPIO rebootstrap. */
    DS18B20_Status_t ds_st;
    OneWire_Status_t ow_st;

    if (App == NULL) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    switch (phase)
    {
        case 1U:
        {
            DS18B20_AppLogErrorDelay(
                "ACT: DS18B20 recovery 1/3 -> soft OneWire re-arm + bus reset\r\n");

            ow_st = OneWire_Init(&App->OW);
            if (ow_st != ONEWIRE_STATUS_OK) {
                return DS18B20_MapOwStatus(ow_st);
            }

            ow_st = OneWire_Reset(&App->OW);
            if (ow_st != ONEWIRE_STATUS_OK) {
                return DS18B20_MapOwStatus(ow_st);
            }

            return DS18B20_AppVerifyAllSensorsReady(App);
        }

        case 2U:
        {
            DS18B20_AppLogErrorDelay(
                "ACT: DS18B20 recovery 2/3 -> full rescan + DS18B20 re-init\r\n");

            DS18B20_AppClearDiscoveryState(App);

            ds_st = DS18B20_Init(&App->DS, &App->OW);
            if (ds_st != DS18B20_STATUS_OK) {
                return ds_st;
            }

            DS18B20_LOG_PRINTF("[DBG][DS18B20] Init result=%s RomCnt=%u Expected=%u\r\n",
                               DS18B20_StatusStr(ds_st),
                               (unsigned int)App->OW.RomCnt,
                               (unsigned int)App->ExpectedSensorCount);

            return DS18B20_AppVerifyAllSensorsReady(App);
        }

        case 3U:
        {
            DS18B20_AppLogErrorDelay(
                "ACT: DS18B20 recovery 3/3 -> cold GPIO re-bootstrap + full re-init\r\n");

            DS18B20_AppGpioInit(App->OW.DataPort, App->OW.DataPin);
            DS18B20_AppSleepMs(10U);

            DS18B20_AppClearDiscoveryState(App);

            ds_st = DS18B20_Init(&App->DS, &App->OW);
            if (ds_st != DS18B20_STATUS_OK) {
                return ds_st;
            }

            return DS18B20_AppVerifyAllSensorsReady(App);
        }

        default:
            return DS18B20_STATUS_BAD_PARAM;
    }
}

/* Runs the three DS18B20 recovery stages in order. */
static DS18B20_AppRuntimeStatus_t DS18B20_AppRunRecoverySequence(DS18B20_App_t *App,
                                                                 const char *reason)
{
    /* Reset runtime state after the first successful recovery phase. */
    DS18B20_AppMarkFaultInProgress();
    uint8_t phase;
    DS18B20_Status_t st;

    if (App == NULL) {
        return DS18B20_APP_RUNTIME_BAD_PARAM;
    }

    for (phase = 1U; phase <= 3U; phase++)
    {
        st = DS18B20_AppApplyRecoveryPhase(App, phase);
        if (st == DS18B20_STATUS_OK)
        {
            DS18B20_AppLogErrorDelay(
                "OK: DS18B20 recovery succeeded on attempt %u/3 (%s)\r\n",
                (unsigned int)phase,
                (reason != NULL) ? reason : "runtime");

            App->RecoveryAttempt = 0U;
            DS18B20_AppClearFaultActive();
            DS18B20_AppResetRuntimeState(App);
            return DS18B20_APP_RUNTIME_RECOVERED;
        }

        DS18B20_AppLogErrorDelay(
            "[ERR] DS18B20 recovery attempt %u/3 failed (%s): %s\r\n",
            (unsigned int)phase,
            (reason != NULL) ? reason : "runtime",
            DS18B20_StatusStr(st));
    }

    DS18B20_AppLogErrorDelay(
        "[FATAL] DS18B20 recovery exhausted 3/3 attempts (%s)\r\n",
        (reason != NULL) ? reason : "runtime");

    DS18B20_AppMarkFaultCompleted();

    return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
}

/* Discovers the DS18B20 group and retries failed bring-up. */
static DS18B20_Status_t DS18B20_AppBringupWithRecovery(DS18B20_App_t *App)
{
    /* Verify sensor readback before accepting initialization. */
    DS18B20_Status_t st;
    DS18B20_AppRuntimeStatus_t rec_st;

    if (App == NULL) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    DS18B20_AppClearDiscoveryState(App);

    st = DS18B20_Init(&App->DS, &App->OW);
    if (st == DS18B20_STATUS_OK) {
        st = DS18B20_AppVerifyAllSensorsReady(App);
    }

    if (st == DS18B20_STATUS_OK) {
        return DS18B20_STATUS_OK;
    }

    DS18B20_AppLogErrorDelay("[ERR] Initial DS18B20 bring-up failed: %s\r\n",
                             DS18B20_StatusStr(st));

    rec_st = DS18B20_AppRunRecoverySequence(App, "initial bring-up");
    if (rec_st == DS18B20_APP_RUNTIME_RECOVERED) {
        return DS18B20_STATUS_OK;
    }

    return st;
}

/* Checks the ROM family code for a DS18B20 device. */
static uint8_t DS18B20_IsValid(const uint8_t *ROM)
{
    /* Require a non-null ROM with the configured family byte. */
    return ((ROM != NULL) && (ROM[0] == DS18B20_FAMILY_CODE)) ? 1U : 0U;
}

/* Maps OneWire errors to DS18B20 status values. */
static DS18B20_Status_t DS18B20_MapOwStatus(OneWire_Status_t st)
{
    /* Treat unrecognized OneWire failures as bus errors. */
    switch (st)
    {
        case ONEWIRE_STATUS_OK:        return DS18B20_STATUS_OK;
        case ONEWIRE_STATUS_BAD_PARAM: return DS18B20_STATUS_BAD_PARAM;
        case ONEWIRE_STATUS_NO_DEVICE: return DS18B20_STATUS_NO_DEVICE;
        case ONEWIRE_STATUS_NOT_FOUND: return DS18B20_STATUS_NOT_FOUND;
        case ONEWIRE_STATUS_BUS_ERROR:
        default:                       return DS18B20_STATUS_BUS_ERROR;
    }
}

/* Returns a diagnostic label for a DS18B20 status. */
static const char *DS18B20_StatusStr(DS18B20_Status_t st)
{
    /* Use the unknown label for an unmapped status. */
    switch (st)
    {
        case DS18B20_STATUS_OK:          return "OK";
        case DS18B20_STATUS_BAD_PARAM:   return "BAD_PARAM";
        case DS18B20_STATUS_INVALID_ROM: return "INVALID_ROM";
        case DS18B20_STATUS_NO_DEVICE:   return "NO_DEVICE";
        case DS18B20_STATUS_NOT_FOUND:   return "NOT_FOUND";
        case DS18B20_STATUS_BUS_ERROR:   return "BUS_ERROR";
        case DS18B20_STATUS_CRC_ERROR:   return "CRC_ERROR";
        case DS18B20_STATUS_TIMEOUT:     return "TIMEOUT";
        default:                         return "UNKNOWN";
    }
}

/* Checks the discovered DS18B20 count. */
static uint8_t DS18B20_AppSensorCountHealthy(const DS18B20_App_t *App)
{
    /* Allow any nonempty group when no exact count is configured. */
    if (App == NULL) {
        return 0U;
    }

    if (App->ExpectedSensorCount == 0U) {
        return (App->OW.RomCnt > 0U) ? 1U : 0U;
    }

    return (App->OW.RomCnt == App->ExpectedSensorCount) ? 1U : 0U;
}

/* Rearms the DS18B20 conversion state machine. */
static void DS18B20_AppResetRuntimeState(DS18B20_App_t *App)
{
    /* Clear the old deadline and one-shot timeout-report flag. */
    App->State           = DS18B20_APP_START_CONVERSION;
    App->DeadlineMs      = 0U;
    App->TimeoutReported = 0U;
}

/* Applies the next DS18B20 runtime recovery stage. */
static DS18B20_AppRuntimeStatus_t DS18B20_AppRecoverAndReinit(DS18B20_App_t *App)
{
    /* Reset the recovery index only when the stage succeeds. */
    DS18B20_Status_t ds_st;
    OneWire_Status_t ow_st;

    DS18B20_AppMarkFaultInProgress();

    if (App == NULL) {
        return DS18B20_APP_RUNTIME_BAD_PARAM;
    }

    App->RecoveryAttempt++;

    switch (App->RecoveryAttempt)
    {
        case 1U:
        {
            DS18B20_AppLogErrorDelay("ACT: DS18B20 recovery 1/3 -> soft OneWire re-arm + bus reset\r\n");

            ow_st = OneWire_Init(&App->OW);
            if (ow_st != ONEWIRE_STATUS_OK) {
                DS18B20_AppLogErrorDelay("[ERR] recovery 1/3 OneWire_Init failed\r\n");
                return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
            }

            ow_st = OneWire_Reset(&App->OW);
            if (ow_st != ONEWIRE_STATUS_OK) {
                DS18B20_AppLogErrorDelay("[ERR] recovery 1/3 OneWire_Reset failed\r\n");
                return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
            }

            App->RecoveryAttempt = 0U;
            DS18B20_AppClearFaultActive();
            DS18B20_AppResetRuntimeState(App);
            return DS18B20_APP_RUNTIME_RECOVERED;
        }

        case 2U:
        {
            DS18B20_AppLogErrorDelay("ACT: DS18B20 recovery 2/3 -> rescan + driver re-init\r\n");

            App->OW.RomCnt = 0U;
            memset(App->DS.DevAddr, 0, sizeof(App->DS.DevAddr));
            memset(App->DS.AlmAddr, 0, sizeof(App->DS.AlmAddr));
            memset(App->DS.Temperature, 0, sizeof(App->DS.Temperature));

            ds_st = DS18B20_Init(&App->DS, &App->OW);
            if (ds_st != DS18B20_STATUS_OK) {
                DS18B20_AppLogErrorDelay("[ERR] recovery 2/3 DS18B20_Init failed: %s\r\n",
                                         DS18B20_StatusStr(ds_st));
                return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
            }

            if (DS18B20_AppSensorCountHealthy(App) == 0U) {
                DS18B20_AppLogErrorDelay("[ERR] recovery 2/3 sensor-count check failed\r\n");
                return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
            }

            App->RecoveryAttempt = 0U;
            DS18B20_AppClearFaultActive();
            DS18B20_AppResetRuntimeState(App);
            return DS18B20_APP_RUNTIME_RECOVERED;
        }

        case 3U:
        {
            DS18B20_AppLogErrorDelay("ACT: DS18B20 recovery 3/3 -> cold GPIO re-bootstrap + full re-init\r\n");

            DS18B20_AppGpioInit(App->OW.DataPort, App->OW.DataPin);

            App->OW.RomCnt = 0U;
            memset(App->DS.DevAddr, 0, sizeof(App->DS.DevAddr));
            memset(App->DS.AlmAddr, 0, sizeof(App->DS.AlmAddr));
            memset(App->DS.Temperature, 0, sizeof(App->DS.Temperature));

            ds_st = DS18B20_Init(&App->DS, &App->OW);
            if (ds_st != DS18B20_STATUS_OK) {
                DS18B20_AppLogErrorDelay("[ERR] recovery 3/3 DS18B20_Init failed: %s\r\n",
                                         DS18B20_StatusStr(ds_st));
                return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
            }

            if (DS18B20_AppSensorCountHealthy(App) == 0U) {
                DS18B20_AppLogErrorDelay("[ERR] recovery 3/3 sensor-count check failed\r\n");
                return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
            }

            App->RecoveryAttempt = 0U;
            DS18B20_AppClearFaultActive();
            DS18B20_AppResetRuntimeState(App);
            return DS18B20_APP_RUNTIME_RECOVERED;
        }

        default:
            DS18B20_AppMarkFaultCompleted();
            return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
    }
}

/* Attempts recovery of a failed DS18B20 within the group. */
static DS18B20_AppRuntimeStatus_t DS18B20_AppRecoverFailedSensor(DS18B20_App_t *App,
                                                                 uint8_t sensor_index)
{
    /* Retain the failing ROM while escalating toward a full rescan. */
    DS18B20_AppMarkFaultInProgress();
    uint8_t attempt;
    uint8_t sensor_rom[8];
    float recovered_temp = 0.0f;
    DS18B20_Status_t ds_st;
    OneWire_Status_t ow_st;

    if ((App == NULL) || (sensor_index >= App->OW.RomCnt)) {
        return DS18B20_APP_RUNTIME_BAD_PARAM;
    }

    memcpy(sensor_rom, App->DS.DevAddr[sensor_index], sizeof(sensor_rom));

    for (attempt = 1U; attempt <= 3U; attempt++)
    {
        switch (attempt)
        {
            case 1U:
            {
                DS18B20_AppLogErrorDelay(
                    "ACT: DS18B20 sensor %u recovery 1/3 -> OneWire re-arm + bus reset\r\n",
                    (unsigned int)(sensor_index + 1U));

                ow_st = OneWire_Init(&App->OW);
                if (ow_st != ONEWIRE_STATUS_OK) {
                    DS18B20_AppLogErrorDelay(
                        "[ERR] DS18B20 sensor %u recovery 1/3 OneWire_Init failed\r\n",
                        (unsigned int)(sensor_index + 1U));
                    continue;
                }

                ow_st = OneWire_Reset(&App->OW);
                if (ow_st != ONEWIRE_STATUS_OK) {
                    DS18B20_AppLogErrorDelay(
                        "[ERR] DS18B20 sensor %u recovery 1/3 OneWire_Reset failed\r\n",
                        (unsigned int)(sensor_index + 1U));
                    continue;
                }

                break;
            }

            case 2U:
            {
                DS18B20_AppLogErrorDelay(
                    "ACT: DS18B20 sensor %u recovery 2/3 -> reconfigure only failed sensor\r\n",
                    (unsigned int)(sensor_index + 1U));

                ow_st = OneWire_Reset(&App->OW);
                if (ow_st != ONEWIRE_STATUS_OK) {
                    DS18B20_AppLogErrorDelay(
                        "[ERR] DS18B20 sensor %u recovery 2/3 OneWire_Reset failed\r\n",
                        (unsigned int)(sensor_index + 1U));
                    continue;
                }

                ds_st = DS18B20_SetResolution(&App->OW, sensor_rom, App->DS.Resolution);
                if (ds_st != DS18B20_STATUS_OK) {
                    DS18B20_AppLogErrorDelay(
                        "[ERR] DS18B20 sensor %u recovery 2/3 SetResolution failed: %s\r\n",
                        (unsigned int)(sensor_index + 1U),
                        DS18B20_StatusStr(ds_st));
                    continue;
                }

                ds_st = DS18B20_SetTempAlarm(&App->OW, sensor_rom, 0, 31);
                if (ds_st != DS18B20_STATUS_OK) {
                    DS18B20_AppLogErrorDelay(
                        "[ERR] DS18B20 sensor %u recovery 2/3 SetTempAlarm failed: %s\r\n",
                        (unsigned int)(sensor_index + 1U),
                        DS18B20_StatusStr(ds_st));
                    continue;
                }

                break;
            }

            case 3U:
            default:
            {
                DS18B20_AppLogErrorDelay(
                    "ACT: DS18B20 sensor %u recovery 3/3 -> stop group + full DS18B20 re-bootstrap\r\n",
                    (unsigned int)(sensor_index + 1U));

                DS18B20_AppGpioInit(App->OW.DataPort, App->OW.DataPin);

                App->OW.RomCnt = 0U;
                memset(App->DS.DevAddr, 0, sizeof(App->DS.DevAddr));
                memset(App->DS.AlmAddr, 0, sizeof(App->DS.AlmAddr));
                memset(App->DS.Temperature, 0, sizeof(App->DS.Temperature));

                ds_st = DS18B20_Init(&App->DS, &App->OW);
                if (ds_st != DS18B20_STATUS_OK) {
                    DS18B20_AppLogErrorDelay(
                        "[ERR] DS18B20 sensor %u recovery 3/3 DS18B20_Init failed: %s\r\n",
                        (unsigned int)(sensor_index + 1U),
                        DS18B20_StatusStr(ds_st));
                    continue;
                }

                if (DS18B20_AppSensorCountHealthy(App) == 0U) {
                    DS18B20_AppLogErrorDelay(
                        "[ERR] DS18B20 sensor %u recovery 3/3 sensor-count check failed\r\n",
                        (unsigned int)(sensor_index + 1U));
                    continue;
                }

                break;
            }
        }

        ds_st = DS18B20_Read(&App->OW, sensor_rom, &recovered_temp);
        if (ds_st == DS18B20_STATUS_OK)
        {
            App->DS.Temperature[sensor_index] = recovered_temp;

            DS18B20_AppLogErrorDelay(
                "OK: DS18B20 sensor %u recovered on attempt %u/3 -> resume normal sampling\r\n",
                (unsigned int)(sensor_index + 1U),
                (unsigned int)attempt);

            App->RecoveryAttempt = 0U;
            DS18B20_AppClearFaultActive();
            DS18B20_AppResetRuntimeState(App);
            return DS18B20_APP_RUNTIME_RECOVERED;
        }

        DS18B20_AppLogErrorDelay(
            "[ERR] DS18B20 sensor %u retry %u/3 readback failed: %s\r\n",
            (unsigned int)(sensor_index + 1U),
            (unsigned int)attempt,
            DS18B20_StatusStr(ds_st));
    }

    DS18B20_AppLogErrorDelay(
        "[FATAL] DS18B20 sensor %u failed after 3 dedicated recovery attempts\r\n",
        (unsigned int)(sensor_index + 1U));

    DS18B20_AppMarkFaultCompleted();

    return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
}

/* Returns the conversion budget for a DS18B20 resolution. */
uint32_t DS18B20_GetConversionTimeMs(DS18B20_Res_t Resolution)
{
    /* Use the 750 ms budget for 12-bit or unspecified resolution. */
    switch (Resolution)
    {
        case DS18B20_Resolution_9bits:  return 94U;
        case DS18B20_Resolution_10bits: return 188U;
        case DS18B20_Resolution_11bits: return 375U;
        case DS18B20_Resolution_12bits:
        default:                        return 750U;
    }
}

/* Reads the DS18B20 scratchpad resolution setting. */
DS18B20_Status_t DS18B20_GetResolution(OneWire_t *OW, const uint8_t *ROM, uint8_t *resolution)
{
    /* Decode R1 and R0 from the scratchpad configuration byte. */
    OneWire_Status_t st;
    uint8_t conf;

    if ((OW == NULL) || (resolution == NULL)) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    if (DS18B20_IsValid(ROM) == 0U) {
        return DS18B20_STATUS_INVALID_ROM;
    }

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_READSCRATCHPAD);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_ReadByte(OW, &conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);

    *resolution = (uint8_t)(((conf & 0x60U) >> 5U) + 9U);
    return DS18B20_STATUS_OK;
}

/* Writes and saves the DS18B20 resolution setting. */
DS18B20_Status_t DS18B20_SetResolution(OneWire_t *OW, const uint8_t *ROM, DS18B20_Res_t Resolution)
{
    /* Preserve the alarm bytes while replacing the resolution bits. */
    OneWire_Status_t st;
    uint8_t th;
    uint8_t tl;
    uint8_t conf;

    if (OW == NULL) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    if (DS18B20_IsValid(ROM) == 0U) {
        return DS18B20_STATUS_INVALID_ROM;
    }

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_READSCRATCHPAD);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_ReadByte(OW, &th);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &tl);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &th);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &tl);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);

    if (Resolution == DS18B20_Resolution_9bits) {
        conf &= (uint8_t)~(1U << DS18B20_RESOLUTION_R1);
        conf &= (uint8_t)~(1U << DS18B20_RESOLUTION_R0);
    } else if (Resolution == DS18B20_Resolution_10bits) {
        conf &= (uint8_t)~(1U << DS18B20_RESOLUTION_R1);
        conf |= (uint8_t)(1U << DS18B20_RESOLUTION_R0);
    } else if (Resolution == DS18B20_Resolution_11bits) {
        conf |= (uint8_t)(1U << DS18B20_RESOLUTION_R1);
        conf &= (uint8_t)~(1U << DS18B20_RESOLUTION_R0);
    } else {
        conf |= (uint8_t)(1U << DS18B20_RESOLUTION_R1);
        conf |= (uint8_t)(1U << DS18B20_RESOLUTION_R0);
    }

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_WRITESCRATCHPAD);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_WriteByte(OW, th);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_WriteByte(OW, tl);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_WriteByte(OW, conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_COPYSCRATCHPAD);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    return DS18B20_STATUS_OK;
}

/* Starts conversion on one addressed DS18B20. */
DS18B20_Status_t DS18B20_Start(OneWire_t *OW, const uint8_t *ROM)
{
    /* Select its ROM before sending the conversion command. */
    OneWire_Status_t st;

    if (OW == NULL) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    if (DS18B20_IsValid(ROM) == 0U) {
        return DS18B20_STATUS_INVALID_ROM;
    }

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_CONVERT);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    return DS18B20_STATUS_OK;
}

/* Starts conversion on every device on the OneWire bus. */
DS18B20_Status_t DS18B20_StartAll(OneWire_t *OW)
{
    /* Use SKIP ROM to broadcast the conversion command. */
    OneWire_Status_t st;

    if (OW == NULL) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, ONEWIRE_CMD_SKIPROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_CONVERT);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    return DS18B20_STATUS_OK;
}

/* Reads the DS18B20 conversion-complete bit. */
DS18B20_Status_t DS18B20_IsConversionDone(OneWire_t *OW, uint8_t *is_done)
{
    /* Propagate a OneWire read error before publishing completion. */
    OneWire_Status_t st;
    uint8_t bit;

    if ((OW == NULL) || (is_done == NULL)) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    st = OneWire_ReadBit(OW, &bit);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    *is_done = bit;
    return DS18B20_STATUS_OK;
}

/* Reads and decodes a DS18B20 temperature scratchpad. */
DS18B20_Status_t DS18B20_Read(OneWire_t *OW, const uint8_t *ROM, float *Destination)
{
    /* Validate the scratchpad CRC before using its temperature bytes. */
    OneWire_Status_t st;
    uint16_t temperature;
    uint8_t resolution;
    int16_t digit;
    uint8_t minus = 0U;
    float decimal;
    uint8_t i;
    uint8_t data[9];
    uint8_t crc;

    if ((OW == NULL) || (Destination == NULL)) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    if (DS18B20_IsValid(ROM) == 0U) {
        return DS18B20_STATUS_INVALID_ROM;
    }

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_READSCRATCHPAD);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    for (i = 0U; i < 9U; i++) {
        st = OneWire_ReadByte(OW, &data[i]);
        if (st != ONEWIRE_STATUS_OK) {
            return DS18B20_MapOwStatus(st);
        }
    }

    crc = OneWire_CRC8(data, 8U);
    if (crc != data[8]) {
        return DS18B20_STATUS_CRC_ERROR;
    }

    temperature = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);

    if ((temperature & 0x8000U) != 0U) {
        temperature = (uint16_t)(~temperature + 1U);
        minus = 1U;
    }

    resolution = (uint8_t)(((data[4] & 0x60U) >> 5U) + 9U);

    digit = (int16_t)(temperature >> 4U);
    digit |= (int16_t)(((temperature >> 8U) & 0x07U) << 4U);

    switch (resolution)
    {
        case 9:
            decimal = (float)((temperature >> 3U) & 0x01U) * DS18B20_DECIMAL_STEPS_9BIT;
            break;
        case 10:
            decimal = (float)((temperature >> 2U) & 0x03U) * DS18B20_DECIMAL_STEPS_10BIT;
            break;
        case 11:
            decimal = (float)((temperature >> 1U) & 0x07U) * DS18B20_DECIMAL_STEPS_11BIT;
            break;
        case 12:
            decimal = (float)(temperature & 0x0FU) * DS18B20_DECIMAL_STEPS_12BIT;
            break;
        default:
            return DS18B20_STATUS_BUS_ERROR;
    }

    *Destination = (float)digit + decimal;
    if (minus != 0U) {
        *Destination = -(*Destination);
    }

    return DS18B20_STATUS_OK;
}

/* Writes and saves the DS18B20 alarm thresholds. */
DS18B20_Status_t DS18B20_SetTempAlarm(OneWire_t *OW, const uint8_t *ROM, int8_t Low, int8_t High)
{
    /* Keep the resolution byte while updating the high and low alarms. */
    OneWire_Status_t st;
    uint8_t dummy;
    uint8_t conf;
    uint8_t tl;
    uint8_t th;

    if (OW == NULL) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    if (DS18B20_IsValid(ROM) == 0U) {
        return DS18B20_STATUS_INVALID_ROM;
    }

    Low  = (((Low  < -55) || (Low  == 0)) ? -55 : Low);
    High = (((High > 125) || (High == 0)) ? 125 : High);

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_READSCRATCHPAD);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_ReadByte(OW, &dummy);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &dummy);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &dummy);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &dummy);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_ReadByte(OW, &conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);

    th = (uint8_t)High;
    tl = (uint8_t)Low;

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_WRITESCRATCHPAD);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_WriteByte(OW, th);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_WriteByte(OW, tl);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);
    st = OneWire_WriteByte(OW, conf);
    if (st != ONEWIRE_STATUS_OK) return DS18B20_MapOwStatus(st);

    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_SelectWithPointer(OW, ROM);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    st = OneWire_WriteByte(OW, DS18B20_CMD_COPYSCRATCHPAD);
    if (st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(st);
    }

    return DS18B20_STATUS_OK;
}

/* Collects ROMs of DS18B20 devices reporting an alarm. */
DS18B20_Status_t DS18B20_AlarmSearch(DS18B20_Drv_t *DS, OneWire_t *OW)
{
    /* Stop searching when no further alarmed device is found. */
    uint8_t t = 0U;
    uint8_t i;
    uint8_t j;
    OneWire_Status_t st;

    if ((DS == NULL) || (OW == NULL)) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    for (i = 0U; i < DS18B20_MaxCnt; i++)
    {
        for (j = 0U; j < 8U; j++)
        {
            DS->AlmAddr[i][j] = 0U;
        }
    }

    while (t < DS18B20_MaxCnt)
    {
        st = OneWire_Search(OW, DS18B20_CMD_ALARM_SEARCH);

        if (st == ONEWIRE_STATUS_NOT_FOUND) {
            break;
        }

        if (st != ONEWIRE_STATUS_OK) {
            return DS18B20_MapOwStatus(st);
        }

        st = OneWire_GetDevRom(OW, DS->AlmAddr[t]);
        if (st != ONEWIRE_STATUS_OK) {
            return DS18B20_MapOwStatus(st);
        }

        t++;
    }

    return (t > 0U) ? DS18B20_STATUS_OK : DS18B20_STATUS_NOT_FOUND;
}

/* Discovers DS18B20 ROMs and applies per-device settings. */
DS18B20_Status_t DS18B20_Init(DS18B20_Drv_t *DS, OneWire_t *OW)
{
    /* Ignore ROMs with another family code during discovery. */
    OneWire_Status_t ow_st;
    DS18B20_Status_t ds_st;

    if ((DS == NULL) || (OW == NULL)) {
        return DS18B20_STATUS_BAD_PARAM;
    }

    if (DS->ConversionTimeoutMs == 0U) {
        DS->ConversionTimeoutMs = DS18B20_GetConversionTimeMs(DS->Resolution) + 50U;
    }

    memset(DS->DevAddr, 0, sizeof(DS->DevAddr));
    memset(DS->AlmAddr, 0, sizeof(DS->AlmAddr));
    memset(DS->Temperature, 0, sizeof(DS->Temperature));

    ow_st = OneWire_Init(OW);
    if (ow_st != ONEWIRE_STATUS_OK) {
        return DS18B20_MapOwStatus(ow_st);
    }

    while (OW->RomCnt < DS18B20_MaxCnt)
    {
        ow_st = OneWire_Search(OW, ONEWIRE_CMD_SEARCHROM);

        if (ow_st == ONEWIRE_STATUS_NOT_FOUND) {
            break;
        }

        if (ow_st != ONEWIRE_STATUS_OK) {
            return DS18B20_MapOwStatus(ow_st);
        }

        ow_st = OneWire_GetDevRom(OW, DS->DevAddr[OW->RomCnt]);
        if (ow_st != ONEWIRE_STATUS_OK) {
            return DS18B20_MapOwStatus(ow_st);
        }

        if (DS18B20_IsValid(DS->DevAddr[OW->RomCnt]) == 0U) {
            continue;
        }

        ds_st = DS18B20_SetResolution(OW, DS->DevAddr[OW->RomCnt], DS->Resolution);
        if (ds_st != DS18B20_STATUS_OK) {
            return ds_st;
        }

        ds_st = DS18B20_SetTempAlarm(OW, DS->DevAddr[OW->RomCnt], 0, 0);
        if (ds_st != DS18B20_STATUS_OK) {
            return ds_st;
        }

        OW->RomCnt++;
    }

    return (OW->RomCnt != 0U) ? DS18B20_STATUS_OK : DS18B20_STATUS_NOT_FOUND;
}

/* Enables the GPIO clock for the OneWire port. */
static void DS18B20_AppEnableGpioClock(GPIO_TypeDef *Port)
{
    /* Select the RCC clock gate that matches the supplied port. */
    if (Port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
    else if (Port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
    else if (Port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
    else if (Port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
    else if (Port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }
#ifdef GPIOF
    else if (Port == GPIOF) { __HAL_RCC_GPIOF_CLK_ENABLE(); }
#endif
#ifdef GPIOG
    else if (Port == GPIOG) { __HAL_RCC_GPIOG_CLK_ENABLE(); }
#endif
#ifdef GPIOH
    else if (Port == GPIOH) { __HAL_RCC_GPIOH_CLK_ENABLE(); }
#endif
}

/* Restores the OneWire pin's released open-drain state. */
static void DS18B20_AppGpioInit(GPIO_TypeDef *Port, uint16_t Pin)
{
    /* Drive the output high before configuring the GPIO mode. */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    DS18B20_AppEnableGpioClock(Port);

    HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Port, &GPIO_InitStruct);
}

 /* Initializes the DS18B20 application and recovery state. */
 DS18B20_Status_t DS18B20_AppInit(DS18B20_App_t *App,
                                  GPIO_TypeDef *Port,
                                  uint16_t Pin,
                                  TIM_HandleTypeDef *DelayTimer,
                                  DS18B20_Res_t Resolution,
                                  uint8_t ExpectedSensorCount)
 {
     /* Set the conversion timeout from resolution with an added margin. */
     GPIO_InitTypeDef GPIO_InitStruct = {0};
     DS18B20_Status_t st;

     if ((App == NULL) || (Port == NULL) || (Pin == 0U) || (DelayTimer == NULL)) {
         return DS18B20_STATUS_BAD_PARAM;
     }

     memset(App, 0, sizeof(*App));

     if (Port == GPIOA)      { __HAL_RCC_GPIOA_CLK_ENABLE(); }
     else if (Port == GPIOB) { __HAL_RCC_GPIOB_CLK_ENABLE(); }
     else if (Port == GPIOC) { __HAL_RCC_GPIOC_CLK_ENABLE(); }
     else if (Port == GPIOD) { __HAL_RCC_GPIOD_CLK_ENABLE(); }
     else if (Port == GPIOE) { __HAL_RCC_GPIOE_CLK_ENABLE(); }

     HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

     GPIO_InitStruct.Pin   = Pin;
     GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
     GPIO_InitStruct.Pull  = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
     HAL_GPIO_Init(Port, &GPIO_InitStruct);

     App->OW.DataPin    = Pin;
     App->OW.DataPort   = Port;
     App->OW.DelayTimer = DelayTimer;

     App->DS.Resolution          = Resolution;
     App->DS.ConversionTimeoutMs = DS18B20_GetConversionTimeMs(Resolution) + 50U;

     App->ExpectedSensorCount = ExpectedSensorCount;
     App->RecoveryAttempt     = 0U;
     App->RecoveryMaxAttempts = 3U;

     DS18B20_AppResetRuntimeState(App);

     st = DS18B20_AppBringupWithRecovery(App);
     if (st != DS18B20_STATUS_OK)
     {
         DS18B20_AppLogErrorDelay("[ERR] DS18B20_AppInit bring-up failed after 3 attempts: %s\r\n",
                                  DS18B20_StatusStr(st));
         return st;
     }

     return DS18B20_STATUS_OK;
 }

 /* ------------------------------------------------------------
  * DS18B20 x5
  *
  * IMPORTANT:
  * This publishes the FIVE temperatures as ONE atomic sensor group.
  *
  * Call this only after:
  *   1. all five DS18B20 reads succeeded
  *   2. AlarmSearch passed
  *
  * A partial 1/5, 2/5, etc. group is never published.
  * ------------------------------------------------------------ */
 /* Publishes the latest DS18B20 group temperatures. */
 bool telemetry_publish_ds18b20(
     const float temperature_c[
         TELEMETRY_DS18B20_COUNT])
 {
     /* Copy every configured sensor temperature into one generation-tagged sample. */
     telemetry_ds18b20_t sample;

     if ((q_ds18b20 == NULL) ||
         (temperature_c == NULL))
     {
         return false;
     }

     memset(&sample, 0, sizeof(sample));

     sample.sample_id =
         telemetry_next_generation(
             &ds18b20_generation);

     for (uint32_t i = 0U;
          i < TELEMETRY_DS18B20_COUNT;
          ++i)
     {
         sample.temperature_c[i] =
             temperature_c[i];
     }

     return
         (xQueueOverwrite(
             q_ds18b20,
             &sample) == pdPASS);
 }

 /* Advances DS18B20 conversion, readout and recovery handling. */
 DS18B20_AppRuntimeStatus_t DS18B20_AppRun(DS18B20_App_t *App)
 {
     /* Recover when conversion times out or a sensor read fails. */
     uint8_t i;
     uint8_t is_done = 0U;
     DS18B20_Status_t st;
     DS18B20_AppRuntimeStatus_t recovery_st;

     if (App == NULL) {
         return DS18B20_APP_RUNTIME_BAD_PARAM;
     }

     switch (App->State)
     {
         case DS18B20_APP_START_CONVERSION:
         {
             if (App->OW.RomCnt == 0U)
             {
                 DS18B20_AppLogErrorDelay("[ERR] No DS18B20 devices available\r\n");

                 recovery_st = DS18B20_AppRunRecoverySequence(App, "no-device at start-conversion");
                 if (recovery_st == DS18B20_APP_RUNTIME_RECOVERED) {
                     return DS18B20_APP_RUNTIME_RECOVERED;
                 }

                 return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
             }

             if ((App->ExpectedSensorCount != 0U) &&
                 (App->OW.RomCnt != App->ExpectedSensorCount))
             {
                 DS18B20_AppLogErrorDelay("[ERR] Expected %u sensors but have %u\r\n",
                                          (unsigned int)App->ExpectedSensorCount,
                                          (unsigned int)App->OW.RomCnt);

                 recovery_st = DS18B20_AppRunRecoverySequence(App, "wrong sensor count at start-conversion");
                 if (recovery_st == DS18B20_APP_RUNTIME_RECOVERED) {
                     return DS18B20_APP_RUNTIME_RECOVERED;
                 }

                 return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
             }

             st = DS18B20_StartAll(&App->OW);
             if (st != DS18B20_STATUS_OK)
             {
                 DS18B20_AppLogErrorDelay("[ERR] DS18B20_StartAll failed: %s\r\n",
                                          DS18B20_StatusStr(st));

                 recovery_st = DS18B20_AppRunRecoverySequence(App, "StartAll");
                 if (recovery_st == DS18B20_APP_RUNTIME_RECOVERED) {
                     return DS18B20_APP_RUNTIME_RECOVERED;
                 }

                 return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
             }

             App->DeadlineMs      = HAL_GetTick() + App->DS.ConversionTimeoutMs;
             App->TimeoutReported = 0U;
             App->State           = DS18B20_APP_WAIT_CONVERSION;
             return DS18B20_APP_RUNTIME_OK;
         }

         case DS18B20_APP_WAIT_CONVERSION:
         {
             st = DS18B20_IsConversionDone(&App->OW, &is_done);

             if (st != DS18B20_STATUS_OK)
             {
                 DS18B20_AppLogErrorDelay("[ERR] DS18B20_IsConversionDone failed: %s\r\n",
                                          DS18B20_StatusStr(st));

                 recovery_st = DS18B20_AppRunRecoverySequence(App, "IsConversionDone");
                 if (recovery_st == DS18B20_APP_RUNTIME_RECOVERED) {
                     return DS18B20_APP_RUNTIME_RECOVERED;
                 }

                 return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
             }

             if (is_done != 0U)
             {
                 App->State = DS18B20_APP_READ_AND_PRINT;
                 return DS18B20_APP_RUNTIME_OK;
             }

             if ((int32_t)(HAL_GetTick() - App->DeadlineMs) >= 0)
             {
                 if (App->TimeoutReported == 0U)
                 {
                     DS18B20_AppLogErrorDelay("[ERR] DS18B20 conversion timeout\r\n");
                     App->TimeoutReported = 1U;
                 }

                 recovery_st = DS18B20_AppRunRecoverySequence(App, "conversion timeout");
                 if (recovery_st == DS18B20_APP_RUNTIME_RECOVERED) {
                     return DS18B20_APP_RUNTIME_RECOVERED;
                 }

                 return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
             }

             DS18B20_AppSleepMs(1U);
             return DS18B20_APP_RUNTIME_OK;
         }

         case DS18B20_APP_READ_AND_PRINT:
         default:
         {
             for (i = 0U; i < App->OW.RomCnt; i++)
             {
                 st = DS18B20_Read(&App->OW, App->DS.DevAddr[i], &App->DS.Temperature[i]);

                 if (st != DS18B20_STATUS_OK)
                 {
                     DS18B20_AppLogErrorDelay("[ERR] DS18B20 sensor %u read failed: %s\r\n",
                                              (unsigned int)(i + 1U),
                                              DS18B20_StatusStr(st));

                     recovery_st = DS18B20_AppRunRecoverySequence(App, "sensor read failure");
                     if (recovery_st == DS18B20_APP_RUNTIME_RECOVERED) {
                         return DS18B20_APP_RUNTIME_RECOVERED;
                     }

                     return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
                 }

                 DS18B20_LogTemperatureSample((uint8_t)(i + 1U),
                                              App->DS.Temperature[i]);
                 DS18B20_AppSleepMs(50U);
             }

             st = DS18B20_AlarmSearch(&App->DS, &App->OW);
             if ((st != DS18B20_STATUS_OK) && (st != DS18B20_STATUS_NOT_FOUND))
             {
                 DS18B20_AppLogErrorDelay("[ERR] DS18B20_AlarmSearch failed: %s\r\n",
                                          DS18B20_StatusStr(st));

                 recovery_st = DS18B20_AppRunRecoverySequence(App, "alarm-search");
                 if (recovery_st == DS18B20_APP_RUNTIME_RECOVERED) {
                     return DS18B20_APP_RUNTIME_RECOVERED;
                 }

                 return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
             }

             (void)telemetry_publish_ds18b20(
                 App->DS.Temperature);

             DS18B20_LogGroupSeparator();

             App->RecoveryAttempt = 0U;
             DS18B20_AppSleepMs(250U);
             App->State = DS18B20_APP_START_CONVERSION;
             return DS18B20_APP_RUNTIME_OK;
         }
     }
 }

 /* Reports a DS18B20 temperature using fixed-point formatting. */
 static void DS18B20_LogTemperatureSample(uint8_t sensor_index, float temperature_c)
 {
     /* Split the scaled magnitude into whole and fractional digits. */
     int32_t scaled;
     int32_t whole;
     int32_t frac;

     if (DS18B20_NormalLoggingAllowed() == 0U)
     {
         return;
     }

     scaled = (int32_t)(temperature_c * 10000.0f);

     if (scaled < 0)
     {
         int32_t abs_scaled = -scaled;
         whole = abs_scaled / 10000;
         frac  = abs_scaled % 10000;

         DS18B20_LOG_PRINTF("[DS18B20][%u] -%ld.%04ld C\r\n",
                            (unsigned int)sensor_index,
                            (long)whole,
                            (long)frac);
     }
     else
     {
         whole = scaled / 10000;
         frac  = scaled % 10000;

         DS18B20_LOG_PRINTF("[DS18B20][%u] %ld.%04ld C\r\n",
                            (unsigned int)sensor_index,
                            (long)whole,
                            (long)frac);
     }
 }

 /* Separates successive DS18B20 diagnostic sample groups. */
 static void DS18B20_LogGroupSeparator(void)
 {
     /* Emit the group boundary through the sensor logging path. */
     if (DS18B20_NormalLoggingAllowed() == 0U)
     {
         return;
     }

     DS18B20_LOG_PRINTF("----- end of DS18B20 sample group -----\r\n");
 }

/* Retains the unused DS18B20 timer callback entry point. */
void DS18B20_AppTimerCallback(DS18B20_App_t *App, TIM_HandleTypeDef *htim)
{
    /* The blocking driver leaves both callback arguments unused. */
    (void)App;
    (void)htim;
}

