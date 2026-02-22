/* ds18b20.c | Author: i fiksuari pas prog | Date: 22 February 2026 */
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
#include "systick.h"

static void DS18B20_LogGroupSeparator(void);
static void DS18B20_LogTemperatureSample(uint8_t sensor_index, float temperature_c);
static void DS18B20_AppResetRuntimeState(DS18B20_App_t *App);
static DS18B20_Status_t DS18B20_MapOwStatus(OneWire_Status_t st);
static const char *DS18B20_StatusStr(DS18B20_Status_t st);
static void DS18B20_AppGpioInit(GPIO_TypeDef *Port, uint16_t Pin);


/* Waits between sensor operations while allowing CPU sleep. */
static void DS18B20_AppSleepMs(uint32_t ms)
{
    /* Use the tick deadline so the delay also works across timer rollover. */
    uint32_t deadline = HAL_GetTick() + ms;

    while ((int32_t)(HAL_GetTick() - deadline) < 0)
    {
        cpu_sleep_until_irq();
    }
}

/* Logs a driver error and applies a short delay. */
static void DS18B20_AppLogErrorDelay(const char *fmt, ...)
{
    char tmp[128];
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
        log_write_bytes((const uint8_t *)tmp, (uint32_t)n);
    }

    /* user requested 10 ms delay after error print */
    delay_ms(10U);
}

/* Clears previously discovered sensor addresses and state. */
static void DS18B20_AppClearDiscoveryState(DS18B20_App_t *App)
{
    /* Reset the saved ROM list before running a new discovery. */
    if (App == NULL) {
        return;
    }

    App->OW.RomCnt = 0U;
    memset(App->DS.DevAddr, 0, sizeof(App->DS.DevAddr));
    memset(App->DS.AlmAddr, 0, sizeof(App->DS.AlmAddr));
    memset(App->DS.Temperature, 0, sizeof(App->DS.Temperature));
}


/* Checks that all expected sensors respond with valid readings. */
static DS18B20_Status_t DS18B20_AppVerifyAllSensorsReady(DS18B20_App_t *App)
{
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

    /*
     * All expected sensors must actually produce a value.
     * So we trigger a conversion, wait it out, then read every sensor.
     */
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


/* Runs one stage of the DS18B20 recovery procedure. */
static DS18B20_Status_t DS18B20_AppApplyRecoveryPhase(DS18B20_App_t *App,
                                                      uint8_t phase)
{
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

            /*
             * Phase 1 does not rebuild the table.
             * It only tries to recover the existing bus state
             * and then verifies that all 5 sensors can still produce values.
             */
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


/* Runs the bounded multi-stage recovery sequence. */
static DS18B20_AppRuntimeStatus_t DS18B20_AppRunRecoverySequence(DS18B20_App_t *App,
                                                                 const char *reason)
{
    /* Stop on the first successful phase and escalate if all phases fail. */
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

    return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
}


/* Initializes the sensor group with recovery on failure. */
static DS18B20_Status_t DS18B20_AppBringupWithRecovery(DS18B20_App_t *App)
{
    /* Retry the sensor bring-up before reporting a persistent fault. */
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


/* Checks the DS18B20 family code in a device ROM. */
static uint8_t DS18B20_IsValid(const uint8_t *ROM)
{
    /* Handle the operation and its status locally. */
    return ((ROM != NULL) && (ROM[0] == DS18B20_FAMILY_CODE)) ? 1U : 0U;
}

/* Maps OneWire driver results to DS18B20 status values. */
static DS18B20_Status_t DS18B20_MapOwStatus(OneWire_Status_t st)
{
    /* Keep the OneWire error mapping in one place. */
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

/* Implements the DS18B20 StatusStr operation. */
static const char *DS18B20_StatusStr(DS18B20_Status_t st)
{
    /* Handle the operation and its status locally. */
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


/* Checks whether the expected number of sensors is present. */
static uint8_t DS18B20_AppSensorCountHealthy(const DS18B20_App_t *App)
{
    /* Compare the bus discovery count with the expected device count. */
    if (App == NULL) {
        return 0U;
    }

    if (App->ExpectedSensorCount == 0U) {
        return (App->OW.RomCnt > 0U) ? 1U : 0U;
    }

    return (App->OW.RomCnt == App->ExpectedSensorCount) ? 1U : 0U;
}


/* Restores the initial application conversion state. */
static void DS18B20_AppResetRuntimeState(DS18B20_App_t *App)
{
    /* Start the conversion cycle from a known state. */
    App->State           = DS18B20_APP_START_CONVERSION;
    App->DeadlineMs      = 0U;
    App->TimeoutReported = 0U;
}


/* Recovers the OneWire bus and reinitializes the sensor group. */
static DS18B20_AppRuntimeStatus_t DS18B20_AppRecoverAndReinit(DS18B20_App_t *App)
{
    /* Rebuild the sensor state after the bus recovery completes. */
    DS18B20_Status_t ds_st;
    OneWire_Status_t ow_st;

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
            DS18B20_AppResetRuntimeState(App);
            return DS18B20_APP_RUNTIME_RECOVERED;
        }

        default:
            return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
    }
}


/* Handles a failed sensor read using the configured recovery steps. */
static DS18B20_AppRuntimeStatus_t DS18B20_AppRecoverFailedSensor(DS18B20_App_t *App,
                                                                 uint8_t sensor_index)
{
    /* Try recovery for the failed sensor without accepting a stale reading. */
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

    return DS18B20_APP_RUNTIME_RECOVERY_FAIL;
}


/* Returns the conversion duration for the selected resolution. */
uint32_t DS18B20_GetConversionTimeMs(DS18B20_Res_t Resolution)
{
    /* Use the configured resolution to select the wait interval. */
    switch (Resolution)
    {
        case DS18B20_Resolution_9bits:  return 94U;
        case DS18B20_Resolution_10bits: return 188U;
        case DS18B20_Resolution_11bits: return 375U;
        case DS18B20_Resolution_12bits:
        default:                        return 750U;
    }
}

/* Reads the resolution configured in the device scratchpad. */
DS18B20_Status_t DS18B20_GetResolution(OneWire_t *OW, const uint8_t *ROM, uint8_t *resolution)
{
    /* Read the configuration byte after selecting the sensor ROM. */
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

/* Updates the DS18B20 conversion resolution. */
DS18B20_Status_t DS18B20_SetResolution(OneWire_t *OW, const uint8_t *ROM, DS18B20_Res_t Resolution)
{
    /* Update the scratchpad configuration of the selected device. */
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

/* Starts temperature conversion on one selected device. */
DS18B20_Status_t DS18B20_Start(OneWire_t *OW, const uint8_t *ROM)
{
    /* Select the sensor before issuing the conversion command. */
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

/* Broadcasts the temperature conversion command to the bus. */
DS18B20_Status_t DS18B20_StartAll(OneWire_t *OW)
{
    /* Use the skip-ROM command to start conversion for all devices. */
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

/* Checks whether the temperature conversion has finished. */
DS18B20_Status_t DS18B20_IsConversionDone(OneWire_t *OW, uint8_t *is_done)
{
    /* Keep conversion completion separate from the temperature read. */
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

/* Reads and converts a temperature measurement from the scratchpad. */
DS18B20_Status_t DS18B20_Read(OneWire_t *OW, const uint8_t *ROM, float *Destination)
{
    /* Check the scratchpad data before updating the temperature value. */
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

/* Configures the high and low temperature alarm thresholds. */
DS18B20_Status_t DS18B20_SetTempAlarm(OneWire_t *OW, const uint8_t *ROM, int8_t Low, int8_t High)
{
    /* Write both alarm thresholds to the selected scratchpad. */
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

/* Searches the bus for devices with active alarms. */
DS18B20_Status_t DS18B20_AlarmSearch(DS18B20_Drv_t *DS, OneWire_t *OW)
{
    /* Only collect ROM addresses returned by the alarm search. */
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

/* Discovers and initializes the connected DS18B20 devices. */
DS18B20_Status_t DS18B20_Init(DS18B20_Drv_t *DS, OneWire_t *OW)
{
    /* Enumerate the ROM codes before applying device settings. */
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

/* Enables the GPIO clock used by the OneWire bus. */
static void DS18B20_AppEnableGpioClock(GPIO_TypeDef *Port)
{
    /* Enable the GPIO port clock before configuring the bus pin. */
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

/* Configures the OneWire GPIO pin. */
static void DS18B20_AppGpioInit(GPIO_TypeDef *Port, uint16_t Pin)
{
    /* Set the data-pin mode needed by the OneWire driver. */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    DS18B20_AppEnableGpioClock(Port);

    HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Port, &GPIO_InitStruct);
}

 /* Sets up the sensor group and runtime state. */
 DS18B20_Status_t DS18B20_AppInit(DS18B20_App_t *App,
                                  GPIO_TypeDef *Port,
                                  uint16_t Pin,
                                  TIM_HandleTypeDef *DelayTimer,
                                  DS18B20_Res_t Resolution,
                                  uint8_t ExpectedSensorCount)
 {
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

     /*
      * On boot and after every MCU reset:
      * all expected sensors must be detected and must produce values.
      * If not, perform 3 recovery attempts here before returning failure.
      */
     st = DS18B20_AppBringupWithRecovery(App);
     if (st != DS18B20_STATUS_OK)
     {
         DS18B20_AppLogErrorDelay("[ERR] DS18B20_AppInit bring-up failed after 3 attempts: %s\r\n",
                                  DS18B20_StatusStr(st));
         return st;
     }

     return DS18B20_STATUS_OK;
 }


 /* Advances the sensor conversion and readout state machine. */
 DS18B20_AppRuntimeStatus_t DS18B20_AppRun(DS18B20_App_t *App)
 {
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

             cpu_sleep_until_irq();
             return DS18B20_APP_RUNTIME_OK;
         }

         case DS18B20_APP_READ_AND_PRINT:
         default:
         {
             /*
              * All expected sensors must produce values.
              * If even one sensor fails, stop the entire group immediately,
              * run the 3 recovery strategies, and only then either resume
              * from scratch or force system reset in main().
              */
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

             DS18B20_LogGroupSeparator();

             App->RecoveryAttempt = 0U;
             DS18B20_AppSleepMs(250U);
             App->State = DS18B20_APP_START_CONVERSION;
             return DS18B20_APP_RUNTIME_OK;
         }
     }
 }


/* Formats and prints a sensor temperature sample. */
static void DS18B20_LogTemperatureSample(uint8_t sensor_index, float temperature_c)
{
    /* Format the sample with its sensor index for easier tracing. */
    int32_t scaled;
    int32_t whole;
    int32_t frac;

    scaled = (int32_t)(temperature_c * 10000.0f);

    if (scaled < 0)
    {
        int32_t abs_scaled = -scaled;
        whole = abs_scaled / 10000;
        frac  = abs_scaled % 10000;
        log_printf("[DS18B20][%u] -%ld.%04ld C\r\n",
                   (unsigned int)sensor_index,
                   (long)whole,
                   (long)frac);
    }
    else
    {
        whole = scaled / 10000;
        frac  = scaled % 10000;
        log_printf("[DS18B20][%u] %ld.%04ld C\r\n",
                   (unsigned int)sensor_index,
                   (long)whole,
                   (long)frac);
    }
}



/* Prints a separator between groups of sensor readings. */
static void DS18B20_LogGroupSeparator(void)
{
    /* Separate successive readout groups in the UART log. */
    log_printf("----- end of DS18B20 sample group -----\r\n");
}

/* Handles the application timer callback when enabled. */
void DS18B20_AppTimerCallback(DS18B20_App_t *App, TIM_HandleTypeDef *htim)
{
    (void)App;
    (void)htim;
    /* Blocking OneWire driver does not use timer callbacks. Kept for API compatibility. */
}


