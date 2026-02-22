/* ds18b20.h | Author: i fiksuari pas prog | Date: 22 February 2026 */
#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_

#include "onewire.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* For one-sensor bring-up keep this as 1U.
 * For final 5-sensor bus, change this to 5U and pass ExpectedSensorCount=5U.
 */
#define DS18B20_MaxCnt                  5U

#define DS18B20_CMD_CONVERT             0x44U
#define DS18B20_CMD_ALARM_SEARCH        0xECU
#define DS18B20_CMD_READSCRATCHPAD      0xBEU
#define DS18B20_CMD_WRITESCRATCHPAD     0x4EU
#define DS18B20_CMD_COPYSCRATCHPAD      0x48U

#define DS18B20_FAMILY_CODE             0x28U

#define DS18B20_RESOLUTION_R1           6U
#define DS18B20_RESOLUTION_R0           5U

#define DS18B20_DECIMAL_STEPS_12BIT     0.0625f
#define DS18B20_DECIMAL_STEPS_11BIT     0.125f
#define DS18B20_DECIMAL_STEPS_10BIT     0.25f
#define DS18B20_DECIMAL_STEPS_9BIT      0.5f

typedef enum {
    DS18B20_Resolution_9bits  = 9,
    DS18B20_Resolution_10bits = 10,
    DS18B20_Resolution_11bits = 11,
    DS18B20_Resolution_12bits = 12
} DS18B20_Res_t;

typedef enum
{
    DS18B20_STATUS_OK = 0,
    DS18B20_STATUS_BAD_PARAM,
    DS18B20_STATUS_INVALID_ROM,
    DS18B20_STATUS_NO_DEVICE,
    DS18B20_STATUS_NOT_FOUND,
    DS18B20_STATUS_BUS_ERROR,
    DS18B20_STATUS_CRC_ERROR,
    DS18B20_STATUS_TIMEOUT
} DS18B20_Status_t;

typedef struct
{
    uint8_t         DevAddr[DS18B20_MaxCnt][8];
    uint8_t         AlmAddr[DS18B20_MaxCnt][8];
    float           Temperature[DS18B20_MaxCnt];
    DS18B20_Res_t   Resolution;
    uint32_t        ConversionTimeoutMs;
} DS18B20_Drv_t;

typedef enum
{
    DS18B20_APP_START_CONVERSION = 0,
    DS18B20_APP_WAIT_CONVERSION,
    DS18B20_APP_READ_AND_PRINT
} DS18B20_AppState_t;

typedef enum
{
    DS18B20_APP_RUNTIME_OK = 0,
    DS18B20_APP_RUNTIME_RECOVERED,
    DS18B20_APP_RUNTIME_BAD_PARAM,
    DS18B20_APP_RUNTIME_NO_DEVICE,
    DS18B20_APP_RUNTIME_START_FAIL,
    DS18B20_APP_RUNTIME_WAIT_FAIL,
    DS18B20_APP_RUNTIME_READ_FAIL,
    DS18B20_APP_RUNTIME_ALARM_FAIL,
    DS18B20_APP_RUNTIME_RECOVERY_FAIL,
    DS18B20_APP_RUNTIME_FATAL
} DS18B20_AppRuntimeStatus_t;

typedef struct
{
    DS18B20_Drv_t              DS;
    OneWire_t                  OW;
    DS18B20_AppState_t         State;
    uint32_t                   DeadlineMs;
    uint8_t                    TimeoutReported;
    uint8_t                    ExpectedSensorCount;

    uint8_t                    RecoveryAttempt;
    uint8_t                    RecoveryMaxAttempts;
} DS18B20_App_t;

DS18B20_Status_t DS18B20_Init(DS18B20_Drv_t *DS, OneWire_t *OW);
DS18B20_Status_t DS18B20_Start(OneWire_t *OW, const uint8_t *ROM);
DS18B20_Status_t DS18B20_StartAll(OneWire_t *OW);
DS18B20_Status_t DS18B20_Read(OneWire_t *OW, const uint8_t *ROM, float *destination);
DS18B20_Status_t DS18B20_SetTempAlarm(OneWire_t *OW, const uint8_t *ROM, int8_t Low, int8_t High);
DS18B20_Status_t DS18B20_AlarmSearch(DS18B20_Drv_t *DS, OneWire_t *OW);
DS18B20_Status_t DS18B20_GetResolution(OneWire_t *OW, const uint8_t *ROM, uint8_t *resolution);
DS18B20_Status_t DS18B20_SetResolution(OneWire_t *OW, const uint8_t *ROM, DS18B20_Res_t Resolution);
DS18B20_Status_t DS18B20_IsConversionDone(OneWire_t *OW, uint8_t *is_done);
uint32_t         DS18B20_GetConversionTimeMs(DS18B20_Res_t Resolution);

DS18B20_Status_t DS18B20_AppInit(DS18B20_App_t *App,
                                 GPIO_TypeDef *Port,
                                 uint16_t Pin,
                                 TIM_HandleTypeDef *DelayTimer,
                                 DS18B20_Res_t Resolution,
                                 uint8_t ExpectedSensorCount);

DS18B20_AppRuntimeStatus_t DS18B20_AppRun(DS18B20_App_t *App);
void DS18B20_AppTimerCallback(DS18B20_App_t *App, TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* INC_DS18B20_H_ */
