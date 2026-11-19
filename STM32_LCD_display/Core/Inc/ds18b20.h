/**
 * @file ds18b20.h
 * @brief DS18B20 driver, runtime state and application API declarations.
 */
#ifndef INC_DS18B20_H_
#define INC_DS18B20_H_
#include "onewire.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Final bus configuration uses five DS18B20 sensors. */
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
/* Shared system fault slot assigned to the DS18B20 group. */
#ifndef SYSTEM_FAULT_OWNER_DS18B20
#define SYSTEM_FAULT_OWNER_DS18B20      5U
#endif
#ifndef SYSTEM_FAULT_MASK_DS18B20
#define SYSTEM_FAULT_MASK_DS18B20       (1UL << 4)
#endif
/* Shared UART2/log mutex created by main.c. */
extern SemaphoreHandle_t xSerialSemaphore;
/* Serialize task-level DS18B20 logs once the scheduler is running. */
#ifndef DS18B20_LOG_PRINTF
#define DS18B20_LOG_PRINTF(...)                                                   \
   do {                                                                          \
       if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&                \
           (xSerialSemaphore != NULL)) {                                         \
           if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) == pdTRUE) { \
               log_printf(__VA_ARGS__);                                          \
               xSemaphoreGive(xSerialSemaphore);                                 \
           }                                                                     \
       } else {                                                                  \
           log_printf(__VA_ARGS__);                                              \
       }                                                                         \
   } while (0)
#endif
/** @brief Supported DS18B20 conversion resolutions. */
typedef enum {
   DS18B20_Resolution_9bits  = 9,
   DS18B20_Resolution_10bits = 10,
   DS18B20_Resolution_11bits = 11,
   DS18B20_Resolution_12bits = 12
} DS18B20_Res_t;
/** @brief DS18B20 driver status codes. */
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
/** @brief DS18B20 bus devices, temperatures and conversion settings. */
typedef struct
{
   uint8_t         DevAddr[DS18B20_MaxCnt][8];
   uint8_t         AlmAddr[DS18B20_MaxCnt][8];
   float           Temperature[DS18B20_MaxCnt];
   DS18B20_Res_t   Resolution;
   uint32_t        ConversionTimeoutMs;
} DS18B20_Drv_t;
/** @brief Non-blocking application state for one conversion cycle. */
typedef enum
{
   DS18B20_APP_START_CONVERSION = 0,
   DS18B20_APP_WAIT_CONVERSION,
   DS18B20_APP_READ_AND_PRINT
} DS18B20_AppState_t;
/** @brief DS18B20 application runtime status. */
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
/** @brief DS18B20 application state and bounded recovery tracking. */
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
/**
 * @brief Initialize the DS18B20 driver and discover devices.
 * @param DS Driver context.
 * @param OW OneWire context.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_Init(DS18B20_Drv_t *DS, OneWire_t *OW);
/**
 * @brief Start conversion on one DS18B20.
 * @param OW OneWire context.
 * @param ROM Device ROM address.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_Start(OneWire_t *OW, const uint8_t *ROM);
/**
 * @brief Start conversion on every device on the bus.
 * @param OW OneWire context.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_StartAll(OneWire_t *OW);
/**
 * @brief Read one DS18B20 temperature.
 * @param OW OneWire context.
 * @param ROM Device ROM address.
 * @param destination Temperature output.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_Read(OneWire_t *OW, const uint8_t *ROM, float *destination);
/**
 * @brief Configure the temperature alarm limits.
 * @param OW OneWire context.
 * @param ROM Device ROM address.
 * @param Low Low alarm threshold.
 * @param High High alarm threshold.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_SetTempAlarm(OneWire_t *OW, const uint8_t *ROM, int8_t Low, int8_t High);
/**
 * @brief Search for devices with an active alarm.
 * @param DS Driver context.
 * @param OW OneWire context.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_AlarmSearch(DS18B20_Drv_t *DS, OneWire_t *OW);
/**
 * @brief Read the configured resolution from one device.
 * @param OW OneWire context.
 * @param ROM Device ROM address.
 * @param resolution Resolution output.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_GetResolution(OneWire_t *OW, const uint8_t *ROM, uint8_t *resolution);
/**
 * @brief Set the resolution of one DS18B20.
 * @param OW OneWire context.
 * @param ROM Device ROM address.
 * @param Resolution Requested resolution.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_SetResolution(OneWire_t *OW, const uint8_t *ROM, DS18B20_Res_t Resolution);
/**
 * @brief Check whether the current conversion is complete.
 * @param OW OneWire context.
 * @param is_done Completion output flag.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_IsConversionDone(OneWire_t *OW, uint8_t *is_done);
/**
 * @brief Get the maximum conversion time for a resolution.
 * @param Resolution Sensor resolution.
 * @return Conversion time in milliseconds.
 */
uint32_t         DS18B20_GetConversionTimeMs(DS18B20_Res_t Resolution);
/**
 * @brief Initialize the five-sensor DS18B20 application context.
 * @param App Application context.
 * @param Port OneWire GPIO port.
 * @param Pin OneWire GPIO pin.
 * @param DelayTimer Timer used for microsecond delays.
 * @param Resolution Sensor resolution.
 * @param ExpectedSensorCount Required sensor count.
 * @return DS18B20 status.
 */
DS18B20_Status_t DS18B20_AppInit(DS18B20_App_t *App,
                                GPIO_TypeDef *Port,
                                uint16_t Pin,
                                TIM_HandleTypeDef *DelayTimer,
                                DS18B20_Res_t Resolution,
                                uint8_t ExpectedSensorCount);
/**
 * @brief Run one DS18B20 application state-machine step.
 * @param App Application context.
 * @return Runtime status.
 */
DS18B20_AppRuntimeStatus_t DS18B20_AppRun(DS18B20_App_t *App);
/**
 * @brief Forward a timer callback to the DS18B20 application.
 * @param App Application context.
 * @param htim Timer handle.
 */
void DS18B20_AppTimerCallback(DS18B20_App_t *App, TIM_HandleTypeDef *htim);
#ifdef __cplusplus
}
#endif
#endif /* INC_DS18B20_H_ */
