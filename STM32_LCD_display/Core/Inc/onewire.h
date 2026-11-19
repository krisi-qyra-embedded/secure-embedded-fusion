/**
 * @file onewire.h
 * @brief Blocking 1-Wire bus, ROM search and CRC API declarations.
 */
#ifndef INC_ONEWIRE_H_
#define INC_ONEWIRE_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Standard 1-Wire ROM commands. */
#define ONEWIRE_CMD_SEARCHROM            0xF0U
#define ONEWIRE_CMD_READROM              0x33U
#define ONEWIRE_CMD_MATCHROM             0x55U
#define ONEWIRE_CMD_SKIPROM              0xCCU
/** @brief GPIO direction used by the 1-Wire bus. */
typedef enum
{
   Input = 0,
   Output
} PinMode;
/** @brief Legacy operation state kept for API compatibility. */
typedef enum
{
   ONEWIRE_OP_NONE = 0,
   ONEWIRE_OP_RESET,
   ONEWIRE_OP_READ_BIT,
   ONEWIRE_OP_WRITE_BIT
} OneWire_Operation_t;
/** @brief 1-Wire driver status. */
typedef enum
{
   ONEWIRE_STATUS_OK = 0,
   ONEWIRE_STATUS_BAD_PARAM,
   ONEWIRE_STATUS_NO_DEVICE,
   ONEWIRE_STATUS_NOT_FOUND,
   ONEWIRE_STATUS_BUS_ERROR
} OneWire_Status_t;
/** @brief 1-Wire bus state and ROM-search context. */
typedef struct
{
   uint8_t             LastDiscrepancy;
   uint8_t             LastFamilyDiscrepancy;
   uint8_t             LastDeviceFlag;
   uint8_t             RomByte[8];
   uint8_t             RomCnt;
   uint16_t            DataPin;
   GPIO_TypeDef       *DataPort;
   /* Kept for DS18B20 application API compatibility. */
   TIM_HandleTypeDef  *DelayTimer;
   /* Legacy interrupt fields are unused by the blocking driver. */
   volatile uint8_t    IrqDone;
   volatile uint8_t    ReadSample;
   volatile uint8_t    ResetPresence;
   volatile uint8_t    WriteBitValue;
   volatile uint8_t    OpPhase;
   volatile OneWire_Operation_t ActiveOp;
} OneWire_t;
/* Shared USART2 logging mutex. */
extern SemaphoreHandle_t xSerialSemaphore;
/* Serialize 1-Wire logs once the scheduler is running. */
#define ONEWIRE_LOG_PRINTF(...)                                                     \
   do {                                                                            \
       if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&                  \
           (xSerialSemaphore != NULL)) {                                           \
           if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) == pdTRUE) {   \
               log_printf(__VA_ARGS__);                                            \
               xSemaphoreGive(xSerialSemaphore);                                   \
           }                                                                       \
       } else {                                                                    \
           log_printf(__VA_ARGS__);                                                \
       }                                                                           \
   } while (0)
/**
 * @brief Initialize the 1-Wire bus context.
 * @param OW Bus context.
 * @return 1-Wire status.
 */
OneWire_Status_t OneWire_Init(OneWire_t *OW);
/**
 * @brief Forward a timer callback to the legacy 1-Wire context.
 * @param OW Bus context.
 * @param htim Timer handle.
 */
void             OneWire_TimerElapsedCallback(OneWire_t *OW, TIM_HandleTypeDef *htim);
/**
 * @brief Run a 1-Wire ROM search.
 * @param OW Bus context.
 * @param Cmd ROM search command.
 * @return 1-Wire status.
 */
OneWire_Status_t OneWire_Search(OneWire_t *OW, uint8_t Cmd);
/**
 * @brief Copy the current discovered ROM address.
 * @param OW Bus context.
 * @param dev ROM output buffer.
 * @return 1-Wire status.
 */
OneWire_Status_t OneWire_GetDevRom(OneWire_t *OW, uint8_t *dev);
/**
 * @brief Reset the 1-Wire bus and detect presence.
 * @param OW Bus context.
 * @return 1-Wire status.
 */
OneWire_Status_t OneWire_Reset(OneWire_t *OW);
/**
 * @brief Read one bit from the 1-Wire bus.
 * @param OW Bus context.
 * @param bit Bit output.
 * @return 1-Wire status.
 */
OneWire_Status_t OneWire_ReadBit(OneWire_t *OW, uint8_t *bit);
/**
 * @brief Read one byte from the 1-Wire bus.
 * @param OW Bus context.
 * @param byte Byte output.
 * @return 1-Wire status.
 */
OneWire_Status_t OneWire_ReadByte(OneWire_t *OW, uint8_t *byte);
/**
 * @brief Write one byte to the 1-Wire bus.
 * @param OW Bus context.
 * @param byte Byte to write.
 * @return 1-Wire status.
 */
OneWire_Status_t OneWire_WriteByte(OneWire_t *OW, uint8_t byte);
/**
 * @brief Select one device by ROM address.
 * @param OW Bus context.
 * @param Rom Device ROM address.
 * @return 1-Wire status.
 */
OneWire_Status_t OneWire_SelectWithPointer(OneWire_t *OW, const uint8_t *Rom);
/**
 * @brief Calculate the Dallas/Maxim CRC-8.
 * @param addr Input buffer.
 * @param len Number of bytes.
 * @return CRC-8 value.
 */
uint8_t OneWire_CRC8(const uint8_t *addr, uint8_t len);
#ifdef __cplusplus
}
#endif
#endif /* INC_ONEWIRE_H_ */
