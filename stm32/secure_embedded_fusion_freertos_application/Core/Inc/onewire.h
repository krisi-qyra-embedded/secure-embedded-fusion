/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
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

#define ONEWIRE_CMD_SEARCHROM            0xF0U
#define ONEWIRE_CMD_READROM              0x33U
#define ONEWIRE_CMD_MATCHROM             0x55U
#define ONEWIRE_CMD_SKIPROM              0xCCU

typedef enum
{
    Input = 0,
    Output
} PinMode;

typedef enum
{
    ONEWIRE_OP_NONE = 0,
    ONEWIRE_OP_RESET,
    ONEWIRE_OP_READ_BIT,
    ONEWIRE_OP_WRITE_BIT
} OneWire_Operation_t;

typedef enum
{
    ONEWIRE_STATUS_OK = 0,
    ONEWIRE_STATUS_BAD_PARAM,
    ONEWIRE_STATUS_NO_DEVICE,
    ONEWIRE_STATUS_NOT_FOUND,
    ONEWIRE_STATUS_BUS_ERROR
} OneWire_Status_t;

typedef struct
{
    uint8_t             LastDiscrepancy;
    uint8_t             LastFamilyDiscrepancy;
    uint8_t             LastDeviceFlag;
    uint8_t             RomByte[8];
    uint8_t             RomCnt;

    uint16_t            DataPin;
    GPIO_TypeDef       *DataPort;

    TIM_HandleTypeDef  *DelayTimer;

    volatile uint8_t    IrqDone;
    volatile uint8_t    ReadSample;
    volatile uint8_t    ResetPresence;
    volatile uint8_t    WriteBitValue;
    volatile uint8_t    OpPhase;
    volatile OneWire_Operation_t ActiveOp;
} OneWire_t;

/*
 * Shared UART2/log mutex.
 *
 * Owned/created in main.c:
 *     xSerialSemaphore = xSemaphoreCreateMutex();
 *
 * Same style as SPI_DMA_LOG_PRINTF:
 * if FreeRTOS is running, take the UART/log mutex before log_printf().
 * If scheduler is not running yet, print directly.
 */
extern SemaphoreHandle_t xSerialSemaphore;

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

OneWire_Status_t OneWire_Init(OneWire_t *OW);
void             OneWire_TimerElapsedCallback(OneWire_t *OW, TIM_HandleTypeDef *htim);

OneWire_Status_t OneWire_Search(OneWire_t *OW, uint8_t Cmd);
OneWire_Status_t OneWire_GetDevRom(OneWire_t *OW, uint8_t *dev);

OneWire_Status_t OneWire_Reset(OneWire_t *OW);
OneWire_Status_t OneWire_ReadBit(OneWire_t *OW, uint8_t *bit);
OneWire_Status_t OneWire_ReadByte(OneWire_t *OW, uint8_t *byte);
OneWire_Status_t OneWire_WriteByte(OneWire_t *OW, uint8_t byte);
OneWire_Status_t OneWire_SelectWithPointer(OneWire_t *OW, const uint8_t *Rom);

uint8_t OneWire_CRC8(const uint8_t *addr, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* INC_ONEWIRE_H_ */
