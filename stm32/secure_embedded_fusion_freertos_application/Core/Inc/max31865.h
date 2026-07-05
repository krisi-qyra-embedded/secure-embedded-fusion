/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#ifndef __MAX31865_H__
#define __MAX31865_H__

#include <stdint.h>
#include "spi_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SemaphoreHandle_t xSerialSemaphore;

#ifndef MAX31865_LOG_PRINTF
#define MAX31865_LOG_PRINTF(...)                                                   \
    do {                                                                           \
        if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&                 \
            (xSerialSemaphore != NULL)) {                                          \
            if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) == pdTRUE) {  \
                log_printf(__VA_ARGS__);                                           \
                xSemaphoreGive(xSerialSemaphore);                                  \
            }                                                                      \
        } else {                                                                   \
            log_printf(__VA_ARGS__);                                               \
        }                                                                          \
    } while (0)
#endif


#define _MAX31865_USE_FREERTOS  1
#define _MAX31865_RREF          430.0f
#define _MAX31865_RNOMINAL      100.0f

#define MAX31856_CONFIG_REG             0x00u
#define MAX31856_CONFIG_BIAS            0x80u
#define MAX31856_CONFIG_MODEAUTO        0x40u
#define MAX31856_CONFIG_MODEOFF         0x00u
#define MAX31856_CONFIG_1SHOT           0x20u
#define MAX31856_CONFIG_3WIRE           0x10u
#define MAX31856_CONFIG_24WIRE          0x00u
#define MAX31856_CONFIG_FAULTSTAT       0x02u
#define MAX31856_CONFIG_FILT50HZ        0x01u
#define MAX31856_CONFIG_FILT60HZ        0x00u

#define MAX31856_RTDMSB_REG             0x01u
#define MAX31856_RTDLSB_REG             0x02u
#define MAX31856_HFAULTMSB_REG          0x03u
#define MAX31856_HFAULTLSB_REG          0x04u
#define MAX31856_LFAULTMSB_REG          0x05u
#define MAX31856_LFAULTLSB_REG          0x06u
#define MAX31856_FAULTSTAT_REG          0x07u

typedef enum
{
    APP_MAX31865_RUNTIME_OK = 0,
    APP_MAX31865_RUNTIME_READ_FAIL,
    APP_MAX31865_RUNTIME_UNEXPECTED_FAIL
} app_max31865_runtime_status_t;

typedef enum
{
    MAX31865_STATUS_OK             = 0,
    MAX31865_STATUS_INVALID_ARG    = -1,
    MAX31865_STATUS_SIZE_ERROR     = -2,
    MAX31865_STATUS_SPI_ERROR      = -3,
    MAX31865_STATUS_TIMEOUT        = -4,
    MAX31865_STATUS_FAULT          = -5,
    MAX31865_STATUS_DMA_ERROR      = -6,
    MAX31865_STATUS_RECOVERY_ERROR = -7,
    MAX31865_STATUS_INVALID_DATA   = -8,
	MAX31865_STATUS_INIT_FAILED    = -9
} max31865_status_t;

#define RREF_MILLI_OHMS 430000UL

#define MAX31865_CS_PORT    GPIOB
#define MAX31865_CS_PIN     GPIO_PIN_6

#define MAX31865_FAULT_HIGHTHRESH       0x80u
#define MAX31865_FAULT_LOWTHRESH        0x40u
#define MAX31865_FAULT_REFINLOW         0x20u
#define MAX31865_FAULT_REFINHIGH        0x10u
#define MAX31865_FAULT_RTDINLOW         0x08u
#define MAX31865_FAULT_OVUV             0x04u

#define CS_SELECT         0u
#define CS_DESELECT       1u

#define RTD_A 3.9083e-3f
#define RTD_B -5.775e-7f

#ifndef TRANSFER_TIMEOUT_MS
#define TRANSFER_TIMEOUT_MS  100u
#endif

#define MAX_STACK_TOTAL 64u

typedef struct
{
    uint8_t lock;
    uint8_t wires;
    uint8_t filter_hz;
} Max31865_t;

extern volatile uint32_t hb_main_ts;
extern Max31865_t max31865_dev;

int gpio_pb6_cs_init(void);
int MPU_CS(uint8_t state);

max31865_status_t Max31865_init(Max31865_t *max31865, uint8_t numwires, uint8_t filterHz);
max31865_status_t Max31865_readTempC(Max31865_t *max31865, float *readTemp);
max31865_status_t Max31865_readTempF(Max31865_t *max31865, float *readTemp);
float Max31865_Filter(float newInput, float lastOutput, float effectiveFactor);
max31865_status_t Max31865_debug_once(Max31865_t *dev);

max31865_status_t Max31865_readRegisterN(Max31865_t *max31865, uint8_t addr, uint8_t *buffer, uint8_t n);
max31865_status_t Max31865_readRegister8(Max31865_t *max31865, uint8_t addr, uint8_t *value);
max31865_status_t Max31865_readRegister16(Max31865_t *max31865, uint8_t addr, uint16_t *value);
max31865_status_t Max31865_writeRegister8(Max31865_t *dev, uint8_t addr, uint8_t data);

max31865_status_t Max31865_readFault(Max31865_t *max31865, uint8_t *out_fault);
max31865_status_t Max31865_clearFault(Max31865_t *max31865);
max31865_status_t Max31865_enableBias(Max31865_t *max31865, uint8_t enable);
max31865_status_t Max31865_autoConvert(Max31865_t *max31865, uint8_t enable);
max31865_status_t Max31865_setWires(Max31865_t *max31865, uint8_t numWires);
max31865_status_t Max31865_setFilter(Max31865_t *max31865, uint8_t filterHz);
max31865_status_t Max31865_readRTD(Max31865_t *max31865, uint16_t *rtd_value);
app_max31865_runtime_status_t app_max31865_runtime_process(void);
max31865_status_t Max31865_monitorLoop(Max31865_t *max31865, uint32_t interval_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MAX31865_H__ */
