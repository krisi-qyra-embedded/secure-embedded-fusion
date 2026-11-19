/**
 * @file max31865.h
 * @brief MAX31865 configuration, SPI access and runtime API declarations.
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
/* Shared USART2 logging mutex. */
extern SemaphoreHandle_t xSerialSemaphore;
/* Serialize MAX31865 logs once the scheduler is running. */
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
/* Driver configuration and RTD reference values. */
#define _MAX31865_USE_FREERTOS  1
#define _MAX31865_RREF          430.0f
#define _MAX31865_RNOMINAL      100.0f
/* Existing MAX31856-prefixed names map the MAX31865 register/config fields. */
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
/** @brief Application-level MAX31865 runtime status. */
typedef enum
{
   APP_MAX31865_RUNTIME_OK = 0,
   APP_MAX31865_RUNTIME_READ_FAIL,
   APP_MAX31865_RUNTIME_UNEXPECTED_FAIL
} app_max31865_runtime_status_t;
/** @brief MAX31865 driver status. */
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
/* RTD reference, chip-select and fault definitions. */
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
/* Callendar-Van Dusen coefficients for the PT100 conversion. */
#define RTD_A 3.9083e-3f
#define RTD_B -5.775e-7f
#ifndef TRANSFER_TIMEOUT_MS
#define TRANSFER_TIMEOUT_MS  100u
#endif
#define MAX_STACK_TOTAL 64u
/** @brief MAX31865 runtime configuration. */
typedef struct
{
   uint8_t lock;
   uint8_t wires;
   uint8_t filter_hz;
} Max31865_t;
/* Shared application heartbeat and device instance. */
extern volatile uint32_t hb_main_ts;
extern Max31865_t max31865_dev;
/**
 * @brief Initialize PB6 as the MAX31865 chip-select pin.
 * @return Initialization result.
 */
int gpio_pb6_cs_init(void);
/**
 * @brief Drive the MAX31865 chip-select state.
 * @param state CS_SELECT or CS_DESELECT.
 * @return Chip-select operation result.
 */
int MPU_CS(uint8_t state);
/**
 * @brief Initialize the MAX31865 device.
 * @param max31865 Device context.
 * @param numwires RTD wiring configuration.
 * @param filterHz Mains filter selection.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_init(Max31865_t *max31865, uint8_t numwires, uint8_t filterHz);
/**
 * @brief Read the RTD temperature in Celsius.
 * @param max31865 Device context.
 * @param readTemp Temperature output.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_readTempC(Max31865_t *max31865, float *readTemp);
/**
 * @brief Read the RTD temperature in Fahrenheit.
 * @param max31865 Device context.
 * @param readTemp Temperature output.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_readTempF(Max31865_t *max31865, float *readTemp);
/**
 * @brief Apply the MAX31865 temperature filter.
 * @param newInput New sample.
 * @param lastOutput Previous filtered value.
 * @param effectiveFactor Filter factor.
 * @return Filtered value.
 */
float Max31865_Filter(float newInput, float lastOutput, float effectiveFactor);
/**
 * @brief Run one MAX31865 debug transaction.
 * @param dev Device context.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_debug_once(Max31865_t *dev);
/**
 * @brief Read multiple bytes from a MAX31865 register.
 * @param max31865 Device context.
 * @param addr Register address.
 * @param buffer Destination buffer.
 * @param n Number of bytes to read.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_readRegisterN(Max31865_t *max31865, uint8_t addr, uint8_t *buffer, uint8_t n);
/**
 * @brief Read one 8-bit MAX31865 register.
 * @param max31865 Device context.
 * @param addr Register address.
 * @param value Register value output.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_readRegister8(Max31865_t *max31865, uint8_t addr, uint8_t *value);
/**
 * @brief Read one 16-bit MAX31865 register value.
 * @param max31865 Device context.
 * @param addr Register address.
 * @param value Register value output.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_readRegister16(Max31865_t *max31865, uint8_t addr, uint16_t *value);
/**
 * @brief Write one 8-bit MAX31865 register.
 * @param dev Device context.
 * @param addr Register address.
 * @param data Register value.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_writeRegister8(Max31865_t *dev, uint8_t addr, uint8_t data);
/**
 * @brief Read the MAX31865 fault register.
 * @param max31865 Device context.
 * @param out_fault Fault register output.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_readFault(Max31865_t *max31865, uint8_t *out_fault);
/**
 * @brief Clear the current MAX31865 fault state.
 * @param max31865 Device context.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_clearFault(Max31865_t *max31865);
/**
 * @brief Enable or disable the MAX31865 bias.
 * @param max31865 Device context.
 * @param enable Bias state.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_enableBias(Max31865_t *max31865, uint8_t enable);
/**
 * @brief Enable or disable automatic conversion.
 * @param max31865 Device context.
 * @param enable Automatic conversion state.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_autoConvert(Max31865_t *max31865, uint8_t enable);
/**
 * @brief Configure the RTD wiring mode.
 * @param max31865 Device context.
 * @param numWires Number of RTD wires.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_setWires(Max31865_t *max31865, uint8_t numWires);
/**
 * @brief Configure the 50/60 Hz rejection filter.
 * @param max31865 Device context.
 * @param filterHz Filter selection.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_setFilter(Max31865_t *max31865, uint8_t filterHz);
/**
 * @brief Read the raw RTD conversion value.
 * @param max31865 Device context.
 * @param rtd_value RTD value output.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_readRTD(Max31865_t *max31865, uint16_t *rtd_value);
/**
 * @brief Run one application-level MAX31865 runtime cycle.
 * @return Application runtime status.
 */
app_max31865_runtime_status_t app_max31865_runtime_process(void);
/**
 * @brief Run the MAX31865 monitor loop at the requested interval.
 * @param max31865 Device context.
 * @param interval_ms Sampling interval in milliseconds.
 * @return MAX31865 status.
 */
max31865_status_t Max31865_monitorLoop(Max31865_t *max31865, uint32_t interval_ms);
#ifdef __cplusplus
}
#endif
#endif /* __MAX31865_H__ */
