#ifndef __MAX31865_H__
#define __MAX31865_H__

#include "spi_dma.h"


#define _MAX31865_USE_FREERTOS  1
#define _MAX31865_RREF      430.0f
#define _MAX31865_RNOMINAL  100.0f

#define MAX31856_CONFIG_REG             0x00
#define MAX31856_CONFIG_BIAS            0x80
#define MAX31856_CONFIG_MODEAUTO        0x40
#define MAX31856_CONFIG_MODEOFF         0x00
#define MAX31856_CONFIG_1SHOT           0x20
#define MAX31856_CONFIG_3WIRE           0x10
#define MAX31856_CONFIG_24WIRE          0x00
#define MAX31856_CONFIG_FAULTSTAT       0x02
#define MAX31856_CONFIG_FILT50HZ        0x01
#define MAX31856_CONFIG_FILT60HZ        0x00

#define MAX31856_RTDMSB_REG             0x01
#define MAX31856_RTDLSB_REG             0x02
#define MAX31856_HFAULTMSB_REG          0x03
#define MAX31856_HFAULTLSB_REG          0x04
#define MAX31856_LFAULTMSB_REG          0x05
#define MAX31856_LFAULTLSB_REG          0x06
#define MAX31856_FAULTSTAT_REG          0x07

/* Error codes */
#define MAX31865_OK           0
#define MAX31865_ERR_ARG     -1
#define MAX31865_ERR_SIZE    -2
#define MAX31865_ERR_HAL     -3
#define MAX31865_ERR_TIMEOUT -4
#define MAX31865_ERR_FAULT   -5
#define MAX31865_ERR_DMA   -6

#define RREF_MILLI_OHMS 430000UL   

/* Choose CS pin and port for the MAX31865 - change if needed */
#define MAX31865_CS_PORT    GPIOB
#define MAX31865_CS_PIN     GPIO_PIN_6

#define MAX31865_FAULT_HIGHTHRESH       0x80
#define MAX31865_FAULT_LOWTHRESH        0x40
#define MAX31865_FAULT_REFINLOW         0x20
#define MAX31865_FAULT_REFINHIGH        0x10
#define MAX31865_FAULT_RTDINLOW         0x08
#define MAX31865_FAULT_OVUV             0x04

#define CS_SELECT         0
#define CS_DESELECT       1


#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

#ifndef TRANSFER_TIMEOUT_MS
#define TRANSFER_TIMEOUT_MS  100u
#endif

#define MAX_STACK_TOTAL 64  /* increase if you need to read larger blocks */

extern volatile uint32_t hb_main_ts;


 
typedef struct
{
  uint8_t           lock;

}Max31865_t;
 

extern Max31865_t max31865_dev;

int Max31865_init(Max31865_t *max31865,uint8_t numwires, uint8_t filterHz);
int Max31865_readTempC(Max31865_t *max31865, float *readTemp);
int Max31865_readTempF(Max31865_t *max31865, float *readTemp);
float Max31865_Filter(float newInput, float lastOutput, float effectiveFactor);
int MPU_CS(uint8_t state);
int Max31865_debug_once(Max31865_t *dev);

#endif
