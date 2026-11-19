#ifndef ADC_H
#define ADC_H

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
HAL_StatusTypeDef ADCI_Init(void);

#endif
