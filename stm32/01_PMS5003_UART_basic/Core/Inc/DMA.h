/* DMA.h | Author: i fiksuari pas prog | Date: 01 February 2026 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include"stm32f4xx.h"

#define  LISR_TCIF0 (1U << 5)
#define  LIFCR_CTCIF0 (1U << 5)
#define  LISR_TEIF0 (1U << 3)
#define  LIFCR_CTEIF0 (1U << 3)
#define GPIOAEN      (1U << 0)
#define ADC1EN        (1U << 8)



#endif /* INC_DMA_H_ */
