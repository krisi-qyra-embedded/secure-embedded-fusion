/* Author: i fiksuari pas prog | Documentation date: 8 May 2026 */
/*
 * DMA.h
 *
 *  Created on: Aug 13, 2024
 *      Author: krisi qyra
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include"stm32f4xx.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

//#define  LISR_TCIF0 (1U << 5)
//#define  LIFCR_CTCIF0 (1U << 5)
//#define  LISR_TEIF0 (1U << 3)
//#define  LIFCR_CTEIF0 (1U << 3)
//#define GPIOAEN      (1U << 0)
//#define ADC1EN        (1U << 8)
/* Global flag set by DMA IRQ */
/* Buffer size in bytes requested by user */
#define BUF_SZ 255

/* ADC DMA init return macros */
#define ADC_DMA_INIT_SUCCESS               0
#define ADC_DMA_INIT_COLLECTION_FAILED   -301
#define ADC_DMA_INIT_RETRY_COUNT            2

/* Three byte buffers (one per ADC channel) */
extern uint8_t buf_pc4[BUF_SZ];
extern uint8_t buf_pc5[BUF_SZ];
extern uint8_t buf_pb0[BUF_SZ];

 /* Write indices */
extern size_t idx_pc4;
extern size_t idx_pc5;
extern size_t idx_pb0;

/* ADC buffer: sample_count sets of 3 values (PC4, PC5, PB0) */
extern uint8_t SAMPLE_COUNT;  /* adjust as needed */
extern uint32_t last_idx;
extern volatile uint8_t g_transfer_complete;
extern bool all_full;
extern volatile uint16_t adc_buffer[30];
extern uint16_t pc4_val,pc5_val,pb0_val;
/* Forward declarations */
void dma2_ADC_pc45(uint16_t *mem_buf, uint32_t sample_count);
int ADC_DMA_INIT(void);

#endif /* INC_DMA_H_ */
