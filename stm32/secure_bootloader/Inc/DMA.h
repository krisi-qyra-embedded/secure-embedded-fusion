/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file DMA.h
 * @brief ADC DMA buffer, status and initialization declarations.
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
/* ADC entropy byte-buffer size. */
#define BUF_SZ 255
/* ADC DMA initialization status and retry count. */
#define ADC_DMA_INIT_SUCCESS               0
#define ADC_DMA_INIT_COLLECTION_FAILED   -301
#define ADC_DMA_INIT_RETRY_COUNT            2
/* One byte buffer for each ADC entropy channel. */
extern uint8_t buf_pc4[BUF_SZ];
extern uint8_t buf_pc5[BUF_SZ];
extern uint8_t buf_pb0[BUF_SZ];
/* Current write position for each channel buffer. */
extern size_t idx_pc4;
extern size_t idx_pc5;
extern size_t idx_pb0;
/* Shared ADC DMA sampling state. */
extern uint8_t SAMPLE_COUNT;
extern uint32_t last_idx;
extern volatile uint8_t g_transfer_complete;
extern bool all_full;
extern volatile uint16_t adc_buffer[30];
extern uint16_t pc4_val,pc5_val,pb0_val;
/**
 * @brief Configure DMA2 for ADC samples from PC4, PC5 and PB0.
 * @param mem_buf Destination ADC buffer.
 * @param sample_count Number of ADC sample groups.
 */
void dma2_ADC_pc45(uint16_t *mem_buf, uint32_t sample_count);
/**
 * @brief Initialize ADC DMA collection.
 * @return ADC DMA initialization status.
 */
int ADC_DMA_INIT(void);
#endif /* INC_DMA_H_ */
