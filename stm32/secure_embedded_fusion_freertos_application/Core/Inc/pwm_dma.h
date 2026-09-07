/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef __PWM_DMA_H__
#define __PWM_DMA_H__
#include "stm32f4xx.h"
#include <stdint.h>
#include <stddef.h>
#define JITTER_CAPTURE_SIZE            256U
#define JITTER_DMA_ENABLE_TIMEOUT      100000U
#define JITTER_DMA_DISABLE_TIMEOUT     100000U
#define JITTER_SAMPLE_TIMEOUT          200000U
#define JITTER_INIT_RETRY_COUNT        2U
typedef enum
{
   JITTER_OK = 0,
   JITTER_ERR_GPIO_CLK        = -10,
   JITTER_ERR_TIM_CLK         = -11,
   JITTER_ERR_DMA_CLK         = -12,
   JITTER_ERR_DMA_TIMEOUT     = -20,
   JITTER_ERR_DMA_ENABLE      = -21,
   JITTER_ERR_DMA_TRANSFER    = -22,
   JITTER_ERR_INVALID_PARAM   = -30,
   JITTER_ERR_NO_SAMPLES      = -31,
   JITTER_ERR_INIT_FAILED     = -40,
   JITTER_ERR_COLLECTION     = -41
} jitter_status_t;
extern volatile uint32_t capture_buffer[JITTER_CAPTURE_SIZE];
extern volatile uint32_t jitter_differences[JITTER_CAPTURE_SIZE];
extern volatile uint8_t  g_jitter_capture_complete;
extern size_t idx_jitter;
jitter_status_t jitter_init(void);
/**
 * @brief Arm DMA for one capture sample.
 * @param index Destination capture-buffer index.
 * @return Jitter status.
 */
jitter_status_t jitter_prepare_dma_sample(uint32_t index);
jitter_status_t jitter_collect_samples(void);
jitter_status_t jitter_compute_differences(void);
jitter_status_t jitter_rearm_dma(void);
uint32_t jitter_assess_entropy(void);
uint32_t jitter_get_capture_size(void);
void jitter_trigger_capture_event(void);
jitter_status_t JITTER_CAPTURE_INIT(void);
#endif /* __PWM_DMA_H__ */
