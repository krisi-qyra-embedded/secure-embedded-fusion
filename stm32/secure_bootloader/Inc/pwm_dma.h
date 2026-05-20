/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file pwm_dma.h
 * @brief TIM1 input-capture jitter collection using DMA.
 */
#ifndef __PWM_DMA_H__
#define __PWM_DMA_H__
#include "stm32f4xx.h"
#include <stdint.h>
#include <stddef.h>
/* One-shot TIM1/DMA jitter capture configuration. */
#define JITTER_CAPTURE_SIZE            256U
#define JITTER_DMA_ENABLE_TIMEOUT      100000U
#define JITTER_DMA_DISABLE_TIMEOUT     100000U
#define JITTER_SAMPLE_TIMEOUT          200000U
#define JITTER_INIT_RETRY_COUNT        2U
/** @brief Jitter capture status. */
/* Error ranges separate peripheral, DMA, parameter and flow failures. */
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
/* Raw capture values and processed jitter differences. */
extern volatile uint32_t capture_buffer[JITTER_CAPTURE_SIZE];
extern volatile uint32_t jitter_differences[JITTER_CAPTURE_SIZE];
extern volatile uint8_t  g_jitter_capture_complete;
extern size_t idx_jitter;
/**
 * @brief Initialize TIM1 input capture, DMA2 Stream6 and its IRQ.
 * @return Jitter status.
 */
jitter_status_t jitter_init(void);
/**
 * @brief Arm DMA for one jitter capture.
 * @param index Capture-buffer index.
 * @return Jitter status.
 */
jitter_status_t jitter_prepare_dma_sample(uint32_t index);
/**
 * @brief Collect the complete jitter sample set.
 * @return Jitter status.
 */
jitter_status_t jitter_collect_samples(void);
/**
 * @brief Convert raw timer captures into timing deltas.
 * @return Jitter status.
 */
jitter_status_t jitter_compute_differences(void);
/**
 * @brief Return the DMA stream to a known idle state.
 * @return Jitter status.
 */
jitter_status_t jitter_rearm_dma(void);
/**
 * @brief Estimate basic diversity in the captured jitter values.
 * @return Diagnostic entropy score.
 */
uint32_t jitter_assess_entropy(void);
/**
 * @brief Get the configured jitter capture size.
 * @return Number of capture samples.
 */
uint32_t jitter_get_capture_size(void);
/**
 * @brief Trigger the TIM1 capture event in software.
 */
void jitter_trigger_capture_event(void);
/**
 * @brief Initialize and collect the project jitter entropy source.
 * @return Jitter status.
 */
jitter_status_t JITTER_CAPTURE_INIT(void);
#endif
