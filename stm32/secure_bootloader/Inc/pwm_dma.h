/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */
#ifndef __PWM_DMA_H__
#define __PWM_DMA_H__

#include "stm32f4xx.h"
#include <stdint.h>
#include <stddef.h>

/*
 * ================================================================
 *  JITTER CAPTURE MODULE – ARCHITECTURE OVERVIEW
 * ================================================================
 *
 *  Purpose:
 *  --------
 *  This module measures fine-grain timing jitter using:
 *      TIM1 Channel 3 Input Capture
 *      DMA2 Stream6 Channel6 single-transfer mode
 *
 *  Each capture is:
 *      - Armed explicitly
 *      - Triggered by software (CC3G)
 *      - Transferred by DMA (1 sample)
 *      - Acknowledged via DMA TC interrupt
 *
 *  Design goals:
 *      - Deterministic behavior (no hidden state)
 *      - No blocking without timeout
 *      - Explicit error reporting
 *      - Safe re-arming between samples
 *      - Maintainable separation of responsibilities
 *
 *  Architecture:
 *      jitter_init()            -> one-time peripheral setup
 *      jitter_prepare_dma()     -> arms DMA for exactly one sample
 *      jitter_trigger_capture() -> triggers capture event
 *      jitter_collect_samples() -> loops, waits, validates, stores
 *      jitter_compute_diff()    -> post-process
 *
 * ================================================================
 */


/* ===================== Configuration ===================== */

/* Number of samples to collect.
 * Must be power-of-two friendly if used later for entropy conditioning.
 */
#define JITTER_CAPTURE_SIZE            256U

/* Safety timeouts.
 * These prevent permanent lockups if hardware misbehaves.
 */
#define JITTER_DMA_ENABLE_TIMEOUT      100000U
#define JITTER_DMA_DISABLE_TIMEOUT     100000U
#define JITTER_SAMPLE_TIMEOUT          200000U
#define JITTER_INIT_RETRY_COUNT        2U


/* ===================== Error / Status Codes ===================== */

/*
 * Error space is intentionally segmented:
 *  -10..-19  : clock / peripheral errors
 *  -20..-29  : DMA related errors
 *  -30..-39  : logic / parameter errors
 *  -40..-49  : high-level flow errors
 *
 * This makes debugging and log filtering easier.
 */
typedef enum
{
    JITTER_OK = 0,

    /* Clock / peripheral failures */
    JITTER_ERR_GPIO_CLK        = -10,
    JITTER_ERR_TIM_CLK         = -11,
    JITTER_ERR_DMA_CLK         = -12,

    /* DMA failures */
    JITTER_ERR_DMA_TIMEOUT     = -20,
    JITTER_ERR_DMA_ENABLE      = -21,
    JITTER_ERR_DMA_TRANSFER    = -22,

    /* Logic / usage errors */
    JITTER_ERR_INVALID_PARAM   = -30,
    JITTER_ERR_NO_SAMPLES      = -31,

    /* High-level flow errors */
    JITTER_ERR_INIT_FAILED     = -40,
    JITTER_ERR_COLLECTION     = -41

} jitter_status_t;


/* ===================== Public Buffers ===================== */

/*
 * capture_buffer:
 *  Raw timer values captured by DMA.
 *  Size is fixed and defined at compile time.
 */
extern volatile uint32_t capture_buffer[JITTER_CAPTURE_SIZE];

/*
 * jitter_differences:
 *  Post-processed delta between successive captures.
 */
extern volatile uint32_t jitter_differences[JITTER_CAPTURE_SIZE];

/*
 * g_jitter_capture_complete:
 *  Set ONLY in DMA TC interrupt.
 *  Polled by collection loop.
 */
extern volatile uint8_t  g_jitter_capture_complete;

/*
 * idx_jitter:
 *  Index of last valid jitter difference.
 *  Mainly diagnostic / debugging utility.
 */
extern size_t idx_jitter;


/* ===================== Public API ===================== */

/*
 * jitter_init()
 * ------------
 * Performs all peripheral configuration:
 *  - GPIO
 *  - TIM1
 *  - DMA2 Stream6
 *  - NVIC
 *
 * Must be called exactly once before any capture.
 */
jitter_status_t jitter_init(void);

/*
 * jitter_prepare_dma_sample()
 * ---------------------------
 * Arms DMA for exactly ONE transfer.
 * No circular mode, no continuous mode.
 * This ensures deterministic per-sample control.
 */
jitter_status_t jitter_prepare_dma_sample(uint32_t index);

/*
 * jitter_collect_samples()
 * ------------------------
 * High-level orchestrator:
 *  - arms DMA
 *  - injects timing noise
 *  - triggers capture
 *  - waits for IRQ
 *  - repeats
 *  - post-processes
 */
jitter_status_t jitter_collect_samples(void);

/*
 * jitter_compute_differences()
 * ----------------------------
 * Converts raw timer captures into delta values.
 */
jitter_status_t jitter_compute_differences(void);

/*
 * jitter_rearm_dma()
 * ------------------
 * Forces DMA into known idle state.
 * Used after failures or retries.
 */
jitter_status_t jitter_rearm_dma(void);

/*
 * jitter_assess_entropy()
 * -----------------------
 * Lightweight heuristic quality estimator.
 * Not cryptographic, but useful for diagnostics.
 */
uint32_t jitter_assess_entropy(void);

/*
 * jitter_get_capture_size()
 * ------------------------
 * Returns compile-time capture size.
 * Useful for external validation.
 */
uint32_t jitter_get_capture_size(void);

/*
 * jitter_trigger_capture_event()
 * ------------------------------
 * Software trigger of CC3 event.
 * Separated for clarity and testability.
 */
void jitter_trigger_capture_event(void);
jitter_status_t JITTER_CAPTURE_INIT(void);


#endif /* __PWM_DMA_H__ */
