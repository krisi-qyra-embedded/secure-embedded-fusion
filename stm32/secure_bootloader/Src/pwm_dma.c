/* DMA synchronization helpers. */
/**
 * @file pwm_dma.c
 * @brief TIM1 input-capture jitter collection through DMA2 Stream6.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include "pwm_dma.h"
#include <string.h>
/* Capture state shared between the collection loop and DMA interrupt. */
volatile uint32_t capture_buffer[JITTER_CAPTURE_SIZE];
volatile uint32_t jitter_differences[JITTER_CAPTURE_SIZE];
volatile uint8_t  g_jitter_capture_complete = 0;
size_t idx_jitter = 0;
/* Order peripheral register writes before continuing. */
static inline void jitter_barrier(void)
{
    /* Check the return status before continuing to the next step. */
    __DSB();
    __ISB();
}
/* Disable DMA2 Stream6 and wait for hardware acknowledgement. */
static jitter_status_t jitter_wait_dma_disable(void)
{
    /* Keep the wait bounded to avoid a stalled execution path. */
    uint32_t timeout = JITTER_DMA_DISABLE_TIMEOUT;
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    while ((DMA2_Stream6->CR & DMA_SxCR_EN) && timeout--)
    {
        __NOP();
    }
    return (timeout == 0U) ? JITTER_ERR_DMA_TIMEOUT : JITTER_OK;
}
/* Enable DMA2 Stream6 and confirm that the stream started. */
static jitter_status_t jitter_wait_dma_enable(void)
{
    /* Keep the wait bounded to avoid a stalled execution path. */
    uint32_t timeout = JITTER_DMA_ENABLE_TIMEOUT;
    DMA2_Stream6->CR |= DMA_SxCR_EN;
    while (!(DMA2_Stream6->CR & DMA_SxCR_EN) && timeout--)
    {
        __NOP();
    }
    return (timeout == 0U) ? JITTER_ERR_DMA_ENABLE : JITTER_OK;
}
/* Clear all pending DMA2 Stream6 status flags. */
static void jitter_clear_dma_flags(void)
{
    /* Check the return status before continuing to the next step. */
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 |
                  DMA_HIFCR_CHTIF6 |
                  DMA_HIFCR_CTEIF6 |
                  DMA_HIFCR_CDMEIF6 |
                  DMA_HIFCR_CFEIF6;
}
/* TIM1/DMA jitter-capture hardware setup. */
/**
 * @brief Configure TIM1 channel 3 and DMA2 Stream6 for jitter capture.
 * @return Jitter initialization status.
 */
jitter_status_t jitter_init(void)
{
    /* Configure TIM1 from a known stopped state. */
    TIM1->CR1 &= ~TIM_CR1_CEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    jitter_barrier();
    if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN)) return JITTER_ERR_GPIO_CLK;
    if (!(RCC->APB2ENR & RCC_APB2ENR_TIM1EN))  return JITTER_ERR_TIM_CLK;
    if (!(RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN))  return JITTER_ERR_DMA_CLK;
    GPIOA->MODER   &= ~(3U << (10 * 2));
    GPIOA->MODER   |=  (2U << (10 * 2));
    GPIOA->AFR[1]  &= ~(0xFU << ((10 - 8) * 4));
    GPIOA->AFR[1]  |=  (0x1U << ((10 - 8) * 4));
    GPIOA->OSPEEDR |=  (3U << (10 * 2));
    GPIOA->PUPDR   &= ~(3U << (10 * 2));
    jitter_barrier();
    TIM1->PSC = 0U;
    TIM1->ARR = 0xFFFFU;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CCMR2 &= ~((3U << 0) | (3U << 2) | (0xFU << 4));
    TIM1->CCMR2 |=  (1U << 0);
    TIM1->CCER  &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
    TIM1->CCER  |=  TIM_CCER_CC3E;
    /* Each CC3 event generates the DMA request used for timestamp capture. */
    TIM1->DIER |= TIM_DIER_CC3DE;
    TIM1->DIER |= TIM_DIER_UIE;
    jitter_wait_dma_disable();
    jitter_clear_dma_flags();
    uint32_t cr = 0;
    cr |= (6U << DMA_SxCR_CHSEL_Pos);
    cr |= DMA_SxCR_MINC;
    cr |= (2U << DMA_SxCR_PSIZE_Pos);
    cr |= (2U << DMA_SxCR_MSIZE_Pos);
    cr |= (2U << DMA_SxCR_PL_Pos);
    cr |= DMA_SxCR_TCIE;
    DMA2_Stream6->CR   = cr;
    DMA2_Stream6->PAR  = (uint32_t)&TIM1->CCR3;
    /* Stream6 reads TIM1_CCR3 into the capture buffer. */
    DMA2_Stream6->M0AR = (uint32_t)capture_buffer;
    DMA2_Stream6->NDTR = 0U;
    NVIC_SetPriority(DMA2_Stream6_IRQn, 2);
    NVIC_ClearPendingIRQ(DMA2_Stream6_IRQn);
    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 3);
    NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->EGR  = TIM_EGR_UG;
    return JITTER_OK;
}
/* Single-sample DMA arming and capture control. */
/**
 * @brief Arm DMA2 Stream6 for one TIM1 capture sample.
 * @param index Destination sample index in the capture buffer.
 * @return Jitter DMA setup status.
 */
jitter_status_t jitter_prepare_dma_sample(uint32_t index)
{
    if (index >= JITTER_CAPTURE_SIZE)
    {
        return JITTER_ERR_INVALID_PARAM;
    }
    jitter_status_t st;
    st = jitter_wait_dma_disable();
    if (st != JITTER_OK) return st;
    jitter_clear_dma_flags();
    DMA2_Stream6->M0AR = (uint32_t)&capture_buffer[index];
    /* Arm exactly one capture transfer for this sample. */
    DMA2_Stream6->NDTR = 1U;
    jitter_barrier();
    st = jitter_wait_dma_enable();
    return st;
}
/**
 * @brief Trigger a TIM1 channel 3 capture event in software.
 */
void jitter_trigger_capture_event(void)
{
    /* Check the return status before continuing to the next step. */
    TIM1->EGR |= TIM_EGR_CC3G;
    jitter_barrier();
}
/* High-level jitter collection and post-processing. */
/**
 * @brief Collect the configured number of TIM1 capture samples through DMA.
 * @return Jitter collection status.
 */
jitter_status_t jitter_collect_samples(void)
{
    uint32_t i;
    uint32_t timeout;
    idx_jitter = 0;
    memset((void*)jitter_differences, 0, sizeof(jitter_differences));
    for (i = 0; i < JITTER_CAPTURE_SIZE; i++)
    {
        g_jitter_capture_complete = 0;
        jitter_status_t st = jitter_prepare_dma_sample(i);
        if (st != JITTER_OK) return st;
        /* Vary the software delay before each capture to perturb sampling time. */
        uint32_t seed = TIM1->CNT ^ (i * 0x45D9F3BU);
        seed ^= (seed << 13);
        seed ^= (seed >> 17);
        seed ^= (seed << 5);
        uint32_t delay = (seed & 0xFFU) + 1U;
        for (volatile uint32_t d = 0; d < delay * 50U; d++) { __NOP(); }
        jitter_trigger_capture_event();
        timeout = JITTER_SAMPLE_TIMEOUT;
        /* Wait for the DMA transfer-complete interrupt with a bounded timeout. */
        while (!g_jitter_capture_complete && timeout--)
        {
            __NOP();
        }
        if (timeout == 0U)
        {
            jitter_wait_dma_disable();
            return JITTER_ERR_DMA_TRANSFER;
        }
        jitter_wait_dma_disable();
    }
    return jitter_compute_differences();
}
/**
 * @brief Convert successive TIM1 captures into wrap-safe timing differences.
 * @return Jitter post-processing status.
 */
jitter_status_t jitter_compute_differences(void)
{
    if (JITTER_CAPTURE_SIZE < 2) return JITTER_ERR_NO_SAMPLES;
    for (uint32_t i = 0; i < (JITTER_CAPTURE_SIZE - 1); i++)
    {
        uint32_t t1 = capture_buffer[i]     & 0xFFFFU;
        uint32_t t2 = capture_buffer[i + 1] & 0xFFFFU;
        uint32_t diff;
        /* Calculate the delta while accounting for the 16-bit timer wrap. */
        if (t2 >= t1) diff = t2 - t1;
        else diff = (0xFFFFU - t1) + t2 + 1U;
        jitter_differences[i] = diff;
    }
    jitter_differences[JITTER_CAPTURE_SIZE - 1] = 0;
    idx_jitter = JITTER_CAPTURE_SIZE - 1;
    return JITTER_OK;
}
/* DMA recovery and entropy diagnostics. */
/**
 * @brief Return DMA2 Stream6 to an idle capture-ready state.
 * @return Jitter DMA status.
 */
jitter_status_t jitter_rearm_dma(void)
{
    /* Check the return status before continuing to the next step. */
    jitter_status_t st = jitter_wait_dma_disable();
    if (st != JITTER_OK) return st;
    jitter_clear_dma_flags();
    DMA2_Stream6->M0AR = (uint32_t)capture_buffer;
    DMA2_Stream6->NDTR = 0;
    jitter_barrier();
    return JITTER_OK;
}
/**
 * @brief Calculate a lightweight heuristic score from the captured timing differences.
 * @return Heuristic jitter score from 0 to 100.
 */
uint32_t jitter_assess_entropy(void)
{
    uint32_t zero = 0, one = 0, small = 0, large = 0;
    for (uint32_t i = 0; i < JITTER_CAPTURE_SIZE - 1; i++)
    {
        uint32_t d = jitter_differences[i];
        if (d == 0) continue;
        if (d & 1U) one++; else zero++;
        if (d < 10U) small++;
        if (d > 1000U) large++;
    }
    uint32_t samples = zero + one;
    if (samples == 0) return 0;
    int32_t diff = (int32_t)zero - (int32_t)one;
    if (diff < 0) diff = -diff;
    /* Use bit balance as the base heuristic, then penalize extreme distributions. */
    uint32_t score = 100U - (100U * (uint32_t)diff) / samples;
    if (small > (samples / 2U)) score = (score > 20U) ? score - 20U : 0U;
    if (large > (samples / 10U)) score = (score > 10U) ? score - 10U : 0U;
    return score;
}
/**
 * @brief Return the configured jitter capture count.
 * @return Number of capture samples.
 */
uint32_t jitter_get_capture_size(void)
{
    /* Check the return status before continuing to the next step. */
    return JITTER_CAPTURE_SIZE;
}
/**
 * @brief Initialize the jitter hardware and collect one complete sample set.
 * @return Jitter capture status.
 */
jitter_status_t JITTER_CAPTURE_INIT(void)
{
    /* Initialize the required state before enabling the peripheral. */
    jitter_status_t st;
    st = jitter_init();
    if (st != JITTER_OK)
    {
        return st;
    }
    st = jitter_collect_samples();
    if (st != JITTER_OK)
    {
        return st;
    }
    return JITTER_OK;
}
/* Interrupt handlers. */
/**
 * @brief Handle TIM1 capture DMA completion and DMA error flags.
 */
void DMA2_Stream6_IRQHandler(void)
{
    /* Clear the handled interrupt flags and update the transfer state. */
    uint32_t hisr = DMA2->HISR;
    if (hisr & DMA_HISR_TCIF6)
    {
        DMA2->HIFCR = DMA_HIFCR_CTCIF6;
        g_jitter_capture_complete = 1;
    }
    if (hisr & DMA_HISR_TEIF6) DMA2->HIFCR = DMA_HIFCR_CTEIF6;
    if (hisr & DMA_HISR_DMEIF6) DMA2->HIFCR = DMA_HIFCR_CDMEIF6;
    if (hisr & DMA_HISR_FEIF6)  DMA2->HIFCR = DMA_HIFCR_CFEIF6;
}
/**
 * @brief Clear the TIM1 update interrupt flag.
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* Clear the handled interrupt flags and update the transfer state. */
    if (TIM1->SR & TIM_SR_UIF)
    {
        TIM1->SR &= ~TIM_SR_UIF;
    }
}
