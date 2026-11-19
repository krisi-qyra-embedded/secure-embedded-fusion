/**
 * @file pwm_dma.c
 * @brief TIM1 input-capture jitter collection using DMA.
 */
#include "pwm_dma.h"
#include <string.h>
volatile uint32_t capture_buffer[JITTER_CAPTURE_SIZE];
volatile uint32_t jitter_differences[JITTER_CAPTURE_SIZE];
volatile uint8_t  g_jitter_capture_complete = 0;
size_t idx_jitter = 0;
/* Flush peripheral register writes before continuing. */
static inline void jitter_barrier(void)
{
    __DSB();
    __ISB();
}
/* Disable DMA2 Stream6 and wait for hardware confirmation. */
static jitter_status_t jitter_wait_dma_disable(void)
{
    uint32_t timeout = JITTER_DMA_DISABLE_TIMEOUT;
    /* Stream must be fully disabled before reconfiguration. */
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    while ((DMA2_Stream6->CR & DMA_SxCR_EN) && timeout--)
    {
        __NOP();
    }
    return (timeout == 0U) ? JITTER_ERR_DMA_TIMEOUT : JITTER_OK;
}
/* Enable DMA2 Stream6 and wait for hardware confirmation. */
static jitter_status_t jitter_wait_dma_enable(void)
{
    uint32_t timeout = JITTER_DMA_ENABLE_TIMEOUT;
    /* Re-enable only after the stream registers are ready. */
    DMA2_Stream6->CR |= DMA_SxCR_EN;
    while (!(DMA2_Stream6->CR & DMA_SxCR_EN) && timeout--)
    {
        __NOP();
    }
    return (timeout == 0U) ? JITTER_ERR_DMA_ENABLE : JITTER_OK;
}
/* Clear all DMA2 Stream6 status flags. */
static void jitter_clear_dma_flags(void)
{
    /* Clear stale completion/error flags before a new capture. */
    DMA2->HIFCR = DMA_HIFCR_CTCIF6 |
                  DMA_HIFCR_CHTIF6 |
                  DMA_HIFCR_CTEIF6 |
                  DMA_HIFCR_CDMEIF6 |
                  DMA_HIFCR_CFEIF6;
}
/**
 * @brief Initialize TIM1 CH3 and DMA2 Stream6 for jitter capture.
 * @return Jitter status.
 */
jitter_status_t jitter_init(void)
{
    /* Stop TIM1 while the capture path is configured. */
    TIM1->CR1 &= ~TIM_CR1_CEN;
    /* Enable GPIOA, TIM1 and DMA2 clocks. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    jitter_barrier();
    if (!(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN)) return JITTER_ERR_GPIO_CLK;
    if (!(RCC->APB2ENR & RCC_APB2ENR_TIM1EN))  return JITTER_ERR_TIM_CLK;
    if (!(RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN))  return JITTER_ERR_DMA_CLK;
    /* PA10 is routed to TIM1_CH3 input capture. */
    GPIOA->MODER   &= ~(3U << (10 * 2));
    GPIOA->MODER   |=  (2U << (10 * 2));
    GPIOA->AFR[1]  &= ~(0xFU << ((10 - 8) * 4));
    GPIOA->AFR[1]  |=  (0x1U << ((10 - 8) * 4));
    GPIOA->OSPEEDR |=  (3U << (10 * 2));
    GPIOA->PUPDR   &= ~(3U << (10 * 2));
    jitter_barrier();
    /* Free-running 16-bit timer keeps the raw timing resolution. */
    TIM1->PSC = 0U;
    TIM1->ARR = 0xFFFFU;
    TIM1->CNT = 0U;
    TIM1->EGR = TIM_EGR_UG;
    /* Map TI3 onto capture channel 3. */
    TIM1->CCMR2 &= ~((3U << 0) | (3U << 2) | (0xFU << 4));
    TIM1->CCMR2 |=  (1U << 0);
    TIM1->CCER  &= ~(TIM_CCER_CC3P | TIM_CCER_CC3NP);
    TIM1->CCER  |=  TIM_CCER_CC3E;
    /* Each CC3 event generates a DMA request. */
    TIM1->DIER |= TIM_DIER_CC3DE;
    TIM1->DIER |= TIM_DIER_UIE;
    jitter_wait_dma_disable();
    jitter_clear_dma_flags();
    /* DMA2 Stream6 reads CCR3 into the capture buffer. */
    uint32_t cr = 0;
    cr |= (6U << DMA_SxCR_CHSEL_Pos);
    cr |= DMA_SxCR_MINC;
    cr |= (2U << DMA_SxCR_PSIZE_Pos);
    cr |= (2U << DMA_SxCR_MSIZE_Pos);
    cr |= (2U << DMA_SxCR_PL_Pos);
    cr |= DMA_SxCR_TCIE;
    DMA2_Stream6->CR   = cr;
    DMA2_Stream6->PAR  = (uint32_t)&TIM1->CCR3;
    /* Return the stream to the base buffer with no active transfer. */
    DMA2_Stream6->M0AR = (uint32_t)capture_buffer;
    DMA2_Stream6->NDTR = 0U;
    /* DMA completion IRQ runs above the TIM1 update IRQ. */
    NVIC_SetPriority(DMA2_Stream6_IRQn, 2);
    NVIC_ClearPendingIRQ(DMA2_Stream6_IRQn);
    NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 3);
    NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    /* Start the free-running capture timer after setup. */
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM1->EGR  = TIM_EGR_UG;
    return JITTER_OK;
}
/**
 * @brief Arm DMA for one jitter capture sample.
 * @param index Capture-buffer index.
 * @return Jitter status.
 */
jitter_status_t jitter_prepare_dma_sample(uint32_t index)
{
    if (index >= JITTER_CAPTURE_SIZE)
    {
        return JITTER_ERR_INVALID_PARAM;
    }
    jitter_status_t st;
    /* Re-arm each sample from a known disabled DMA state. */
    st = jitter_wait_dma_disable();
    if (st != JITTER_OK) return st;
    jitter_clear_dma_flags();
    DMA2_Stream6->M0AR = (uint32_t)&capture_buffer[index];
    DMA2_Stream6->NDTR = 1U;
    jitter_barrier();
    st = jitter_wait_dma_enable();
    return st;
}
/* Trigger one TIM1 CH3 capture event in software. */
void jitter_trigger_capture_event(void)
{
    /* Software CC3 event feeds the same DMA request path as hardware capture. */
    TIM1->EGR |= TIM_EGR_CC3G;
    jitter_barrier();
}
/**
 * @brief Collect the full jitter sample set.
 * @return Jitter status.
 */
jitter_status_t jitter_collect_samples(void)
{
    uint32_t i;
    uint32_t timeout;
    /* Start a fresh capture batch. */
    idx_jitter = 0;
    memset((void*)jitter_differences, 0, sizeof(jitter_differences));
    for (i = 0; i < JITTER_CAPTURE_SIZE; i++)
    {
        g_jitter_capture_complete = 0;
        jitter_status_t st = jitter_prepare_dma_sample(i);
        if (st != JITTER_OK) return st;
        /* Vary the software delay from the live timer value and sample index. */
        uint32_t seed = TIM1->CNT ^ (i * 0x45D9F3BU);
        seed ^= (seed << 13);
        seed ^= (seed >> 17);
        seed ^= (seed << 5);
        uint32_t delay = (seed & 0xFFU) + 1U;
        for (volatile uint32_t d = 0; d < delay * 50U; d++) { __NOP(); }
        /* Capture TIM1 after the variable delay. */
        jitter_trigger_capture_event();
        /* Wait for DMA transfer-complete from this sample only. */
        timeout = JITTER_SAMPLE_TIMEOUT;
        while (!g_jitter_capture_complete && timeout--)
        {
            __NOP();
        }
        if (timeout == 0U)
        {
            jitter_wait_dma_disable();
            return JITTER_ERR_DMA_TRANSFER;
        }
        /* Leave the stream disabled before the next sample is armed. */
        jitter_wait_dma_disable();
    }
    return jitter_compute_differences();
}
/**
 * @brief Convert timer captures into adjacent timing differences.
 * @return Jitter status.
 */
jitter_status_t jitter_compute_differences(void)
{
    if (JITTER_CAPTURE_SIZE < 2) return JITTER_ERR_NO_SAMPLES;
    for (uint32_t i = 0; i < (JITTER_CAPTURE_SIZE - 1); i++)
    {
        /* Use adjacent captures so timer wrap can be handled explicitly. */
        uint32_t t1 = capture_buffer[i]     & 0xFFFFU;
        uint32_t t2 = capture_buffer[i + 1] & 0xFFFFU;
        uint32_t diff;
        /* Account for the 16-bit timer wrapping between captures. */
        if (t2 >= t1) diff = t2 - t1;
        else diff = (0xFFFFU - t1) + t2 + 1U;
        jitter_differences[i] = diff;
    }
    jitter_differences[JITTER_CAPTURE_SIZE - 1] = 0;
    idx_jitter = JITTER_CAPTURE_SIZE - 1;
    return JITTER_OK;
}
/**
 * @brief Return DMA2 Stream6 to an idle capture state.
 * @return Jitter status.
 */
jitter_status_t jitter_rearm_dma(void)
{
    jitter_status_t st = jitter_wait_dma_disable();
    if (st != JITTER_OK) return st;
    jitter_clear_dma_flags();
    DMA2_Stream6->M0AR = (uint32_t)capture_buffer;
    DMA2_Stream6->NDTR = 0;
    jitter_barrier();
    return JITTER_OK;
}
/**
 * @brief Calculate a simple variability score from jitter differences.
 * @return Heuristic score from 0 to 100.
 */
uint32_t jitter_assess_entropy(void)
{
    uint32_t zero = 0, one = 0, small = 0, large = 0;
    for (uint32_t i = 0; i < JITTER_CAPTURE_SIZE - 1; i++)
    {
        uint32_t d = jitter_differences[i];
        /* Ignore empty differences in the quick variability check. */
        if (d == 0) continue;
        if (d & 1U) one++; else zero++;
        if (d < 10U) small++;
        if (d > 1000U) large++;
    }
    uint32_t samples = zero + one;
    if (samples == 0) return 0;
    int32_t diff = (int32_t)zero - (int32_t)one;
    if (diff < 0) diff = -diff;
    /* Start from bit-balance, then penalize suspicious distributions. */
    uint32_t score = 100U - (100U * (uint32_t)diff) / samples;
    if (small > (samples / 2U)) score = (score > 20U) ? score - 20U : 0U;
    if (large > (samples / 10U)) score = (score > 10U) ? score - 10U : 0U;
    return score;
}
/**
 * @brief Get configured jitter capture count.
 * @return Number of capture entries.
 */
uint32_t jitter_get_capture_size(void)
{
    return JITTER_CAPTURE_SIZE;
}
/**
 * @brief Initialize jitter hardware and collect one sample set.
 * @return Jitter status.
 */
jitter_status_t JITTER_CAPTURE_INIT(void)
{
    jitter_status_t st;
    /* Hardware setup comes before collection. */
    st = jitter_init();
    if (st != JITTER_OK)
    {
        return st;
    }
    /* Upper layer gets one status for the complete capture stage. */
    st = jitter_collect_samples();
    if (st != JITTER_OK)
    {
        return st;
    }
    return JITTER_OK;
}
/* TIM1 CH3 capture DMA interrupt. */
void DMA2_Stream6_IRQHandler(void)
{
    uint32_t hisr = DMA2->HISR;
    /* Transfer complete releases the collection loop. */
    if (hisr & DMA_HISR_TCIF6)
    {
        DMA2->HIFCR = DMA_HIFCR_CTCIF6;
        g_jitter_capture_complete = 1;
    }
    if (hisr & DMA_HISR_TEIF6) DMA2->HIFCR = DMA_HIFCR_CTEIF6;
    if (hisr & DMA_HISR_DMEIF6) DMA2->HIFCR = DMA_HIFCR_CDMEIF6;
    if (hisr & DMA_HISR_FEIF6)  DMA2->HIFCR = DMA_HIFCR_CFEIF6;
}
/* Clear TIM1 update interrupt state. */
void TIM1_UP_TIM10_IRQHandler(void)
{
    /* TIM1 overflow is expected from the free-running counter. */
    if (TIM1->SR & TIM_SR_UIF)
    {
        TIM1->SR &= ~TIM_SR_UIF;
    }
}
