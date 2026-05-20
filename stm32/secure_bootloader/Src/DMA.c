/**
 * @file DMA.c
 * @brief ADC1 DMA sampling for the PC4, PC5 and PB0 entropy inputs.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include "DMA.h"
/* ADC1/DMA sampling state shared with the collection path. */
uint8_t SAMPLE_COUNT = 10;
volatile uint8_t g_transfer_complete = 0;
volatile uint16_t adc_buffer[30];
 uint32_t last_idx;
 uint16_t pc4_val,pc5_val,pb0_val;
 bool all_full;
  uint8_t buf_pc4[BUF_SZ];
  uint8_t buf_pc5[BUF_SZ];
  uint8_t buf_pb0[BUF_SZ];
  size_t idx_pc4;
  size_t idx_pc5;
  size_t idx_pb0;
/**
 * @brief Configure ADC1 and DMA2 Stream4 for continuous sampling of PC4, PC5 and PB0.
 * @param mem_buf DMA destination buffer.
 * @param sample_count Number of three-channel sample groups.
 */
void dma2_ADC_pc45(uint16_t *mem_buf, uint32_t sample_count)
{
    if (mem_buf == NULL || sample_count == 0) return;
    /* Enable GPIO clocks before configuring the three analog inputs. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    __DSB(); __ISB();
    GPIOC->MODER &= ~((3U << (4*2)) | (3U << (5*2)));
    GPIOC->MODER |=  ((3U << (4*2)) | (3U << (5*2)));
    GPIOC->PUPDR &= ~((3U << (4*2)) | (3U << (5*2)));
    GPIOC->OSPEEDR &= ~((3U << (4*2)) | (3U << (5*2)));
    GPIOC->OTYPER &= ~((1U << 4) | (1U << 5));
    GPIOC->AFR[0] &= ~((0xFU << (4*4)) | (0xFU << (5*4)));
    GPIOB->MODER &= ~(3U << (0*2));
    GPIOB->MODER |=  (3U << (0*2));
    GPIOB->PUPDR &= ~(3U << (0*2));
    GPIOB->OSPEEDR &= ~(3U << (0*2));
    GPIOB->OTYPER &= ~(1U << 0);
    GPIOB->AFR[0] &= ~(0xFU << (0*4));
    __DSB(); __ISB();
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    __DSB(); __ISB();
    for (volatile int i = 0; i < 200; ++i) __NOP();
    /* Use the longest sample time for the three entropy-source channels. */
    ADC1->SMPR1 &= ~((7U << ((14 - 10) * 3)) | (7U << ((15 - 10) * 3)));
    ADC1->SMPR1 |=  ((7U << ((14 - 10) * 3)) | (7U << ((15 - 10) * 3)));
    ADC1->SMPR2 &= ~(7U << (8 * 3));
    ADC1->SMPR2 |=  (7U << (8 * 3));
    /* Regular conversion order: PC4/CH14, PC5/CH15, PB0/CH8. */
    ADC1->SQR1 &= ~(0xFUL << 20);
    ADC1->SQR1 |=  (2U << 20);
    ADC1->SQR3  = 0;
    ADC1->SQR3 |= (14U << 0);
    ADC1->SQR3 |= (15U << 5);
    ADC1->SQR3 |= (8U  << 10);
    /* Continuous scan mode keeps ADC1 issuing DMA requests. */
    ADC1->CR1 |= ADC_CR1_SCAN;
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_DDS;
    __DSB(); __ISB();
    /* DMA2 Stream4 Channel 0 carries ADC1 samples into memory. */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    __DSB(); __ISB();
    DMA2_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream4->CR & DMA_SxCR_EN) { __NOP(); }
    DMA2->HIFCR = (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |
                   DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4);
    uint32_t cr = 0;
    cr |= (0U << DMA_SxCR_CHSEL_Pos);
    cr &= ~(3U << DMA_SxCR_DIR_Pos);
    cr |= DMA_SxCR_MINC;
    cr &= ~DMA_SxCR_PINC;
    cr &= ~(3U << DMA_SxCR_PSIZE_Pos);
    cr |= (1U << DMA_SxCR_PSIZE_Pos);
    cr &= ~(3U << DMA_SxCR_MSIZE_Pos);
    cr |= (1U << DMA_SxCR_MSIZE_Pos);
    cr |= DMA_SxCR_CIRC;
    cr |= (2U << DMA_SxCR_PL_Pos);
    cr |= DMA_SxCR_TCIE;
    DMA2_Stream4->CR = cr;
    DMA2_Stream4->PAR  = (uint32_t)(&ADC1->DR);
    DMA2_Stream4->M0AR = (uint32_t)mem_buf;
    /* Three 16-bit transfers are stored for each sample group. */
    DMA2_Stream4->NDTR = sample_count * 3U;
    /* DMA completion signals the collection loop through g_transfer_complete. */
    NVIC_SetPriority(DMA2_Stream4_IRQn, 3);
    NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    __DSB(); __ISB();
    DMA2_Stream4->CR |= DMA_SxCR_EN;
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 5000; ++i) __NOP();
    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    //printf("ADC+DMA continuous mode started (PC4=CH14, PC5=CH15, PB0=CH8)\r\n\\");
}
/**
 * @brief Reset the three ADC byte buffers and their write indices.
 */
void adc_byte_buffers_init(void)
{
    /* Initialize the required state before enabling the peripheral. */
    idx_pc4 = idx_pc5 = idx_pb0 = 0;
    memset(buf_pc4, 0, sizeof(buf_pc4));
    memset(buf_pc5, 0, sizeof(buf_pc5));
    memset(buf_pb0, 0, sizeof(buf_pb0));
}
/**
 * @brief Store the low byte from one PC4/PC5/PB0 ADC sample triplet.
 * @param pc4_val PC4 ADC sample.
 * @param pc5_val PC5 ADC sample.
 * @param pb0_val PB0 ADC sample.
 * @return True when all three byte buffers are full.
 */
bool store_adc_byte_values(uint16_t pc4_val, uint16_t pc5_val, uint16_t pb0_val)
{
    /* Check the sampling result before using the values. */
    if (idx_pc4 < BUF_SZ) {
        buf_pc4[idx_pc4++] = (uint8_t)(pc4_val & 0xFF);
    }
    if (idx_pc5 < BUF_SZ) {
        buf_pc5[idx_pc5++] = (uint8_t)(pc5_val & 0xFF);
    }
    if (idx_pb0 < BUF_SZ) {
        buf_pb0[idx_pb0++] = (uint8_t)(pb0_val & 0xFF);
    }
    return (idx_pc4 >= BUF_SZ) && (idx_pc5 >= BUF_SZ) && (idx_pb0 >= BUF_SZ);
}
/**
 * @brief Read the latest PC4, PC5 and PB0 triplet from the DMA buffer.
 * @param adc_buffer DMA sample buffer.
 * @param sample_count Number of sample groups in the buffer.
 * @param pc4_val PC4 sample output.
 * @param pc5_val PC5 sample output.
 * @param pb0_val PB0 sample output.
 */
void adc_get_latest_triplet(uint16_t *adc_buffer,
                            uint8_t sample_count,
                            uint16_t *pc4_val,
                            uint16_t *pc5_val,
                            uint16_t *pb0_val)
{
    /* Check the sampling result before using the values. */
    size_t last_idx = ((size_t)sample_count - 1U) * 3U;
    *pc4_val = adc_buffer[last_idx + 0];
    *pc5_val = adc_buffer[last_idx + 1];
    *pb0_val = adc_buffer[last_idx + 2];
}
/**
 * @brief Consume DMA-complete events until all three ADC byte buffers are full.
 * @return 0 on success, otherwise a negative error code.
 */
int collect_adc_samples_until_full(void)
{
    for (;;) {
    	if (!g_transfer_complete) {
    		continue;
    	}
        /* Consume the DMA-complete flag atomically against the IRQ handler. */
        __disable_irq();
        g_transfer_complete = 0;
        __enable_irq();
        adc_get_latest_triplet(adc_buffer, SAMPLE_COUNT,
                               &pc4_val, &pc5_val, &pb0_val);
        all_full = store_adc_byte_values(pc4_val, pc5_val, pb0_val);
        if (all_full) {
            return 0;
        }
    }
    return -1;
}
/**
 * @brief Start ADC DMA sampling and collect entropy-source bytes with bounded retries.
 * @return ADC DMA initialization status.
 */
int ADC_DMA_INIT(void)
{
    /* Initialize the required state before enabling the peripheral. */
    int rv = -1;
    int attempt;
    for (attempt = 0; attempt < ADC_DMA_INIT_RETRY_COUNT; ++attempt) {
        dma2_ADC_pc45((uint16_t *)adc_buffer, SAMPLE_COUNT);
        for (volatile int i = 0; i < 1000000; ++i) {
            __NOP();
        }
        rv = collect_adc_samples_until_full();
        if (rv == 0) {
            break;
        }
        log_printf("ADC_DMA_INIT: attempt %d - collect_adc_samples_until_full failed (rv=%d)\n",
                   attempt + 1, rv);
    }
    if (rv != 0) {
        log_printf("ADC_DMA_INIT: ADC collection ultimately failed.\n");
        return ADC_DMA_INIT_COLLECTION_FAILED;
    }
    return ADC_DMA_INIT_SUCCESS;
}
/**
 * @brief Handle ADC1 DMA2 Stream4 completion and error flags.
 */
void DMA2_Stream4_IRQHandler(void)
{
    uint32_t hisr = DMA2->HISR;
    if (hisr & DMA_HISR_TCIF4) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF4;
        g_transfer_complete = 1;
        return;
    }
    if (hisr & DMA_HISR_TEIF4) {
        DMA2->HIFCR = DMA_HIFCR_CTEIF4;
        //printf(\\"DMA Transfer Error!\r\n\\");
    }
    if (hisr & DMA_HISR_DMEIF4) {
        DMA2->HIFCR = DMA_HIFCR_CDMEIF4;
        //printf(\\"DMA Direct Mode Error!\r\n\\");
    }
    if (hisr & DMA_HISR_FEIF4) {
        DMA2->HIFCR = DMA_HIFCR_CFEIF4;
        //sprintf(\\"DMA FIFO Error!\r\n\\");
    }
}
