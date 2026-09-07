/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "DMA.h"


uint8_t SAMPLE_COUNT = 10;
volatile uint8_t g_transfer_complete = 0;
volatile uint16_t adc_buffer[30];

uint32_t last_idx;
uint16_t pc4_val, pc5_val, pb0_val;
bool all_full;

uint8_t buf_pc4[BUF_SZ];
uint8_t buf_pc5[BUF_SZ];
uint8_t buf_pb0[BUF_SZ];

size_t idx_pc4;
size_t idx_pc5;
size_t idx_pb0;

/* Configures circular ADC1 DMA for PC4, PC5 and PB0. */
void dma2_ADC_pc45(uint16_t *mem_buf, uint32_t sample_count)
{
    /* Disable Stream4 and clear its pending flags before replacing the transfer settings. */
    if (mem_buf == NULL || sample_count == 0) {
        return;
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    __DSB();
    __ISB();

    GPIOC->MODER &= ~((3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOC->MODER |=  ((3U << (4 * 2)) | (3U << (5 * 2)));

    GPIOC->PUPDR &= ~((3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOC->OSPEEDR &= ~((3U << (4 * 2)) | (3U << (5 * 2)));
    GPIOC->OTYPER &= ~((1U << 4) | (1U << 5));
    GPIOC->AFR[0] &= ~((0xFU << (4 * 4)) | (0xFU << (5 * 4)));

    GPIOB->MODER &= ~(3U << (0 * 2));
    GPIOB->MODER |=  (3U << (0 * 2));

    GPIOB->PUPDR &= ~(3U << (0 * 2));
    GPIOB->OSPEEDR &= ~(3U << (0 * 2));
    GPIOB->OTYPER &= ~(1U << 0);
    GPIOB->AFR[0] &= ~(0xFU << (0 * 4));

    __DSB();
    __ISB();

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    __DSB();
    __ISB();

    for (volatile int i = 0; i < 200; ++i) {
        __NOP();
    }

    ADC1->SMPR1 &=
        ~((7U << ((14 - 10) * 3)) |
          (7U << ((15 - 10) * 3)));

    ADC1->SMPR1 |=
         ((7U << ((14 - 10) * 3)) |
          (7U << ((15 - 10) * 3)));

    ADC1->SMPR2 &= ~(7U << (8 * 3));
    ADC1->SMPR2 |=  (7U << (8 * 3));

    /*
     * 4) Regular sequence:
     * CH14 -> CH15 -> CH8
     *
     * L = 2 means 3 conversions.
     */
    ADC1->SQR1 &= ~(0xFUL << 20);
    ADC1->SQR1 |=  (2U << 20);

    ADC1->SQR3 = 0;
    ADC1->SQR3 |= (14U << 0);   /* SQ1 = CH14 (PC4) */
    ADC1->SQR3 |= (15U << 5);   /* SQ2 = CH15 (PC5) */
    ADC1->SQR3 |= (8U << 10);   /* SQ3 = CH8  (PB0) */

    ADC1->CR1 |= ADC_CR1_SCAN;

    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_DDS;

    __DSB();
    __ISB();

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    __DSB();
    __ISB();

    DMA2_Stream4->CR &= ~DMA_SxCR_EN;

    while (DMA2_Stream4->CR & DMA_SxCR_EN) {
        __NOP();
    }

    DMA2->HIFCR =
          DMA_HIFCR_CTCIF4
        | DMA_HIFCR_CHTIF4
        | DMA_HIFCR_CTEIF4
        | DMA_HIFCR_CDMEIF4
        | DMA_HIFCR_CFEIF4;

    uint32_t cr = 0;

    cr |= (0U << DMA_SxCR_CHSEL_Pos);
    cr &= ~(3U << DMA_SxCR_DIR_Pos);

    cr |= DMA_SxCR_MINC;
    cr &= ~DMA_SxCR_PINC;

    /* PSIZE = 16-bit, MSIZE = 16-bit */
    cr &= ~(3U << DMA_SxCR_PSIZE_Pos);
    cr |=  (1U << DMA_SxCR_PSIZE_Pos);

    cr &= ~(3U << DMA_SxCR_MSIZE_Pos);
    cr |=  (1U << DMA_SxCR_MSIZE_Pos);

    cr |= DMA_SxCR_CIRC;
    cr |= (2U << DMA_SxCR_PL_Pos);
    cr |= DMA_SxCR_TCIE;

    DMA2_Stream4->CR = cr;

    DMA2_Stream4->PAR = (uint32_t)(&ADC1->DR);
    DMA2_Stream4->M0AR = (uint32_t)mem_buf;

    /*
     * Total transfers = sample_count * 3
     * PC4, PC5 and PB0 per sequence.
     */
    DMA2_Stream4->NDTR = sample_count * 3U;

    NVIC_SetPriority(DMA2_Stream4_IRQn, 3);
    NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);

    __DSB();
    __ISB();

    DMA2_Stream4->CR |= DMA_SxCR_EN;

    ADC1->CR2 |= ADC_CR2_ADON;

    for (volatile int i = 0; i < 5000; ++i) {
        __NOP();
    }

    ADC1->SR = 0;

    ADC1->CR2 |= ADC_CR2_SWSTART;

    //printf("ADC+DMA continuous mode started (PC4=CH14, PC5=CH15, PB0=CH8)\r\n");
}

/* Resets the three ADC entropy byte buffers. */
void adc_byte_buffers_init(void)
{
    /* Clear each channel's buffer and collection index together. */
    idx_pc4 = idx_pc5 = idx_pb0 = 0;

    memset(buf_pc4, 0, sizeof(buf_pc4));
    memset(buf_pc5, 0, sizeof(buf_pc5));
    memset(buf_pb0, 0, sizeof(buf_pb0));
}

/* Stores the low byte of each ADC channel sample. */
bool store_adc_byte_values(uint16_t pc4_val,
                           uint16_t pc5_val,
                           uint16_t pb0_val)
{
    /* Stop appending to a channel once its entropy buffer is full. */
    if (idx_pc4 < BUF_SZ) {
        buf_pc4[idx_pc4++] = (uint8_t)(pc4_val & 0xFF);
    }

    if (idx_pc5 < BUF_SZ) {
        buf_pc5[idx_pc5++] = (uint8_t)(pc5_val & 0xFF);
    }

    if (idx_pb0 < BUF_SZ) {
        buf_pb0[idx_pb0++] = (uint8_t)(pb0_val & 0xFF);
    }

    return (idx_pc4 >= BUF_SZ) &&
           (idx_pc5 >= BUF_SZ) &&
           (idx_pb0 >= BUF_SZ);
}

/* Extracts the final channel triplet from an ADC DMA block. */
void adc_get_latest_triplet(uint16_t *adc_buffer,
                            uint8_t sample_count,
                            uint16_t *pc4_val,
                            uint16_t *pc5_val,
                            uint16_t *pb0_val)
{
    /* Use the last three-sample sequence in the interleaved buffer. */
    size_t last_idx =
        ((size_t)sample_count - 1U) * 3U;

    *pc4_val = adc_buffer[last_idx + 0];    
    *pc5_val = adc_buffer[last_idx + 1];    
    *pb0_val = adc_buffer[last_idx + 2];    
}

/* Collects ADC bytes until all entropy buffers are full. */
int collect_adc_samples_until_full(void)
{
    /* Consume the DMA-complete flag under interrupt masking; collection is unbounded. */
    for (;;) {

        if (!g_transfer_complete) {
            continue;
        }

        __disable_irq();

        g_transfer_complete = 0;

        __enable_irq();

        adc_get_latest_triplet(
            (uint16_t *)adc_buffer,
            SAMPLE_COUNT,
            &pc4_val,
            &pc5_val,
            &pb0_val);

        all_full =
            store_adc_byte_values(
                pc4_val,
                pc5_val,
                pb0_val);

        if (all_full) {
            return 0;
        }
    }

    return -1;
}

/* Starts ADC DMA collection with the configured retry limit. */
int ADC_DMA_INIT(void)
{
    /* Retry only when the collection routine returns a failure. */
    int rv = -1;
    int attempt;

    for (attempt = 0;
         attempt < ADC_DMA_INIT_RETRY_COUNT;
         ++attempt) {

        dma2_ADC_pc45(
            (uint16_t *)adc_buffer,
            SAMPLE_COUNT);

        for (volatile int i = 0;
             i < 1000000;
             ++i) {

            __NOP();
        }

        rv = collect_adc_samples_until_full();

        if (rv == 0) {
            break;
        }

        log_printf(
            "ADC_DMA_INIT: attempt %d - "
            "collect_adc_samples_until_full failed (rv=%d)\n",
            attempt + 1,
            rv);
    }

    if (rv != 0) {

        log_printf(
            "ADC_DMA_INIT: ADC collection ultimately failed.\n");

        return ADC_DMA_INIT_COLLECTION_FAILED;
    }

    return ADC_DMA_INIT_SUCCESS;
}

/* Acknowledges ADC DMA completion and error flags. */
void DMA2_Stream4_IRQHandler(void)
{
    /* Publish transfer completion only after clearing TCIF4. */
    uint32_t hisr = DMA2->HISR;

    if (hisr & DMA_HISR_TCIF4) {

        DMA2->HIFCR = DMA_HIFCR_CTCIF4;

        g_transfer_complete = 1;

        return;
    }

    if (hisr & DMA_HISR_TEIF4) {

        DMA2->HIFCR = DMA_HIFCR_CTEIF4;

        //printf("DMA Transfer Error!\r\n");

    }

    if (hisr & DMA_HISR_DMEIF4) {

        DMA2->HIFCR = DMA_HIFCR_CDMEIF4;

        //printf("DMA Direct Mode Error!\r\n");
    }

    if (hisr & DMA_HISR_FEIF4) {

        DMA2->HIFCR = DMA_HIFCR_CFEIF4;

        //sprintf("DMA FIFO Error!\r\n");
    }
}
