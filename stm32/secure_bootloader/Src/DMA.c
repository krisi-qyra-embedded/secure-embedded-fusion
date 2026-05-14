/* Author: i fiksuari pas prog | Documentation date: 14 May 2026 */
#include "DMA.h"

/* adc_dma_continuous.c - Continuous ADC sampling on PC4, PC5, PB0 with DMA2 Stream4
 *
 * Configuration:
 * - ADC1: CH14 (PC4), CH15 (PC5), CH8 (PB0)
 * - DMA2 Stream4, Channel 0 (ADC1)
 * - Circular mode for continuous sampling
 * - 3 values per sequence cycle
 */
uint8_t SAMPLE_COUNT = 10;
volatile uint8_t g_transfer_complete = 0;
volatile uint16_t adc_buffer[30];
 uint32_t last_idx;
 uint16_t pc4_val,pc5_val,pb0_val;
 bool all_full;
 /* Three byte buffers (one per ADC channel) */
  uint8_t buf_pc4[BUF_SZ];
  uint8_t buf_pc5[BUF_SZ];
  uint8_t buf_pb0[BUF_SZ];
  /* Write indices */
  size_t idx_pc4;
  size_t idx_pc5;
  size_t idx_pb0;

/* ========== CORRECTED ADC DMA SETUP ========== */
void dma2_ADC_pc45(uint16_t *mem_buf, uint32_t sample_count)
{
    /* basic argument check */
    if (mem_buf == NULL || sample_count == 0) return;

    /* 1) Enable GPIOC and GPIOB clocks early */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    __DSB(); __ISB();

    /* Configure PC4, PC5 as analog (MODER = 11), no pull/AF */
    GPIOC->MODER &= ~((3U << (4*2)) | (3U << (5*2))); /* clear */
    GPIOC->MODER |=  ((3U << (4*2)) | (3U << (5*2))); /* set analog */
    GPIOC->PUPDR &= ~((3U << (4*2)) | (3U << (5*2))); /* no pull */
    GPIOC->OSPEEDR &= ~((3U << (4*2)) | (3U << (5*2)));
    GPIOC->OTYPER &= ~((1U << 4) | (1U << 5));
    GPIOC->AFR[0] &= ~((0xFU << (4*4)) | (0xFU << (5*4))); /* clear AF bits */

    /* Configure PB0 as analog (MODER = 11), no pull/AF */
    GPIOB->MODER &= ~(3U << (0*2)); /* clear */
    GPIOB->MODER |=  (3U << (0*2)); /* set analog */
    GPIOB->PUPDR &= ~(3U << (0*2)); /* no pull */
    GPIOB->OSPEEDR &= ~(3U << (0*2));
    GPIOB->OTYPER &= ~(1U << 0);
    GPIOB->AFR[0] &= ~(0xFU << (0*4)); /* clear AF0 bits */

    __DSB(); __ISB();

    /* 2) ADC clock enable */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    __DSB(); __ISB();

    /* Settle after clock enable */
    for (volatile int i = 0; i < 200; ++i) __NOP();

    /* 3) ADC sample time: 480 cycles for all channels (0x7) */
    /* CH14, CH15 are in SMPR1 (channels 10-18) */
    ADC1->SMPR1 &= ~((7U << ((14 - 10) * 3)) | (7U << ((15 - 10) * 3)));
    ADC1->SMPR1 |=  ((7U << ((14 - 10) * 3)) | (7U << ((15 - 10) * 3)));

    /* CH8 is in SMPR2 (channels 0-9) */
    ADC1->SMPR2 &= ~(7U << (8 * 3));
    ADC1->SMPR2 |=  (7U << (8 * 3));

    /* 4) Regular sequence: 3 conversions (CH14 -> CH15 -> CH8)
       L = 2 (for 3 conversions: 0, 1, 2)
       SQR3 holds SQ1, SQ2, SQ3 (first 3 conversions)
    */
    ADC1->SQR1 &= ~(0xFUL << 20);    /* clear L field */
    ADC1->SQR1 |=  (2U << 20);       /* L = 2 => 3 conversions */

    ADC1->SQR3  = 0;                 /* clear sequence register */
    ADC1->SQR3 |= (14U << 0);        /* SQ1 = CH14 (PC4) */
    ADC1->SQR3 |= (15U << 5);        /* SQ2 = CH15 (PC5) */
    ADC1->SQR3 |= (8U  << 10);       /* SQ3 = CH8  (PB0) */

    /* 5) Enable SCAN mode and DMA */
    ADC1->CR1 |= ADC_CR1_SCAN;       /* SCAN mode for multiple channels */

    /* CRITICAL FIX: Enable CONTINUOUS mode for truly continuous sampling */
    ADC1->CR2 |= ADC_CR2_CONT;       /* Continuous conversion mode */
    ADC1->CR2 |= ADC_CR2_DMA;        /* Enable DMA requests */
    ADC1->CR2 |= ADC_CR2_DDS;        /* DMA requests issued as long as data is converted */

    __DSB(); __ISB();

    /* 6) DMA2 Stream4 Channel 0 config */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    __DSB(); __ISB();

    /* Disable stream first and wait */
    DMA2_Stream4->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream4->CR & DMA_SxCR_EN) { __NOP(); }

    /* Clear all stream4 flags (in HIFCR for streams 4-7) */
    DMA2->HIFCR = (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 |
                   DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4);

    /* Build DMA CR:
       - Channel 0 (ADC1)
       - P2M (peripheral to memory)
       - 16-bit transfers (ADC DR is 16-bit)
       - Memory increment
       - Circular mode
       - High priority
       - Transfer complete interrupt
    */
    uint32_t cr = 0;
    cr |= (0U << DMA_SxCR_CHSEL_Pos);   /* Channel 0 */
    cr &= ~(3U << DMA_SxCR_DIR_Pos);    /* DIR = 00 (P2M) */
    cr |= DMA_SxCR_MINC;                /* Memory increment */
    cr &= ~DMA_SxCR_PINC;               /* Peripheral no increment */

    /* PSIZE = 01 (16-bit), MSIZE = 01 (16-bit) */
    cr &= ~(3U << DMA_SxCR_PSIZE_Pos);
    cr |= (1U << DMA_SxCR_PSIZE_Pos);   /* 16-bit peripheral */
    cr &= ~(3U << DMA_SxCR_MSIZE_Pos);
    cr |= (1U << DMA_SxCR_MSIZE_Pos);   /* 16-bit memory */

    cr |= DMA_SxCR_CIRC;                /* CIRCULAR mode */
    cr |= (2U << DMA_SxCR_PL_Pos);      /* High priority */
    cr |= DMA_SxCR_TCIE;                /* Transfer complete interrupt */

    DMA2_Stream4->CR = cr;

    /* Set addresses and transfer count */
    DMA2_Stream4->PAR  = (uint32_t)(&ADC1->DR);  /* Peripheral address: ADC data register */
    DMA2_Stream4->M0AR = (uint32_t)mem_buf;      /* Memory address */

    /* NDTR = total 16-bit transfers = sample_count * 3
       (3 values per cycle: PC4, PC5, PB0) */
    DMA2_Stream4->NDTR = sample_count * 3U;

    /* Enable NVIC for DMA2 Stream4 IRQ */
    NVIC_SetPriority(DMA2_Stream4_IRQn, 3);  /* Medium priority */
    NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);

    /* 7) Enable DMA stream */
    __DSB(); __ISB();
    DMA2_Stream4->CR |= DMA_SxCR_EN;

    /* 8) Enable ADC */
    ADC1->CR2 |= ADC_CR2_ADON;

    /* Wait for ADC stabilization (minimum 3us, we wait longer) */
    for (volatile int i = 0; i < 5000; ++i) __NOP();

    /* Clear any pending ADC status flags */
    ADC1->SR = 0;

    /* 9) Start continuous conversions with software trigger */
    ADC1->CR2 |= ADC_CR2_SWSTART;

    //printf("ADC+DMA continuous mode started (PC4=CH14, PC5=CH15, PB0=CH8)\r\n\");
}

/* Initialize/reset buffers */
void adc_byte_buffers_init(void)
{
    /* Initialize the required state before enabling the peripheral. */
    idx_pc4 = idx_pc5 = idx_pb0 = 0;
    memset(buf_pc4, 0, sizeof(buf_pc4));
    memset(buf_pc5, 0, sizeof(buf_pc5));
    memset(buf_pb0, 0, sizeof(buf_pb0));
}


/* Handles ADC sampling or sample conversion. */
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

/* Handles ADC sampling or sample conversion. */
void adc_get_latest_triplet(uint16_t *adc_buffer,
                            uint8_t sample_count,
                            uint16_t *pc4_val,
                            uint16_t *pc5_val,
                            uint16_t *pb0_val)
{
    size_t last_idx = ((size_t)sample_count - 1U) * 3U;

    *pc4_val = adc_buffer[last_idx + 0];  /* CH14 (PC4) */
    *pc5_val = adc_buffer[last_idx + 1];  /* CH15 (PC5) */
    *pb0_val = adc_buffer[last_idx + 2];  /* CH8  (PB0) */
}

/* adc_capture_helper.c
 *
 * Helper to collect ADC triplets produced by DMA and store them via
 * your existing store_adc_byte_values() helper until buffers are full.
 *
 * Usage:
 *   int rc = collect_adc_samples_until_full();
 *   if (rc != 0) handle_error("collection failed");
 *
 * Assumptions (must exist in your project as globals or functions):
 *  - volatile uint8_t g_transfer_complete;
 *  - uint16_t adc_buffer[];            // DMA destination buffer used by dma2_ADC_pc45()
 *  - #define SAMPLE_COUNT ...
 *  - void adc_get_latest_triplet(uint16_t *adc_buffer, uint32_t sample_count,
 *                                uint16_t *pc4, uint16_t *pc5, uint16_t *pb0);
 *  - int  store_adc_byte_values(uint16_t pc4, uint16_t pc5, uint16_t pb0);
 *
 * Returns:
 *   0   -> success (buffers full)
 *  -1   -> unexpected error (optional; currently unused)
 */

int collect_adc_samples_until_full(void)
{

    for (;;) {
        /* Wait for DMA transfer complete flag (spin). */
    	if (!g_transfer_complete) {
    		continue;
    	}

        /* Atomically consume flag to avoid races with IRQ */
        __disable_irq();
        g_transfer_complete = 0;
        __enable_irq();

        /* Extract the latest ADC triplet from DMA buffer */
        adc_get_latest_triplet(adc_buffer, SAMPLE_COUNT,
                               &pc4_val, &pc5_val, &pb0_val);

        /* Store bytes derived from the three channel readings.
           store_adc_byte_values returns non-zero when buffers are full. */
        all_full = store_adc_byte_values(pc4_val, pc5_val, pb0_val);

        if (all_full) {
            return 0; /* success: buffers full */
        }
    }

    /* unreachable */
    return -1;
}

/*
 * ADC_DMA_INIT
 *
 * Starts ADC DMA for PC4/PC5, waits a short time for first conversions,
 * then collects samples until buffers are full. Retries a small number
 * of times on failure. Returns 0 on success or a negative error macro.
 */
int ADC_DMA_INIT(void)
{
    int rv = -1;
    int attempt;

    for (attempt = 0; attempt < ADC_DMA_INIT_RETRY_COUNT; ++attempt) {
        /* Start DMA (fills adc_buffer). Function assumed to initiate DMA and return immediately. */
        dma2_ADC_pc45((uint16_t *)adc_buffer, SAMPLE_COUNT);

        /* Small delay to let first conversions complete */
        for (volatile int i = 0; i < 1000000; ++i) {
            __NOP();
        }

        /* collect until buffers full */
        rv = collect_adc_samples_until_full();
        if (rv == 0) {
            /* success */
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


/* ========== DMA IRQ HANDLER ========== */
void DMA2_Stream4_IRQHandler(void)
{
    uint32_t hisr = DMA2->HISR;

    /* Transfer complete interrupt */
    if (hisr & DMA_HISR_TCIF4) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF4;  /* Clear TC flag */
        g_transfer_complete = 1;         /* Signal main loop */
        return;
    }

    /* Transfer error interrupt */
    if (hisr & DMA_HISR_TEIF4) {
        DMA2->HIFCR = DMA_HIFCR_CTEIF4;  /* Clear TE flag */
        //printf(\"DMA Transfer Error!\r\n\");
        /* Could implement error recovery here */
    }

    /* Direct mode error */
    if (hisr & DMA_HISR_DMEIF4) {
        DMA2->HIFCR = DMA_HIFCR_CDMEIF4;
        //printf(\"DMA Direct Mode Error!\r\n\");
    }

    /* FIFO error */
    if (hisr & DMA_HISR_FEIF4) {
        DMA2->HIFCR = DMA_HIFCR_CFEIF4;
        //sprintf(\"DMA FIFO Error!\r\n\");
    }
}



