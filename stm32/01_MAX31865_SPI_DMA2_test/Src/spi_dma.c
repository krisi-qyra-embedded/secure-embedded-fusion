/* spi_dma_expert.c -- CMSIS SPI4 + DMA2 (Stream0=RX, Stream1=TX) Expert-level revision. */

#include "spi_dma.h"
#include "tim.h"
#include "uart.h"   
#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"  
#include "systick.h"
#include "iwdg.h"


 
volatile uint8_t spi_dma_rx_cmplt = 0;
volatile uint8_t spi_dma_tx_cmplt = 0;
 
static volatile uint32_t dma_rx_tc_count = 0;
static volatile uint32_t dma_tx_tc_count = 0;
static volatile uint32_t dma_rx_err_count = 0;
static volatile uint32_t dma_tx_err_count = 0;
static volatile uint32_t dma_rx_ht_count = 0;
static volatile uint32_t dma_tx_ht_count = 0;

 
#ifndef SPI4_DMA_CHANNEL
#define SPI4_DMA_CHANNEL       4U
#endif

 
#define SPI_DMA_STREAM_RX      DMA2_Stream0
#define SPI_DMA_STREAM_TX      DMA2_Stream1

 
#ifndef DMA_LIFCR_ALL_FLAGS_STREAM0
#define DMA_LIFCR_ALL_FLAGS_STREAM0 (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0)
#endif
#ifndef DMA_LIFCR_ALL_FLAGS_STREAM1
#define DMA_LIFCR_ALL_FLAGS_STREAM1 (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1)
#endif

 
#ifndef SPI4_AF_SCK_MISO
 
#define SPI4_AF_SCK_MISO   6U
#endif

#ifndef SPI4_AF_MOSI_PA1
 
#define SPI4_AF_MOSI_PA1   5U
#endif

 
#ifndef SPI_DMA_SHORT_WAIT_COUNT
#define SPI_DMA_SHORT_WAIT_COUNT 10000U    
#endif

#ifndef SPI_DMA_LONG_WAIT_COUNT
#define SPI_DMA_LONG_WAIT_COUNT  200000U   
#endif


 
#ifndef DMA_LISR_TCIF0
#define DMA_LISR_TCIF0 (1U<<5)
#define DMA_LISR_HTIF0 (1U<<4)
#define DMA_LISR_TEIF0 (1U<<3)
#define DMA_LISR_DMEIF0 (1U<<2)
#define DMA_LISR_FEIF0 (1U<<0)
#define DMA_LIFCR_CTCIF0 (1U<<5)
#define DMA_LIFCR_CHTIF0 (1U<<4)
#define DMA_LIFCR_CTEIF0 (1U<<3)
#define DMA_LIFCR_CDMEIF0 (1U<<2)
#define DMA_LIFCR_CFEIF0 (1U<<0)
#endif

#ifndef DMA_LISR_TCIF1
#define DMA_LISR_TCIF1 (1U<<11)
#define DMA_LISR_HTIF1 (1U<<10)
#define DMA_LISR_TEIF1 (1U<<9)
#define DMA_LISR_DMEIF1 (1U<<8)
#define DMA_LISR_FEIF1 (1U<<7)
#define DMA_LIFCR_CTCIF1 (1U<<11)
#define DMA_LIFCR_CHTIF1 (1U<<10)
#define DMA_LIFCR_CTEIF1 (1U<<9)
#define DMA_LIFCR_CDMEIF1 (1U<<8)
#define DMA_LIFCR_CFEIF1 (1U<<7)
#endif

 
int spi4_set_baudrate_prescaler(uint8_t br_code);
static void gpio_config_spi4_pins(void);
static int wait_dma_stream_disabled(volatile __IO uint32_t *stream_CR, volatile __IO uint32_t *flag_reg, uint32_t cr_en_mask, uint32_t clear_mask);
static void dma_clear_all_flags(void);

 
/* Handles the spi4 short delay ms operation. */
 void spi4_short_delay_ms(uint32_t ms)
{
    /* Check the current status before proceeding. */
#ifdef systick_delay_ms
    systick_delay_ms(ms);
#elif defined(tim2_delay_ms)
    tim2_delay_ms(ms);
#else
    volatile uint32_t loops = ms * 1000U;
    while (loops--) { __asm__ volatile("nop"); }
#endif
}


 
/* Accesses the requested setting through set rx complete. */
static inline void set_rx_complete(uint8_t v) {
    /* Check the current status before proceeding. */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    spi_dma_rx_cmplt = v;
    if (!prim) __enable_irq();
}
/* Accesses the requested setting through set tx complete. */
static inline void set_tx_complete(uint8_t v) {
    /* Check the current status before proceeding. */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    spi_dma_tx_cmplt = v;
    if (!prim) __enable_irq();
}

 
/* Initializes the resources used by spi4 dma init. */
int spi4_dma_init(void)
{
    /* Set up the required peripheral configuration. */
     

     
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    __DSB(); __ISB();
    /* log_printf("spi4_dma_init: GPIOA GPIOB clocks enabled\r\n"); */

    gpio_config_spi4_pins();
    /* log_printf("spi4_dma_init: pins configured\r\n"); */

     
#ifdef RCC_APB2ENR_SPI4EN
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
    /* log_printf("spi4_dma_init: RCC_APB2ENR_SPI4EN macro used\r\n"); */
#else
    RCC->APB2ENR |= (1U << 13);
    log_printf("spi4_dma_init: RCC_APB2ENR SPI4EN bit set via (1<<13)\r\n");
#endif
    __DSB(); __ISB();

     
    SPI4->CR1 = 0U;
    SPI4->CR2 = 0U;
    SPI4->SR  = 0U;  
    /* log_printf("spi4_dma_init: SPI4 CR1/CR2/SR cleared\r\n"); */

     
    {
        uint32_t cr1 = 0;

         
#ifdef SPI_CR1_BIDIMODE
        cr1 = SPI4->CR1 & ~(SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE);
#else
        cr1 = SPI4->CR1;
#endif

         
#ifdef SPI_CR1_CRCEN
        cr1 &= ~SPI_CR1_CRCEN;       
#endif
#ifdef SPI_CR1_DFF
        cr1 &= ~SPI_CR1_DFF;         
#endif
        cr1 &= ~SPI_CR1_LSBFIRST;    

         
        cr1 |= SPI_CR1_MSTR;
        cr1 |= SPI_CR1_SSM;
        cr1 |= SPI_CR1_SSI;

         
#ifdef SPI_CR1_BIDIMODE
        cr1 &= ~SPI_CR1_BIDIMODE;    
#endif
        cr1 &= ~SPI_CR1_RXONLY;      

         
        cr1 &= ~SPI_CR1_CPOL;
        cr1 |=  SPI_CR1_CPHA;

         
        cr1 &= ~SPI_CR1_BR;  
        cr1 |= (0x3U << SPI_CR1_BR_Pos);  

         
        SPI4->CR1 = cr1;
        /* log_printf("spi4_dma_init: SPI4 CR1 (HAL-match BR=/4, mode1) = 0x%08X\r\n", (unsigned)SPI4->CR1); */

         
#ifdef SPI_CR2_FRF
        SPI4->CR2 &= ~SPI_CR2_FRF;   
#endif

         
#ifdef SPI_CR2_DS
         
        SPI4->CR2 &= ~SPI_CR2_DS;
        SPI4->CR2 |= (0x7U << SPI_CR2_DS_Pos);  
#endif

         
        SPI4->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
        SPI4->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

        /* log_printf("spi4_dma_init: SPI4 CR2 (HAL-match) = 0x%08X\r\n", (unsigned)SPI4->CR2); */
    }

     
    SPI4->CR1 |= SPI_CR1_SPE;
    /* log_printf("spi4_dma_init: SPI4 enabled CR1=0x%08X\r\n", (unsigned)SPI4->CR1); */

     
#ifdef RCC_AHB1ENR_DMA2EN
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
#else
    RCC->AHB1ENR |= (1U << 22);
#endif
    __DSB(); __ISB();
    /* log_printf("spi4_dma_init: DMA2 clock enabled\r\n"); */

     
    DMA2_Stream0->CR = 0U;
    DMA2_Stream1->CR = 0U;
    /* log_printf("spi4_dma_init: DMA2 Stream0/1 CR cleared\r\n"); */

    if (wait_dma_stream_disabled(&DMA2_Stream0->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM0) < 0) {
        log_printf("spi4_dma_init: error: stream0 did not disable cleanly\r\n");
         return-1;
    }
    if (wait_dma_stream_disabled(&DMA2_Stream1->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM1) < 0) {
        log_printf("spi4_dma_init: error: stream1 did not disable cleanly\r\n");
        return -2;
    }

     
    uint32_t s0cr = 0U;
    s0cr |= (SPI4_DMA_CHANNEL << 25);  
    s0cr &= ~((3U<<6)|(1U<<7));        
    s0cr |= (1U<<10);                  
    s0cr &= ~(1U<<9);                  
    s0cr &= ~((3U<<11) | (3U<<13));    
    s0cr |= (2U<<16);                  
    s0cr |= (1U<<4);                   
    s0cr |= (1U<<2);                   
    DMA2_Stream0->CR = s0cr;
    /* log_printf("spi4_dma_init: DMA2_Stream0->CR (wrote)=0x%08X\r\n", (unsigned)DMA2_Stream0->CR); */

     
    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM0;
    /* log_printf("spi4_dma_init: DMA2 LIFCR cleared for stream0\r\n"); */

     
    uint32_t s1cr = 0U;
    s1cr |= (SPI4_DMA_CHANNEL << 25);  
    s1cr |= (1U<<6);                   
    s1cr |= (1U<<10);                  
    s1cr &= ~(1U<<9);                  
    s1cr &= ~((3U<<11) | (3U<<13));    
    s1cr |= (2U<<16);                  
    s1cr |= (1U<<4);                   
    s1cr |= (1U<<2);                   
    DMA2_Stream1->CR = s1cr;
    /* log_printf("spi4_dma_init: DMA2_Stream1->CR (wrote)=0x%08X\r\n", (unsigned)DMA2_Stream1->CR); */

     
    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM1;
    /* log_printf("spi4_dma_init: DMA2 LIFCR cleared for stream1\r\n"); */

     
    {
        uint32_t s0cr_read = DMA2_Stream0->CR;
        uint32_t s1cr_read = DMA2_Stream1->CR;
        uint32_t dir1 = (s1cr_read >> 6) & 0x3U;          
        uint32_t ch0  = (s0cr_read >> 25) & 0x7U;         
        uint32_t ch1  = (s1cr_read >> 25) & 0x7U;         

        if (ch0 != SPI4_DMA_CHANNEL) {
            log_printf("spi4_dma_init: ERROR - Stream0 channel mismatch: expected %u got %u (CR=0x%08X)\r\n",
                       (unsigned)SPI4_DMA_CHANNEL, (unsigned)ch0, (unsigned)s0cr_read);
            return -3;
        }
        if (ch1 != SPI4_DMA_CHANNEL) {
            log_printf("spi4_dma_init: ERROR - Stream1 channel mismatch: expected %u got %u (CR=0x%08X)\r\n",
                       (unsigned)SPI4_DMA_CHANNEL, (unsigned)ch1, (unsigned)s1cr_read);
            return -4;
        }
        if (dir1 != 1U) {
            log_printf("spi4_dma_init: ERROR - Stream1 DIR != M2P (CR=0x%08X)\r\n", (unsigned)s1cr_read);
            return -5;
        }
    }

     
    NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 5);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
    NVIC_SetPriority(DMA2_Stream1_IRQn, 5);
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    /* log_printf("spi4_dma_init: NVIC enabled for Stream0/1 with priority 5\r\n"); */

     
    dma_rx_tc_count = dma_tx_tc_count = 0;
    dma_rx_err_count = dma_tx_err_count = 0;
    dma_rx_ht_count = dma_tx_ht_count = 0;
    hb_spi_ts = 0;

    set_rx_complete(0);
    set_tx_complete(0);

    /* log_printf("spi4_dma_init: done (MAX31865 ready, SCLK = PCLK/4 -> ~4.0 MHz @ PCLK=16MHz)\r\n"); */
    return 0;
}

/* gpio helper (verbose, with before/after register prints) */
static void gpio_config_spi4_pins(void)
{
    /* Set up the required peripheral configuration. */
     
    {
        const uint32_t pin = 13U;
        uint32_t moder_before   = GPIOB->MODER;
        uint32_t afr1_before    = GPIOB->AFR[1];
        uint32_t ospeedr_before = GPIOB->OSPEEDR;
        uint32_t pupdr_before   = GPIOB->PUPDR;

         
        GPIOB->MODER &= ~(3U << (pin * 2));
        GPIOB->MODER |=  (2U << (pin * 2));
        __DSB(); __ISB();

         
        {
            uint32_t shift = (pin - 8U) * 4U;
            GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFU << shift)) | (SPI4_AF_SCK_MISO << shift);
        }
         
        GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(3U << (pin * 2))) | (2U << (pin * 2));
        GPIOB->PUPDR &= ~(3U << (pin * 2));
        __DSB(); __ISB();

         
 
 
 
/* (unsigned)afr1_before, (unsigned)GPIOB->AFR[1], (unsigned)((pin-8U)*4U), (unsigned)SPI4_AF_SCK_MISO); */
 
 
 
 
    }

     
    {
        const uint32_t pin = 11U;
        uint32_t moder_before   = GPIOA->MODER;
        uint32_t afr1_before    = GPIOA->AFR[1];
        uint32_t ospeedr_before = GPIOA->OSPEEDR;
        uint32_t pupdr_before   = GPIOA->PUPDR;

         
        GPIOA->MODER &= ~(3U << (pin * 2));
        GPIOA->MODER |=  (2U << (pin * 2));
        __DSB(); __ISB();

         
        {
            uint32_t shift = (pin - 8U) * 4U;
            GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFU << shift)) | (SPI4_AF_SCK_MISO << shift);
        }
        GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (pin * 2))) | (2U << (pin * 2));
        GPIOA->PUPDR &= ~(3U << (pin * 2));
        __DSB(); __ISB();

         
 
 
 
/* (unsigned)afr1_before, (unsigned)GPIOA->AFR[1], (unsigned)((pin-8U)*4U), (unsigned)SPI4_AF_SCK_MISO); */
 
 
 
 
    }

     
    {
        const uint32_t pin = 1U;
        uint32_t moder_before   = GPIOA->MODER;
        uint32_t afr0_before    = GPIOA->AFR[0];
        uint32_t ospeedr_before = GPIOA->OSPEEDR;
        uint32_t pupdr_before   = GPIOA->PUPDR;

         
        GPIOA->MODER &= ~(3U << (pin * 2));
        GPIOA->MODER |=  (2U << (pin * 2));
        __DSB(); __ISB();

         
        {
            uint32_t shift = (pin * 4U);
            GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << shift)) | (SPI4_AF_MOSI_PA1 << shift);
        }
        GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (pin * 2))) | (2U << (pin * 2));
        GPIOA->PUPDR &= ~(3U << (pin * 2));
        __DSB(); __ISB();

         
 
 
 
/* (unsigned)afr0_before, (unsigned)GPIOA->AFR[0], (unsigned)(pin*4U), (unsigned)SPI4_AF_MOSI_PA1); */
 
 
 
 
    }

     
 
 
}


/* set SPI4 baudrate prescaler (BR field 3 bits: 0..7) br_code: 0 -> fPCLK/2, 1 -> /4, 2 -> /8, ... */
int spi4_set_baudrate_prescaler(uint8_t br_code)
{
    /* Check the current status before proceeding. */
    if (br_code > 7U) return -1;

    uint32_t was_enabled = (SPI4->CR1 & SPI_CR1_SPE) ? 1U : 0U;
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

     
    SPI4->CR1 &= ~SPI_CR1_SPE;
    __DSB(); __ISB();

     
    SPI4->CR1 &= ~SPI_CR1_BR;
    SPI4->CR1 |= ((uint32_t)br_code << SPI_CR1_BR_Pos);

     
    if (was_enabled) {
        __DSB(); __ISB();
        SPI4->CR1 |= SPI_CR1_SPE;
    }

    if (!prim) __enable_irq();
    return 0;
}


/* small helpers for SPI correctness */
 
 void spi4_flush_rx(void)
{
    /* Check the current status before proceeding. */
    uint32_t limit = 64U;  
    while ((SPI4->SR & SPI_SR_RXNE) && limit--) {
        volatile uint32_t tmp = SPI4->DR;   
        (void)tmp;
        __DSB(); __ISB();
    }
}

/* Clear Overrun (OVR) condition if present. */
 void spi4_check_and_clear_ovr(void)
{
    /* Check the current status before proceeding. */
    if (SPI4->SR & SPI_SR_OVR) {
        /* read DR then SR to clear OVR flag (defensive reads) */
        volatile uint32_t tmp = SPI4->DR; (void)tmp;
        tmp = SPI4->SR; (void)tmp;
        __DSB(); __ISB();
        /* optional diagnostic: // log_printf("spi4_check_and_clear_ovr: OVR cleared\r\n"); */
    }
}

 
/* Handles the dma clear all flags operation. */
static void dma_clear_all_flags(void)
{
    /* Check the current status before proceeding. */
    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM0;
    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM1;
}

 
/* Handles the spi4 dma transmit operation. */
int spi4_dma_transmit(uint8_t *tx_buf, uint32_t len)
{
    /* Check the current status before proceeding. */
    if (tx_buf == NULL || len == 0U) {
        log_printf("spi4_dma_transmit: invalid args\r\n");
        return -1;
    }

    /* Ensure SPI is enabled and RX is drained to avoid OVR when TX generates clocks */
    spi4_flush_rx();
    spi4_check_and_clear_ovr();

     
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;

    /* NOTE: wait_dma_stream_disabled() does NOT call spi4_recover()/spi4_restore_and_init(). */
    if (wait_dma_stream_disabled(&DMA2_Stream1->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM1) < 0)
    {
        log_printf("spi4_dma_transmit: wait for stream1 disable failed \n\r");
        return -2;  
    }

    /* stream disabled: clear flags and program DMA stream */
    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM1;

    DMA2_Stream1->PAR  = (uint32_t)&(SPI4->DR);
    DMA2_Stream1->M0AR = (uint32_t)tx_buf;
    DMA2_Stream1->NDTR = len;

    __DSB(); __ISB();
    set_tx_complete(0);
    DMA2_Stream1->CR |= DMA_SxCR_EN;

    return 0;
}

 
/* Handles the spi4 dma receive operation. */
int spi4_dma_receive(uint8_t *rx_buf, uint32_t len)
{
    /* Check the current status before proceeding. */
    log_printf("spi4_dma_receive: requested len=%u, rx_buf=%p\r\n", (unsigned)len, (void*)rx_buf);
    if (rx_buf == NULL || len == 0U) {
        log_printf("spi4_dma_receive: invalid args\r\n");
        return -1;
    }

    /* Ensure SPI is enabled and RX is drained to avoid OVR when RX/TX generates clocks */
    spi4_flush_rx();
    spi4_check_and_clear_ovr();

     
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;

    /* NOTE: wait_dma_stream_disabled() does not call spi4_recover()/spi4_restore_and_init(). */
    if (wait_dma_stream_disabled(&DMA2_Stream0->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM0) < 0)
    {
        log_printf("spi4_dma_receive: wait for stream0 disable failed \n\r");
        return -2;  
    }

    /* stream disabled: clear flags and program DMA stream */
    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM0;

    DMA2_Stream0->PAR  = (uint32_t)&(SPI4->DR);
    DMA2_Stream0->M0AR = (uint32_t)rx_buf;
    DMA2_Stream0->NDTR = len;

    __DSB(); __ISB();
    set_rx_complete(0);
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    return 0;
}

 
/* Handles the spi4 dma transmit receive operation. */
int spi4_dma_transmit_receive(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    /* log_printf("spi4_dma_transmit_receive: tx=%p rx=%p len=%u\r\n", (void*)tx_buf, (void*)rx_buf. */
    if (tx_buf == NULL || rx_buf == NULL || len == 0U) {
        log_printf("spi4_dma_transmit_receive: invalid args\r\n");
        return -1;
    }

    /* Ensure SPI is enabled and RX is drained to avoid OVR when TX generates clocks */
    spi4_flush_rx();
    spi4_check_and_clear_ovr();

     
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;

    /* ensure stream1 is disabled, with staged recovery fallback */
    if (wait_dma_stream_disabled(&DMA2_Stream1->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM1) < 0)
    {
        log_printf("spi4_dma_transmit_receive: stream1 disable failed (attempt %d/%d)\n\r");
        return -2;
    }

    /* ensure stream0 is disabled, with staged recovery fallback */
    if (wait_dma_stream_disabled(&DMA2_Stream0->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM0) < 0)
    {
        log_printf("spi4_dma_transmit_receive: stream0 disable failed \n\r");
        return -2;
    }

    /* both streams disabled: clear flags and program RX then TX */
    dma_clear_all_flags();

     
    DMA2_Stream0->PAR  = (uint32_t)&(SPI4->DR);
    DMA2_Stream0->M0AR = (uint32_t)rx_buf;
    DMA2_Stream0->NDTR = len;

     
    DMA2_Stream1->PAR  = (uint32_t)&(SPI4->DR);
    DMA2_Stream1->M0AR = (uint32_t)tx_buf;
    DMA2_Stream1->NDTR = len;

    __DSB(); __ISB();
    set_rx_complete(0);
    set_tx_complete(0);

     
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    DMA2_Stream1->CR |= DMA_SxCR_EN;

    return 0;
}

/* blocking wait helpers (CMSIS-style) */
int spi_dma_stream0_wait_complete(uint32_t timeout_ms)
{
    /* log_printf("spi_dma_stream0_wait_complete: waiting up to %u ms\r\n", (unsigned)timeout_ms); */
    tim2_1s_start();
    uint32_t elapsed_ms = 0;
    while (spi_dma_rx_cmplt == 0) {
        for (volatile uint32_t i=0;i<100;i++) __asm__ volatile ("nop");
        if (tim2_1s_check_flag()) {
            elapsed_ms += 1000;
            if (elapsed_ms >= timeout_ms) {
                tim2_1s_stop();
                /* log_printf("spi_dma_stream0_wait_complete: timeout after %u ms, CR=0x%08X NDTR=%u\r\n". */
                return -1;
            }
        }
    }
    tim2_1s_stop();
    /* log_printf("spi_dma_stream0_wait_complete: done status=%u\r\n", (unsigned)spi_dma_rx_cmplt); */
    return (spi_dma_rx_cmplt == 1) ? 0 : -2;
}

/* Waits for spi dma stream1 wait complete with a bounded timeout. */
int spi_dma_stream1_wait_complete(uint32_t timeout_ms)
{
    /* log_printf("spi_dma_stream1_wait_complete: waiting up to %u ms\r\n", (unsigned)timeout_ms); */
    tim2_1s_start();
    uint32_t elapsed_ms = 0;
    while (spi_dma_tx_cmplt == 0) {
        for (volatile uint32_t i=0;i<100;i++) __asm__ volatile ("nop");
        if (tim2_1s_check_flag()) {
            elapsed_ms += 1000;
            if (elapsed_ms >= timeout_ms) {
                tim2_1s_stop();
                /* log_printf("spi_dma_stream1_wait_complete: timeout after %u ms, CR=0x%08X NDTR=%u\r\n". */
                return -1;
            }
        }
    }
    tim2_1s_stop();
    /* log_printf("spi_dma_stream1_wait_complete: done status=%u\r\n", (unsigned)spi_dma_tx_cmplt); */
    return (spi_dma_tx_cmplt == 1) ? 0 : -2;
}

 
/* Checks the state required by tim2 1s check flag. */
uint8_t tim2_1s_check_flag(void) {
    /* Check the current status before proceeding. */
    if (tim2_1s_flag) { tim2_1s_flag = 0;  
        return 1;
    }
    return 0;
}

/* existing IRQ: OK as you had it, it stops timer and sets flag */
void TIM2_IRQHandler(void) {
    /* Update the transfer state for handled interrupt flags. */
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN;  
        TIM2->DIER &= ~TIM_DIER_UIE;  
        tim2_1s_flag = 1;
    }
}



 
/* Handles the interrupt state for DMA2 Stream0 IRQHandler. */
void DMA2_Stream0_IRQHandler(void)
{
    /* Update the transfer state for handled interrupt flags. */
    uint32_t lisr = DMA2->LISR;
    /* log_printf("DMA2_Stream0_IRQHandler: entry LISR=0x%08X LIFCR=0x%08X\r\n", (unsigned)lisr. */

    if (lisr & DMA_LISR_TCIF0) {
        dma_rx_tc_count++;
        heartbeat_signal_spi();
        /* log_printf("DMA2_Stream0_IRQHandler: TCIF0 detected\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();
        set_rx_complete(1);
        /* log_printf("DMA2_Stream0_IRQHandler: RX complete set, NDTR now=%u CR=0x%08X\r\n". */
         
        spi4_check_and_clear_ovr();
        return;
    }

    if (lisr & DMA_LISR_HTIF0) {
        dma_rx_ht_count++;
        /* log_printf("DMA2_Stream0_IRQHandler: HTIF0 detected (half transfer)\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CHTIF0;
         
        return;
    }

    if (lisr & DMA_LISR_TEIF0) {
        dma_rx_err_count++;
        /* log_printf("DMA2_Stream0_IRQHandler: TEIF0 detected (transfer error)\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CTEIF0;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();
        set_rx_complete(2);
        return;
    }

    if (lisr & DMA_LISR_DMEIF0) {
        dma_rx_err_count++;
        /* log_printf("DMA2_Stream0_IRQHandler: DMEIF0 detected (direct mode error)\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CDMEIF0;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();
        set_rx_complete(2);
        return;
    }

    if (lisr & DMA_LISR_FEIF0) {
        dma_rx_err_count++;
        /* log_printf("DMA2_Stream0_IRQHandler: FEIF0 detected (fifo error)\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CFEIF0;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();
        set_rx_complete(2);
        return;
    }

    /* log_printf("DMA2_Stream0_IRQHandler: no handled flag found LISR=0x%08X\r\n", (unsigned)lisr); */
}

/* Handles the interrupt state for DMA2 Stream1 IRQHandler. */
void DMA2_Stream1_IRQHandler(void)
{
    /* Update the transfer state for handled interrupt flags. */
    uint32_t lisr = DMA2->LISR;
    /* log_printf("DMA2_Stream1_IRQHandler: entry LISR=0x%08X LIFCR=0x%08X\r\n", (unsigned)lisr. */

    if (lisr & DMA_LISR_TCIF1) {
        dma_tx_tc_count++;
        /* log_printf("DMA2_Stream1_IRQHandler: TCIF1 detected\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CTCIF1;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();
        set_tx_complete(1);
        /* log_printf("DMA2_Stream1_IRQHandler: TX complete set, NDTR now=%u CR=0x%08X\r\n". */
        return;
    }

    if (lisr & DMA_LISR_HTIF1) {
        dma_tx_ht_count++;
        /* log_printf("DMA2_Stream1_IRQHandler: HTIF1 detected (half transfer)\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CHTIF1;
        return;
    }

    if (lisr & DMA_LISR_TEIF1) {
        dma_tx_err_count++;
        /* log_printf("DMA2_Stream1_IRQHandler: TEIF1 detected (transfer error)\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CTEIF1;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();
        set_tx_complete(2);
        return;
    }

    if (lisr & DMA_LISR_DMEIF1) {
        dma_tx_err_count++;
        /* log_printf("DMA2_Stream1_IRQHandler: DMEIF1 detected (direct mode error)\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CDMEIF1;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();
        set_tx_complete(2);
        return;
    }

    if (lisr & DMA_LISR_FEIF1) {
        dma_tx_err_count++;
        /* log_printf("DMA2_Stream1_IRQHandler: FEIF1 detected (fifo error)\r\n"); */
        DMA2->LIFCR = DMA_LIFCR_CFEIF1;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();
        set_tx_complete(2);
        return;
    }

    /* log_printf("DMA2_Stream1_IRQHandler: no handled flag found LISR=0x%08X\r\n", (unsigned)lisr); */
}

 
/* Waits for wait dma stream disabled with a bounded timeout. */
static int wait_dma_stream_disabled(volatile __IO uint32_t *stream_CR, volatile __IO uint32_t *flag_reg, uint32_t cr_en_mask, uint32_t clear_mask)
{
    /* log_printf("wait_dma_stream_disabled: waiting on CR=0x%08X mask=0x%08X\r\n", (unsigned)(*stream_CR). */

    uint32_t short_wait = SPI_DMA_SHORT_WAIT_COUNT;
    while (((*stream_CR) & cr_en_mask) && short_wait--) { __asm__ volatile("nop"); }
    if (((*stream_CR) & cr_en_mask) == 0U) {
        /* log_printf("wait_dma_stream_disabled: disabled by short spin\r\n"); */
        return 0;  
    }

    /* log_printf("wait_dma_stream_disabled: short spin expired, using tim fallback\r\n"); */
    tim2_1s_start();
    uint32_t long_wait = SPI_DMA_LONG_WAIT_COUNT;
    while (((*stream_CR) & cr_en_mask) && !tim2_1s_check_flag() && long_wait--) { __asm__ volatile ("nop"); }
    tim2_1s_stop();

    if (((*stream_CR) & cr_en_mask) == 0U) {
        log_printf("wait_dma_stream_disabled: disabled after long wait\r\n");
        return 0;  
    }

    /* final forced path: clear CR and flags explicitly */
    /* log_printf("wait_dma_stream_disabled: forcing disable and clearing flags\r\n"); */
    * (volatile uint32_t *) stream_CR = 0U;
    *flag_reg = clear_mask;
    for (volatile uint32_t i=0;i<2000;i++) __asm__ volatile ("nop");

    int ok = (((*stream_CR) & cr_en_mask) == 0U) ? 1 : 0;
    /* log_printf("wait_dma_stream_disabled: final state CR=0x%08X ok=%d\r\n", (unsigned)(*stream_CR), ok); */
    return ok ? 0 : 1;  
}

/* Weak hook: app may implement to reinitialise attached SPI device(s) after SPI/DMA reset. */
__attribute__((weak)) int spi4_device_reinit(void) {
    /* Set up the required peripheral configuration. */ return 0; }

/* Weak hook: optional lightweight self-test. */
__attribute__((weak)) int spi4_selftest(void) {
    /* Check the current status before proceeding. */ return 0; }

/* attempt staged recovery: return 0 = recovered, -1 = unrecoverable */
int spi4_recover(void)
{
    /* Recover the peripheral state before retrying the transfer. */
    int recovered = 0;

    log_printf("spi4_recover: start\n\r");

    /* Step 0: disable DMA streams and clear flags (cheap attempt) */
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    __DSB(); __ISB();
    dma_clear_all_flags();

     
    spi4_short_delay_ms(2);

    /* Try to flush and clear any SPI RX/OVR conditions */
    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    /* Try to gracefully stop SPI and wait for BSY to clear */
    SPI4->CR1 &= ~SPI_CR1_SPE;  
    __DSB(); __ISB();

     
    {
        uint32_t timeout = 1000U;  
        while ((SPI4->SR & SPI_SR_BSY) && timeout--) {
            __asm__ volatile ("nop");
        }
    }

    /* If streams are disabled and SPI is not busy, consider recovered */
    if (((DMA2_Stream0->CR & DMA_SxCR_EN) == 0U) &&
        ((DMA2_Stream1->CR & DMA_SxCR_EN) == 0U) &&
        ((SPI4->SR & SPI_SR_BSY) == 0U)) {
        log_printf("spi4_recover: graceful disable succeeded\n\r");
        recovered = 1;
    }

    /* If graceful path failed, escalate to APB reset of SPI peripheral */
    if (!recovered) {
        log_printf("spi4_recover: graceful disable failed -> toggling SPI APB reset\n\r");
#ifdef RCC_APB2RSTR_SPI4RST
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI4RST;
        spi4_short_delay_ms(1);
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI4RST;
#else
        RCC->APB2RSTR |= (1U << 13);
        spi4_short_delay_ms(1);
        RCC->APB2RSTR &= ~(1U << 13);
#endif
        __DSB(); __ISB();

        /* Clear DMA flags and force streams off (AHB domain) */
        dma_clear_all_flags();
        DMA2_Stream0->CR = 0U;
        DMA2_Stream1->CR = 0U;
        __DSB(); __ISB();

         
        spi4_short_delay_ms(2);

        /* If streams now disabled and SPI not busy, mark recovered and attempt lightweight reprogram */
        if (((DMA2_Stream0->CR & DMA_SxCR_EN) == 0U) &&
            ((DMA2_Stream1->CR & DMA_SxCR_EN) == 0U) &&
            ((SPI4->SR & SPI_SR_BSY) == 0U)) {
            log_printf("spi4_recover: peripheral reset cleared stuck state\n\r");
            recovered = 1;
        } else {
            log_printf("spi4_recover: peripheral reset did not clear stuck state\n\r");
            recovered = 0;
        }
    }

    /* If we got to recovered==1, attempt lightweight restore: restore DMA templates / reprogram PAR/NDTR and. */
    if (recovered) {
         
        dma_clear_all_flags();
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        __DSB(); __ISB();

         
        spi4_flush_rx();
        spi4_check_and_clear_ovr();

        /* Re-enable SPI with safe default settings if CR1/CR2 were reset (best-effort). */
        SPI4->CR1 |= SPI_CR1_SPE;  
        __DSB(); __ISB();

        /* Run optional device reinit hook (weak). */
        if (spi4_device_reinit() != 0) {
            log_printf("spi4_recover: spi4_device_reinit() failed\n\r");
            recovered = 0;  
        } else {
             
            if (spi4_selftest() != 0) {
                log_printf("spi4_recover: spi4_selftest() failed\n\r");
                recovered = 0;  
            } else {
                log_printf("spi4_recover: lightweight restore OK\n\r");
                return 0;  
            }
        }
    }

     
    log_printf("spi4_recover: attempting full spi4_dma_init() re-init\n\r");

     
    if (spi4_dma_init() == 0) {
         
        if (spi4_device_reinit() == 0 && spi4_selftest() == 0) {
            log_printf("spi4_recover: full re-init and self-test succeeded\n\r");
            return 0;
        } else {
            log_printf("spi4_recover: device reinit or self-test failed after full init\n\r");
            return -1;
        }
    }

    log_printf("spi4_recover: full re-init failed\n\r");
    return -1;
}


/* Initializes the resources used by spi4 restore and init. */
int spi4_restore_and_init(void)
{
    /* Set up the required peripheral configuration. */
    int rc;

    log_printf("spi4_restore_and_init: starting full restore/init\n\r");

    /* 1) Full (idempotent) init of SPI+DMA. */
    rc = spi4_dma_init();
    if (rc != 0) {
        log_printf("spi4_restore_and_init: spi4_dma_init() failed rc=%d\n\r", rc);
        return -1;
    }

     
    spi4_short_delay_ms(2);

    /* 2) Ensure SPI is enabled and RX is drained to avoid OVR when TX generates clocks */
    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    /* 3) Disable both streams explicitly and wait for them to be disabled */
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    __DSB(); __ISB();

    log_printf("spi4_restore_and_init: restore & init completed OK\n\r");
    return 0;
}

