/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#include "spi_dma.h"
#include "tim.h"
#include "uart.h"
#include "systick.h"
#include "iwdg.h"
#include <stdint.h>


#ifndef SPI4_SYSCLK_HZ
#define SPI4_SYSCLK_HZ              (96000000UL)
#endif

/*
 * IMPORTANT:
 * SPI4 on STM32F4 is clocked from APB2 (PCLK2), not directly from SYSCLK.
 *
 * For a common 96 MHz setup:
 *   SYSCLK = 96 MHz
 *   APB2   = 96 MHz  (/1)
 *
 * If your APB2 prescaler is different, change ONLY SPI4_PCLK2_HZ.
 */
#ifndef SPI4_PCLK2_HZ
#define SPI4_PCLK2_HZ               (96000000UL)
#endif

/*
 * Choose a conservative SPI clock for MAX31865.
 * Here target is 3 MHz.
 *
 * Examples:
 * - if PCLK2 = 96 MHz  -> divider /32 -> 3 MHz
 * - if PCLK2 = 48 MHz  -> divider /16 -> 3 MHz
 */
#ifndef SPI4_TARGET_SCK_HZ
#define SPI4_TARGET_SCK_HZ          (3000000UL)
#endif

#if   ((SPI4_PCLK2_HZ / 2UL) <= SPI4_TARGET_SCK_HZ)
    #define SPI4_BR_CODE            (0x0U)
    #define SPI4_BAUD_DIV           (2UL)
#elif ((SPI4_PCLK2_HZ / 4UL) <= SPI4_TARGET_SCK_HZ)
    #define SPI4_BR_CODE            (0x1U)
    #define SPI4_BAUD_DIV           (4UL)
#elif ((SPI4_PCLK2_HZ / 8UL) <= SPI4_TARGET_SCK_HZ)
    #define SPI4_BR_CODE            (0x2U)
    #define SPI4_BAUD_DIV           (8UL)
#elif ((SPI4_PCLK2_HZ / 16UL) <= SPI4_TARGET_SCK_HZ)
    #define SPI4_BR_CODE            (0x3U)
    #define SPI4_BAUD_DIV           (16UL)
#elif ((SPI4_PCLK2_HZ / 32UL) <= SPI4_TARGET_SCK_HZ)
    #define SPI4_BR_CODE            (0x4U)
    #define SPI4_BAUD_DIV           (32UL)
#elif ((SPI4_PCLK2_HZ / 64UL) <= SPI4_TARGET_SCK_HZ)
    #define SPI4_BR_CODE            (0x5U)
    #define SPI4_BAUD_DIV           (64UL)
#elif ((SPI4_PCLK2_HZ / 128UL) <= SPI4_TARGET_SCK_HZ)
    #define SPI4_BR_CODE            (0x6U)
    #define SPI4_BAUD_DIV           (128UL)
#else
    #define SPI4_BR_CODE            (0x7U)
    #define SPI4_BAUD_DIV           (256UL)
#endif

#define SPI4_EFFECTIVE_SCK_HZ       (SPI4_PCLK2_HZ / SPI4_BAUD_DIV)


#ifndef SPI4_DMA_CHANNEL
#define SPI4_DMA_CHANNEL       4U
#endif

#define SPI_DMA_STREAM_RX      DMA2_Stream0
#define SPI_DMA_STREAM_TX      DMA2_Stream1

#ifndef DMA_LIFCR_ALL_FLAGS_STREAM0
#define DMA_LIFCR_ALL_FLAGS_STREAM0 \
    (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0)
#endif

#ifndef DMA_LIFCR_ALL_FLAGS_STREAM1
#define DMA_LIFCR_ALL_FLAGS_STREAM1 \
    (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1)
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

#ifndef SPI_DMA_FORCE_DISABLE_SETTLE_LOOPS
#define SPI_DMA_FORCE_DISABLE_SETTLE_LOOPS 2000U
#endif

volatile spi_dma_transfer_state_t max31865_spi_dma_rx_cmplt = SPI_DMA_TRANSFER_IDLE;
volatile spi_dma_transfer_state_t max31865_spi_dma_tx_cmplt = SPI_DMA_TRANSFER_IDLE;

spi_dma_diag_t g_spi_dma_diag = {0};

static void gpio_config_spi4_pins(void);
static void dma_clear_all_flags(void);
static void spi4_log_diag_snapshot(const char *tag);
static spi_dma_status_t wait_dma_stream_disabled(volatile __IO uint32_t *stream_cr,
                                                 volatile __IO uint32_t *flag_reg,
                                                 uint32_t cr_en_mask,
                                                 uint32_t clear_mask);

/* Provides the short SPI recovery delay. */
void spi4_short_delay_ms(uint32_t ms)
{
    /* Use the configured millisecond delay path for settling. */
#ifdef systick_delay_ms
    systick_delay_ms(ms);
#elif defined(tim2_delay_ms)
    tim2_delay_ms(ms);
#else
    volatile uint32_t loops = ms * 1000U;
    while (loops--) { __asm__ volatile("nop"); }
#endif
}

#ifdef HAVE_CPU_SLEEP
#include "pwr.h"
#else
/* Waits for an interrupt during SPI polling. */
static inline void cpu_sleep_until_irq(void)
{
    /* Order register accesses before sleeping the core. */
    __DSB();
    __ISB();
    __WFI();
}
#endif

/* Publishes the SPI receive transfer state. */
static inline void set_rx_complete(spi_dma_transfer_state_t state)
{
    /* Update the RX state observed by the completion waiter. */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    max31865_spi_dma_rx_cmplt = state;
    if (!prim) {
        __enable_irq();
    }
}

/* Publishes the SPI transmit transfer state. */
static inline void set_tx_complete(spi_dma_transfer_state_t state)
{
    /* Keep the TX result separate from receive completion. */
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    max31865_spi_dma_tx_cmplt = state;
    if (!prim) {
        __enable_irq();
    }
}

/* Configures SPI4 and its receive and transmit DMA streams. */
spi_dma_status_t spi4_dma_init(void)
{
    /* Prepare both streams before allowing SPI data transfers. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    __DSB();
    __ISB();

    gpio_config_spi4_pins();

#ifdef RCC_APB2ENR_SPI4EN
    RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
#else
    RCC->APB2ENR |= (1U << 13);
#endif
    __DSB();
    __ISB();

    SPI4->CR1 = 0U;
    SPI4->CR2 = 0U;
    SPI4->SR  = 0U;

    {
        uint32_t cr1 = 0U;

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
        cr1 |= SPI_CR1_CPHA;

        cr1 &= ~SPI_CR1_BR;
        cr1 |= (SPI4_BR_CODE << SPI_CR1_BR_Pos);  

        SPI4->CR1 = cr1;

#ifdef SPI_CR2_FRF
        SPI4->CR2 &= ~SPI_CR2_FRF;
#endif
#ifdef SPI_CR2_DS
        SPI4->CR2 &= ~SPI_CR2_DS;
        SPI4->CR2 |= (0x7U << SPI_CR2_DS_Pos);
#endif
        SPI4->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
        SPI4->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    }

    SPI4->CR1 |= SPI_CR1_SPE;

#ifdef RCC_AHB1ENR_DMA2EN
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
#else
    RCC->AHB1ENR |= (1U << 22);
#endif
    __DSB();
    __ISB();

    DMA2_Stream0->CR = 0U;
    DMA2_Stream1->CR = 0U;

    if (wait_dma_stream_disabled(&DMA2_Stream0->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM0) != SPI_DMA_STATUS_OK) {
        log_printf("spi4_dma_init: stream0 did not disable cleanly\r\n");
        spi4_log_diag_snapshot("init_stream0_disable_timeout");
        tim2_delay_ms(20U);
        return SPI_DMA_STATUS_STREAM_DISABLE_TIMEOUT;
    }

    if (wait_dma_stream_disabled(&DMA2_Stream1->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM1) != SPI_DMA_STATUS_OK) {
        log_printf("spi4_dma_init: stream1 did not disable cleanly\r\n");
        spi4_log_diag_snapshot("init_stream1_disable_timeout");
        tim2_delay_ms(20U);
        return SPI_DMA_STATUS_STREAM_DISABLE_TIMEOUT;
    }

    {
        uint32_t s0cr = 0U;
        s0cr |= (SPI4_DMA_CHANNEL << 25);
        s0cr &= ~((3U << 6) | (1U << 7));
        s0cr |= (1U << 10);
        s0cr &= ~(1U << 9);
        s0cr &= ~((3U << 11) | (3U << 13));
        s0cr |= (2U << 16);
        s0cr |= DMA_SxCR_TCIE;
        DMA2_Stream0->CR = s0cr;
        DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM0;
    }

    {
        uint32_t s1cr = 0U;
        s1cr |= (SPI4_DMA_CHANNEL << 25);
        s1cr |= (1U << 6);
        s1cr |= (1U << 10);
        s1cr &= ~(1U << 9);
        s1cr &= ~((3U << 11) | (3U << 13));
        s1cr |= (2U << 16);
        s1cr |= DMA_SxCR_TCIE;
        DMA2_Stream1->CR = s1cr;
        DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM1;
    }

    {
        const uint32_t s0cr_read = DMA2_Stream0->CR;
        const uint32_t s1cr_read = DMA2_Stream1->CR;
        const uint32_t dir1 = (s1cr_read >> 6) & 0x3U;
        const uint32_t ch0  = (s0cr_read >> 25) & 0x7U;
        const uint32_t ch1  = (s1cr_read >> 25) & 0x7U;

        if (ch0 != SPI4_DMA_CHANNEL || ch1 != SPI4_DMA_CHANNEL || dir1 != 1U) {
            log_printf("spi4_dma_init: DMA configuration mismatch\r\n");
            spi4_log_diag_snapshot("init_dma_config_mismatch");
            tim2_delay_ms(20U);
            return SPI_DMA_STATUS_CONFIG_MISMATCH;
        }
    }

    NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    NVIC_SetPriority(DMA2_Stream0_IRQn, 5);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
    NVIC_SetPriority(DMA2_Stream1_IRQn, 5);
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    g_spi_dma_diag.rx_tc_count = 0U;
    g_spi_dma_diag.tx_tc_count = 0U;
    g_spi_dma_diag.rx_err_count = 0U;
    g_spi_dma_diag.tx_err_count = 0U;
    g_spi_dma_diag.rx_ht_count = 0U;
    g_spi_dma_diag.tx_ht_count = 0U;

    hb_spi_ts = 0U;
    set_rx_complete(SPI_DMA_TRANSFER_IDLE);
    set_tx_complete(SPI_DMA_TRANSFER_IDLE);

    return SPI_DMA_STATUS_OK;
}

/* Routes the SPI4 bus to its GPIO alternate functions. */
static void gpio_config_spi4_pins(void)
{
    /* Set the clock and data pins before enabling the peripheral. */
    const uint32_t pb13 = 13U;
    const uint32_t pa11 = 11U;
    const uint32_t pa1  = 1U;

    GPIOB->MODER &= ~(3U << (pb13 * 2U));
    GPIOB->MODER |=  (2U << (pb13 * 2U));
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFU << ((pb13 - 8U) * 4U))) | (SPI4_AF_SCK_MISO << ((pb13 - 8U) * 4U));
    GPIOB->OSPEEDR = (GPIOB->OSPEEDR & ~(3U << (pb13 * 2U))) | (2U << (pb13 * 2U));
    GPIOB->PUPDR &= ~(3U << (pb13 * 2U));

    GPIOA->MODER &= ~(3U << (pa11 * 2U));
    GPIOA->MODER |=  (2U << (pa11 * 2U));
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFU << ((pa11 - 8U) * 4U))) | (SPI4_AF_SCK_MISO << ((pa11 - 8U) * 4U));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (pa11 * 2U))) | (2U << (pa11 * 2U));
    GPIOA->PUPDR &= ~(3U << (pa11 * 2U));

    GPIOA->MODER &= ~(3U << (pa1 * 2U));
    GPIOA->MODER |=  (2U << (pa1 * 2U));
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFU << (pa1 * 4U))) | (SPI4_AF_MOSI_PA1 << (pa1 * 4U));
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3U << (pa1 * 2U))) | (2U << (pa1 * 2U));
    GPIOA->PUPDR &= ~(3U << (pa1 * 2U));

    __DSB();
    __ISB();
}

/* Updates the SPI4 clock prescaler. */
spi_dma_status_t spi4_set_baudrate_prescaler(uint8_t br_code)
{
    /* Disable SPI while replacing its BR field. */
    if (br_code > 7U) {
        return SPI_DMA_STATUS_INVALID_ARG;
    }

    const uint32_t was_enabled = ((SPI4->CR1 & SPI_CR1_SPE) != 0U) ? 1U : 0U;
    const uint32_t prim = __get_PRIMASK();

    __disable_irq();

    SPI4->CR1 &= ~SPI_CR1_SPE;
    __DSB();
    __ISB();

    SPI4->CR1 &= ~SPI_CR1_BR;
    SPI4->CR1 |= ((uint32_t)br_code << SPI_CR1_BR_Pos);

    if (was_enabled != 0U) {
        __DSB();
        __ISB();
        SPI4->CR1 |= SPI_CR1_SPE;
    }

    if (!prim) {
        __enable_irq();
    }

    return SPI_DMA_STATUS_OK;
}

/* Drains stale bytes from the SPI4 receive register. */
void spi4_flush_rx(void)
{
    /* Bound the RXNE drain loop to avoid an endless flush. */
    uint32_t limit = 64U;
    while (((SPI4->SR & SPI_SR_RXNE) != 0U) && (limit-- != 0U)) {
        volatile uint32_t tmp = SPI4->DR;
        (void)tmp;
        __DSB();
        __ISB();
    }
}

/* Clears a pending SPI4 receive overrun. */
void spi4_check_and_clear_ovr(void)
{
    /* Read DR before SR to acknowledge OVR. */
    if ((SPI4->SR & SPI_SR_OVR) != 0U) {
        volatile uint32_t tmp = SPI4->DR;
        (void)tmp;
        tmp = SPI4->SR;
        (void)tmp;
        __DSB();
        __ISB();
    }
}

/* Clears the SPI4 DMA stream status flags. */
static void dma_clear_all_flags(void)
{
    /* Acknowledge all pending flags for Stream0 and Stream1. */
    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM0;
    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM1;
}

/* Logs the SPI4 and DMA diagnostic register snapshot. */
static void spi4_log_diag_snapshot(const char *tag)
{
    /* Allow the UART diagnostic output to drain after reporting state. */
    log_printf("[SPI4][DIAG][%s] SR=0x%08lX CR1=0x%08lX CR2=0x%08lX LISR=0x%08lX S0CR=0x%08lX S1CR=0x%08lX RX_STATE=%u TX_STATE=%u RX_TC=%lu TX_TC=%lu RX_ERR=%lu TX_ERR=%lu\r\n",
               tag ? tag : "unknown",
               (unsigned long)SPI4->SR,
               (unsigned long)SPI4->CR1,
               (unsigned long)SPI4->CR2,
               (unsigned long)DMA2->LISR,
               (unsigned long)DMA2_Stream0->CR,
               (unsigned long)DMA2_Stream1->CR,
               (unsigned)max31865_spi_dma_rx_cmplt,
               (unsigned)max31865_spi_dma_tx_cmplt,
               (unsigned long)g_spi_dma_diag.rx_tc_count,
               (unsigned long)g_spi_dma_diag.tx_tc_count,
               (unsigned long)g_spi_dma_diag.rx_err_count,
               (unsigned long)g_spi_dma_diag.tx_err_count);
    tim2_delay_ms(2U);
}

/* Starts a SPI4 DMA transmit operation. */
spi_dma_status_t spi4_dma_transmit(uint8_t *tx_buf, uint32_t len)
{
    /* Stop Stream1 before loading the new buffer and length. */
    if ((tx_buf == NULL) || (len == 0U)) {
        log_printf("spi4_dma_transmit: invalid args\r\n");
        return SPI_DMA_STATUS_INVALID_ARG;
    }

    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    if (wait_dma_stream_disabled(&DMA2_Stream1->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM1) != SPI_DMA_STATUS_OK) {
        log_printf("spi4_dma_transmit: stream1 disable failed\r\n");
        return SPI_DMA_STATUS_STREAM_DISABLE_TIMEOUT;
    }

    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM1;

    DMA2_Stream1->PAR  = (uint32_t)&(SPI4->DR);
    DMA2_Stream1->M0AR = (uint32_t)tx_buf;
    DMA2_Stream1->NDTR = len;

    __DSB();
    __ISB();
    set_tx_complete(SPI_DMA_TRANSFER_IDLE);
    DMA2_Stream1->CR |= DMA_SxCR_EN;

    return SPI_DMA_STATUS_OK;
}

/* Arms a SPI4 DMA receive operation. */
spi_dma_status_t spi4_dma_receive(uint8_t *rx_buf, uint32_t len)
{
    /* Clear Stream0 flags before enabling its new destination. */
    if ((rx_buf == NULL) || (len == 0U)) {
        log_printf("spi4_dma_receive: invalid args\r\n");
        return SPI_DMA_STATUS_INVALID_ARG;
    }

    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    if (wait_dma_stream_disabled(&DMA2_Stream0->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM0) != SPI_DMA_STATUS_OK) {
        log_printf("spi4_dma_receive: stream0 disable failed\r\n");
        return SPI_DMA_STATUS_STREAM_DISABLE_TIMEOUT;
    }

    DMA2->LIFCR = DMA_LIFCR_ALL_FLAGS_STREAM0;

    DMA2_Stream0->PAR  = (uint32_t)&(SPI4->DR);
    DMA2_Stream0->M0AR = (uint32_t)rx_buf;
    DMA2_Stream0->NDTR = len;

    __DSB();
    __ISB();
    set_rx_complete(SPI_DMA_TRANSFER_IDLE);
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    return SPI_DMA_STATUS_OK;
}

/* Starts paired SPI4 DMA transmit and receive operations. */
spi_dma_status_t spi4_dma_transmit_receive(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    /* Enable the receive stream before the transmit stream. */
    if ((tx_buf == NULL) || (rx_buf == NULL) || (len == 0U)) {
        log_printf("spi4_dma_transmit_receive: invalid args\r\n");
        return SPI_DMA_STATUS_INVALID_ARG;
    }

    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;

    if (wait_dma_stream_disabled(&DMA2_Stream1->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM1) != SPI_DMA_STATUS_OK) {
        log_printf("spi4_dma_transmit_receive: stream1 disable failed\r\n");
        spi4_log_diag_snapshot("txrx_stream1_disable_timeout");
        return SPI_DMA_STATUS_STREAM_DISABLE_TIMEOUT;
    }

    if (wait_dma_stream_disabled(&DMA2_Stream0->CR, &DMA2->LIFCR, DMA_SxCR_EN, DMA_LIFCR_ALL_FLAGS_STREAM0) != SPI_DMA_STATUS_OK) {
        log_printf("spi4_dma_transmit_receive: stream0 disable failed\r\n");
        spi4_log_diag_snapshot("txrx_stream0_disable_timeout");
        return SPI_DMA_STATUS_STREAM_DISABLE_TIMEOUT;
    }

    dma_clear_all_flags();

    DMA2_Stream0->PAR  = (uint32_t)&(SPI4->DR);
    DMA2_Stream0->M0AR = (uint32_t)rx_buf;
    DMA2_Stream0->NDTR = len;

    DMA2_Stream1->PAR  = (uint32_t)&(SPI4->DR);
    DMA2_Stream1->M0AR = (uint32_t)tx_buf;
    DMA2_Stream1->NDTR = len;

    __DSB();
    __ISB();

    set_rx_complete(SPI_DMA_TRANSFER_IDLE);
    set_tx_complete(SPI_DMA_TRANSFER_IDLE);

    DMA2_Stream0->CR |= DMA_SxCR_EN;
    DMA2_Stream1->CR |= DMA_SxCR_EN;

    return SPI_DMA_STATUS_OK;
}

/* Waits for a DMA stream to stop before reconfiguration. */
static spi_dma_status_t wait_dma_stream_disabled(volatile __IO uint32_t *stream_cr,
                                                 volatile __IO uint32_t *flag_reg,
                                                 uint32_t cr_en_mask,
                                                 uint32_t clear_mask)
{
    /* Force disable and clear flags if the initial wait fails. */
    uint32_t short_wait = SPI_DMA_SHORT_WAIT_COUNT;

    while (((*stream_cr & cr_en_mask) != 0U) && (short_wait-- != 0U)) {
        cpu_sleep_until_irq();
    }

    if (((*stream_cr & cr_en_mask) == 0U)) {
        return SPI_DMA_STATUS_OK;
    }

    *(volatile uint32_t *)stream_cr = (*stream_cr & ~cr_en_mask);
    *flag_reg = clear_mask;

    for (volatile uint32_t i = 0U; i < SPI_DMA_FORCE_DISABLE_SETTLE_LOOPS; ++i) {
        __asm__ volatile("nop");
    }

    return (((*stream_cr & cr_en_mask) == 0U) ? SPI_DMA_STATUS_OK : SPI_DMA_STATUS_STREAM_DISABLE_TIMEOUT);
}

/* Waits for the SPI DMA result within a millisecond timeout. */
static spi_dma_status_t spi_dma_wait_complete(volatile spi_dma_transfer_state_t *state, uint32_t timeout_ms)
{
    /* Distinguish a reported transfer error from an elapsed timeout. */
    const uint32_t start_ms = ms_get();

    while (*state == SPI_DMA_TRANSFER_IDLE) {
#ifdef USE_IWDG
        IWDG_Refresh();
#endif
        cpu_sleep_until_irq();

        if ((ms_get() - start_ms) >= timeout_ms) {
            spi4_log_diag_snapshot("dma_wait_timeout");
            return SPI_DMA_STATUS_TIMEOUT;
        }
    }

    if (*state == SPI_DMA_TRANSFER_SUCCESS) {
        return SPI_DMA_STATUS_OK;
    }

    spi4_log_diag_snapshot("dma_wait_transfer_error");
    return SPI_DMA_STATUS_TRANSFER_ERROR;
}

/* Waits for SPI4 receive DMA completion. */
spi_dma_status_t spi_dma_stream0_wait_complete(uint32_t timeout_ms)
{
    /* Apply the supplied timeout to the RX completion state. */
    return spi_dma_wait_complete(&max31865_spi_dma_rx_cmplt, timeout_ms);
}

/* Waits for SPI4 transmit DMA completion. */
spi_dma_status_t spi_dma_stream1_wait_complete(uint32_t timeout_ms)
{
    /* Apply the supplied timeout to the TX completion state. */
    return spi_dma_wait_complete(&max31865_spi_dma_tx_cmplt, timeout_ms);
}

/* Consumes the shared TIM2 one-second flag. */
uint8_t tim2_1s_check_flag(void)
{
    /* Clear the flag once its elapsed interval has been observed. */
    if (tim2_1s_flag != 0U) {
        tim2_1s_flag = 0U;
        return 1U;
    }
    return 0U;
}

//void TIM2_IRQHandler(void)
//{
//    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
//        TIM2->SR &= ~TIM_SR_UIF;
//        TIM2->CR1 &= ~TIM_CR1_CEN;
//        TIM2->DIER &= ~TIM_DIER_UIE;
//        tim2_1s_flag = 1U;
//    }
//}

/* Services SPI4 receive DMA events. */
void DMA2_Stream0_IRQHandler(void)
{
    /* Clear Stream0 flags before publishing completion or error. */
    const uint32_t lisr = DMA2->LISR;

    if ((lisr & DMA_LISR_TCIF0) != 0U) {
        g_spi_dma_diag.rx_tc_count++;
        heartbeat_signal_spi();
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        __DSB();
        __ISB();
        set_rx_complete(SPI_DMA_TRANSFER_SUCCESS);
        spi4_check_and_clear_ovr();
        return;
    }

    if ((lisr & DMA_LISR_HTIF0) != 0U) {
        g_spi_dma_diag.rx_ht_count++;
        DMA2->LIFCR = DMA_LIFCR_CHTIF0;
        return;
    }

    if ((lisr & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) != 0U) {
        g_spi_dma_diag.rx_err_count++;
        DMA2->LIFCR = ((lisr & DMA_LISR_TEIF0)  ? DMA_LIFCR_CTEIF0  : 0U) |
                      ((lisr & DMA_LISR_DMEIF0) ? DMA_LIFCR_CDMEIF0 : 0U) |
                      ((lisr & DMA_LISR_FEIF0)  ? DMA_LIFCR_CFEIF0  : 0U);
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        __DSB();
        __ISB();
        set_rx_complete(SPI_DMA_TRANSFER_ERROR);
    }
}

/* Services SPI4 transmit DMA events. */
void DMA2_Stream1_IRQHandler(void)
{
    /* Stop Stream1 when its transfer result is published. */
    const uint32_t lisr = DMA2->LISR;

    if ((lisr & DMA_LISR_TCIF1) != 0U) {
        g_spi_dma_diag.tx_tc_count++;
        DMA2->LIFCR = DMA_LIFCR_CTCIF1;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        __DSB();
        __ISB();
        set_tx_complete(SPI_DMA_TRANSFER_SUCCESS);
        return;
    }

    if ((lisr & DMA_LISR_HTIF1) != 0U) {
        g_spi_dma_diag.tx_ht_count++;
        DMA2->LIFCR = DMA_LIFCR_CHTIF1;
        return;
    }

    if ((lisr & (DMA_LISR_TEIF1 | DMA_LISR_DMEIF1 | DMA_LISR_FEIF1)) != 0U) {
        g_spi_dma_diag.tx_err_count++;
        DMA2->LIFCR = ((lisr & DMA_LISR_TEIF1)  ? DMA_LIFCR_CTEIF1  : 0U) |
                      ((lisr & DMA_LISR_DMEIF1) ? DMA_LIFCR_CDMEIF1 : 0U) |
                      ((lisr & DMA_LISR_FEIF1)  ? DMA_LIFCR_CFEIF1  : 0U);
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        __DSB();
        __ISB();
        set_tx_complete(SPI_DMA_TRANSFER_ERROR);
    }
}

/* Provides the overridable SPI4 device reinitialization hook. */
__attribute__((weak)) int spi4_device_reinit(void)
{
    /* The weak default returns success without changing the device. */
    return 0;
}

/* Provides the overridable SPI4 recovery self-test. */
__attribute__((weak)) int spi4_selftest(void)
{
    /* The default stub accepts recovery without a device transaction. */
    return 0;
}

/* Restores SPI4 after a stalled or failed DMA operation. */
spi_dma_status_t spi4_recover(void)
{
    /* Stop both DMA streams before escalating to an APB reset. */
    log_printf("spi4_recover: start\r\n");

    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    __DSB();
    __ISB();
    dma_clear_all_flags();

    spi4_short_delay_ms(2U);
    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    SPI4->CR1 &= ~SPI_CR1_SPE;
    __DSB();
    __ISB();

    {
        uint32_t timeout = 1000U;
        while (((SPI4->SR & SPI_SR_BSY) != 0U) && (timeout-- != 0U)) {
            __asm__ volatile("nop");
        }
    }

    if (((DMA2_Stream0->CR & DMA_SxCR_EN) == 0U) &&
        ((DMA2_Stream1->CR & DMA_SxCR_EN) == 0U) &&
        ((SPI4->SR & SPI_SR_BSY) == 0U)) {
        log_printf("spi4_recover: graceful disable succeeded\r\n");
    } else {
        log_printf("spi4_recover: graceful disable failed -> toggling SPI APB reset\r\n");
        spi4_log_diag_snapshot("recover_graceful_disable_failed");
#ifdef RCC_APB2RSTR_SPI4RST
        RCC->APB2RSTR |= RCC_APB2RSTR_SPI4RST;
        spi4_short_delay_ms(1U);
        RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI4RST;
#else
        RCC->APB2RSTR |= (1U << 13);
        spi4_short_delay_ms(1U);
        RCC->APB2RSTR &= ~(1U << 13);
#endif
        __DSB();
        __ISB();

        dma_clear_all_flags();
        DMA2_Stream0->CR = 0U;
        DMA2_Stream1->CR = 0U;
        spi4_short_delay_ms(2U);
    }

    dma_clear_all_flags();
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    __DSB();
    __ISB();

    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    SPI4->CR1 |= SPI_CR1_SPE;
    __DSB();
    __ISB();

    if (spi4_device_reinit() != 0) {
        log_printf("spi4_recover: spi4_device_reinit() failed\r\n");
        spi4_log_diag_snapshot("recover_device_reinit_failed");
        return SPI_DMA_STATUS_DEVICE_REINIT_FAILED;
    }

    if (spi4_selftest() != 0) {
        log_printf("spi4_recover: spi4_selftest() failed\r\n");
        spi4_log_diag_snapshot("recover_selftest_failed");
        return SPI_DMA_STATUS_SELFTEST_FAILED;
    }

    log_printf("spi4_recover: lightweight restore OK\r\n");
    return SPI_DMA_STATUS_OK;
}

/* Reapplies the SPI4 and DMA configuration after recovery. */
spi_dma_status_t spi4_restore_and_init(void)
{
    /* Flush stale receive data before returning with the streams stopped. */
    log_printf("spi4_restore_and_init: starting full restore/init\r\n");

    if (spi4_dma_init() != SPI_DMA_STATUS_OK) {
        log_printf("spi4_restore_and_init: spi4_dma_init() failed\r\n");
        spi4_log_diag_snapshot("restore_spi4_dma_init_failed");
        return SPI_DMA_STATUS_INIT_FAILED;
    }

    spi4_short_delay_ms(2U);
    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    __DSB();
    __ISB();

    log_printf("spi4_restore_and_init: restore & init completed OK\r\n");
    return SPI_DMA_STATUS_OK;
}

/* Waits for SPI4 to finish shifting data. */
spi_dma_status_t spi4_wait_bsy_clear_ms(uint32_t timeout_ms)
{
    /* Reject a disabled SPI peripheral and bound the BSY wait. */
    const uint32_t start_ms = ms_get();

    if ((SPI4->CR1 & SPI_CR1_SPE) == 0U) {
        return SPI_DMA_STATUS_INVALID_ARG;
    }

    while ((SPI4->SR & SPI_SR_BSY) != 0U) {
#ifdef USE_IWDG
        IWDG_Refresh();
#endif
        cpu_sleep_until_irq();

        if ((ms_get() - start_ms) >= timeout_ms) {
            return SPI_DMA_STATUS_TIMEOUT;
        }
    }

    return SPI_DMA_STATUS_OK;
}
