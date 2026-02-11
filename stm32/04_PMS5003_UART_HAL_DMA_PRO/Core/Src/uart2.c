/* uart2.c | Author: i fiksuari pas prog | Date: 11 February 2026 */
#include "uart2.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*
 * Assumption:
 * USART2 peripheral clock really is 96,000,000 Hz.
 *
 * If your MCU runs at 96 MHz system clock but APB1 is divided
 * (for example PCLK1 = 48 MHz), then change this define to the
 * real USART2 peripheral clock.
 */
#define USART2_PCLK_HZ   96000000UL

/* ---------- ring buffer state (single producer, single consumer) ---------- */
uint8_t log_buf[LOG_BUF_SIZE];
volatile uint32_t log_head = 0U; /* producer index */
volatile uint32_t log_tail = 0U; /* consumer index */
const uint32_t LOG_MASK = (LOG_BUF_SIZE - 1U); /* LOG_BUF_SIZE must be power-of-two */

static uint16_t compute_uart_div(uint32_t periph_clk, uint32_t baudrate_hz);
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periph_clk, uint32_t baudrate_hz);

/* ---------- init ---------- */
void uart2_tx_rx_init(void)
{
    /* GPIOA clock enable */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* USART2 clock enable */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /*
     * PA2 = USART2_TX
     * PA3 = USART2_RX
     * Alternate function AF7
     */

    /* PA2 -> AF */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (2U << (2U * 2U));

    /* PA3 -> AF */
    GPIOA->MODER &= ~(3U << (3U * 2U));
    GPIOA->MODER |=  (2U << (3U * 2U));

    /* AF7 for PA2 */
    GPIOA->AFR[0] &= ~(0xFU << (4U * 2U));
    GPIOA->AFR[0] |=  (7U   << (4U * 2U));

    /* AF7 for PA3 */
    GPIOA->AFR[0] &= ~(0xFU << (4U * 3U));
    GPIOA->AFR[0] |=  (7U   << (4U * 3U));

    /* Optional: no pull-up/pull-down */
    GPIOA->PUPDR &= ~(3U << (2U * 2U));
    GPIOA->PUPDR &= ~(3U << (3U * 2U));

    /* Configure baud rate from real USART2 peripheral clock */
    uart_set_baudrate(USART2, USART2_PCLK_HZ, baudrate);

    /* Enable TX and RX */
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

    /* Enable RXNE interrupt */
    USART2->CR1 |= USART_CR1_RXNEIE;

    /* USART enable */
    USART2->CR1 |= USART_CR1_UE;

    /* NVIC priority and enable IRQ for USART2 */
    NVIC_SetPriority(USART2_IRQn, 3U);
    NVIC_EnableIRQ(USART2_IRQn);

    /* init ring buffer indices */
    log_head = 0U;
    log_tail = 0U;
}

/* ---------- baudrate helpers ---------- */
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periph_clk, uint32_t baudrate_hz)
{
    /* Use the configured peripheral clock and requested baud rate. */
    USARTx->BRR = compute_uart_div(periph_clk, baudrate_hz);
}

/* Calculate or program the UART timing in compute_uart_div. */
static uint16_t compute_uart_div(uint32_t periph_clk, uint32_t baudrate_hz)
{
    /* Oversampling by 16, rounded integer divider */
    return (uint16_t)((periph_clk + (baudrate_hz / 2U)) / baudrate_hz);
}

/* ---------- ring buffer helpers ---------- */
uint32_t log_count(void)
{
    /* Validate the current state before continuing. */
    return ((log_head - log_tail) & LOG_MASK);
}

/* Process the log_free operation. */
uint32_t log_free(void)
{
    /* Validate the current state before continuing. */
    return ((LOG_BUF_SIZE - 1U) - log_count());
}

/* ---------- non-blocking ring buffer writer ---------- */
void log_write_bytes(const uint8_t *data, uint32_t len)
{
    /* Check the transmit state before finishing this operation. */
    uint32_t idx;
    uint32_t first;

    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    /* Drop message if not enough space */
    if (len > log_free())
    {
        return;
    }

    idx = (log_head & LOG_MASK);
    first = LOG_BUF_SIZE - idx;

    if (first >= len)
    {
        memcpy(&log_buf[idx], data, len);
    }
    else
    {
        memcpy(&log_buf[idx], data, first);
        memcpy(&log_buf[0], data + first, len - first);
    }

    __disable_irq();
    log_head = (log_head + len) & LOG_MASK;
    __enable_irq();

    /* Kick TX if idle */
    if ((USART2->CR1 & USART_CR1_TXEIE) == 0U)
    {
        if (log_tail != log_head)
        {
            uint8_t c = log_buf[log_tail & LOG_MASK];
            log_tail = (log_tail + 1U) & LOG_MASK;
            USART2->DR = c;
            USART2->CR1 |= USART_CR1_TXEIE;
        }
    }
}

/* ---------- public logging API ---------- */
void log_puts(const char *s)
{
    /* Validate the current state before continuing. */
    if (s == NULL)
    {
        return;
    }

    log_write_bytes((const uint8_t *)s, (uint32_t)strlen(s));
}

/* Process the log_putc operation. */
void log_putc(char c)
{
    /* Validate the current state before continuing. */
    log_write_bytes((const uint8_t *)&c, 1U);
}

/* Process the log_printf operation. */
void log_printf(const char *fmt, ...)
{
    /* Validate the current state before continuing. */
    char tmp[LOG_SCRATCH_SZ];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    if (n > 0)
    {
        if (n > (int)(sizeof(tmp) - 1))
        {
            n = (int)(sizeof(tmp) - 1);
        }

        log_write_bytes((const uint8_t *)tmp, (uint32_t)n);
    }
}

/* ---------- USART2 IRQ handler helper ---------- */
void uart_log_irq_handler(void)
{
    /* RXNE: received one byte */
    if ((USART2->SR & USART_SR_RXNE) != 0U)
    {
        char received = (char)(USART2->DR & 0xFFU);

        /* Optional echo */
        if ((USART2->CR1 & USART_CR1_TXEIE) == 0U)
        {
            USART2->DR = (uint8_t)received;
        }
        else
        {
            log_putc(received);
        }
    }

    /* TXE: send next byte from ring buffer */
    if ((USART2->SR & USART_SR_TXE) != 0U)
    {
        if (log_tail != log_head)
        {
            uint8_t c = log_buf[log_tail & LOG_MASK];
            log_tail = (log_tail + 1U) & LOG_MASK;
            USART2->DR = c;
        }
        else
        {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ---------- IRQ vector ---------- */
void USART2_IRQHandler(void)
{
    /* Clear the handled event and update the transfer state. */
    uart_log_irq_handler();
}
