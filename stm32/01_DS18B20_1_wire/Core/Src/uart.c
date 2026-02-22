/* uart.c | Author: i fiksuari pas prog | Date: 22 February 2026 */
#include "uart.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* ---------- ring buffer state (single producer, single consumer) ---------- */
uint8_t log_buf[LOG_BUF_SIZE];
volatile uint32_t log_head = 0U; /* producer index (next write) */
volatile uint32_t log_tail = 0U; /* consumer index (next read) */
const uint32_t LOG_MASK = LOG_BUF_SIZE - 1U; /* valid when LOG_BUF_SIZE is power-of-two */

/* USART2 is on APB1 on STM32F4 */
static uint32_t uart2_get_periph_clock(void)
{
    /* Use the APB1 clock for the USART2 divider. */
    return HAL_RCC_GetPCLK1Freq();
}

/* Configures GPIO and USART2 for logging. */
void uart2_tx_rx_init(void)
{
    uint32_t periphclk;

    /* Enable GPIOA clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    /* PA2 -> USART2_TX, PA3 -> USART2_RX */
    GPIOA->MODER &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->MODER |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));   /* AF mode */

    GPIOA->OTYPER &= ~((1U << 2U) | (1U << 3U));                /* push-pull */
    GPIOA->OSPEEDR |= ((2U << (2U * 2U)) | (2U << (3U * 2U)));  /* fast */
    GPIOA->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->PUPDR |=  ((1U << (2U * 2U)) | (1U << (3U * 2U)));   /* pull-up */

    GPIOA->AFR[0] &= ~((0xFU << (4U * 2U)) | (0xFU << (4U * 3U)));
    GPIOA->AFR[0] |=  ((7U   << (4U * 2U)) | (7U   << (4U * 3U))); /* AF7 */

    /* Enable USART2 clock */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    /* Disable USART before configuration */
    USART2->CR1 &= ~USART_CR1_UE;

    /* Use the real APB1 clock derived from the current 96 MHz clock tree */
    periphclk = uart2_get_periph_clock();
    uart_set_baudrate(USART2, periphclk, baudrate);

    /* 8N1, oversampling by 16 (default) */
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;

    /* Enable TX, RX, RXNE interrupt */
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* NVIC config */
    NVIC_SetPriority(USART2_IRQn, 3U);
    NVIC_EnableIRQ(USART2_IRQn);

    /* Enable USART */
    USART2->CR1 |= USART_CR1_UE;

    /* init ring buffer indices */
    log_head = 0U;
    log_tail = 0U;
}

/* Programs the UART baud-rate divisor. */
void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Apply the divisor calculated for the requested baud rate. */
    USARTx->BRR = compute_uart_div(PeriphClk, BaudRate);
}

/* Calculates the UART divisor from the peripheral clock. */
uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate)
{
    /* For OVER8 = 0, BRR = round(PeriphClk / BaudRate) */
    return (uint16_t)((PeriphClk + (BaudRate / 2U)) / BaudRate);
}

/* ---------- non-blocking ring buffer writer ---------- */
void log_write_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t idx;
    uint32_t first;

    if ((data == NULL) || (len == 0U)) {
        return;
    }

    /* quick check: drop message if not enough space (non-blocking) */
    if (len > log_free()) {
        return;
    }

    idx = log_head & LOG_MASK;
    first = LOG_BUF_SIZE - idx;

    if (first >= len) {
        memcpy(&log_buf[idx], data, len);
    } else {
        memcpy(&log_buf[idx], data, first);
        memcpy(&log_buf[0], data + first, len - first);
    }

    __disable_irq();
    log_head = (log_head + len) & LOG_MASK;
    __enable_irq();

    /* Kick TX if idle */
    if ((USART2->CR1 & USART_CR1_TXEIE) == 0U) {
        if (log_tail != log_head) {
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
    /* Forward the message bytes to the UART log buffer. */
    if (s == NULL) {
        return;
    }
    log_write_bytes((const uint8_t *)s, (uint32_t)strlen(s));
}

/* Writes a single character to the logging interface. */
void log_putc(char c)
{
    /* Queue the character without blocking the sensor path. */
    log_write_bytes((const uint8_t *)&c, 1U);
}

/* Formats and queues a log message. */
void log_printf(const char *fmt, ...)
{
    /* Bound formatted output to the temporary message buffer. */
    char tmp[LOG_SCRATCH_SZ];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)(sizeof(tmp) - 1)) {
            n = (int)(sizeof(tmp) - 1);
        }
        log_write_bytes((const uint8_t *)tmp, (uint32_t)n);
    }
}

/* ---------- USART2 IRQ handler helper (TXE & RXNE) ---------- */
void uart_log_irq_handler(void)
{
    /* RXNE */
    if ((USART2->SR & USART_SR_RXNE) != 0U) {
        char received = (char)(USART2->DR & 0xFFU);

        if ((USART2->CR1 & USART_CR1_TXEIE) == 0U) {
            USART2->DR = (uint8_t)received;
        } else {
            log_putc(received);
        }
    }

    /* TXE */
    if ((USART2->SR & USART_SR_TXE) != 0U) {
        if (log_tail != log_head) {
            uint8_t c = log_buf[log_tail & LOG_MASK];
            log_tail = (log_tail + 1U) & LOG_MASK;
            USART2->DR = c;
        } else {
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ---------- actual IRQ vector ---------- */
void USART2_IRQHandler(void)
{
    /* Service the USART2 transmit interrupt. */
    uart_log_irq_handler();
}
