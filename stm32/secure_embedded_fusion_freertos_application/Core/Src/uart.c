/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "uart.h"

/*
 * USART2 is on APB1.
 * For a typical STM32F411 96 MHz system-clock configuration:
 *   SYSCLK = 96 MHz
 *   APB1   = 48 MHz
 * So the USART2 baud-rate generator must use 48 MHz unless your RCC tree says otherwise.
 */
#ifndef UART2_PERIPH_CLK_HZ
#define UART2_PERIPH_CLK_HZ    (48000000UL)
#endif

/* ---------- ring buffer state (single producer, single consumer) ---------- */
uint8_t log_buf[LOG_BUF_SIZE];
volatile uint32_t log_head = 0U;  
volatile uint32_t log_tail = 0U;  
const uint32_t LOG_MASK = LOG_BUF_SIZE - 1U;  

/* Routes USART2 to PA2 and PA3. */
static void uart2_gpio_init(void)
{
    /* Select AF7 with pull-ups for both UART pins. */
    /* PA2 = USART2_TX (AF7), PA3 = USART2_RX (AF7) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    GPIOA->MODER &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->MODER |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));

    GPIOA->OTYPER &= ~((1U << 2U) | (1U << 3U));

    GPIOA->OSPEEDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->OSPEEDR |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));

    GPIOA->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->PUPDR |=  ((1U << (2U * 2U)) | (1U << (3U * 2U)));

    GPIOA->AFR[0] &= ~((0xFU << (2U * 4U)) | (0xFU << (3U * 4U)));
    GPIOA->AFR[0] |=  ((7U   << (2U * 4U)) | (7U   << (3U * 4U)));
}

/* Starts USART2 interrupt-driven debug logging. */
void uart2_tx_rx_init(void)
{
    /* Reset the log-ring indices after enabling the UART and IRQ. */
    uart2_gpio_init();

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    USART2->CR1 &= ~USART_CR1_UE;

    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;

    uart_set_baudrate(USART2, UART2_PERIPH_CLK_HZ, UART2_BAUDRATE);

    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);

    NVIC_SetPriority(USART2_IRQn, 3U);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);

    USART2->CR1 |= USART_CR1_UE;

    log_head = 0U;
    log_tail = 0U;
}

/* Programs the USART baud divisor. */
void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Write the clock-derived divisor to BRR. */
    USARTx->BRR = compute_uart_div(PeriphClk, BaudRate);
}

/* Calculates the rounded USART BRR value. */
uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Round the peripheral-clock quotient to the nearest divisor. */
    return (uint16_t)((PeriphClk + (BaudRate / 2U)) / BaudRate);
}

/* ---------- non-blocking ring buffer writer ---------- */
/* Queues a complete byte sequence for UART logging. */
void log_write_bytes(const uint8_t *data, uint32_t len)
{
    /* Drop the write if the ring lacks space, then start TXE draining when idle. */
    if ((data == NULL) || (len == 0U)) {
        return;
    }

    if (len > log_free()) {
        return;
    }

    {
        uint32_t idx = log_head & LOG_MASK;
        uint32_t first = LOG_BUF_SIZE - idx;

        if (first >= len) {
            memcpy(&log_buf[idx], data, len);
        } else {
            memcpy(&log_buf[idx], data, first);
            memcpy(&log_buf[0], data + first, len - first);
        }
    }

    __disable_irq();
    log_head = (log_head + len) & LOG_MASK;
    __enable_irq();

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
/* Queues a null-terminated log string. */
void log_puts(const char *s)
{
    /* Pass the string length to the ring-buffer writer. */
    if (s == NULL) {
        return;
    }

    log_write_bytes((const uint8_t *)s, (uint32_t)strlen(s));
}

/* Queues a single log character. */
void log_putc(char c)
{
    /* Copy the character through the same bounded ring-buffer path. */
    log_write_bytes((const uint8_t *)&c, 1U);
}

/* Formats a message into the UART log queue. */
void log_printf(const char *fmt, ...)
{
    /* Clamp truncated output to the scratch buffer's valid bytes. */
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
/* Services USART2 receive and transmit events. */
void uart_log_irq_handler(void)
{
    /* Disable TXE interrupts once the queued output is empty. */
    if ((USART2->SR & USART_SR_RXNE) != 0U) {
        char received = (char)(USART2->DR & 0xFFU);

        if ((USART2->CR1 & USART_CR1_TXEIE) == 0U) {
            USART2->DR = (uint8_t)received;
        } else {
            log_putc(received);
        }
    }

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

/* Dispatches USART2 events to the log handler. */
void USART2_IRQHandler(void)
{
    /* Keep receive echo and transmit draining in the shared IRQ helper. */
    uart_log_irq_handler();
}
