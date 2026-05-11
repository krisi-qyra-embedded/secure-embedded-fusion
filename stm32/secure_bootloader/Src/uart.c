/* Author: i fiksuari pas prog | Documentation date: 11 May 2026 */
#include "uart.h"

/* ---------- ring buffer state (single producer, single consumer) ---------- */
uint8_t log_buf[LOG_BUF_SIZE];
volatile uint32_t log_head = 0; /* producer index (next write) */
volatile uint32_t log_tail = 0; /* consumer index (next read) */
const uint32_t LOG_MASK = LOG_BUF_SIZE - 1; /* only valid when LOG_BUF_SIZE power-of-two */


void uart2_tx_rx_init(void)
{
	RCC->AHB1ENR |= (1U << 0);
	GPIOA->MODER &=~(1U << 4);
	GPIOA->MODER |= (1U << 5);
	GPIOA->AFR[0]  |= (1U << 8);
	GPIOA->AFR[0]  |= (1U << 9);
	GPIOA->AFR[0]  |= (1U << 10);
	GPIOA->AFR[0]  &=~(1U << 11);
	GPIOA->MODER &=~(1U << 6);
	GPIOA->MODER |= (1U << 7);
	GPIOA->AFR[0]  |= (1U << 12);
	GPIOA->AFR[0]  |= (1U << 13);
	GPIOA->AFR[0]  |= (1U << 14);
	GPIOA->AFR[0]  &=~(1U << 15);
	RCC->APB1ENR |= (1U << 17);
	uart_set_baudrate(USART2,periphclck,baudrate);
	USART2->CR1 |= ((1U << 3) | (1U << 2));
	USART2->CR1 |= (1U << 5);
	/* set NVIC priority and enable IRQ for USART2
	Important: set numerically larger priority than I2C/DMA IRQs so logging never delays I2C */
	NVIC_SetPriority(USART2_IRQn, 3); /* choose 3 or adapt to your system */
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->CR1 |= (1U << 13);
	/* init ring buffer indices */
	log_head = log_tail = 0;
}


/* Handles debug UART output or transport state. */
void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Check the transport state before writing another message. */
    USARTx->BRR = compute_uart_div(PeriphClk, BaudRate);
}

/* Handles debug UART output or transport state. */
uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate)
{
    return ((PeriphClk + (BaudRate / 2U)) / BaudRate);
//  past error : return ((PeriphClk + (BaudRate/2U)/BaudRate));
}


/* ---------- non-blocking ring buffer writer ---------- */
void log_write_bytes(const uint8_t *data, uint32_t len)
{
    if (len == 0) return;

    /* quick check: drop message if not enough space (non-blocking) */
    if (len > log_free()) return;

    /* copy possibly in two parts */
    uint32_t idx = log_head & LOG_MASK;
    uint32_t first = LOG_BUF_SIZE - idx;
    if (first >= len) {
        memcpy(&log_buf[idx], data, len);
    } else {
        memcpy(&log_buf[idx], data, first);
        memcpy(&log_buf[0], data + first, len - first);
    }

    /* publish head under critical section to avoid race with IRQ consumer */
    __disable_irq();
    log_head = (log_head + len) & LOG_MASK;
    __enable_irq();

    /* kick TX if idle: write first byte directly and enable TXE interrupt */
    if (!(USART2->CR1 & (1U << 7))) { /* TXEIE not enabled => transmitter idle for our buffer */
        if (log_tail != log_head) {
            uint8_t c = log_buf[log_tail & LOG_MASK];
            log_tail = (log_tail + 1) & LOG_MASK;
            USART2->DR = c;                 /* start first byte transmission */
            USART2->CR1 |= (1U << 7);       /* enable TXE interrupt (bit 7) */
        }
    }
}

/* ---------- public logging API ---------- */

/* fast string copy (non-blocking). OK to call from ISRs but avoid long strings. */
void log_puts(const char *s)
{
    /* Check the transport state before writing another message. */
    log_write_bytes((const uint8_t *)s, (uint32_t)strlen(s));
}


/* single char helper */
void log_putc(char c)
{
    /* Check the transport state before writing another message. */
    log_write_bytes((const uint8_t *)&c, 1);
}


/* printf-style logger - uses temporary buffer (not ISR-safe) */
void log_printf(const char *fmt, ...)
{
    /* Check the transport state before writing another message. */
    char tmp[LOG_SCRATCH_SZ];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n > (int)(sizeof(tmp) - 1)) n = (int)(sizeof(tmp) - 1);
        log_write_bytes((const uint8_t *)tmp, (uint32_t)n);
    }
}


/* ---------- USART2 IRQ handler helper (TXE & RXNE) ---------- */
void uart_log_irq_handler(void)
{
    /* Handle RXNE first (read clears RXNE) */
    if (USART2->SR & (1U << 5)) { /* RXNE */
        char received = (char)(USART2->DR & 0xFF); /* read data -> clears RXNE */
        /* Optional: echo back (your original logic) - but keep light */
        /* If you want to echo only, use log_putc or write direct to DR if TX idle */
        /* Echo back if transmitter idle - else drop echo (avoid blocking) */
        if (!(USART2->CR1 & (1U << 7))) { /* TXEIE not enabled -> idle */
            USART2->DR = (uint8_t)received;
        } else {
            /* enqueue echo as a log char (non-blocking) */
            log_putc(received);
        }
    }

    /* Handle TXE: Data register empty, ready for next byte */
    if (USART2->SR & (1U << 7)) { /* TXE flag is bit 7 in SR on many MCUs */
        if (log_tail != log_head) {
            uint8_t c = log_buf[log_tail & LOG_MASK];
            log_tail = (log_tail + 1) & LOG_MASK;
            USART2->DR = c;
        } else {
            /* No more data -> disable TXE interrupt to stop further IRQs */
            USART2->CR1 &= ~(1U << 7);
        }
    }

    /* Note: keep IRQ handler minimal. Do not call heavy libs here. */
}


/* ---------- actual IRQ vector (weak symbol typically in startup file) ---------- */
void USART2_IRQHandler(void)
{
    /* Clear the handled interrupt flags and update the transfer state. */
    uart_log_irq_handler();
}


///* ---------- small convenience wrappers you had declared ---------- */
//void uart2_transmit(int ch)
//{
//    /* blocking transmit (rarely used). Avoid in hot paths. */
//    while (!(USART2->SR & (1U << 7))) { __asm__ volatile ("nop"); } /* wait TXE */
//    USART2->DR = (uint8_t)ch;
//    /* wait until TC (transmission complete) optionally */
//    while (!(USART2->SR & (1U << 6))) { __asm__ volatile ("nop"); } /* wait TC */
//}
//
//char uart2_read(void)
//{
//    /* blocking read - wait until RXNE */
//    while (!(USART2->SR & (1U << 5)));
//    return (char)(USART2->DR & 0xFF);
//}
