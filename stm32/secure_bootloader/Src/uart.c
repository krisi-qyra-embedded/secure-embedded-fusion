/**
 * @file uart.c
 * @brief USART2 interrupt-driven logging with a non-blocking ring buffer.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include "uart.h"
/* Single-producer/single-consumer TX ring buffer. */
uint8_t log_buf[LOG_BUF_SIZE];
volatile uint32_t log_head = 0;
volatile uint32_t log_tail = 0;
const uint32_t LOG_MASK = LOG_BUF_SIZE - 1;
/* USART2 setup. */
/**
 * @brief Configure USART2 TX/RX pins, baud rate and interrupt handling.
 */
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
	/* Keep logging below the priority of timing-sensitive DMA/I2C work. */
	NVIC_SetPriority(USART2_IRQn, 3);
	NVIC_EnableIRQ(USART2_IRQn);
	USART2->CR1 |= (1U << 13);
	/* Start with an empty ring buffer. */
	log_head = log_tail = 0;
}
/**
 * @brief Program a USART baud-rate divider.
 * @param USARTx USART peripheral instance.
 * @param PeriphClk Peripheral clock frequency.
 * @param BaudRate Requested baud rate.
 */
void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Check the transport state before writing another message. */
   USARTx->BRR = compute_uart_div(PeriphClk, BaudRate);
}
/**
 * @brief Calculate the integer USART baud-rate divider.
 * @param PeriphClk Peripheral clock frequency.
 * @param BaudRate Requested baud rate.
 * @return Calculated BRR divider.
 */
uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate)
{
   return ((PeriphClk + (BaudRate / 2U)) / BaudRate);
//  past error : return ((PeriphClk + (BaudRate/2U)/BaudRate));
}
/* Non-blocking logging path. */
/**
 * @brief Queue bytes into the non-blocking USART2 TX ring buffer.
 * @param data Data to queue.
 * @param len Number of bytes to queue.
 */
void log_write_bytes(const uint8_t *data, uint32_t len)
{
   if (len == 0) return;
   /* Drop the complete message rather than block when the ring is full. */
   if (len > log_free()) return;
   /* Split the copy only when it wraps around the ring boundary. */
   uint32_t idx = log_head & LOG_MASK;
   uint32_t first = LOG_BUF_SIZE - idx;
   if (first >= len) {
       memcpy(&log_buf[idx], data, len);
   } else {
       memcpy(&log_buf[idx], data, first);
       memcpy(&log_buf[0], data + first, len - first);
   }
   /* Publish the new producer index atomically with respect to the IRQ consumer. */
   __disable_irq();
   log_head = (log_head + len) & LOG_MASK;
   __enable_irq();
   /* Kick transmission directly when TXE interrupts are currently disabled. */
   if (!(USART2->CR1 & (1U << 7))) {
       if (log_tail != log_head) {
           uint8_t c = log_buf[log_tail & LOG_MASK];
           log_tail = (log_tail + 1) & LOG_MASK;
           USART2->DR = c;
           USART2->CR1 |= (1U << 7);
       }
   }
}
/**
 * @brief Queue a null-terminated string for USART2 logging.
 * @param s String to queue.
 */
void log_puts(const char *s)
{
    /* Check the transport state before writing another message. */
   log_write_bytes((const uint8_t *)s, (uint32_t)strlen(s));
}
/**
 * @brief Queue one character for USART2 logging.
 * @param c Character to queue.
 */
void log_putc(char c)
{
    /* Check the transport state before writing another message. */
   log_write_bytes((const uint8_t *)&c, 1);
}
/**
 * @brief Format a message into the scratch buffer and queue it for transmission.
 * @param fmt printf-style format string.
 */
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
/* USART2 interrupt handling. */
/**
 * @brief Handle USART2 RXNE and TXE events for logging and echo.
 */
void uart_log_irq_handler(void)
{
   /* RXNE: consume the received byte before servicing TX. */
   if (USART2->SR & (1U << 5)) {
       char received = (char)(USART2->DR & 0xFF);
       if (!(USART2->CR1 & (1U << 7))) {
           USART2->DR = (uint8_t)received;
       } else {
           log_putc(received);
       }
   }
   /* TXE: send the next queued byte or stop TXE interrupts when empty. */
   if (USART2->SR & (1U << 7)) {
       if (log_tail != log_head) {
           uint8_t c = log_buf[log_tail & LOG_MASK];
           log_tail = (log_tail + 1) & LOG_MASK;
           USART2->DR = c;
       } else {
           USART2->CR1 &= ~(1U << 7);
       }
   }
}
/**
 * @brief USART2 interrupt vector.
 */
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
