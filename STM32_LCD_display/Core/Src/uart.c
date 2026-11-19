/**
 * @file uart.c
 * @brief USART2 logging, ring-buffer TX and RX/TX interrupt handling.
 */
#include "uart.h"
/* USART2 runs from APB1 in this clock setup. */
#ifndef UART2_PERIPH_CLK_HZ
#define UART2_PERIPH_CLK_HZ    (48000000UL)
#endif
uint8_t log_buf[LOG_BUF_SIZE];
volatile uint32_t log_head = 0U;
volatile uint32_t log_tail = 0U;
const uint32_t LOG_MASK = LOG_BUF_SIZE - 1U;
/* Configure PA2/PA3 for USART2 AF7. */
static void uart2_gpio_init(void)
{
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
   (void)RCC->AHB1ENR;
   /* PA2/PA3 use alternate-function mode. */
   GPIOA->MODER &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
   GPIOA->MODER |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));
   GPIOA->OTYPER &= ~((1U << 2U) | (1U << 3U));
   GPIOA->OSPEEDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
   GPIOA->OSPEEDR |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));
   GPIOA->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
   GPIOA->PUPDR |=  ((1U << (2U * 2U)) | (1U << (3U * 2U)));
   /* AF7 routes both pins to USART2. */
   GPIOA->AFR[0] &= ~((0xFU << (2U * 4U)) | (0xFU << (3U * 4U)));
   GPIOA->AFR[0] |=  ((7U   << (2U * 4U)) | (7U   << (3U * 4U)));
}
/**
 * @brief Initialize USART2 and the logging ring buffer.
 */
void uart2_tx_rx_init(void)
{
   uart2_gpio_init();
   RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
   (void)RCC->APB1ENR;
   /* Configure USART only while UE is disabled. */
   USART2->CR1 &= ~USART_CR1_UE;
   USART2->CR1 = 0U;
   USART2->CR2 = 0U;
   USART2->CR3 = 0U;
   uart_set_baudrate(USART2, UART2_PERIPH_CLK_HZ, UART2_BAUDRATE);
   /* Enable TX, RX and RXNE interrupt handling. */
   USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
   /* Logging stays below the more critical sensor/DMA IRQs. */
   NVIC_SetPriority(USART2_IRQn, 3U);
   NVIC_ClearPendingIRQ(USART2_IRQn);
   NVIC_EnableIRQ(USART2_IRQn);
   USART2->CR1 |= USART_CR1_UE;
   /* Ring buffer starts empty. */
   log_head = 0U;
   log_tail = 0U;
}
/**
 * @brief Program a USART baud-rate divider.
 * @param USARTx USART peripheral.
 * @param PeriphClk Peripheral clock in hertz.
 * @param BaudRate Requested baud rate.
 */
void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
   USARTx->BRR = compute_uart_div(PeriphClk, BaudRate);
}
/**
 * @brief Calculate the USART baud-rate divider.
 * @param PeriphClk Peripheral clock in hertz.
 * @param BaudRate Requested baud rate.
 * @return BRR divider value.
 */
uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate)
{
   return (uint16_t)((PeriphClk + (BaudRate / 2U)) / BaudRate);
}
/**
 * @brief Queue bytes for non-blocking USART2 transmission.
 * @param data Source bytes.
 * @param len Number of bytes.
 */
void log_write_bytes(const uint8_t *data, uint32_t len)
{
   if ((data == NULL) || (len == 0U)) {
       return;
   }
   /* Drop the whole message if the ring buffer cannot hold it. */
   if (len > log_free()) {
       return;
   }
   /* Copy once or wrap at the end of the ring buffer. */
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
   /* Publish the new head atomically against the IRQ consumer. */
   __disable_irq();
   log_head = (log_head + len) & LOG_MASK;
   __enable_irq();
   /* If TX is idle, send the first byte and let TXE continue the rest. */
   if ((USART2->CR1 & USART_CR1_TXEIE) == 0U) {
       if (log_tail != log_head) {
           uint8_t c = log_buf[log_tail & LOG_MASK];
           log_tail = (log_tail + 1U) & LOG_MASK;
           USART2->DR = c;
           USART2->CR1 |= USART_CR1_TXEIE;
       }
   }
}
/**
 * @brief Queue a null-terminated string for logging.
 * @param s String to send.
 */
void log_puts(const char *s)
{
   if (s == NULL) {
       return;
   }
   log_write_bytes((const uint8_t *)s, (uint32_t)strlen(s));
}
/**
 * @brief Queue one character for logging.
 * @param c Character to send.
 */
void log_putc(char c)
{
   log_write_bytes((const uint8_t *)&c, 1U);
}
/**
 * @brief Format and queue one log message.
 * @param fmt printf-style format string.
 */
void log_printf(const char *fmt, ...)
{
   char tmp[LOG_SCRATCH_SZ];
   va_list ap;
   int n;
   va_start(ap, fmt);
   n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
   va_end(ap);
   /* Truncate to the local scratch buffer before queueing. */
   if (n > 0) {
       if (n > (int)(sizeof(tmp) - 1)) {
           n = (int)(sizeof(tmp) - 1);
       }
       log_write_bytes((const uint8_t *)tmp, (uint32_t)n);
   }
}
/**
 * @brief Handle USART2 RXNE and TXE events.
 */
void uart_log_irq_handler(void)
{
   /* RXNE is handled first because reading DR clears the receive event. */
   if ((USART2->SR & USART_SR_RXNE) != 0U) {
       char received = (char)(USART2->DR & 0xFFU);
       /* Echo directly when TX is idle, otherwise queue it. */
       if ((USART2->CR1 & USART_CR1_TXEIE) == 0U) {
           USART2->DR = (uint8_t)received;
       } else {
           log_putc(received);
       }
   }
   /* TXE drains one queued byte per interrupt. */
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
/* USART2 interrupt entry point. */
void USART2_IRQHandler(void)
{
   uart_log_irq_handler();
}
