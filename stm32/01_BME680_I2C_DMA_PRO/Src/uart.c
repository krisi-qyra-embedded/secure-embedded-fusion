/* adc_dma.c | Author: i fiksuari mbas prog | Date: 28 January 2026 */
#include "uart.h"

/* USART2 is on APB1. For a typical STM32F411 96 MHz system-clock configuration: SYSCLK = 96 MHz APB1 = 48. */
#ifndef UART2_PERIPH_CLK_HZ
#define UART2_PERIPH_CLK_HZ    (48000000UL)
#endif

/* ring buffer state (single producer, single consumer) */
uint8_t log_buf[LOG_BUF_SIZE];
volatile uint32_t log_head = 0U; 
volatile uint32_t log_tail = 0U; 
const uint32_t LOG_MASK = LOG_BUF_SIZE - 1U; 

/* Configures uart2 gpio init for normal operation. */
static void uart2_gpio_init(void)
{
    /* Configure only the resources required by this module. */
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    
    GPIOA->MODER &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->MODER |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));

    
    GPIOA->OTYPER &= ~((1U << 2U) | (1U << 3U));

    
    GPIOA->OSPEEDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->OSPEEDR |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));

    /* PUPDR: optional pull-up for RX/TX idle-high UART lines */
    GPIOA->PUPDR &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->PUPDR |=  ((1U << (2U * 2U)) | (1U << (3U * 2U)));

    
    GPIOA->AFR[0] &= ~((0xFU << (2U * 4U)) | (0xFU << (3U * 4U)));
    GPIOA->AFR[0] |=  ((7U   << (2U * 4U)) | (7U   << (3U * 4U)));
}

/* Configures uart2 tx rx init for normal operation. */
void uart2_tx_rx_init(void)
{
    /* Configure only the resources required by this module. */
    uart2_gpio_init();

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    
    USART2->CR1 &= ~USART_CR1_UE;

    /* 8 data bits, no parity, 1 stop bit, oversampling by 16 */
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;

    uart_set_baudrate(USART2, UART2_PERIPH_CLK_HZ, baudrate);

    
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);

    /* Logging IRQ priority lower than critical I2C/DMA IRQs */
    NVIC_SetPriority(USART2_IRQn, 3U);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);

    
    USART2->CR1 |= USART_CR1_UE;

    
    log_head = 0U;
    log_tail = 0U;
}

/* Updates set baudrate and propagates any transfer error. */
void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Stop immediately if the peripheral reports an error. */
    USARTx->BRR = compute_uart_div(PeriphClk, BaudRate);
}

/* Handles the compute uart div operation and returns the resulting status. */
uint16_t compute_uart_div(uint32_t PeriphClk, uint32_t BaudRate)
{
    /* Check the current status before continuing with the operation. */
    return (uint16_t)((PeriphClk + (BaudRate / 2U)) / BaudRate);
}

/* non-blocking ring buffer writer */
void log_write_bytes(const uint8_t *data, uint32_t len)
{
    /* Stop immediately if the peripheral reports an error. */
    if ((data == NULL) || (len == 0U)) {
        return;
    }

    /* quick check: drop message if not enough space (non-blocking) */
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

    /* publish head under critical section to avoid race with IRQ consumer */
    __disable_irq();
    log_head = (log_head + len) & LOG_MASK;
    __enable_irq();

    /* kick TX if idle: write first byte directly and enable TXE interrupt */
    if ((USART2->CR1 & USART_CR1_TXEIE) == 0U) {
        if (log_tail != log_head) {
            uint8_t c = log_buf[log_tail & LOG_MASK];
            log_tail = (log_tail + 1U) & LOG_MASK;
            USART2->DR = c;
            USART2->CR1 |= USART_CR1_TXEIE;
        }
    }
}


/* Handles the log puts operation and returns the resulting status. */
void log_puts(const char *s)
{
    /* Check the current status before continuing with the operation. */
    if (s == NULL) {
        return;
    }

    log_write_bytes((const uint8_t *)s, (uint32_t)strlen(s));
}

/* Handles the log putc operation and returns the resulting status. */
void log_putc(char c)
{
    /* Check the current status before continuing with the operation. */
    log_write_bytes((const uint8_t *)&c, 1U);
}

/* Handles the log printf operation and returns the resulting status. */
void log_printf(const char *fmt, ...)
{
    /* Check the current status before continuing with the operation. */
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

/* USART2 IRQ handler helper (TXE & RXNE) */
void uart_log_irq_handler(void)
{
    /* Clear handled flags and update the transfer state. */
    
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

/* Handles the interrupt events associated with USART2 IRQHandler. */
void USART2_IRQHandler(void)
{
    /* Clear handled flags and update the transfer state. */
    uart_log_irq_handler();
}
