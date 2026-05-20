/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file uart.h
 * @brief USART2 logging and non-blocking ring-buffer declarations.
 */
#ifndef INC_UART_H_
#define INC_UART_H_

#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include <stdarg.h>
#include "stm32f411xe.h"

#define periphclck 16000000//max clock speed
#define baudrate 115200

/** @brief Initialize USART2 TX/RX and logging interrupts. */
void uart2_tx_rx_init(void);

/**
 * @brief Program a USART baud-rate divider.
 * @param USARTx USART peripheral instance.
 * @param PeriphClk Peripheral clock frequency.
 * @param BaudRate Requested baud rate.
 */
void uart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t BaudRate);

/**
 * @brief Calculate the USART baud-rate divider.
 * @param PeriphClk Peripheral clock frequency.
 * @param BaudRate Requested baud rate.
 * @return Calculated divider value.
 */
uint16_t compute_uart_div(uint32_t PeriphClk,uint32_t BaudRate);

/**
 * @brief Transmit one character through USART2.
 * @param ch Character value.
 */
void uart2_transmit(int ch);

/**
 * @brief Read one character from USART2.
 * @return Received character.
 */
char uart2_read(void);

/* Non-blocking logging configuration. */
#define LOG_BUF_SIZE 512
#define LOG_SCRATCH_SZ 256

/** @brief Runtime logging level. */
typedef enum { LOG_LEVEL_ERROR = 0, LOG_LEVEL_INFO = 1, LOG_LEVEL_DEBUG = 2 } log_level_t;

#ifndef LOG_LEVEL
 #define LOG_LEVEL LOG_LEVEL_DEBUG
#endif

#define LOG_ERROR(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_ERROR)  log_printf("[ERR] " fmt, ##__VA_ARGS__); } while(0)
#define LOG_INFO(fmt, ...)   do { if (LOG_LEVEL >= LOG_LEVEL_INFO)   log_printf("[INF] " fmt, ##__VA_ARGS__); } while(0)
#define LOG_DEBUG(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_DEBUG)  log_printf("[DBG] " fmt, ##__VA_ARGS__); } while(0)

/* Single-producer/single-consumer log ring state. */
extern uint8_t log_buf[LOG_BUF_SIZE];
extern volatile uint32_t log_head;
extern volatile uint32_t log_tail;
extern const uint32_t LOG_MASK;

/**
 * @brief Queue a string in the logging ring buffer.
 * @param s String to queue.
 */
void log_puts(const char *s);

/**
 * @brief Queue one character in the logging ring buffer.
 * @param c Character to queue.
 */
void log_putc(char c);

/**
 * @brief Format and queue one log message.
 * @param fmt printf-style format string.
 */
void log_printf(const char *fmt, ...);

/**
 * @brief Queue raw bytes in the logging ring buffer.
 * @param data Source bytes.
 * @param len Number of bytes to queue.
 */
void log_write_bytes(const uint8_t *data, uint32_t len);

/** @brief Handle USART2 logging interrupts. */
void uart_log_irq_handler(void);

/**
 * @brief Get the number of bytes currently used in the log ring.
 * @return Used byte count.
 */
static inline uint32_t log_used(void) { return (log_head - log_tail) & LOG_MASK; }

/**
 * @brief Get the remaining ring-buffer capacity.
 * @return Free byte count.
 */
static inline uint32_t log_free(void) { return (LOG_BUF_SIZE - 1) - log_used(); }

#endif /* INC_UART_H_ */
