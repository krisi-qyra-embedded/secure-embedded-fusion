/**
 * @file uart.h
 * @brief USART2 logging, ring-buffer TX and RX/TX helper declarations.
 */
#ifndef INC_UART_H_
#define INC_UART_H_
#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include <stdarg.h>
#include "stm32f411xe.h"
/* USART2 clock and baud-rate configuration. */
#define periphclck 			16000000//max clock speed
#define UART2_BAUDRATE 		115200U
/**
 * @brief Initialize USART2 TX/RX.
 */
void uart2_tx_rx_init(void);
/**
 * @brief Configure a USART baud rate.
 * @param USARTx USART peripheral.
 * @param PeriphClk Peripheral clock frequency.
 * @param BaudRate Requested baud rate.
 */
void uart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t BaudRate);
/**
 * @brief Calculate the USART baud-rate divider.
 * @param PeriphClk Peripheral clock frequency.
 * @param BaudRate Requested baud rate.
 * @return USART divider value.
 */
uint16_t compute_uart_div(uint32_t PeriphClk,uint32_t BaudRate);
/**
 * @brief Transmit one character on USART2.
 * @param ch Character value.
 */
void uart2_transmit(int ch);
/**
 * @brief Read one character from USART2.
 * @return Received character.
 */
char uart2_read(void);
/* Logging ring-buffer and formatting sizes. */
#define LOG_BUF_SIZE 512
#define LOG_SCRATCH_SZ 256
/** @brief Runtime logging level. */
typedef enum { LOG_LEVEL_ERROR = 0, LOG_LEVEL_INFO = 1, LOG_LEVEL_DEBUG = 2 } log_level_t;
#ifndef LOG_LEVEL
  #define LOG_LEVEL LOG_LEVEL_DEBUG
#endif
/* Lightweight logging macros for hot paths. */
#define LOG_ERROR(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_ERROR)  log_printf("[ERR] " fmt, ##__VA_ARGS__); } while(0)
#define LOG_INFO(fmt, ...)   do { if (LOG_LEVEL >= LOG_LEVEL_INFO)   log_printf("[INF] " fmt, ##__VA_ARGS__); } while(0)
#define LOG_DEBUG(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_DEBUG)  log_printf("[DBG] " fmt, ##__VA_ARGS__); } while(0)
/* Single-producer/single-consumer log ring state. */
extern uint8_t log_buf[LOG_BUF_SIZE];
extern volatile uint32_t log_head;
extern volatile uint32_t log_tail;
extern const uint32_t LOG_MASK;
/**
 * @brief Queue a null-terminated string for logging.
 * @param s String to queue.
 */
void log_puts(const char *s);
/**
 * @brief Queue one character for logging.
 * @param c Character to queue.
 */
void log_putc(char c);
/**
 * @brief Format and queue a log message.
 * @param fmt printf-style format string.
 */
void log_printf(const char *fmt, ...);
/**
 * @brief Queue a raw byte buffer for logging.
 * @param data Source buffer.
 * @param len Number of bytes.
 */
void log_write_bytes(const uint8_t *data, uint32_t len);
/**
 * @brief Handle USART2 logging interrupts.
 */
void uart_log_irq_handler(void);
/* Ring-buffer occupancy helpers. */
static inline uint32_t log_used(void) { return (log_head - log_tail) & LOG_MASK; }
static inline uint32_t log_free(void) { return (LOG_BUF_SIZE - 1) - log_used(); }
#endif /* INC_UART_H_ */
