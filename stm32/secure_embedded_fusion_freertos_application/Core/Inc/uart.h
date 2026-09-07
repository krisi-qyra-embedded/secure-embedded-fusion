/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include <stdarg.h>
#include "stm32f411xe.h"
#define periphclck 			16000000//max clock speed
#define UART2_BAUDRATE 		115200U
void uart2_tx_rx_init(void);
void uart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t BaudRate);
uint16_t compute_uart_div(uint32_t PeriphClk,uint32_t BaudRate);
void uart2_transmit(int ch);
char uart2_read(void);
#define LOG_BUF_SIZE 512
#define LOG_SCRATCH_SZ 256
typedef enum { LOG_LEVEL_ERROR = 0, LOG_LEVEL_INFO = 1, LOG_LEVEL_DEBUG = 2 } log_level_t;
#ifndef LOG_LEVEL
  #define LOG_LEVEL LOG_LEVEL_DEBUG
#endif
#define LOG_ERROR(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_ERROR)  log_printf("[ERR] " fmt, ##__VA_ARGS__); } while(0)
#define LOG_INFO(fmt, ...)   do { if (LOG_LEVEL >= LOG_LEVEL_INFO)   log_printf("[INF] " fmt, ##__VA_ARGS__); } while(0)
#define LOG_DEBUG(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_DEBUG)  log_printf("[DBG] " fmt, ##__VA_ARGS__); } while(0)
extern uint8_t log_buf[LOG_BUF_SIZE];
extern volatile uint32_t log_head;
extern volatile uint32_t log_tail;
extern const uint32_t LOG_MASK;
void log_puts(const char *s);
void log_putc(char c);
void log_printf(const char *fmt, ...);
void log_write_bytes(const uint8_t *data, uint32_t len);
void uart_log_irq_handler(void);
/* Returns the number of queued log bytes. */
static inline uint32_t log_used(void) { /* Mask the index difference for ring-buffer wrap. */  return (log_head - log_tail) & LOG_MASK; }
/* Returns the available log-ring capacity. */
static inline uint32_t log_free(void) { /* Reserve one slot so equal indices identify an empty ring. */  return (LOG_BUF_SIZE - 1) - log_used(); }
#endif /* INC_UART_H_ */
