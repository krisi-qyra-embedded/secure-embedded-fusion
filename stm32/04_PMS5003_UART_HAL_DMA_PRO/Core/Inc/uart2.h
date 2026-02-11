/* uart2.h | Author: i fiksuari pas prog | Date: 11 February 2026 */

#ifndef INC_UART2_H_
#define INC_UART2_H_

#include<stdio.h>
#include<stdint.h>
#include<string.h>
#include <stdarg.h>
#include "stm32f411xe.h"
void uart2_tx_rx_init(void);

#define periphclck 16000000//max clock speed
#define baudrate 115200
void uart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk,uint32_t BaudRate);
uint16_t compute_uart_div(uint32_t PeriphClk,uint32_t BaudRate);
void uart2_transmit(int ch);
char uart2_read(void);

/* ---------- Logging subsystem configuration ---------- */
#define LOG_BUF_SIZE 512               /* power of two preferred */
#define LOG_SCRATCH_SZ 256             /* temporary buffer for formatting */

/* Log level */
typedef enum { LOG_LEVEL_ERROR = 0, LOG_LEVEL_INFO = 1, LOG_LEVEL_DEBUG = 2 } log_level_t;
#ifndef LOG_LEVEL
  #define LOG_LEVEL LOG_LEVEL_DEBUG
#endif

/* LOG macros (use these instead of printf in hot code) */
#define LOG_ERROR(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_ERROR)  log_printf("[ERR] " fmt, ##__VA_ARGS__); } while(0)
#define LOG_INFO(fmt, ...)   do { if (LOG_LEVEL >= LOG_LEVEL_INFO)   log_printf("[INF] " fmt, ##__VA_ARGS__); } while(0)
#define LOG_DEBUG(fmt, ...)  do { if (LOG_LEVEL >= LOG_LEVEL_DEBUG)  log_printf("[DBG] " fmt, ##__VA_ARGS__); } while(0)

/* ---------- ring buffer state (single producer, single consumer) ---------- */
extern uint8_t log_buf[LOG_BUF_SIZE];
extern volatile uint32_t log_head; /* producer index (next write) */
extern volatile uint32_t log_tail; /* consumer index (next read) */
extern const uint32_t LOG_MASK; /* only valid when LOG_BUF_SIZE power-of-two */

/* ---------- forward declarations ---------- */
void log_puts(const char *s);           /* non-blocking, safe to call from main/ISRs (avoid heavy formatting) */
void log_putc(char c);
void log_printf(const char *fmt, ...);  /* formatting helper - ok from task/main context (uses stack) */
void log_write_bytes(const uint8_t *data, uint32_t len);
void uart_log_irq_handler(void);        /* call from USART2 IRQ handler */

/* ---------- helper inline functions ---------- */
static inline uint32_t log_used(void) { return (log_head - log_tail) & LOG_MASK; }
static inline uint32_t log_free(void) { return (LOG_BUF_SIZE - 1) - log_used(); }


#endif /* INC_UART_H_ */
