/* Author: i fiksuari pas prog
 * Documentation date: 2026-05-28
 */

#include "fault_log.h"

#include "stm32f4xx.h"
#include "uart.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>


#ifndef FAULT_LOG_BME680_BUF_SIZE
#define FAULT_LOG_BME680_BUF_SIZE      (24576U)
#endif

#ifndef FAULT_LOG_MPU9250_BUF_SIZE
#define FAULT_LOG_MPU9250_BUF_SIZE     (24576U)
#endif

#ifndef FAULT_LOG_LINE_SIZE
#define FAULT_LOG_LINE_SIZE            (768U)
#endif

#ifndef FAULT_LOG_FLUSH_CHUNK_SIZE
#define FAULT_LOG_FLUSH_CHUNK_SIZE     (96U)
#endif

#ifndef SYSTEM_FAULT_OWNER_NONE
#define SYSTEM_FAULT_OWNER_NONE        (0U)
#endif

#ifndef SYSTEM_FAULT_OWNER_BME680
#define SYSTEM_FAULT_OWNER_BME680      (1U)
#endif

#ifndef SYSTEM_FAULT_OWNER_MPU9250
#define SYSTEM_FAULT_OWNER_MPU9250     (2U)
#endif

extern void log_printf(const char *fmt, ...);

extern SemaphoreHandle_t xSerialSemaphore;
extern volatile uint8_t system_fault_owner;

typedef struct {
    char     *buf;
    uint32_t  size;
    uint32_t  len;
    uint8_t   overflow;
} fault_log_buffer_t;

static char g_bme680_fault_buffer[FAULT_LOG_BME680_BUF_SIZE];
static char g_mpu9250_fault_buffer[FAULT_LOG_MPU9250_BUF_SIZE];

static fault_log_buffer_t g_bme680_log = {
    g_bme680_fault_buffer,
    FAULT_LOG_BME680_BUF_SIZE,
    0U,
    0U
};

static fault_log_buffer_t g_mpu9250_log = {
    g_mpu9250_fault_buffer,
    FAULT_LOG_MPU9250_BUF_SIZE,
    0U,
    0U
};

static volatile uint8_t g_fault_capture_active = 0U;
static volatile uint8_t g_fault_captured_mask  = 0U;
static volatile uint8_t g_fault_log_flushing   = 0U;

/* Protects fault-log state from interrupts. */
static uint32_t fault_log_enter_critical(void)
{
    /* Save PRIMASK before masking interrupts. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/* Restores the fault-log interrupt state. */
static void fault_log_exit_critical(uint32_t primask)
{
    /* Restore the saved PRIMASK rather than assuming IRQs were enabled. */
    __set_PRIMASK(primask);
}

/* Acquires the UART mutex for an ordered fault dump. */
static uint8_t fault_log_uart_take(TickType_t timeout_ticks)
{
    /* Use the supplied timeout only when the scheduler and mutex are available. */
    if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&
        (xSerialSemaphore != NULL)) {
        return (xSemaphoreTake(xSerialSemaphore, timeout_ticks) == pdTRUE) ? 1U : 0U;
    }

    return 1U;
}

/* Releases an acquired fault-log UART mutex. */
static void fault_log_uart_give(uint8_t taken)
{
    /* Avoid releasing the mutex when acquisition did not succeed. */
    if (taken == 0U) {
        return;
    }

    if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&
        (xSerialSemaphore != NULL)) {
        xSemaphoreGive(xSerialSemaphore);
    }
}

/* Allows buffered fault output to drain. */
static void fault_log_uart_drain_delay(uint32_t bytes)
{
    /* Clamp the byte-count-based delay between 12 and 60 ms. */
    uint32_t delay_ms = 12U + (bytes / 4U);

    if (delay_ms < 12U) {
        delay_ms = 12U;
    }

    if (delay_ms > 60U) {
        delay_ms = 60U;
    }

    tim2_delay_ms(delay_ms);
}

/* Selects the fault buffer for a sensor source. */
static fault_log_buffer_t *fault_log_get_buffer(fault_log_src_t src)
{
    /* Return no buffer for sources other than BME680 and MPU9250. */
    if (src == FAULT_LOG_SRC_BME680) {
        return &g_bme680_log;
    }

    if (src == FAULT_LOG_SRC_MPU9250) {
        return &g_mpu9250_log;
    }

    return NULL;
}

/* Resets one fault-log buffer. */
static void fault_log_buffer_clear(fault_log_buffer_t *fb)
{
    /* Clear the length, truncation flag and terminating byte together. */
    if (fb == NULL) {
        return;
    }

    fb->len = 0U;
    fb->overflow = 0U;

    if ((fb->buf != NULL) && (fb->size > 0U)) {
        fb->buf[0] = '\0';
    }
}

/* Appends a line to a bounded sensor fault buffer. */
static void fault_log_buffer_append(fault_log_buffer_t *fb, const char *line)
{
    /* Preserve string termination when a line must be truncated. */
    uint32_t line_len;
    uint32_t free_len;

    if ((fb == NULL) || (fb->buf == NULL) || (fb->size == 0U) || (line == NULL)) {
        return;
    }

    line_len = (uint32_t)strlen(line);

    if (fb->len >= (fb->size - 1U)) {
        fb->overflow = 1U;
        return;
    }

    free_len = (fb->size - 1U) - fb->len;

    if (line_len > free_len) {
        memcpy(&fb->buf[fb->len], line, free_len);
        fb->len += free_len;
        fb->buf[fb->len] = '\0';
        fb->overflow = 1U;
        return;
    }

    memcpy(&fb->buf[fb->len], line, line_len);
    fb->len += line_len;
    fb->buf[fb->len] = '\0';
}

/* Identifies diagnostic lines that indicate a fault or recovery. */
static uint8_t fault_log_line_is_faulty(const char *line)
{
    /* Match the configured error and recovery text markers. */
    if (line == NULL) {
        return 0U;
    }

    if (strstr(line, "[ERROR]")       != NULL) { return 1U; }
    if (strstr(line, "[WARN]")        != NULL) { return 1U; }
    if (strstr(line, "[FATAL]")       != NULL) { return 1U; }
    if (strstr(line, "failed")        != NULL) { return 1U; }
    if (strstr(line, "timeout")       != NULL) { return 1U; }
    if (strstr(line, "did not ACK")   != NULL) { return 1U; }
    if (strstr(line, "BUSY")          != NULL) { return 1U; }
    if (strstr(line, "RECOVER")       != NULL) { return 1U; }
    if (strstr(line, "recovery")      != NULL) { return 1U; }

    return 0U;
}

/* Associates an I2C diagnostic line with its sensor. */
static fault_log_src_t fault_log_src_from_i2c_line(const char *line)
{
    /* Use bus addresses and tags before falling back to the active owner. */
    if (line == NULL) {
        return FAULT_LOG_SRC_SYSTEM;
    }

    if (strstr(line, "bus=0x40005400") != NULL) {
        return FAULT_LOG_SRC_BME680;      
    }

    if (strstr(line, "bus=0x40005C00") != NULL) {
        return FAULT_LOG_SRC_MPU9250;     
    }

    if (strstr(line, "[I2C1][BME680]") != NULL) {
        return FAULT_LOG_SRC_BME680;
    }

    if (strstr(line, "[I2C3][MPU9250]") != NULL) {
        return FAULT_LOG_SRC_MPU9250;
    }

    if (system_fault_owner == SYSTEM_FAULT_OWNER_BME680) {
        return FAULT_LOG_SRC_BME680;
    }

    if (system_fault_owner == SYSTEM_FAULT_OWNER_MPU9250) {
        return FAULT_LOG_SRC_MPU9250;
    }

    return FAULT_LOG_SRC_SYSTEM;
}

/* Decides whether to retain a sensor diagnostic line. */
static uint8_t fault_log_should_capture_sensor_line(fault_log_src_t src, const char *line)
{
    /* Keep subsequent lines once that sensor's capture bit is latched. */
    uint8_t mask;

    if ((src != FAULT_LOG_SRC_BME680) && (src != FAULT_LOG_SRC_MPU9250)) {
        return 0U;
    }

    mask = g_fault_captured_mask;

    if ((mask & (uint8_t)src) != 0U) {
        return 1U;
    }

    if (fault_log_line_is_faulty(line) != 0U) {
        return 1U;
    }

    return 0U;
}

/* Appends a line to the selected sensor fault buffer. */
static void fault_log_capture_line(fault_log_src_t src, const char *line)
{
    /* Protect the buffer update with the saved interrupt mask. */
    fault_log_buffer_t *fb;
    uint32_t primask;

    fb = fault_log_get_buffer(src);
    if (fb == NULL) {
        return;
    }

    primask = fault_log_enter_critical();
    fault_log_buffer_append(fb, line);
    fault_log_exit_critical(primask);
}

/* Prints fault text while the caller owns the UART. */
static void fault_log_print_raw_locked(const char *s)
{
    /* Allow the output to drain according to its byte length. */
    uint32_t n;

    if (s == NULL) {
        return;
    }

    n = (uint32_t)strlen(s);
    log_printf("%s", s);
    fault_log_uart_drain_delay(n);
}

/* Flushes a captured fault buffer in bounded chunks. */
static void fault_log_print_buffer_locked(const char *s)
{
    /* Stop a chunk at a newline when possible. */
    char chunk[FAULT_LOG_FLUSH_CHUNK_SIZE + 1U];
    uint32_t len;
    uint32_t pos = 0U;

    if (s == NULL) {
        return;
    }

    len = (uint32_t)strlen(s);

    while (pos < len) {
        uint32_t n = 0U;

        while (((pos + n) < len) && (n < FAULT_LOG_FLUSH_CHUNK_SIZE)) {
            chunk[n] = s[pos + n];
            n++;

            if (chunk[n - 1U] == '\n') {
                break;
            }
        }

        chunk[n] = '\0';

        log_printf("%s", chunk);
        fault_log_uart_drain_delay(n);

        pos += n;
    }
}

/* Initializes the retained sensor fault logs. */
void fault_log_init(void)
{
    /* Clear both buffers and all capture state. */
    fault_log_clear_all();
}

/* Clears every sensor fault log and capture flag. */
void fault_log_clear_all(void)
{
    /* Reset buffer and shared capture state with interrupts masked. */
    uint32_t primask = fault_log_enter_critical();

    fault_log_buffer_clear(&g_bme680_log);
    fault_log_buffer_clear(&g_mpu9250_log);

    g_fault_capture_active = 0U;
    g_fault_captured_mask  = 0U;
    g_fault_log_flushing   = 0U;

    fault_log_exit_critical(primask);
}

/* Latches capture for a supported sensor source. */
void fault_log_mark_source(fault_log_src_t src)
{
    /* Reject sources without a dedicated capture buffer. */
    uint32_t primask;

    if ((src != FAULT_LOG_SRC_BME680) && (src != FAULT_LOG_SRC_MPU9250)) {
        return;
    }

    primask = fault_log_enter_critical();

    g_fault_capture_active = 1U;
    g_fault_captured_mask |= (uint8_t)src;

    fault_log_exit_critical(primask);
}

/* Marks the fault-log source owning an I2C bus. */
void fault_log_mark_from_i2c_bus(void *i2c_instance)
{
    /* Map I2C1 to BME680 and I2C3 to MPU9250. */
    if (i2c_instance == (void *)I2C1) {
        fault_log_mark_source(FAULT_LOG_SRC_BME680);
    } else if (i2c_instance == (void *)I2C3) {
        fault_log_mark_source(FAULT_LOG_SRC_MPU9250);
    }
}

/* Reports whether sensor fault capture is active. */
uint8_t fault_log_capture_is_active(void)
{
    /* Read the capture gate used to suppress routine output. */
    return g_fault_capture_active;
}

/* Returns the sensor fault-capture mask. */
uint8_t fault_log_get_captured_mask(void)
{
    /* Expose the latched source bits for ordered flushing. */
    return g_fault_captured_mask;
}

/* Formats a diagnostic line and routes it to fault capture. */
void fault_log_printf(fault_log_src_t src, const char *fmt, ...)
{
    /* Retain sensor fault lines instead of interleaving them with normal output. */
    char line[FAULT_LOG_LINE_SIZE];
    va_list ap;
    int n;
    uint8_t should_capture;

    if (fmt == NULL) {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (n < 0) {
        return;
    }

    line[sizeof(line) - 1U] = '\0';

    should_capture = fault_log_should_capture_sensor_line(src, line);

    if (((src == FAULT_LOG_SRC_BME680) || (src == FAULT_LOG_SRC_MPU9250)) &&
        (should_capture != 0U)) {
        fault_log_mark_source(src);
        fault_log_capture_line(src, line);
        return;
    }

    if (g_fault_log_flushing != 0U) {
        return;
    }

    log_printf("%s", line);
}

/* Formats an I2C diagnostic line and selects its fault buffer. */
void fault_log_i2c_printf(const char *fmt, ...)
{
    /* Resolve the sensor source before choosing capture or direct output. */
    char line[FAULT_LOG_LINE_SIZE];
    va_list ap;
    int n;
    fault_log_src_t src;

    if (fmt == NULL) {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (n < 0) {
        return;
    }

    line[sizeof(line) - 1U] = '\0';

    src = fault_log_src_from_i2c_line(line);

    if ((src == FAULT_LOG_SRC_BME680) || (src == FAULT_LOG_SRC_MPU9250)) {
        if (fault_log_should_capture_sensor_line(src, line) != 0U) {
            fault_log_mark_source(src);
            fault_log_capture_line(src, line);
            return;
        }
    }

    if (g_fault_log_flushing != 0U) {
        return;
    }

    log_printf("%s", line);
}

/* Flushes captured BME680 and MPU9250 faults in a fixed order. */
void fault_log_flush_ordered(uint8_t fault_mask)
{
    /* Hold the UART mutex across both sensor reports and the summary. */
    uint8_t bme_present;
    uint8_t mpu_present;
    uint8_t uart_taken;

    uart_taken = fault_log_uart_take(pdMS_TO_TICKS(2000U));

    g_fault_log_flushing = 1U;

    bme_present = (uint8_t)(((fault_mask | g_fault_captured_mask) & FAULT_SRC_BME680) != 0U);
    mpu_present = (uint8_t)(((fault_mask | g_fault_captured_mask) & FAULT_SRC_MPU9250) != 0U);

    fault_log_print_raw_locked("\r\n================ ORDERED FAULT LOG DUMP ================\r\n");

    if (bme_present != 0U) {
        fault_log_print_raw_locked("\r\n---------------- BME680 FAULT LOG ----------------\r\n");

        if (g_bme680_log.len > 0U) {
            fault_log_print_buffer_locked(g_bme680_log.buf);
        } else {
            fault_log_print_raw_locked("[FAULT_LOG][BME680] fault bit captured, but buffer is empty\r\n");
        }

        if (g_bme680_log.overflow != 0U) {
            fault_log_print_raw_locked("\r\n[FAULT_LOG][BME680] buffer overflow/truncated\r\n");
        }
    }

    if (mpu_present != 0U) {
        fault_log_print_raw_locked("\r\n---------------- MPU9250 FAULT LOG ----------------\r\n");

        if (g_mpu9250_log.len > 0U) {
            fault_log_print_buffer_locked(g_mpu9250_log.buf);
        } else {
            fault_log_print_raw_locked("[FAULT_LOG][MPU9250] fault bit captured, but buffer is empty\r\n");
        }

        if (g_mpu9250_log.overflow != 0U) {
            fault_log_print_raw_locked("\r\n[FAULT_LOG][MPU9250] buffer overflow/truncated\r\n");
        }
    }

    {
        char summary[160];

        fault_log_print_raw_locked("\r\n---------------- FAULT SUMMARY ----------------\r\n");

        (void)snprintf(summary,
                       sizeof(summary),
                       "[FAULT_SUMMARY] fault_mask=0x%02X captured_mask=0x%02X\r\n",
                       (unsigned)fault_mask,
                       (unsigned)g_fault_captured_mask);
        summary[sizeof(summary) - 1U] = '\0';
        fault_log_print_raw_locked(summary);
    }

    if (bme_present != 0U) {
        fault_log_print_raw_locked("[FAULT_SUMMARY][BME680] fault captured during collection window\r\n");
    }

    if (mpu_present != 0U) {
        fault_log_print_raw_locked("[FAULT_SUMMARY][MPU9250] fault captured during collection window\r\n");
    }

    fault_log_print_raw_locked("=========================================================\r\n\r\n");

    g_fault_log_flushing = 0U;

    fault_log_uart_give(uart_taken);
}
