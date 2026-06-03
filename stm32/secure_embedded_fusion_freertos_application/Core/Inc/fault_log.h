/* Author: i fiksuari pas prog
 * Documentation date: 2026-05-28
 */

#ifndef FAULT_LOG_H
#define FAULT_LOG_H

#include <stdint.h>
#include <stdarg.h>


#ifndef FAULT_SRC_BME680
#define FAULT_SRC_BME680      (1U << 0)
#endif

#ifndef FAULT_SRC_MPU9250
#define FAULT_SRC_MPU9250     (1U << 1)
#endif

typedef enum {
    FAULT_LOG_SRC_SYSTEM  = 0,
    FAULT_LOG_SRC_BME680  = FAULT_SRC_BME680,
    FAULT_LOG_SRC_MPU9250 = FAULT_SRC_MPU9250
} fault_log_src_t;

void fault_log_init(void);
void fault_log_clear_all(void);

void fault_log_mark_source(fault_log_src_t src);
void fault_log_mark_from_i2c_bus(void *i2c_instance);

void fault_log_printf(fault_log_src_t src, const char *fmt, ...);
void fault_log_i2c_printf(const char *fmt, ...);

void fault_log_flush_ordered(uint8_t fault_mask);

uint8_t fault_log_capture_is_active(void);
uint8_t fault_log_get_captured_mask(void);

#endif /* FAULT_LOG_H */
