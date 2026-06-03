/*
 * bme680.h
 *
 *  Created on: Oct 9, 2025
 *      Author: user
 */

#ifndef BME680_H_
#define BME680_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "i2c_dma.h"
#include <bme680_defs.h>
#include "fault_log.h"

#ifdef BME680_LOG_PRINTF
#undef BME680_LOG_PRINTF
#endif
/* ----------------------------- Adapter policy ----------------------------- */
/*
 * Lower layer is single-shot only.
 * No nested retries here. Runtime/app layer owns the 3 total retries.
 */
extern SemaphoreHandle_t xSerialSemaphore;
#define BME680_USER_I2C_TIMEOUT_ITER    (200000U)

#define BME680_LOG_PRINTF(...)                                                     \
    do {                                                                           \
        if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&                 \
            (xSerialSemaphore != NULL)) {                                          \
            if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) == pdTRUE) {  \
                fault_log_printf(FAULT_LOG_SRC_BME680, __VA_ARGS__);               \
                xSemaphoreGive(xSerialSemaphore);                                  \
            }                                                                      \
        } else {                                                                   \
            fault_log_printf(FAULT_LOG_SRC_BME680, __VA_ARGS__);                   \
        }                                                                          \
    } while (0)

/* --------------------------------------------------------------------------
 * Explicit return codes for Bosch user adapter hooks.
 * -------------------------------------------------------------------------- */
typedef enum {
    BME680_USER_I2C_OK            =  0,
    BME680_USER_I2C_E_INVALID_ARG = -1,
    BME680_USER_I2C_E_START_FAIL  = -2,
    BME680_USER_I2C_E_TIMEOUT     = -3
} bme680_user_i2c_status_t;

typedef enum
{
    APP_BME680_RUNTIME_OK = 0,
    APP_BME680_RUNTIME_INVALID_ARG,
    APP_BME680_RUNTIME_I2C_RECOVER_FAIL,
    APP_BME680_RUNTIME_PERIPH_REINIT_FAIL,
    APP_BME680_RUNTIME_SENSOR_REINIT_FAIL,
    APP_BME680_RUNTIME_SENSOR_DATA_FAIL,
    APP_BME680_RUNTIME_GAS_REF_FAIL,
    APP_BME680_RUNTIME_SET_MODE_FAIL
} app_bme680_runtime_status_t;

extern app_bme680_runtime_status_t rc;

typedef enum
{
    SENSOR_STACK_INIT_FAIL = 0,
    SENSOR_STACK_OK = -1,
} sensor_stack_init_status_t;

void user_delay_ms(uint32_t period);

/* Bosch bme680_com_fptr_t requires int8_t return. */
int8_t user_i2c_read(void *handle,
                     uint8_t dev_id,
                     uint8_t reg_addr,
                     uint8_t *reg_data,
                     uint16_t len);

int8_t user_i2c_write(void *handle,
                      uint8_t dev_id,
                      uint8_t reg_addr,
                      uint8_t *reg_data,
                      uint16_t len);

uint8_t to7bit(uint32_t addr_const);

int8_t null_ptr_check(const struct bme680_dev *dev);

sensor_stack_init_status_t app_bme680_sensor_stack_init(void);
app_bme680_runtime_status_t app_bme680_sensor_runtime_process(void);

/* Bosch API prototypes */
int8_t bme680_init(struct bme680_dev *dev);
int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme680_dev *dev);
int8_t bme680_soft_reset(struct bme680_dev *dev);
int8_t bme680_set_sensor_mode(struct bme680_dev *dev);
int8_t bme680_get_sensor_mode(struct bme680_dev *dev);
void   bme680_set_profile_dur(uint16_t duration, struct bme680_dev *dev);
int8_t bme680_get_profile_dur(uint16_t *duration, const struct bme680_dev *dev);
int8_t bme680_get_sensor_data(struct bme680_field_data *data, struct bme680_dev *dev);
int8_t bme680_set_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev);
int8_t bme680_get_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* BME680_H_ */
