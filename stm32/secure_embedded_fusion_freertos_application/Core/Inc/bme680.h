/**
 * @file bme680.h
 * @brief BME680 adapter, runtime status and driver API declarations.
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
/* Adapter is single-shot; runtime layer owns the retry budget. */
extern SemaphoreHandle_t xSerialSemaphore;
#define BME680_USER_I2C_TIMEOUT_ITER    (200000U)
/* Serialize task-level BME680 logs once the scheduler is running. */
#define BME680_LOG_PRINTF(...)                                                     \
   do {                                                                           \
       if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&                 \
           (xSerialSemaphore != NULL)) {                                          \
           if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) == pdTRUE) {  \
               log_printf(__VA_ARGS__);                                           \
               xSemaphoreGive(xSerialSemaphore);                                  \
           }                                                                      \
       } else {                                                                   \
           log_printf(__VA_ARGS__);                                               \
       }                                                                          \
   } while (0)
/** @brief Bosch adapter I2C status codes. */
typedef enum {
   BME680_USER_I2C_OK            =  0,
   BME680_USER_I2C_E_INVALID_ARG = -1,
   BME680_USER_I2C_E_START_FAIL  = -2,
   BME680_USER_I2C_E_TIMEOUT     = -3
} bme680_user_i2c_status_t;
/** @brief BME680 runtime processing status. */
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
/** @brief BME680 sensor-stack initialization status. */
typedef enum
{
   SENSOR_STACK_INIT_FAIL = 0,
   SENSOR_STACK_OK = -1,
} sensor_stack_init_status_t;
/**
 * @brief Delay callback used by the BME680 driver.
 * @param period Delay in milliseconds.
 */
void user_delay_ms(uint32_t period);
/**
 * @brief Read BME680 registers through the project I2C adapter.
 * @param handle Optional bus handle.
 * @param dev_id BME680 I2C address.
 * @param reg_addr Register address.
 * @param reg_data Destination buffer.
 * @param len Number of bytes to read.
 * @return Bosch-compatible adapter status.
 */
int8_t user_i2c_read(void *handle,
                    uint8_t dev_id,
                    uint8_t reg_addr,
                    uint8_t *reg_data,
                    uint16_t len);
/**
 * @brief Write BME680 registers through the project I2C adapter.
 * @param handle Optional bus handle.
 * @param dev_id BME680 I2C address.
 * @param reg_addr Register address.
 * @param reg_data Source buffer.
 * @param len Number of bytes to write.
 * @return Bosch-compatible adapter status.
 */
int8_t user_i2c_write(void *handle,
                     uint8_t dev_id,
                     uint8_t reg_addr,
                     uint8_t *reg_data,
                     uint16_t len);
/**
 * @brief Convert the configured address value to 7-bit I2C form.
 * @param addr_const Address value.
 * @return 7-bit address.
 */
uint8_t to7bit(uint32_t addr_const);
/**
 * @brief Validate the BME680 device context.
 * @param dev Device context.
 * @return Bosch driver status.
 */
int8_t null_ptr_check(const struct bme680_dev *dev);
/**
 * @brief Initialize the BME680 sensor stack.
 * @return Sensor-stack initialization status.
 */
sensor_stack_init_status_t app_bme680_sensor_stack_init(void);
/**
 * @brief Run one BME680 runtime sample/recovery cycle.
 * @return Runtime processing status.
 */
app_bme680_runtime_status_t app_bme680_sensor_runtime_process(void);
/* Bosch BME680 driver API. */
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
