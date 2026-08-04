#include <bme680.h>
#include "tim.h"
#include "i2c_dma.h"
#include "uart.h"
#include "iwdg.h"
#include "pwr.h"
#include "sig_sensors.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include "telemetry.h"

/**@PINOUT

 *	PB8 :  SCL
 *	PB9 :  SDA
 *	VCC :  3.3v
 *	GND :  GND
 *	SDO :  GND
 *	CS  :  3.3v
*/

#define MPU_READ_TIMEOUT_ITER  100000U   /* adjust for your CPU clock if needed */
#define MPU_READ_RETRIES        3U

#ifndef BME680_USER_I2C_RTOS_TIMEOUT_MS
#define BME680_USER_I2C_RTOS_TIMEOUT_MS    (100U)
#endif


extern uint32_t bme680_generation;
extern QueueHandle_t q_bme680;


/* FreeRTOS binary semaphores owned by i2c_dma.c and given by DMA1_Stream5/6 IRQs. */
extern SemaphoreHandle_t bme680_rx_sem;
extern SemaphoreHandle_t bme680_tx_sem;
extern SemaphoreHandle_t xSerialSemaphore;
extern volatile uint8_t system_fault_logging_active;
extern volatile uint8_t system_fault_owner;
extern void mpu9250_pause_for_bme680_gas_ref(void);
extern void mpu9250_resume_after_bme680_gas_ref(void);

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bme680_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme680_soft_reset(struct bme680_dev *dev);

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len : No of bytes of data to be read.
 * @param[in] dev : Structure instance of bme680_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme680_dev *dev);

/*!
 * @brief This API is used to set the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme680_dev
 * @note : Pass the value to bme680_dev.power_mode structure variable.
 *
 *  value	|	mode
 * -------------|------------------
 *	0x00	|	BME680_SLEEP_MODE
 *	0x01	|	BME680_FORCED_MODE
 *
 * * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme680_set_sensor_mode(struct bme680_dev *dev);

/*!
 * @brief This internal API is used to read the calibrated data from the sensor.
 *
 * This function is used to retrieve the calibration
 * data from the image registers of the sensor.
 *
 * @note Registers 89h  to A1h for calibration data 1 to 24
 *        from bit 0 to 7
 * @note Registers E1h to F0h for calibration data 25 to 40
 *        from bit 0 to 7
 * @param[in] dev	:Structure instance of bme680_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_calib_data(struct bme680_dev *dev);

/*!
 * @brief This internal API is used to set the gas configuration of the sensor.
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_gas_config(struct bme680_dev *dev);

/*!
 * @brief This internal API is used to get the gas configuration of the sensor.
 * @note heatr_temp and heatr_dur values are currently register data
 * and not the actual values set
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_gas_config(struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the Heat duration value.
 *
 * @param[in] dur	:Value of the duration to be shared.
 *
 * @return uint8_t threshold duration after calculation.
 */
static uint8_t calc_heater_dur(uint16_t dur);

#ifndef BME680_FLOAT_POINT_COMPENSATION

/*!
 * @brief This internal API is used to calculate the temperature value.
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 * @param[in] temp_adc	:Contains the temperature ADC value .
 *
 * @return uint32_t calculated temperature.
 */
static int16_t calc_temperature(uint32_t temp_adc, struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the pressure value.
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 * @param[in] pres_adc	:Contains the pressure ADC value .
 *
 * @return uint32_t calculated pressure.
 */
static uint32_t calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the humidity value.
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 * @param[in] hum_adc	:Contains the humidity ADC value.
 *
 * @return uint32_t calculated humidity.
 */
static uint32_t calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the Gas Resistance value.
 *
 * @param[in] dev		:Structure instance of bme680_dev.
 * @param[in] gas_res_adc	:Contains the Gas Resistance ADC value.
 * @param[in] gas_range		:Contains the range of gas values.
 *
 * @return uint32_t calculated gas resistance.
 */
static uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the Heat Resistance value.
 *
 * @param[in] dev	: Structure instance of bme680_dev
 * @param[in] temp	: Contains the target temperature value.
 *
 * @return uint8_t calculated heater resistance.
 */
static uint8_t calc_heater_res(uint16_t temp, const struct bme680_dev *dev);

#else
/*!
 * @brief This internal API is used to calculate the
 * temperature value value in float format
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 * @param[in] temp_adc	:Contains the temperature ADC value .
 *
 * @return Calculated temperature in float
 */
static float calc_temperature(uint32_t temp_adc, struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the
 * pressure value value in float format
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 * @param[in] pres_adc	:Contains the pressure ADC value .
 *
 * @return Calculated pressure in float.
 */
static float calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the
 * humidity value value in float format
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 * @param[in] hum_adc	:Contains the humidity ADC value.
 *
 * @return Calculated humidity in float.
 */
static float calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the
 * gas resistance value value in float format
 *
 * @param[in] dev		:Structure instance of bme680_dev.
 * @param[in] gas_res_adc	:Contains the Gas Resistance ADC value.
 * @param[in] gas_range		:Contains the range of gas values.
 *
 * @return Calculated gas resistance in float.
 */
static float calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev);

/*!
 * @brief This internal API is used to calculate the
 * heater resistance value in float format
 *
 * @param[in] temp	: Contains the target temperature value.
 * @param[in] dev	: Structure instance of bme680_dev.
 *
 * @return Calculated heater resistance in float.
 */
static float calc_heater_res(uint16_t temp, const struct bme680_dev *dev);

#endif

/*!
 * @brief This internal API is used to calculate the field data of sensor.
 *
 * @param[out] data :Structure instance to hold the data
 * @param[in] dev	:Structure instance of bme680_dev.
 *
 *  @return int8_t result of the field data from sensor.
 */
static int8_t read_field_data(struct bme680_field_data *data, struct bme680_dev *dev);

/*!
 * @brief This internal API is used to set the memory page
 * based on register address.
 *
 * The value of memory page
 *  value  | Description
 * --------|--------------
 *   0     | BME680_PAGE0_SPI
 *   1     | BME680_PAGE1_SPI
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 * @param[in] reg_addr	:Contains the register address array.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t set_mem_page(uint8_t reg_addr, struct bme680_dev *dev);

/*!
 * @brief This internal API is used to get the memory page based
 * on register address.
 *
 * The value of memory page
 *  value  | Description
 * --------|--------------
 *   0     | BME680_PAGE0_SPI
 *   1     | BME680_PAGE1_SPI
 *
 * @param[in] dev	:Structure instance of bme680_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t get_mem_page(struct bme680_dev *dev);

/*!
 * @brief This internal API is used to check the boundary
 * conditions.
 *
 * @param[in] value	:pointer to the value.
 * @param[in] min	:minimum value.
 * @param[in] max	:maximum value.
 * @param[in] dev	:Structure instance of bme680_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
static int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme680_dev *dev);

/* helper: accept 7-bit or 8-bit address and normalize to 7-bit */
uint8_t to7bit(uint32_t addr_const)
{
    return (addr_const > 0x7FU) ? (uint8_t)(addr_const >> 1) : (uint8_t)addr_const;
}

/****************** Global Function Definitions *******************************/
void user_delay_ms(uint32_t period)
{
	app_fault_delay_ms(period);
}

static void bme680_clear_dma_semaphore_if_rtos(SemaphoreHandle_t sem)
{
    if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && (sem != NULL)) {
        while (xSemaphoreTake(sem, 0U) == pdTRUE) {
            /* Drain stale completion events before starting a new transfer. */
        }
    }
}

static int bme680_wait_dma_complete_or_timeout(volatile i2c_dma_transfer_state_t *state,
                                               SemaphoreHandle_t sem)
{
    uint32_t t = 0U;

    if (state == NULL) {
        return 0;
    }

    /* Blocking/non-DMA paths may complete before an IRQ semaphore is involved. */
    if (*state == I2C_DMA_TRANSFER_COMPLETE) {
        return 1;
    }

    if (*state == I2C_DMA_TRANSFER_ERROR) {
        return 0;
    }

    /* FreeRTOS path: block the current task until DMA IRQ gives the semaphore. */
    if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && (sem != NULL)) {
        if (xSemaphoreTake(sem, pdMS_TO_TICKS(BME680_USER_I2C_RTOS_TIMEOUT_MS)) == pdTRUE) {
            return (*state == I2C_DMA_TRANSFER_COMPLETE) ? 1 : 0;
        }

        return 0;
    }

    /* Bare-metal/startup fallback: used before scheduler start during sensor init. */
    while ((*state == I2C_DMA_TRANSFER_IDLE) &&
           (t++ < BME680_USER_I2C_TIMEOUT_ITER)) {
        cpu_sleep_until_irq();
    }

    return (*state == I2C_DMA_TRANSFER_COMPLETE) ? 1 : 0;
}

/* Return codes (int8_t):
 *   BME680_USER_I2C_OK            (0)
 *   BME680_USER_I2C_E_INVALID_ARG (-1)
 *   BME680_USER_I2C_E_START_FAIL  (-2)
 *   BME680_USER_I2C_E_TIMEOUT     (-3)
 */
int8_t user_i2c_read(void *handle,
                     uint8_t dev_id,
                     uint8_t reg_addr,
                     uint8_t *reg_data,
                     uint16_t len)
{
    I2C_TypeDef *I2Cx;
    const uint8_t slave7 = to7bit((uint32_t)dev_id);
    i2c_dma_status_t st;

    if ((handle == NULL) || (reg_data == NULL) || (len == 0U)) {
        return (int8_t)BME680_USER_I2C_E_INVALID_ARG;
    }

    I2Cx = (I2C_TypeDef *)handle;
    if (I2Cx != I2C1) {
        return (int8_t)BME680_USER_I2C_E_INVALID_ARG;
    }

    bme680g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;

    /* Clear any stale DMA IRQ semaphore token before starting a new RX transfer. */
    bme680_clear_dma_semaphore_if_rtos(bme680_rx_sem);

    st = i2c_dma_read(I2Cx, slave7, reg_addr, reg_data, len);
    if (st != I2C_DMA_OK) {
        return (int8_t)BME680_USER_I2C_E_START_FAIL;
    }

    if (bme680_wait_dma_complete_or_timeout(&bme680g_rx_cmplt, bme680_rx_sem)) {
        bme680g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
        return (int8_t)BME680_USER_I2C_OK;
    }

    bme680g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
    I2Cx->CR1 |= I2C_CR1_STOP;

    return (int8_t)BME680_USER_I2C_E_TIMEOUT;
}

/* Return codes (int8_t):
 *   BME680_USER_I2C_OK            (0)
 *   BME680_USER_I2C_E_INVALID_ARG (-1)
 *   BME680_USER_I2C_E_START_FAIL  (-2)
 *   BME680_USER_I2C_E_TIMEOUT     (-3)
 */
int8_t user_i2c_write(void *handle,
                      uint8_t dev_id,
                      uint8_t reg_addr,
                      uint8_t *reg_data,
                      uint16_t len)
{
    I2C_TypeDef *I2Cx;
    const uint8_t slave7 = to7bit((uint32_t)dev_id);
    i2c_dma_status_t st;

    if ((handle == NULL) || (reg_data == NULL) || (len == 0U)) {
        return (int8_t)BME680_USER_I2C_E_INVALID_ARG;
    }

    I2Cx = (I2C_TypeDef *)handle;
    if (I2Cx != I2C1) {
        return (int8_t)BME680_USER_I2C_E_INVALID_ARG;
    }

    bme680g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;

    /* Clear any stale DMA IRQ semaphore token before starting a new TX transfer. */
    bme680_clear_dma_semaphore_if_rtos(bme680_tx_sem);

    st = i2c_dma_write_reg(I2Cx, slave7, reg_addr, reg_data, len);
    if (st != I2C_DMA_OK) {
        return (int8_t)BME680_USER_I2C_E_START_FAIL;
    }

    if (bme680_wait_dma_complete_or_timeout(&bme680g_tx_cmplt, bme680_tx_sem)) {
        bme680g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;
        return (int8_t)BME680_USER_I2C_OK;
    }

    bme680g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;
    I2Cx->CR1 |= I2C_CR1_STOP;

    return (int8_t)BME680_USER_I2C_E_TIMEOUT;
}

/*!
 *@brief Initialize BME680: soft-reset, read chip-id, and calibration data.
 *
 *@param[in,out] dev Pointer to BME680 device structure.
 *@return BME680_OK on success, or one of the BME680_E_* error codes on failure.
 */
int8_t bme680_init(struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL; /* Default error */

    BME680_LOG_PRINTF("[INFO] bme680_init: entry\r\n");
    app_fault_delay_ms(20);

    /* Step 1 — Null pointer check */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_init: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* Step 2 — Soft reset */
    rslt = bme680_soft_reset(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_init: soft reset failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_COM_FAIL;
    }

    /* Step 3 — Read CHIP_ID register */
    rslt = bme680_get_regs(BME680_CHIP_ID_ADDR, (uint8_t *)(&dev->chip_id), 1, dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_init: failed to read CHIP_ID register (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_COM_FAIL;
    }

    /* Step 4 — Validate CHIP_ID */
    if (dev->chip_id != BME680_CHIP_ID) {
    	BME680_LOG_PRINTF("[ERROR] bme680_init: unexpected CHIP_ID (0x%02X != 0x%02X)\r\n",
                   (uint8_t)dev->chip_id, (uint8_t)BME680_CHIP_ID);
    	app_fault_delay_ms(20);
        return BME680_E_DEV_NOT_FOUND;
    }

    /* Step 5 — Get calibration data */
    rslt = get_calib_data(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_init: get_calib_data failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_COM_FAIL;
    }

    return BME680_OK;
}

/*!
 * @brief Reads data from the given register address of the sensor.
 *
 * @param[in] reg_addr Register address to read from.
 * @param[out] reg_data Pointer to buffer to store register data.
 * @param[in] len Number of bytes to read.
 * @param[in,out] dev Pointer to BME680 device structure.
 * @return BME680_OK on success, or one of the BME680_E_* error codes.
 */
int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL; /* Default to communication failure */

    /* Step 1 — Null pointer check */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_regs: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* Step 2 — Handle SPI interface specifics */
    if (dev->intf == BME680_SPI_INTF) {
        rslt = set_mem_page(reg_addr, dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] bme680_get_regs: set_mem_page failed (err=%d)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
            return rslt;
        }
        BME680_LOG_PRINTF("[DEBUG] bme680_get_regs: SPI memory page set successfully\r\n");
        app_fault_delay_ms(20);

        reg_addr |= BME680_SPI_RD_MSK;
        BME680_LOG_PRINTF("[DEBUG] bme680_get_regs: SPI read mask applied, reg_addr=0x%02X\r\n", reg_addr);
        app_fault_delay_ms(20);
    }

    /* Step 3 — Perform the read operation */
    dev->com_rslt = dev->read(dev->handle, dev->dev_id, reg_addr, reg_data, len);
    if (dev->com_rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_regs: read failed (com_rslt=%d)\r\n", dev->com_rslt);
    	app_fault_delay_ms(20);
        rslt = BME680_E_COM_FAIL;
    } else {
        rslt = BME680_OK;
    }

    return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL;
    uint8_t tmp_buff[BME680_TMP_BUFFER_LENGTH] = { 0 };
    uint16_t index = 0;

    /* Null pointer check */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_regs: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* This implementation is I2C-only — SPI support intentionally removed */
    if (dev->intf != BME680_I2C_INTF) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_regs: device interface is not I2C (intf=%d). SPI removed.\r\n", dev->intf);
    	app_fault_delay_ms(20);
        return BME680_E_COM_FAIL;
    }

    /* Validate length: now we only buffer reg_data bytes, so len must be < tmp buffer length */
    if ((len == 0) || (len > (int)BME680_TMP_BUFFER_LENGTH)) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_regs: invalid length (%u)\r\n", (unsigned)len);
    	app_fault_delay_ms(20);
        return BME680_E_INVALID_LENGTH;
    }

    /* Iterate through reg_addr/reg_data pairs and batch consecutive registers */
    index = 0;
    while (index < len) {
        uint8_t start_reg = reg_addr[index];
        uint8_t run_len = 1;
        tmp_buff[0] = reg_data[index];

        /* Build longest run of consecutive registers starting at index */
        while ((index + run_len) < len) {
            uint8_t prev = reg_addr[index + run_len - 1];
            uint8_t cur  = reg_addr[index + run_len];
            if ((uint8_t)(cur) == (uint8_t)(prev + 1) && run_len < (uint8_t)BME680_TMP_BUFFER_LENGTH) {
                tmp_buff[run_len] = reg_data[index + run_len];
                run_len++;
            } else {
                break;
            }
        }

        /* Perform I2C write: start_reg, tmp_buff[0..run_len-1] */
        dev->com_rslt = dev->write(dev->handle, dev->dev_id, start_reg, tmp_buff, run_len);
        if (dev->com_rslt != 0) {
        	BME680_LOG_PRINTF("[ERROR] bme680_set_regs: I2C write failed for start_reg=0x%02X (com_rslt=%d)\r\n",
                       (unsigned)start_reg, (int)dev->com_rslt);
        	app_fault_delay_ms(20);
            rslt = BME680_E_COM_FAIL;
            break;
        }

        /* advance index by run_len */
        index += run_len;
    }

    if (index == len && dev->com_rslt == 0) {
        rslt = BME680_OK;
    }

    return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t bme680_soft_reset(struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL; /* Default to failure */
    uint8_t reg_addr = BME680_SOFT_RESET_ADDR;
    uint8_t soft_rst_cmd = BME680_SOFT_RESET_CMD; /* 0xB6 soft reset command */

    /* Step 1 — Null pointer check */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_soft_reset: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* Step 2 — Handle SPI memory page before reset */
    if (dev->intf == BME680_SPI_INTF) {
        rslt = get_mem_page(dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] bme680_soft_reset: get_mem_page failed before reset (err=%d)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
            return rslt;
        }
        BME680_LOG_PRINTF("[DEBUG] bme680_soft_reset: SPI memory page set before reset\r\n");
        app_fault_delay_ms(20);
    }

    /* Step 3 — Perform soft reset by writing the reset command */
    rslt = bme680_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_soft_reset: bme680_set_regs failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return rslt;
    }

    /* Step 4 — Wait for reset period */
    dev->delay_ms(BME680_RESET_PERIOD);

    /* Step 5 — Handle SPI memory page after reset */
    if (dev->intf == BME680_SPI_INTF) {
        rslt = get_mem_page(dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] bme680_soft_reset: get_mem_page failed after reset (err=%d)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
            return rslt;
        }
        BME680_LOG_PRINTF("[DEBUG] bme680_soft_reset: SPI memory page set after reset\r\n");
        app_fault_delay_ms(20);
    }

    return rslt;
}

/*!
 * @brief This API is used to set the oversampling, filter and T,P,H, gas selection
 * settings in the sensor.
 */
int8_t bme680_set_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL; /* default */
    uint8_t reg_addr;
    uint8_t data = 0;
    uint8_t count = 0;
    uint8_t reg_array[BME680_REG_BUFFER_LENGTH] = { 0 };
    uint8_t data_array[BME680_REG_BUFFER_LENGTH] = { 0 };
    uint8_t intended_power_mode;

    /* Step 0 — Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_settings: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* Save intended power mode so we can restore it at exit (preserve original behavior) */
    intended_power_mode = dev->power_mode;

    /* Use a single structured block so we can restore power mode at the end */
    do {
        /* If gas config needs to be set, do that first */
        if (desired_settings & BME680_GAS_MEAS_SEL) {
            rslt = set_gas_config(dev);
            if (rslt != BME680_OK) {
            	BME680_LOG_PRINTF("[WARN] bme680_set_sensor_settings: set_gas_config returned %d (continuing)\r\n", (int)rslt);
            	app_fault_delay_ms(20);
            }
        }

        /* Put device into sleep mode before changing settings, then set sensor mode */
        dev->power_mode = BME680_SLEEP_MODE;

        if (rslt == BME680_OK)
            rslt = bme680_set_sensor_mode(dev);

        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[WARN] bme680_set_sensor_settings: bme680_set_sensor_mode returned %d (continuing)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
        }

        /* ------------------------
         * FILTER selection
         * ------------------------ */
        if (desired_settings & BME680_FILTER_SEL) {
            rslt = boundary_check(&dev->tph_sett.filter, BME680_FILTER_SIZE_0, BME680_FILTER_SIZE_127, dev);
            reg_addr = BME680_CONF_ODR_FILT_ADDR;

            if (rslt == BME680_OK) {
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
                if (rslt != BME680_OK) {
                	BME680_LOG_PRINTF("[WARN] bme680_set_sensor_settings: bme680_get_regs(filter) returned %d (will still append)\r\n", (int)rslt);
                	app_fault_delay_ms(20);
                }
            }

            if (desired_settings & BME680_FILTER_SEL)
                data = BME680_SET_BITS(data, BME680_FILTER, dev->tph_sett.filter);

            reg_array[count] = reg_addr;
            data_array[count] = data;
            count++;
        }

        /* ------------------------
         * HEATER CONTROL selection
         * ------------------------ */
        if (desired_settings & BME680_HCNTRL_SEL) {
            rslt = boundary_check(&dev->gas_sett.heatr_ctrl, BME680_ENABLE_HEATER, BME680_DISABLE_HEATER, dev);
            reg_addr = BME680_CONF_HEAT_CTRL_ADDR;

            if (rslt == BME680_OK) {
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
                if (rslt != BME680_OK) {
                	BME680_LOG_PRINTF("[WARN] bme680_set_sensor_settings: bme680_get_regs(hctrl) returned %d (will still append)\r\n", (int)rslt);
                	app_fault_delay_ms(20);
                }
            }

            data = BME680_SET_BITS_POS_0(data, BME680_HCTRL, dev->gas_sett.heatr_ctrl);

            reg_array[count] = reg_addr;
            data_array[count] = data;
            count++;
        }

        /* ------------------------
         * TEMP/PRESS oversampling selection
         * ------------------------ */
        if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
            rslt = boundary_check(&dev->tph_sett.os_temp, BME680_OS_NONE, BME680_OS_16X, dev);
            reg_addr = BME680_CONF_T_P_MODE_ADDR;

            if (rslt == BME680_OK) {
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
                if (rslt != BME680_OK) {
                	BME680_LOG_PRINTF("[WARN] bme680_set_sensor_settings: bme680_get_regs(t/p) returned %d (will still append)\r\n", (int)rslt);
                	app_fault_delay_ms(20);
                }
            }

            if (desired_settings & BME680_OST_SEL)
                data = BME680_SET_BITS(data, BME680_OST, dev->tph_sett.os_temp);

            if (desired_settings & BME680_OSP_SEL)
                data = BME680_SET_BITS(data, BME680_OSP, dev->tph_sett.os_pres);

            reg_array[count] = reg_addr;
            data_array[count] = data;
            count++;
        }

        /* ------------------------
         * HUMIDITY oversampling selection
         * ------------------------ */
        if (desired_settings & BME680_OSH_SEL) {
            rslt = boundary_check(&dev->tph_sett.os_hum, BME680_OS_NONE, BME680_OS_16X, dev);
            reg_addr = BME680_CONF_OS_H_ADDR;

            if (rslt == BME680_OK) {
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
                if (rslt != BME680_OK) {
                	BME680_LOG_PRINTF("[WARN] bme680_set_sensor_settings: bme680_get_regs(os_h) returned %d (will still append)\r\n", (int)rslt);
                	app_fault_delay_ms(20);
                }
            }

            data = BME680_SET_BITS_POS_0(data, BME680_OSH, dev->tph_sett.os_hum);

            reg_array[count] = reg_addr;
            data_array[count] = data;
            count++;
        }

        /* ------------------------
         * RUN_GAS / NBCONV selection
         * ------------------------ */
        if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
            rslt = boundary_check(&dev->gas_sett.run_gas, BME680_RUN_GAS_DISABLE, BME680_RUN_GAS_ENABLE, dev);
            if (rslt == BME680_OK) {
                rslt = boundary_check(&dev->gas_sett.nb_conv, BME680_NBCONV_MIN, BME680_NBCONV_MAX, dev);
            }

            reg_addr = BME680_CONF_ODR_RUN_GAS_NBC_ADDR;

            if (rslt == BME680_OK) {
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
                if (rslt != BME680_OK) {
                	BME680_LOG_PRINTF("[WARN] bme680_set_sensor_settings: bme680_get_regs(run_gas) returned %d (will still append)\r\n", (int)rslt);
                	app_fault_delay_ms(20);
                }
            }

            if (desired_settings & BME680_RUN_GAS_SEL)
                data = BME680_SET_BITS(data, BME680_RUN_GAS, dev->gas_sett.run_gas);

            if (desired_settings & BME680_NBCONV_SEL)
                data = BME680_SET_BITS_POS_0(data, BME680_NBCONV, dev->gas_sett.nb_conv);

            reg_array[count] = reg_addr;
            data_array[count] = data;
            count++;
        }

        /* If all checks passed so far, write the collected register/data pairs */
        if (rslt == BME680_OK) {
            rslt = bme680_set_regs(reg_array, data_array, count, dev);
            if (rslt != BME680_OK) {
            	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_settings: bme680_set_regs failed (err=%d)\r\n", (int)rslt);
            	app_fault_delay_ms(20);
            }
        }

    } while (0);

    /* Restore previous intended power mode (preserve original behavior) */
    dev->power_mode = intended_power_mode;

    return rslt;
}

/*!
 * @brief This API is used to get the oversampling, filter and T,P,H, gas selection
 * settings in the sensor.
 */
int8_t bme680_get_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL; /* Default to communication failure */
    uint8_t reg_addr = BME680_CONF_HEAT_CTRL_ADDR; /* starting address of the register array for burst read */
    uint8_t data_array[BME680_REG_BUFFER_LENGTH] = { 0 };

    BME680_LOG_PRINTF("[INFO] bme680_get_sensor_settings: entry (settings=0x%04X)\r\n", (unsigned)desired_settings);
    app_fault_delay_ms(20);

    /* Step 1 — Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_sensor_settings: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }
    BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_settings: null_ptr_check passed\r\n");
    app_fault_delay_ms(20);

    /* Step 2 — Read the block of registers needed */
    rslt = bme680_get_regs(reg_addr, data_array, BME680_REG_BUFFER_LENGTH, dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_sensor_settings: bme680_get_regs failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_COM_FAIL;
    }
    BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_settings: register burst read successful\r\n");
    app_fault_delay_ms(20);

    /* Step 3 — Parse values into device structure according to requested settings */
    if (desired_settings & BME680_GAS_MEAS_SEL) {
        rslt = get_gas_config(dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[WARN] bme680_get_sensor_settings: get_gas_config returned %d (continuing)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
        } else {
        	BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_settings: gas config updated from sensor\r\n");
        	app_fault_delay_ms(20);
        }
    }

    if (desired_settings & BME680_FILTER_SEL) {
        dev->tph_sett.filter = BME680_GET_BITS(data_array[BME680_REG_FILTER_INDEX], BME680_FILTER);
        BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_settings: filter = %u\r\n", (unsigned)dev->tph_sett.filter);
        app_fault_delay_ms(20);
    }

    if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
        dev->tph_sett.os_temp = BME680_GET_BITS(data_array[BME680_REG_TEMP_INDEX], BME680_OST);
        dev->tph_sett.os_pres = BME680_GET_BITS(data_array[BME680_REG_PRES_INDEX], BME680_OSP);
        BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_settings: os_temp=%u os_pres=%u\r\n",
                   (unsigned)dev->tph_sett.os_temp, (unsigned)dev->tph_sett.os_pres);
        app_fault_delay_ms(20);
    }

    if (desired_settings & BME680_OSH_SEL) {
        dev->tph_sett.os_hum = BME680_GET_BITS_POS_0(data_array[BME680_REG_HUM_INDEX], BME680_OSH);
        BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_settings: os_hum=%u\r\n", (unsigned)dev->tph_sett.os_hum);
        app_fault_delay_ms(20);
    }

    if (desired_settings & BME680_HCNTRL_SEL) {
        dev->gas_sett.heatr_ctrl = BME680_GET_BITS_POS_0(data_array[BME680_REG_HCTRL_INDEX], BME680_HCTRL);
        BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_settings: heatr_ctrl=%u\r\n", (unsigned)dev->gas_sett.heatr_ctrl);
        app_fault_delay_ms(20);
    }

    if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
        dev->gas_sett.nb_conv = BME680_GET_BITS_POS_0(data_array[BME680_REG_NBCONV_INDEX], BME680_NBCONV);
        dev->gas_sett.run_gas = BME680_GET_BITS(data_array[BME680_REG_RUN_GAS_INDEX], BME680_RUN_GAS);
        BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_settings: nb_conv=%u run_gas=%u\r\n",
                   (unsigned)dev->gas_sett.nb_conv, (unsigned)dev->gas_sett.run_gas);
        app_fault_delay_ms(20);
    }

    BME680_LOG_PRINTF("[INFO] bme680_get_sensor_settings: exit (result=%d)\r\n", (int)BME680_OK);
    app_fault_delay_ms(20);
    return BME680_OK;
}

/*!
 * @brief This API is used to set the power mode of the sensor.
 *
 * Behavior preserved exactly:
 *  - Repeatedly poll the T/P mode register until sensor is in SLEEP.
 *  - If dev->power_mode != SLEEP, write the requested power mode after sensor is sleeping.
 */
int8_t bme680_set_sensor_mode(struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL;
    uint8_t tmp_pow_mode = 0;
    uint8_t pow_mode = 0;
    const uint8_t reg_addr = BME680_CONF_T_P_MODE_ADDR;

    /* Step 1 — Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_mode: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* Step 2 — Poll until device enters sleep mode */
    do {
        rslt = bme680_get_regs(BME680_CONF_T_P_MODE_ADDR, &tmp_pow_mode, 1, dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_mode: failed to read power-mode reg (err=%d)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
            return rslt;
        }

        pow_mode = (uint8_t)(tmp_pow_mode & BME680_MODE_MSK);

        if (pow_mode != BME680_SLEEP_MODE) {
            /* Put sensor to sleep (clear mode bits) */
            uint8_t sleep_val = (uint8_t)(tmp_pow_mode & (~BME680_MODE_MSK));
            rslt = bme680_set_regs(&reg_addr, &sleep_val, 1, dev);
            if (rslt != BME680_OK) {
            	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_mode: failed to write sleep mode (err=%d)\r\n", (int)rslt);
            	app_fault_delay_ms(20);
                return rslt;
            }
            /* Wait a bit before polling again */
            dev->delay_ms(BME680_POLL_PERIOD_MS);
        }
        /* loop until pow_mode == SLEEP */
    } while (pow_mode != BME680_SLEEP_MODE);

    /* Step 3 — If requested mode is not sleep, program it now */
    if (dev->power_mode != BME680_SLEEP_MODE) {
        uint8_t new_val = (uint8_t)((tmp_pow_mode & ~BME680_MODE_MSK) | (dev->power_mode & BME680_MODE_MSK));
        if (rslt == BME680_OK) {
            rslt = bme680_set_regs(&reg_addr, &new_val, 1, dev);
            if (rslt != BME680_OK) {
            	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_mode: failed to set requested power_mode=%u (err=%d)\r\n",
                           (unsigned)dev->power_mode, (int)rslt);
            	app_fault_delay_ms(20);
                return rslt;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to get the power mode of the sensor.
 */
int8_t bme680_get_sensor_mode(struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL;
    uint8_t mode = 0;

    BME680_LOG_PRINTF("[INFO] bme680_get_sensor_mode: entry\r\n");
    app_fault_delay_ms(20);

    /* Step 1 — Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_sensor_mode: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }
    BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_mode: null_ptr_check passed\r\n");
    app_fault_delay_ms(20);

    /* Step 2 — Read the T/P mode register */
    rslt = bme680_get_regs(BME680_CONF_T_P_MODE_ADDR, &mode, 1, dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_sensor_mode: failed to read mode register (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return rslt;
    }
    BME680_LOG_PRINTF("[DEBUG] bme680_get_sensor_mode: raw mode reg=0x%02X\r\n", mode);
    app_fault_delay_ms(20);

    /* Step 3 — Mask out unrelated bits and store the power mode */
    dev->power_mode = (uint8_t)(mode & BME680_MODE_MSK);
    BME680_LOG_PRINTF("[INFO] bme680_get_sensor_mode: power_mode=%u\r\n", (unsigned)dev->power_mode);
    app_fault_delay_ms(20);

    BME680_LOG_PRINTF("[INFO] bme680_get_sensor_mode: exit (result=%d)\r\n", (int)rslt);
    app_fault_delay_ms(20);
    return rslt;
}


/*!
 * @brief This API is used to set the profile duration of the sensor.
 *
 * Notes:
 *  - Behavior unchanged: computes approximate TPH duration (ms) and subtracts
 *    it from the requested profile duration to produce heater duration.
 *  - No logging (math-only function).
 */
void bme680_set_profile_dur(uint16_t duration, struct bme680_dev *dev)
{
    /* Calculate durations in microseconds then convert to ms as in original logic */
    uint32_t tph_dur_us = 0U;       /* TPH-related duration in microseconds */
    uint32_t meas_cycles = 0U;      /* effective number of measurement cycles */

    /* map oversampling setting -> measurement cycles (same mapping you used) */
    const uint8_t os_to_meas_cycles[6] = { 0U, 1U, 2U, 4U, 8U, 16U };

    /* sum cycles for temperature, pressure and humidity oversampling */
    meas_cycles  = (uint32_t)os_to_meas_cycles[dev->tph_sett.os_temp];
    meas_cycles += (uint32_t)os_to_meas_cycles[dev->tph_sett.os_pres];
    meas_cycles += (uint32_t)os_to_meas_cycles[dev->tph_sett.os_hum];

    /* TPH measurement duration (microseconds) */
    tph_dur_us  = meas_cycles * UINT32_C(1963);      /* base per-cycle time */
    tph_dur_us += UINT32_C(477 * 4);                 /* TPH switching time */
    tph_dur_us += UINT32_C(477 * 5);                 /* gas measurement time */
    tph_dur_us += UINT32_C(500);                     /* rounding pad */

    /* Convert to milliseconds (integer division) */
    tph_dur_us /= UINT32_C(1000);

    /* add wake-up time (1 ms) */
    tph_dur_us += UINT32_C(1);

    /* final heater duration = requested duration - computed TPH duration (both in ms) */
    dev->gas_sett.heatr_dur = (uint16_t)((uint32_t)duration - (uint32_t)tph_dur_us);
}


/*!
 * @brief This API is used to get the profile duration of the sensor.
 *
 * @param[out] duration Pointer to receive the profile duration (ms).
 * @param[in]  dev      Pointer to device structure (const).
 * @return BME680_OK on success or one of the BME680_E_* error codes.
 */
int8_t bme680_get_profile_dur(uint16_t *duration, const struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL; /* default */
    uint32_t tph_dur_ms = 0U;        /* computed TPH duration in ms */
    uint32_t meas_cycles = 0U;
    const uint8_t os_to_meas_cycles[6] = { 0, 1, 2, 4, 8, 16 };

    /* Validate pointers */
    if (duration == NULL) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_profile_dur: NULL duration pointer\r\n");
    	app_fault_delay_ms(20);
        return BME680_E_INVALID_LENGTH;
    }

    rslt = null_ptr_check((struct bme680_dev *)dev); /* null_ptr_check expects non-const */
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_profile_dur: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* Sum measurement cycles from oversampling settings */
    meas_cycles  = (uint32_t)os_to_meas_cycles[dev->tph_sett.os_temp];
    meas_cycles += (uint32_t)os_to_meas_cycles[dev->tph_sett.os_pres];
    meas_cycles += (uint32_t)os_to_meas_cycles[dev->tph_sett.os_hum];

    /* Compute TPH measurement duration (microseconds -> ms) using same constants as original */
    tph_dur_ms  = meas_cycles * UINT32_C(1963);    /* microseconds contribution per cycle */
    tph_dur_ms += UINT32_C(477 * 4);               /* TPH switching duration (us) */
    tph_dur_ms += UINT32_C(477 * 5);               /* Gas measurement duration (us) */
    tph_dur_ms += UINT32_C(500);                   /* rounding pad (us) */
    tph_dur_ms /= UINT32_C(1000);                  /* convert to ms */
    tph_dur_ms += UINT32_C(1);                     /* wake-up duration (ms) */

    /* Base profile duration is TPH duration; add heater duration if run_gas enabled */
    *duration = (uint16_t) tph_dur_ms;
    if (dev->gas_sett.run_gas) {
        /* safe addition: clamp to 0xFFFF on overflow (defensive) */
        uint32_t total = (uint32_t)(*duration) + (uint32_t)dev->gas_sett.heatr_dur;
        if (total > UINT16_MAX) total = UINT16_MAX;
        *duration = (uint16_t)total;
    }

    return BME680_OK;
}


/*!
 * @brief This API reads the pressure, temperature and humidity and gas data
 * from the sensor, compensates the data and stores it in the bme680_field_data
 * structure instance passed by the user.
 */
int8_t bme680_get_sensor_data(struct bme680_field_data *data, struct bme680_dev *dev)
{
    int8_t rslt;

    /*log_printf("[INFO] bme680_get_sensor_data: entry\r\n");*/

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);
    if (rslt == BME680_OK) {
        /*log_printf("[DEBUG] bme680_get_sensor_data: null_ptr_check passed\r\n");*/

        /* Reading the sensor data in forced mode only */
        rslt = read_field_data(data, dev);
        if (rslt == BME680_OK) {
            if (data->status & BME680_NEW_DATA_MSK) {
                dev->new_fields = 1;
                /*log_printf("[DEBUG] bme680_get_sensor_data: new_fields = 1 (status=0x%02X)\r\n",
                           (unsigned)data->status);*/
            } else {
                dev->new_fields = 0;
                /*log_printf("[DEBUG] bme680_get_sensor_data: new_fields = 0 (status=0x%02X)\r\n",
                           (unsigned)data->status);*/
            }
        } else {
            /*log_printf("[ERROR] bme680_get_sensor_data: read_field_data failed (err=%d)\r\n", (int)rslt);*/
        }
    } else {
        /*log_printf("[ERROR] bme680_get_sensor_data: null_ptr_check failed (err=%d)\r\n", (int)rslt);*/
    }

    /*log_printf("[INFO] bme680_get_sensor_data: exit (result=%d)\r\n", (int)rslt);*/
    return rslt;
}


/*!
 * @brief This internal API is used to read the calibrated data from the sensor.
 *
 * Note: This routine preserves the original behaviour and decoding logic.
 *       No logging is emitted (math/parsing only).
 */
static int8_t get_calib_data(struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL;
    uint8_t coeff_array[BME680_COEFF_SIZE] = { 0 };
    uint8_t temp_var = 0;
    uint8_t i;

    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
        return rslt;
    }

    /*
     * IMPORTANT:
     * Calibration read is done byte-by-byte on purpose.
     *
     * Your newer DMA block-read path is producing zero for the early
     * temperature calibration bytes. Byte reads force the safe blocking
     * path inside i2c_dma_read(), avoiding the >2-byte DMA RX path during
     * BME680 init.
     */

    for (i = 0U; i < BME680_COEFF_ADDR1_LEN; i++) {
        rslt = bme680_get_regs((uint8_t)(BME680_COEFF_ADDR1 + i),
                               &coeff_array[i],
                               1U,
                               dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] get_calib_data: coeff block1 byte %u read failed err=%d\r\n",
                       (unsigned int)i,
                       (int)rslt);
        	app_fault_delay_ms(20);
        	app_fault_delay_ms(2);
            return rslt;
        }
    }

    for (i = 0U; i < BME680_COEFF_ADDR2_LEN; i++) {
        rslt = bme680_get_regs((uint8_t)(BME680_COEFF_ADDR2 + i),
                               &coeff_array[BME680_COEFF_ADDR1_LEN + i],
                               1U,
                               dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] get_calib_data: coeff block2 byte %u read failed err=%d\r\n",
                       (unsigned int)i,
                       (int)rslt);
        	app_fault_delay_ms(20);
        	app_fault_delay_ms(2);
            return rslt;
        }
    }

//    log_printf("[BME680 CAL RAW] coeff[0..8]: "
//               "%02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//               coeff_array[0],
//               coeff_array[1],
//               coeff_array[2],
//               coeff_array[3],
//               coeff_array[4],
//               coeff_array[5],
//               coeff_array[6],
//               coeff_array[7],
//               coeff_array[8]);
//    tim2_delay_ms(200);

    /*
     * Temperature related coefficients.
     */
    dev->calib.par_t1 = (uint16_t)BME680_CONCAT_BYTES(coeff_array[BME680_T1_MSB_REG],
                                                      coeff_array[BME680_T1_LSB_REG]);

    dev->calib.par_t2 = (int16_t)BME680_CONCAT_BYTES(coeff_array[BME680_T2_MSB_REG],
                                                     coeff_array[BME680_T2_LSB_REG]);

    dev->calib.par_t3 = (int8_t)coeff_array[BME680_T3_REG];

    /*
     * Validate temperature calibration.
     * par_t2 should not be zero on a normal working BME680.
     * par_t3 can theoretically be small, but if both t2 and t3 are zero,
     * treat the calibration block as bad.
     */
    if ((dev->calib.par_t2 == 0) && (dev->calib.par_t3 == 0)) {
    	BME680_LOG_PRINTF("[ERROR] get_calib_data: invalid temperature calibration: par_t2=%d par_t3=%d\r\n",
                   (int)dev->calib.par_t2,
                   (int)dev->calib.par_t3);
    	app_fault_delay_ms(20);
    	app_fault_delay_ms(2);

        BME680_LOG_PRINTF("[ERROR] raw T2/T3 bytes: T2_MSB=0x%02X T2_LSB=0x%02X T3=0x%02X\r\n",
                   coeff_array[BME680_T2_MSB_REG],
                   coeff_array[BME680_T2_LSB_REG],
                   coeff_array[BME680_T3_REG]);
        app_fault_delay_ms(20);
        app_fault_delay_ms(2);

        return BME680_E_COM_FAIL;
    }

    /*
     * Pressure related coefficients.
     */
    dev->calib.par_p1 = (uint16_t)BME680_CONCAT_BYTES(coeff_array[BME680_P1_MSB_REG],
                                                      coeff_array[BME680_P1_LSB_REG]);

    dev->calib.par_p2 = (int16_t)BME680_CONCAT_BYTES(coeff_array[BME680_P2_MSB_REG],
                                                     coeff_array[BME680_P2_LSB_REG]);

    dev->calib.par_p3  = (int8_t)coeff_array[BME680_P3_REG];

    dev->calib.par_p4  = (int16_t)BME680_CONCAT_BYTES(coeff_array[BME680_P4_MSB_REG],
                                                      coeff_array[BME680_P4_LSB_REG]);

    dev->calib.par_p5  = (int16_t)BME680_CONCAT_BYTES(coeff_array[BME680_P5_MSB_REG],
                                                      coeff_array[BME680_P5_LSB_REG]);

    dev->calib.par_p6  = (int8_t)coeff_array[BME680_P6_REG];
    dev->calib.par_p7  = (int8_t)coeff_array[BME680_P7_REG];

    dev->calib.par_p8  = (int16_t)BME680_CONCAT_BYTES(coeff_array[BME680_P8_MSB_REG],
                                                      coeff_array[BME680_P8_LSB_REG]);

    dev->calib.par_p9  = (int16_t)BME680_CONCAT_BYTES(coeff_array[BME680_P9_MSB_REG],
                                                      coeff_array[BME680_P9_LSB_REG]);

    dev->calib.par_p10 = (uint8_t)coeff_array[BME680_P10_REG];

    /*
     * Humidity related coefficients.
     */
    dev->calib.par_h1 = (uint16_t)(((uint16_t)coeff_array[BME680_H1_MSB_REG] << BME680_HUM_REG_SHIFT_VAL) |
                                    (coeff_array[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));

    dev->calib.par_h2 = (uint16_t)(((uint16_t)coeff_array[BME680_H2_MSB_REG] << BME680_HUM_REG_SHIFT_VAL) |
                                    (coeff_array[BME680_H2_LSB_REG] >> BME680_HUM_REG_SHIFT_VAL));

    dev->calib.par_h3 = (int8_t)coeff_array[BME680_H3_REG];
    dev->calib.par_h4 = (int8_t)coeff_array[BME680_H4_REG];
    dev->calib.par_h5 = (int8_t)coeff_array[BME680_H5_REG];
    dev->calib.par_h6 = (uint8_t)coeff_array[BME680_H6_REG];
    dev->calib.par_h7 = (int8_t)coeff_array[BME680_H7_REG];

    /*
     * Gas heater related coefficients.
     */
    dev->calib.par_gh1 = (int8_t)coeff_array[BME680_GH1_REG];

    dev->calib.par_gh2 = (int16_t)BME680_CONCAT_BYTES(coeff_array[BME680_GH2_MSB_REG],
                                                      coeff_array[BME680_GH2_LSB_REG]);

    dev->calib.par_gh3 = (int8_t)coeff_array[BME680_GH3_REG];

    /*
     * Additional single-byte calibration registers.
     */
    rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_RANGE_ADDR, &temp_var, 1U, dev);
    if (rslt != BME680_OK) {
        return rslt;
    }

    dev->calib.res_heat_range = (uint8_t)((temp_var & BME680_RHRANGE_MSK) / 16U);

    rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_VAL_ADDR, &temp_var, 1U, dev);
    if (rslt != BME680_OK) {
        return rslt;
    }

    dev->calib.res_heat_val = (int8_t)temp_var;

    rslt = bme680_get_regs(BME680_ADDR_RANGE_SW_ERR_ADDR, &temp_var, 1U, dev);
    if (rslt != BME680_OK) {
        return rslt;
    }

    dev->calib.range_sw_err = ((int8_t)temp_var & (int8_t)BME680_RSERROR_MSK) / 16;

    return BME680_OK;
}


/*!
 * @brief This internal API is used to set the gas configuration of the sensor.
 */
static int8_t set_gas_config(struct bme680_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr[2] = { 0 };
    uint8_t reg_data[2] = { 0 };

    /* Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] set_gas_config: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* Only allowed in forced mode: prepare heater resistance and heater duration */
    if (dev->power_mode == BME680_FORCED_MODE) {
        reg_addr[0] = BME680_RES_HEAT0_ADDR;
        reg_data[0] = calc_heater_res(dev->gas_sett.heatr_temp, dev);

        reg_addr[1] = BME680_GAS_WAIT0_ADDR;
        reg_data[1] = calc_heater_dur(dev->gas_sett.heatr_dur);

        /* Reset conversion counter as per original behavior */
        dev->gas_sett.nb_conv = 0;

        /* Write the two registers */
        rslt = bme680_set_regs(reg_addr, reg_data, 2, dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] set_gas_config: bme680_set_regs failed (err=%d)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
        }
    } else {
        /* Not allowed unless forced mode — return the same warning/error code as original design */
        rslt = BME680_W_DEFINE_PWR_MODE;
        BME680_LOG_PRINTF("[WARN] set_gas_config: device not in FORCED mode, skipping gas config (code=%d)\r\n", (int)rslt);
        app_fault_delay_ms(20);
    }

    return rslt;
}

/*!
 * @brief This internal API is used to get the gas configuration of the sensor.
 * @note heatr_temp and heatr_dur values are currently register data
 * and not the actual values set
 */
/*!
 * @brief This internal API is used to get the gas configuration of the sensor.
 * @note heatr_temp and heatr_dur values are currently register data
 * and not the actual values set
 */
static int8_t get_gas_config(struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL;
    const uint8_t reg_addr1 = BME680_ADDR_SENS_CONF_START; /* starting address for burst read */
    const uint8_t reg_addr2 = BME680_ADDR_GAS_CONF_START;
    uint8_t reg_data = 0;

    BME680_LOG_PRINTF("[INFO] get_gas_config: entry\r\n");
    app_fault_delay_ms(20);

    /* Step 1 — Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] get_gas_config: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }
    BME680_LOG_PRINTF("[DEBUG] get_gas_config: null_ptr_check passed\r\n");
    app_fault_delay_ms(20);

    /* Step 2 — If SPI, ensure correct memory page */
    if (dev->intf == BME680_SPI_INTF) {
        rslt = set_mem_page(reg_addr1, dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] get_gas_config: set_mem_page failed (err=%d)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
            return rslt;
        }
        BME680_LOG_PRINTF("[DEBUG] get_gas_config: SPI memory page set for reg 0x%02X\r\n", reg_addr1);
        app_fault_delay_ms(20);
    }

    /* Step 3 — Read sensor config register (heater temp) */
    rslt = bme680_get_regs(reg_addr1, &reg_data, 1, dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] get_gas_config: failed to read reg 0x%02X (err=%d)\r\n", reg_addr1, (int)rslt);
    	app_fault_delay_ms(20);
        return rslt;
    }
    dev->gas_sett.heatr_temp = reg_data;
    BME680_LOG_PRINTF("[DEBUG] get_gas_config: read reg 0x%02X -> heatr_temp=0x%02X\r\n", reg_addr1, reg_data);
    app_fault_delay_ms(20);

    /* Step 4 — Read gas config register (heater duration) */
    rslt = bme680_get_regs(reg_addr2, &reg_data, 1, dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] get_gas_config: failed to read reg 0x%02X (err=%d)\r\n", reg_addr2, (int)rslt);
    	app_fault_delay_ms(20);
        return rslt;
    }
    dev->gas_sett.heatr_dur = reg_data;
    BME680_LOG_PRINTF("[DEBUG] get_gas_config: read reg 0x%02X -> heatr_dur=0x%02X\r\n", reg_addr2, reg_data);
    app_fault_delay_ms(20);

    BME680_LOG_PRINTF("[INFO] get_gas_config: exit (result=%d)\r\n", (int)rslt);
    app_fault_delay_ms(20);
    return rslt;
}


#ifndef BME680_FLOAT_POINT_COMPENSATION

/*!
 * @brief This internal API is used to calculate the temperature value.
 *
 * @note Preserves the original fixed-point algorithm and bit-shift sequence.
 *       No logging or side-effect changes — only minor style/clarity improvements.
 */
static int16_t calc_temperature(uint32_t temp_adc, struct bme680_dev *dev)
{
    int64_t var1 = 0;
    int64_t var2 = 0;
    int64_t var3 = 0;
    int16_t calc_temp = 0;

    /* Replica of the original fixed-point algorithm:
     * var1 = (temp_adc >> 3) - (par_t1 << 1)
     * var2 = (var1 * par_t2) >> 11
     * var3 = (((var1 >> 1) * (var1 >> 1)) >> 12) * (par_t3 << 4) >> 14
     * t_fine = var2 + var3
     * calc_temp = ((t_fine * 5) + 128) >> 8
     */

    var1 = ((int64_t)temp_adc >> 3) - ((int64_t)dev->calib.par_t1 << 1);
    var2 = (var1 * (int64_t)dev->calib.par_t2) >> 11;

    var3 = (var1 >> 1);
    var3 = (var3 * var3) >> 12;
    var3 = (var3 * ((int64_t)dev->calib.par_t3 << 4)) >> 14;

    dev->calib.t_fine = (int32_t)(var2 + var3);
    calc_temp = (int16_t)(((dev->calib.t_fine * 5) + 128) >> 8);

    return calc_temp;
}



/*!
 * @brief This internal API is used to calculate the pressure value.
 *
 * Note: Preserves the original fixed-point algorithm and shifts precisely.
 *       No logging; pure math helper.
 */
static uint32_t calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev)
{
    int32_t var1 = 0;
    int32_t var2 = 0;
    int32_t var3 = 0;
    int32_t pressure_comp = 0;

    /* Following the original algorithm and shift sequence exactly */

    var1 = (((int32_t)dev->calib.t_fine) >> 1) - 64000;

    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)dev->calib.par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)dev->calib.par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)dev->calib.par_p4 << 16);

    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)dev->calib.par_p3 << 5)) >> 3) +
           (((int32_t)dev->calib.par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)dev->calib.par_p1) >> 15;

    pressure_comp = 1048576 - (int32_t)pres_adc;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125U));

    if (pressure_comp >= BME680_MAX_OVERFLOW_VAL) {
        pressure_comp = (int32_t)((pressure_comp / var1) << 1);
    } else {
        pressure_comp = (int32_t)((pressure_comp << 1) / var1);
    }

    var1 = ((int32_t)dev->calib.par_p9 * (int32_t)(((pressure_comp >> 3) * (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) * (int32_t)dev->calib.par_p8) >> 13;
    var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
            (int32_t)(pressure_comp >> 8) * (int32_t)dev->calib.par_p10) >> 17;

    pressure_comp = pressure_comp + ((var1 + var2 + var3 + ((int32_t)dev->calib.par_p7 << 7)) >> 4);

    return (uint32_t)pressure_comp;
}

/*!
 * @brief This internal API is used to calculate the humidity value.
 *
 * Note: preserves original integer/fixed-point algorithm and clamps output to 0..100000.
 */
static uint32_t calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev)
{
    int32_t var1 = 0;
    int32_t var2 = 0;
    int32_t var3 = 0;
    int32_t var4 = 0;
    int32_t var5 = 0;
    int32_t var6 = 0;
    int32_t temp_scaled = 0;
    int32_t calc_hum = 0;

    /* temp_scaled = ((t_fine * 5) + 128) >> 8  (same as temperature calc) */
    temp_scaled = (((int32_t)dev->calib.t_fine * 5) + 128) >> 8;

    /* var1: humidity ADC minus offset and humidity/temperature correction term */
    var1 = (int32_t)hum_adc - ((int32_t)dev->calib.par_h1 * 16);
    var1 = var1 - (((temp_scaled * (int32_t)dev->calib.par_h3) / 100) >> 1);

    /* var2: combined factor from calibration coeffs and scaled temperature */
    var2 = (int32_t)dev->calib.par_h2 *
           ( ((temp_scaled * (int32_t)dev->calib.par_h4) / 100) +
             ( ((temp_scaled * ((temp_scaled * (int32_t)dev->calib.par_h5) / 100)) >> 6) / 100 ) +
             (1 << 14) );
    var2 = var2 >> 10;

    /* var3 = var1 * var2 */
    var3 = var1 * var2;

    /* var4: par_h6 and par_h7 temperature-dependent term */
    var4 = ((int32_t)dev->calib.par_h6 << 7);
    var4 = (var4 + (temp_scaled * (int32_t)dev->calib.par_h7) / 100) >> 4;

    /* var5 and var6 are higher-order corrections */
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;

    /* final humidity calculation scaled to 1/1000 %RH (so 100000 == 100.000 %RH) */
    calc_hum = (((var3 + var6) >> 10) * 1000) >> 12;

    /* clamp to [0, 100000] (0..100.000 %RH) */
    if (calc_hum > 100000) {
        calc_hum = 100000;
    } else if (calc_hum < 0) {
        calc_hum = 0;
    }

    return (uint32_t)calc_hum;
}

/*!
 * @brief This internal API is used to calculate the Gas Resistance value.
 *
 * Note: Preserves the original integer/fixed-point algorithm exactly.
 *       Lookup tables are placed in static const to avoid stack pressure.
 *       No logging — pure math helper.
 */
static uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev)
{
    int64_t var1 = 0;
    int64_t var2 = 0;
    int64_t var3 = 0;
    uint32_t calc_gas_res = 0;

    /* Lookup tables (static const to keep them in RO data, not on stack) */
    static const uint32_t lookupTable1[16] = {
        UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
        UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
        UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
        UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647)
    };

    static const uint32_t lookupTable2[16] = {
        UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
        UINT32_C(255744255),  UINT32_C(127110228),  UINT32_C(64000000),   UINT32_C(32258064),
        UINT32_C(16016016),   UINT32_C(8000000),    UINT32_C(4000000),    UINT32_C(2000000),
        UINT32_C(1000000),    UINT32_C(500000),     UINT32_C(250000),     UINT32_C(125000)
    };

    /* Reproduce original fixed-point sequence:
     * var1 = ((1340 + 5 * range_sw_err) * lookupTable1[gas_range]) >> 16
     * var2 = (gas_res_adc << 15) - 16777216 + var1
     * var3 = (lookupTable2[gas_range] * var1) >> 9
     * calc = (var3 + (var2 >> 1)) / var2
     */
    var1 = ((int64_t)1340 + ((int64_t)5 * (int64_t)dev->calib.range_sw_err)) *
           (int64_t)lookupTable1[gas_range];
    var1 = var1 >> 16;

    var2 = ((int64_t)gas_res_adc << 15) - (int64_t)16777216 + var1;

    var3 = ((int64_t)lookupTable2[gas_range] * var1) >> 9;

    /* Match original behaviour exactly (no zero-division guard) */
    calc_gas_res = (uint32_t)((var3 + (var2 >> 1)) / var2);

    return calc_gas_res;
}

/*!
 * @brief This internal API is used to calculate the Heat Resistance value.
 *
 * Note: Preserves the original integer arithmetic and behavior exactly.
 *       No logging — pure math helper. Temperature is capped at 400°C as in original.
 */
static uint8_t calc_heater_res(uint16_t temp, const struct bme680_dev *dev)
{
    int32_t var1 = 0;
    int32_t var2 = 0;
    int32_t var3 = 0;
    int32_t var4 = 0;
    int32_t var5 = 0;
    int32_t heatr_res_x100 = 0;
    uint8_t heatr_res = 0;

    /* Cap requested heater temperature at 400 (preserve original behaviour) */
    if (temp > 400U) {
        temp = 400U;
    }

    /*
     * Original algorithm (preserved):
     * var1 = (((int32_t) dev->amb_temp * dev->calib.par_gh3) / 1000) * 256;
     * var2 = (dev->calib.par_gh1 + 784) * (((((dev->calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
     * var3 = var1 + (var2 / 2);
     * var4 = (var3 / (dev->calib.res_heat_range + 4));
     * var5 = (131 * dev->calib.res_heat_val) + 65536;
     * heatr_res_x100 = (((var4 / var5) - 250) * 34);
     * heatr_res = (heatr_res_x100 + 50) / 100;
     */

    var1 = (((int32_t)dev->amb_temp * (int32_t)dev->calib.par_gh3) / 1000) * 256;

    var2 = (int32_t)(dev->calib.par_gh2 + 154009);
    var2 = (int32_t)((((int64_t)var2 * temp * 5) / 100) + 3276800);
    var2 = (int32_t)((int32_t)(dev->calib.par_gh1 + 784) * (var2 / 10));

    var3 = var1 + (var2 / 2);

    /* dev->calib.res_heat_range + 4 follows original spec */
    var4 = var3 / (int32_t)(dev->calib.res_heat_range + 4);

    var5 = (131 * (int32_t)dev->calib.res_heat_val) + 65536;

    /* Multiply/divide sequence preserved exactly */
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);

    /* Round to nearest by adding 50 before dividing by 100 (same as original) */
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

#else
/*!
 * @brief This internal API is used to calculate the
 * temperature value in float format
 *
 * Note: Preserves original float math exactly. No logging.
 */
static float calc_temperature(uint32_t temp_adc, struct bme680_dev *dev)
{
    float var1 = 0.0f;
    float var2 = 0.0f;
    float calc_temp = 0.0f;

    /* var1 = (adc/16384 - par_t1/1024) * par_t2 */
    var1 = (((float)temp_adc / 16384.0f) - ((float)dev->calib.par_t1 / 1024.0f)) *
           ((float)dev->calib.par_t2);

    /* var2 = ( (adc/131072 - par_t1/8192)^2 ) * (par_t3 * 16) */
    {
        float tmp = ((float)temp_adc / 131072.0f) - ((float)dev->calib.par_t1 / 8192.0f);
        var2 = (tmp * tmp) * ((float)dev->calib.par_t3 * 16.0f);
    }

    /* t_fine (float assignment mirrors original behavior) */
    dev->calib.t_fine = (int32_t)(var1 + var2); /* keep same storage as original code expects */

    /* final compensated temperature = t_fine / 5120 */
    calc_temp = (var1 + var2) / 5120.0f;

    return calc_temp;
}

/*!
 * @brief This internal API is used to calculate the
 * pressure value in float format
 */
static float calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev)
{
    float var1 = 0.0f;
    float var2 = 0.0f;
    float var3 = 0.0f;
    float calc_pres = 0.0f;

    /* Step 1: precompute base variables */
    var1 = (((float)dev->calib.t_fine / 2.0f) - 64000.0f);

    var2 = var1 * var1 * ((float)dev->calib.par_p6 / 131072.0f);
    var2 = var2 + (var1 * ((float)dev->calib.par_p5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)dev->calib.par_p4) * 65536.0f);

    var1 = (((((float)dev->calib.par_p3 * var1 * var1) / 16384.0f) +
            ((float)dev->calib.par_p2 * var1)) / 524288.0f);
    var1 = (1.0f + (var1 / 32768.0f)) * ((float)dev->calib.par_p1);

    /* Base compensated value */
    calc_pres = 1048576.0f - (float)pres_adc;

    /* Avoid division by zero; original used (int)var1 check */
    if ((int)var1 != 0) {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);

        var1 = ((float)dev->calib.par_p9) * calc_pres * calc_pres / 2147483648.0f;
        var2 = calc_pres * (((float)dev->calib.par_p8) / 32768.0f);
        var3 = (calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) *
               ((float)dev->calib.par_p10 / 131072.0f);

        calc_pres = calc_pres + (var1 + var2 + var3 + ((float)dev->calib.par_p7 * 128.0f)) / 16.0f;
    } else {
        calc_pres = 0.0f;
    }

    return calc_pres;
}

/*!
 * @brief This internal API is used to calculate the
 * humidity value in float format
 */
static float calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev)
{
    float calc_hum = 0.0f;
    float var1 = 0.0f;
    float var2 = 0.0f;
    float var3 = 0.0f;
    float var4 = 0.0f;
    float temp_comp = 0.0f;

    /* compensated temperature (same as floating temperature helper) */
    temp_comp = ((float)dev->calib.t_fine) / 5120.0f;

    /* var1: raw adjusted humidity */
    var1 = (float)hum_adc - (((float)dev->calib.par_h1 * 16.0f) +
            (((float)dev->calib.par_h3 / 2.0f) * temp_comp));

    /* var2: scaling by calibration and temperature dependence */
    var2 = var1 * ( ( (float)dev->calib.par_h2 / 262144.0f ) *
                    (1.0f + ((float)dev->calib.par_h4 / 16384.0f) * temp_comp +
                     ((float)dev->calib.par_h5 / 1048576.0f) * temp_comp * temp_comp ) );

    var3 = (float)dev->calib.par_h6 / 16384.0f;
    var4 = (float)dev->calib.par_h7 / 2097152.0f;

    calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    /* Clamp to 0..100% (float) */
    if (calc_hum > 100.0f) {
        calc_hum = 100.0f;
    } else if (calc_hum < 0.0f) {
        calc_hum = 0.0f;
    }

    return calc_hum;
}

/*!
 * @brief This internal API is used to calculate the
 * gas resistance value in float format
 */
static float calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev)
{
    float calc_gas_res = 0.0f;
    float var1 = 0.0f;
    float var2 = 0.0f;
    float var3 = 0.0f;

    /* Lookup tables (kept const in RO memory) */
    static const float lookup_k1_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f,
        0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
    };

    static const float lookup_k2_range[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f,
        -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    var1 = 1340.0f + (5.0f * (float)dev->calib.range_sw_err);
    var2 = var1 * (1.0f + lookup_k1_range[gas_range] / 100.0f);
    var3 = 1.0f + (lookup_k2_range[gas_range] / 100.0f);

    /* Recreate the original formula using float math:
     * calc_gas_res = 1 / ( var3 * 0.000000125f * (1<<gas_range) * ( ((gas_res_adc - 512)/var2) + 1 ) )
     */
    {
        const float factor = 0.000000125f * (float)(1u << gas_range);
        const float adc_term = (((float)gas_res_adc) - 512.0f) / var2 + 1.0f;
        /* protect against accidental zero/div by keeping behavior identical (original had no guard) */
        calc_gas_res = 1.0f / (var3 * factor * adc_term);
    }

    return calc_gas_res;
}

/*!
 * @brief This internal API is used to calculate the
 * heater resistance value in float format
 */
static float calc_heater_res(uint16_t temp, const struct bme680_dev *dev)
{
    float var1 = 0.0f;
    float var2 = 0.0f;
    float var3 = 0.0f;
    float var4 = 0.0f;
    float var5 = 0.0f;
    float res_heat = 0.0f;
    float res_heat_range_div = 0.0f;
    float res_heat_val_term = 0.0f;

    /* Cap temperature at 400°C (preserve original behaviour) */
    if (temp > 400U) {
        temp = 400U;
    }

    var1 = ((float)dev->calib.par_gh1 / 16.0f) + 49.0f;
    var2 = (((float)dev->calib.par_gh2 / 32768.0f) * 0.0005f) + 0.00235f;
    var3 = (float)dev->calib.par_gh3 / 1024.0f;

    /* Compute intermediate values exactly as original */
    var4 = var1 * (1.0f + (var2 * (float)temp));
    var5 = var4 + (var3 * (float)dev->amb_temp);

    /* following original formula — keep denominator expressions as floats */
    res_heat_range_div = 4.0f / (4.0f + (float)dev->calib.res_heat_range);
    res_heat_val_term = 1.0f / (1.0f + ((float)dev->calib.res_heat_val * 0.002f));

    /* final expression matches original cast-to-uint8_t behavior, result returned as float */
    res_heat = (float)( (uint8_t)(3.4f * ((var5 * res_heat_range_div * res_heat_val_term) - 25.0f)) );

    return res_heat;
}

#endif


/*!
 * @brief This internal API is used to calculate the Heat duration value.
 *
 * Behavior preserved:
 *  - Durations >= 0xFC0 map to 0xFF (max).
 *  - Otherwise repeatedly divide duration by 4 until it fits <= 0x3F,
 *    counting the divisions as factor; result = dur + factor*64.
 */
static uint8_t calc_heater_dur(uint16_t dur)
{
    uint32_t remaining = (uint32_t)dur;
    uint8_t factor = 0;
    uint8_t durval = 0;

    /* Max duration shortcut (preserve original behavior) */
    if (remaining >= 0xFC0U) {
        return 0xFFU;
    }

    /* Keep dividing by 4 until the value fits in 6 bits (<= 0x3F) */
    while (remaining > 0x3FU) {
        remaining = remaining / 4U;
        /* factor increments correspond to how many times we divided by 4 */
        if (factor < 0xFFU) { /* defensive guard, though factor will stay small */
            ++factor;
        }
    }

    /* Compose register value: low 6 bits = remaining, high 2 bits = factor */
    durval = (uint8_t)(remaining + ((uint32_t)factor << 6));

    return durval;
}


/*!
 * @brief This internal API is used to calculate the field data of sensor.
 *
 * Behavior preserved exactly:
 *  - Performs null-pointer check.
 *  - Reads BME680_FIELD_LENGTH bytes from BME680_FIELD0_ADDR into local buffer.
 *  - Decodes raw ADC values, computes compensated values via helper functions,
 *    and fills the user-provided bme680_field_data structure.
 *  - Polls up to 10 times (BME680_POLL_PERIOD_MS delay) waiting for NEW_DATA flag.
 *  - Returns BME680_W_NO_NEW_DATA if no new data after attempts.
 */
/* The function as requested */
static int8_t read_field_data(struct bme680_field_data *data, struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL;
    uint8_t buff[BME680_FIELD_LENGTH] = { 0 };
    uint8_t gas_range = 0;
    uint32_t adc_temp = 0;
    uint32_t adc_pres = 0;
    uint16_t adc_hum = 0;
    uint16_t adc_gas_res = 0;
    uint8_t tries = 10U;

    /*log_printf("[INFO] read_field_data: entry\r\n");*/

    /* Step 1 — Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] read_field_data: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* Loop until we get NEW_DATA or exhaust retries */
    do {
        /* Read the whole field (status + measurement bytes) */
        rslt = bme680_get_regs((uint8_t)BME680_FIELD0_ADDR, buff, (uint16_t)BME680_FIELD_LENGTH, dev);
        if (rslt != BME680_OK) {
        	BME680_LOG_PRINTF("[ERROR] read_field_data: bme680_get_regs failed (err=%d)\r\n", (int)rslt);
        	app_fault_delay_ms(20);
            /* If read failed, break out — preserve rslt for caller */
            break;
        }

        /* Status and indices */
        data->status = buff[0] & BME680_NEW_DATA_MSK;
        data->gas_index = buff[0] & BME680_GAS_INDEX_MSK;
        data->meas_index = buff[1];

        /* Decode raw ADC values (matching original layout) */
        adc_pres = (uint32_t)(((uint32_t)buff[2] * 4096U) |
                              ((uint32_t)buff[3] * 16U) |
                              ((uint32_t)buff[4] / 16U));

        adc_temp = (uint32_t)(((uint32_t)buff[5] * 4096U) |
                              ((uint32_t)buff[6] * 16U) |
                              ((uint32_t)buff[7] / 16U));

        adc_hum = (uint16_t)(((uint32_t)buff[8] * 256U) | (uint32_t)buff[9]);

        adc_gas_res = (uint16_t)(((uint32_t)buff[13] * 4U) | ((uint32_t)buff[14] / 64U));
        gas_range = buff[14] & 0x0F; /* BME680_GAS_RANGE_MSK assumed lower nibble */

        /* Merge additional status bits from the gas byte as original code does */
        data->status |= (buff[14] & BME680_GASM_VALID_MSK);
        data->status |= (buff[14] & BME680_HEAT_STAB_MSK);

        /*log_printf("[DEBUG] read_field_data: status=0x%02X gas_index=0x%02X meas_index=0x%02X\r\n",
                   (unsigned)buff[0], (unsigned)data->gas_index, (unsigned)data->meas_index);

        log_printf("[DEBUG] read_field_data: adc_pres=%lu adc_temp=%lu adc_hum=%u adc_gas_res=%u gas_range=%u\r\n",
                   (unsigned long)adc_pres, (unsigned long)adc_temp, (unsigned)adc_hum,
                   (unsigned)adc_gas_res, (unsigned)gas_range);*/

        /* If NEW_DATA bit set — compute compensated values and return success */
        if (data->status & BME680_NEW_DATA_MSK) {
            data->temperature = calc_temperature(adc_temp, dev);
            data->pressure = calc_pressure(adc_pres, dev);
            data->humidity = calc_humidity(adc_hum, dev);
            data->gas_resistance = calc_gas_resistance(adc_gas_res, gas_range, dev);

            /*log_printf("[INFO] read_field_data: new data acquired\r\n");*/
            rslt = BME680_OK;
            break;
        }

        /* Wait before next poll */
        dev->delay_ms(BME680_POLL_PERIOD_MS);
        tries--;
    } while (tries > 0U);

    /* If loop exhausted without seeing NEW_DATA and no other error, report no new data */
    if ((rslt != BME680_OK) && (tries == 0U) && ((data->status & BME680_NEW_DATA_MSK) == 0U)) {
        rslt = BME680_W_NO_NEW_DATA;
        /*log_printf("[WARN] read_field_data: no new data after retries\r\n");*/
    }

    /*log_printf("[INFO] read_field_data: exit (result=%d)\r\n", (int)rslt);*/
    return rslt;
}

/*!
 * @brief This internal API is used to set the memory page based on register address.
 *
 * Behavior preserved:
 *  - If register address > 0x7F select MEM_PAGE1, else MEM_PAGE0.
 *  - If the selected page differs from dev->mem_page, update the device register.
 *  - Uses dev->read/dev->write and maps their com_rslt to BME680_E_COM_FAIL on error.
 */
static int8_t set_mem_page(uint8_t reg_addr, struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL;
    uint8_t current_reg = 0;
    uint8_t mem_page = 0;

    BME680_LOG_PRINTF("[INFO] set_mem_page: entry (reg_addr=0x%02X)\r\n", reg_addr);
    app_fault_delay_ms(20);

    /* Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] set_mem_page: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }
    BME680_LOG_PRINTF("[DEBUG] set_mem_page: null_ptr_check passed\r\n");
    app_fault_delay_ms(20);

    /* Decide desired memory page from register address */
    if (reg_addr > 0x7FU) {
        mem_page = BME680_MEM_PAGE1;
    } else {
        mem_page = BME680_MEM_PAGE0;
    }
    BME680_LOG_PRINTF("[DEBUG] set_mem_page: desired mem_page=%u current mem_page=%u\r\n",
               (unsigned)mem_page, (unsigned)dev->mem_page);
    app_fault_delay_ms(20);

    /* If page already set, nothing to do */
    if (mem_page == dev->mem_page) {
    	BME680_LOG_PRINTF("[DEBUG] set_mem_page: mem_page already set, nothing to do\r\n");
    	app_fault_delay_ms(20);
        return BME680_OK;
    }

    /* Update dev->mem_page and write to the device register */
    dev->mem_page = mem_page;

    /* Read the current mem_page register (SPI read mask if required) */
    rslt = BME680_E_COM_FAIL;
    dev->com_rslt = dev->read(dev->handle, dev->dev_id, (uint8_t)(BME680_MEM_PAGE_ADDR | BME680_SPI_RD_MSK),
                              &current_reg, 1);
    if (dev->com_rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] set_mem_page: read of MEM_PAGE_ADDR failed (com_rslt=%d)\r\n", dev->com_rslt);
    	app_fault_delay_ms(20);
        rslt = BME680_E_COM_FAIL;
        return rslt;
    }
    rslt = BME680_OK;
    BME680_LOG_PRINTF("[DEBUG] set_mem_page: read mem page reg -> 0x%02X\r\n", current_reg);
    app_fault_delay_ms(20);

    /* Clear the mem_page bits and set new value (preserve other bits) */
    current_reg &= (uint8_t)(~BME680_MEM_PAGE_MSK);
    current_reg |= (uint8_t)(dev->mem_page & BME680_MEM_PAGE_MSK);

    /* Write updated page value (apply SPI write mask if required) */
    dev->com_rslt = dev->write(dev->handle, dev->dev_id,
                               (uint8_t)(BME680_MEM_PAGE_ADDR & BME680_SPI_WR_MSK),
                               &current_reg, 1);
    if (dev->com_rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] set_mem_page: write of MEM_PAGE_ADDR failed (com_rslt=%d)\r\n", dev->com_rslt);
    	app_fault_delay_ms(20);
        rslt = BME680_E_COM_FAIL;
    } else {
        rslt = BME680_OK;
        BME680_LOG_PRINTF("[INFO] set_mem_page: memory page set to %u successfully\r\n", (unsigned)dev->mem_page);
        app_fault_delay_ms(20);
    }

    BME680_LOG_PRINTF("[INFO] set_mem_page: exit (result=%d)\r\n", (int)rslt);
    app_fault_delay_ms(20);
    return rslt;
}


/*!
 * @brief This internal API is used to get the memory page from the device.
 *
 * Behavior preserved:
 *  - Performs a null-pointer check.
 *  - Reads the MEM_PAGE register (with SPI read mask if required).
 *  - Updates dev->mem_page with the masked value on success.
 */
static int8_t get_mem_page(struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_COM_FAIL;
    uint8_t reg = 0;

    BME680_LOG_PRINTF("[INFO] get_mem_page: entry\r\n");
    app_fault_delay_ms(20);

    /* Null pointer check (must remain) */
    rslt = null_ptr_check(dev);
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] get_mem_page: null_ptr_check failed (err=%d)\r\n", (int)rslt);
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }
    BME680_LOG_PRINTF("[DEBUG] get_mem_page: null_ptr_check passed\r\n");
    app_fault_delay_ms(20);

    /* Read mem-page register (apply SPI read mask if needed) */
    dev->com_rslt = dev->read(dev->handle, dev->dev_id, (uint8_t)(BME680_MEM_PAGE_ADDR | BME680_SPI_RD_MSK),
                              &reg, 1);
    if (dev->com_rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] get_mem_page: read failed (com_rslt=%d)\r\n", dev->com_rslt);
    	app_fault_delay_ms(20);
        rslt = BME680_E_COM_FAIL;
    } else {
        dev->mem_page = (uint8_t)(reg & BME680_MEM_PAGE_MSK);
        rslt = BME680_OK;
        BME680_LOG_PRINTF("[DEBUG] get_mem_page: read reg=0x%02X mem_page=%u\r\n", reg, (unsigned)dev->mem_page);
        app_fault_delay_ms(20);
    }

    BME680_LOG_PRINTF("[INFO] get_mem_page: exit (result=%d)\r\n", (int)rslt);
    app_fault_delay_ms(20);
    return rslt;
}


/*!
 * @brief This internal API is used to validate the boundary conditions.
 *
 * Behavior preserved:
 *  - If value == NULL returns BME680_E_NULL_PTR.
 *  - If *value < min, set *value = min and set info flag BME680_I_MIN_CORRECTION.
 *  - If *value > max, set *value = max and set info flag BME680_I_MAX_CORRECTION.
 */
static int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme680_dev *dev)
{
    int8_t rslt = BME680_OK;

    /*log_printf("[INFO] boundary_check: entry (min=%u max=%u)\r\n", (unsigned)min, (unsigned)max);*/

    if (value == NULL) {
    	BME680_LOG_PRINTF("[ERROR] boundary_check: NULL value pointer\r\n");
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    /* If below minimum — correct and flag */
    if (*value < min) {
        /*log_printf("[DEBUG] boundary_check: value %u < min %u, correcting to min\r\n",
                   (unsigned)*value, (unsigned)min);*/
        *value = min;
        dev->info_msg |= BME680_I_MIN_CORRECTION;
    }

    /* If above maximum — correct and flag */
    if (*value > max) {
        /*log_printf("[DEBUG] boundary_check: value %u > max %u, correcting to max\r\n",
                   (unsigned)*value, (unsigned)max);*/
        *value = max;
        dev->info_msg |= BME680_I_MAX_CORRECTION;
    }

    /*log_printf("[INFO] boundary_check: exit (result=%d adjusted_value=%u)\r\n",
               (int)rslt, (unsigned)*value);*/
    return rslt;
}


/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 *
 * Behavior preserved:
 *  - Returns BME680_E_NULL_PTR if dev, dev->read, dev->write or dev->delay_ms is NULL.
 *  - Otherwise returns BME680_OK.
 */
int8_t null_ptr_check(const struct bme680_dev *dev)
{
    int8_t rslt = BME680_E_NULL_PTR;

    /*log_printf("[INFO] null_ptr_check: entry\r\n");*/

    if (dev == NULL) {
    	BME680_LOG_PRINTF("[ERROR] null_ptr_check: dev == NULL\r\n");
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    if (dev->read == NULL) {
    	BME680_LOG_PRINTF("[ERROR] null_ptr_check: dev->read == NULL\r\n");
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    if (dev->write == NULL) {
    	BME680_LOG_PRINTF("[ERROR] null_ptr_check: dev->write == NULL\r\n");
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    if (dev->delay_ms == NULL) {
    	BME680_LOG_PRINTF("[ERROR] null_ptr_check: dev->delay_ms == NULL\r\n");
    	app_fault_delay_ms(20);
        return BME680_E_NULL_PTR;
    }

    rslt = BME680_OK;
    /*log_printf("[DEBUG] null_ptr_check: all required pointers are valid\r\n");*/
    /*log_printf("[INFO] null_ptr_check: exit (result=%d)\r\n", (int)rslt);*/

    return rslt;
}


/*
 * Assumptions  on the project:
 *  - global BME context exists as: bme
 *  - global/helper APIs exist:
 *      log_printf(...)
 *      tim2_delay_ms(...)
 *      user_delay_ms(...)
 *      init_bme680(void)
 *      i2c_dma_bme680_peripherals_init(void)
 *      i2c_recover(...)
 *      bme_GetGasReference(void)
 *      bme_GetHumidityScore(void)
 *      bme_GetGasScore(void)
 *      bme_calculateIAQ(int16_t score)
 *  - Bosch sample type is struct bme680_field_data
 *  - app_status_t / APP_STATUS_OK / APP_STATUS_INIT_FAIL already exist
 *  - BME path is on I2C1
 */

static void app_bme680_log_runtime_error(const char *tag,
                                          int32_t rc,
                                          int attempt,
                                          int max_attempts)
{
    system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_BME680);

    BME680_LOG_PRINTF("[ERROR] %s failed with code %ld (attempt %d/%d)\r\n",
                      tag ? tag : "BME680 runtime",
                      (long)rc,
                      attempt,
                      max_attempts);
    app_fault_delay_ms(20U);
}


static app_bme680_runtime_status_t app_bme680_recover_and_reinit(int attempt,
                                                                 int max_attempts)
{
    i2c_dma_init_status_t init_status;

    /*
     * Runtime recovery must be bounded.
     *
     * Do NOT call init_bme680() from here during fault-injection/runtime retry.
     * If the BME680 is physically disconnected or being fault-injected,
     * init_bme680() enters soft-reset / chip-id / calibration reads and can
     * consume the retry window before the outer loop reaches attempt 2/3 and 3/3.
     *
     * The outer app_bme680_sensor_runtime_process() loop owns the 3-attempt budget.
     * This function only restores I2C/DMA state between attempts.
     */
    BME680_LOG_PRINTF("ACT: BME680 recovery after attempt %d/%d -> I2C/DMA restore only\r\n",
                      attempt,
                      max_attempts);
    app_fault_delay_ms(20U);

    if (i2c_recover(I2C1, &I2C1->SR2, I2C_SR2_BUSY, 0U, 0U) != I2C_RECOVER_OK) {
        return APP_BME680_RUNTIME_I2C_RECOVER_FAIL;
    }

    init_status = i2c_dma_bme680_peripherals_init();
    if (init_status != I2C_DMA_INIT_OK) {
        return APP_BME680_RUNTIME_PERIPH_REINIT_FAIL;
    }

    user_delay_ms(10U);
    return APP_BME680_RUNTIME_OK;
}


sensor_stack_init_status_t app_bme680_sensor_stack_init(void)
{
    i2c_dma_init_status_t init_status;
    int8_t rslt;

    init_status = i2c_dma_bme680_peripherals_init();
    if (init_status != I2C_DMA_INIT_OK) {
    	BME680_LOG_PRINTF("[ERROR] app_bme680_sensor_stack_init: i2c_dma_bme680_peripherals_init failed rc=%d\r\n",
                   (int)init_status);
    	app_fault_delay_ms(20);
        return SENSOR_STACK_INIT_FAIL;
    }

    rslt = init_bme680();
    if (rslt != BME680_OK) {
    	BME680_LOG_PRINTF("[ERROR] app_bme680_sensor_stack_init: init_bme680 failed rc=%d\r\n",
                   (int)rslt);
    	app_fault_delay_ms(20);
        return SENSOR_STACK_INIT_FAIL;
    }

    return SENSOR_STACK_OK;
}



bool telemetry_publish_bme680(
    float temperature_c,
    float pressure_hpa,
    float humidity_pct,
    int16_t iaq_score,
    const char *iaq_label)
{
    telemetry_bme680_t sample;

    if (q_bme680 == NULL)
    {
        return false;
    }

    memset(&sample, 0, sizeof(sample));

    sample.sample_id =
        telemetry_next_generation(&bme680_generation);

    sample.temperature_c = temperature_c;
    sample.pressure_hpa  = pressure_hpa;
    sample.humidity_pct  = humidity_pct;
    sample.iaq_score     = iaq_score;

    if (iaq_label != NULL)
    {
        strncpy(
            sample.iaq_label,
            iaq_label,
            TELEMETRY_IAQ_LABEL_SIZE - 1U);

        sample.iaq_label[
            TELEMETRY_IAQ_LABEL_SIZE - 1U] = '\0';
    }

    return
        (xQueueOverwrite(q_bme680, &sample) == pdPASS);
}



app_bme680_runtime_status_t app_bme680_sensor_runtime_process(void)
{
    enum { MAX_ATTEMPTS = 3 };

    int16_t iaq_score = 0;
    const char *iaq_label = "Unknown";

    heartbeat_signal_bme680main();

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
        struct bme680_field_data data;
        float temp;
        float pres;
        float hum;
        int8_t rslt;

        memset(&data, 0, sizeof(data));

        rslt = bme680_get_sensor_data(&data, &bme.gas_sensor);
        if (rslt != BME680_OK) {
            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_BME680);

            app_bme680_log_runtime_error("bme680_get_sensor_data",
                                         rslt,
                                         attempt,
                                         MAX_ATTEMPTS);

            if (attempt == MAX_ATTEMPTS) {
                BME680_LOG_PRINTF("[FATAL] BME680 failed after retry %d/%d -> reporting fatal to task\r\n",
                                  attempt,
                                  MAX_ATTEMPTS);
                app_fault_delay_ms(20U);
                return APP_BME680_RUNTIME_SENSOR_DATA_FAIL;
            }

            {
                app_bme680_runtime_status_t recovery_status;

                recovery_status = app_bme680_recover_and_reinit(attempt, MAX_ATTEMPTS);
                if (recovery_status != APP_BME680_RUNTIME_OK) {
                    BME680_LOG_PRINTF("[WARN] BME680 recovery/reinit failed after attempt %d/%d, continuing retry budget rc=%d\r\n",
                                      attempt,
                                      MAX_ATTEMPTS,
                                      (int)recovery_status);
                    app_fault_delay_ms(20U);
                }
            }

            BME680_LOG_PRINTF("ACT: BME680 short backoff before retry\r\n");
            app_fault_delay_ms(20U);
            continue;
        }

        /* Skip startup / invalid samples so 0.00 does not get published. */
        if ((data.temperature == 0) || (data.pressure == 0U) || (data.humidity == 0U)) {

            if (bme.gas_sensor.power_mode == BME680_FORCED_MODE) {
                rslt = bme680_set_sensor_mode(&bme.gas_sensor);
                if (rslt != BME680_OK) {
                    system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_BME680);

                    app_bme680_log_runtime_error("bme680_set_sensor_mode",
                                                 rslt,
                                                 attempt,
                                                 MAX_ATTEMPTS);

                    if (attempt == MAX_ATTEMPTS) {
                        BME680_LOG_PRINTF("[FATAL] BME680 failed after retry %d/%d -> reporting fatal to task\r\n",
                                          attempt,
                                          MAX_ATTEMPTS);
                        app_fault_delay_ms(20U);
                        return APP_BME680_RUNTIME_SET_MODE_FAIL;
                    }

                    {
                        app_bme680_runtime_status_t recovery_status;

                        recovery_status = app_bme680_recover_and_reinit(attempt, MAX_ATTEMPTS);
                        if (recovery_status != APP_BME680_RUNTIME_OK) {
                            BME680_LOG_PRINTF("[WARN] BME680 recovery/reinit failed after attempt %d/%d, continuing retry budget rc=%d\r\n",
                                              attempt,
                                              MAX_ATTEMPTS,
                                              (int)recovery_status);
                            app_fault_delay_ms(20U);
                        }
                    }

                    BME680_LOG_PRINTF("ACT: BME680 short backoff before retry\r\n");
                    app_fault_delay_ms(20U);
                    continue;
                }

                {
                    uint32_t wait_ms = (bme.meas_period > 0U) ? (uint32_t)bme.meas_period : 2000U;
                    user_delay_ms(wait_ms);
                }
            }

            heartbeat_signal_bme680main();

            system_fault_clear_active(SYSTEM_FAULT_OWNER_BME680);

            return APP_BME680_RUNTIME_OK;
        }

        temp = (float)data.temperature / 100.0f;
        pres = (float)data.pressure    / 100.0f;
        hum  = (float)data.humidity    / 1000.0f;

        bme.temperature = temp;
        bme.pressure    = pres;
        bme.humidity    = hum;

        if ((data.status & BME680_GASM_VALID_MSK) != 0U) {
            bme.gas_resistance = data.gas_resistance;
        }

        if (system_fault_logging_active == 0U) {
            BME680_LOG_PRINTF("[BME680] t:%.2f; p:%.0f; h:%.0f, ",
                              bme.temperature,
                              bme.pressure,
                              bme.humidity);
        }

        {
            int16_t hum_score;
            int16_t gas_score;
            int16_t raw_score;

            hum_score = bme_GetHumidityScore();
            gas_score = bme_GetGasScore();
            raw_score = (int16_t)(hum_score + gas_score);

            iaq_score =
                bme_calculateIAQ(
                    raw_score,
                    &iaq_label);
        }

        if (bme.gas_sensor.power_mode == BME680_FORCED_MODE) {
            rslt = bme680_set_sensor_mode(&bme.gas_sensor);
            if (rslt != BME680_OK) {
                system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_BME680);

                app_bme680_log_runtime_error("bme680_set_sensor_mode",
                                             rslt,
                                             attempt,
                                             MAX_ATTEMPTS);

                if (attempt == MAX_ATTEMPTS) {
                    BME680_LOG_PRINTF("[FATAL] BME680 failed after retry %d/%d -> reporting fatal to task\r\n",
                                      attempt,
                                      MAX_ATTEMPTS);
                    app_fault_delay_ms(20U);
                    return APP_BME680_RUNTIME_SET_MODE_FAIL;
                }

                {
                    app_bme680_runtime_status_t recovery_status;

                    recovery_status = app_bme680_recover_and_reinit(attempt, MAX_ATTEMPTS);
                    if (recovery_status != APP_BME680_RUNTIME_OK) {
                        BME680_LOG_PRINTF("[WARN] BME680 recovery/reinit failed after attempt %d/%d, continuing retry budget rc=%d\r\n",
                                          attempt,
                                          MAX_ATTEMPTS,
                                          (int)recovery_status);
                        app_fault_delay_ms(20U);
                    }
                }

                BME680_LOG_PRINTF("ACT: BME680 short backoff before retry\r\n");
                app_fault_delay_ms(20U);
                continue;
            }
        }


        (void)telemetry_publish_bme680(
            bme.temperature,
            bme.pressure,
            bme.humidity,
            iaq_score,
            iaq_label);


        heartbeat_signal_bme680main();

        system_fault_clear_active(SYSTEM_FAULT_OWNER_BME680);

        return APP_BME680_RUNTIME_OK;
    }

    system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_BME680);
    return APP_BME680_RUNTIME_SENSOR_DATA_FAIL;
}




