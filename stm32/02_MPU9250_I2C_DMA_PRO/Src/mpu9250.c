/* mpu9250.c | Author: i fiksuari pas prog | Date: 25 January 2026 */
#include "mpu9250.h"
#include "tim.h"
#include "i2c_dma.h"
#include "uart.h"
#include <math.h>
#include "pwr.h"
#include "iwdg.h"

/** @PINOUT
 *
MPU9250 VCC    -> 3.3V
MPU9250 GND    -> GND
MPU9250 SCL    -> PA8
MPU9250 SDA    -> PB4
MPU9250 EDA    -> NC
MPU9250 ECL    -> NC
MPU9250 AD0    -> GND
MPU9250 INT    -> NC
MPU9250 NCS    -> 3.3V
MPU9250 FSYNC  -> GND
 */

volatile uint8_t dummy_buff[MAX_TRANSFER_LEN + 1U];
volatile uint8_t accel_buff[MAX_TRANSFER_LEN + 1U];
volatile uint8_t gyro_buff[MAX_TRANSFER_LEN + 1U];
volatile uint8_t i2c_data_buff[I2C_DATA_BUFF_LEN];

double g_accel_range, gyroRange;
float gyro_x_offset, gyro_y_offset, gyro_z_offset;
float gyro_gx, gyro_gy, gyro_gz;
float tau = 0.98f, dt = 0.004f;
float r = 0.0f, p = 0.0f, y = 0.0f;
uint8_t firstRun = 1U;

MPU9250_t MPU9250;

static app_status_t app_mpu9250_runtime_restore_config_only(void);

static void app_mpu9250_log_runtime_error(const char *tag,
                                          int32_t rc,
                                          int attempt,
                                          int max_attempts);

static app_mpu9250_runtime_status_t app_mpu9250_recover_and_reinit(int attempt,
                                                                   int max_attempts);


 

/* Requests a STOP condition on the MPU9250 I2C bus. */
static void mpu9250_request_stop(void)
{
    /* Set the STOP bit to end the transfer without another read. */
    MPU9250_I2C_INSTANCE->CR1 |= I2C_CR1_STOP;
    __asm__ volatile ("nop");
}

/* Waits for the MPU9250 DMA completion flag with a bounded timeout. */
static mpu9250_status_t mpu9250_wait_for_dma_completion(volatile uint8_t *p_flag,
                                                        uint32_t timeout_iter,
                                                        const char *context,
                                                        uint8_t is_tx)
{
    /* Check the completion state and return an error if the transfer stalls. */
    uint32_t t = 0U;

    if (p_flag == NULL) {
        log_printf("[ERROR] %s: NULL DMA completion flag pointer\r\n", context);
        tim2_delay_ms(2);
        return MPU9250_STATUS_NULL_PTR;
    }

    /* Low-power wait: Sleep until an interrupt occurs, then re-check the DMA completion flag. */
    while (((*p_flag) == I2C_DMA_TRANSFER_IDLE) && (t++ < timeout_iter)) {
        cpu_sleep_until_irq();
    }

    if ((*p_flag) == I2C_DMA_TRANSFER_COMPLETE) {
        *p_flag = I2C_DMA_TRANSFER_IDLE;
        return MPU9250_STATUS_OK;
    }

    if ((*p_flag) == I2C_DMA_TRANSFER_ERROR) {
        *p_flag = I2C_DMA_TRANSFER_IDLE;
        log_printf("[ERROR] %s: DMA transfer error\r\n", context);
        tim2_delay_ms(2);
        return (is_tx != 0U) ? MPU9250_STATUS_TX_TRANSFER_ERROR
                             : MPU9250_STATUS_RX_TRANSFER_ERROR;
    }

    *p_flag = I2C_DMA_TRANSFER_IDLE;
    log_printf("[ERROR] %s: DMA completion timeout\r\n", context);
    tim2_delay_ms(2);
    return (is_tx != 0U) ? MPU9250_STATUS_TX_TIMEOUT
                         : MPU9250_STATUS_RX_TIMEOUT;
}

/* Writes mpu9250 write single reg blocking to the selected interface. */
static mpu9250_status_t mpu9250_write_single_reg_blocking(uint8_t reg_addr,
                                                          uint8_t reg_val,
                                                          const char *context)
{
    /* Check status before reporting a completed transfer. */
    i2c_dma_status_t dma_status;

    i2c_data_buff[0] = reg_val;
    mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;

    dma_status = i2c_dma_write_reg(MPU9250_I2C_INSTANCE,
                                   mpu9250_addr,
                                   reg_addr,
                                   (const uint8_t *)i2c_data_buff,
                                   I2C_DATA_BUFF_LEN);

    if (dma_status != I2C_DMA_OK) {
        log_printf("[ERROR] %s: i2c_dma_write_reg failed, status=%d\r\n",
                   context, (int)dma_status);
        tim2_delay_ms(2);
        return MPU9250_STATUS_I2C_WRITE_START_FAILED;
    }

    return mpu9250_wait_for_dma_completion(&mpu9250g_tx_cmplt,
                                           MPU9250_TX_TIMEOUT_ITER,
                                           context,
                                           1U);
}

/* Reads mpu9250 read blocking from the device or configured buffer. */
static mpu9250_status_t mpu9250_read_blocking(uint8_t start_reg,
                                              uint8_t *p_dst,
                                              uint16_t len,
                                              const char *context)
{
    /* Check the transfer result before returning the data. */
    i2c_dma_status_t dma_status;

    if ((p_dst == NULL) || (len == 0U)) {
        log_printf("[ERROR] %s: invalid buffer/length\r\n", context);
        tim2_delay_ms(2);
        return MPU9250_STATUS_INVALID_ARG;
    }

    mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;

    dma_status = i2c_dma_read(MPU9250_I2C_INSTANCE,
                              mpu9250_addr,
                              start_reg,
                              p_dst,
                              len);

    if (dma_status != I2C_DMA_OK) {
        log_printf("[ERROR] %s: i2c_dma_read failed, status=%d\r\n",
                   context, (int)dma_status);
        tim2_delay_ms(2);
        return MPU9250_STATUS_I2C_READ_START_FAILED;
    }

    return mpu9250_wait_for_dma_completion(&mpu9250g_rx_cmplt,
                                           MPU9250_READ_TIMEOUT_ITER,
                                           context,
                                           0U);
}

 

/* Configures the accelerometer measurement range. */
mpu9250_status_t mpu9250_accel_config(uint8_t mode, MPU9250_t *pMPU9250)
{
    /* Select the range and update its conversion scale. */
    uint8_t reg_val = 0x00U;
    float accel_range = 2.0f;
    float ascale = 16384.0f;
    mpu9250_status_t status;

    if (pMPU9250 == NULL) {
        log_printf("[ERROR] mpu9250_accel_config: NULL pointer\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_NULL_PTR;
    }

    switch (mode) {
        case ACC_FULL_SCALE_2_G:
            accel_range = 2.0f;
            ascale = 16384.0f;
            reg_val = 0x00U;
            break;

        case ACC_FULL_SCALE_4_G:
            accel_range = 4.0f;
            ascale = 8192.0f;
            reg_val = 0x08U;
            break;

        case ACC_FULL_SCALE_8_G:
            accel_range = 8.0f;
            ascale = 4096.0f;
            reg_val = 0x10U;
            break;

        case ACC_FULL_SCALE_16_G:
            accel_range = 16.0f;
            ascale = 2048.0f;
            reg_val = 0x18U;
            break;

        default:
            log_printf("[WARN] mpu9250_accel_config: unknown mode 0x%02X, falling back to +/-2g\r\n",
                       (unsigned)mode);
            tim2_delay_ms(2);
            accel_range = 2.0f;
            ascale = 16384.0f;
            reg_val = 0x00U;
            break;
    }

    pMPU9250->range.g_accel_range = accel_range;
    pMPU9250->sensorData.aScaleFactor = ascale;
    pMPU9250->settings.aFullScaleRange = reg_val;

    status = mpu9250_write_single_reg_blocking(MPU9250_ADDR_ACCELCONFIG,
                                               reg_val,
                                               "mpu9250_accel_config");
    return status;
}

 

/* Configures the gyroscope measurement range. */
mpu9250_status_t MPU9250_beginGyro(uint8_t mode, MPU9250_t *pMPU9250)
{
    /* Program the sensor range and retain its scale factor. */
    uint8_t reg_val = 0x08U;
    float gyro_range = 500.0f;
    float gscale = 65.5f;
    mpu9250_status_t status;

    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU9250_beginGyro: NULL pointer\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_NULL_PTR;
    }

    switch (mode) {
        case GYRO_FULL_SCALE_250_DPS:
            gyro_range = 250.0f;
            gscale = 131.0f;
            reg_val = 0x00U;
            break;

        case GYRO_FULL_SCALE_500_DPS:
            gyro_range = 500.0f;
            gscale = 65.5f;
            reg_val = 0x08U;
            break;

        case GYRO_FULL_SCALE_1000_DPS:
            gyro_range = 1000.0f;
            gscale = 32.8f;
            reg_val = 0x10U;
            break;

        case GYRO_FULL_SCALE_2000_DPS:
            gyro_range = 2000.0f;
            gscale = 16.4f;
            reg_val = 0x18U;
            break;

        default:
            log_printf("[WARN] MPU9250_beginGyro: unknown mode 0x%02X, falling back to 500 DPS\r\n",
                       (unsigned)mode);
            tim2_delay_ms(2);
            gyro_range = 500.0f;
            gscale = 65.5f;
            reg_val = 0x08U;
            break;
    }

    pMPU9250->range.gyroRange = gyro_range;
    pMPU9250->sensorData.gScaleFactor = gscale;
    pMPU9250->settings.gFullScaleRange = reg_val;

    status = mpu9250_write_single_reg_blocking(MPU9250_ADDR_GYROCONFIG,
                                               reg_val,
                                               "MPU9250_beginGyro");
    return status;
}

 

/* Reads the accelerometer registers through the blocking transfer path. */
mpu9250_status_t mpu9250_accel_update_blocking(void)
{
    /* Check the transfer before converting the received bytes. */
    mpu9250_status_t status;

    status = mpu9250_read_blocking(MPU9250_ACCEL_XOUT_H,
                                   (uint8_t *)accel_buff,
                                   (uint16_t)MAX_TRANSFER_LEN,
                                   "mpu9250_accel_update_blocking");

    if (status != MPU9250_STATUS_OK) {
        log_printf("[WARN] mpu9250_accel_update_blocking: read failed, status=%d\r\n",
                   (int)status);
        tim2_delay_ms(2);

        mpu9250_request_stop();
        return status;
    }

    return MPU9250_STATUS_OK;
}


 

/* Reads the gyroscope registers through the blocking transfer path. */
mpu9250_status_t mpu9250_gyro_update_blocking(void)
{
    /* Check the transfer before converting the received bytes. */
    mpu9250_status_t status;

    status = mpu9250_read_blocking(MPU9250_GYRO_XOUT_H,
                                   (uint8_t *)gyro_buff,
                                   (uint16_t)MAX_TRANSFER_LEN,
                                   "mpu9250_gyro_update_blocking");

    if (status != MPU9250_STATUS_OK) {
        log_printf("[WARN] mpu9250_gyro_update_blocking: read failed, status=%d\r\n",
                   (int)status);
        tim2_delay_ms(2);

        mpu9250_request_stop();
        return status;
    }

    return MPU9250_STATUS_OK;
}

 

/* Returns the accelerometer values in scaled units. */
mpu9250_status_t MPU9250_accelGet(MPU9250_t *pMPU9250)
{
    /* Use the configured sensitivity when converting the raw reading. */
    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU9250_accelGet: NULL pointer passed\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_NULL_PTR;
    }

    {
        mpu9250_status_t status = mpu9250_accel_update_blocking();
        if (status != MPU9250_STATUS_OK) {
            log_printf("[ERROR] MPU9250_accelGet: accel DMA/read failed, status=%d\r\n",
                       (int)status);
            tim2_delay_ms(2);
            return status;
        }
    }

    pMPU9250->rawData.ax = (int16_t)(((uint16_t)accel_buff[0] << 8) | (uint16_t)accel_buff[1]);
    pMPU9250->rawData.ay = (int16_t)(((uint16_t)accel_buff[2] << 8) | (uint16_t)accel_buff[3]);
    pMPU9250->rawData.az = (int16_t)(((uint16_t)accel_buff[4] << 8) | (uint16_t)accel_buff[5]);

    return MPU9250_STATUS_OK;
}

 

/* Returns the gyroscope values in scaled units. */
mpu9250_status_t MPU9250_gyroGet(MPU9250_t *pMPU9250)
{
    /* Apply the calibration offset to the gyro reading. */
    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU9250_gyroGet: NULL pointer passed\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_NULL_PTR;
    }

    {
        mpu9250_status_t status = mpu9250_gyro_update_blocking();
        if (status != MPU9250_STATUS_OK) {
            log_printf("[ERROR] MPU9250_gyroGet: gyro DMA/read failed, status=%d\r\n",
                       (int)status);
            tim2_delay_ms(2);
            return status;
        }
    }

    pMPU9250->rawData.gx = (int16_t)(((uint16_t)gyro_buff[0] << 8) | (uint16_t)gyro_buff[1]);
    pMPU9250->rawData.gy = (int16_t)(((uint16_t)gyro_buff[2] << 8) | (uint16_t)gyro_buff[3]);
    pMPU9250->rawData.gz = (int16_t)(((uint16_t)gyro_buff[4] << 8) | (uint16_t)gyro_buff[5]);

    return MPU9250_STATUS_OK;
}


/* Initializes app mpu9250 sensor stack init for the requested operation. */
 app_status_t app_mpu9250_sensor_stack_init(void)
{
    /* Validate required parameters before applying the setup. */
    enum { MAX_ATTEMPTS = 3 };
    mpu9250_status_t mpu_rc = MPU9250_STATUS_OK;

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {

        /* Preflight recovery at startup: important after watchdog resets, because the sensor/bus may not be in a */
        if (i2c_recover(I2C3, &I2C3->SR2, I2C_SR2_BUSY, 0U, 0U) != I2C_RECOVER_OK) {
            log_printf("[WARN] app_mpu9250_sensor_stack_init: i2c_recover failed (attempt %d/%d)\r\n",
                       attempt, MAX_ATTEMPTS);
            tim2_delay_ms(2);
        }


        mpu_rc = mpu9250_accel_config(ACC_FULL_SCALE_2_G, &MPU9250);
        if (mpu_rc != MPU9250_STATUS_OK) {
            log_printf("[ERROR] app_mpu9250_sensor_stack_init: mpu9250_accel_config failed rc=%d (attempt %d/%d)\r\n",
                       (int)mpu_rc, attempt, MAX_ATTEMPTS);
            tim2_delay_ms(2);

            if (attempt < MAX_ATTEMPTS) {
                continue;
            }

            return APP_STATUS_INIT_FAIL;
        }

        mpu_rc = MPU9250_beginGyro(GYRO_FULL_SCALE_250_DPS, &MPU9250);
        if (mpu_rc != MPU9250_STATUS_OK) {
            log_printf("[ERROR] app_mpu9250_sensor_stack_init: MPU9250_beginGyro failed rc=%d (attempt %d/%d)\r\n",
                       (int)mpu_rc, attempt, MAX_ATTEMPTS);
            tim2_delay_ms(2);

            if (attempt < MAX_ATTEMPTS) {
                continue;
            }

            return APP_STATUS_INIT_FAIL;
        }

        mpu_rc = mpu9250_calibrateGyro(&MPU9250, MPU_GYRO_CAL_SAMPLES);
        if (mpu_rc != MPU9250_STATUS_OK) {
            log_printf("[ERROR] app_mpu9250_sensor_stack_init: mpu9250_calibrateGyro failed rc=%d (attempt %d/%d)\r\n",
                       (int)mpu_rc, attempt, MAX_ATTEMPTS);
            tim2_delay_ms(2);

            if (attempt < MAX_ATTEMPTS) {
                continue;
            }

            return APP_STATUS_INIT_FAIL;
        }

        return APP_STATUS_OK;
    }

    log_printf("[ERROR] app_mpu9250_sensor_stack_init: unexpected fallthrough\r\n");
    tim2_delay_ms(2);
    return APP_STATUS_INIT_FAIL;
}


 

/* Calculates gyroscope offsets from a set of reference samples. */
mpu9250_status_t mpu9250_calibrateGyro(MPU9250_t *pMPU9250, uint16_t numCalPoints)
{
    /* Accumulate samples only while the device is reporting valid data. */
    float sx = 0.0f;
    float sy = 0.0f;
    float sz = 0.0f;

    if (pMPU9250 == NULL) {
        log_printf("[ERROR] mpu9250_calibrateGyro: NULL pointer\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_NULL_PTR;
    }

    if (numCalPoints == 0U) {
        numCalPoints = 1000U;
    }

    log_printf("Gyro calibration: %u samples. Keep device perfectly still...\r\n",
               (unsigned)numCalPoints);

    for (uint16_t i = 0U; i < numCalPoints; i++) {
        mpu9250_status_t status = MPU9250_gyroGet(pMPU9250);
        if (status != MPU9250_STATUS_OK) {
            log_printf("[ERROR] mpu9250_calibrateGyro: gyro read failed at sample %u, status=%d\r\n",
                       (unsigned)i, (int)status);
            tim2_delay_ms(2);
            return status;
        }

        sx += (float)pMPU9250->rawData.gx;
        sy += (float)pMPU9250->rawData.gy;
        sz += (float)pMPU9250->rawData.gz;

         
    }

    pMPU9250->gyroCal.x = sx / (float)numCalPoints;
    pMPU9250->gyroCal.y = sy / (float)numCalPoints;
    pMPU9250->gyroCal.z = sz / (float)numCalPoints;

    log_printf("Calibration done. Offsets (raw): X=%.3f Y=%.3f Z=%.3f\r\n",
               pMPU9250->gyroCal.x,
               pMPU9250->gyroCal.y,
               pMPU9250->gyroCal.z);

    return MPU9250_STATUS_OK;
}

/* Read and scale sensor data */

/* Reads accelerometer and gyroscope data and applies the configured processing. */
mpu9250_status_t MPU_readProcessedData(MPU9250_t *pMPU9250)
{
    /* Return a failure when the sensor transfer or data conversion fails. */
    mpu9250_status_t status;

    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU_readProcessedData: NULL pointer\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_NULL_PTR;
    }

    status = MPU9250_accelGet(pMPU9250);
    if (status != MPU9250_STATUS_OK) {
        log_printf("[ERROR] MPU_readProcessedData: accel read failed, status=%d\r\n",
                   (int)status);
        tim2_delay_ms(2);
        return status;
    }

    if (pMPU9250->sensorData.aScaleFactor == 0.0f) {
        log_printf("[ERROR] MPU_readProcessedData: accelerometer scale factor is zero\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_SCALE_FACTOR_ZERO;
    }

    pMPU9250->sensorData.ax = pMPU9250->rawData.ax / pMPU9250->sensorData.aScaleFactor;
    pMPU9250->sensorData.ay = pMPU9250->rawData.ay / pMPU9250->sensorData.aScaleFactor;
    pMPU9250->sensorData.az = pMPU9250->rawData.az / pMPU9250->sensorData.aScaleFactor;

    log_printf("ACCEL: X=%.3f g , Y=%.3f g , Z=%.3f g\r\n",
               pMPU9250->sensorData.ax,
               pMPU9250->sensorData.ay,
               pMPU9250->sensorData.az);

    status = MPU9250_gyroGet(pMPU9250);
    if (status != MPU9250_STATUS_OK) {
        log_printf("[ERROR] MPU_readProcessedData: gyro read failed, status=%d\r\n",
                   (int)status);
        tim2_delay_ms(2);
        return status;
    }

    if (pMPU9250->sensorData.gScaleFactor == 0.0f) {
        log_printf("[ERROR] MPU_readProcessedData: gyroscope scale factor is zero\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_SCALE_FACTOR_ZERO;
    }

    pMPU9250->sensorData.gx = pMPU9250->rawData.gx - pMPU9250->gyroCal.x;
    pMPU9250->sensorData.gy = pMPU9250->rawData.gy - pMPU9250->gyroCal.y;
    pMPU9250->sensorData.gz = pMPU9250->rawData.gz - pMPU9250->gyroCal.z;

    pMPU9250->sensorData.gx /= pMPU9250->sensorData.gScaleFactor;
    pMPU9250->sensorData.gy /= pMPU9250->sensorData.gScaleFactor;
    pMPU9250->sensorData.gz /= pMPU9250->sensorData.gScaleFactor;

    log_printf("GYRO:  X=%.3f dps , Y=%.3f dps , Z=%.3f dps\r\n",
               pMPU9250->sensorData.gx,
               pMPU9250->sensorData.gy,
               pMPU9250->sensorData.gz);

    return MPU9250_STATUS_OK;
}


/* Logs the MPU9250 failure reason and current retry attempt. */
static void app_mpu9250_log_runtime_error(const char *tag,
                                          int32_t rc,
                                          int attempt,
                                          int max_attempts)
{
    /* Include the error status to identify the failing stage. */
    if (rc > 0) {
        log_printf("[WARN] %s returned warning code %ld (attempt %d/%d)\r\n",
                   tag, (long)rc, attempt, max_attempts);
        tim2_delay_ms(2);
    } else {
        log_printf("[ERROR] %s failed with code %ld (attempt %d/%d)\r\n",
                   tag, (long)rc, attempt, max_attempts);
        tim2_delay_ms(2);
    }
}


/* Initializes the peripheral resources used by MPU9250. */
app_status_t app_mpu9250_peripherals_init(void)
{
    /* Return failure if the I2C or DMA setup cannot finish. */
    i2c_select_status_t rc;
    dma_status_t st;

    /* 1) Basic MPU9250 runtime parameters */
    MPU9250.attitude.tau = 0.98f;
    MPU9250.attitude.dt  = 0.004f;

     
    rc = i2c_init_select(3U, 8U, 4U);
    if (rc != I2C_SELECT_OK) {
        log_printf("[ERROR] app_mpu9250_peripherals_init: i2c_init_select(I2C3, PA8/PB4) failed rc=%d\r\n",
                   (int)rc);
        tim2_delay_ms(2);
        return APP_STATUS_INIT_FAIL;
    }

    /* 3) Enable FPU (CP10/CP11 full access) */
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    /* 4) DMA streams for I2C3 MPU9250 (RX=Stream1, TX=Stream4) */
    st = dma_stream_init(DMA1_Stream1,
                         1U,
                         DMA_DIR_P2M,
                         &DMA1->LIFCR,
                         DMA1_STREAM1_TCIF,
                         DMA1_Stream1_IRQn);
    if (st != DMA_OK) {
        log_printf("[ERROR] app_mpu9250_peripherals_init: DMA1_Stream1 init failed rc=%d\r\n",
                   (int)st);
        tim2_delay_ms(2);
        return APP_STATUS_INIT_FAIL;
    }

    st = dma_stream_init(DMA1_Stream4,
                         3U,
                         DMA_DIR_M2P,
                         &DMA1->HIFCR,
                         DMA1_STREAM4_TCIF,
                         DMA1_Stream4_IRQn);
    if (st != DMA_OK) {
        log_printf("[ERROR] app_mpu9250_peripherals_init: DMA1_Stream4 init failed rc=%d\r\n",
                   (int)st);
        tim2_delay_ms(2);
        return APP_STATUS_INIT_FAIL;
    }

    return APP_STATUS_OK;
}



/* Recovers the MPU9250 I2C interface and restores the driver state. */
static app_mpu9250_runtime_status_t app_mpu9250_recover_and_reinit(int attempt,
                                                                   int max_attempts)
{
    /* Restore peripheral configuration before repeating the sensor read. */
    i2c_recover_status_t recover_status;


    recover_status = i2c_recover(I2C3, &I2C3->SR2, I2C_SR2_BUSY, 0U, 0U);
    if (recover_status != I2C_RECOVER_OK) {
        log_printf("[WARN] app_mpu9250: i2c_recover() failed (attempt %d/%d)\r\n",
                   attempt, max_attempts);
        tim2_delay_ms(2);
        return APP_MPU9250_RUNTIME_I2C_RECOVER_FAIL;
    }

    if (app_mpu9250_peripherals_init() != APP_STATUS_OK) {
        log_printf("[WARN] app_mpu9250: app_mpu9250_peripherals_init() failed (attempt %d/%d)\r\n",
                   attempt, max_attempts);
        tim2_delay_ms(2);
        return APP_MPU9250_RUNTIME_PERIPH_REINIT_FAIL;
    }

    return APP_MPU9250_RUNTIME_OK;
}


/* Restores sensor configuration after a transport recovery. */
static app_status_t app_mpu9250_runtime_restore_config_only(void)
{
    /* Reuse the existing device state while restoring configuration. */
    mpu9250_status_t mpu_rc;


    mpu_rc = mpu9250_accel_config(ACC_FULL_SCALE_2_G, &MPU9250);
    if (mpu_rc != MPU9250_STATUS_OK) {
        log_printf("[ERROR] app_mpu9250_runtime_restore_config_only: mpu9250_accel_config failed rc=%d\r\n",
                   (int)mpu_rc);
        tim2_delay_ms(2);
        return APP_STATUS_INIT_FAIL;
    }

    mpu_rc = MPU9250_beginGyro(GYRO_FULL_SCALE_250_DPS, &MPU9250);
    if (mpu_rc != MPU9250_STATUS_OK) {
        log_printf("[ERROR] app_mpu9250_runtime_restore_config_only: MPU9250_beginGyro failed rc=%d\r\n",
                   (int)mpu_rc);
        tim2_delay_ms(2);
        return APP_STATUS_INIT_FAIL;
    }

    return APP_STATUS_OK;
}

/* Handles one MPU9250 acquisition cycle with bounded retries. */
app_mpu9250_runtime_status_t app_mpu9250_runtime_process(void)
{
    /* Attempt recovery before returning an unrecoverable sensor error. */
    enum { MAX_ATTEMPTS = 3 };

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
        mpu9250_status_t mpu_rc;

        mpu_rc = MPU_readProcessedData(&MPU9250);
        if (mpu_rc == MPU9250_STATUS_OK) {
            heartbeat_signal_mpu9250main();
            return APP_MPU9250_RUNTIME_OK;
        }

        app_mpu9250_log_runtime_error("MPU_readProcessedData",
                                      (int32_t)mpu_rc,
                                      attempt,
                                      MAX_ATTEMPTS);

        if (attempt < MAX_ATTEMPTS) {
            app_mpu9250_runtime_status_t recovery_status;

            recovery_status = app_mpu9250_recover_and_reinit(attempt, MAX_ATTEMPTS);
            if (recovery_status == APP_MPU9250_RUNTIME_OK) {
                continue;
            }

            log_printf("ACT: MPU9250 short backoff before retry\r\n");
            tim2_delay_ms(2);
            continue;
        }

        log_printf("[ERROR] MPU_readProcessedData failed after %d attempts\r\n", attempt);
        tim2_delay_ms(2);
        return APP_MPU9250_RUNTIME_READ_FAIL;
    }

    log_printf("[ERROR] app_mpu9250_runtime_process: unexpected fallthrough\r\n");
    tim2_delay_ms(2);
    return APP_MPU9250_RUNTIME_READ_FAIL;
}
 

/* Updates the pitch, roll and yaw estimate from measured motion. */
mpu9250_status_t MPU_calcAttitude(MPU9250_t *pMPU9250)
{
    /* Blend the sensor readings using the configured filter settings. */
    float accelPitch;
    float accelRoll;
    mpu9250_status_t status;

    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU_calcAttitude: NULL pointer\r\n");
        tim2_delay_ms(2);
        return MPU9250_STATUS_NULL_PTR;
    }

    status = MPU_readProcessedData(pMPU9250);
    if (status != MPU9250_STATUS_OK) {
        log_printf("[ERROR] MPU_calcAttitude: failed to read processed sensor data, status=%d\r\n",
                   (int)status);
        tim2_delay_ms(2);
        return status;
    }

    accelPitch = atan2f(pMPU9250->sensorData.ay, pMPU9250->sensorData.az) * RAD2DEG;
    accelRoll  = atan2f(pMPU9250->sensorData.ax, pMPU9250->sensorData.az) * RAD2DEG;

    pMPU9250->attitude.r =
        pMPU9250->attitude.tau *
        (pMPU9250->attitude.r - pMPU9250->sensorData.gy * pMPU9250->attitude.dt) +
        (1.0f - pMPU9250->attitude.tau) * accelRoll;

    pMPU9250->attitude.p =
        pMPU9250->attitude.tau *
        (pMPU9250->attitude.p - pMPU9250->sensorData.gx * pMPU9250->attitude.dt) +
        (1.0f - pMPU9250->attitude.tau) * accelPitch;

    pMPU9250->attitude.y += (pMPU9250->sensorData.gz * pMPU9250->attitude.dt);

    return MPU9250_STATUS_OK;
}
