/* mpu9250.c | Author: i fiksuari pas prog | Date: 17 January 2026 */
#include "mpu9250.h"
#include"tim.h"
#include "i2c_dma.h"
#include "uart.h"
/**@PINOUT

 *	PB8 :  SCL
 *	PB9 :  SDA
*/
#define MPU_READ_TIMEOUT_ITER  100000U   /* adjust for your CPU clock if needed */
#define MPU_READ_RETRIES        3U

volatile uint8_t dummy_buff[MAX_TRANSFER_LEN + 1];
volatile uint8_t accel_buff[MAX_TRANSFER_LEN + 1];
volatile uint8_t gyro_buff[MAX_TRANSFER_LEN + 1];
volatile uint8_t i2c_data_buff[I2C_DATA_BUFF_LEN];

double g_accel_range,gyroRange;
float gyro_x_offset, gyro_y_offset, gyro_z_offset;
float gyro_gx,gyro_gy,gyro_gz;
float tau=0.98f, dt=0.004f;
float r=0.0f, p=0.0f, y=0.0f;
uint8_t firstRun = 1;

typedef enum {
    DMA_OK = 0,
    DMA_ERR_DISABLE_TIMEOUT,
} dma_status_t;

typedef enum {
    DMA_DIR_P2M = 0,   // Peripheral-to-Memory
    DMA_DIR_M2P = 1,   // Memory-to-Peripheral
} dma_direction_t;

/* Return codes: 0 = success -1 = NULL pointer -2 = DMA / TX timeout */
/* Configures the accelerometer measurement range. */
int mpu9250_accel_config(uint8_t mode, MPU9250_t *pMPU9250)
{
    /* Select the range and update its conversion scale. */
    if (pMPU9250 == NULL) {
        log_printf("[ERROR] mpu9250_accel_config: NULL pointer  \r\n");
        return -1;
    }

    uint8_t reg_val = 0;
    float accel_range = 2.0f;
    float ascale = 16384.0f;  

    switch (mode) {
        case ACC_FULL_SCALE_2_G:
            accel_range = 2.0f;
            ascale = 16384.0f;
            reg_val = 0x00;
            break;
        case ACC_FULL_SCALE_4_G:
            accel_range = 4.0f;
            ascale = 8192.0f;
            reg_val = 0x08;
            break;
        case ACC_FULL_SCALE_8_G:
            accel_range = 8.0f;
            ascale = 4096.0f;
            reg_val = 0x10;
            break;
        case ACC_FULL_SCALE_16_G:
            accel_range = 16.0f;
            ascale = 2048.0f;
            reg_val = 0x18;
            break;
        default:
            log_printf("[WARN] mpu9250_accel_config: unknown mode 0x%02X, falling back to ±2g  \r\n", (unsigned)mode);
            accel_range = 2.0f;
            ascale = 16384.0f;
            reg_val = 0x00;
            break;
    }

     
    pMPU9250->range.g_accel_range = accel_range;
    pMPU9250->sensorData.aScaleFactor = ascale;

     
    i2c_data_buff[0] = MPU9250_ADDR_ACCELCONFIG;

    /* Kick off DMA TX and wait (bounded) for completion */
    g_tx_cmplt = 0;
	dma_status_t status = i2c_dma_write_reg(I2C3, mpu9250_addr, reg_val,i2c_data_buff, I2C_DATA_BUFF_LEN);

	/* Write mode to ACCELCONFIG register */
		if (status != DMA_OK) {
		    log_printf("[ERROR] I2C DMA write failed, status=%d \r\n", status);
		    /* handle error: retry, blink LED, or system reset */
		    return -1;
		}

    const uint32_t tx_timeout = 200000u;  
    uint32_t t = 0;
    while (!g_tx_cmplt) {
        if (++t > tx_timeout) {
            log_printf("[ERROR] mpu9250_accel_config: i2c DMA TX timeout  \r\n");
            g_tx_cmplt = 0;
            return -2;
        }
        __asm__ volatile("nop");
    }
    g_tx_cmplt = 0;  

    return 0;
}


/* Return codes: 0 = success -1 = NULL pointer -2 = DMA / TX timeout */
/* Configures the gyroscope measurement range. */
int MPU9250_beginGyro(uint8_t mode, MPU9250_t *pMPU9250)
{
    /* Program the sensor range and retain its scale factor. */
    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU9250_beginGyro: NULL pointer \r\n");
        return -1;
    }

    uint8_t reg_val = 0;
    float gyro_range = 500.0f;
    float gscale = 65.5f;  

    switch (mode) {
        case GYRO_FULL_SCALE_250_DPS:
            gyro_range = 250.0f;
            gscale = 131.0f;
            reg_val = 0x00;
            break;
        case GYRO_FULL_SCALE_500_DPS:
            gyro_range = 500.0f;
            gscale = 65.5f;
            reg_val = 0x08;
            break;
        case GYRO_FULL_SCALE_1000_DPS:
            gyro_range = 1000.0f;
            gscale = 32.8f;
            reg_val = 0x10;
            break;
        case GYRO_FULL_SCALE_2000_DPS:
            gyro_range = 2000.0f;
            gscale = 16.4f;
            reg_val = 0x18;
            break;
        default:
            log_printf("[WARN] MPU9250_beginGyro: unknown mode 0x%02X, falling back to 500 DPS  \r\n", (unsigned)mode);
            gyro_range = 500.0f;
            gscale = 65.5f;
            reg_val = 0x08;
            break;
    }

     
    pMPU9250->range.gyroRange = gyro_range;
    pMPU9250->sensorData.gScaleFactor = gscale;

    /* Prepare register write (ACC/GYRO config register address constant used) */
    i2c_data_buff[0] = MPU9250_ADDR_GYROCONFIG;

    /* kick off DMA TX and wait (bounded) for completion */
    g_tx_cmplt = 0;
  	 
  	dma_status_t status = i2c_dma_write_reg(I2C3, mpu9250_addr, reg_val,i2c_data_buff, I2C_DATA_BUFF_LEN);
  	/* Write mode to ACCELCONFIG register */
  	  	if (status != DMA_OK) {
  	  		log_printf("[ERROR] I2C DMA write failed, status=%d\r\n", status);
  	  		/* handle error: retry, blink LED, or system reset */
  	  		return -1;
  	  	}
    /* Wait for DMA transfer complete (small software timeout) */
    const uint32_t tx_timeout = 200000u;
    uint32_t t = 0;
    while (!g_tx_cmplt) {
        if (++t > tx_timeout) {
            log_printf("[ERROR] MPU9250_beginGyro: i2c DMA TX timeout \r\n");
            g_tx_cmplt = 0;  
            return -2;
        }
        __asm__ volatile ("nop");
    }
    g_tx_cmplt = 0;  

    return 0;
}


// blocking wrapper for accel read (starts DMA then waits for completion)
/* Reads the accelerometer registers through the blocking transfer path. */
int mpu9250_accel_update_blocking(void) {
    /* Check the transfer before converting the received bytes. */
    uint8_t attempt = 0;
    while (attempt < MPU_READ_RETRIES) {
    	dma_status_t st;
         
        g_rx_cmplt = 0;
        st = i2c_dma_read(I2C3, mpu9250_addr, MPU9250_ACCEL_XOUT_H,
                          (uint8_t *)accel_buff, (uint16_t)MAX_TRANSFER_LEN);
        if (st != DMA_OK) {
            log_printf("[WARN] i2c_dma_read returned %d (attempt %u) \r\n", (int)st, attempt + 1);
            attempt++;
            continue;  
        }
        /* wait for DMA IRQ to set g_rx_cmplt (bounded) */
        uint32_t t = 0U;
        while (!g_rx_cmplt && (t++ < MPU_READ_TIMEOUT_ITER)) {
            __asm__ volatile ("nop");
        }
        if (g_rx_cmplt) {
            g_rx_cmplt = 0;  
            return 0;        
        }
        /* timeout: try lightweight recovery and retry */
        log_printf("[WARN] mpu read timeout (attempt %u) \r\n", attempt + 1);
        /* request STOP to free the bus (best-effort) */
        I2C1->CR1 |= I2C_CR1_STOP;

        attempt++;
    }
    /* all attempts failed -> fatal recovery: system reset */
    log_printf("[FATAL] mpu read failed after %u attempts -> system reset \r\n", MPU_READ_RETRIES);
    return -1;  
}

/* Reads the gyroscope registers through the blocking transfer path. */
int mpu9250_gyro_update_blocking(void)
{
    /* Check the transfer before converting the received bytes. */
    uint8_t attempt = 0;
    while (attempt < MPU_READ_RETRIES) {
    	dma_status_t st;
         
        g_rx_cmplt = 0;
        st = i2c_dma_read(I2C3, mpu9250_addr, MPU9250_GYRO_XOUT_H,
                          (uint8_t *)gyro_buff, (uint16_t)MAX_TRANSFER_LEN);
        if (st != DMA_OK) {
            log_printf("[WARN] i2c_dma_read returned %d (attempt %u) \r\n", (int)st, attempt + 1);
            attempt++;
            continue;  
        }

        /* wait for DMA IRQ to set g_rx_cmplt (bounded) */
        uint32_t t = 0U;
        while (!g_rx_cmplt && (t++ < MPU_READ_TIMEOUT_ITER)) {
            __asm__ volatile ("nop");
        }

        if (g_rx_cmplt) {
            g_rx_cmplt = 0;  
            return 0;        
        }

        /* timeout: try lightweight recovery and retry */
        log_printf("[WARN] mpu gyro read timeout (attempt %u) \r\n", attempt + 1);
        /* request STOP to free the bus (best-effort) -- use same I2C instance we read from */
        I2C1->CR1 |= I2C_CR1_STOP;
        attempt++;
    }

    /* all attempts failed -> report failure to caller (caller may choose to reset) */
    log_printf("[ERROR] mpu gyro read failed after %u attempts -> giving up (caller must recover) \r\n", MPU_READ_RETRIES);
    return -1;  
}


 

/* Returns the accelerometer values in scaled units. */
int MPU9250_accelGet(MPU9250_t *pMPU9250)
{
    /* Use the configured sensitivity when converting the raw reading. */
    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU9250_accelGet: NULL pointer passed \r\n");
        return -1;
    }

    if (mpu9250_accel_update_blocking() != 0) {
        log_printf("[ERROR] MPU9250_accelGet: accel DMA/read failed \r\n");
        return -2;
    }

     
    pMPU9250->rawData.ax = (int16_t)(((uint16_t)accel_buff[0] << 8) | (uint16_t)accel_buff[1]);
    pMPU9250->rawData.ay = (int16_t)(((uint16_t)accel_buff[2] << 8) | (uint16_t)accel_buff[3]);
    pMPU9250->rawData.az = (int16_t)(((uint16_t)accel_buff[4] << 8) | (uint16_t)accel_buff[5]);

    return 0;
}

/* Returns the gyroscope values in scaled units. */
int MPU9250_gyroGet(MPU9250_t *pMPU9250)
{
    /* Apply the calibration offset to the gyro reading. */
    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU9250_gyroGet: NULL pointer passed \r\n");
        return -1;
    }
    if (mpu9250_gyro_update_blocking() != 0) {
        log_printf("[ERROR] MPU9250_gyroGet: gyro DMA/read failed \r\n");
        return -2;
    }
     
    pMPU9250->rawData.gx = (int16_t)(((uint16_t)gyro_buff[0] << 8) | (uint16_t)gyro_buff[1]);
    pMPU9250->rawData.gy = (int16_t)(((uint16_t)gyro_buff[2] << 8) | (uint16_t)gyro_buff[3]);
    pMPU9250->rawData.gz = (int16_t)(((uint16_t)gyro_buff[4] << 8) | (uint16_t)gyro_buff[5]);

    return 0;
}

/* Calculates gyroscope offsets from a set of reference samples. */
int mpu9250_calibrateGyro(MPU9250_t *pMPU9250, uint16_t numCalPoints)
{
    /* Accumulate samples only while the device is reporting valid data. */
    if (pMPU9250 == NULL) {
        log_printf("[ERROR] mpu9250_calibrateGyro: NULL pointer\r\n");
        return -1;
    }
    if (numCalPoints == 0) {
        numCalPoints = 1000U;  
    }

    float sx = 0.0f, sy = 0.0f, sz = 0.0f;

    log_printf("Gyro calibration: %u samples. Keep device perfectly still...\r\n", (unsigned)numCalPoints);

    for (uint16_t i = 0; i < numCalPoints; i++) {
         
        if (MPU9250_gyroGet(pMPU9250) != 0) {
            log_printf("[ERROR] gyro read timeout during calibration (sample %u)\r\n", i);
            return -2;
        }

         
        float gx = (float)pMPU9250->rawData.gx;
        float gy = (float)pMPU9250->rawData.gy;
        float gz = (float)pMPU9250->rawData.gz;

        sx += gx;
        sy += gy;
        sz += gz;

        tim2_3ms_delay();  
    }

     
    pMPU9250->gyroCal.x = sx / (float)numCalPoints;
    pMPU9250->gyroCal.y = sy / (float)numCalPoints;
    pMPU9250->gyroCal.z = sz / (float)numCalPoints;

    log_printf("Calibration done. Offsets (°/s): X=%.3f Y=%.3f Z=%.3f\r\n",
               pMPU9250->gyroCal.x, pMPU9250->gyroCal.y, pMPU9250->gyroCal.z);

    return 0;  
}


/// @brief Read sensor raw data (via DMA blocking helpers) and compute scaled values
/// @param pMPU9250 Pointer to master MPU9250 struct
/// @return 0 on success, negative on error
/* Reads accelerometer and gyroscope data and applies the configured processing. */
int MPU_readProcessedData(MPU9250_t *pMPU9250)
{
    /* Return a failure when the sensor transfer or data conversion fails. */
    if (pMPU9250 == NULL) {
        log_printf("[ERROR] MPU_readProcessedData: NULL pointer  \r\n");
        return -1;
    }
     
    if (MPU9250_accelGet(pMPU9250) != 0) {
        log_printf("[ERROR] MPU_readProcessedData: accel read timeout/failure  \r\n");
        return -2;
    }
    // Convert accelerometer values to g's
        pMPU9250->sensorData.ax = pMPU9250->rawData.ax / pMPU9250->sensorData.aScaleFactor;
        pMPU9250->sensorData.ay = pMPU9250->rawData.ay / pMPU9250->sensorData.aScaleFactor;
        pMPU9250->sensorData.az = pMPU9250->rawData.az / pMPU9250->sensorData.aScaleFactor;

        // Print accelerometer data
        log_printf("ACCEL: X=%.3f g , Y=%.3f g , Z=%.3f g \r\n",
        pMPU9250->sensorData.ax,
        pMPU9250->sensorData.ay,
        pMPU9250->sensorData.az);

         
    if (MPU9250_gyroGet(pMPU9250) != 0) {
        log_printf("[ERROR] MPU_readProcessedData: gyro read timeout/failure  \r\n");
        return -3;
    }
    // Compensate for gyro offset
        pMPU9250->sensorData.gx = pMPU9250->rawData.gx - pMPU9250->gyroCal.x;
        pMPU9250->sensorData.gy = pMPU9250->rawData.gy - pMPU9250->gyroCal.y;
        pMPU9250->sensorData.gz = pMPU9250->rawData.gz - pMPU9250->gyroCal.z;
        // Convert gyro values to deg/s
        pMPU9250->sensorData.gx /= pMPU9250->sensorData.gScaleFactor;
        pMPU9250->sensorData.gy /= pMPU9250->sensorData.gScaleFactor;
        pMPU9250->sensorData.gz /= pMPU9250->sensorData.gScaleFactor;

        // Print gyro data
        log_printf("GYRO:  X=%.3f dps , Y=%.3f dps , Z=%.3f dps \r\n",
        pMPU9250->sensorData.gx,
        pMPU9250->sensorData.gy,
        pMPU9250->sensorData.gz);

    return 0;  
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter
/// @param SPIx Pointer to SPI structure config
/// @param pMPU9250 Pointer to master MPU9250 struct
/* Updates the pitch, roll and yaw estimate from measured motion. */
void MPU_calcAttitude(MPU9250_t *pMPU9250)
{
	/* Blend the sensor readings using the configured filter settings. */
	 
	if (pMPU9250 == NULL) {
	    log_printf("[ERROR] MPU_calcAttitude: NULL pointer  \r\n");
	    return -1;
		}
	 
	if (MPU_readProcessedData(pMPU9250) != 0) {
	    log_printf("[ERROR] MPU_calcAttitude: failed to read processed sensor data – attitude calculation aborted  \r\n");
	    return -2;
	}

    // Complementary filter
    float accelPitch = atan2(pMPU9250->sensorData.ay, pMPU9250->sensorData.az) * RAD2DEG;
    float accelRoll = atan2(pMPU9250->sensorData.ax, pMPU9250->sensorData.az) * RAD2DEG;

    pMPU9250->attitude.r = pMPU9250->attitude.tau * (pMPU9250->attitude.r - pMPU9250->sensorData.gy * pMPU9250->attitude.dt) + (1 - pMPU9250->attitude.tau) * accelRoll;
    pMPU9250->attitude.p = pMPU9250->attitude.tau * (pMPU9250->attitude.p - pMPU9250->sensorData.gx * pMPU9250->attitude.dt) + (1 - pMPU9250->attitude.tau) * accelPitch;
    pMPU9250->attitude.y += (pMPU9250->sensorData.gz * pMPU9250->attitude.dt);
    return 0;
}




