/* mpu9250.h | Author: i fiksuari pas prog | Date: 17 January 2026 */
#ifndef __MPU9250_H__
#define __MPU9250_H__

#include "i2c_dma.h"

// Constants
#define RAD2DEG 57.2957795131
#define G_TO_MS2 9.80665f
#define I2C_DATA_BUFF_LEN    1
#define MAX_TRANSFER_LEN			  6
#define USER_CTRL_I2C_IF_DIS		  (1U<<4)
#define GPIOAEN						  (1U<<0)
#define ACC_FULL_SCALE_2_G            0x00
#define ACC_FULL_SCALE_4_G            0x08
#define ACC_FULL_SCALE_8_G            0x10
#define ACC_FULL_SCALE_16_G           0x18
#define GYRO_FULL_SCALE_250_DPS       0x00
#define GYRO_FULL_SCALE_500_DPS       0x08
#define GYRO_FULL_SCALE_1000_DPS      0x10
#define GYRO_FULL_SCALE_2000_DPS 	  0x18
#define MPU9250_ADDR_ACCELCONFIG  	  0x1C
#define MPU9250_ADDR_GYROCONFIG  	  0x1B
#define MPU9250_ADDR_USER_CTRL		  0x6A
#define MPU9250_ACCEL_XOUT_H		  0x3B
#define MPU9250_GYRO_XOUT_H			  0x43
#define WHO_AM_I_9250_ANS             0x71
#define WHO_AM_I          			  0x75
#define mpu9250_addr           		  0x68
#define AD0_HIGH          			  0x69
#define PWR_MGMT_1        			  0x6B
#define READ_FLAG					  0x80
#define GYRO_SCALE_FACTOR   		  131.0f

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

// Master structure
typedef struct MPU9250
{
    struct RawData
    {
        int16_t ax, ay, az, gx, gy, gz;
    } rawData;

    struct SensorData
    {
        float aScaleFactor, gScaleFactor;
        float ax, ay, az, gx, gy, gz;
    } sensorData;

    struct GyroCal
    {
        float x, y, z;
    } gyroCal;

    struct Attitude
    {
        float tau, dt;
        float r, p, y;
    } attitude;

    struct Range
        {
            float g_accel_range;
            float gyroRange;
        } range;

    struct Settings
    {
    	uint8_t aFullScaleRange, gFullScaleRange;
    } settings;
} MPU9250_t;

int MPU9250_beginGyro(uint8_t mode, MPU9250_t *pMPU9250);
int mpu9250_accel_config(uint8_t mode, MPU9250_t *pMPU9250);
int mpu9250_accel_update_blocking(void);
int mpu9250_gyro_update_blocking(void);
int MPU9250_accelGet(MPU9250_t *pMPU9250);
int MPU9250_gyroGet(MPU9250_t *pMPU9250);
int mpu9250_calibrateGyro(MPU9250_t *pMPU9250, uint16_t numCalPoints);
int MPU_readProcessedData(MPU9250_t *pMPU9250);


#endif
