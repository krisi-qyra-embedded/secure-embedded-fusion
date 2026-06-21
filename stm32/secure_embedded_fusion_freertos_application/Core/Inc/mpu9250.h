/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#ifndef __MPU9250_H__
#define __MPU9250_H__

#include <stdint.h>
#include "i2c_dma.h"

/* ----------------------------- Constants ----------------------------- */

#define RAD2DEG                        57.2957795131f
#define G_TO_MS2                       9.80665f

#define I2C_DATA_BUFF_LEN              1U
#define MAX_TRANSFER_LEN               6U

#define MPU_GYRO_CAL_SAMPLES 			2000U

#define USER_CTRL_I2C_IF_DIS           (1U << 4)
#define GPIOAEN                        (1U << 0)

#define ACC_FULL_SCALE_2_G             0x00U
#define ACC_FULL_SCALE_4_G             0x08U
#define ACC_FULL_SCALE_8_G             0x10U
#define ACC_FULL_SCALE_16_G            0x18U

#define GYRO_FULL_SCALE_250_DPS        0x00U
#define GYRO_FULL_SCALE_500_DPS        0x08U
#define GYRO_FULL_SCALE_1000_DPS       0x10U
#define GYRO_FULL_SCALE_2000_DPS       0x18U

#define MPU9250_ADDR_ACCELCONFIG       0x1CU
#define MPU9250_ADDR_GYROCONFIG        0x1BU
#define MPU9250_ADDR_USER_CTRL         0x6AU
#define MPU9250_ACCEL_XOUT_H           0x3BU
#define MPU9250_GYRO_XOUT_H            0x43U
#define WHO_AM_I_9250_ANS              0x71U
#define WHO_AM_I                       0x75U
#define mpu9250_addr                   0x68U
#define AD0_HIGH                       0x69U
#define PWR_MGMT_1                     0x6BU
#define READ_FLAG                      0x80U
#define GYRO_SCALE_FACTOR              131.0f

#define MPU9250_I2C_INSTANCE           I2C3
#define MPU9250_READ_TIMEOUT_ITER      100000U
#define MPU9250_TX_TIMEOUT_ITER        200000U
#define MPU9250_READ_RETRIES           3U

/* ----------------------------- Status / Return codes ----------------------------- */
typedef enum {
    APP_STATUS_OK = 0,
    APP_STATUS_INIT_FAIL = 1,
} app_status_t;

typedef enum
{
    MPU9250_STATUS_OK = 0,
    MPU9250_STATUS_NULL_PTR = -1,
    MPU9250_STATUS_INVALID_ARG = -2,
    MPU9250_STATUS_I2C_WRITE_START_FAILED = -3,
    MPU9250_STATUS_I2C_READ_START_FAILED = -4,
    MPU9250_STATUS_TX_TIMEOUT = -5,
    MPU9250_STATUS_RX_TIMEOUT = -6,
    MPU9250_STATUS_TX_TRANSFER_ERROR = -7,
    MPU9250_STATUS_RX_TRANSFER_ERROR = -8,
    MPU9250_STATUS_RETRIES_EXHAUSTED = -9,
    MPU9250_STATUS_SCALE_FACTOR_ZERO = -10
} mpu9250_status_t;

typedef enum {
    APP_MPU9250_RUNTIME_OK = 0,
    APP_MPU9250_RUNTIME_I2C_RECOVER_FAIL,
    APP_MPU9250_RUNTIME_PERIPH_REINIT_FAIL,
    APP_MPU9250_RUNTIME_SENSOR_REINIT_FAIL,
    APP_MPU9250_RUNTIME_READ_FAIL,
} app_mpu9250_runtime_status_t;

/* ----------------------------- Full-scale ranges ----------------------------- */

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

/* ----------------------------- Master structure ----------------------------- */

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
        uint8_t aFullScaleRange;
        uint8_t gFullScaleRange;
    } settings;
} MPU9250_t;

extern MPU9250_t MPU9250;

/* ----------------------------- Public API ----------------------------- */

mpu9250_status_t MPU9250_beginGyro(uint8_t mode, MPU9250_t *pMPU9250);
mpu9250_status_t mpu9250_accel_config(uint8_t mode, MPU9250_t *pMPU9250);
mpu9250_status_t mpu9250_accel_update_blocking(void);
mpu9250_status_t mpu9250_gyro_update_blocking(void);
mpu9250_status_t MPU9250_accelGet(MPU9250_t *pMPU9250);
mpu9250_status_t MPU9250_gyroGet(MPU9250_t *pMPU9250);
mpu9250_status_t mpu9250_calibrateGyro(MPU9250_t *pMPU9250, uint16_t numCalPoints);
mpu9250_status_t MPU_readProcessedData(MPU9250_t *pMPU9250);
mpu9250_status_t MPU_calcAttitude(MPU9250_t *pMPU9250);
app_status_t app_mpu9250_sensor_stack_init(void);
app_mpu9250_runtime_status_t app_mpu9250_runtime_process(void);
app_status_t app_mpu9250_peripherals_init(void);

#endif
