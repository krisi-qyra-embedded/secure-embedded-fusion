/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"

#define TELEMETRY_IAQ_LABEL_SIZE        32U
#define TELEMETRY_DS18B20_COUNT         5U
#define STM32_TELEMETRY_PAYLOAD_LEN     159U
#define STM32_TELEMETRY_FRAME_LEN       169U
#define STM32_TELEMETRY_WIRE_LEN        197U

typedef struct
{
    uint32_t sample_id;

    float temperature_c;
    float pressure_hpa;
    float humidity_pct;

    int16_t iaq_score;
    char iaq_label[TELEMETRY_IAQ_LABEL_SIZE];

} telemetry_bme680_t;

typedef struct
{
    uint32_t sample_id;

    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

} telemetry_mpu9250_t;

typedef struct
{
    uint32_t sample_id;

    uint16_t pm1_0_standard;
    uint16_t pm2_5_standard;
    uint16_t pm10_standard;

    uint16_t pm1_0_atmospheric;
    uint16_t pm2_5_atmospheric;
    uint16_t pm10_atmospheric;

    uint16_t particles_0_3um;
    uint16_t particles_0_5um;
    uint16_t particles_1_0um;
    uint16_t particles_2_5um;
    uint16_t particles_5_0um;
    uint16_t particles_10um;

} telemetry_pm25_t;

typedef struct
{
    uint32_t sample_id;

    float temperature_c[TELEMETRY_DS18B20_COUNT];

} telemetry_ds18b20_t;

typedef struct
{
    uint32_t sample_id;

    float temperature_c;

} telemetry_max31865_t;

typedef struct
{
    telemetry_bme680_t   bme680;
    telemetry_max31865_t max31865;
    telemetry_ds18b20_t  ds18b20;
    telemetry_pm25_t     pm25;
    telemetry_mpu9250_t  mpu9250;

} telemetry_snapshot_t;

typedef struct
{
    uint32_t bme680_id;
    uint32_t max31865_id;
    uint32_t ds18b20_id;
    uint32_t pm25_id;
    uint32_t mpu9250_id;

} telemetry_consumed_ids_t;

bool telemetry_queues_init(void);

bool telemetry_publish_bme680(
    float temperature_c,
    float pressure_hpa,
    float humidity_pct,
    int16_t iaq_score,
    const char *iaq_label);

bool telemetry_publish_mpu9250(
    float ax,
    float ay,
    float az,
    float gx,
    float gy,
    float gz);

bool telemetry_publish_pm25(
    uint16_t pm1_0_standard,
    uint16_t pm2_5_standard,
    uint16_t pm10_standard,
    uint16_t pm1_0_atmospheric,
    uint16_t pm2_5_atmospheric,
    uint16_t pm10_atmospheric,
    uint16_t particles_0_3um,
    uint16_t particles_0_5um,
    uint16_t particles_1_0um,
    uint16_t particles_2_5um,
    uint16_t particles_5_0um,
    uint16_t particles_10um);

bool telemetry_publish_ds18b20(
    const float temperature_c[TELEMETRY_DS18B20_COUNT]);

bool telemetry_publish_max31865(
    float temperature_c);

uint32_t telemetry_next_generation(uint32_t *generation);

bool telemetry_peek_fresh_snapshot(
    telemetry_snapshot_t *snapshot,
    const telemetry_consumed_ids_t *consumed);

void telemetry_commit_snapshot(
    telemetry_consumed_ids_t *consumed,
    const telemetry_snapshot_t *snapshot);

#endif
