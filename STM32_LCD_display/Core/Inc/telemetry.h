/**
 * @file telemetry.h
 * @brief Shared sensor telemetry queues and snapshot freshness declarations.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
/* Fixed telemetry layout used by the STM32-ESP32 encrypted link. */
#define TELEMETRY_IAQ_LABEL_SIZE        32U
#define TELEMETRY_DS18B20_COUNT         5U
#define STM32_TELEMETRY_PAYLOAD_LEN     159U
#define STM32_TELEMETRY_FRAME_LEN       169U
#define STM32_TELEMETRY_WIRE_LEN        197U
/** @brief BME680 telemetry sample. */
typedef struct
{
   uint32_t sample_id;
   float temperature_c;
   float pressure_hpa;
   float humidity_pct;
   int16_t iaq_score;
   char iaq_label[TELEMETRY_IAQ_LABEL_SIZE];
} telemetry_bme680_t;
/** @brief MPU9250 telemetry sample. */
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
/** @brief PMS5003 telemetry sample. */
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
/** @brief Five-channel DS18B20 telemetry sample. */
typedef struct
{
   uint32_t sample_id;
   float temperature_c[TELEMETRY_DS18B20_COUNT];
} telemetry_ds18b20_t;
/** @brief MAX31865 telemetry sample. */
typedef struct
{
   uint32_t sample_id;
   float temperature_c;
} telemetry_max31865_t;
/** @brief Complete synchronized sensor snapshot. */
typedef struct
{
   telemetry_bme680_t   bme680;
   telemetry_max31865_t max31865;
   telemetry_ds18b20_t  ds18b20;
   telemetry_pm25_t     pm25;
   telemetry_mpu9250_t  mpu9250;
} telemetry_snapshot_t;
/** @brief Last transmitted sample ID for each sensor source. */
typedef struct
{
   uint32_t bme680_id;
   uint32_t max31865_id;
   uint32_t ds18b20_id;
   uint32_t pm25_id;
   uint32_t mpu9250_id;
} telemetry_consumed_ids_t;
/**
 * @brief Create the five length-one telemetry queues.
 * @return True on success.
 */
bool telemetry_queues_init(void);
/**
 * @brief Publish the latest BME680 sample.
 * @return True on success.
 */
bool telemetry_publish_bme680(
   float temperature_c,
   float pressure_hpa,
   float humidity_pct,
   int16_t iaq_score,
   const char *iaq_label);
/**
 * @brief Publish the latest MPU9250 sample.
 * @return True on success.
 */
bool telemetry_publish_mpu9250(
   float ax,
   float ay,
   float az,
   float gx,
   float gy,
   float gz);
/**
 * @brief Publish the latest PMS5003 sample.
 * @return True on success.
 */
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
/**
 * @brief Publish the latest DS18B20 temperature set.
 * @param temperature_c Five sensor temperatures.
 * @return True on success.
 */
bool telemetry_publish_ds18b20(
   const float temperature_c[TELEMETRY_DS18B20_COUNT]);
/**
 * @brief Publish the latest MAX31865 temperature.
 * @param temperature_c RTD temperature.
 * @return True on success.
 */
bool telemetry_publish_max31865(
   float temperature_c);
/**
 * @brief Advance a telemetry generation counter.
 * @param generation Counter to update.
 * @return New generation value.
 */
uint32_t telemetry_next_generation(uint32_t *generation);
/**
 * @brief Build a snapshot only when all five sources are newer.
 * @param snapshot Snapshot output.
 * @param consumed Last transmitted sample IDs.
 * @return True when a fully fresh snapshot is available.
 */
bool telemetry_peek_fresh_snapshot(
   telemetry_snapshot_t *snapshot,
   const telemetry_consumed_ids_t *consumed);
/**
 * @brief Commit the sample IDs from a successfully transmitted snapshot.
 * @param consumed Last transmitted sample IDs.
 * @param snapshot Successfully transmitted snapshot.
 */
void telemetry_commit_snapshot(
   telemetry_consumed_ids_t *consumed,
   const telemetry_snapshot_t *snapshot);
#endif
