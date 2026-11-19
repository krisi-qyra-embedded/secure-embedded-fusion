/**
 * @file telemetry.c
 * @brief Shared sensor telemetry queues and snapshot freshness tracking.
 */
#include "telemetry.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
QueueHandle_t q_bme680   = NULL;
QueueHandle_t q_mpu9250  = NULL;
QueueHandle_t q_pm25     = NULL;
QueueHandle_t q_ds18b20  = NULL;
QueueHandle_t q_max31865 = NULL;
uint32_t bme680_generation   = 0U;
uint32_t mpu9250_generation  = 0U;
uint32_t pm25_generation     = 0U;
uint32_t ds18b20_generation  = 0U;
uint32_t max31865_generation = 0U;
/**
 * @brief Advance one sensor generation counter.
 * @param generation Generation counter to update.
 * @return New non-zero generation value.
 */
 uint32_t telemetry_next_generation(uint32_t *generation)
{
   (*generation)++;
   /* Zero stays reserved for "no sample produced yet". */
   if (*generation == 0U)
   {
       *generation = 1U;
   }
   return *generation;
}
/**
 * @brief Create the latest-value telemetry queues.
 * @return true when all sensor queues are created.
 */
bool telemetry_queues_init(void)
{
   /* Each queue keeps only the latest sample from one sensor. */
   q_bme680 =
       xQueueCreate(1U, sizeof(telemetry_bme680_t));
   q_mpu9250 =
       xQueueCreate(1U, sizeof(telemetry_mpu9250_t));
   q_pm25 =
       xQueueCreate(1U, sizeof(telemetry_pm25_t));
   q_ds18b20 =
       xQueueCreate(1U, sizeof(telemetry_ds18b20_t));
   q_max31865 =
       xQueueCreate(1U, sizeof(telemetry_max31865_t));
   return
       (q_bme680   != NULL) &&
       (q_mpu9250  != NULL) &&
       (q_pm25     != NULL) &&
       (q_ds18b20  != NULL) &&
       (q_max31865 != NULL);
}
/**
 * @brief Peek one complete telemetry snapshot and check freshness.
 * @param snapshot Snapshot output.
 * @param consumed Last committed sample IDs.
 * @return true when every sensor has a fresh sample.
 */
bool telemetry_peek_fresh_snapshot(
   telemetry_snapshot_t *snapshot,
   const telemetry_consumed_ids_t *consumed)
{
   if ((snapshot == NULL) || (consumed == NULL))
   {
       return false;
   }
   /* Build the snapshot without consuming queue contents. */
   if (xQueuePeek(
           q_bme680,
           &snapshot->bme680,
           0U) != pdPASS)
   {
       return false;
   }
   if (xQueuePeek(
           q_max31865,
           &snapshot->max31865,
           0U) != pdPASS)
   {
       return false;
   }
   if (xQueuePeek(
           q_ds18b20,
           &snapshot->ds18b20,
           0U) != pdPASS)
   {
       return false;
   }
   if (xQueuePeek(
           q_pm25,
           &snapshot->pm25,
           0U) != pdPASS)
   {
       return false;
   }
   if (xQueuePeek(
           q_mpu9250,
           &snapshot->mpu9250,
           0U) != pdPASS)
   {
       return false;
   }
   /* Every sensor must be newer than the last committed transaction. */
   if (snapshot->bme680.sample_id ==
       consumed->bme680_id)
   {
       return false;
   }
   if (snapshot->max31865.sample_id ==
       consumed->max31865_id)
   {
       return false;
   }
   if (snapshot->ds18b20.sample_id ==
       consumed->ds18b20_id)
   {
       return false;
   }
   if (snapshot->pm25.sample_id ==
       consumed->pm25_id)
   {
       return false;
   }
   if (snapshot->mpu9250.sample_id ==
       consumed->mpu9250_id)
   {
       return false;
   }
   return true;
}
/**
 * @brief Commit sample IDs from an accepted telemetry snapshot.
 * @param consumed Stored consumed IDs.
 * @param snapshot Snapshot that was successfully used.
 */
void telemetry_commit_snapshot(
   telemetry_consumed_ids_t *consumed,
   const telemetry_snapshot_t *snapshot)
{
   /* Commit only after the complete snapshot has been accepted. */
   consumed->bme680_id =
       snapshot->bme680.sample_id;
   consumed->max31865_id =
       snapshot->max31865.sample_id;
   consumed->ds18b20_id =
       snapshot->ds18b20.sample_id;
   consumed->pm25_id =
       snapshot->pm25.sample_id;
   consumed->mpu9250_id =
       snapshot->mpu9250.sample_id;
}
