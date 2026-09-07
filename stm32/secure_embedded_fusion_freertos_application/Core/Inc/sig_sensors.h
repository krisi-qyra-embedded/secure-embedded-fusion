/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef SIG_SENSORS_H_
#define SIG_SENSORS_H_
#include <stdint.h>
#include <math.h>
#include <bme680_defs.h>
#include <bme680.h>
#ifdef __cplusplus
extern "C" {
#endif
#define SIG_SENS_TOTAL_RETRIES           (3U)
#define SIG_SENS_LOG_BACKOFF_MS          (2U)
#define SIG_SENS_BACKOFF_BASE_MS         (10U)
#define SIG_SENS_GASREF_VALID_SAMPLES    (10U)
#define SIG_SENS_GASREF_MAX_SAMPLE_LOOPS (20U)
typedef enum {
   SIG_SENS_OK                  = 0,
   SIG_SENS_E_INVALID_ARG       = -1,
   SIG_SENS_E_BME_INIT_FAIL     = -2,
   SIG_SENS_E_BME_SETTINGS_FAIL = -3,
   SIG_SENS_E_BME_MODE_FAIL     = -4,
   SIG_SENS_E_BME_PROFILE_FAIL  = -5,
   SIG_SENS_E_GASREF_FAIL       = -6,
   SIG_SENS_E_NO_VALID_SAMPLES  = -7,
   SIG_SENS_E_SENSOR_READ_FAIL  = -8,
   SIG_SENS_E_BAD_LIMITS        = -9,
   SIG_SENS_E_NUMERIC_FAIL      = -10
} sig_sensors_status_t;
typedef struct {
   uint16_t voltage;
} st_battery_t;
typedef struct {
   struct bme680_dev gas_sensor;
   float    temperature;
   float    pressure;
   float    humidity;
   uint32_t gas_resistance;
   uint16_t meas_period;
   float    gas_weighting;
   float    gas_reference;
   uint8_t  getgasreference_count;
   float    hum_weighting;
   float    hum_reference;
   int32_t  gas_lower_limit;
   int32_t  gas_upper_limit;
} st_bme680_t;
extern volatile st_bme680_t bme;
extern volatile st_battery_t bat;
void fnAPP_Init_Standard_Values(void);
int8_t  init_bme680(void);
int8_t  bme_GetGasReference(void);
int16_t bme_GetGasScore(void);
int16_t bme_GetHumidityScore(void);
int16_t bme_calculateIAQ(
   int16_t score,
   const char **label_out);
sensor_stack_init_status_t app_bme680_sensor_stack_init(void);
app_bme680_runtime_status_t app_bme680_sensor_runtime_process(void);
#ifdef __cplusplus
}
#endif
#endif /* SIG_SENSORS_H_ */
