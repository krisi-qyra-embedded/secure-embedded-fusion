/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#ifndef MAIN_CCS811_SENSOR_H_
#define MAIN_CCS811_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

#define CCS811_MAX_CONSECUTIVE_FAILURES 5
#define CCS811_RECOVERY_DELAY_MS        500

typedef struct
{
    bool valid;
    uint16_t eco2_ppm;
    uint16_t tvoc_ppb;
    uint8_t raw_current_ua;
    uint16_t raw_voltage_adc;
    float raw_voltage;
} ccs811_sensor_data_t;

void CCS811_task_start(void);
bool CCS811_get_latest_reading(ccs811_sensor_data_t *out);

#endif /* MAIN_CCS811_SENSOR_H_ */