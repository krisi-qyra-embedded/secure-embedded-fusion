/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#ifndef MAIN_BMP280_SENSOR_H_
#define MAIN_BMP280_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    bool valid;
    uint8_t chip_id;
    bool humidity_available;
    float temperature_c;
    float pressure_pa;
    float pressure_hpa;
    float humidity_percent;
} bmp280_sensor_data_t;

void BMP280_task_start(void);
void BME280_task_start(void);

bool BMP280_get_latest_reading(bmp280_sensor_data_t *out);
bool BME280_get_latest_reading(bmp280_sensor_data_t *out);

#endif /* MAIN_BMP280_SENSOR_H_ */