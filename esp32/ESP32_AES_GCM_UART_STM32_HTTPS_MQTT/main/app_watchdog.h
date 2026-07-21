/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

#include <stdbool.h>

typedef enum
{
    APP_WD_CCS811 = 0,
    APP_WD_BME280,
    APP_WD_WIFI,
    APP_WD_AWS,
    APP_WD_HTTPS,
    APP_WD_COUNT
} app_watchdog_task_id_t;

void app_watchdog_init(void);
void app_watchdog_task_start(void);

void app_watchdog_kick(app_watchdog_task_id_t id);
void app_watchdog_set_enabled(app_watchdog_task_id_t id, bool enabled);

#endif