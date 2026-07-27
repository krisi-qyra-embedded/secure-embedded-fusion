/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#ifndef MAIN_TASKS_COMMON_H_
#define MAIN_TASKS_COMMON_H_

#define CCS811_TASK_STACK_SIZE              4096
#define CCS811_TASK_PRIORITY                3
#define CCS811_TASK_CORE_ID                 1

#define BMP280_TASK_STACK_SIZE              4096
#define BMP280_TASK_PRIORITY                3
#define BMP280_TASK_CORE_ID                 1

#define WIFI_APP_TASK_STACK_SIZE            4096
#define WIFI_APP_TASK_PRIORITY              5
#define WIFI_APP_TASK_CORE_ID               0

#define HTTP_SERVER_TASK_STACK_SIZE         8192
#define HTTP_SERVER_TASK_PRIORITY           4
#define HTTP_SERVER_TASK_CORE_ID            0

#define WIFI_RESET_BUTTON_TASK_STACK_SIZE   2048
#define WIFI_RESET_BUTTON_TASK_PRIORITY     6
#define WIFI_RESET_BUTTON_TASK_CORE_ID      0

#define HTTP_SERVER_MONITOR_STACK_SIZE      4096
#define HTTP_SERVER_MONITOR_PRIORITY        3
#define HTTP_SERVER_MONITOR_CORE_ID         0

#define SNTP_TIME_SYNC_TASK_STACK_SIZE      4096
#define SNTP_TIME_SYNC_TASK_PRIORITY        4
#define SNTP_TIME_SYNC_TASK_CORE_ID         1

#define AWS_IOT_TASK_STACK_SIZE             9216
#define AWS_IOT_TASK_PRIORITY               6
#define AWS_IOT_TASK_CORE_ID                1

#define AES_GCM_RUNTIME_TASK_STACK_SIZE     6144
#define AES_GCM_RUNTIME_TASK_PRIORITY       5
#define AES_GCM_RUNTIME_TASK_CORE_ID        1
#define AES_GCM_RUNTIME_TASK_PERIOD_MS      1000U

#endif /* MAIN_TASKS_COMMON_H_ */
