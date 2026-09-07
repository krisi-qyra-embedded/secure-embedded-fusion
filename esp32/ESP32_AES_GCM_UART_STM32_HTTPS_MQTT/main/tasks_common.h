/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#ifndef MAIN_TASKS_COMMON_H_
#define MAIN_TASKS_COMMON_H_

#define CCS811_TASK_STACK_SIZE              4096 /**< Task stack size. */
#define CCS811_TASK_PRIORITY                3 /**< FreeRTOS task priority. */
#define CCS811_TASK_CORE_ID                 1 /**< Assigned ESP32 core. */

#define BMP280_TASK_STACK_SIZE              4096 /**< Task stack size. */
#define BMP280_TASK_PRIORITY                3 /**< FreeRTOS task priority. */
#define BMP280_TASK_CORE_ID                 1 /**< Assigned ESP32 core. */

#define WIFI_APP_TASK_STACK_SIZE            4096 /**< Task stack size. */
#define WIFI_APP_TASK_PRIORITY              5 /**< FreeRTOS task priority. */
#define WIFI_APP_TASK_CORE_ID               0 /**< Assigned ESP32 core. */

#define HTTP_SERVER_TASK_STACK_SIZE         8192 /**< Task stack size. */
#define HTTP_SERVER_TASK_PRIORITY           4 /**< FreeRTOS task priority. */
#define HTTP_SERVER_TASK_CORE_ID            0 /**< Assigned ESP32 core. */

#define WIFI_RESET_BUTTON_TASK_STACK_SIZE   2048 /**< Task stack size. */
#define WIFI_RESET_BUTTON_TASK_PRIORITY     6 /**< FreeRTOS task priority. */
#define WIFI_RESET_BUTTON_TASK_CORE_ID      0 /**< Assigned ESP32 core. */

#define HTTP_SERVER_MONITOR_STACK_SIZE      4096 /**< Task stack size. */
#define HTTP_SERVER_MONITOR_PRIORITY        3 /**< FreeRTOS task priority. */
#define HTTP_SERVER_MONITOR_CORE_ID         0 /**< Assigned ESP32 core. */

#define SNTP_TIME_SYNC_TASK_STACK_SIZE      4096 /**< Task stack size. */
#define SNTP_TIME_SYNC_TASK_PRIORITY        4 /**< FreeRTOS task priority. */
#define SNTP_TIME_SYNC_TASK_CORE_ID         1 /**< Assigned ESP32 core. */

#define AWS_IOT_TASK_STACK_SIZE             9216 /**< Task stack size. */
#define AWS_IOT_TASK_PRIORITY               6 /**< FreeRTOS task priority. */
#define AWS_IOT_TASK_CORE_ID                1 /**< Assigned ESP32 core. */

#define AES_GCM_RUNTIME_TASK_STACK_SIZE     6144 /**< Task stack size. */
#define AES_GCM_RUNTIME_TASK_PRIORITY       3 /**< FreeRTOS task priority. */
#define AES_GCM_RUNTIME_TASK_CORE_ID        1 /**< Assigned ESP32 core. */
#define AES_GCM_RUNTIME_TASK_PERIOD_MS      1000U /**< Runtime period in milliseconds. */

#endif /* MAIN_TASKS_COMMON_H_ */
