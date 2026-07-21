/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/apps/sntp.h"

#include "tasks_common.h"
#include "http_server.h"
#include "sntp_time_sync.h"
#include "wifi_app.h"

static const char TAG[] = "sntp_time_sync";

static bool sntp_op_mode_set = false;

/* Starts SNTP polling and notifies the HTTP monitor. */
static void sntp_time_sync_init_sntp(void)
{
	ESP_LOGI(TAG, "Initializing the SNTP service");

	/* Set the operating mode once before starting the SNTP client. */
	if (!sntp_op_mode_set)
	{
		sntp_setoperatingmode(SNTP_OPMODE_POLL);
		sntp_op_mode_set = true;
	}

	sntp_setservername(0, "pool.ntp.org");

	sntp_init();

	http_server_monitor_send_message(HTTP_MSG_TIME_SERVICE_INITIALIZED);
}

/* Starts time synchronization when the system clock is unset. */
static void sntp_time_sync_obtain_time(void)
{
	time_t now = 0;
	struct tm time_info = {0};

	time(&now);
	localtime_r(&now, &time_info);

	/* A clock before the configured year threshold triggers SNTP setup. */
	if (time_info.tm_year < (2016 - 1900))
	{
		sntp_time_sync_init_sntp();
		setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
		tzset();
	}
}

/* Periodically checks whether SNTP initialization is needed. */
static void sntp_time_sync(void *pvParam)
{
	while (1)
	{
		sntp_time_sync_obtain_time();
		/* Wait ten seconds between clock checks. */
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}

	vTaskDelete(NULL);
}

/* Formats the current local time into a shared buffer. */
char* sntp_time_sync_get_time(void)
{
	static char time_buffer[100] = {0};

	time_t now = 0;
	struct tm time_info = {0};

	time(&now);
	localtime_r(&now, &time_info);

	/* Leave the existing buffer untouched until the clock is set. */
	if (time_info.tm_year < (2016 - 1900))
	{
		ESP_LOGI(TAG, "Time is not set yet");
	}
	else
	{
		strftime(time_buffer, sizeof(time_buffer), "%d.%m.%Y %H:%M:%S", &time_info);
		ESP_LOGI(TAG, "Current time info: %s", time_buffer);
	}

	return time_buffer;
}

/* Starts the periodic clock-check task. */
void sntp_time_sync_task_start(void)
{
	/* Use the configured stack, priority and core for SNTP polling. */
	xTaskCreatePinnedToCore(&sntp_time_sync, "sntp_time_sync", SNTP_TIME_SYNC_TASK_STACK_SIZE, NULL, SNTP_TIME_SYNC_TASK_PRIORITY, NULL, SNTP_TIME_SYNC_TASK_CORE_ID);
}

