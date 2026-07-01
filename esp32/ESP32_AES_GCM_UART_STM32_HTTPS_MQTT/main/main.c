/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-01
 */

#include "esp_log.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "aws_iot.h"
#include "sntp_time_sync.h"
#include "nvs_flash.h"
#include "wifi_app.h"
#include "ccs811_sensor.h"
#include "wifi_reset_button.h"

static const char TAG[] = "main";

/* Starts clock synchronization and AWS after station connection. */
void wifi_application_connected_events(void)
{
	ESP_LOGI(TAG, "WiFi Application Connected!!");
	/* Start the SNTP task before launching the MQTT task. */
	sntp_time_sync_task_start();
	aws_iot_start();
}

/* Initializes storage and starts the gas sensor test. */
void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		/* Erase incompatible or full NVS before retrying initialization. */
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
    CCS811_task_start();

}
