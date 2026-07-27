/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "tasks_common.h"
#include "wifi_app.h"
#include "wifi_reset_button.h"

static const char TAG[] = "wifi_reset_button";

SemaphoreHandle_t wifi_reset_semphore = NULL;

/* Signals a BOOT button press to the reset task. */
void IRAM_ATTR wifi_reset_button_isr_handler(void *arg)
{
	/* Wake the task through the ISR-safe semaphore operation. */
	xSemaphoreGiveFromISR(wifi_reset_semphore, NULL);
}

/* Requests station disconnection after a BOOT button interrupt. */
void wifi_reset_button_task(void *pvParam)
{
	for (;;)
	{
		/* Wait on the semaphore before sending the Wi-Fi reset request. */
		if (xSemaphoreTake(wifi_reset_semphore, portMAX_DELAY) ==  pdTRUE)
		{
			ESP_LOGI(TAG, "WIFI RESET BUTTON INTERRUPT OCCURRED");

			wifi_app_send_message(WIFI_APP_MSG_USER_REQUESTED_STA_DISCONNECT);

			vTaskDelay(2000 / portTICK_PERIOD_MS);
		}
	}
}

/* Configures the BOOT button interrupt and reset task. */
void wifi_reset_button_config(void)
{
	wifi_reset_semphore = xSemaphoreCreateBinary();

	esp_rom_gpio_pad_select_gpio(WIFI_RESET_BUTTON);
	gpio_set_direction(WIFI_RESET_BUTTON, GPIO_MODE_INPUT);

	gpio_set_intr_type(WIFI_RESET_BUTTON, GPIO_INTR_NEGEDGE);

	xTaskCreatePinnedToCore(&wifi_reset_button_task, "wifi_reset_button", WIFI_RESET_BUTTON_TASK_STACK_SIZE, NULL, WIFI_RESET_BUTTON_TASK_PRIORITY, NULL, WIFI_RESET_BUTTON_TASK_CORE_ID);

	/* Install a falling-edge interrupt after the semaphore and task are ready. */
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

	gpio_isr_handler_add(WIFI_RESET_BUTTON, wifi_reset_button_isr_handler, NULL);
}

