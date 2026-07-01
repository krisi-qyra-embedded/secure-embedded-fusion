/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-01
 */

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>

#include <ccs811.h>
#include <i2cdev.h>

#include "ccs811_sensor.h"
#include "tasks_common.h"

#define CCS811_I2C_PORT    0

#if defined(CONFIG_CCS811_I2C_ADDRESS_0)
#define CCS811_I2C_ADDR    CCS811_I2C_ADDRESS_1   // 0x5A
#elif defined(CONFIG_CCS811_I2C_ADDRESS_1)
#define CCS811_I2C_ADDR    CCS811_I2C_ADDRESS_2   // 0x5B
#else
#define CCS811_I2C_ADDR    CCS811_I2C_ADDRESS_1   // fallback default 0x5A
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "ccs811_sensor";

/* Samples and logs the CCS811 gas measurements. */
static void ccs811_test(void *pvParameters)
{
    ccs811_dev_t dev;
    memset(&dev, 0, sizeof(ccs811_dev_t));

    ESP_LOGI(TAG, "CCS811 descriptor initialization");

    ESP_ERROR_CHECK(ccs811_init_desc(&dev,
                                     CCS811_I2C_ADDR,
                                     CCS811_I2C_PORT,
                                     CONFIG_CCS811_I2C_MASTER_SDA,
                                     CONFIG_CCS811_I2C_MASTER_SCL));

    ESP_LOGI(TAG, "CCS811 sensor initialization");

    ESP_ERROR_CHECK(ccs811_init(&dev));

    ESP_LOGI(TAG, "CCS811 setting measurement mode: 1 sample / second");

    /* Select one-second measurement mode before reading gas results. */
    ESP_ERROR_CHECK(ccs811_set_mode(&dev, CCS811_MODE_1S));

    ESP_LOGI(TAG, "CCS811 started successfully");
    ESP_LOGI(TAG, "Waiting for gas measurements...");

    while (1)
{
    uint16_t tvoc = 0;
    uint16_t eco2 = 0;
    uint8_t raw_current = 0;
    uint16_t raw_voltage_adc = 0;

    esp_err_t res = ccs811_get_results(&dev,
                                       &tvoc,
                                       &eco2,
                                       &raw_current,
                                       &raw_voltage_adc);

    if (res == ESP_OK)
    {
        float raw_voltage = ((float)raw_voltage_adc * 1.65f) / 1023.0f;

        ESP_LOGI(TAG,
                 "eCO2=%u ppm | TVOC=%u ppb | I=%u uA | ADC=%u | V=%.3f",
                 eco2,
                 tvoc,
                 raw_current,
                 raw_voltage_adc,
                 raw_voltage);
    }
    else if (res == CCS811_ERR_NO_NEW_DATA)
    {
        ESP_LOGI(TAG, "CCS811: no new data yet");
    }
    else
    {
        ESP_LOGE(TAG,
                 "CCS811 read failed: %d - %s",
                 res,
                 esp_err_to_name(res));
    }

    vTaskDelay(pdMS_TO_TICKS(1100));
}

}

/* Starts the CCS811 sampling task. */
void CCS811_task_start(void)
{
	
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("ccs811", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_ERROR_CHECK(i2cdev_init());

    BaseType_t task_status = xTaskCreatePinnedToCore(ccs811_test,
                                                     "ccs811_test",
                                                     CCS811_TASK_STACK_SIZE,
                                                     NULL,
                                                     CCS811_TASK_PRIORITY,
                                                     NULL,
                                                     CCS811_TASK_CORE_ID);

    /* Abort if the gas sampling task cannot be allocated. */
    if (task_status != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create CCS811 task");
        abort();
    }
}

