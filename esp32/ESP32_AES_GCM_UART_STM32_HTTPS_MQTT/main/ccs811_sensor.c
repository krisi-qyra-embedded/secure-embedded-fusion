/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>

#include <ccs811.h>
#include <i2cdev.h>

#include "app_watchdog.h"
#include "ccs811_sensor.h"
#include "tasks_common.h"

#define CCS811_I2C_PORT    0
#define CCS811_I2C_SDA     21
#define CCS811_I2C_SCL     22

#if defined(CONFIG_CCS811_I2C_ADDRESS_0)
#define CCS811_I2C_ADDR    CCS811_I2C_ADDRESS_1   /* 0x5A */
#elif defined(CONFIG_CCS811_I2C_ADDRESS_1)
#define CCS811_I2C_ADDR    CCS811_I2C_ADDRESS_2   /* 0x5B */
#else
#define CCS811_I2C_ADDR    CCS811_I2C_ADDRESS_1   /* fallback default 0x5A */
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#ifndef CCS811_MAX_CONSECUTIVE_FAILURES
#define CCS811_MAX_CONSECUTIVE_FAILURES    5U
#endif

#ifndef CCS811_MAX_INIT_RETRIES
#define CCS811_MAX_INIT_RETRIES            5U
#endif

#ifndef CCS811_RECOVERY_DELAY_MS
#define CCS811_RECOVERY_DELAY_MS           500U
#endif

#ifndef CCS811_SAMPLE_DELAY_MS
#define CCS811_SAMPLE_DELAY_MS             1100U
#endif

/*
 * Your log showed:
 *
 * Sensor resistance measurement has reached or exceeded the maximum range.
 * CCS811 read failed: 40970 - ERROR
 *
 * 40970 decimal = 0xA00A.
 *
 * This is a CCS811 internal gas-sensor range issue.
 * It is not the same as unplugged SDA/SCL/VCC.
 */
#define CCS811_ERR_SENSOR_RESISTANCE_MAX   ((esp_err_t)0xA00A)

static const char *TAG = "ccs811_sensor";

static ccs811_sensor_data_t g_ccs811_latest = {0};
static portMUX_TYPE g_ccs811_mux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t s_consecutive_severe_failures = 0;
static uint32_t s_soft_fault_count = 0;
static uint32_t s_no_new_data_count = 0;

/* Copies the cached CCS811 gas reading. */
bool CCS811_get_latest_reading(ccs811_sensor_data_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    /* Read the sample and validity flag under the same cache lock. */
    portENTER_CRITICAL(&g_ccs811_mux);
    *out = g_ccs811_latest;
    bool valid = g_ccs811_latest.valid;
    portEXIT_CRITICAL(&g_ccs811_mux);

    return valid;
}

/* Marks the cached gas reading unusable. */
static void ccs811_mark_latest_invalid(void)
{
    /* Change validity under the sensor cache lock. */
    portENTER_CRITICAL(&g_ccs811_mux);
    g_ccs811_latest.valid = false;
    portEXIT_CRITICAL(&g_ccs811_mux);
}

/* Identifies the tolerated CCS811 resistance-limit fault. */
static bool ccs811_is_soft_fault(esp_err_t err)
{
    /* This sensor fault invalidates a reading without requiring a reset. */
    return (err == CCS811_ERR_SENSOR_RESISTANCE_MAX);
}

/* Stops gas sensor heartbeats after recovery fails. */
static void ccs811_fatal_stop_kicking_watchdog(const char *reason)
{
    ESP_LOGE(TAG,
             "CCS811_FATAL | %s | stop kicking watchdog",
             reason);

    /* Invalidate the last sample before yielding without further heartbeats. */
    ccs811_mark_latest_invalid();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Creates the CCS811 I2C descriptor with bounded retries. */
static bool ccs811_init_desc_with_retries(ccs811_dev_t *dev)
{
    esp_err_t err = ESP_FAIL;

    for (uint32_t attempt = 1U; attempt <= CCS811_MAX_INIT_RETRIES; attempt++)
    {
        ESP_LOGI(TAG,
                 "CCS811_INIT | step=descriptor | attempt=%lu/%u",
                 (unsigned long)attempt,
                 CCS811_MAX_INIT_RETRIES);

        err = ccs811_init_desc(dev,
                               CCS811_I2C_ADDR,
                               CCS811_I2C_PORT,
                               CCS811_I2C_SDA,
                               CCS811_I2C_SCL);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "CCS811_INIT | step=descriptor | status=ok");
            return true;
        }

        ESP_LOGW(TAG,
                 "CCS811_INIT_RETRY | step=descriptor | err=%d - %s | attempt=%lu/%u",
                 err,
                 esp_err_to_name(err),
                 (unsigned long)attempt,
                 CCS811_MAX_INIT_RETRIES);

        /* Keep the sensor heartbeat current between initialization attempts. */
        app_watchdog_kick(APP_WD_CCS811);
        vTaskDelay(pdMS_TO_TICKS(CCS811_RECOVERY_DELAY_MS));
    }

    ESP_LOGE(TAG,
             "CCS811_INIT_FAIL | step=descriptor | final_err=%d - %s",
             err,
             esp_err_to_name(err));

    return false;
}

/* Retries CCS811 initialization and one-second measurement mode. */
static bool ccs811_sensor_init_and_mode_with_retries(ccs811_dev_t *dev,
                                                     const char *phase)
{
    esp_err_t err = ESP_FAIL;

    for (uint32_t attempt = 1U; attempt <= CCS811_MAX_INIT_RETRIES; attempt++)
    {
        ESP_LOGI(TAG,
                 "CCS811_%s | step=sensor_init | attempt=%lu/%u",
                 phase,
                 (unsigned long)attempt,
                 CCS811_MAX_INIT_RETRIES);

        err = ccs811_init(dev);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "CCS811_%s | step=sensor_init | status=ok",
                     phase);
            break;
        }

        ESP_LOGW(TAG,
                 "CCS811_%s_RETRY | step=sensor_init | err=%d - %s | attempt=%lu/%u",
                 phase,
                 err,
                 esp_err_to_name(err),
                 (unsigned long)attempt,
                 CCS811_MAX_INIT_RETRIES);

        app_watchdog_kick(APP_WD_CCS811);
        vTaskDelay(pdMS_TO_TICKS(CCS811_RECOVERY_DELAY_MS));
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "CCS811_%s_FAIL | step=sensor_init | final_err=%d - %s",
                 phase,
                 err,
                 esp_err_to_name(err));
        return false;
    }

    err = ESP_FAIL;

    for (uint32_t attempt = 1U; attempt <= CCS811_MAX_INIT_RETRIES; attempt++)
    {
        ESP_LOGI(TAG,
                 "CCS811_%s | step=set_mode | attempt=%lu/%u",
                 phase,
                 (unsigned long)attempt,
                 CCS811_MAX_INIT_RETRIES);

        /* Retry mode selection separately from sensor initialization. */
        err = ccs811_set_mode(dev, CCS811_MODE_1S);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "CCS811_%s | step=set_mode | status=ok",
                     phase);
            return true;
        }

        ESP_LOGW(TAG,
                 "CCS811_%s_RETRY | step=set_mode | err=%d - %s | attempt=%lu/%u",
                 phase,
                 err,
                 esp_err_to_name(err),
                 (unsigned long)attempt,
                 CCS811_MAX_INIT_RETRIES);

        app_watchdog_kick(APP_WD_CCS811);
        vTaskDelay(pdMS_TO_TICKS(CCS811_RECOVERY_DELAY_MS));
    }

    ESP_LOGE(TAG,
             "CCS811_%s_FAIL | step=set_mode | final_err=%d - %s",
             phase,
             err,
             esp_err_to_name(err));

    return false;
}

/* Reinitializes the CCS811 and clears its fault counters. */
static bool ccs811_try_recover(ccs811_dev_t *dev)
{
    ESP_LOGW(TAG, "CCS811_RECOVERY | trying to reinitialize sensor");

    /* Invalidate cached data until sensor initialization succeeds. */
    ccs811_mark_latest_invalid();

    bool recovered = ccs811_sensor_init_and_mode_with_retries(dev, "RECOVERY");

    if (!recovered)
    {
        ESP_LOGE(TAG, "CCS811_RECOVERY | recovery failed after retries");
        return false;
    }

    s_consecutive_severe_failures = 0;
    s_soft_fault_count = 0;
    s_no_new_data_count = 0;

    ESP_LOGW(TAG, "CCS811_RECOVERY | sensor recovered successfully");

    return true;
}

/* Samples gas readings and handles transient and severe sensor faults. */
static void ccs811_test(void *pvParameters)
{
    (void)pvParameters;

    ccs811_dev_t dev;
    memset(&dev, 0, sizeof(ccs811_dev_t));

    ESP_LOGI(TAG, "CCS811 descriptor initialization");

    if (!ccs811_init_desc_with_retries(&dev))
    {
        ccs811_fatal_stop_kicking_watchdog("descriptor initialization failed after retries");
    }

    ESP_LOGI(TAG, "CCS811 sensor initialization and measurement mode setup");

    if (!ccs811_sensor_init_and_mode_with_retries(&dev, "INIT"))
    {
        ccs811_fatal_stop_kicking_watchdog("sensor init/set_mode failed after retries");
    }

    ESP_LOGI(TAG, "CCS811 started successfully");
    ESP_LOGI(TAG, "Waiting for gas measurements...");

    app_watchdog_kick(APP_WD_CCS811);

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
            s_consecutive_severe_failures = 0;
            s_soft_fault_count = 0;
            s_no_new_data_count = 0;

            float raw_voltage = ((float)raw_voltage_adc * 1.65f) / 1023.0f;

            ccs811_sensor_data_t latest = {
                .valid = true,
                .eco2_ppm = eco2,
                .tvoc_ppb = tvoc,
                .raw_current_ua = raw_current,
                .raw_voltage_adc = raw_voltage_adc,
                .raw_voltage = raw_voltage
            };

            portENTER_CRITICAL(&g_ccs811_mux);
            g_ccs811_latest = latest;
            portEXIT_CRITICAL(&g_ccs811_mux);

            ESP_LOGI(TAG,
                     "eCO2=%u ppm | TVOC=%u ppb | I=%u uA | ADC=%u | V=%.3f",
                     eco2,
                     tvoc,
                     raw_current,
                     raw_voltage_adc,
                     raw_voltage);

            app_watchdog_kick(APP_WD_CCS811);
        }
        /* Missing fresh data keeps the task alive without counting as a severe fault. */
        else if (res == CCS811_ERR_NO_NEW_DATA)
        {
            s_consecutive_severe_failures = 0;
            s_no_new_data_count++;

            ESP_LOGI(TAG,
                     "CCS811: no new data yet | count=%lu",
                     (unsigned long)s_no_new_data_count);

            app_watchdog_kick(APP_WD_CCS811);
        }
        else if (ccs811_is_soft_fault(res))
        {
            s_consecutive_severe_failures = 0;
            s_soft_fault_count++;

            ccs811_mark_latest_invalid();

            ESP_LOGW(TAG,
                     "CCS811_SOFT_FAULT | sensor alive but gas reading invalid | err=%d - %s | soft_fault_count=%lu | action=no_reset",
                     res,
                     esp_err_to_name(res),
                     (unsigned long)s_soft_fault_count);

            app_watchdog_kick(APP_WD_CCS811);
        }
        else
        {
            s_consecutive_severe_failures++;

            ESP_LOGE(TAG,
                     "CCS811_SEVERE_READ_FAIL | err=%d - %s | consecutive_failures=%lu/%u",
                     res,
                     esp_err_to_name(res),
                     (unsigned long)s_consecutive_severe_failures,
                     CCS811_MAX_CONSECUTIVE_FAILURES);

            if (s_consecutive_severe_failures >= CCS811_MAX_CONSECUTIVE_FAILURES)
            {
                ESP_LOGE(TAG,
                         "CCS811_FATAL_CANDIDATE | too many consecutive severe failures | attempting recovery");

                bool recovered = ccs811_try_recover(&dev);

                if (recovered)
                {
                    app_watchdog_kick(APP_WD_CCS811);
                }
                else
                {
                    ccs811_fatal_stop_kicking_watchdog("recovery failed after consecutive severe failures");
                }
            }
            else
            {
                app_watchdog_kick(APP_WD_CCS811);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CCS811_SAMPLE_DELAY_MS));
    }
}

/* Starts the CCS811 sampling task. */
void CCS811_task_start(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("ccs811", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // ESP_ERROR_CHECK(i2cdev_init());

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
