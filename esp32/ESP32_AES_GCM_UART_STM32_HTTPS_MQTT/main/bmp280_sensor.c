/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_system.h>
#include <esp_log.h>
#include <esp_err.h>

#include <driver/i2c.h>

#include <bmp280.h>
#include <i2cdev.h>

#include "bmp280_sensor.h"
#include "tasks_common.h"
#include "app_watchdog.h"


#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define BMP280_SENSOR_I2C_PORT    I2C_NUM_0

#ifndef CONFIG_BMP280_I2C_MASTER_SDA
#define CONFIG_BMP280_I2C_MASTER_SDA    21
#endif

#ifndef CONFIG_BMP280_I2C_MASTER_SCL
#define CONFIG_BMP280_I2C_MASTER_SCL    22
#endif

#if defined(CONFIG_BMP280_I2C_ADDRESS_0)
#define BMP280_PRIMARY_I2C_ADDR      BMP280_I2C_ADDRESS_0
#define BMP280_SECONDARY_I2C_ADDR    BMP280_I2C_ADDRESS_1
#elif defined(CONFIG_BMP280_I2C_ADDRESS_1)
#define BMP280_PRIMARY_I2C_ADDR      BMP280_I2C_ADDRESS_1
#define BMP280_SECONDARY_I2C_ADDR    BMP280_I2C_ADDRESS_0
#else
#define BMP280_PRIMARY_I2C_ADDR      BMP280_I2C_ADDRESS_0   /* 0x76 */
#define BMP280_SECONDARY_I2C_ADDR    BMP280_I2C_ADDRESS_1   /* 0x77 */
#endif

#ifndef BMP280_TASK_STACK_SIZE
#define BMP280_TASK_STACK_SIZE    4096
#endif

#ifndef BMP280_TASK_PRIORITY
#define BMP280_TASK_PRIORITY      5
#endif

#ifndef BMP280_TASK_CORE_ID
#define BMP280_TASK_CORE_ID       APP_CPU_NUM
#endif

static const char *TAG = "bmp280_sensor";

#define BMP280_MAX_CONSECUTIVE_FAILURES   5U
#define BMP280_RECOVERY_DELAY_MS          500U

static bmp280_sensor_data_t g_bmp280_latest = {0};
static portMUX_TYPE g_bmp280_mux = portMUX_INITIALIZER_UNLOCKED;

/* Copies the cached BMP280 or BME280 sample. */
bool BMP280_get_latest_reading(bmp280_sensor_data_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    /* Hold the cache lock while copying the sample and its validity. */
    portENTER_CRITICAL(&g_bmp280_mux);
    *out = g_bmp280_latest;
    bool valid = g_bmp280_latest.valid;
    portEXIT_CRITICAL(&g_bmp280_mux);

    return valid;
}

/* Returns the shared BMP280/BME280 cached reading. */
bool BME280_get_latest_reading(bmp280_sensor_data_t *out)
{
    /* Use the same cache for both supported sensor IDs. */
    return BMP280_get_latest_reading(out);
}

/* Initializes the shared I2C helper layer. */
static esp_err_t bmp280_sensor_i2cdev_init_once(void)
{
    esp_err_t res = i2cdev_init();

    if (res == ESP_OK)
    {
        ESP_LOGI(TAG, "i2cdev initialized successfully");
        return ESP_OK;
    }

    /* Treat an already initialized bus helper as usable. */
    if (res == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "i2cdev already initialized, continuing");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "i2cdev initialization failed: %d - %s",
             res,
             esp_err_to_name(res));

    return res;
}

/* Initializes a BMP280 or BME280 at the supplied I2C address. */
static esp_err_t bmp280_sensor_try_init_at_address(bmp280_t *dev,
                                                   bmp280_params_t *params,
                                                   uint8_t address)
{
    memset(dev, 0, sizeof(bmp280_t));

    ESP_LOGI(TAG,
             "Trying BMP280/BME280 at I2C address 0x%02X | SDA=%d | SCL=%d | port=%d",
             address,
             CONFIG_BMP280_I2C_MASTER_SDA,
             CONFIG_BMP280_I2C_MASTER_SCL,
             BMP280_SENSOR_I2C_PORT);

    esp_err_t res = bmp280_init_desc(dev,
                                     address,
                                     BMP280_SENSOR_I2C_PORT,
                                     CONFIG_BMP280_I2C_MASTER_SDA,
                                     CONFIG_BMP280_I2C_MASTER_SCL);

    if (res != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "Descriptor init failed at address 0x%02X: %d - %s",
                 address,
                 res,
                 esp_err_to_name(res));
        return res;
    }

    res = bmp280_init(dev, params);

    if (res != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "Sensor init failed at address 0x%02X: %d - %s",
                 address,
                 res,
                 esp_err_to_name(res));

        /* Free the descriptor if sensor initialization fails. */
        bmp280_free_desc(dev);
        memset(dev, 0, sizeof(bmp280_t));
        return res;
    }

    ESP_LOGI(TAG,
             "Sensor initialized at address 0x%02X | chip_id=0x%02X",
             address,
             dev->id);

    return ESP_OK;
}

/* Invalidates the cached pressure sensor sample. */
static void bmp280_mark_latest_invalid(void)
{
    /* Protect the validity flag against concurrent readers. */
    portENTER_CRITICAL(&g_bmp280_mux);
    g_bmp280_latest.valid = false;
    portEXIT_CRITICAL(&g_bmp280_mux);
}

/* Stops pressure sensor heartbeats after an unrecoverable fault. */
static void bmp280_fatal_stop_kicking_watchdog(const char *reason)
{
    ESP_LOGE(TAG,
             "BME280_FATAL | %s | stop kicking watchdog",
             reason);

    /* Invalidate cached data before waiting for watchdog recovery. */
    bmp280_mark_latest_invalid();

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Recreates the sensor descriptor and retries both I2C addresses. */
static bool bmp280_try_recover(bmp280_t *dev, bmp280_params_t *params)
{
    ESP_LOGW(TAG, "BME280_RECOVERY | trying to reinitialize sensor");

    bmp280_mark_latest_invalid();

    /* Release the failed descriptor before probing the primary address again. */
    bmp280_free_desc(dev);
    memset(dev, 0, sizeof(bmp280_t));

    vTaskDelay(pdMS_TO_TICKS(BMP280_RECOVERY_DELAY_MS));

    esp_err_t res = bmp280_sensor_try_init_at_address(dev,
                                                      params,
                                                      BMP280_PRIMARY_I2C_ADDR);

    if (res != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "BME280_RECOVERY | primary address failed, trying secondary address 0x%02X",
                 BMP280_SECONDARY_I2C_ADDR);

        res = bmp280_sensor_try_init_at_address(dev,
                                                params,
                                                BMP280_SECONDARY_I2C_ADDR);
    }

    if (res != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "BME280_RECOVERY | recovery failed: %d - %s",
                 res,
                 esp_err_to_name(res));
        return false;
    }

    if ((dev->id != BME280_CHIP_ID) && (dev->id != BMP280_CHIP_ID))
    {
        ESP_LOGE(TAG,
                 "BME280_RECOVERY | unsupported chip ID after recovery: 0x%02X",
                 dev->id);

        bmp280_free_desc(dev);
        memset(dev, 0, sizeof(bmp280_t));
        return false;
    }

    ESP_LOGW(TAG,
             "BME280_RECOVERY | sensor recovered successfully | chip_id=0x%02X",
             dev->id);

    return true;
}

/* Samples the pressure sensor and maintains its cached reading. */
static void bmp280_test(void *pvParameters)
{
    (void)pvParameters;

    bmp280_t dev;
    bmp280_params_t params;

    ESP_LOGI(TAG, "BMP280/BME280 task started");

    //ESP_ERROR_CHECK(bmp280_sensor_i2cdev_init_once());

    ESP_ERROR_CHECK(bmp280_init_default_params(&params));

    /*
     * Normal mode = continuous measurement.
     * Standard oversampling is a good stable default.
     */
    params.mode = BMP280_MODE_NORMAL;
    params.filter = BMP280_FILTER_OFF;
    params.oversampling_pressure = BMP280_STANDARD;
    params.oversampling_temperature = BMP280_STANDARD;
    params.oversampling_humidity = BMP280_STANDARD;
    params.standby = BMP280_STANDBY_1000;

    ESP_LOGI(TAG, "BMP280/BME280 default parameters configured");

    esp_err_t res = bmp280_sensor_try_init_at_address(&dev,
                                                      &params,
                                                      BMP280_PRIMARY_I2C_ADDR);

    if (res != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "Primary address failed, trying secondary address 0x%02X",
                 BMP280_SECONDARY_I2C_ADDR);

        res = bmp280_sensor_try_init_at_address(&dev,
                                                &params,
                                                BMP280_SECONDARY_I2C_ADDR);
    }

    if (res != ESP_OK)
	{
    ESP_LOGE(TAG, "BMP280/BME280 sensor not found on 0x76 or 0x77");
    bmp280_fatal_stop_kicking_watchdog("sensor not found on 0x76 or 0x77");
	}

    if (dev.id == BME280_CHIP_ID)
    {
        ESP_LOGI(TAG,
                 "Detected BME280 | chip_id=0x%02X | measurements=temp,pressure,humidity",
                 dev.id);
    }
    else if (dev.id == BMP280_CHIP_ID)
    {
        ESP_LOGI(TAG,
                 "Detected BMP280 | chip_id=0x%02X | measurements=temp,pressure",
                 dev.id);
    }
    else
	{
    ESP_LOGE(TAG,
             "Unsupported chip ID after init: 0x%02X",
             dev.id);

    bmp280_free_desc(&dev);
    bmp280_fatal_stop_kicking_watchdog("unsupported chip ID after init");
	}

    ESP_LOGI(TAG, "Waiting for BMP280/BME280 measurements...");

	uint32_t consecutive_failures = 0;

while (1)
{
    float temperature_c = 0.0f;
    float pressure_pa = 0.0f;
    float humidity_percent = 0.0f;

    if (dev.id == BME280_CHIP_ID)
    {
        res = bmp280_read_float(&dev,
                                &temperature_c,
                                &pressure_pa,
                                &humidity_percent);
    }
    else
    {
        res = bmp280_read_float(&dev,
                                &temperature_c,
                                &pressure_pa,
                                NULL);
    }

    if (res == ESP_OK)
    {
        consecutive_failures = 0;

        float pressure_hpa = pressure_pa / 100.0f;

        bmp280_sensor_data_t latest = {
            .valid = true,
            .chip_id = dev.id,
            .humidity_available = (dev.id == BME280_CHIP_ID),
            .temperature_c = temperature_c,
            .pressure_pa = pressure_pa,
            .pressure_hpa = pressure_hpa,
            .humidity_percent = (dev.id == BME280_CHIP_ID) ? humidity_percent : 0.0f
        };

        portENTER_CRITICAL(&g_bmp280_mux);
        g_bmp280_latest = latest;
        portEXIT_CRITICAL(&g_bmp280_mux);

        if (dev.id == BME280_CHIP_ID)
        {
            ESP_LOGI(TAG,
                     "BME280 | Temp=%.2f C | Pressure=%.2f Pa | %.2f hPa | Humidity=%.2f %%",
                     temperature_c,
                     pressure_pa,
                     pressure_hpa,
                     humidity_percent);
        }
        else
        {
            ESP_LOGI(TAG,
                     "BMP280 | Temp=%.2f C | Pressure=%.2f Pa | %.2f hPa",
                     temperature_c,
                     pressure_pa,
                     pressure_hpa);
        }

        app_watchdog_kick(APP_WD_BME280);
    }
    else
    {
        consecutive_failures++;

        ESP_LOGE(TAG,
                 "BMP280/BME280 read failed: %d - %s | consecutive_failures=%lu/%u",
                 res,
                 esp_err_to_name(res),
                 (unsigned long)consecutive_failures,
                 BMP280_MAX_CONSECUTIVE_FAILURES);

        /* Try sensor recovery after the configured consecutive failure limit. */
        if (consecutive_failures >= BMP280_MAX_CONSECUTIVE_FAILURES)
        {
            ESP_LOGE(TAG,
                     "BME280_FATAL | too many consecutive failures | attempting recovery");

            bool recovered = bmp280_try_recover(&dev, &params);

            if (recovered)
            {
                consecutive_failures = 0;

                app_watchdog_kick(APP_WD_BME280);
            }
            else
            {
                bmp280_fatal_stop_kicking_watchdog("recovery failed after consecutive read failures");
            }
        }
        else
        {
            app_watchdog_kick(APP_WD_BME280);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
}
}

/* Starts the BMP280/BME280 sampling task. */
void BMP280_task_start(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("bmp280", ESP_LOG_DEBUG);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    BaseType_t task_status = xTaskCreatePinnedToCore(bmp280_test,
                                                     "bmp280_test",
                                                     BMP280_TASK_STACK_SIZE,
                                                     NULL,
                                                     BMP280_TASK_PRIORITY,
                                                     NULL,
                                                     BMP280_TASK_CORE_ID);

    /* Abort if the sampling task cannot be created. */
    if (task_status != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create BMP280/BME280 task");
        abort();
    }
}

/* Starts sampling through the common pressure sensor task. */
void BME280_task_start(void)
{
    /* Both chip variants share the same task entry point. */
    BMP280_task_start();
}