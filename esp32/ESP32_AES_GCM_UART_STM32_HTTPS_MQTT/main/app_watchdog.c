/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "app_watchdog.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "app_watchdog";

#define APP_WD_TWDT_TIMEOUT_SECONDS       15
#define APP_WD_SUPERVISOR_PERIOD_MS       1000
#define APP_WD_HTTPS_MAX_AGE_MS 		  10000
#define APP_WD_CCS811_MAX_AGE_MS          8000
#define APP_WD_BME280_MAX_AGE_MS          8000
#define APP_WD_WIFI_MAX_AGE_MS            8000
#define APP_WD_AWS_MAX_AGE_MS             30000

typedef struct
{
    const char *name;
    uint32_t max_age_ms;
    volatile uint32_t last_kick_ms;
    volatile bool enabled;
} app_watchdog_entry_t;

static app_watchdog_entry_t s_wd[APP_WD_COUNT] =
{
    [APP_WD_CCS811] = { "ccs811", APP_WD_CCS811_MAX_AGE_MS, 0, true  },
    [APP_WD_BME280] = { "bme280", APP_WD_BME280_MAX_AGE_MS, 0, true  },
    [APP_WD_WIFI]   = { "wifi",   APP_WD_WIFI_MAX_AGE_MS,   0, true  },
    [APP_WD_AWS]    = { "aws",    APP_WD_AWS_MAX_AGE_MS,    0, false },
	[APP_WD_HTTPS]  = { "https", APP_WD_HTTPS_MAX_AGE_MS, 0, false },
};

static bool s_watchdog_initialized = false;

/* Records a heartbeat for a supervised task. */
void app_watchdog_kick(app_watchdog_task_id_t id)
{
    /* Ignore IDs outside the watchdog table. */
    if (id >= APP_WD_COUNT)
    {
        return;
    }

    s_wd[id].last_kick_ms = (uint32_t)esp_log_timestamp();
}

/* Enables or suspends supervision for a task. */
void app_watchdog_set_enabled(app_watchdog_task_id_t id, bool enabled)
{
    if (id >= APP_WD_COUNT)
    {
        return;
    }

    s_wd[id].enabled = enabled;

    /* Seed a fresh heartbeat when supervision is enabled. */
    if (enabled)
    {
        app_watchdog_kick(id);
        ESP_LOGI(TAG, "WATCHDOG | task=%s | enabled", s_wd[id].name);
    }
    else
    {
        ESP_LOGW(TAG, "WATCHDOG | task=%s | disabled", s_wd[id].name);
    }
}

/* Checks heartbeat ages for all enabled tasks. */
static bool app_watchdog_all_healthy(void)
{
    uint32_t now_ms = (uint32_t)esp_log_timestamp();

    for (int i = 0; i < APP_WD_COUNT; i++)
    {
        /* Disabled services do not participate in the health check. */
        if (!s_wd[i].enabled)
        {
            continue;
        }

        uint32_t age_ms = now_ms - s_wd[i].last_kick_ms;

        if (age_ms > s_wd[i].max_age_ms)
        {
            ESP_LOGE(TAG,
                     "WATCHDOG_FAULT | task=%s | age_ms=%lu | limit_ms=%lu",
                     s_wd[i].name,
                     (unsigned long)age_ms,
                     (unsigned long)s_wd[i].max_age_ms);

            return false;
        }
    }

    return true;
}

/* Feeds the task watchdog while supervised services remain healthy. */
static void app_watchdog_supervisor_task(void *param)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "WATCHDOG | supervisor subscribed to TWDT");

    while (1)
    {
        /* I leave TWDT unfed when a task heartbeat expires. */
        if (app_watchdog_all_healthy())
        {
            esp_task_wdt_reset();
        }
        else
        {
            ESP_LOGE(TAG, "WATCHDOG_FAULT | system unhealthy | not feeding TWDT");
        }

        vTaskDelay(pdMS_TO_TICKS(APP_WD_SUPERVISOR_PERIOD_MS));
    }
}

/* Initializes TWDT and seeds the task heartbeat table. */
void app_watchdog_init(void)
{
    if (s_watchdog_initialized)
    {
        return;
    }

    esp_task_wdt_config_t twdt_config =
    {
        .timeout_ms = APP_WD_TWDT_TIMEOUT_SECONDS * 1000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true,
    };

    esp_err_t err = esp_task_wdt_init(&twdt_config);

    /* Accept an already initialized TWDT without resetting its configuration. */
    if (err == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "WATCHDOG | TWDT already initialized");
    }
    else
    {
        ESP_ERROR_CHECK(err);
    }

    uint32_t now_ms = (uint32_t)esp_log_timestamp();

    for (int i = 0; i < APP_WD_COUNT; i++)
    {
        s_wd[i].last_kick_ms = now_ms;
    }

    s_watchdog_initialized = true;

    ESP_LOGI(TAG,
             "WATCHDOG | initialized | timeout=%d seconds",
             APP_WD_TWDT_TIMEOUT_SECONDS);
}

/* Starts the watchdog supervisor on core zero. */
void app_watchdog_task_start(void)
{
    /* Run supervision above the application task priorities. */
    xTaskCreatePinnedToCore(
        app_watchdog_supervisor_task,
        "wd_supervisor",
        4096,
        NULL,
        6,
        NULL,
        0
    );
}

