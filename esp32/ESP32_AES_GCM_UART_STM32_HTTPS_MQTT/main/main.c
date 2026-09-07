/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "STM32_uart.h"
#include <stdio.h>
#include <stdbool.h>
#include "http_server.h"
#include <unistd.h>
#include "aws_iot.h"
#include "sntp_time_sync.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "wifi_app.h"
#include "ccs811_sensor.h"
#include "bmp280_sensor.h"
#include "wifi_reset_button.h"
#include <i2cdev.h>
#include "app_watchdog.h"
#include "esp_log.h"
#include "mbedtls_ecdh.h"
#include "mbedtls_aes_gcm.h"
#include "bootloader_random.h"
#include <stdlib.h>
#include "esp_heap_caps.h"

static const char TAG[] = "main";
static const char *CRYPTO_TAG = "CRYPTO_BRINGUP";

typedef enum
{
    APP_MODE_UNKNOWN = 0,  
    APP_MODE_LOCAL_DASHBOARD,  
    APP_MODE_CLOUD  
} app_mode_e;

static app_mode_e g_app_mode = APP_MODE_UNKNOWN;
static bool g_sta_has_internet = false;
static bool g_sntp_started = false;
static TaskHandle_t g_cloud_debounce_task = NULL;

/* Start both crypto contexts from zeroed static storage. */
static esp32_aes_gcm_ctx_t g_aes_gcm_ctx;
static esp32_ecdh_ctx_t g_ecdh_ctx;

static void app_enter_local_dashboard_mode(void);
static void app_enter_cloud_mode(void);

/* Switches to cloud mode after the SoftAP stays idle. */
static void cloud_mode_debounce_task(void *param)
{
    (void)param;

    ESP_LOGW(TAG, "AP clients became 0, waiting before entering CLOUD mode");

    /* Wait five seconds before checking whether an AP client returned. */
    vTaskDelay(pdMS_TO_TICKS(5000));

    if (wifi_app_get_softap_client_count() == 0)
    {
        ESP_LOGW(TAG, "AP still has 0 clients after debounce, entering CLOUD mode");
        app_enter_cloud_mode();
    }
    else
    {
        ESP_LOGW(TAG, "AP client reconnected during debounce, staying in LOCAL mode");
    }

    g_cloud_debounce_task = NULL;
    vTaskDelete(NULL);
}

/* Waits briefly for the AWS task to shut down. */
static void wait_for_aws_to_stop(void)
{
    /* Poll the task handle for up to five seconds. */
    for (int i = 0; i < 50; i++)
    {
        if (!aws_iot_is_running())
        {
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGW(TAG, "AWS IoT did not stop within timeout");
}

/* Switches the application to the local HTTPS dashboard. */
static void app_enter_local_dashboard_mode(void)
{
    if (g_app_mode == APP_MODE_LOCAL_DASHBOARD)
        return;

    ESP_LOGW(TAG, "MODE_SWITCH | entering LOCAL DASHBOARD mode");

    /* Request AWS shutdown before starting the HTTPS server. */
    aws_iot_stop();

    wait_for_aws_to_stop();

    ESP_LOGI(
        TAG,
        "HEAP_BEFORE_HTTPS | internal_free=%u | largest_internal=%u | total_free=%u",
        (unsigned)heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_largest_free_block(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)esp_get_free_heap_size()
    );

    http_server_start();

    ESP_LOGI(
        TAG,
        "HEAP_AFTER_HTTPS_START | internal_free=%u | largest_internal=%u | total_free=%u",
        (unsigned)heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_largest_free_block(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)esp_get_free_heap_size()
    );

    g_app_mode = APP_MODE_LOCAL_DASHBOARD;

    ESP_LOGI(TAG,
             "MODE_ACTIVE | LOCAL | HTTPS dashboard active | AWS stopped");
}

/* Starts cloud services when the SoftAP is idle. */
static void app_enter_cloud_mode(void)
{
    if (g_app_mode == APP_MODE_CLOUD)
    {
        return;
    }

    ESP_LOGW(TAG, "MODE_SWITCH | entering CLOUD mode");

    /* Keep local mode active while an AP client is connected. */
    if (wifi_app_get_softap_client_count() > 0)
    {
        ESP_LOGW(TAG, "CLOUD mode blocked because AP client is still connected");
        app_enter_local_dashboard_mode();
        return;
    }

    http_server_stop();

    vTaskDelay(pdMS_TO_TICKS(1000));

    if (!g_sta_has_internet)
    {
        ESP_LOGW(TAG, "CLOUD mode blocked because STA has no internet yet");
        g_app_mode = APP_MODE_UNKNOWN;
        return;
    }

    if (!g_sntp_started)
    {
        sntp_time_sync_task_start();
        g_sntp_started = true;
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    aws_iot_start();

    g_app_mode = APP_MODE_CLOUD;

    ESP_LOGI(TAG, "MODE_ACTIVE | CLOUD | AWS active | HTTPS dashboard stopped");
}

/* Selects local mode when a SoftAP client connects. */
static void softap_client_connected_events(void)
{
    ESP_LOGW(TAG, "AP client connected to ESP32_AP");

    /* Cancel the pending cloud transition before serving the dashboard. */
    if (g_cloud_debounce_task != NULL)
    {
        vTaskDelete(g_cloud_debounce_task);
        g_cloud_debounce_task = NULL;
        ESP_LOGW(TAG, "Cancelled pending CLOUD mode debounce");
    }

    app_enter_local_dashboard_mode();
}

/* Schedules a cloud transition after the last AP client leaves. */
static void softap_last_client_disconnected_events(void)
{
    ESP_LOGW(TAG, "Last AP client disconnected from ESP32_AP");

    /* Create only one debounce task for the pending transition. */
    if (g_cloud_debounce_task == NULL)
    {
        xTaskCreatePinnedToCore(
            cloud_mode_debounce_task,
            "cloud_debounce",
            4096,
            NULL,
            4,
            &g_cloud_debounce_task,
            0);
    }
}

/* Selects the service mode after the station obtains an IP address. */
void wifi_application_connected_events(void)
{
    ESP_LOGI(TAG, "WiFi Application Connected to router");

    g_sta_has_internet = true;

    /* Use the AP client count to choose local or cloud services. */
    if (wifi_app_get_softap_client_count() == 0)
    {
        app_enter_cloud_mode();
    }
    else
    {
        app_enter_local_dashboard_mode();
    }
}

/* Prepares directional ECDH keys and the GCM transmit session. */
static bool esp32_stm32_duplex_crypto_bringup(void)
{
    esp32_ecdh_status_t ecdh_status;
    esp32_aes_gcm_status_t gcm_status;

    ecdh_status =
        esp32_ecdh_init(
            &g_ecdh_ctx);

    if (ecdh_status != ESP32_ECDH_SUCCESS)
    {
        ESP_LOGE(
            CRYPTO_TAG,
            "esp32_ecdh_init failed: status=%d",
            (int)ecdh_status);

        return false;
    }

    ecdh_status =
        esp32_ecdh_generate_keypairs(
            &g_ecdh_ctx);

    if (ecdh_status != ESP32_ECDH_SUCCESS)
    {
        ESP_LOGE(
            CRYPTO_TAG,
            "Full-duplex ECDH material import failed: status=%d",
            (int)ecdh_status);

        esp32_ecdh_free(
            &g_ecdh_ctx);

        return false;
    }

    ecdh_status =
        esp32_ecdh_compute_and_verify_shared_secrets(
            &g_ecdh_ctx);

    if (ecdh_status != ESP32_ECDH_SUCCESS)
    {
        ESP_LOGE(
            CRYPTO_TAG,
            "Full-duplex ECDH shared-secret computation failed: "
            "status=%d",
            (int)ecdh_status);

        esp32_ecdh_free(
            &g_ecdh_ctx);

        return false;
    }

    ecdh_status =
        esp32_ecdh_derive_and_verify_keys(
            &g_ecdh_ctx);

    if (ecdh_status != ESP32_ECDH_SUCCESS)
    {
        ESP_LOGE(
            CRYPTO_TAG,
            "Full-duplex AES/MAC key derivation failed: status=%d",
            (int)ecdh_status);

        esp32_ecdh_free(
            &g_ecdh_ctx);

        return false;
    }

    gcm_status =
        esp32_aes_gcm_init(
            &g_aes_gcm_ctx);

    if (gcm_status != ESP32_AES_GCM_SUCCESS)
    {
        ESP_LOGE(
            CRYPTO_TAG,
            "esp32_aes_gcm_init failed: status=%d",
            (int)gcm_status);

        esp32_ecdh_free(
            &g_ecdh_ctx);

        return false;
    }

    /* Initialize the TX nonce session only after both keys are derived. */
    gcm_status =
        esp32_gcm_nonce_session_init(
            &g_aes_gcm_ctx,
            &g_ecdh_ctx);

    if (gcm_status != ESP32_AES_GCM_SUCCESS)
    {
        ESP_LOGE(
            CRYPTO_TAG,
            "ESP32 transmit nonce-session initialization failed: "
            "status=%d",
            (int)gcm_status);

        esp32_aes_gcm_free(
            &g_aes_gcm_ctx);

        esp32_ecdh_free(
            &g_ecdh_ctx);

        return false;
    }

    ESP_LOGI(
        CRYPTO_TAG,
        "Full-duplex STM32/ESP32 crypto initialized");

    ESP_LOGI(
        CRYPTO_TAG,
        "RX request wire size=42 bytes; "
        "TX encrypted ACK/NACK wire size=39 bytes");

    return true;
}

/* Starts encrypted UART, sensor tasks and local/cloud services. */
void app_main(void)
{
    esp_err_t ret;
    bool crypto_ready;
    esp32_aes_gcm_status_t gcm_status;

    ret =
    nvs_flash_init();

	/* Preserve persistent GCM state by refusing to erase NVS on initialization failure. */
	if (ret != ESP_OK)
	{
    	ESP_LOGE(
        "NVS_SECURITY",
        "NVS initialization failed: %s. "
        "Persistent AES-GCM state was not erased. "
        "Refusing to start encrypted communication.",
        esp_err_to_name(ret));

       abort();
	 }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(
        TAG,
        "RAM_INFO | "
        "internal_total=%u | "
        "internal_free=%u | "
        "internal_largest=%u | "
        "8bit_total=%u | "
        "8bit_free=%u | "
        "spiram_total=%u | "
        "spiram_free=%u",
        (unsigned)heap_caps_get_total_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_largest_free_block(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),

        (unsigned)heap_caps_get_total_size(
            MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_free_size(
            MALLOC_CAP_8BIT),

        (unsigned)heap_caps_get_total_size(
            MALLOC_CAP_SPIRAM),
        (unsigned)heap_caps_get_free_size(
            MALLOC_CAP_SPIRAM)
    );

    app_watchdog_init();
    app_watchdog_task_start();

    if (STM32_UART_LinkTest_Start() !=
        STM32_UART_STATUS_OK)
    {
        ESP_LOGE(
            "APP",
            "STM32 encrypted UART receiver failed to start");

        return;
    }

    ESP_LOGI(
        CRYPTO_TAG,
        "STM32 UART receiver is running first; expecting 42-byte encrypted requests");

    ESP_LOGI(
        CRYPTO_TAG,
        "Enabling temporary ESP32 hardware entropy source");

    bootloader_random_enable();

    crypto_ready =
        esp32_stm32_duplex_crypto_bringup();

    bootloader_random_disable();

    ESP_LOGI(
        CRYPTO_TAG,
        "Temporary ESP32 hardware entropy source disabled");

    if (!crypto_ready)
    {
        ESP_LOGE(
            CRYPTO_TAG,
            "Full-duplex crypto initialization failed");

        return;
    }

    gcm_status =
        esp32_aes_gcm_runtime_task_start(
            &g_aes_gcm_ctx,
            &g_ecdh_ctx);

    if (gcm_status != ESP32_AES_GCM_SUCCESS)
    {
        ESP_LOGE(
            CRYPTO_TAG,
            "Failed to start UART AES-GCM full-duplex task: status=%d",
            (int)gcm_status);

        esp32_aes_gcm_free(
            &g_aes_gcm_ctx);

        esp32_ecdh_free(
            &g_ecdh_ctx);

        return;
    }

    ESP_LOGI(
        CRYPTO_TAG,
        "ESP32 ready: RX request=42 bytes, TX encrypted ACK/NACK=39 bytes");

#if 1


    ESP_ERROR_CHECK(i2cdev_init());

    CCS811_task_start();

    vTaskDelay(pdMS_TO_TICKS(500));

    BMP280_task_start();

    wifi_app_set_callback(
        &wifi_application_connected_events);

    wifi_app_set_softap_client_connected_callback(
        &softap_client_connected_events);

    wifi_app_set_softap_last_client_disconnected_callback(
        &softap_last_client_disconnected_events);

    wifi_app_start();

    wifi_reset_button_config();
#endif
}
