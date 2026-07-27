/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

#include "app_nvs.h"
#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "app_watchdog.h"
#include <esp_timer.h>

static const char TAG[] = "wifi_app";

static wifi_connected_event_callback_t wifi_connected_event_cb = NULL;
static wifi_app_softap_client_connected_cb_t g_softap_client_connected_cb = NULL;
static wifi_app_softap_last_client_disconnected_cb_t g_softap_last_client_disconnected_cb = NULL;

static volatile int g_softap_client_count = 0;

wifi_config_t *wifi_config = NULL;

static int g_retry_number = 0;

static EventGroupHandle_t wifi_app_event_group = NULL;
const int WIFI_APP_CONNECTING_USING_SAVED_CREDS_BIT   = BIT0;
const int WIFI_APP_CONNECTING_FROM_HTTP_SERVER_BIT    = BIT1;
const int WIFI_APP_USER_REQUESTED_STA_DISCONNECT_BIT  = BIT2;
const int WIFI_APP_STA_CONNECTED_GOT_IP_BIT           = BIT3;

static QueueHandle_t wifi_app_queue_handle = NULL;

#define WIFI_APP_MONITOR_PERIOD_MS               1000U
#define WIFI_APP_QUEUE_SEND_TIMEOUT_MS           1000U
#define WIFI_APP_MAX_CONSECUTIVE_API_FAILURES    5U

#define WIFI_WATCHDOG_FAULT_TEST        0
#define WIFI_WATCHDOG_FAULT_AFTER_MS    30000

static uint32_t s_wifi_consecutive_api_failures = 0;
static volatile bool s_wifi_fatal = false;

esp_netif_t *esp_netif_sta = NULL;
esp_netif_t *esp_netif_ap  = NULL;

#if WIFI_WATCHDOG_FAULT_TEST
/* Checks the delay for the optional Wi-Fi heartbeat stall. */
static bool wifi_watchdog_fault_test_expired(void)
{
    /* Start timing on the first Wi-Fi watchdog check. */
    static int64_t start_ms = 0;

    if (start_ms == 0)
    {
        start_ms = esp_timer_get_time() / 1000;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;

    return ((now_ms - start_ms) >= WIFI_WATCHDOG_FAULT_AFTER_MS);
}
#endif


/* Records progress for the Wi-Fi application. */
static void wifi_app_watchdog_kick(void)
{
#if WIFI_WATCHDOG_FAULT_TEST
    if (wifi_watchdog_fault_test_expired())
    {
        ESP_LOGE(TAG, "WIFI_WD_TEST | intentionally freezing Wi-Fi task");

        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#endif

    /* Refresh the Wi-Fi heartbeat during normal event and queue processing. */
    app_watchdog_kick(APP_WD_WIFI);
}

/* Clears the consecutive Wi-Fi API failure count. */
static void wifi_app_note_api_success(void)
{
    s_wifi_consecutive_api_failures = 0;
    /* A successful API call also refreshes the Wi-Fi heartbeat. */
    wifi_app_watchdog_kick();
}

/* Latches a fatal Wi-Fi fault and waits for the watchdog. */
static void wifi_app_fatal_stop_kicking_watchdog(const char *reason)
{
    ESP_LOGE(TAG,
             "WIFI_FATAL | reason=%s | stop_kicking_watchdog",
             reason);

    /* Yield without renewing the Wi-Fi heartbeat after a fatal error. */
    s_wifi_fatal = true;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Counts Wi-Fi API failures and escalates repeated errors. */
static void wifi_app_note_api_failure(esp_err_t err, const char *reason)
{
    s_wifi_consecutive_api_failures++;

    ESP_LOGE(TAG,
             "WIFI_API_FAIL | reason=%s | err=%d - %s | consecutive_failures=%lu/%u",
             reason,
             err,
             esp_err_to_name(err),
             (unsigned long)s_wifi_consecutive_api_failures,
             WIFI_APP_MAX_CONSECUTIVE_API_FAILURES);

    /* Enter the fatal wait when the consecutive failure limit is reached. */
    if (s_wifi_consecutive_api_failures >= WIFI_APP_MAX_CONSECUTIVE_API_FAILURES)
    {
        wifi_app_fatal_stop_kicking_watchdog(reason);
    }

    wifi_app_watchdog_kick();
}

/* Checks a Wi-Fi initialization result and stops on failure. */
static void wifi_app_check_critical(esp_err_t err, const char *reason)
{
    if (err == ESP_OK)
    {
        wifi_app_note_api_success();
        return;
    }

    ESP_LOGE(TAG,
             "WIFI_CRITICAL_FAIL | reason=%s | err=%d - %s",
             reason,
             err,
             esp_err_to_name(err));

    /* Critical setup errors bypass the retry counter. */
    wifi_app_fatal_stop_kicking_watchdog(reason);
}

/* Accepts successful or already initialized Wi-Fi setup. */
static void wifi_app_check_critical_allow_invalid_state(esp_err_t err,
                                                       const char *reason)
{
    /* Allow ESP_ERR_INVALID_STATE for resources that may already be ready. */
    if ((err == ESP_OK) || (err == ESP_ERR_INVALID_STATE))
    {
        wifi_app_note_api_success();
        return;
    }

    ESP_LOGE(TAG,
             "WIFI_CRITICAL_FAIL | reason=%s | err=%d - %s",
             reason,
             err,
             esp_err_to_name(err));

    wifi_app_fatal_stop_kicking_watchdog(reason);
}

/* Registers the callback for a new SoftAP client. */
void wifi_app_set_softap_client_connected_callback(wifi_app_softap_client_connected_cb_t cb)
{
    /* The Wi-Fi event handler invokes this callback on client association. */
    g_softap_client_connected_cb = cb;
}

/* Registers the callback for an idle SoftAP. */
void wifi_app_set_softap_last_client_disconnected_callback(wifi_app_softap_last_client_disconnected_cb_t cb)
{
    /* This callback runs only after the tracked client count reaches zero. */
    g_softap_last_client_disconnected_cb = cb;
}

/* Returns the tracked SoftAP client count. */
int wifi_app_get_softap_client_count(void)
{
    /* Service selection reads the count maintained by Wi-Fi events. */
    return g_softap_client_count;
}

/* Dispatches Wi-Fi and IP events to application control. */
static void wifi_app_event_handler(void *arg,
                                   esp_event_base_t event_base,
                                   int32_t event_id,
                                   void *event_data)
{
    (void)arg;

    wifi_app_watchdog_kick();

    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WIFI_EVT | interface=softap | event=start | status=ok");
                break;

            case WIFI_EVENT_AP_STOP:
                ESP_LOGW(TAG, "WIFI_EVT | interface=softap | event=stop | status=stopped");
                break;

            case WIFI_EVENT_AP_STACONNECTED:
            {
                g_softap_client_count++;

                ESP_LOGI(TAG,
                         "WIFI_EVT | interface=softap | event=client_connected | clients=%d",
                         g_softap_client_count);

                if (g_softap_client_connected_cb != NULL)
                {
                    g_softap_client_connected_cb();
                }

                break;
            }

            case WIFI_EVENT_AP_STADISCONNECTED:
            {
                if (g_softap_client_count > 0)
                {
                    g_softap_client_count--;
                }

                ESP_LOGI(TAG,
                         "WIFI_EVT | interface=softap | event=client_disconnected | clients=%d",
                         g_softap_client_count);

                if (g_softap_client_count == 0)
                {
                    if (g_softap_last_client_disconnected_cb != NULL)
                    {
                        g_softap_last_client_disconnected_cb();
                    }
                }

                break;
            }

            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WIFI_EVT | interface=station | event=start | status=ready");
                break;

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "WIFI_EVT | interface=station | event=connected_to_ap | status=link_up");
                break;

            case WIFI_EVENT_STA_DISCONNECTED:
            {
                wifi_event_sta_disconnected_t *wifi_event_sta_disconnected =
                    (wifi_event_sta_disconnected_t *)event_data;

                ESP_LOGW(TAG,
                         "WIFI_EVT | interface=station | event=disconnected | reason=%d | retry=%d/%d",
                         wifi_event_sta_disconnected->reason,
                         g_retry_number,
                         MAX_CONNECTION_RETRIES);

                /* Bound station reconnect attempts before notifying the Wi-Fi task. */
                if (g_retry_number < MAX_CONNECTION_RETRIES)
                {
                    ESP_LOGI(TAG,
                             "WIFI_STA | event=reconnect_attempt | next_retry=%d/%d",
                             g_retry_number + 1,
                             MAX_CONNECTION_RETRIES);

                    esp_err_t err = esp_wifi_connect();
                    if (err == ESP_OK)
                    {
                        g_retry_number++;
                        wifi_app_note_api_success();
                    }
                    else
                    {
                        wifi_app_note_api_failure(err, "esp_wifi_connect retry failed");
                    }
                }
                else
                {
                    ESP_LOGE(TAG,
                             "WIFI_STA | event=reconnect_exhausted | retries=%d | action=notify_wifi_task",
                             g_retry_number);

                    wifi_app_send_message(WIFI_APP_MSG_STA_DISCONNECTED);
                }

                break;
            }

            default:
                ESP_LOGD(TAG,
                         "WIFI_EVT | interface=unknown | event_id=%d | status=ignored",
                         (int)event_id);
                break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        switch (event_id)
        {
            case IP_EVENT_STA_GOT_IP:
                ESP_LOGI(TAG, "IP_EVT | interface=station | event=got_ip | status=network_ready");
                wifi_app_send_message(WIFI_APP_MSG_STA_CONNECTED_GOT_IP);
                break;

            default:
                ESP_LOGD(TAG,
                         "IP_EVT | interface=unknown | event_id=%d | status=ignored",
                         (int)event_id);
                break;
        }
    }
}

/* Registers the Wi-Fi and station-IP event handlers. */
static void wifi_app_event_handler_init(void)
{
    ESP_LOGI(TAG, "WIFI_INIT | step=event_loop_create | status=start");

    /* Reuse an existing default event loop when it is already initialized. */
    esp_err_t err = esp_event_loop_create_default();

    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE))
    {
        wifi_app_check_critical(err, "esp_event_loop_create_default failed");
    }
    else
    {
        wifi_app_note_api_success();
    }

    esp_event_handler_instance_t instance_wifi_event;
    esp_event_handler_instance_t instance_ip_event;

    wifi_app_check_critical(
        esp_event_handler_instance_register(WIFI_EVENT,
                                            ESP_EVENT_ANY_ID,
                                            &wifi_app_event_handler,
                                            NULL,
                                            &instance_wifi_event),
        "register WIFI_EVENT handler failed"
    );

    wifi_app_check_critical(
        esp_event_handler_instance_register(IP_EVENT,
                                            ESP_EVENT_ANY_ID,
                                            &wifi_app_event_handler,
                                            NULL,
                                            &instance_ip_event),
        "register IP_EVENT handler failed"
    );

    ESP_LOGI(TAG, "WIFI_INIT | step=event_handlers_registered | events=wifi,ip | status=ok");
}

/* Reads signal strength for the connected station. */
int8_t wifi_app_get_rssi(void)
{
    wifi_ap_record_t ap_info;

    /* Return zero when the station AP record is unavailable. */
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
        return ap_info.rssi;
    }

    return 0;
}

/* Initializes the Wi-Fi driver and AP/station network interfaces. */
static void wifi_app_default_wifi_init(void)
{
    ESP_LOGI(TAG, "WIFI_INIT | step=netif_init | status=start");

    esp_err_t err = esp_netif_init();

    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE))
    {
        wifi_app_check_critical(err, "esp_netif_init failed");
    }
    else
    {
        wifi_app_note_api_success();
    }

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();

    wifi_app_check_critical(esp_wifi_init(&wifi_init_config),
                            "esp_wifi_init failed");

    /* Keep runtime Wi-Fi configuration in RAM. */
    wifi_app_check_critical(esp_wifi_set_storage(WIFI_STORAGE_RAM),
                            "esp_wifi_set_storage failed");

    esp_netif_sta = esp_netif_create_default_wifi_sta();
    if (esp_netif_sta == NULL)
    {
        wifi_app_fatal_stop_kicking_watchdog("esp_netif_create_default_wifi_sta failed");
    }

    esp_netif_ap = esp_netif_create_default_wifi_ap();
    if (esp_netif_ap == NULL)
    {
        wifi_app_fatal_stop_kicking_watchdog("esp_netif_create_default_wifi_ap failed");
    }

    wifi_app_watchdog_kick();

    ESP_LOGI(TAG,
             "WIFI_INIT | step=wifi_driver_ready | storage=RAM | sta_netif=%p | ap_netif=%p | status=ok",
             esp_netif_sta,
             esp_netif_ap);
}

/**
 * Configures the WiFi access point settings and assigns the static IP to the SoftAP.
 */
/* Configures the SoftAP network and simultaneous station mode. */
static void wifi_app_soft_ap_config(void)
{
    ESP_LOGI(TAG,
             "SOFTAP_CFG | event=configure_start | ssid=%s | ip=%s | channel=%d | max_clients=%d",
             WIFI_AP_SSID,
             WIFI_AP_IP,
             WIFI_AP_CHANNEL,
             WIFI_AP_MAX_CONNECTIONS);

    wifi_config_t ap_config =
    {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .password = WIFI_AP_PASSWORD,
            .channel = WIFI_AP_CHANNEL,
            .ssid_hidden = WIFI_AP_SSID_HIDDEN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = WIFI_AP_MAX_CONNECTIONS,
            .beacon_interval = WIFI_AP_BEACON_INTERVAL,
        },
    };

    esp_netif_ip_info_t ap_ip_info;
    memset(&ap_ip_info, 0x00, sizeof(ap_ip_info));

    /* Stop DHCP before assigning the configured AP address. */
    esp_err_t err = esp_netif_dhcps_stop(esp_netif_ap);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG,
                 "SOFTAP_CFG | event=dhcp_stop_warning | err=%d - %s",
                 err,
                 esp_err_to_name(err));
    }

    if (inet_pton(AF_INET, WIFI_AP_IP, &ap_ip_info.ip) != 1)
    {
        wifi_app_fatal_stop_kicking_watchdog("invalid WIFI_AP_IP");
    }

    if (inet_pton(AF_INET, WIFI_AP_GATEWAY, &ap_ip_info.gw) != 1)
    {
        wifi_app_fatal_stop_kicking_watchdog("invalid WIFI_AP_GATEWAY");
    }

    if (inet_pton(AF_INET, WIFI_AP_NETMASK, &ap_ip_info.netmask) != 1)
    {
        wifi_app_fatal_stop_kicking_watchdog("invalid WIFI_AP_NETMASK");
    }

    wifi_app_check_critical(esp_netif_set_ip_info(esp_netif_ap, &ap_ip_info),
                            "esp_netif_set_ip_info failed");

	wifi_app_check_critical_allow_invalid_state(esp_netif_dhcps_start(esp_netif_ap),
                                            "esp_netif_dhcps_start failed");
                                            
    ESP_LOGI(TAG,
             "SOFTAP_CFG | event=dhcp_server_ready | ip=%s | gateway=%s | netmask=%s | status=ok",
             WIFI_AP_IP,
             WIFI_AP_GATEWAY,
             WIFI_AP_NETMASK);

    wifi_app_check_critical(esp_wifi_set_mode(WIFI_MODE_APSTA),
                            "esp_wifi_set_mode APSTA failed");

    wifi_app_check_critical(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config),
                            "esp_wifi_set_config AP failed");

esp_err_t bw_err = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_AP_BANDWIDTH);
if (bw_err != ESP_OK)
{
    ESP_LOGW(TAG,
             "SOFTAP_CFG | event=set_bandwidth_warning | err=%d - %s",
             bw_err,
             esp_err_to_name(bw_err));
}
else
{
    wifi_app_note_api_success();
}

   esp_err_t ps_err = esp_wifi_set_ps(WIFI_STA_POWER_SAVE);
if (ps_err != ESP_OK)
{
    ESP_LOGW(TAG,
             "WIFI_CFG | event=set_power_save_warning | err=%d - %s",
             ps_err,
             esp_err_to_name(ps_err));
}
else
{
    wifi_app_note_api_success();
}

    ESP_LOGI(TAG,
             "SOFTAP_CFG | event=wifi_mode_ready | mode=APSTA | ap_bandwidth=%d | power_save=%d | status=ok",
             WIFI_AP_BANDWIDTH,
             WIFI_STA_POWER_SAVE);
}

/* Applies station credentials and requests a connection. */
static void wifi_app_connect_sta(void)
{
    ESP_LOGI(TAG, "WIFI_STA | event=connect_request | source=current_sta_config | status=start");

    if (wifi_app_get_wifi_config() == NULL)
    {
        wifi_app_note_api_failure(ESP_ERR_INVALID_STATE, "wifi_config is NULL");
        return;
    }

    /* Set the station configuration before asking the driver to connect. */
    esp_err_t err = esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_app_get_wifi_config());
    if (err != ESP_OK)
    {
        wifi_app_note_api_failure(err, "esp_wifi_set_config STA failed");
        return;
    }

    err = esp_wifi_connect();
    if (err != ESP_OK)
    {
        wifi_app_note_api_failure(err, "esp_wifi_connect failed");
        return;
    }

    ESP_LOGI(TAG, "WIFI_STA | event=connect_request | status=submitted");

    wifi_app_note_api_success();
}

/* Processes Wi-Fi control messages and connection state changes. */
static void wifi_app_task(void *pvParameters)
{
    (void)pvParameters;

    wifi_app_queue_message_t msg;
    EventBits_t eventBits;

    ESP_LOGI(TAG, "WIFI_TASK | event=init_sequence_start | task=wifi_app_task");

    wifi_app_watchdog_kick();

    wifi_app_event_handler_init();
    wifi_app_default_wifi_init();
    wifi_app_soft_ap_config();

    ESP_LOGI(TAG, "WIFI_TASK | event=driver_start | status=start");

    wifi_app_check_critical(esp_wifi_start(), "esp_wifi_start failed");

    ESP_LOGI(TAG, "WIFI_TASK | event=driver_start | status=ok");

    wifi_app_send_message(WIFI_APP_MSG_LOAD_SAVED_CREDENTIALS);

    ESP_LOGI(TAG, "WIFI_TASK | event=initial_message_sent | msg=LOAD_SAVED_CREDENTIALS");

    for (;;)
    {
        if (xQueueReceive(wifi_app_queue_handle,
                          &msg,
                          pdMS_TO_TICKS(WIFI_APP_MONITOR_PERIOD_MS)) == pdTRUE)
        {
            wifi_app_watchdog_kick();

            switch (msg.msgID)
            {
                case WIFI_APP_MSG_LOAD_SAVED_CREDENTIALS:
                    ESP_LOGI(TAG, "WIFI_MSG | id=LOAD_SAVED_CREDENTIALS | action=load_nvs_sta_credentials");

                    if (app_nvs_load_sta_creds())
                    {
                        ESP_LOGI(TAG, "WIFI_NVS | event=sta_credentials_loaded | action=connect_using_saved_credentials");

                        g_retry_number = 0;
                        wifi_app_connect_sta();

                        /* Saved credentials are not rewritten after a successful reconnect. */
                        xEventGroupSetBits(wifi_app_event_group,
                                           WIFI_APP_CONNECTING_USING_SAVED_CREDS_BIT);

                        ESP_LOGI(TAG, "WIFI_STATE | bit=CONNECTING_USING_SAVED_CREDS | action=set");
                    }
                    else
                    {
                        ESP_LOGW(TAG, "WIFI_NVS | event=sta_credentials_missing | action=start_provisioning_web_server");
                    }

                    wifi_app_send_message(WIFI_APP_MSG_START_HTTP_SERVER);
                    break;

                case WIFI_APP_MSG_START_HTTP_SERVER:
                    ESP_LOGI(TAG, "WIFI_MSG | id=START_HTTP_SERVER | action=start_https_provisioning_server");
                    http_server_start();
                    break;

                case WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER:
                    ESP_LOGI(TAG, "WIFI_MSG | id=CONNECTING_FROM_HTTPS_SERVER | action=connect_station_from_user_credentials");

                    xEventGroupSetBits(wifi_app_event_group,
                                       WIFI_APP_CONNECTING_FROM_HTTP_SERVER_BIT);

                    ESP_LOGI(TAG, "WIFI_STATE | bit=CONNECTING_FROM_HTTP_SERVER | action=set");

                    g_retry_number = 0;

                    ESP_LOGI(TAG,
                             "WIFI_STA | event=retry_counter_reset | retry=0/%d",
                             MAX_CONNECTION_RETRIES);

                    wifi_app_connect_sta();

                    http_server_monitor_send_message(HTTP_MSG_WIFI_CONNECT_INIT);
                    break;

                case WIFI_APP_MSG_STA_CONNECTED_GOT_IP:
                    ESP_LOGI(TAG, "WIFI_MSG | id=STA_CONNECTED_GOT_IP | action=mark_station_network_ready");

                    xEventGroupSetBits(wifi_app_event_group,
                                       WIFI_APP_STA_CONNECTED_GOT_IP_BIT);

                    ESP_LOGI(TAG,
                             "WIFI_STATE | bit=STA_CONNECTED_GOT_IP | action=set | rssi=%d",
                             wifi_app_get_rssi());

                    http_server_monitor_send_message(HTTP_MSG_WIFI_CONNECT_SUCCESS);

                    eventBits = xEventGroupGetBits(wifi_app_event_group);

                    if (eventBits & WIFI_APP_CONNECTING_USING_SAVED_CREDS_BIT)
                    {
                        ESP_LOGI(TAG,
                                 "WIFI_NVS | event=connected_using_saved_credentials | action=do_not_rewrite_nvs");

                        xEventGroupClearBits(wifi_app_event_group,
                                             WIFI_APP_CONNECTING_USING_SAVED_CREDS_BIT);

                        ESP_LOGI(TAG, "WIFI_STATE | bit=CONNECTING_USING_SAVED_CREDS | action=clear");
                    }
                    else
                    {
                        ESP_LOGI(TAG,
                                 "WIFI_NVS | event=connected_from_user_credentials | action=save_sta_credentials");

                        app_nvs_save_sta_creds();
                    }

                    if (eventBits & WIFI_APP_CONNECTING_FROM_HTTP_SERVER_BIT)
                    {
                        xEventGroupClearBits(wifi_app_event_group,
                                             WIFI_APP_CONNECTING_FROM_HTTP_SERVER_BIT);

                        ESP_LOGI(TAG, "WIFI_STATE | bit=CONNECTING_FROM_HTTP_SERVER | action=clear");
                    }

                    if (wifi_connected_event_cb)
                    {
                        ESP_LOGI(TAG, "WIFI_CB | event=station_connected_callback | action=execute");
                        wifi_app_call_callback();
                    }
                    else
                    {
                        ESP_LOGD(TAG, "WIFI_CB | event=station_connected_callback | action=none_registered");
                    }

                    break;

                case WIFI_APP_MSG_USER_REQUESTED_STA_DISCONNECT:
                    ESP_LOGI(TAG, "WIFI_MSG | id=USER_REQUESTED_STA_DISCONNECT | action=disconnect_station");

                    eventBits = xEventGroupGetBits(wifi_app_event_group);

                    if (eventBits & WIFI_APP_STA_CONNECTED_GOT_IP_BIT)
                    {
                        xEventGroupSetBits(wifi_app_event_group,
                                           WIFI_APP_USER_REQUESTED_STA_DISCONNECT_BIT);

                        ESP_LOGI(TAG, "WIFI_STATE | bit=USER_REQUESTED_STA_DISCONNECT | action=set");

                        g_retry_number = MAX_CONNECTION_RETRIES;

                        ESP_LOGI(TAG,
                                 "WIFI_STA | event=user_disconnect | retry_counter=%d | action=disconnect_and_clear_saved_creds",
                                 g_retry_number);

                        esp_err_t err = esp_wifi_disconnect();
                        if (err == ESP_OK)
                        {
                            wifi_app_note_api_success();
                        }
                        else
                        {
                            wifi_app_note_api_failure(err, "esp_wifi_disconnect failed");
                        }

                        app_nvs_clear_sta_creds();
                    }
                    else
                    {
                        ESP_LOGW(TAG,
                                 "WIFI_STA | event=user_disconnect | status=ignored | reason=station_not_connected");
                    }

                    break;

                case WIFI_APP_MSG_STA_DISCONNECTED:
                    ESP_LOGW(TAG, "WIFI_MSG | id=STA_DISCONNECTED | action=classify_disconnect_source");

                    eventBits = xEventGroupGetBits(wifi_app_event_group);

                    if (eventBits & WIFI_APP_CONNECTING_USING_SAVED_CREDS_BIT)
                    {
                        ESP_LOGW(TAG,
                                 "WIFI_STA | event=disconnect_classified | source=saved_credentials | action=clear_invalid_saved_credentials");

                        xEventGroupClearBits(wifi_app_event_group,
                                             WIFI_APP_CONNECTING_USING_SAVED_CREDS_BIT);

                        app_nvs_clear_sta_creds();
                    }
                    else if (eventBits & WIFI_APP_CONNECTING_FROM_HTTP_SERVER_BIT)
                    {
                        ESP_LOGW(TAG,
                                 "WIFI_STA | event=disconnect_classified | source=https_provisioning | action=notify_http_connect_fail");

                        xEventGroupClearBits(wifi_app_event_group,
                                             WIFI_APP_CONNECTING_FROM_HTTP_SERVER_BIT);

                        http_server_monitor_send_message(HTTP_MSG_WIFI_CONNECT_FAIL);
                    }
                    else if (eventBits & WIFI_APP_USER_REQUESTED_STA_DISCONNECT_BIT)
                    {
                        ESP_LOGI(TAG,
                                 "WIFI_STA | event=disconnect_classified | source=user_request | action=notify_http_user_disconnect");

                        xEventGroupClearBits(wifi_app_event_group,
                                             WIFI_APP_USER_REQUESTED_STA_DISCONNECT_BIT);

                        http_server_monitor_send_message(HTTP_MSG_WIFI_USER_DISCONNECT);
                    }
                    else
                    {
                        ESP_LOGW(TAG,
                                 "WIFI_STA | event=disconnect_classified | source=unknown_or_link_loss | action=no_retry_policy_here");
                    }

                    if (eventBits & WIFI_APP_STA_CONNECTED_GOT_IP_BIT)
                    {
                        xEventGroupClearBits(wifi_app_event_group,
                                             WIFI_APP_STA_CONNECTED_GOT_IP_BIT);

                        ESP_LOGI(TAG, "WIFI_STATE | bit=STA_CONNECTED_GOT_IP | action=clear");
                    }

                    break;

                default:
                    ESP_LOGW(TAG,
                             "WIFI_MSG | id=%d | status=ignored | reason=unknown_message",
                             msg.msgID);
                    break;
            }
        }
        else
        {
            wifi_app_watchdog_kick();
        }
    }
}

/* Queues a control message for the Wi-Fi task. */
BaseType_t wifi_app_send_message(wifi_app_message_e msgID)
{
    if (wifi_app_queue_handle == NULL)
    {
        ESP_LOGE(TAG,
                 "WIFI_QUEUE | event=send_failed | reason=queue_not_ready | msgID=%d",
                 msgID);

        return pdFALSE;
    }

    wifi_app_queue_message_t msg;
    msg.msgID = msgID;

    /* Use a bounded queue wait so callers can detect a stalled Wi-Fi task. */
    BaseType_t result = xQueueSend(wifi_app_queue_handle,
                                   &msg,
                                   pdMS_TO_TICKS(WIFI_APP_QUEUE_SEND_TIMEOUT_MS));

    if (result == pdTRUE)
    {
        wifi_app_watchdog_kick();
        return pdTRUE;
    }

    ESP_LOGE(TAG,
             "WIFI_QUEUE | event=send_failed | reason=timeout_or_full | msgID=%d",
             msgID);

    wifi_app_note_api_failure(ESP_ERR_TIMEOUT, "wifi_app queue send failed");

    return pdFALSE;
}

/* Returns the shared station configuration. */
wifi_config_t *wifi_app_get_wifi_config(void)
{
    /* Credential loading and provisioning update this same configuration. */
    return wifi_config;
}

/* Registers the station connection callback. */
void wifi_app_set_callback(wifi_connected_event_callback_t cb)
{
    /* The Wi-Fi task calls it after the station obtains an IP address. */
    wifi_connected_event_cb = cb;
    ESP_LOGI(TAG, "WIFI_CB | event=station_connected_callback_registered | status=ok");
}

/* Invokes the registered station connection callback. */
void wifi_app_call_callback(void)
{
    /* Check the callback before dispatching the connection event. */
    if (wifi_connected_event_cb != NULL)
    {
        wifi_connected_event_cb();
    }
}

/* Allocates Wi-Fi control state and starts the application task. */
void wifi_app_start(void)
{
    ESP_LOGI(TAG, "WIFI_APP | event=start | task=wifi_app_task | mode=APSTA");

    s_wifi_fatal = false;
    s_wifi_consecutive_api_failures = 0;

    app_watchdog_set_enabled(APP_WD_WIFI, true);
    app_watchdog_kick(APP_WD_WIFI);

    /* rgb_led_wifi_app_started(); */

    esp_log_level_set("wifi", ESP_LOG_NONE);
    ESP_LOGI(TAG, "WIFI_APP | event=driver_log_filter | component=wifi | level=NONE");

    wifi_config = (wifi_config_t *)malloc(sizeof(wifi_config_t));
    if (wifi_config == NULL)
    {
        wifi_app_fatal_stop_kicking_watchdog("wifi_config malloc failed");
    }

    memset(wifi_config, 0x00, sizeof(wifi_config_t));

    ESP_LOGI(TAG,
             "WIFI_APP | event=sta_config_allocated | ptr=%p | size=%u",
             wifi_config,
             (unsigned int)sizeof(wifi_config_t));

    /* Create the queue and event group before launching Wi-Fi control. */
    wifi_app_queue_handle = xQueueCreate(3, sizeof(wifi_app_queue_message_t));
    if (wifi_app_queue_handle == NULL)
    {
        wifi_app_fatal_stop_kicking_watchdog("wifi_app_queue_create failed");
    }

    ESP_LOGI(TAG,
             "WIFI_APP | event=queue_created | depth=3 | item_size=%u | handle=%p",
             (unsigned int)sizeof(wifi_app_queue_message_t),
             wifi_app_queue_handle);

    wifi_app_event_group = xEventGroupCreate();
    if (wifi_app_event_group == NULL)
    {
        wifi_app_fatal_stop_kicking_watchdog("wifi_app_event_group_create failed");
    }

    ESP_LOGI(TAG,
             "WIFI_APP | event=event_group_created | handle=%p",
             wifi_app_event_group);

    BaseType_t task_status = xTaskCreatePinnedToCore(&wifi_app_task,
                                                     "wifi_app_task",
                                                     WIFI_APP_TASK_STACK_SIZE,
                                                     NULL,
                                                     WIFI_APP_TASK_PRIORITY,
                                                     NULL,
                                                     WIFI_APP_TASK_CORE_ID);

    if (task_status != pdPASS)
    {
        wifi_app_fatal_stop_kicking_watchdog("wifi_app_task_create failed");
    }

    ESP_LOGI(TAG,
             "WIFI_APP | event=task_created | name=wifi_app_task | core=%d | priority=%d | stack=%d",
             WIFI_APP_TASK_CORE_ID,
             WIFI_APP_TASK_PRIORITY,
             WIFI_APP_TASK_STACK_SIZE);
}
