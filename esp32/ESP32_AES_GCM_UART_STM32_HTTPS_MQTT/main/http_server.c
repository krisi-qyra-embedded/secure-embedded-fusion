/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_https_server.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "sys/param.h"

#include "sntp_time_sync.h"
#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "ccs811_sensor.h"
#include "bmp280_sensor.h"
#include "app_watchdog.h"

static const char TAG[] = "http_server";

static int g_wifi_connect_status = NONE;

static int g_fw_update_status = OTA_UPDATE_PENDING;

static bool g_is_local_time_set = false;

static httpd_handle_t http_server_handle = NULL;

static TaskHandle_t task_http_server_monitor = NULL;

static QueueHandle_t http_server_monitor_queue_handle;

#define HTTP_SERVER_MONITOR_PERIOD_MS    1000U
#define HTTPS_WATCHDOG_FAULT_TEST        0
#define HTTPS_WATCHDOG_FAULT_AFTER_MS    30000
#define HTTPS_MAX_CONSECUTIVE_FAILURES   3U


static uint32_t s_https_consecutive_failures = 0;
static volatile bool s_https_fatal = false;

#if HTTPS_WATCHDOG_FAULT_TEST
/* Checks the delay for the optional HTTPS heartbeat stall. */
static bool https_watchdog_fault_test_expired(void)
{
    /* Track elapsed time from the first HTTPS watchdog check. */
    static int64_t start_ms = 0;

    if (start_ms == 0)
    {
        start_ms = esp_timer_get_time() / 1000;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;

    return ((now_ms - start_ms) >= HTTPS_WATCHDOG_FAULT_AFTER_MS);
}
#endif


/* Refreshes supervision for a healthy active HTTPS server. */
static void http_server_watchdog_kick(void)
{
    /* Do not feed HTTPS supervision after a fatal fault or server shutdown. */
    if (!s_https_fatal && (http_server_handle != NULL))
    {
#if HTTPS_WATCHDOG_FAULT_TEST
        if (https_watchdog_fault_test_expired())
        {
            ESP_LOGE(TAG, "HTTPS_WD_TEST | intentionally freezing HTTPS watchdog kick");

            while (1)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
#endif

        app_watchdog_kick(APP_WD_HTTPS);
    }
}

/* Stops HTTPS heartbeats after a fatal server error. */
static void http_server_fatal_stop_kicking_watchdog(const char *reason)
{
    ESP_LOGE(TAG,
             "HTTPS_FATAL | %s | stop kicking watchdog",
             reason);

    /* Latch the failure before yielding without further heartbeat updates. */
    s_https_fatal = true;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Sends a response and tracks consecutive HTTPS failures. */
static esp_err_t http_server_send_response_checked(httpd_req_t *req,
                                                   const char *data,
                                                   size_t data_len)
{
    esp_err_t err = httpd_resp_send(req, data, data_len);

    if (err == ESP_OK)
    {
        s_https_consecutive_failures = 0;
        http_server_watchdog_kick();
        return ESP_OK;
    }

    s_https_consecutive_failures++;

    ESP_LOGE(TAG,
             "HTTPS_RESP_FAIL | err=%d - %s | consecutive_failures=%lu/%u",
             err,
             esp_err_to_name(err),
             (unsigned long)s_https_consecutive_failures,
             HTTPS_MAX_CONSECUTIVE_FAILURES);

    /* Escalate repeated send failures to watchdog recovery. */
    if (s_https_consecutive_failures >= HTTPS_MAX_CONSECUTIVE_FAILURES)
    {
        http_server_fatal_stop_kicking_watchdog("too many HTTPS response failures");
    }

    return err;
}

const esp_timer_create_args_t fw_update_reset_args = {
		.callback = &http_server_fw_update_reset_callback,
		.arg = NULL,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "fw_update_reset"
};
esp_timer_handle_t fw_update_reset;

extern const uint8_t jquery_3_3_1_min_js_start[]	asm("_binary_jquery_3_3_1_min_js_start");
extern const uint8_t jquery_3_3_1_min_js_end[]		asm("_binary_jquery_3_3_1_min_js_end");
extern const uint8_t index_html_start[]				asm("_binary_index_html_start");
extern const uint8_t index_html_end[]				asm("_binary_index_html_end");
extern const uint8_t app_css_start[]				asm("_binary_app_css_start");
extern const uint8_t app_css_end[]					asm("_binary_app_css_end");
extern const uint8_t app_js_start[]					asm("_binary_app_js_start");
extern const uint8_t app_js_end[]					asm("_binary_app_js_end");
extern const uint8_t favicon_ico_start[]			asm("_binary_favicon_ico_start");
extern const uint8_t favicon_ico_end[]				asm("_binary_favicon_ico_end");
extern const uint8_t servercert_pem_start[] 		asm("_binary_servercert_pem_start");
extern const uint8_t servercert_pem_end[]   		asm("_binary_servercert_pem_end");
extern const uint8_t prvtkey_pem_start[] 			asm("_binary_prvtkey_pem_start");
extern const uint8_t prvtkey_pem_end[]   			asm("_binary_prvtkey_pem_end");

/* Schedules a delayed restart after a successful OTA update. */
static void http_server_fw_update_reset_timer(void)
{
	/* Arm the eight-second timer only after OTA reports success. */
	if (g_fw_update_status == OTA_UPDATE_SUCCESSFUL)
	{
		ESP_LOGI(TAG, "OTA_RESET_TIMER | state=armed | reason=ota_success | delay_ms=8000");

		ESP_ERROR_CHECK(esp_timer_create(&fw_update_reset_args, &fw_update_reset));
		ESP_ERROR_CHECK(esp_timer_start_once(fw_update_reset, 8000000));
	}
	else
	{
		ESP_LOGW(TAG, "OTA_RESET_TIMER | state=skipped | reason=ota_not_successful | ota_status=%d", g_fw_update_status);
	}
}

/* Updates dashboard status from queued service events. */
static void http_server_monitor(void *parameter)
{
    (void)parameter;
    http_server_queue_message_t msg;

    for (;;)
    {
        if (xQueueReceive(http_server_monitor_queue_handle,
                          &msg,
                          pdMS_TO_TICKS(HTTP_SERVER_MONITOR_PERIOD_MS)) == pdTRUE)
        {
            switch (msg.msgID)
            {
                case HTTP_MSG_WIFI_CONNECT_INIT:
                    ESP_LOGI(TAG, "HTTP_MONITOR | event=WIFI_CONNECT_INIT | next_status=CONNECTING");
                    g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECTING;
                    break;

                case HTTP_MSG_WIFI_CONNECT_SUCCESS:
                    ESP_LOGI(TAG, "HTTP_MONITOR | event=WIFI_CONNECT_SUCCESS | next_status=CONNECTED");
                    g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECT_SUCCESS;
                    break;

                case HTTP_MSG_WIFI_CONNECT_FAIL:
                    ESP_LOGW(TAG, "HTTP_MONITOR | event=WIFI_CONNECT_FAIL | next_status=FAILED");
                    g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECT_FAILED;
                    break;

                case HTTP_MSG_WIFI_USER_DISCONNECT:
                    ESP_LOGI(TAG, "HTTP_MONITOR | event=WIFI_USER_DISCONNECT | next_status=DISCONNECTED");
                    g_wifi_connect_status = HTTP_WIFI_STATUS_DISCONNECTED;
                    break;

                /* Schedule the reboot only after a successful OTA completion message. */
                case HTTP_MSG_OTA_UPDATE_SUCCESSFUL:
                    ESP_LOGI(TAG, "HTTP_MONITOR | event=OTA_UPDATE_SUCCESSFUL | next_status=OTA_SUCCESS | action=schedule_reboot");
                    g_fw_update_status = OTA_UPDATE_SUCCESSFUL;
                    http_server_fw_update_reset_timer();
                    break;

                case HTTP_MSG_OTA_UPDATE_FAILED:
                    ESP_LOGE(TAG, "HTTP_MONITOR | event=OTA_UPDATE_FAILED | next_status=OTA_FAILED");
                    g_fw_update_status = OTA_UPDATE_FAILED;
                    break;

                case HTTP_MSG_TIME_SERVICE_INITIALIZED:
                    ESP_LOGI(TAG, "HTTP_MONITOR | event=TIME_SERVICE_INITIALIZED | local_time_state=READY");
                    g_is_local_time_set = true;
                    break;

                default:
                    ESP_LOGW(TAG, "HTTP_MONITOR | event=unknown_msg | msgID=%d", msg.msgID);
                    break;
            }
        }

        http_server_watchdog_kick();
    }
}

/* Serves the embedded jQuery script. */
static esp_err_t http_server_jquery_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/jquery-3.3.1.min.js | type=static_asset | mime=application/javascript");

    httpd_resp_set_type(req, "application/javascript");

    /* The linker symbol range defines the exact script length. */
    return http_server_send_response_checked(
        req,
        (const char *)jquery_3_3_1_min_js_start,
        jquery_3_3_1_min_js_end - jquery_3_3_1_min_js_start
    );
}

/* Returns the embedded dashboard page. */
static esp_err_t http_server_index_html_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/ | type=static_asset | mime=text/html");

    httpd_resp_set_type(req, "text/html");

    /* Send the HTML bytes embedded by the component build. */
    return http_server_send_response_checked(
        req,
        (const char *)index_html_start,
        index_html_end - index_html_start
    );
}

/* Serves the dashboard stylesheet. */
static esp_err_t http_server_app_css_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/app.css | type=static_asset | mime=text/css");

    httpd_resp_set_type(req, "text/css");

    /* Use the CSS linker boundaries rather than a string terminator. */
    return http_server_send_response_checked(
        req,
        (const char *)app_css_start,
        app_css_end - app_css_start
    );
}

/* Returns the embedded dashboard JavaScript. */
static esp_err_t http_server_app_js_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/app.js | type=static_asset | mime=application/javascript");

    httpd_resp_set_type(req, "application/javascript");

    /* The embedded script is sent with its linker-provided byte count. */
    return http_server_send_response_checked(
        req,
        (const char *)app_js_start,
        app_js_end - app_js_start
    );
}

/* Serves the embedded browser icon. */
static esp_err_t http_server_favicon_ico_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/favicon.ico | type=static_asset | mime=image/x-icon");

    httpd_resp_set_type(req, "image/x-icon");

    /* The icon is binary data, so its length comes from the linker symbols. */
    return http_server_send_response_checked(
        req,
        (const char *)favicon_ico_start,
        favicon_ico_end - favicon_ico_start
    );
}

/* Writes an uploaded firmware image to the next OTA partition. */
esp_err_t http_server_OTA_update_handler(httpd_req_t *req)
{
	esp_ota_handle_t ota_handle;

	char ota_buff[1024];
	int content_length = req->content_len;
	int content_received = 0;
	int recv_len;
	bool is_req_body_started = false;
	bool flash_successful = false;

	const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

	ESP_LOGI(TAG, "OTA_UPLOAD | event=start | content_length=%d | target_partition_ptr=%p",
			 content_length, (void *)update_partition);

	http_server_watchdog_kick();

	do
	{
		if ((recv_len = httpd_req_recv(req, ota_buff, MIN(content_length, sizeof(ota_buff)))) < 0)
		{
			/* Retry socket timeouts while receiving the upload stream. */
			if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
			{
				ESP_LOGW(TAG, "OTA_UPLOAD | event=recv_timeout | action=retry | received_bytes=%d | expected_bytes=%d", content_received, content_length);
				continue;  
			}
			ESP_LOGE(TAG, "OTA_UPLOAD | event=recv_failed | error_code=%d | received_bytes=%d | expected_bytes=%d", recv_len, content_received, content_length);
			return ESP_FAIL;
		}
		ESP_LOGD(TAG, "OTA_UPLOAD | event=chunk_received | received_bytes=%d | expected_bytes=%d", content_received, content_length);

		if (!is_req_body_started)
		{
			is_req_body_started = true;

			char *body_start_p = strstr(ota_buff, "\r\n\r\n");
			if (body_start_p == NULL)
			{
				ESP_LOGE(TAG, "OTA_UPLOAD | event=parse_failed | reason=missing_multipart_body_separator");
				return ESP_FAIL;
			}
			body_start_p += 4;
			int body_part_len = recv_len - (body_start_p - ota_buff);

			ESP_LOGI(TAG, "OTA_UPLOAD | event=body_detected | total_request_bytes=%d | first_body_chunk_bytes=%d", content_length, body_part_len);

			esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
			if (err != ESP_OK)
			{
				ESP_LOGE(TAG, "OTA_FLASH | event=begin_failed | error=%s | partition_ptr=%p", esp_err_to_name(err), (void *)update_partition);
				return ESP_FAIL;
			}
			else
			{
				ESP_LOGI(TAG, "OTA_FLASH | event=begin_success | partition_subtype=%d | partition_addr=0x%08lx", update_partition->subtype, update_partition->address);
			}

			esp_ota_write(ota_handle, body_start_p, body_part_len);
			content_received += body_part_len;
			http_server_watchdog_kick();
		}
		else
		{
			esp_ota_write(ota_handle, ota_buff, recv_len);
			content_received += recv_len;
			http_server_watchdog_kick();
		}

	} while (recv_len > 0 && content_received < content_length);

	if (esp_ota_end(ota_handle) == ESP_OK)
	{
		if (esp_ota_set_boot_partition(update_partition) == ESP_OK)
		{
			const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
			ESP_LOGI(TAG, "OTA_FLASH | event=boot_partition_updated | next_partition_subtype=%d | next_partition_addr=0x%08lx", boot_partition->subtype, boot_partition->address);
			flash_successful = true;
		}
		else
		{
			ESP_LOGE(TAG, "OTA_FLASH | event=set_boot_partition_failed | partition_subtype=%d | partition_addr=0x%08lx", update_partition->subtype, update_partition->address);
		}
	}
	else
	{
		ESP_LOGE(TAG, "OTA_FLASH | event=end_failed | received_bytes=%d | expected_bytes=%d", content_received, content_length);
	}

	if (flash_successful)
	{
		http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_SUCCESSFUL);
	}
	else
	{
		http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_FAILED);
	}

	return ESP_OK;
}

/* Returns the OTA result and firmware compile metadata. */
esp_err_t http_server_OTA_status_handler(httpd_req_t *req)
{
	char otaJSON[100];

	ESP_LOGI(TAG, "HTTPS_REQ | method=POST | uri=/OTAstatus | type=json_status | ota_status=%d", g_fw_update_status);
	
	http_server_watchdog_kick();
	
	/* Keep the original compiler-provided build date and time in the response. */
	sprintf(otaJSON, "{\"ota_update_status\":%d,\"compile_time\":\"%s\",\"compile_date\":\"%s\"}", g_fw_update_status, __TIME__, __DATE__);

	httpd_resp_set_type(req, "application/json");

	return http_server_send_response_checked(req, 
									otaJSON, 
									strlen(otaJSON));
}

/* Accepts station credentials and requests Wi-Fi connection. */
static esp_err_t http_server_wifi_connect_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "HTTPS_REQ | method=POST | uri=/wifiConnect.json | type=wifi_credentials");

	http_server_watchdog_kick();

	size_t len_ssid = 0, len_pass = 0;
	char *ssid_str = NULL, *pass_str = NULL;

	/* Read credentials from the request headers before updating station state. */
	len_ssid = httpd_req_get_hdr_value_len(req, "my-connect-ssid") + 1;
	if (len_ssid > 1)
	{
		ssid_str = malloc(len_ssid);
		if (ssid_str == NULL)
		{
			return ESP_ERR_NO_MEM;
		}

		if (httpd_req_get_hdr_value_str(req, "my-connect-ssid", ssid_str, len_ssid) == ESP_OK)
		{
			ESP_LOGI(TAG, "WIFI_PROVISIONING | event=ssid_received | ssid=%s | ssid_len=%u", ssid_str, (unsigned)(len_ssid - 1));
		}
	}

	len_pass = httpd_req_get_hdr_value_len(req, "my-connect-pwd") + 1;
	if (len_pass > 1)
	{
		pass_str = malloc(len_pass);
		if (pass_str == NULL)
		{
			free(ssid_str);
			return ESP_ERR_NO_MEM;
		}

		if (httpd_req_get_hdr_value_str(req, "my-connect-pwd", pass_str, len_pass) == ESP_OK)
		{
			ESP_LOGI(TAG, "WIFI_PROVISIONING | event=password_received | password_len=%u | value=REDACTED", (unsigned)(len_pass - 1));
		}
	}

	if ((ssid_str == NULL) || (pass_str == NULL))
	{
		ESP_LOGW(TAG, "WIFI_PROVISIONING | event=credentials_missing | ssid_present=%d | password_present=%d",
				 ssid_str != NULL,
				 pass_str != NULL);
		free(ssid_str);
		free(pass_str);
		return ESP_FAIL;
	}

	wifi_config_t* wifi_config = wifi_app_get_wifi_config();
	memset(wifi_config, 0x00, sizeof(wifi_config_t));
	memcpy(wifi_config->sta.ssid, ssid_str, len_ssid);
	memcpy(wifi_config->sta.password, pass_str, len_pass);
	wifi_app_send_message(WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER);
	ESP_LOGI(TAG, "WIFI_PROVISIONING | event=connect_request_forwarded | destination=wifi_app | ssid=%s", ssid_str);

	free(ssid_str);
	free(pass_str);

	return ESP_OK;
}

/* Returns the dashboard's current station connection status. */
static esp_err_t http_server_wifi_connect_status_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "HTTPS_REQ | method=POST | uri=/wifiConnectStatus | type=json_status | wifi_status=%d", g_wifi_connect_status);
	
	 http_server_watchdog_kick();
	
	char statusJSON[100];

	/* The monitor task supplies the status used by this endpoint. */
	sprintf(statusJSON, "{\"wifi_connect_status\":%d}", g_wifi_connect_status);

	httpd_resp_set_type(req, "application/json");
	return http_server_send_response_checked(req, statusJSON, strlen(statusJSON));

}

/* Returns network details for a connected station. */
static esp_err_t http_server_get_wifi_connect_info_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/wifiConnectInfo.json | type=json_info | wifi_status=%d", g_wifi_connect_status);

	http_server_watchdog_kick();

	char ipInfoJSON[200];
	memset(ipInfoJSON, 0, sizeof(ipInfoJSON));

	char ip[IP4ADDR_STRLEN_MAX];
	char netmask[IP4ADDR_STRLEN_MAX];
	char gw[IP4ADDR_STRLEN_MAX];

	/* Read station IP information only after connection success is recorded. */
	if (g_wifi_connect_status == HTTP_WIFI_STATUS_CONNECT_SUCCESS)
	{
		wifi_ap_record_t wifi_data;
		ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&wifi_data));
		char *ssid = (char*)wifi_data.ssid;

		esp_netif_ip_info_t ip_info;
		ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_sta, &ip_info));
		esp_ip4addr_ntoa(&ip_info.ip, ip, IP4ADDR_STRLEN_MAX);
		esp_ip4addr_ntoa(&ip_info.netmask, netmask, IP4ADDR_STRLEN_MAX);
		esp_ip4addr_ntoa(&ip_info.gw, gw, IP4ADDR_STRLEN_MAX);

		sprintf(ipInfoJSON, "{\"ip\":\"%s\",\"netmask\":\"%s\",\"gw\":\"%s\",\"ap\":\"%s\"}", ip, netmask, gw, ssid);
		ESP_LOGI(TAG, "WIFI_INFO | event=sta_info_ready | ssid=%s | ip=%s | netmask=%s | gateway=%s", ssid, ip, netmask, gw);
	}

	httpd_resp_set_type(req, "application/json");
	return http_server_send_response_checked(req, ipInfoJSON, strlen(ipInfoJSON));
 
}

/* Forwards a station disconnect request to Wi-Fi control. */
static esp_err_t http_server_wifi_disconnect_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "HTTPS_REQ | method=DELETE | uri=/wifiDisconnect.json | type=wifi_control | action=disconnect_sta");

	http_server_watchdog_kick();

	/* Use the Wi-Fi task to apply the disconnect and credential reset. */
	wifi_app_send_message(WIFI_APP_MSG_USER_REQUESTED_STA_DISCONNECT);

	return ESP_OK;
}

/* Returns the synchronized local time to the dashboard. */
static esp_err_t http_server_get_local_time_json_handler(httpd_req_t *req)
{
	/* Only format a response after the monitor marks time service ready. */
	ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/localTime.json | type=json_time | time_ready=%d", g_is_local_time_set);

	http_server_watchdog_kick();

	char localTimeJSON[100] = {0};

	if (g_is_local_time_set)
	{
		sprintf(localTimeJSON, "{\"time\":\"%s\"}", sntp_time_sync_get_time());
	}

	httpd_resp_set_type(req, "application/json");
	return http_server_send_response_checked(req, localTimeJSON, strlen(localTimeJSON));

}

/* Returns the cached gas reading as dashboard JSON. */
static esp_err_t http_server_get_ccs811_sensor_readings_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/ccs811Sensor.json | type=json_sensor");

	http_server_watchdog_kick();

	char sensorJSON[192] = {0};
	ccs811_sensor_data_t data;

	/* Report invalid data instead of using an unavailable sensor cache. */
	if (CCS811_get_latest_reading(&data) && data.valid)
	{
		snprintf(sensorJSON,
				 sizeof(sensorJSON),
				 "{\"valid\":true,\"eco2\":%u,\"tvoc\":%u,\"raw_current\":%u,\"raw_voltage_adc\":%u,\"raw_voltage\":\"%.3f\"}",
				 data.eco2_ppm,
				 data.tvoc_ppb,
				 data.raw_current_ua,
				 data.raw_voltage_adc,
				 data.raw_voltage);
	}
	else
	{
		snprintf(sensorJSON, sizeof(sensorJSON), "{\"valid\":false}");
	}

	httpd_resp_set_type(req, "application/json");

	return http_server_send_response_checked(
    	req,
    	sensorJSON,
    	strlen(sensorJSON)
);
}

/* Returns cached pressure sensor data as dashboard JSON. */
static esp_err_t http_server_get_bmp280_sensor_readings_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=%s | type=json_sensor", req->uri);

	http_server_watchdog_kick();

	char sensorJSON[224] = {0};
	bmp280_sensor_data_t data;

	if (BMP280_get_latest_reading(&data) && data.valid)
	{
		/* Distinguish BME280 humidity support from the BMP280 variant. */
		const char *sensor_name = data.humidity_available ? "BME280" : "BMP280";

		snprintf(sensorJSON,
				 sizeof(sensorJSON),
				 "{\"valid\":true,\"sensor\":\"%s\",\"chip_id\":%u,\"temperature\":\"%.2f\",\"pressure_pa\":\"%.2f\",\"pressure_hpa\":\"%.2f\",\"humidity_available\":%s,\"humidity\":\"%.2f\"}",
				 sensor_name,
				 data.chip_id,
				 data.temperature_c,
				 data.pressure_pa,
				 data.pressure_hpa,
				 data.humidity_available ? "true" : "false",
				 data.humidity_percent);
	}
	else
	{
		snprintf(sensorJSON, sizeof(sensorJSON), "{\"valid\":false}");
	}

	httpd_resp_set_type(req, "application/json");
	return http_server_send_response_checked(req, sensorJSON, strlen(sensorJSON));
 
}

/* Returns the configured SoftAP SSID. */
static esp_err_t http_server_get_ap_ssid_json_handler(httpd_req_t *req)
{
	ESP_LOGI(TAG, "HTTPS_REQ | method=GET | uri=/apSSID.json | type=json_ap_info");

	http_server_watchdog_kick();

	char ssidJSON[80];

	wifi_config_t wifi_config;
	memset(&wifi_config, 0, sizeof(wifi_config));

	/* Read the AP interface configuration rather than station credentials. */
	esp_wifi_get_config(WIFI_IF_AP, &wifi_config);

	snprintf(ssidJSON, sizeof(ssidJSON), "{\"ssid\":\"%s\"}", (char *)wifi_config.ap.ssid);

	httpd_resp_set_type(req, "application/json");
	return http_server_send_response_checked(req, ssidJSON, strlen(ssidJSON));

}

/* Configures TLS and registers the dashboard and control endpoints. */
static httpd_handle_t http_server_configure(void)
{
    httpd_ssl_config_t config = HTTPD_SSL_CONFIG_DEFAULT();

    /* Load the server certificate and key embedded by the build. */
    config.servercert = servercert_pem_start;
    config.servercert_len = servercert_pem_end - servercert_pem_start;

    config.prvtkey_pem = prvtkey_pem_start;
    config.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

    config.transport_mode = HTTPD_SSL_TRANSPORT_SECURE;

http_server_monitor_queue_handle = xQueueCreate(3, sizeof(http_server_queue_message_t));
if (http_server_monitor_queue_handle == NULL)
{
    ESP_LOGE(TAG, "HTTPS_SERVER | event=queue_create_failed");
    return NULL;
}

BaseType_t monitor_task_status = xTaskCreatePinnedToCore(
    &http_server_monitor,
    "http_server_monitor",
    HTTP_SERVER_MONITOR_STACK_SIZE,
    NULL,
    HTTP_SERVER_MONITOR_PRIORITY,
    &task_http_server_monitor,
    HTTP_SERVER_MONITOR_CORE_ID
);

if (monitor_task_status != pdPASS)
{
    ESP_LOGE(TAG, "HTTPS_SERVER | event=monitor_task_create_failed");

    vQueueDelete(http_server_monitor_queue_handle);
    http_server_monitor_queue_handle = NULL;

    return NULL;
}

    config.httpd.core_id = HTTP_SERVER_TASK_CORE_ID;

    config.httpd.task_priority = HTTP_SERVER_TASK_PRIORITY;

    config.httpd.stack_size = HTTP_SERVER_TASK_STACK_SIZE;

    config.httpd.max_uri_handlers = 20;

    config.httpd.recv_wait_timeout = 10;
    config.httpd.send_wait_timeout = 10;

    ESP_LOGI(TAG,
             "HTTPS_SERVER | event=start | transport=TLS | port=%d | task_priority=%d | stack_size=%d | core_id=%d | max_uri_handlers=%d | recv_timeout_s=%d | send_timeout_s=%d | cert_len=%u | key_len=%u",
             config.port_secure,
             config.httpd.task_priority,
             config.httpd.stack_size,
             config.httpd.core_id,
             config.httpd.max_uri_handlers,
             config.httpd.recv_wait_timeout,
             config.httpd.send_wait_timeout,
             (unsigned)config.servercert_len,
             (unsigned)config.prvtkey_len);

    if (httpd_ssl_start(&http_server_handle, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTPS_SERVER | event=register_uri_handlers | state=start");

        httpd_uri_t jquery_js = {
            .uri = "/jquery-3.3.1.min.js",
            .method = HTTP_GET,
            .handler = http_server_jquery_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &jquery_js);

        httpd_uri_t index_html = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = http_server_index_html_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &index_html);

        httpd_uri_t app_css = {
            .uri = "/app.css",
            .method = HTTP_GET,
            .handler = http_server_app_css_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &app_css);

        httpd_uri_t app_js = {
            .uri = "/app.js",
            .method = HTTP_GET,
            .handler = http_server_app_js_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &app_js);

        httpd_uri_t favicon_ico = {
            .uri = "/favicon.ico",
            .method = HTTP_GET,
            .handler = http_server_favicon_ico_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &favicon_ico);

        httpd_uri_t OTA_update = {
            .uri = "/OTAupdate",
            .method = HTTP_POST,
            .handler = http_server_OTA_update_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &OTA_update);

        httpd_uri_t OTA_status = {
            .uri = "/OTAstatus",
            .method = HTTP_POST,
            .handler = http_server_OTA_status_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &OTA_status);

        httpd_uri_t wifi_connect_json = {
            .uri = "/wifiConnect.json",
            .method = HTTP_POST,
            .handler = http_server_wifi_connect_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &wifi_connect_json);

        httpd_uri_t wifi_connect_status_json = {
            .uri = "/wifiConnectStatus",
            .method = HTTP_POST,
            .handler = http_server_wifi_connect_status_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &wifi_connect_status_json);

        httpd_uri_t wifi_connect_info_json = {
            .uri = "/wifiConnectInfo.json",
            .method = HTTP_GET,
            .handler = http_server_get_wifi_connect_info_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &wifi_connect_info_json);

        httpd_uri_t wifi_disconnect_json = {
            .uri = "/wifiDisconnect.json",
            .method = HTTP_DELETE,
            .handler = http_server_wifi_disconnect_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &wifi_disconnect_json);

        httpd_uri_t local_time_json = {
            .uri = "/localTime.json",
            .method = HTTP_GET,
            .handler = http_server_get_local_time_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &local_time_json);

        httpd_uri_t ccs811_sensor_json = {
            .uri = "/ccs811Sensor.json",
            .method = HTTP_GET,
            .handler = http_server_get_ccs811_sensor_readings_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &ccs811_sensor_json);

        httpd_uri_t bmp280_sensor_json = {
            .uri = "/bmp280Sensor.json",
            .method = HTTP_GET,
            .handler = http_server_get_bmp280_sensor_readings_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &bmp280_sensor_json);

        httpd_uri_t bme280_sensor_json = {
            .uri = "/bme280Sensor.json",
            .method = HTTP_GET,
            .handler = http_server_get_bmp280_sensor_readings_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &bme280_sensor_json);

        httpd_uri_t ap_ssid_json = {
            .uri = "/apSSID.json",
            .method = HTTP_GET,
            .handler = http_server_get_ap_ssid_json_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(http_server_handle, &ap_ssid_json);

        return http_server_handle;
    }

    return NULL;
}

/* Starts the local HTTPS service when it is stopped. */
void http_server_start(void)
{
    if (http_server_handle == NULL)
    {
        s_https_fatal = false;
        s_https_consecutive_failures = 0;

        /* Enable HTTPS supervision only after checking server startup. */
        http_server_handle = http_server_configure();

        if (http_server_handle != NULL)
        {
            app_watchdog_set_enabled(APP_WD_HTTPS, true);
            http_server_watchdog_kick();

            ESP_LOGI(TAG, "HTTPS_SERVER | event=watchdog_enabled");
        }
        else
        {
            app_watchdog_set_enabled(APP_WD_HTTPS, true);
            http_server_fatal_stop_kicking_watchdog("http_server_configure failed");
        }
    }
}

/* Stops HTTPS and releases its monitor task and queue. */
void http_server_stop(void)
{
    /* Disable supervision before tearing down the server resources. */
    app_watchdog_set_enabled(APP_WD_HTTPS, false);

    s_https_fatal = false;
    s_https_consecutive_failures = 0;

    if (http_server_handle)
    {
        httpd_ssl_stop(http_server_handle);
        ESP_LOGI(TAG, "HTTPS_SERVER | event=stop | target=https_server | state=stopped");
        http_server_handle = NULL;
    }

    if (task_http_server_monitor)
    {
        vTaskDelete(task_http_server_monitor);
        ESP_LOGI(TAG, "HTTPS_SERVER | event=stop | target=monitor_task | state=stopped");
        task_http_server_monitor = NULL;
    }

    if (http_server_monitor_queue_handle)
    {
        vQueueDelete(http_server_monitor_queue_handle);
        ESP_LOGI(TAG, "HTTPS_SERVER | event=stop | target=monitor_queue | state=deleted");
        http_server_monitor_queue_handle = NULL;
    }
}

/* Queues a service event for the HTTP monitor. */
BaseType_t http_server_monitor_send_message(http_server_message_e msgID)
{
    /* Skip delivery when the monitor queue has not been created. */
    if (http_server_monitor_queue_handle == NULL)
    {
        ESP_LOGW(TAG,
                 "HTTP_MONITOR | event=send_skipped | reason=queue_not_ready | msgID=%d",
                 msgID);

        return pdFALSE;
    }

    http_server_queue_message_t msg;
    msg.msgID = msgID;

    return xQueueSend(http_server_monitor_queue_handle, &msg, portMAX_DELAY);
}

/* Restarts the ESP32 when the OTA delay expires. */
void http_server_fw_update_reset_callback(void *arg)
{
	(void)arg;
	ESP_LOGW(TAG, "OTA_RESET_TIMER | event=timeout | action=esp_restart");
	/* Reboot into the partition selected by the completed update. */
	esp_restart();
}
