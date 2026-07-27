/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#ifndef MAIN_WIFI_APP_H_
#define MAIN_WIFI_APP_H_

#include "esp_netif.h"
#include <stdint.h>
#include "esp_wifi_types.h"
#include "freertos/FreeRTOS.h"

typedef void (*wifi_connected_event_callback_t)(void);
typedef void (*wifi_app_softap_client_connected_cb_t)(void);
typedef void (*wifi_app_softap_last_client_disconnected_cb_t)(void);

#define WIFI_AP_SSID				"ESP32_AP"			// AP name
#define WIFI_AP_PASSWORD			"password"			// AP password
#define WIFI_AP_CHANNEL				1					// AP channel
#define WIFI_AP_SSID_HIDDEN			0					// AP visibility
#define WIFI_AP_MAX_CONNECTIONS		5					// AP max clients
#define WIFI_AP_BEACON_INTERVAL		100					// AP beacon: 100 milliseconds recommended
#define WIFI_AP_IP                  "192.168.4.1"		// AP default IP
#define WIFI_AP_GATEWAY             "192.168.4.1"		// AP default Gateway (should be the same as the IP)
#define WIFI_AP_NETMASK             "255.255.255.0"		// AP netmask
#define WIFI_AP_BANDWIDTH			WIFI_BW_HT20		// AP bandwidth 20 MHz (40 MHz is the other option)
#define WIFI_STA_POWER_SAVE			WIFI_PS_NONE		// Power save not used
#define MAX_SSID_LENGTH				32					// IEEE standard maximum
#define MAX_PASSWORD_LENGTH			64					// IEEE standard maximum
#define MAX_CONNECTION_RETRIES		5					// Retry number on disconnect

extern esp_netif_t* esp_netif_sta;
extern esp_netif_t* esp_netif_ap;

typedef enum wifi_app_message
{
	WIFI_APP_MSG_START_HTTP_SERVER = 0,
	WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER,
	WIFI_APP_MSG_STA_CONNECTED_GOT_IP,
	WIFI_APP_MSG_USER_REQUESTED_STA_DISCONNECT,
	WIFI_APP_MSG_LOAD_SAVED_CREDENTIALS,	
	WIFI_APP_MSG_STA_DISCONNECTED,
} wifi_app_message_e;

typedef struct wifi_app_queue_message
{
	wifi_app_message_e msgID;
} wifi_app_queue_message_t;

BaseType_t wifi_app_send_message(wifi_app_message_e msgID);

void wifi_app_start(void);

wifi_config_t* wifi_app_get_wifi_config(void);

void wifi_app_set_callback(wifi_connected_event_callback_t cb);

void wifi_app_call_callback(void);

void wifi_app_set_softap_client_connected_callback(wifi_app_softap_client_connected_cb_t cb);
void wifi_app_set_softap_last_client_disconnected_callback(wifi_app_softap_last_client_disconnected_cb_t cb);
int wifi_app_get_softap_client_count(void);

int8_t wifi_app_get_rssi(void);

#endif /* MAIN_WIFI_APP_H_ */


