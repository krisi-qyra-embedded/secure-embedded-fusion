/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <stdbool.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "driver/sdmmc_host.h"

#include "aws_iot.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tasks_common.h"
#include "app_watchdog.h"

#include "ccs811_sensor.h"
#include "bmp280_sensor.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

static const char *TAG = "aws_iot";

static TaskHandle_t task_aws_iot = NULL;

static volatile bool g_aws_iot_stop_requested = false;

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");

char HostAddress[255] = AWS_IOT_MQTT_HOST;

uint32_t port = AWS_IOT_MQTT_PORT;

#define AWS_IOT_CONNECT_RETRY_DELAY_MS      1000U
#define AWS_WATCHDOG_FAULT_TEST        		0
#define AWS_WATCHDOG_FAULT_AFTER_MS    		30000
#define AWS_IOT_PUBLISH_PERIOD_MS           3000U
#define AWS_IOT_YIELD_TIMEOUT_MS            100U

static volatile bool s_aws_iot_fatal = false;

#if AWS_WATCHDOG_FAULT_TEST
/* Checks whether the optional AWS stall test has reached its delay. */
static bool aws_watchdog_fault_test_expired(void)
{
    /* Measure the test delay from the first watchdog check. */
    static int64_t start_ms = 0;

    if (start_ms == 0)
    {
        start_ms = esp_timer_get_time() / 1000;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;

    return ((now_ms - start_ms) >= AWS_WATCHDOG_FAULT_AFTER_MS);
}
#endif


/* Updates the AWS heartbeat unless a fatal fault is latched. */
static void aws_iot_watchdog_kick(void)
{
    /* Suppress heartbeats after AWS enters its fatal state. */
    if (!s_aws_iot_fatal)
    {
#if AWS_WATCHDOG_FAULT_TEST
        if (aws_watchdog_fault_test_expired())
        {
            ESP_LOGE(TAG, "AWS_WD_TEST | intentionally freezing AWS watchdog kick");

            while (1)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
#endif

        app_watchdog_kick(APP_WD_AWS);
    }
}

/* Enables supervision for a newly started AWS task. */
static void aws_iot_watchdog_enable(void)
{
    /* Clear the fatal latch and seed the AWS heartbeat. */
    s_aws_iot_fatal = false;
    app_watchdog_set_enabled(APP_WD_AWS, true);
    app_watchdog_kick(APP_WD_AWS);
}

/* Suspends heartbeat checks while AWS is stopped. */
static void aws_iot_watchdog_disable(void)
{
    /* A deliberately stopped MQTT service must not expire the watchdog. */
    app_watchdog_set_enabled(APP_WD_AWS, false);
}

/* Latches a fatal MQTT error and waits for watchdog recovery. */
static void aws_iot_fatal_stop_kicking_watchdog(const char *reason, IoT_Error_t rc)
{
    ESP_LOGE(TAG,
             "AWS_FATAL | reason=%s | rc=%d | stop_kicking_watchdog",
             reason,
             rc);

    /* Stop feeding the AWS heartbeat while yielding to other tasks. */
    s_aws_iot_fatal = true;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Disables AWS supervision and deletes the current task. */
static void aws_iot_task_exit_cleanly(void)
{
    ESP_LOGW(TAG, "AWS_WD | event=disable | reason=aws_task_exit_cleanly");

    aws_iot_watchdog_disable();
    /* Clear the task handle so mode switching can observe shutdown. */
    task_aws_iot = NULL;
    vTaskDelete(NULL);
}

/* Logs the topic and payload delivered by MQTT. */
void iot_subscribe_callback_handler(AWS_IoT_Client *pClient,
                                    char *topicName,
                                    uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params,
                                    void *pData)
{
    aws_iot_watchdog_kick();

    /* Use the supplied lengths because MQTT buffers need not be terminated. */
    ESP_LOGI(TAG,
             "Subscribe callback Test: %.*s\t%.*s",
             topicNameLen,
             topicName,
             (int)params->payloadLen,
             (char *)params->payload);
}

/* Handles MQTT disconnection and requests reconnection when needed. */
void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data)
{
    aws_iot_watchdog_kick();

    ESP_LOGW(TAG, "MQTT Disconnect");

    IoT_Error_t rc = FAILURE;

    if (pClient == NULL)
    {
        return;
    }

    /* Attempt a manual reconnect only when automatic reconnect is disabled. */
    if (aws_iot_is_autoreconnect_enabled(pClient))
    {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, reconnecting attempt will start now");
    }
    else
    {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");

        rc = aws_iot_mqtt_attempt_reconnect(pClient);

        if (NETWORK_RECONNECTED == rc)
        {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        }
        else
        {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

/* Builds MQTT JSON from the available sensor caches. */
static void build_sensor_json_payload(char *payload, size_t payload_size)
{
    ccs811_sensor_data_t ccs811_data;
    bmp280_sensor_data_t bmp280_data;

    /* Include invalid flags when a cached sensor reading is unavailable. */
    bool ccs811_ok = CCS811_get_latest_reading(&ccs811_data) && ccs811_data.valid;
    bool bmp280_ok = BMP280_get_latest_reading(&bmp280_data) && bmp280_data.valid;

    if (ccs811_ok && bmp280_ok)
    {
        const char *sensor_name = bmp280_data.humidity_available ? "BME280" : "BMP280";

        snprintf(payload,
                 payload_size,
                 "{"
                 "\"device\":\"ESP32_SECURE_FUSION\","
                 "\"ccs811\":{"
                 "\"valid\":true,"
                 "\"eco2_ppm\":%u,"
                 "\"tvoc_ppb\":%u,"
                 "\"raw_current_ua\":%u,"
                 "\"raw_voltage_adc\":%u,"
                 "\"raw_voltage\":%.3f"
                 "},"
                 "\"environment\":{"
                 "\"valid\":true,"
                 "\"sensor\":\"%s\","
                 "\"chip_id\":%u,"
                 "\"temperature_c\":%.2f,"
                 "\"pressure_pa\":%.2f,"
                 "\"pressure_hpa\":%.2f,"
                 "\"humidity_available\":%s,"
                 "\"humidity_percent\":%.2f"
                 "}"
                 "}",
                 ccs811_data.eco2_ppm,
                 ccs811_data.tvoc_ppb,
                 ccs811_data.raw_current_ua,
                 ccs811_data.raw_voltage_adc,
                 ccs811_data.raw_voltage,
                 sensor_name,
                 bmp280_data.chip_id,
                 bmp280_data.temperature_c,
                 bmp280_data.pressure_pa,
                 bmp280_data.pressure_hpa,
                 bmp280_data.humidity_available ? "true" : "false",
                 bmp280_data.humidity_percent);
    }
    else if (ccs811_ok && !bmp280_ok)
    {
        snprintf(payload,
                 payload_size,
                 "{"
                 "\"device\":\"ESP32_SECURE_FUSION\","
                 "\"ccs811\":{"
                 "\"valid\":true,"
                 "\"eco2_ppm\":%u,"
                 "\"tvoc_ppb\":%u,"
                 "\"raw_current_ua\":%u,"
                 "\"raw_voltage_adc\":%u,"
                 "\"raw_voltage\":%.3f"
                 "},"
                 "\"environment\":{\"valid\":false}"
                 "}",
                 ccs811_data.eco2_ppm,
                 ccs811_data.tvoc_ppb,
                 ccs811_data.raw_current_ua,
                 ccs811_data.raw_voltage_adc,
                 ccs811_data.raw_voltage);
    }
    else if (!ccs811_ok && bmp280_ok)
    {
        const char *sensor_name = bmp280_data.humidity_available ? "BME280" : "BMP280";

        snprintf(payload,
                 payload_size,
                 "{"
                 "\"device\":\"ESP32_SECURE_FUSION\","
                 "\"ccs811\":{\"valid\":false},"
                 "\"environment\":{"
                 "\"valid\":true,"
                 "\"sensor\":\"%s\","
                 "\"chip_id\":%u,"
                 "\"temperature_c\":%.2f,"
                 "\"pressure_pa\":%.2f,"
                 "\"pressure_hpa\":%.2f,"
                 "\"humidity_available\":%s,"
                 "\"humidity_percent\":%.2f"
                 "}"
                 "}",
                 sensor_name,
                 bmp280_data.chip_id,
                 bmp280_data.temperature_c,
                 bmp280_data.pressure_pa,
                 bmp280_data.pressure_hpa,
                 bmp280_data.humidity_available ? "true" : "false",
                 bmp280_data.humidity_percent);
    }
    else
    {
        snprintf(payload,
                 payload_size,
                 "{"
                 "\"device\":\"ESP32_SECURE_FUSION\","
                 "\"ccs811\":{\"valid\":false},"
                 "\"environment\":{\"valid\":false}"
                 "}");
    }
}

/* Connects to AWS and periodically publishes cached sensor readings. */
void aws_iot_task(void *param)
{
    (void)param;

    char cPayload[512];

    IoT_Error_t rc = FAILURE;
    bool mqtt_connected = false;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;
    IoT_Publish_Message_Params paramsQOS0;

    aws_iot_watchdog_kick();

    ESP_LOGI(TAG,
             "AWS IoT SDK Version %d.%d.%d-%s",
             VERSION_MAJOR,
             VERSION_MINOR,
             VERSION_PATCH,
             VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false;
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if (SUCCESS != rc)
    {
        aws_iot_fatal_stop_kicking_watchdog("aws_iot_mqtt_init failed", rc);
    }

    aws_iot_watchdog_kick();

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;

    connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t)strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");

    do
    {
        if (g_aws_iot_stop_requested)
        {
            ESP_LOGW(TAG, "AWS IoT start cancelled before MQTT connect");
            aws_iot_task_exit_cleanly();
        }

        aws_iot_watchdog_kick();

        rc = aws_iot_mqtt_connect(&client, &connectParams);

        if (SUCCESS != rc)
        {
            ESP_LOGW(TAG,
                     "AWS_CONNECT | event=failed | rc=%d | host=%s | port=%d | action=retry",
                     rc,
                     mqttInitParams.pHostURL,
                     mqttInitParams.port);

            aws_iot_watchdog_kick();
            vTaskDelay(pdMS_TO_TICKS(AWS_IOT_CONNECT_RETRY_DELAY_MS));
        }

    } while ((SUCCESS != rc) && !g_aws_iot_stop_requested);

    if (g_aws_iot_stop_requested)
    {
        ESP_LOGW(TAG, "AWS IoT stop requested after connect loop");
        aws_iot_task_exit_cleanly();
    }

    ESP_LOGI(TAG, "Connected to AWS IoT Core");
    mqtt_connected = true;
    aws_iot_watchdog_kick();

    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if (SUCCESS != rc)
    {
        if (mqtt_connected)
        {
            aws_iot_mqtt_disconnect(&client);
            mqtt_connected = false;
        }

        aws_iot_fatal_stop_kicking_watchdog("aws_iot_mqtt_autoreconnect_set_status failed", rc);
    }

    aws_iot_watchdog_kick();

    if (g_aws_iot_stop_requested)
    {
        ESP_LOGW(TAG, "AWS stop requested after connect, disconnecting before subscribe");

        if (mqtt_connected)
        {
            aws_iot_mqtt_disconnect(&client);
            mqtt_connected = false;
        }

        aws_iot_task_exit_cleanly();
    }

    const char *TOPIC = "test_topic/esp32";
    const int TOPIC_LEN = strlen(TOPIC);

    ESP_LOGI(TAG, "Subscribing to topic: %s", TOPIC);

    rc = aws_iot_mqtt_subscribe(&client,
                                TOPIC,
                                TOPIC_LEN,
                                QOS0,
                                iot_subscribe_callback_handler,
                                NULL);

    if (SUCCESS != rc)
    {
        if (g_aws_iot_stop_requested)
        {
            ESP_LOGW(TAG, "Subscribe failed during stop request, exiting AWS task");

            if (mqtt_connected)
            {
                aws_iot_mqtt_disconnect(&client);
                mqtt_connected = false;
            }

            aws_iot_task_exit_cleanly();
        }

        ESP_LOGW(TAG,
                 "AWS_SUBSCRIBE | event=failed | rc=%d | action=continue_publish_path",
                 rc);

        rc = SUCCESS;
    }
    else
    {
        ESP_LOGI(TAG, "Subscribed successfully to topic: %s", TOPIC);
    }

    aws_iot_watchdog_kick();

    paramsQOS0.qos = QOS0;
    paramsQOS0.payload = (void *)cPayload;
    paramsQOS0.isRetained = 0;

    while (!g_aws_iot_stop_requested)
    {
        rc = aws_iot_mqtt_yield(&client, AWS_IOT_YIELD_TIMEOUT_MS);
        aws_iot_watchdog_kick();

        /* Skip publishing while the MQTT client is attempting reconnection. */
        if (NETWORK_ATTEMPTING_RECONNECT == rc)
        {
            ESP_LOGW(TAG, "MQTT reconnecting... skipping publish");
            vTaskDelay(pdMS_TO_TICKS(AWS_IOT_CONNECT_RETRY_DELAY_MS));
            aws_iot_watchdog_kick();
            continue;
        }

        if ((SUCCESS != rc) && (NETWORK_RECONNECTED != rc))
        {
            ESP_LOGW(TAG,
                     "AWS_YIELD | event=non_success | rc=%d | action=continue",
                     rc);

            vTaskDelay(pdMS_TO_TICKS(AWS_IOT_CONNECT_RETRY_DELAY_MS));
            aws_iot_watchdog_kick();
            continue;
        }

        ESP_LOGI(TAG,
                 "Stack remaining for task '%s' is %d bytes",
                 pcTaskGetName(NULL),
                 uxTaskGetStackHighWaterMark(NULL));

        vTaskDelay(pdMS_TO_TICKS(AWS_IOT_PUBLISH_PERIOD_MS));
        aws_iot_watchdog_kick();

        build_sensor_json_payload(cPayload, sizeof(cPayload));

        paramsQOS0.payload = (void *)cPayload;
        paramsQOS0.payloadLen = strlen(cPayload);

        ESP_LOGI(TAG,
                 "Publishing sensor data to AWS IoT topic [%s]: %s",
                 TOPIC,
                 cPayload);

        rc = aws_iot_mqtt_publish(&client,
                                  TOPIC,
                                  TOPIC_LEN,
                                  &paramsQOS0);

        if (SUCCESS == rc)
        {
            ESP_LOGI(TAG, "Sensor publish successful");
        }
        else
        {
            ESP_LOGW(TAG, "Sensor publish returned rc = %d", rc);
        }

        aws_iot_watchdog_kick();
    }

    ESP_LOGW(TAG, "AWS IoT task stopping cleanly");

    if (mqtt_connected)
    {
        aws_iot_mqtt_disconnect(&client);
        mqtt_connected = false;
    }

    aws_iot_task_exit_cleanly();
}

/* Starts the AWS task if it is not already running. */
void aws_iot_start(void)
{
    if (task_aws_iot == NULL)
    {
        ESP_LOGI(TAG, "AWS_WD | event=enable | reason=aws_iot_start");

        g_aws_iot_stop_requested = false;
        /* Enable supervision before creating the MQTT task. */
        aws_iot_watchdog_enable();

        BaseType_t task_status = xTaskCreatePinnedToCore(&aws_iot_task,
                                                         "aws_iot_task",
                                                         AWS_IOT_TASK_STACK_SIZE,
                                                         NULL,
                                                         AWS_IOT_TASK_PRIORITY,
                                                         &task_aws_iot,
                                                         AWS_IOT_TASK_CORE_ID);

        if (task_status != pdPASS)
        {
            aws_iot_fatal_stop_kicking_watchdog("aws_iot_task_create failed", FAILURE);
        }
    }
}

/* Requests an orderly shutdown of the AWS task. */
void aws_iot_stop(void)
{
    if (task_aws_iot != NULL)
    {
        ESP_LOGW(TAG, "AWS IoT stop requested");
        /* Let the task leave its connection loop and release the MQTT connection. */
        g_aws_iot_stop_requested = true;
    }
    else
    {
        aws_iot_watchdog_disable();
    }
}

/* Reports whether the AWS task handle is still present. */
bool aws_iot_is_running(void)
{
    /* Mode changes use this handle to wait for task shutdown. */
    return task_aws_iot != NULL;
}
