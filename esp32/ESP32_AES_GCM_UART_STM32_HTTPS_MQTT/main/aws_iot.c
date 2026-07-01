/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-01
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
#include "wifi_app.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

static const char *TAG = "aws_iot";

static TaskHandle_t task_aws_iot = NULL;

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");

char HostAddress[255] = AWS_IOT_MQTT_HOST;

uint32_t port = AWS_IOT_MQTT_PORT;

/* Logs the topic and payload delivered by MQTT. */
void iot_subscribe_callback_handler(AWS_IoT_Client *pClient,
                                    char *topicName,
                                    uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params,
                                    void *pData)
{
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
    ESP_LOGW(TAG, "MQTT Disconnect");

    IoT_Error_t rc = FAILURE;

    if (NULL == pClient)
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

/* Connects to AWS and publishes station signal strength. */
void aws_iot_task(void *param)
{
    char cPayload[100];

    IoT_Error_t rc = FAILURE;

    AWS_IoT_Client client;
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    IoT_Publish_Message_Params paramsQOS0;

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
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error: %d", rc);
        abort();
    }

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;

    connectParams.pClientID = CONFIG_AWS_EXAMPLE_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t)strlen(CONFIG_AWS_EXAMPLE_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");

    do
    {
        rc = aws_iot_mqtt_connect(&client, &connectParams);

        if (SUCCESS != rc)
        {
            ESP_LOGE(TAG,
                     "Error(%d) connecting to %s:%d",
                     rc,
                     mqttInitParams.pHostURL,
                     mqttInitParams.port);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

    } while (SUCCESS != rc);

    ESP_LOGI(TAG, "Connected to AWS IoT Core");

    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if (SUCCESS != rc)
    {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
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
        ESP_LOGE(TAG, "Error subscribing: %d", rc);
        abort();
    }

    ESP_LOGI(TAG, "Subscribed successfully to topic: %s", TOPIC);

    paramsQOS0.qos = QOS0;
    paramsQOS0.payload = (void *)cPayload;
    paramsQOS0.isRetained = 0;

    /* Skip RSSI publication while MQTT reconnection is in progress. */
    while ((NETWORK_ATTEMPTING_RECONNECT == rc ||
            NETWORK_RECONNECTED == rc ||
            SUCCESS == rc))
    {
        rc = aws_iot_mqtt_yield(&client, 100);

        if (NETWORK_ATTEMPTING_RECONNECT == rc)
        {
            ESP_LOGW(TAG, "MQTT reconnecting... skipping publish");
            continue;
        }

        ESP_LOGI(TAG,
                 "Stack remaining for task '%s' is %d bytes",
                 pcTaskGetName(NULL),
                 uxTaskGetStackHighWaterMark(NULL));

        vTaskDelay(3000 / portTICK_PERIOD_MS);

        snprintf(cPayload,
                 sizeof(cPayload),
                 "%s : %d",
                 "WiFi RSSI",
                 wifi_app_get_rssi());

        paramsQOS0.payloadLen = strlen(cPayload);

        ESP_LOGI(TAG,
                 "Publishing to AWS IoT topic [%s]: %s",
                 TOPIC,
                 cPayload);

        rc = aws_iot_mqtt_publish(&client,
                                  TOPIC,
                                  TOPIC_LEN,
                                  &paramsQOS0);

        if (SUCCESS == rc)
        {
            ESP_LOGI(TAG, "Publish successful");
        }
        else
        {
            ESP_LOGW(TAG, "Publish returned rc = %d", rc);
        }
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}

/* Starts the AWS MQTT task once. */
void aws_iot_start(void)
{
    /* The task handle prevents another MQTT worker from being created. */
    if (task_aws_iot == NULL)
    {
        xTaskCreatePinnedToCore(&aws_iot_task,
                                "aws_iot_task",
                                AWS_IOT_TASK_STACK_SIZE,
                                NULL,
                                AWS_IOT_TASK_PRIORITY,
                                &task_aws_iot,
                                AWS_IOT_TASK_CORE_ID);
    }
}