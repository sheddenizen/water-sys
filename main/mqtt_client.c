/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file subscribe_publish_sample.c
 * @brief simple MQTT publish and subscribe on the same topic
 *
 * This example takes the parameters from the build configuration and establishes a connection to the AWS IoT MQTT Platform.
 * It subscribes and publishes to the same topic - "test_topic/esp32"
 *
 * Some setup is required. See example README for details.
 *
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

#include "nvs.h"
#include "nvs_flash.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

#include "mqtt_client.h"

// Number of entries in queue
#define MSG_QUEUE_SIZE (64)
// If making blocking post to queue, wait this long before giving up
#define Q_SEND_WAIT_TIME (5000 / portTICK_PERIOD_MS)

#define Q_RECEIVE_WAIT_TIME (500 / portTICK_PERIOD_MS)

typedef struct post_data_msg
{
    mqtt_msg_serialize_fn serializer;
    char const * topic;
    mqtt_size_t data_size;
    uint8_t qos;
    bool retain;
    union
    {
        char data[0];
        char * data_ptr;
    };
} post_data_msg_t;


typedef union
{
    post_data_msg_t post_data_msg;

} cmd_payload_t;

typedef void (*msg_cmd_fn)(cmd_payload_t const * data_ptr);

typedef struct cmd_msg
{
    msg_cmd_fn msg_cmd;
    cmd_payload_t msg_data;
} cmd_msg_t;

static const char *TAG = "subpub";
static AWS_IoT_Client aws_iot_client;


/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t connection_event_group;

static QueueHandle_t msg_queue;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static mqtt_subscription_t const * mqtt_subscriptions;
static size_t mqtt_num_subscriptions;


/* CA Root certificate, device ("Thing") certificate and device
 * ("Thing") key.

   Example can be configured one of two ways:

   "Embedded Certs" are loaded from files in "certs/" and embedded into the app binary.

   "Filesystem Certs" are loaded from the filesystem (SD card, etc.)

   See example README for more details.
*/
#if defined(CONFIG_MQTT_EMBEDDED_CERTS)

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

#elif defined(CONFIG_MQTT_FILESYSTEM_CERTS)

static const char * DEVICE_CERTIFICATE_PATH = CONFIG_MQTT_CERTIFICATE_PATH;
static const char * DEVICE_PRIVATE_KEY_PATH = CONFIG_MQTT_PRIVATE_KEY_PATH;
static const char * ROOT_CA_PATH = CONFIG_MQTT_ROOT_CA_PATH;

#else
#error "Invalid method for loading certs"
#endif

/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
uint32_t port = AWS_IOT_MQTT_PORT;

static size_t q_width = 0;
static mqtt_size_t q_max_user_payload = 0;
static char pub_buffer[2500];


static void post_data(post_data_msg_t const * post_data_msg, char const * data_ptr)
{
//    ESP_LOGI(TAG, "publish, serializer at %p, payload at %p", post_data_msg->serializer, data_ptr);
    size_t out_len = post_data_msg->serializer(data_ptr, post_data_msg->data_size, pub_buffer, sizeof(pub_buffer));
    IoT_Publish_Message_Params pub_params;

    if (out_len >= sizeof(pub_buffer))
    {
        ESP_LOGI(TAG, "Result %u chars, truncated to %u", out_len, sizeof(pub_buffer) - 1);
        out_len = sizeof(pub_buffer) - 1;
    }
    // Safety null
    pub_buffer[out_len] = 0;

    pub_params.payloadLen = out_len;
    pub_params.qos = post_data_msg->qos == 0 ? QOS0 : QOS1;
    pub_params.payload = pub_buffer;
    pub_params.isRetained = post_data_msg->retain ? 1 : 0;

    IoT_Error_t rc = aws_iot_mqtt_publish(&aws_iot_client, post_data_msg->topic, strlen(post_data_msg->topic), &pub_params);
    if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
        ESP_LOGW(TAG, "QOS1 publish ack not received.");
        rc = SUCCESS;
    }
}

static void exec_post_data_cmd(cmd_payload_t const * payload)
{
    post_data(&payload->post_data_msg, payload->post_data_msg.data);
}

static void exec_post_data_ptr_cmd(cmd_payload_t const * payload)
{
    post_data(&payload->post_data_msg, payload->post_data_msg.data_ptr);
    free(payload->post_data_msg.data_ptr);
}

bool mqtt_post_data(char const * topic, mqtt_msg_serialize_fn serializer, char const * in_data, mqtt_size_t in_data_size, uint8_t qos, bool retain, bool can_block)
{
    char msg_buf[q_width];
    cmd_msg_t * cmd_msg_buf = (cmd_msg_t *)msg_buf;
    char * user_data_ptr;

    // Can user's data fit directly in the queue or do we need to alloc memory for it?
    if (in_data_size > q_max_user_payload)
    {
        // Set corresponding cmd wrapper - this one frees the data when done
        cmd_msg_buf->msg_cmd = &exec_post_data_ptr_cmd;
        user_data_ptr = malloc(in_data_size);
        cmd_msg_buf->msg_data.post_data_msg.data_ptr = user_data_ptr;
    }
    else
    {
        cmd_msg_buf->msg_cmd = &exec_post_data_cmd;
        user_data_ptr = cmd_msg_buf->msg_data.post_data_msg.data;
    }
    memcpy(user_data_ptr, in_data, in_data_size);
    cmd_msg_buf->msg_data.post_data_msg.serializer = serializer;
    cmd_msg_buf->msg_data.post_data_msg.data_size = in_data_size;
    cmd_msg_buf->msg_data.post_data_msg.topic = topic;
    cmd_msg_buf->msg_data.post_data_msg.qos = qos;
    cmd_msg_buf->msg_data.post_data_msg.retain = retain;
    xQueueSendToBack(msg_queue, cmd_msg_buf, can_block ? Q_SEND_WAIT_TIME : 0);
    return true;
}

static bool process_q(TickType_t timeout_ticks)
{
    char msg_buf[q_width];
    cmd_msg_t * cmd_msg_buf = (cmd_msg_t *)msg_buf;

    if  (pdTRUE == xQueueReceive(msg_queue, msg_buf, timeout_ticks))
    {
        // We got something. Execute it!
        cmd_msg_buf->msg_cmd(&cmd_msg_buf->msg_data);
    }
    return 0 != uxQueueMessagesWaiting(msg_queue);
}

void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    mqtt_msg_rx_fn handler = (mqtt_msg_rx_fn)pData;
    ESP_LOGI(TAG, "Rx'd: %.*s: '%.*s'", topicNameLen, topicName, params->payloadLen, (char const *)params->payload);

    // Copy and null-terminate the topic name and payload for the sake of convenience
    char *topic_buf = malloc(topicNameLen +1);
    char *payload_buf = malloc(params->payloadLen +1);

    if (!topic_buf || !payload_buf)
    {
        ESP_LOGE(TAG, "Unable to allocate memory for message %p (%hu) %p (%u)",
            topic_buf, topicNameLen, payload_buf, (size_t)params->payloadLen);
        free(payload_buf);
        free(topic_buf);
        return;
    }

    memcpy(topic_buf, topicName, topicNameLen);
    topic_buf[topicNameLen] = 0;
    memcpy(payload_buf, params->payload, params->payloadLen);
    payload_buf[params->payloadLen] = 0;

    ESP_LOGI(TAG, "%s\t%s", topic_buf, payload_buf);
    (*handler)(topic_buf, topicNameLen, payload_buf, (size_t)params->payloadLen);

    free(payload_buf);
    free(topic_buf);
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

void aws_iot_task(void *param) {

    IoT_Error_t rc = FAILURE;

    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

#if defined(CONFIG_MQTT_EMBEDDED_CERTS)
    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

#elif defined(CONFIG_MQTT_FILESYSTEM_CERTS)
    mqttInitParams.pRootCALocation = ROOT_CA_PATH;
    mqttInitParams.pDeviceCertLocation = DEVICE_CERTIFICATE_PATH;
    mqttInitParams.pDevicePrivateKeyLocation = DEVICE_PRIVATE_KEY_PATH;
#endif

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

#ifdef CONFIG_MQTT_SDCARD_CERTS
    ESP_LOGI(TAG, "Mounting SD card...");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
    };
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
        abort();
    }
#endif

    rc = aws_iot_mqtt_init(&aws_iot_client, &mqttInitParams);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(connection_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    /* Client ID is set in the menuconfig */
    connectParams.pClientID = CONFIG_MQTT_CLIENT_ID;
    connectParams.clientIDLen = (uint16_t) strlen(CONFIG_MQTT_CLIENT_ID);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");
    do {
        rc = aws_iot_mqtt_connect(&aws_iot_client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    } while(SUCCESS != rc);

    rc = aws_iot_mqtt_autoreconnect_set_status(&aws_iot_client, true);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    ESP_LOGI(TAG, "Subscribing...");
    for (int n = 0; n < mqtt_num_subscriptions; ++n)
    {
        rc = aws_iot_mqtt_subscribe(&aws_iot_client, mqtt_subscriptions[n].topic, strlen(mqtt_subscriptions[n].topic), QOS0, iot_subscribe_callback_handler, mqtt_subscriptions[n].handler);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error subscribing to %s, result: %d ", mqtt_subscriptions[n].topic, rc);
            break;
        }
    }

    IoT_Error_t rc_old = (IoT_Error_t)-1;

    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&aws_iot_client, 100);
        if (rc != rc_old)
        {
            ESP_LOGW(TAG, "State changed to %s ", rc == SUCCESS ? "Success" : (rc == NETWORK_RECONNECTED ? "Reconnected" : "Attempting Reconnect"));
            rc_old = rc;
        }
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

//        while (process_q(Q_RECEIVE_WAIT_TIME));
        while (process_q(0));
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}


void mqtt_connection_notify(bool is_connected)
{
    if (is_connected)
    {
        xEventGroupSetBits(connection_event_group, CONNECTED_BIT);
    }
    else
    {
        xEventGroupClearBits(connection_event_group, CONNECTED_BIT);
    }
}

void mqtt_init(mqtt_subscription_t const subscriptions[], size_t num_subscriptions, mqtt_size_t data_size)
{
    mqtt_subscriptions = subscriptions;
    mqtt_num_subscriptions = num_subscriptions;
    connection_event_group = xEventGroupCreate();
    q_width = data_size + offsetof(cmd_msg_t, msg_data.post_data_msg.data);
    q_width = q_width > sizeof(cmd_msg_t) ? q_width : sizeof(cmd_msg_t);
    q_max_user_payload = q_width - offsetof(cmd_msg_t, msg_data.post_data_msg.data);
    ESP_LOGI(TAG, "Create mqtt q, width %u, for payload %hu", q_width, data_size);
    msg_queue = xQueueCreate( MSG_QUEUE_SIZE, q_width);

    xTaskCreatePinnedToCore(&aws_iot_task, "mqtt_client_task", 9216, NULL, 5, NULL, 1);
}

