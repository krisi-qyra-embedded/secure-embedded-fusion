/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "STM32_uart.h"

#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "driver/gpio.h"


static const char *TAG = "STM32_UART";

#define STM32_GCM_NONCE_SIZE              12U
#define STM32_GCM_PLAINTEXT_SIZE          14U
#define STM32_GCM_TAG_SIZE                16U
#define STM32_GCM_CIPHERTEXT_TAG_SIZE     \
    (STM32_GCM_PLAINTEXT_SIZE + STM32_GCM_TAG_SIZE)
#define STM32_GCM_WIRE_PACKET_SIZE        \
    (STM32_GCM_NONCE_SIZE + STM32_GCM_CIPHERTEXT_TAG_SIZE)
#define STM32_GCM_PACKET_QUEUE_LENGTH     4U

#define STM32_UART_RESYNC_QUIET_US        INT64_C(50000)
#define STM32_UART_READ_TIMEOUT_MS        20U


_Static_assert(
    STM32_GCM_NONCE_SIZE == 12U,
    "STM32 nonce must be 12 bytes");

_Static_assert(
    STM32_GCM_PLAINTEXT_SIZE == 14U,
    "STM32 ciphertext must be 14 bytes");

_Static_assert(
    STM32_GCM_TAG_SIZE == 16U,
    "STM32 GCM tag must be 16 bytes");

_Static_assert(
    STM32_GCM_CIPHERTEXT_TAG_SIZE == 30U,
    "STM32 ciphertext || tag must be 30 bytes");

_Static_assert(
    STM32_GCM_WIRE_PACKET_SIZE == 42U,
    "STM32 wire packet must be 42 bytes");

#if STM32_UART_TEST_MAX_FRAME_SIZE < STM32_GCM_WIRE_PACKET_SIZE
#error "STM32_UART_TEST_MAX_FRAME_SIZE must be at least 42 bytes"
#endif


typedef struct
{
    size_t len;
    uint8_t bytes[STM32_GCM_WIRE_PACKET_SIZE];

} STM32_UART_EncryptedPacket_t;

static QueueHandle_t g_stm32_uart_event_queue = NULL;
static QueueHandle_t g_encrypted_packet_queue = NULL;
static TaskHandle_t g_stm32_uart_event_task_handle = NULL;

static volatile bool g_stm32_uart_initialized = false;
static volatile bool g_stm32_uart_test_started = false;
static int64_t g_uart_resync_until_us = 0;
static STM32_UART_Stats_t g_stats;

static uint8_t g_encrypted_assembly[
    STM32_GCM_WIRE_PACKET_SIZE];

static size_t g_encrypted_assembly_count = 0U;
static uint32_t g_complete_encrypted_packets = 0U;
static uint32_t g_dropped_encrypted_packets = 0U;

static void STM32_UART_EventTask(void *arg);
static void STM32_UART_ConsumeEncryptedBytes(
    const uint8_t *data,
    size_t len);

/* Discards the partial encrypted packet assembly. */
static void STM32_UART_ResetEncryptedAssembly(void)
{
    g_encrypted_assembly_count = 0U;

    /* Clear the staging buffer before collecting another fixed-size packet. */
    memset(
        g_encrypted_assembly,
        0,
        sizeof(g_encrypted_assembly));
}

/* Reports whether UART RX is in its discard interval. */
static bool STM32_UART_ResynchronizationActive(void)
{
    /* Compare the monotonic timer with the resynchronization deadline. */
    return esp_timer_get_time() <
           g_uart_resync_until_us;
}

/* Flushes UART RX and starts a short discard interval. */
static void STM32_UART_EnterResynchronization(void)
{
    g_uart_resync_until_us =
        esp_timer_get_time() +
        STM32_UART_RESYNC_QUIET_US;

    STM32_UART_ResetEncryptedAssembly();

    uart_flush_input(
        STM32_UART_PORT);

    if (g_stm32_uart_event_queue != NULL)
    {
        /* Discard queued UART events before assembling the next packet. */
        xQueueReset(
            g_stm32_uart_event_queue);
    }
}

/* Configures the STM32 UART pins, framing and driver queues. */
STM32_UART_Status_t STM32_UART_Init(void)
{
    esp_err_t err;

    if (g_stm32_uart_initialized)
    {
        return STM32_UART_STATUS_OK;
    }

    memset(
        &g_stats,
        0,
        sizeof(g_stats));

    memset(
        g_encrypted_assembly,
        0,
        sizeof(g_encrypted_assembly));

    g_encrypted_assembly_count = 0U;
    g_uart_resync_until_us = 0;

    const uart_config_t uart_config =
    {
        .baud_rate = STM32_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    err =
        uart_param_config(
            STM32_UART_PORT,
            &uart_config);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "uart_param_config failed: %s",
            esp_err_to_name(err));

        return STM32_UART_STATUS_ERROR;
    }

    err =
        uart_set_pin(
            STM32_UART_PORT,
            STM32_UART_TX_GPIO,
            STM32_UART_RX_GPIO,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "uart_set_pin failed: %s",
            esp_err_to_name(err));

        return STM32_UART_STATUS_ERROR;
    }

    err =
        gpio_pulldown_dis(
            STM32_UART_RX_GPIO);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "gpio_pulldown_dis failed: %s",
            esp_err_to_name(err));

        return STM32_UART_STATUS_ERROR;
    }

    /* Enable the RX pull-up before installing the UART driver. */
    err =
        gpio_pullup_en(
            STM32_UART_RX_GPIO);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "gpio_pullup_en failed: %s",
            esp_err_to_name(err));

        return STM32_UART_STATUS_ERROR;
    }

    err =
        uart_driver_install(
            STM32_UART_PORT,
            STM32_UART_RX_BUF_SIZE,
            STM32_UART_TX_BUF_SIZE,
            STM32_UART_EVENT_QUEUE_SIZE,
            &g_stm32_uart_event_queue,
            0);

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "uart_driver_install failed: %s",
            esp_err_to_name(err));

        return STM32_UART_STATUS_ERROR;
    }

    (void)uart_set_rx_timeout(
        STM32_UART_PORT,
        2U);

    uart_flush_input(
        STM32_UART_PORT);

    xQueueReset(
        g_stm32_uart_event_queue);

    g_stm32_uart_initialized = true;

    ESP_LOGI(
        TAG,
        "UART initialized: port=%d baud=%d TX_GPIO=%d RX_GPIO=%d",
        STM32_UART_PORT,
        STM32_UART_BAUDRATE,
        STM32_UART_TX_GPIO,
        STM32_UART_RX_GPIO);

    return STM32_UART_STATUS_OK;
}

/* Stops UART reception and releases its task and queues. */
void STM32_UART_DeInit(void)
{
    if (g_stm32_uart_test_started &&
        (g_stm32_uart_event_task_handle != NULL))
    {
        /* Delete the event task before removing the UART driver. */
        vTaskDelete(
            g_stm32_uart_event_task_handle);

        g_stm32_uart_event_task_handle = NULL;
        g_stm32_uart_test_started = false;
    }

    if (g_encrypted_packet_queue != NULL)
    {
        vQueueDelete(
            g_encrypted_packet_queue);

        g_encrypted_packet_queue = NULL;
    }

    if (g_stm32_uart_initialized)
    {
        uart_driver_delete(
            STM32_UART_PORT);

        g_stm32_uart_event_queue = NULL;
        g_stm32_uart_initialized = false;
    }

    STM32_UART_ResetEncryptedAssembly();
    g_uart_resync_until_us = 0;
}

/* Writes a buffer and waits for UART transmission to finish. */
STM32_UART_Status_t STM32_UART_Write(
    const uint8_t *data,
    size_t len,
    uint32_t timeout_ms)
{
    int written;

    if ((data == NULL) ||
        (len == 0U))
    {
        return STM32_UART_STATUS_INVALID_PARAM;
    }

    if (!g_stm32_uart_initialized)
    {
        return STM32_UART_STATUS_NOT_INITIALIZED;
    }

    written =
        uart_write_bytes(
            STM32_UART_PORT,
            (const char *)data,
            len);

    if (written < 0)
    {
        return STM32_UART_STATUS_ERROR;
    }

    if ((size_t)written != len)
    {
        return STM32_UART_STATUS_TIMEOUT;
    }

    /* Apply the caller's timeout while waiting for the TX FIFO to drain. */
    if (uart_wait_tx_done(
            STM32_UART_PORT,
            pdMS_TO_TICKS(timeout_ms)) != ESP_OK)
    {
        return STM32_UART_STATUS_TIMEOUT;
    }

    g_stats.tx_bytes +=
        (uint32_t)written;

    g_stats.tx_frames++;

    return STM32_UART_STATUS_OK;
}

/* Reads buffered UART bytes with a bounded wait. */
STM32_UART_Status_t STM32_UART_Read(
    uint8_t *data,
    size_t max_len,
    uint32_t timeout_ms,
    size_t *bytes_read)
{
    int len;

    if ((data == NULL) ||
        (max_len == 0U))
    {
        return STM32_UART_STATUS_INVALID_PARAM;
    }

    if (!g_stm32_uart_initialized)
    {
        return STM32_UART_STATUS_NOT_INITIALIZED;
    }

    if (bytes_read != NULL)
    {
        *bytes_read = 0U;
    }

    /* Convert the supplied timeout to FreeRTOS ticks for the driver read. */
    len =
        uart_read_bytes(
            STM32_UART_PORT,
            data,
            max_len,
            pdMS_TO_TICKS(timeout_ms));

    if (len < 0)
    {
        return STM32_UART_STATUS_ERROR;
    }

    if (len == 0)
    {
        return STM32_UART_STATUS_NO_DATA;
    }

    if (bytes_read != NULL)
    {
        *bytes_read =
            (size_t)len;
    }

    g_stats.rx_bytes +=
        (uint32_t)len;

    return STM32_UART_STATUS_OK;
}

/* Returns the number of buffered UART RX bytes. */
int STM32_UART_RxAvailable(void)
{
    size_t buffered = 0U;

    if (!g_stm32_uart_initialized)
    {
        return 0;
    }

    /* An uninitialized or failed driver query reports no available data. */
    if (uart_get_buffered_data_len(
            STM32_UART_PORT,
            &buffered) != ESP_OK)
    {
        return 0;
    }

    return (int)buffered;
}

/* Discards pending RX bytes and partial encrypted data. */
void STM32_UART_FlushRx(void)
{
    if (!g_stm32_uart_initialized)
    {
        return;
    }

    uart_flush_input(
        STM32_UART_PORT);

    if (g_stm32_uart_event_queue != NULL)
    {
        xQueueReset(
            g_stm32_uart_event_queue);
    }

    /* Reset the packet queue so stale candidates cannot reach GCM processing. */
    if (g_encrypted_packet_queue != NULL)
    {
        xQueueReset(
            g_encrypted_packet_queue);
    }

    STM32_UART_ResetEncryptedAssembly();
}

/* Copies the current UART transport counters. */
void STM32_UART_GetStats(
    STM32_UART_Stats_t *stats)
{
    /* Reject a null destination before copying the counter structure. */
    if (stats == NULL)
    {
        return;
    }

    memcpy(
        stats,
        &g_stats,
        sizeof(*stats));
}

/* Resets UART and encrypted packet counters. */
void STM32_UART_ClearStats(void)
{
    memset(
        &g_stats,
        0,
        sizeof(g_stats));

    /* Clear completed and dropped packet counts with the driver statistics. */
    g_complete_encrypted_packets = 0U;
    g_dropped_encrypted_packets = 0U;
}

/* Starts UART event processing and the encrypted packet queue. */
STM32_UART_Status_t STM32_UART_LinkTest_Start(void)
{
    STM32_UART_Status_t status;
    BaseType_t task_result;

    status =
        STM32_UART_Init();

    if (status != STM32_UART_STATUS_OK)
    {
        return status;
    }

    if (g_stm32_uart_test_started)
    {
        return STM32_UART_STATUS_OK;
    }

    if (g_encrypted_packet_queue == NULL)
    {
        /* Create the packet queue before launching the UART event task. */
        g_encrypted_packet_queue =
            xQueueCreate(
                STM32_GCM_PACKET_QUEUE_LENGTH,
                sizeof(STM32_UART_EncryptedPacket_t));

        if (g_encrypted_packet_queue == NULL)
        {
            ESP_LOGE(
                TAG,
                "Failed to create encrypted packet queue");

            return STM32_UART_STATUS_ERROR;
        }
    }

    STM32_UART_FlushRx();
    STM32_UART_ClearStats();

    ESP_LOGI(
        TAG,
        "============================================================");

    ESP_LOGI(
        TAG,
        "STM32 encrypted UART receiver starting first");

    ESP_LOGI(
        TAG,
        "ESP32 RX GPIO%d <- STM32 PC6 USART6_TX",
        STM32_UART_RX_GPIO);

    ESP_LOGI(
        TAG,
        "Wire format: nonce[12] || ciphertext[14] || tag[16]");

    ESP_LOGI(
        TAG,
        "Fixed startup-test packet size: %u bytes (12 + 14 + 16)",
        (unsigned int)STM32_GCM_WIRE_PACKET_SIZE);

    ESP_LOGI(
        TAG,
        "UART event task will monitor incoming bytes continuously");

    ESP_LOGI(
        TAG,
        "============================================================");

    task_result =
        xTaskCreate(
            STM32_UART_EventTask,
            "stm32_uart_evt",
            STM32_UART_EVENT_TASK_STACK,
            NULL,
            STM32_UART_EVENT_TASK_PRIORITY,
            &g_stm32_uart_event_task_handle);

    if (task_result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create STM32 UART event task");

        return STM32_UART_STATUS_ERROR;
    }

    g_stm32_uart_test_started = true;

    return STM32_UART_STATUS_OK;
}

/* Waits for a complete encrypted packet from the RX queue. */
bool STM32_UART_ReceiveEncryptedPacket(
    uint8_t *packet,
    size_t packet_capacity,
    size_t *packet_len,
    uint32_t timeout_ms)
{
    STM32_UART_EncryptedPacket_t queued_packet;

    if (packet_len != NULL)
    {
        *packet_len = 0U;
    }

    if ((packet == NULL) ||
        (packet_len == NULL) ||
        (packet_capacity < STM32_GCM_WIRE_PACKET_SIZE) ||
        (g_encrypted_packet_queue == NULL))
    {
        return false;
    }

    memset(
        &queued_packet,
        0,
        sizeof(queued_packet));

    if (xQueueReceive(
            g_encrypted_packet_queue,
            &queued_packet,
            pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return false;
    }

    /* Verify the queued length fits the caller's buffer before copying it. */
    if ((queued_packet.len != STM32_GCM_WIRE_PACKET_SIZE) ||
        (queued_packet.len > packet_capacity))
    {
        return false;
    }

    memcpy(
        packet,
        queued_packet.bytes,
        queued_packet.len);

    *packet_len =
        queued_packet.len;

    return true;
}

/* Logs UART traffic, assembly progress and receive errors. */
void STM32_UART_LinkTest_PrintHealth(void)
{
    STM32_UART_Stats_t stats;

    STM32_UART_GetStats(
        &stats);

    ESP_LOGI(
        TAG,
        "--------------- ENCRYPTED UART HEALTH ---------------");

    ESP_LOGI(
        TAG,
        "rx_bytes=%" PRIu32,
        stats.rx_bytes);

    ESP_LOGI(
        TAG,
        "rx_frames=%" PRIu32,
        stats.rx_frames);

    ESP_LOGI(
        TAG,
        "complete_encrypted_packets=%" PRIu32,
        g_complete_encrypted_packets);

    ESP_LOGI(
        TAG,
        "dropped_encrypted_packets=%" PRIu32,
        g_dropped_encrypted_packets);

    /* Include partial assembly size to expose stalled or misaligned frames. */
    ESP_LOGI(
        TAG,
        "assembly_bytes=%u/%u",
        (unsigned int)g_encrypted_assembly_count,
        (unsigned int)STM32_GCM_WIRE_PACKET_SIZE);

    ESP_LOGI(
        TAG,
        "fifo_overflows=%" PRIu32,
        stats.fifo_overflows);

    ESP_LOGI(
        TAG,
        "buffer_full=%" PRIu32,
        stats.buffer_full);

    ESP_LOGI(
        TAG,
        "parity_errors=%" PRIu32,
        stats.parity_errors);

    ESP_LOGI(
        TAG,
        "frame_errors=%" PRIu32,
        stats.frame_errors);

    ESP_LOGI(
        TAG,
        "rx_available=%d",
        STM32_UART_RxAvailable());

    ESP_LOGI(
        TAG,
        "-----------------------------------------------------");
}

/* Consumes UART events and feeds the encrypted packet assembler. */
static void STM32_UART_EventTask(void *arg)
{
    (void)arg;

    uart_event_t event;
    uint8_t rx_data[STM32_UART_TEST_MAX_FRAME_SIZE];
    uint32_t health_counter = 0U;

    for (;;)
    {
        if (xQueueReceive(
                g_stm32_uart_event_queue,
                &event,
                pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            switch (event.type)
            {
                case UART_DATA:
                {
                    size_t remaining =
                        event.size;

                    while (remaining > 0U)
                    {
                        size_t chunk_size =
                            remaining;

                        size_t bytes_read = 0U;
                        STM32_UART_Status_t status;

                        if (chunk_size > sizeof(rx_data))
                        {
                            chunk_size =
                                sizeof(rx_data);
                        }

                        memset(
                            rx_data,
                            0,
                            sizeof(rx_data));

                        status =
                            STM32_UART_Read(
                                rx_data,
                                chunk_size,
                                STM32_UART_READ_TIMEOUT_MS,
                                &bytes_read);

                        if ((status != STM32_UART_STATUS_OK) ||
                            (bytes_read == 0U))
                        {
                            break;
                        }

                        if (bytes_read >= remaining)
                        {
                            remaining = 0U;
                        }
                        else
                        {
                            remaining -= bytes_read;
                        }

                        if (STM32_UART_ResynchronizationActive())
                        {
                            ESP_LOGW(
                                TAG,
                                "Discarding %u UART bytes during "
                                "resynchronization",
                                (unsigned int)bytes_read);

                            continue;
                        }

                        STM32_UART_ConsumeEncryptedBytes(
                            rx_data,
                            bytes_read);
                    }

                    break;
                }

                /* Flush partial data after FIFO, parity or framing errors. */
                case UART_FIFO_OVF:
    				g_stats.fifo_overflows++;

    				ESP_LOGW(
        			TAG,
        			"UART FIFO overflow; discarding partial packet");

    				STM32_UART_EnterResynchronization();
    				break;

                case UART_BUFFER_FULL:
    				g_stats.buffer_full++;

    				ESP_LOGW(
        			TAG,
        			"UART buffer full; discarding partial packet");

    				STM32_UART_EnterResynchronization();
    				break;

                case UART_PARITY_ERR:
    				g_stats.parity_errors++;

    				ESP_LOGW(
        			TAG,
        			"UART parity error; discarding partial packet");

    				STM32_UART_EnterResynchronization();
    				break;

                case UART_FRAME_ERR:
    				g_stats.frame_errors++;

    				ESP_LOGW(
        			TAG,
        			"UART frame error; discarding partial packet");

    				STM32_UART_EnterResynchronization();
    				break;

                case UART_BREAK:
    				ESP_LOGW(
        			TAG,
        			"UART break detected; discarding partial packet");

    				STM32_UART_EnterResynchronization();
    				break;

                case UART_PATTERN_DET:
                    g_stats.pattern_detected++;
                    break;

                default:
                    g_stats.unknown_events++;
                    break;
            }
        }

        health_counter++;

        if ((health_counter % 10U) == 0U)
        {
            STM32_UART_LinkTest_PrintHealth();
        }
    }
}

/* Assembles fixed-size encrypted candidates from UART chunks. */
static void STM32_UART_ConsumeEncryptedBytes(
    const uint8_t *data,
    size_t len)
{
    if ((data == NULL) ||
        (len == 0U))
    {
        return;
    }

    for (size_t i = 0U; i < len; ++i)
    {
        g_encrypted_assembly[
            g_encrypted_assembly_count++] =
                data[i];

        /* Queue a candidate only after all required wire bytes are collected. */
        if (g_encrypted_assembly_count ==
            STM32_GCM_WIRE_PACKET_SIZE)
        {
            STM32_UART_EncryptedPacket_t packet;

            memset(
                &packet,
                0,
                sizeof(packet));

            packet.len =
                STM32_GCM_WIRE_PACKET_SIZE;

            memcpy(
                packet.bytes,
                g_encrypted_assembly,
                STM32_GCM_WIRE_PACKET_SIZE);

            if (xQueueSend(
                    g_encrypted_packet_queue,
                    &packet,
                    0U) != pdTRUE)
            {
                STM32_UART_EncryptedPacket_t discarded;

                (void)xQueueReceive(
                    g_encrypted_packet_queue,
                    &discarded,
                    0U);

                if (xQueueSend(
                        g_encrypted_packet_queue,
                        &packet,
                        0U) != pdTRUE)
                {
                    g_dropped_encrypted_packets++;
                }
            }

            g_complete_encrypted_packets++;
            g_stats.rx_frames++;

            ESP_LOGI(
                TAG,
                "42-byte encrypted packet candidate queued; awaiting AES-GCM authentication");

            STM32_UART_ResetEncryptedAssembly();
        }
    }
}

