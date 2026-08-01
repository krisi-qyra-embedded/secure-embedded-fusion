/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include "ESP32_uart.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#include "ecdh.h"
#include "i2c_dma.h"
#include "iwdg.h"
#include <string.h>

extern crypto_ctx_t g_crypto_ctx;

extern void heartbeat_signal_esp32_uart_aes_gcm(void);

SemaphoreHandle_t esp32_uart_rx_sem = NULL;
SemaphoreHandle_t esp32_uart_tx_sem = NULL;

volatile ESP32_UART_TransferState_t esp32_uart_rx_cmplt =
    ESP32_UART_TRANSFER_IDLE;

volatile ESP32_UART_TransferState_t esp32_uart_tx_cmplt =
    ESP32_UART_TRANSFER_IDLE;

/* ========================================================================
 * Bare-metal encrypted STM32 <-> ESP32 runtime transaction.
 *
 * Forward request:
 *   nonce[12] || ciphertext[14] || tag[16] = 42 bytes
 *
 * Reverse ACK/NACK:
 *   nonce[12] || ciphertext[11] || tag[16] = 39 bytes
 *
 * USART2:
 *   debug logs through ESP32_LOG_PRINTF()
 *
 * USART6:
 *   PC6 = USART6_TX -> ESP32 RX
 *   PC7 = USART6_RX <- ESP32 TX
 *
 * The reverse packet is authenticated and decrypted before its
 * ACK/NACK header, sequence, reason or checksum is interpreted.
 * ======================================================================== */

#define TEST_MSG_TYPE_HEARTBEAT       0x10U
#define TEST_MSG_TYPE_ACK             0x20U
#define TEST_MSG_TYPE_NACK            0x21U
#define TEST_RESPONSE_PAYLOAD_LEN     1U
#define TEST_RESPONSE_FRAME_LEN       11U
#define ESP32_ACK_TIMEOUT_MS          1000U
#define ESP32_MAX_RETRIES             3U

#define ESP32_TEST_PERIOD_MS        1000U
#define ESP32_RX_POLL_TIME_MS       100U
#define ESP32_RX_MAX_BYTES          128U

#define TEST_MAGIC_0                0xA5U
#define TEST_MAGIC_1                0x5AU
#define TEST_PROTO_VERSION          0x01U

#define ESP32_UART_SENSOR_FAULT_MASK (SYSTEM_FAULT_MASK_BME680   |  SYSTEM_FAULT_MASK_MPU9250  |  SYSTEM_FAULT_MASK_MAX31865 |  SYSTEM_FAULT_MASK_PM25     |    SYSTEM_FAULT_MASK_DS18B20)

static uint32_t g_test_sequence = 0U;
static uint32_t g_tx_ok_count = 0U;
static uint32_t g_tx_fail_count = 0U;
static uint32_t g_rx_packet_count = 0U;
static uint32_t g_loop_count = 0U;

/* ---------------- private driver state ---------------- */

static volatile uint8_t g_initialized = 0U;

static volatile uint8_t g_rx_buffer[ESP32_UART_RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head = 0U;
static volatile uint16_t g_rx_tail = 0U;

static volatile uint8_t g_tx_buffer[ESP32_UART_TX_BUFFER_SIZE];
static volatile uint16_t g_tx_head = 0U;
static volatile uint16_t g_tx_tail = 0U;

static volatile uint8_t g_tx_busy = 0U;

static volatile uint16_t g_rx_wait_target = 0U;

static volatile ESP32_UART_Stats_t g_stats;

static uint8_t ESP32_UART_TestChecksum8(const uint8_t *data, uint16_t len);
static void ESP32_UART_TestPrintHex(const uint8_t *data, uint16_t len);
static uint16_t ESP32_UART_BuildTestFrame(uint8_t *buf,
                                          uint16_t buf_size,
                                          uint32_t seq);

typedef enum {
    TEST_NACK_REASON_NONE             = 0x00U,
    TEST_NACK_REASON_BAD_CHECKSUM     = 0x01U,
    TEST_NACK_REASON_BAD_MAGIC        = 0x02U,
    TEST_NACK_REASON_BAD_FORMAT       = 0x03U,
    TEST_NACK_REASON_GCM_AUTH         = 0x04U,
    TEST_NACK_REASON_INTERNAL_ERROR   = 0x05U
} test_nack_reason_t;

typedef enum {
    ESP32_LINK_RESPONSE_ACK = 0,
    ESP32_LINK_RESPONSE_NACK,
    ESP32_LINK_RESPONSE_TIMEOUT,
    ESP32_LINK_RESPONSE_INVALID
} ESP32_LinkResponse_t;

static uint32_t g_tx_attempt_count = 0U;
static uint32_t g_ack_count = 0U;
static uint32_t g_nack_count = 0U;
static uint32_t g_ack_timeout_count = 0U;
static uint32_t g_retry_count = 0U;
static uint32_t g_stale_response_count = 0U;
static uint32_t g_bad_response_count = 0U;

static ESP32_LinkResponse_t ESP32_UART_WaitForEncryptedResponse(
    uint32_t expected_sequence,
    uint32_t timeout_ms,
    uint8_t *nack_reason);

static bool ESP32_UART_LoadEncryptedResponse(
    crypto_ctx_t *ctx,
    const uint8_t *wire_packet,
    size_t wire_len);

static const char *ESP32_UART_NackReasonToString(
    uint8_t reason);

static ESP32_LinkResponse_t ESP32_UART_RunEncryptedAttempt(
    uint8_t *nack_reason);

static bool ESP32_UART_PeriodicLogAllowed(void);

/* Checks whether routine ESP32 link logging is allowed. */
static bool ESP32_UART_PeriodicLogAllowed(void)
{
    /* Suppress success logs while a sensor fault mask is active. */
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return true;
    }

    return
        ((system_fault_active_mask &
          ESP32_UART_SENSOR_FAULT_MASK) == 0U);
}

/* ---------------- private helpers ---------------- */

/* Advances an ESP32 UART ring-buffer index. */
static uint16_t rb_next(
    uint16_t index,
    uint16_t size)
{
    /* Wrap at the configured buffer size rather than allowing overflow. */
    index++;

    if (index >= size)
    {
        index = 0U;
    }

    return index;
}

/* Reads the ESP32 link timeout timebase. */
static uint32_t esp32_uart_now_ms(void)
{
    /* Use the HAL millisecond tick shared with UART waits. */
    return HAL_GetTick();
}

/* Provides a short ESP32 UART polling delay. */
static void esp32_uart_delay_1ms(void)
{
    /* Wait one HAL millisecond before retrying an unavailable ring slot. */
    HAL_Delay(1U);
}

/* Calculates elapsed time for an ESP32 UART wait. */
static uint32_t esp32_uart_elapsed_ms(uint32_t start_ms)
{
    /* Use unsigned subtraction across tick-counter wrap. */
    return esp32_uart_now_ms() - start_ms;
}

/* Protects ESP32 UART state from its interrupt handler. */
static uint32_t esp32_uart_enter_critical(void)
{
    /* Save PRIMASK before disabling interrupts. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/* Restores the prior ESP32 UART interrupt state. */
static void esp32_uart_exit_critical(uint32_t primask)
{
    /* Re-enable interrupts only when they were originally enabled. */
    if (primask == 0U) {
        __enable_irq();
    }
}

/* Calculates the USART6 baud divisor from APB2. */
static uint32_t esp32_uart_calc_brr(uint32_t baudrate)
{
    /* Use 9600 baud if the requested baud value is zero. */
    uint32_t pclk;
    uint32_t brr;

    /*
     * USART6 is on APB2.
     * Oversampling by 16 is used, so BRR = PCLK / baudrate rounded.
     */
    pclk = HAL_RCC_GetPCLK2Freq();

    if (baudrate == 0U) {
        baudrate = 9600U;
    }

    brr = (pclk + (baudrate / 2U)) / baudrate;

    return brr;
}

/* Routes the ESP32 link to USART6 PC6 and PC7. */
static void esp32_uart_gpio_init(void)
{
    /* Select AF8 and pull-ups for the transmit and receive pins. */
    /*
     * PC6 -> USART6_TX AF8
     * PC7 -> USART6_RX AF8
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    __DSB();

    GPIOC->MODER &= ~((3UL << (6U * 2U)) | (3UL << (7U * 2U)));
    GPIOC->MODER |=  ((2UL << (6U * 2U)) | (2UL << (7U * 2U)));

    GPIOC->OTYPER &= ~((1UL << 6U) | (1UL << 7U));

    GPIOC->OSPEEDR &= ~((3UL << (6U * 2U)) | (3UL << (7U * 2U)));
    GPIOC->OSPEEDR |=  ((2UL << (6U * 2U)) | (2UL << (7U * 2U)));

    GPIOC->PUPDR &= ~((3UL << (6U * 2U)) | (3UL << (7U * 2U)));
    GPIOC->PUPDR |=  ((1UL << (6U * 2U)) | (1UL << (7U * 2U)));

    GPIOC->AFR[0] &= ~((0xFUL << (6U * 4U)) | (0xFUL << (7U * 4U)));
    GPIOC->AFR[0] |=  ((8UL   << (6U * 4U)) | (8UL   << (7U * 4U)));
}

/* Starts draining the ESP32 transmit ring. */
static void esp32_uart_start_tx_interrupt(void)
{
    /* Enable TXE interrupts only when queued bytes remain. */
    if (g_tx_head != g_tx_tail) {
        g_tx_busy = 1U;
        USART6->CR1 |= USART_CR1_TXEIE;
    }
}

/* Counts buffered ESP32 receive bytes. */
static uint16_t esp32_uart_rx_available_locked(void)
{
    /* Account for head/tail wrap while the caller protects the indices. */
    if (g_rx_head >= g_rx_tail) {
        return (uint16_t)(g_rx_head - g_rx_tail);
    }

    return (uint16_t)(ESP32_UART_RX_BUFFER_SIZE - g_rx_tail + g_rx_head);
}

/* Calculates available ESP32 transmit-ring capacity. */
static uint16_t esp32_uart_tx_free_locked(void)
{
    /* Reserve one slot to distinguish an empty ring from a full ring. */
    uint16_t used;

    if (g_tx_head >= g_tx_tail) {
        used = (uint16_t)(g_tx_head - g_tx_tail);
    } else {
        used = (uint16_t)(ESP32_UART_TX_BUFFER_SIZE - g_tx_tail + g_tx_head);
    }

    return (uint16_t)((ESP32_UART_TX_BUFFER_SIZE - 1U) - used);
}

/* Checks whether ESP32 waits can use FreeRTOS services. */
static bool esp32_uart_scheduler_running(void)
{
    /* Require the scheduler to be running rather than merely initialized. */
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

/* Drains stale ESP32 UART semaphore tokens. */
static void esp32_uart_clear_sem_if_rtos(SemaphoreHandle_t sem)
{
    /* Take pending tokens without waiting when the scheduler is running. */
    if (esp32_uart_scheduler_running() && (sem != NULL))
    {
        while (xSemaphoreTake(sem, 0U) == pdTRUE)
        {
        }
    }
}

/* Signals ESP32 UART progress to a waiting task. */
static void esp32_uart_give_sem_from_isr(SemaphoreHandle_t sem)
{
    /* Yield at ISR exit if a higher-priority task was woken. */
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (sem != NULL)
    {
        (void)xSemaphoreGiveFromISR(
            sem,
            &higher_priority_task_woken);

        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

/* Starts ESP32 link fault capture from task context. */
static void esp32_uart_mark_fault_owner_task(void)
{
    /* Use the encrypted-link owner only after scheduler startup. */
    if (!esp32_uart_scheduler_running())
    {
        return;
    }

    system_fault_mark_in_progress(
        SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM);
}

/* Latches ESP32 link faults from interrupt context. */
static void esp32_uart_mark_fault_owner_from_isr(void)
{
    /* Use the ISR-safe fault marker while FreeRTOS is running. */
    if (!esp32_uart_scheduler_running())
    {
        return;
    }

    system_fault_mark_active_from_isr(
        SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM);
}

/* Waits for the required ESP32 receive byte count. */
static ESP32_UART_Status_t esp32_uart_wait_for_rx_target(
    uint16_t target,
    uint32_t timeout_ms)
{
    /* Limit semaphore waits to the remaining receive timeout. */
    const uint32_t start_ms = esp32_uart_now_ms();

    if (target == 0U)
    {
        return ESP32_UART_STATUS_INVALID_PARAM;
    }

    for (;;)
    {
        if (ESP32_UART_RxAvailable() >= target)
        {
            esp32_uart_rx_cmplt =
                ESP32_UART_TRANSFER_COMPLETE;

            return ESP32_UART_STATUS_OK;
        }

        if (esp32_uart_rx_cmplt ==
            ESP32_UART_TRANSFER_ERROR)
        {
            return ESP32_UART_STATUS_ERROR;
        }

        if ((timeout_ms == 0U) ||
            (esp32_uart_elapsed_ms(start_ms) >= timeout_ms))
        {
            return ESP32_UART_STATUS_TIMEOUT;
        }

        if (esp32_uart_scheduler_running() &&
            (esp32_uart_rx_sem != NULL))
        {
            const uint32_t elapsed =
                esp32_uart_elapsed_ms(start_ms);

            const uint32_t remaining_ms =
                (elapsed < timeout_ms) ?
                (timeout_ms - elapsed) : 0U;

            if (remaining_ms == 0U)
            {
                return ESP32_UART_STATUS_TIMEOUT;
            }

            (void)xSemaphoreTake(
                esp32_uart_rx_sem,
                pdMS_TO_TICKS(remaining_ms));
        }
        else
        {
            esp32_uart_delay_1ms();
        }
    }
}

/* ---------------- public API ---------------- */

/* Starts the interrupt-driven USART6 ESP32 link. */
ESP32_UART_Status_t ESP32_UART_Init(void)
{
    /* Clear stale status and data before enabling receive and error interrupts. */
    uint32_t dummy;

    if (g_initialized != 0U)
    {
        return ESP32_UART_STATUS_OK;
    }

    memset((void *)&g_stats, 0, sizeof(g_stats));

    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_busy = 0U;
    g_rx_wait_target = 0U;

    esp32_uart_rx_cmplt =
        ESP32_UART_TRANSFER_IDLE;

    esp32_uart_tx_cmplt =
        ESP32_UART_TRANSFER_IDLE;

    esp32_uart_gpio_init();

    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    __DSB();

    USART6->CR1 = 0U;
    USART6->CR2 = 0U;
    USART6->CR3 = 0U;

    dummy = USART6->SR;
    dummy = USART6->DR;
    (void)dummy;

    USART6->BRR =
        esp32_uart_calc_brr(
            ESP32_UART_BAUDRATE);

    USART6->CR1 =
        USART_CR1_TE |
        USART_CR1_RE |
        USART_CR1_RXNEIE;

    USART6->CR3 = USART_CR3_EIE;

    NVIC_SetPriority(
        USART6_IRQn,
        ESP32_UART_IRQ_PRIORITY);

    NVIC_ClearPendingIRQ(USART6_IRQn);
    NVIC_EnableIRQ(USART6_IRQn);

    USART6->CR1 |= USART_CR1_UE;

    g_initialized = 1U;

    return ESP32_UART_STATUS_OK;
}

/* Stops the USART6 ESP32 link. */
void ESP32_UART_DeInit(void)
{
    /* Disable the IRQ and UART before gating its peripheral clock. */
    NVIC_DisableIRQ(USART6_IRQn);

    USART6->CR1 = 0U;
    USART6->CR2 = 0U;
    USART6->CR3 = 0U;

    RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;

    g_initialized = 0U;
    g_tx_busy = 0U;
    g_rx_wait_target = 0U;

    esp32_uart_rx_cmplt =
        ESP32_UART_TRANSFER_IDLE;

    esp32_uart_tx_cmplt =
        ESP32_UART_TRANSFER_IDLE;
}

/* Queues ESP32 UART bytes within the supplied timeout. */
ESP32_UART_Status_t ESP32_UART_Write(
    const uint8_t *data,
    uint16_t len,
    uint32_t timeout_ms)
{
    /* Wait for ring space without modifying indices outside the critical section. */
    uint16_t written = 0U;
    const uint32_t start_ms =
        esp32_uart_now_ms();

    if ((data == NULL) || (len == 0U))
    {
        esp32_uart_mark_fault_owner_task();
        return ESP32_UART_STATUS_INVALID_PARAM;
    }

    if (g_initialized == 0U)
    {
        esp32_uart_mark_fault_owner_task();
        return ESP32_UART_STATUS_NOT_INITIALIZED;
    }

    esp32_uart_tx_cmplt =
        ESP32_UART_TRANSFER_IDLE;

    esp32_uart_clear_sem_if_rtos(
        esp32_uart_tx_sem);

    while (written < len)
    {
        uint32_t primask;
        uint16_t next_head;
        bool queued = false;

        primask =
            esp32_uart_enter_critical();

        next_head =
            rb_next(
                g_tx_head,
                ESP32_UART_TX_BUFFER_SIZE);

        if (next_head != g_tx_tail)
        {
            g_tx_buffer[g_tx_head] =
                data[written];

            g_tx_head = next_head;
            written++;
            queued = true;

            esp32_uart_start_tx_interrupt();
        }

        esp32_uart_exit_critical(primask);

        if (!queued)
        {
            if ((timeout_ms == 0U) ||
                (esp32_uart_elapsed_ms(start_ms) >= timeout_ms))
            {
                g_stats.tx_overflows++;
                esp32_uart_tx_cmplt =
                    ESP32_UART_TRANSFER_ERROR;

                esp32_uart_mark_fault_owner_task();

                return ESP32_UART_STATUS_TIMEOUT;
            }

            esp32_uart_delay_1ms();
        }
    }

    return ESP32_UART_STATUS_OK;
}

/* Waits until USART6 finishes the ESP32 transmission. */
ESP32_UART_Status_t ESP32_UART_WaitTxComplete(
    uint32_t timeout_ms)
{
    /* Distinguish transfer errors from an elapsed completion timeout. */
    const uint32_t start_ms =
        esp32_uart_now_ms();

    if (g_initialized == 0U)
    {
        esp32_uart_mark_fault_owner_task();
        return ESP32_UART_STATUS_NOT_INITIALIZED;
    }

    for (;;)
    {
        if (esp32_uart_tx_cmplt ==
            ESP32_UART_TRANSFER_COMPLETE)
        {
            return ESP32_UART_STATUS_OK;
        }

        if (esp32_uart_tx_cmplt ==
            ESP32_UART_TRANSFER_ERROR)
        {
            esp32_uart_mark_fault_owner_task();
            return ESP32_UART_STATUS_ERROR;
        }

        if ((timeout_ms == 0U) ||
            (esp32_uart_elapsed_ms(start_ms) >= timeout_ms))
        {
            esp32_uart_mark_fault_owner_task();
            return ESP32_UART_STATUS_TIMEOUT;
        }

        if (esp32_uart_scheduler_running() &&
            (esp32_uart_tx_sem != NULL))
        {
            const uint32_t elapsed =
                esp32_uart_elapsed_ms(start_ms);

            const uint32_t remaining_ms =
                (elapsed < timeout_ms) ?
                (timeout_ms - elapsed) : 0U;

            if (remaining_ms == 0U)
            {
                esp32_uart_mark_fault_owner_task();
                return ESP32_UART_STATUS_TIMEOUT;
            }

            (void)xSemaphoreTake(
                esp32_uart_tx_sem,
                pdMS_TO_TICKS(remaining_ms));
        }
        else
        {
            esp32_uart_delay_1ms();
        }
    }
}

/* Reads ESP32 UART bytes within the supplied timeout. */
ESP32_UART_Status_t ESP32_UART_Read(
    uint8_t *data,
    uint16_t len,
    uint32_t timeout_ms,
    uint16_t *bytes_read)
{
    /* Report the actual byte count when a partial receive times out. */
    ESP32_UART_Status_t wait_status;
    uint16_t count = 0U;
    uint32_t primask;

    if (bytes_read != NULL)
    {
        *bytes_read = 0U;
    }

    if ((data == NULL) || (len == 0U))
    {
        esp32_uart_mark_fault_owner_task();
        return ESP32_UART_STATUS_INVALID_PARAM;
    }

    if (g_initialized == 0U)
    {
        esp32_uart_mark_fault_owner_task();
        return ESP32_UART_STATUS_NOT_INITIALIZED;
    }

    esp32_uart_rx_cmplt =
        ESP32_UART_TRANSFER_IDLE;

    esp32_uart_clear_sem_if_rtos(
        esp32_uart_rx_sem);

    g_rx_wait_target = len;

    wait_status =
        esp32_uart_wait_for_rx_target(
            len,
            timeout_ms);

    primask =
        esp32_uart_enter_critical();

    while ((count < len) &&
           (g_rx_head != g_rx_tail))
    {
        data[count] =
            g_rx_buffer[g_rx_tail];

        g_rx_tail =
            rb_next(
                g_rx_tail,
                ESP32_UART_RX_BUFFER_SIZE);

        count++;
    }

    g_rx_wait_target = 0U;

    esp32_uart_exit_critical(primask);

    if (bytes_read != NULL)
    {
        *bytes_read = count;
    }

    if ((wait_status == ESP32_UART_STATUS_OK) &&
        (count == len))
    {
        return ESP32_UART_STATUS_OK;
    }

    esp32_uart_mark_fault_owner_task();

    return wait_status;
}

/* Reads one byte from the ESP32 receive ring. */
ESP32_UART_Status_t ESP32_UART_ReadByte(uint8_t *byte,
                                        uint32_t timeout_ms)
{
    /* Pass the timeout through to the buffered receive path. */
    uint16_t bytes_read = 0U;

    if (byte == NULL) {
        return ESP32_UART_STATUS_INVALID_PARAM;
    }

    return ESP32_UART_Read(byte, 1U, timeout_ms, &bytes_read);
}

/* Snapshots the ESP32 receive-ring occupancy. */
uint16_t ESP32_UART_RxAvailable(void)
{
    /* Protect the head and tail while counting buffered bytes. */
    uint16_t available;
    uint32_t primask;

    primask = esp32_uart_enter_critical();
    available = esp32_uart_rx_available_locked();
    esp32_uart_exit_critical(primask);

    return available;
}

/* Snapshots free space in the ESP32 transmit ring. */
uint16_t ESP32_UART_TxFreeSpace(void)
{
    /* Read both indices with interrupts masked. */
    uint16_t free_space;
    uint32_t primask;

    primask = esp32_uart_enter_critical();
    free_space = esp32_uart_tx_free_locked();
    esp32_uart_exit_critical(primask);

    return free_space;
}

/* Discards buffered ESP32 receive data. */
void ESP32_UART_FlushRx(void)
{
    /* Reset the receive indices while excluding the UART ISR. */
    uint32_t primask;

    primask =
        esp32_uart_enter_critical();

    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_rx_wait_target = 0U;

    esp32_uart_rx_cmplt =
        ESP32_UART_TRANSFER_IDLE;

    esp32_uart_exit_critical(primask);

    esp32_uart_clear_sem_if_rtos(
        esp32_uart_rx_sem);
}

/* Discards queued ESP32 transmit data. */
void ESP32_UART_FlushTx(void)
{
    /* Disable TXE and TC interrupts with the transmit state reset. */
    uint32_t primask;

    primask =
        esp32_uart_enter_critical();

    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_busy = 0U;

    esp32_uart_tx_cmplt =
        ESP32_UART_TRANSFER_IDLE;

    USART6->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

    esp32_uart_exit_critical(primask);

    esp32_uart_clear_sem_if_rtos(
        esp32_uart_tx_sem);
}

/* Checks for active or queued ESP32 transmission. */
bool ESP32_UART_IsTxBusy(void)
{
    /* Consider both the busy flag and pending ring bytes. */
    bool busy;
    uint32_t primask;

    primask = esp32_uart_enter_critical();

    busy = ((g_tx_busy != 0U) || (g_tx_head != g_tx_tail));

    esp32_uart_exit_critical(primask);

    return busy;
}

/* Copies the ESP32 UART diagnostic counters. */
void ESP32_UART_GetStats(ESP32_UART_Stats_t *stats)
{
    /* Snapshot the ISR-updated statistics inside the critical section. */
    uint32_t primask;

    if (stats == NULL) {
        return;
    }

    primask = esp32_uart_enter_critical();
    *stats = g_stats;
    esp32_uart_exit_critical(primask);
}

/* Resets the ESP32 UART diagnostic counters. */
void ESP32_UART_ClearStats(void)
{
    /* Exclude the UART ISR while clearing its shared statistics. */
    uint32_t primask;

    primask = esp32_uart_enter_critical();
    memset((void *)&g_stats, 0, sizeof(g_stats));
    esp32_uart_exit_critical(primask);
}

/* ------------------------------------------------------------
 * Public test init
 * ------------------------------------------------------------ */
/* Initializes the ESP32 transaction test state and UART. */
ESP32_UART_Status_t ESP32_UART_LinkTest_Init(void)
{
    /* Flush both rings and reset the link success and failure counters. */
    ESP32_UART_Status_t st;

    g_tx_attempt_count = 0U;
    g_ack_count = 0U;
    g_nack_count = 0U;
    g_ack_timeout_count = 0U;
    g_retry_count = 0U;
    g_stale_response_count = 0U;
    g_bad_response_count = 0U;

    st = ESP32_UART_Init();

    if (st != ESP32_UART_STATUS_OK)
    {
        ESP32_LOG_PRINTF(
            "[ERROR][LINK][INIT] ESP32_UART_Init failed, status=%d\r\n",
            (int)st);

        app_fault_delay_ms(20U);
        return st;
    }

    ESP32_LOG_PRINTF(
        "[INFO][LINK][INIT] uptime_ms=%lu USART6=%lu AES-GCM tx=42B rx=39B auth-before-parse\r\n",
        (unsigned long)HAL_GetTick(),
        (unsigned long)ESP32_UART_BAUDRATE);

    return ESP32_UART_STATUS_OK;
}

/* ------------------------------------------------------------
 * Execute one encrypted request/response attempt.
 * ------------------------------------------------------------ */
/* Sends one encrypted ESP32 request and awaits its response. */
static ESP32_LinkResponse_t ESP32_UART_RunEncryptedAttempt(
    uint8_t *nack_reason)
{
    /* Wait for physical TX completion before receiving the encrypted ACK or NACK. */
    uint8_t tx_frame[32];
    uint8_t nonce[AES_GCM_AEAD_IV_SIZE];

    uint8_t ciphertext_and_tag[
        ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE +
        AES_GCM_TAG_SIZE];

    uint8_t wire_packet[
        AES_GCM_AEAD_IV_SIZE +
        ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE +
        AES_GCM_TAG_SIZE];

    uint16_t tx_len;
    uint16_t wire_len;
    size_t encrypted_len = 0U;
    size_t ciphertext_len;
    gcm_sender_status_t crypto_status;
    ESP32_UART_Status_t uart_status;
    ESP32_LinkResponse_t response;

    if (nack_reason != NULL)
    {
        *nack_reason =
            TEST_NACK_REASON_NONE;
    }

    memset(tx_frame, 0, sizeof(tx_frame));
    memset(nonce, 0, sizeof(nonce));
    memset(
        ciphertext_and_tag,
        0,
        sizeof(ciphertext_and_tag));
    memset(wire_packet, 0, sizeof(wire_packet));

    tx_len =
        ESP32_UART_BuildTestFrame(
            tx_frame,
            sizeof(tx_frame),
            g_test_sequence);

    if (tx_len == 0U)
    {
        ESP32_LOG_PRINTF(
            "[ERROR][LINK][TX] Failed to build UART plaintext frame\r\n");
        app_fault_delay_ms(20U);

        return ESP32_LINK_RESPONSE_INVALID;
    }

#if 0 /* expert runtime logging: raw TX packet dumps disabled */
    ESP32_LOG_PRINTF(
        "[TX][PLAINTEXT] Original UART frame [%u bytes]: ",
        (unsigned)tx_len);

    ESP32_UART_TestPrintHex(
        tx_frame,
        tx_len);

    ESP32_LOG_PRINTF("\r\n");
#endif

    crypto_status =
        ecdh_encrypt_uart_payload_once(
            &g_crypto_ctx,
            tx_frame,
            tx_len,
            nonce,
            ciphertext_and_tag,
            sizeof(ciphertext_and_tag),
            &encrypted_len);

    if (crypto_status != GCM_SENDER_SUCCESS)
    {
        ESP32_LOG_PRINTF(
            "[ERROR][LINK][TX] UART AES-GCM encryption failed, rc=%d\r\n",
            (int)crypto_status);
        app_fault_delay_ms(20U);

        return ESP32_LINK_RESPONSE_INVALID;
    }

    if ((encrypted_len !=
            ((size_t)tx_len + AES_GCM_TAG_SIZE)) ||
        (encrypted_len > UINT16_MAX))
    {
        ESP32_LOG_PRINTF(
            "[ERROR][LINK][TX] Invalid encrypted size=%u\r\n",
            (unsigned)encrypted_len);
        app_fault_delay_ms(20U);

        return ESP32_LINK_RESPONSE_INVALID;
    }

    ciphertext_len =
        encrypted_len -
        AES_GCM_TAG_SIZE;

    memcpy(
        &wire_packet[0],
        nonce,
        AES_GCM_AEAD_IV_SIZE);

    memcpy(
        &wire_packet[AES_GCM_AEAD_IV_SIZE],
        ciphertext_and_tag,
        encrypted_len);

    wire_len =
        (uint16_t)(
            AES_GCM_AEAD_IV_SIZE +
            encrypted_len);

#if 0 /* expert runtime logging: raw TX packet dumps disabled */
    ESP32_LOG_PRINTF(
        "[TX][NONCE] [%u bytes]: ",
        (unsigned)AES_GCM_AEAD_IV_SIZE);

    ESP32_UART_TestPrintHex(
        nonce,
        AES_GCM_AEAD_IV_SIZE);

    ESP32_LOG_PRINTF("\r\n");

    ESP32_LOG_PRINTF(
        "[TX][CIPHERTEXT] [%u bytes]: ",
        (unsigned)ciphertext_len);

    ESP32_UART_TestPrintHex(
        ciphertext_and_tag,
        (uint16_t)ciphertext_len);

    ESP32_LOG_PRINTF("\r\n");

    ESP32_LOG_PRINTF(
        "[TX][TAG] [%u bytes]: ",
        (unsigned)AES_GCM_TAG_SIZE);

    ESP32_UART_TestPrintHex(
        &ciphertext_and_tag[ciphertext_len],
        AES_GCM_TAG_SIZE);

    ESP32_LOG_PRINTF("\r\n");

    ESP32_LOG_PRINTF(
        "[TX][WIRE] nonce || ciphertext || tag [%u bytes]: ",
        (unsigned)wire_len);

    ESP32_UART_TestPrintHex(
        wire_packet,
        wire_len);

    ESP32_LOG_PRINTF("\r\n");
#endif

    ESP32_UART_FlushRx();

    g_tx_attempt_count++;

    uart_status =
        ESP32_UART_Write(
            wire_packet,
            wire_len,
            200U);

    if (uart_status != ESP32_UART_STATUS_OK)
    {
        ESP32_LOG_PRINTF(
            "[ERROR][LINK][TX] USART6 queue failed, status=%d\r\n",
            (int)uart_status);
        app_fault_delay_ms(20U);

        return ESP32_LINK_RESPONSE_INVALID;
    }

    uart_status =
        ESP32_UART_WaitTxComplete(500U);

    if (uart_status != ESP32_UART_STATUS_OK)
    {
        ESP32_LOG_PRINTF(
            "[ERROR][LINK][TX] USART6 transmission completion failed, status=%d\r\n",
            (int)uart_status);
        app_fault_delay_ms(20U);

        ESP32_UART_FlushTx();
        return ESP32_LINK_RESPONSE_INVALID;
    }

    if (ESP32_UART_PeriodicLogAllowed())
    {
        ESP32_LOG_PRINTF(
            "[INFO][LINK][TX] uptime_ms=%lu seq=%lu payload=REDACTED | "
            "crypto=AES-256-GCM | wire=%uB | route=USART6->ESP32 | status=SENT\r\n",
            (unsigned long)HAL_GetTick(),
            (unsigned long)g_test_sequence,
            (unsigned)wire_len);
    }

    response =
        ESP32_UART_WaitForEncryptedResponse(
            g_test_sequence,
            ESP32_ACK_TIMEOUT_MS,
            nack_reason);

    memset(
        tx_frame,
        0,
        sizeof(tx_frame));

    memset(
        nonce,
        0,
        sizeof(nonce));

    memset(
        ciphertext_and_tag,
        0,
        sizeof(ciphertext_and_tag));

    memset(
        wire_packet,
        0,
        sizeof(wire_packet));

    return response;
}

/* ------------------------------------------------------------
 * FreeRTOS encrypted runtime cycle with a three-attempt recovery budget.
 * ------------------------------------------------------------ */
/* Runs one ESP32 request/response transaction. */
ESP32_UART_RuntimeStatus_t ESP32_UART_LinkTest_RunOnce(void)
{
    /* Advance the message sequence only after the expected ACK is accepted. */
    uint8_t nack_reason =
        TEST_NACK_REASON_NONE;

    for (uint32_t attempt = 1U;
         attempt <= ESP32_UART_RUNTIME_MAX_ATTEMPTS;
         ++attempt)
    {
        const ESP32_LinkResponse_t response =
            ESP32_UART_RunEncryptedAttempt(
                &nack_reason);

        g_loop_count++;

        if (response == ESP32_LINK_RESPONSE_ACK)
        {
            g_ack_count++;
            g_tx_ok_count++;
            g_test_sequence++;

            heartbeat_signal_esp32_uart_aes_gcm();

            system_fault_clear_active(
                SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM);

            /*
             * Per-packet logging is intentionally compact:
             * one TX summary + one authenticated RX/ACK summary.
             * Emit an aggregate health record after every 10 complete,
             * authenticated STM32->ESP32->STM32 transactions.
             */
            if (((g_tx_ok_count % 10U) == 0U) &&
                ESP32_UART_PeriodicLogAllowed())
            {
                ESP32_UART_LinkTest_PrintHealth();
            }

            return ESP32_UART_RUNTIME_OK;
        }

        esp32_uart_mark_fault_owner_task();

        g_tx_fail_count++;

        if (response == ESP32_LINK_RESPONSE_NACK)
        {
            g_nack_count++;

            ESP32_LOG_PRINTF(
                "[WARN][LINK][NACK] uptime_ms=%lu seq=%lu reason=%u (%s) attempt=%lu/%u\r\n",
                (unsigned long)HAL_GetTick(),
                (unsigned long)g_test_sequence,
                (unsigned)nack_reason,
                ESP32_UART_NackReasonToString(
                    nack_reason),
                (unsigned long)attempt,
                (unsigned)ESP32_UART_RUNTIME_MAX_ATTEMPTS);

            app_fault_delay_ms(20U);
        }
        else if (response == ESP32_LINK_RESPONSE_TIMEOUT)
        {
            g_ack_timeout_count++;

            ESP32_LOG_PRINTF(
                "[WARN][LINK][TIMEOUT] uptime_ms=%lu ack_timeout_ms=%lu attempt=%lu/%u\r\n",
                (unsigned long)HAL_GetTick(),
                (unsigned long)ESP32_ACK_TIMEOUT_MS,
                (unsigned long)attempt,
                (unsigned)ESP32_UART_RUNTIME_MAX_ATTEMPTS);

            app_fault_delay_ms(20U);
        }
        else
        {
            g_bad_response_count++;

            ESP32_LOG_PRINTF(
                "[ERROR][LINK][TXN] uptime_ms=%lu AES-GCM/UART transaction failed attempt=%lu/%u\r\n",
                (unsigned long)HAL_GetTick(),
                (unsigned long)attempt,
                (unsigned)ESP32_UART_RUNTIME_MAX_ATTEMPTS);

            app_fault_delay_ms(20U);
        }

        if (attempt <
            ESP32_UART_RUNTIME_MAX_ATTEMPTS)
        {
            g_retry_count++;

            ESP32_LOG_PRINTF(
                "[WARN][LINK][RETRY] uptime_ms=%lu next=%lu/%u\r\n",
                (unsigned long)HAL_GetTick(),
                (unsigned long)(attempt + 1U),
                (unsigned)ESP32_UART_RUNTIME_MAX_ATTEMPTS);

            ESP32_UART_FlushRx();
            ESP32_UART_FlushTx();

            app_fault_delay_ms(
                ESP32_UART_RUNTIME_RETRY_DELAY_MS);
        }
    }

    ESP32_LOG_PRINTF(
        "[ERROR][LINK][FATAL] uptime_ms=%lu recovery_budget_exhausted attempts=%u\r\n",
        (unsigned long)HAL_GetTick(),
        (unsigned)ESP32_UART_RUNTIME_MAX_ATTEMPTS);

    app_fault_delay_ms(20U);

    return ESP32_UART_RUNTIME_FATAL;
}

/* ------------------------------------------------------------
 * Public health logger
 * ------------------------------------------------------------ */
/* Reports ESP32 transaction and UART health counters. */
void ESP32_UART_LinkTest_PrintHealth(void)
{
    /* Snapshot the UART statistics before formatting the report. */
    ESP32_UART_Stats_t stats;

    memset(&stats, 0, sizeof(stats));
    ESP32_UART_GetStats(&stats);

    ESP32_LOG_PRINTF(
        "[INFO][LINK][HEALTH] uptime_ms=%lu cycles=%lu last_seq=%lu tx_ok=%lu rx_auth_ack=%lu "
        "fail=%lu nack=%lu timeout=%lu retry=%lu "
        "uart={ovf:%lu par:%lu frm:%lu ovr:%lu}\r\n",
        (unsigned long)HAL_GetTick(),
        (unsigned long)g_tx_ok_count,
        (unsigned long)((g_test_sequence == 0U) ? 0U : (g_test_sequence - 1U)),
        (unsigned long)g_tx_ok_count,
        (unsigned long)g_ack_count,
        (unsigned long)g_tx_fail_count,
        (unsigned long)g_nack_count,
        (unsigned long)g_ack_timeout_count,
        (unsigned long)g_retry_count,
        (unsigned long)(stats.rx_overflows + stats.tx_overflows),
        (unsigned long)stats.parity_errors,
        (unsigned long)stats.framing_errors,
        (unsigned long)stats.overrun_errors);
}

/* ------------------------------------------------------------
 * Public bare-metal delay
 * ------------------------------------------------------------ */
/* Waits between ESP32 link transactions. */
void ESP32_UART_LinkTest_DelayMs(uint32_t delay_ms)
{
    /* Use elapsed HAL ticks to measure the requested interval. */
    app_fault_delay_ms(delay_ms);
}

/* ------------------------------------------------------------
 * Test frame builder
 *
 * byte 0:  magic 0xA5
 * byte 1:  magic 0x5A
 * byte 2:  protocol version
 * byte 3:  message type
 * byte 4:  payload length
 * byte 5:  sequence byte 0
 * byte 6:  sequence byte 1
 * byte 7:  sequence byte 2
 * byte 8:  sequence byte 3
 * byte 9:  payload[0]
 * byte 10: payload[1]
 * byte 11: payload[2]
 * byte 12: payload[3]
 * byte 13: checksum8 over bytes 0..12
 * ------------------------------------------------------------ */
/* Builds the fixed ESP32 heartbeat test frame. */
static uint16_t ESP32_UART_BuildTestFrame(uint8_t *buf,
                                          uint16_t buf_size,
                                          uint32_t seq)
{
    /* Encode the sequence little-endian before appending the additive checksum. */
    uint16_t i = 0U;
    uint8_t checksum;

    if ((buf == NULL) || (buf_size < 14U))
    {
        return 0U;
    }

    buf[i++] = TEST_MAGIC_0;
    buf[i++] = TEST_MAGIC_1;
    buf[i++] = TEST_PROTO_VERSION;
    buf[i++] = TEST_MSG_TYPE_HEARTBEAT;

    /*
     * Payload length = 4 bytes.
     */
    buf[i++] = 4U;

    buf[i++] = (uint8_t)((seq >> 0U)  & 0xFFU);
    buf[i++] = (uint8_t)((seq >> 8U)  & 0xFFU);
    buf[i++] = (uint8_t)((seq >> 16U) & 0xFFU);
    buf[i++] = (uint8_t)((seq >> 24U) & 0xFFU);

    buf[i++] = 0x11U;
    buf[i++] = 0x22U;
    buf[i++] = 0x33U;
    buf[i++] = 0x44U;

    checksum = ESP32_UART_TestChecksum8(buf, i);
    buf[i++] = checksum;

    return i;
}

/* Logs an ESP32 packet as hexadecimal bytes. */
static void ESP32_UART_TestPrintHex(
    const uint8_t *data,
    uint16_t len)
{
    /* Skip a null packet before walking its bytes. */
    (void)data;

    if (len == 0U)
    {
        ESP32_LOG_PRINTF("<empty>");
        return;
    }

    ESP32_LOG_PRINTF("**********");
}

/* Calculates the ESP32 frame's additive byte checksum. */
static uint8_t ESP32_UART_TestChecksum8(const uint8_t *data, uint16_t len)
{
    /* Accumulate modulo 256 across the supplied frame bytes. */
    uint16_t i;
    uint8_t sum = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        sum = (uint8_t)(sum + data[i]);
    }

    return sum;
}

/* Receives and authenticates an encrypted ESP32 ACK or NACK. */
static ESP32_LinkResponse_t ESP32_UART_WaitForEncryptedResponse(
    uint32_t expected_sequence,
    uint32_t timeout_ms,
    uint8_t *nack_reason)
{
    /* Require the complete wire packet before loading the crypto context. */
    uint8_t wire_packet[
        GCM_UART_RESPONSE_WIRE_SIZE];

    gcm_uart_response_t decoded_response;
    gcm_receiver_status_t crypto_status;
    ESP32_UART_Status_t uart_status;
    uint16_t bytes_read = 0U;

    if (nack_reason != NULL)
    {
        *nack_reason =
            TEST_NACK_REASON_NONE;
    }

    memset(
        wire_packet,
        0,
        sizeof(wire_packet));

    memset(
        &decoded_response,
        0,
        sizeof(decoded_response));

    uart_status =
        ESP32_UART_Read(
            wire_packet,
            (uint16_t)sizeof(wire_packet),
            timeout_ms,
            &bytes_read);

    if ((uart_status != ESP32_UART_STATUS_OK) ||
        (bytes_read != sizeof(wire_packet)))
    {
        ESP32_LOG_PRINTF(
            "[WARN][LINK][RX_TIMEOUT] uptime_ms=%lu bytes=%u/%u status=%d\r\n",
            (unsigned long)HAL_GetTick(),
            (unsigned)bytes_read,
            (unsigned)sizeof(wire_packet),
            (int)uart_status);
        app_fault_delay_ms(20U);

        memset(
            wire_packet,
            0,
            sizeof(wire_packet));

        return ESP32_LINK_RESPONSE_TIMEOUT;
    }

    if (!ESP32_UART_LoadEncryptedResponse(
            &g_crypto_ctx,
            wire_packet,
            sizeof(wire_packet)))
    {
        memset(
            wire_packet,
            0,
            sizeof(wire_packet));

        return ESP32_LINK_RESPONSE_INVALID;
    }

#if 0 /* expert runtime logging: raw RX packet dumps disabled */

    ESP32_LOG_PRINTF(
        "\r\n"
        "========== ESP32 -> STM32 ENCRYPTED RESPONSE ==========\r\n");

    ESP32_LOG_PRINTF(
        "[RX][WIRE] nonce || ciphertext || tag [%u bytes]: ",
        (unsigned)sizeof(wire_packet));

    ESP32_UART_TestPrintHex(
        wire_packet,
        (uint16_t)sizeof(wire_packet));

    ESP32_LOG_PRINTF("\r\n");

    ESP32_LOG_PRINTF(
        "[RX][NONCE] [%u bytes]: ",
        (unsigned)AES_GCM_AEAD_IV_SIZE);

    ESP32_UART_TestPrintHex(
        g_crypto_ctx.IV_GCM_receiver,
        AES_GCM_AEAD_IV_SIZE);

    ESP32_LOG_PRINTF("\r\n");

    ESP32_LOG_PRINTF(
        "[RX][CIPHERTEXT] [%u bytes]: ",
        (unsigned)GCM_UART_RESPONSE_CIPHERTEXT_SIZE);

    ESP32_UART_TestPrintHex(
        g_crypto_ctx.auth.ciphertext_sender,
        GCM_UART_RESPONSE_CIPHERTEXT_SIZE);

    ESP32_LOG_PRINTF("\r\n");

    ESP32_LOG_PRINTF(
        "[RX][TAG] [%u bytes]: ",
        (unsigned)AES_GCM_TAG_SIZE);

    ESP32_UART_TestPrintHex(
        g_crypto_ctx.auth.tag_sender,
        AES_GCM_TAG_SIZE);

    ESP32_LOG_PRINTF("\r\n");

#endif /* ESP32_UART_DEBUG_PRINT_RX_ENCRYPTED_PACKET */

    crypto_status =
        decrypt_verify_recombine_gcm_receiver(
            &g_crypto_ctx,
            expected_sequence,
            &decoded_response);

    memset(
        wire_packet,
        0,
        sizeof(wire_packet));

    if (crypto_status != GCM_RECEIVER_SUCCESS)
    {
        ESP32_LOG_PRINTF(
            "[ERROR][LINK][RX] uptime_ms=%lu encrypted ACK/NACK processing failed rc=%d\r\n",
            (unsigned long)HAL_GetTick(),
            (int)crypto_status);
        app_fault_delay_ms(20U);

        return ESP32_LINK_RESPONSE_INVALID;
    }

    g_rx_packet_count++;

    if ((decoded_response.message_type ==
            GCM_UART_MSG_TYPE_ACK) &&
        (decoded_response.reason ==
            TEST_NACK_REASON_NONE))
    {
        if (ESP32_UART_PeriodicLogAllowed())
        {
            ESP32_LOG_PRINTF(
                "[INFO][LINK][RX] uptime_ms=%lu seq=%lu source=ESP32 | wire=%uB | "
                "auth=OK | decrypt=OK | ack=VALID\r\n",
                (unsigned long)HAL_GetTick(),
                (unsigned long)decoded_response.sequence,
                (unsigned)sizeof(wire_packet));
        }

        return ESP32_LINK_RESPONSE_ACK;
    }

    if (decoded_response.message_type ==
        GCM_UART_MSG_TYPE_NACK)
    {
        if (nack_reason != NULL)
        {
            *nack_reason =
                decoded_response.reason;
        }

        ESP32_LOG_PRINTF(
            "[WARN][LINK][RX_NACK] uptime_ms=%lu seq=%lu reason=%u (%s) auth=OK\r\n",
            (unsigned long)HAL_GetTick(),
            (unsigned long)decoded_response.sequence,
            (unsigned)decoded_response.reason,
            ESP32_UART_NackReasonToString(
                decoded_response.reason));

        return ESP32_LINK_RESPONSE_NACK;
    }

    return ESP32_LINK_RESPONSE_INVALID;
}

/* Loads the received nonce, ciphertext and tag into the crypto context. */
static bool ESP32_UART_LoadEncryptedResponse(
    crypto_ctx_t *ctx,
    const uint8_t *wire_packet,
    size_t wire_len)
{
    /* Clear prior receive state before copying the fixed packet fields. */
    const size_t nonce_offset = 0U;
    const size_t ciphertext_offset =
        AES_GCM_AEAD_IV_SIZE;
    const size_t tag_offset =
        AES_GCM_AEAD_IV_SIZE +
        GCM_UART_RESPONSE_CIPHERTEXT_SIZE;

    if ((ctx == NULL) ||
        (wire_packet == NULL) ||
        (wire_len != GCM_UART_RESPONSE_WIRE_SIZE) ||
        (sizeof(ctx->IV_GCM_receiver) !=
            AES_GCM_AEAD_IV_SIZE) ||
        (sizeof(ctx->auth.tag_sender) <
            AES_GCM_TAG_SIZE) ||
        (sizeof(ctx->auth.ciphertext_sender) <
            GCM_UART_RESPONSE_CIPHERTEXT_SIZE))
    {
        return false;
    }

    memset(
        ctx->IV_GCM_receiver,
        0,
        sizeof(ctx->IV_GCM_receiver));

    memset(
        ctx->auth.ciphertext_sender,
        0,
        sizeof(ctx->auth.ciphertext_sender));

    memset(
        ctx->auth.tag_sender,
        0,
        sizeof(ctx->auth.tag_sender));

    memset(
        ctx->buffers.decrypted_bytes,
        0,
        sizeof(ctx->buffers.decrypted_bytes));

    ctx->auth.ciphertext_len = 0U;
    ctx->auth.mac_len = 0U;
    ctx->tmp.computed_size = 0U;

    memcpy(
        ctx->IV_GCM_receiver,
        &wire_packet[nonce_offset],
        AES_GCM_AEAD_IV_SIZE);

    memcpy(
        ctx->auth.ciphertext_sender,
        &wire_packet[ciphertext_offset],
        GCM_UART_RESPONSE_CIPHERTEXT_SIZE);

    memcpy(
        ctx->auth.tag_sender,
        &wire_packet[tag_offset],
        AES_GCM_TAG_SIZE);

    ctx->auth.ciphertext_len =
        GCM_UART_RESPONSE_CIPHERTEXT_SIZE;

    ctx->auth.mac_len =
        AES_GCM_TAG_SIZE;

    return true;
}

/* Returns a diagnostic label for an ESP32 NACK reason. */
static const char *ESP32_UART_NackReasonToString(
    uint8_t reason)
{
    /* Use the unknown label when the reason is not recognized. */
    switch (reason)
    {
        case TEST_NACK_REASON_NONE:
            return "NONE";

        case TEST_NACK_REASON_BAD_CHECKSUM:
            return "BAD_CHECKSUM";

        case TEST_NACK_REASON_BAD_MAGIC:
            return "BAD_MAGIC";

        case TEST_NACK_REASON_BAD_FORMAT:
            return "BAD_FORMAT";

        case TEST_NACK_REASON_GCM_AUTH:
            return "GCM_AUTH_FAILURE";

        case TEST_NACK_REASON_INTERNAL_ERROR:
            return "INTERNAL_ERROR";

        default:
            return "UNKNOWN";
    }
}

/* ---------------- IRQ handler ---------------- */

/* Services USART6 receive, transmit and error events. */
void ESP32_UART_IRQHandler(void)
{
    /* Switch from TXE to TC interrupts after the transmit ring empties. */
    const uint32_t sr =
        USART6->SR;

    if ((sr &
        (USART_SR_PE |
         USART_SR_FE |
         USART_SR_NE |
         USART_SR_ORE)) != 0U)
    {
        volatile uint32_t dummy =
            USART6->DR;

        (void)dummy;

        if ((sr & USART_SR_PE) != 0U)
        {
            g_stats.parity_errors++;
        }

        if ((sr & USART_SR_FE) != 0U)
        {
            g_stats.framing_errors++;
        }

        if ((sr & USART_SR_NE) != 0U)
        {
            g_stats.noise_errors++;
        }

        if ((sr & USART_SR_ORE) != 0U)
        {
            g_stats.overrun_errors++;
        }

        esp32_uart_rx_cmplt =
            ESP32_UART_TRANSFER_ERROR;

        esp32_uart_tx_cmplt =
            ESP32_UART_TRANSFER_ERROR;

        g_tx_busy = 0U;

        esp32_uart_mark_fault_owner_from_isr();

        esp32_uart_give_sem_from_isr(
            esp32_uart_rx_sem);

        esp32_uart_give_sem_from_isr(
            esp32_uart_tx_sem);

        ESP32_UART_ErrorCallback(sr);
        return;
    }

    if ((sr & USART_SR_RXNE) != 0U)
    {
        const uint8_t byte =
            (uint8_t)(USART6->DR & 0xFFU);

        const uint16_t next_head =
            rb_next(
                g_rx_head,
                ESP32_UART_RX_BUFFER_SIZE);

        if (next_head != g_rx_tail)
        {
            g_rx_buffer[g_rx_head] = byte;
            g_rx_head = next_head;
            g_stats.rx_bytes++;

            if ((g_rx_wait_target != 0U) &&
                (esp32_uart_rx_available_locked() >=
                 g_rx_wait_target))
            {
                esp32_uart_rx_cmplt =
                    ESP32_UART_TRANSFER_COMPLETE;

                esp32_uart_give_sem_from_isr(
                    esp32_uart_rx_sem);
            }
        }
        else
        {
            g_stats.rx_overflows++;

            esp32_uart_rx_cmplt =
                ESP32_UART_TRANSFER_ERROR;

            esp32_uart_mark_fault_owner_from_isr();

            esp32_uart_give_sem_from_isr(
                esp32_uart_rx_sem);

            ESP32_UART_RxOverflowCallback();
        }
    }

    if (((sr & USART_SR_TXE) != 0U) &&
        ((USART6->CR1 & USART_CR1_TXEIE) != 0U))
    {
        if (g_tx_tail != g_tx_head)
        {
            USART6->DR =
                g_tx_buffer[g_tx_tail];

            g_tx_tail =
                rb_next(
                    g_tx_tail,
                    ESP32_UART_TX_BUFFER_SIZE);

            g_stats.tx_bytes++;
            g_tx_busy = 1U;
        }
        else
        {
            USART6->CR1 &= ~USART_CR1_TXEIE;

            USART6->CR1 |=
                USART_CR1_TCIE;
        }
    }

    if (((sr & USART_SR_TC) != 0U) &&
        ((USART6->CR1 & USART_CR1_TCIE) != 0U))
    {
        USART6->CR1 &= ~USART_CR1_TCIE;

        USART6->SR &= ~USART_SR_TC;

        g_tx_busy = 0U;

        esp32_uart_tx_cmplt =
            ESP32_UART_TRANSFER_COMPLETE;

        esp32_uart_give_sem_from_isr(
            esp32_uart_tx_sem);
    }
}

#if (ESP32_UART_PROVIDE_IRQ_HANDLER == 1U)
/* Dispatches the ESP32 USART6 interrupt. */
void USART6_IRQHandler(void)
{
    /* Keep ring-buffer servicing in the shared UART IRQ helper. */
    ESP32_UART_IRQHandler();
}
#endif

/* ---------------- weak callbacks ---------------- */

/* Provides the overridable ESP32 RX overflow hook. */
__attribute__((weak)) void ESP32_UART_RxOverflowCallback(void)
{
    /* The default weak hook leaves overflow handling to the driver counters. */
}

/* Provides the overridable ESP32 UART error hook. */
__attribute__((weak)) void ESP32_UART_ErrorCallback(uint32_t usart_sr)
{
    /* The default weak hook does not act on the status snapshot. */
    (void)usart_sr;

}
