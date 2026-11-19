/**
 * @file ESP32_uart.c
 * @brief USART6 link between STM32 and ESP32.
 *
 * Handles UART buffering, encrypted request/response traffic and link recovery.
 */
#include "ESP32_uart.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "telemetry.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#include "ecdh.h"
#include "i2c_dma.h"
#include "iwdg.h"
#include <string.h>

/* Crypto context is initialized in main.c. */
extern crypto_ctx_t g_crypto_ctx;

/* ESP32 link heartbeat from iwdg.c. */
extern void heartbeat_signal_esp32_uart_aes_gcm(void);

/* UART completion state and semaphores. */
SemaphoreHandle_t esp32_uart_rx_sem = NULL;
SemaphoreHandle_t esp32_uart_tx_sem = NULL;
volatile ESP32_UART_TransferState_t esp32_uart_rx_cmplt =
    ESP32_UART_TRANSFER_IDLE;
volatile ESP32_UART_TransferState_t esp32_uart_tx_cmplt =
    ESP32_UART_TRANSFER_IDLE;

/*
 * Encrypted UART packet layout.
 * TX: nonce[12] + ciphertext + tag[16]
 * RX: nonce[12] + ciphertext + tag[16]
 * USART6: PC6 TX, PC7 RX.
 */
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
#define TEST_MSG_TYPE_TELEMETRY     0x11U

/* Hide periodic success logs while a sensor fault is active. */
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

/* RX byte count expected by the current blocking read. */
static volatile uint16_t g_rx_wait_target = 0U;
static volatile ESP32_UART_Stats_t g_stats;
static uint8_t ESP32_UART_TestChecksum8(const uint8_t *data, uint16_t len);
static void ESP32_UART_TestPrintHex(const uint8_t *data, uint16_t len);
static uint16_t ESP32_UART_BuildTestFrame(uint8_t *buf,
                                          uint16_t buf_size,
                                          uint32_t seq);
static uint16_t ESP32_UART_BuildTelemetryFrame(
    uint8_t *buf,
    uint16_t buf_size,
    uint32_t seq,
    const telemetry_snapshot_t *t);
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
static ESP32_LinkResponse_t
ESP32_UART_RunEncryptedAttempt(
    const telemetry_snapshot_t *snapshot,
    uint8_t *nack_reason);
static bool ESP32_UART_PeriodicLogAllowed(void);

/* Check if periodic link logs can be printed. */
static bool ESP32_UART_PeriodicLogAllowed(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return true;
    }
    return
        ((system_fault_active_mask &
          ESP32_UART_SENSOR_FAULT_MASK) == 0U);
}

/* Get next ring-buffer index. */
static uint16_t rb_next(
    uint16_t index,
    uint16_t size)
{
    index++;
    if (index >= size)
    {
        index = 0U;
    }
    return index;
}

/* Current HAL tick in ms. */
static uint32_t esp32_uart_now_ms(void)
{
    return HAL_GetTick();
}

/* Small polling delay. */
static void esp32_uart_delay_1ms(void)
{
    HAL_Delay(1U);
}

/* Elapsed time from start tick. */
static uint32_t esp32_uart_elapsed_ms(uint32_t start_ms)
{
    return esp32_uart_now_ms() - start_ms;
}

/* Enter short IRQ-protected section. */
static uint32_t esp32_uart_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/* Restore IRQ state after critical section. */
static void esp32_uart_exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* Calculate USART6 BRR from APB2 clock. */
static uint32_t esp32_uart_calc_brr(uint32_t baudrate)
{
    uint32_t pclk;
    uint32_t brr;
    /* USART6 uses APB2 clock. */
    pclk = HAL_RCC_GetPCLK2Freq();
    if (baudrate == 0U) {
        baudrate = 9600U;
    }
    brr = (pclk + (baudrate / 2U)) / baudrate;
    return brr;
}

/* Configure PC6/PC7 for USART6. */
static void esp32_uart_gpio_init(void)
{
    /* PC6 TX, PC7 RX, both AF8. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    __DSB();
    /* Alternate function mode */
    GPIOC->MODER &= ~((3UL << (6U * 2U)) | (3UL << (7U * 2U)));
    GPIOC->MODER |=  ((2UL << (6U * 2U)) | (2UL << (7U * 2U)));
    /* Push-pull */
    GPIOC->OTYPER &= ~((1UL << 6U) | (1UL << 7U));
    /* High speed */
    GPIOC->OSPEEDR &= ~((3UL << (6U * 2U)) | (3UL << (7U * 2U)));
    GPIOC->OSPEEDR |=  ((2UL << (6U * 2U)) | (2UL << (7U * 2U)));
    /* Pull-up keeps RX stable if ESP32 is reset or disconnected. */
    GPIOC->PUPDR &= ~((3UL << (6U * 2U)) | (3UL << (7U * 2U)));
    GPIOC->PUPDR |=  ((1UL << (6U * 2U)) | (1UL << (7U * 2U)));
    /* AFRL: pins 0..7. PC6/PC7 use AF8 for USART6. */
    GPIOC->AFR[0] &= ~((0xFUL << (6U * 4U)) | (0xFUL << (7U * 4U)));
    GPIOC->AFR[0] |=  ((8UL   << (6U * 4U)) | (8UL   << (7U * 4U)));
}

/* Start TX interrupt when data is queued. */
static void esp32_uart_start_tx_interrupt(void)
{
    if (g_tx_head != g_tx_tail) {
        g_tx_busy = 1U;
        USART6->CR1 |= USART_CR1_TXEIE;
    }
}

/* RX bytes available. Caller holds critical section. */
static uint16_t esp32_uart_rx_available_locked(void)
{
    if (g_rx_head >= g_rx_tail) {
        return (uint16_t)(g_rx_head - g_rx_tail);
    }
    return (uint16_t)(ESP32_UART_RX_BUFFER_SIZE - g_rx_tail + g_rx_head);
}

/* TX free space. Caller holds critical section. */
static uint16_t esp32_uart_tx_free_locked(void)
{
    uint16_t used;
    if (g_tx_head >= g_tx_tail) {
        used = (uint16_t)(g_tx_head - g_tx_tail);
    } else {
        used = (uint16_t)(ESP32_UART_TX_BUFFER_SIZE - g_tx_tail + g_tx_head);
    }
    /* Keep one slot empty to separate full from empty. */
    return (uint16_t)((ESP32_UART_TX_BUFFER_SIZE - 1U) - used);
}

/* Check if FreeRTOS scheduler is running. */
static bool esp32_uart_scheduler_running(void)
{
    return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

/* Clear old semaphore tokens before a new transfer. */
static void esp32_uart_clear_sem_if_rtos(SemaphoreHandle_t sem)
{
    if (esp32_uart_scheduler_running() && (sem != NULL))
    {
        while (xSemaphoreTake(sem, 0U) == pdTRUE)
        {
            /* Clear stale completion token. */
        }
    }
}

/* Signal transfer completion from IRQ. */
static void esp32_uart_give_sem_from_isr(SemaphoreHandle_t sem)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (sem != NULL)
    {
        (void)xSemaphoreGiveFromISR(
            sem,
            &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

/* Mark UART/AES-GCM fault from task context. */
static void esp32_uart_mark_fault_owner_task(void)
{
    if (!esp32_uart_scheduler_running())
    {
        return;
    }
    system_fault_mark_in_progress(
        SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM);
}

/* Mark UART/AES-GCM fault from IRQ context. */
static void esp32_uart_mark_fault_owner_from_isr(void)
{
    if (!esp32_uart_scheduler_running())
    {
        return;
    }
    system_fault_mark_active_from_isr(
        SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM);
}

/* Wait until the requested RX byte count is available. */
static ESP32_UART_Status_t esp32_uart_wait_for_rx_target(
    uint16_t target,
    uint32_t timeout_ms)
{
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

/**
 * @brief Initialize USART6 and the UART driver state.
 *
 * @return ESP32 UART status.
 */
ESP32_UART_Status_t ESP32_UART_Init(void)
{
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

/**
 * @brief Disable USART6 and reset driver state.
 */
void ESP32_UART_DeInit(void)
{
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

/**
 * @brief Queue bytes for USART6 transmission.
 *
 * @param data Data to send.
 * @param len Number of bytes.
 * @param timeout_ms Write timeout in ms.
 * @return ESP32 UART status.
 */
ESP32_UART_Status_t ESP32_UART_Write(
    const uint8_t *data,
    uint16_t len,
    uint32_t timeout_ms)
{
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

/**
 * @brief Wait for the current USART6 transfer to finish.
 *
 * @param timeout_ms Timeout in ms.
 * @return ESP32 UART status.
 */
ESP32_UART_Status_t ESP32_UART_WaitTxComplete(
    uint32_t timeout_ms)
{
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

/**
 * @brief Read bytes from the USART6 RX ring buffer.
 *
 * @param data Output buffer.
 * @param len Number of bytes requested.
 * @param timeout_ms Read timeout in ms.
 * @param bytes_read Number of bytes returned.
 * @return ESP32 UART status.
 */
ESP32_UART_Status_t ESP32_UART_Read(
    uint8_t *data,
    uint16_t len,
    uint32_t timeout_ms,
    uint16_t *bytes_read)
{
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
    /* Return partial RX data on timeout for diagnostics. */
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

/**
 * @brief Read one byte from USART6.
 *
 * @param byte Output byte.
 * @param timeout_ms Read timeout in ms.
 * @return ESP32 UART status.
 */
ESP32_UART_Status_t ESP32_UART_ReadByte(uint8_t *byte,
                                        uint32_t timeout_ms)
{
    uint16_t bytes_read = 0U;
    if (byte == NULL) {
        return ESP32_UART_STATUS_INVALID_PARAM;
    }
    return ESP32_UART_Read(byte, 1U, timeout_ms, &bytes_read);
}

/**
 * @brief Get number of bytes waiting in the RX buffer.
 *
 * @return Available RX bytes.
 */
uint16_t ESP32_UART_RxAvailable(void)
{
    uint16_t available;
    uint32_t primask;
    primask = esp32_uart_enter_critical();
    available = esp32_uart_rx_available_locked();
    esp32_uart_exit_critical(primask);
    return available;
}

/**
 * @brief Get free space in the TX buffer.
 *
 * @return Free TX bytes.
 */
uint16_t ESP32_UART_TxFreeSpace(void)
{
    uint16_t free_space;
    uint32_t primask;
    primask = esp32_uart_enter_critical();
    free_space = esp32_uart_tx_free_locked();
    esp32_uart_exit_critical(primask);
    return free_space;
}

/**
 * @brief Clear the RX ring buffer.
 */
void ESP32_UART_FlushRx(void)
{
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

/**
 * @brief Clear the TX ring buffer.
 */
void ESP32_UART_FlushTx(void)
{
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

/**
 * @brief Check if USART6 still has data to send.
 *
 * @return true while TX is busy.
 */
bool ESP32_UART_IsTxBusy(void)
{
    bool busy;
    uint32_t primask;
    primask = esp32_uart_enter_critical();
    busy = ((g_tx_busy != 0U) || (g_tx_head != g_tx_tail));
    esp32_uart_exit_critical(primask);
    return busy;
}

/**
 * @brief Copy current UART statistics.
 *
 * @param stats Output statistics structure.
 */
void ESP32_UART_GetStats(ESP32_UART_Stats_t *stats)
{
    uint32_t primask;
    if (stats == NULL) {
        return;
    }
    primask = esp32_uart_enter_critical();
    *stats = g_stats;
    esp32_uart_exit_critical(primask);
}

/**
 * @brief Clear UART statistics.
 */
void ESP32_UART_ClearStats(void)
{
    uint32_t primask;
    primask = esp32_uart_enter_critical();
    memset((void *)&g_stats, 0, sizeof(g_stats));
    esp32_uart_exit_critical(primask);
}

/**
 * @brief Initialize the encrypted STM32-ESP32 link test.
 *
 * @return ESP32 UART status.
 */
ESP32_UART_Status_t ESP32_UART_LinkTest_Init(void)
{
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

/**
 * @brief Run one encrypted request/response attempt.
 *
 * @param snapshot Telemetry snapshot to send.
 * @param nack_reason NACK reason when returned by ESP32.
 * @return Link response.
 */
static ESP32_LinkResponse_t
ESP32_UART_RunEncryptedAttempt(
    const telemetry_snapshot_t *snapshot,
    uint8_t *nack_reason)
{
	uint8_t tx_frame[STM32_TELEMETRY_FRAME_LEN];
	uint8_t nonce[AES_GCM_AEAD_IV_SIZE];
	uint8_t ciphertext_and_tag[
	    STM32_TELEMETRY_FRAME_LEN +
	    AES_GCM_TAG_SIZE];
	uint8_t wire_packet[
	    AES_GCM_AEAD_IV_SIZE +
	    STM32_TELEMETRY_FRAME_LEN +
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
        ESP32_UART_BuildTelemetryFrame(
            tx_frame,
            sizeof(tx_frame),
            g_test_sequence,
            snapshot);
    if (tx_len == 0U)
    {
        ESP32_LOG_PRINTF(
            "[ERROR][LINK][TX] Failed to build UART plaintext frame\r\n");
        app_fault_delay_ms(20U);
        return ESP32_LINK_RESPONSE_INVALID;
    }
#if 0 /* raw TX packet debug */
    ESP32_LOG_PRINTF(
        "[TX][PLAINTEXT] Original UART frame [%u bytes]: ",
        (unsigned)tx_len);
    ESP32_UART_TestPrintHex(
        tx_frame,
        tx_len);
    ESP32_LOG_PRINTF("\r\n");
#endif
    /* Retry gets a fresh nonce. Sequence changes only after ACK. */
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
#if 0 /* raw TX packet debug */
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
    //Fault injection test, integrity corruption.
//    for(int i = AES_GCM_AEAD_IV_SIZE;i < AES_GCM_AEAD_IV_SIZE + STM32_TELEMETRY_FRAME_LEN;i++)
//    {
//    wire_packet[i] ^= 0x01U;/*replace the ciphertext with corrupted val*/
//    }
    //Fault injection test, authenticity(tag) corruption.
//    for(int i = AES_GCM_AEAD_IV_SIZE + STM32_TELEMETRY_FRAME_LEN;i < AES_GCM_AEAD_IV_SIZE + STM32_TELEMETRY_FRAME_LEN + AES_GCM_TAG_SIZE;i++)
//    {
//    wire_packet[i] ^= 0x01U;/*replace the ciphertext with corrupted val*/
//    }
    uart_status =
        ESP32_UART_Write(
            wire_packet,
            wire_len,
            500U);
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

/**
 * @brief Run one encrypted link cycle with retry handling.
 *
 * @param snapshot Telemetry snapshot to send.
 * @return Runtime link status.
 */
ESP32_UART_RuntimeStatus_t ESP32_UART_LinkTest_RunOnce(const telemetry_snapshot_t *snapshot)
{
    uint8_t nack_reason =
        TEST_NACK_REASON_NONE;
    for (uint32_t attempt = 1U;
         attempt <= ESP32_UART_RUNTIME_MAX_ATTEMPTS;
         ++attempt)
    {
        const ESP32_LinkResponse_t response =
        		ESP32_UART_RunEncryptedAttempt(snapshot,&nack_reason);
        g_loop_count++;
        if (response == ESP32_LINK_RESPONSE_ACK)
        {
            g_ack_count++;
            g_tx_ok_count++;
            g_test_sequence++;
            heartbeat_signal_esp32_uart_aes_gcm();
            system_fault_clear_active(
                SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM);
            /* Print health summary every 10 authenticated transactions. */
            if (((g_tx_ok_count % 10U) == 0U) &&
                ESP32_UART_PeriodicLogAllowed())
            {
                ESP32_UART_LinkTest_PrintHealth();
            }
            return ESP32_UART_RUNTIME_OK;
        }
        /* First failed attempt marks the link fault active. */
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

/**
 * @brief Print current encrypted-link health counters.
 */
void ESP32_UART_LinkTest_PrintHealth(void)
{
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

/**
 * @brief Delay the link test task.
 *
 * @param delay_ms Delay in ms.
 */
void ESP32_UART_LinkTest_DelayMs(uint32_t delay_ms)
{
    app_fault_delay_ms(delay_ms);
}

/* Add 16-bit value to telemetry buffer, little endian. */
static void telemetry_put_u16_le(
    uint8_t *buf,
    uint16_t *index,
    uint16_t value)
{
    buf[(*index)++] =
        (uint8_t)(value & 0xFFU);
    buf[(*index)++] =
        (uint8_t)((value >> 8U) & 0xFFU);
}

/* Add signed 16-bit value to telemetry buffer. */
static void telemetry_put_i16_le(
    uint8_t *buf,
    uint16_t *index,
    int16_t value)
{
    telemetry_put_u16_le(
        buf,
        index,
        (uint16_t)value);
}

/* Add 32-bit value to telemetry buffer, little endian. */
static void telemetry_put_u32_le(
    uint8_t *buf,
    uint16_t *index,
    uint32_t value)
{
    buf[(*index)++] =
        (uint8_t)((value >> 0U) & 0xFFU);
    buf[(*index)++] =
        (uint8_t)((value >> 8U) & 0xFFU);
    buf[(*index)++] =
        (uint8_t)((value >> 16U) & 0xFFU);
    buf[(*index)++] =
        (uint8_t)((value >> 24U) & 0xFFU);
}

/* Add float bytes to telemetry buffer. */
static void telemetry_put_float_le(
    uint8_t *buf,
    uint16_t *index,
    float value)
{
    uint32_t raw;
    memcpy(
        &raw,
        &value,
        sizeof(raw));
    telemetry_put_u32_le(
        buf,
        index,
        raw);
}

/* Add fixed label bytes to telemetry buffer. */
static void telemetry_put_label(
    uint8_t *buf,
    uint16_t *index,
    const char *label,
    uint8_t label_len)
{
    buf[(*index)++] =
        label_len;
    memcpy(
        &buf[*index],
        label,
        label_len);
    *index =
        (uint16_t)(
            *index + label_len);
}

/* Add fixed 32-byte string field. */
static void telemetry_put_fixed_string32(
    uint8_t *buf,
    uint16_t *index,
    const char *text)
{
    uint8_t field[32];
    memset(
        field,
        0,
        sizeof(field));
    if (text != NULL)
    {
        strncpy(
            (char *)field,
            text,
            sizeof(field) - 1U);
    }
    memcpy(
        &buf[*index],
        field,
        sizeof(field));
    *index =
        (uint16_t)(
            *index + sizeof(field));
}

/**
 * @brief Build the plaintext telemetry frame sent to ESP32.
 *
 * @param buf Output frame buffer.
 * @param buf_size Output buffer size.
 * @param seq Frame sequence number.
 * @param t Telemetry snapshot.
 * @return Frame length, or 0 on failure.
 */
static uint16_t ESP32_UART_BuildTelemetryFrame(
    uint8_t *buf,
    uint16_t buf_size,
    uint32_t seq,
    const telemetry_snapshot_t *t)
{
    uint16_t i = 0U;
    uint8_t checksum;
    if ((buf == NULL) ||
        (t == NULL) ||
        (buf_size < STM32_TELEMETRY_FRAME_LEN))
    {
        return 0U;
    }
    /*
     * Header.
     */
    buf[i++] = TEST_MAGIC_0;
    buf[i++] = TEST_MAGIC_1;
    buf[i++] = TEST_PROTO_VERSION;
    buf[i++] = TEST_MSG_TYPE_TELEMETRY;
    buf[i++] =
        (uint8_t)STM32_TELEMETRY_PAYLOAD_LEN;
    telemetry_put_u32_le(
        buf,
        &i,
        seq);
    /* --------------------------------
     * BME680
     * -------------------------------- */
    telemetry_put_label(
        buf,
        &i,
        "bme680",
        6U);
    telemetry_put_float_le(
        buf,
        &i,
        t->bme680.temperature_c);
    telemetry_put_float_le(
        buf,
        &i,
        t->bme680.pressure_hpa);
    telemetry_put_float_le(
        buf,
        &i,
        t->bme680.humidity_pct);
    telemetry_put_i16_le(
        buf,
        &i,
        t->bme680.iaq_score);
    telemetry_put_fixed_string32(
        buf,
        &i,
        t->bme680.iaq_label);
    /* --------------------------------
     * MAX31865
     * -------------------------------- */
    telemetry_put_label(
        buf,
        &i,
        "max31865",
        8U);
    telemetry_put_float_le(
        buf,
        &i,
        t->max31865.temperature_c);
    /* --------------------------------
     * DS18B20 x5
     * -------------------------------- */
    telemetry_put_label(
        buf,
        &i,
        "ds18b20x5",
        10U);
    for (uint32_t n = 0U;
         n < TELEMETRY_DS18B20_COUNT;
         ++n)
    {
        telemetry_put_float_le(
            buf,
            &i,
            t->ds18b20.temperature_c[n]);
    }
    /* --------------------------------
     * PM2.5
     * -------------------------------- */
    telemetry_put_label(
        buf,
        &i,
        "pm2.5",
        5U);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.pm1_0_standard);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.pm2_5_standard);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.pm10_standard);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.pm1_0_atmospheric);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.pm2_5_atmospheric);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.pm10_atmospheric);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.particles_0_3um);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.particles_0_5um);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.particles_1_0um);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.particles_2_5um);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.particles_5_0um);
    telemetry_put_u16_le(
        buf, &i,
        t->pm25.particles_10um);
    /* --------------------------------
     * MPU9250
     * -------------------------------- */
    telemetry_put_label(
        buf,
        &i,
        "mpu9250",
        7U);
    telemetry_put_float_le(
        buf, &i,
        t->mpu9250.ax_g);
    telemetry_put_float_le(
        buf, &i,
        t->mpu9250.ay_g);
    telemetry_put_float_le(
        buf, &i,
        t->mpu9250.az_g);
    telemetry_put_float_le(
        buf, &i,
        t->mpu9250.gx_dps);
    telemetry_put_float_le(
        buf, &i,
        t->mpu9250.gy_dps);
    telemetry_put_float_le(
        buf, &i,
        t->mpu9250.gz_dps);
    /* 168 payload bytes must exist before checksum. */
    if (i !=
        (STM32_TELEMETRY_FRAME_LEN - 1U))
    {
        return 0U;
    }
    checksum =
        ESP32_UART_TestChecksum8(
            buf,
            i);
    buf[i++] = checksum;
    /* Final telemetry frame is 169 bytes. */
    return i;
}

/* Build the small heartbeat test frame. */
static uint16_t ESP32_UART_BuildTestFrame(uint8_t *buf,
                                          uint16_t buf_size,
                                          uint32_t seq)
{
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
    /* 4-byte test payload. */
    buf[i++] = 4U;
    /* Sequence number is little endian. */
    buf[i++] = (uint8_t)((seq >> 0U)  & 0xFFU);
    buf[i++] = (uint8_t)((seq >> 8U)  & 0xFFU);
    buf[i++] = (uint8_t)((seq >> 16U) & 0xFFU);
    buf[i++] = (uint8_t)((seq >> 24U) & 0xFFU);
    /* Fixed payload used by the test frame. */
    buf[i++] = 0x11U;
    buf[i++] = 0x22U;
    buf[i++] = 0x33U;
    buf[i++] = 0x44U;
    checksum = ESP32_UART_TestChecksum8(buf, i);
    buf[i++] = checksum;
    return i;
}

/* Print test buffer in hex. */
static void ESP32_UART_TestPrintHex(
    const uint8_t *data,
    uint16_t len)
{
    (void)data;
    if (len == 0U)
    {
        ESP32_LOG_PRINTF("<empty>");
        return;
    }
    ESP32_LOG_PRINTF("**********");
}

/* Calculate 8-bit frame checksum. */
static uint8_t ESP32_UART_TestChecksum8(const uint8_t *data, uint16_t len)
{
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

/**
 * @brief Wait for and process an encrypted ACK/NACK.
 *
 * @param expected_sequence Sequence expected from ESP32.
 * @param timeout_ms Response timeout in ms.
 * @param nack_reason NACK reason output.
 * @return Link response.
 */
static ESP32_LinkResponse_t ESP32_UART_WaitForEncryptedResponse(
    uint32_t expected_sequence,
    uint32_t timeout_ms,
    uint8_t *nack_reason)
{
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
    /* Encrypted response is a fixed 39-byte packet. */
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
#if 0 /* raw RX packet debug */
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
    /* Authenticate/decrypt before checking ACK/NACK fields. */
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

/* Load received nonce, ciphertext and tag into crypto context. */
static bool ESP32_UART_LoadEncryptedResponse(
    crypto_ctx_t *ctx,
    const uint8_t *wire_packet,
    size_t wire_len)
{
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

/* NACK reason used in link logs. */
static const char *ESP32_UART_NackReasonToString(
    uint8_t reason)
{
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

/**
 * @brief Handle USART6 RX, TX and UART errors.
 */
void ESP32_UART_IRQHandler(void)
{
    const uint32_t sr =
        USART6->SR;
    /* UART error clears after SR/DR read; wake any waiting transfer. */
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

/* USART6 vector entry. */
void USART6_IRQHandler(void)
{
    ESP32_UART_IRQHandler();
}

#endif

/* Optional RX overflow hook. Keep ISR-safe. */
__attribute__((weak)) void ESP32_UART_RxOverflowCallback(void)
{
    /* Optional ISR-safe hook. */
}

/* Optional UART error hook. Keep ISR-safe. */
__attribute__((weak)) void ESP32_UART_ErrorCallback(uint32_t usart_sr)
{
    (void)usart_sr;
    /* Optional ISR-safe hook. */
}
