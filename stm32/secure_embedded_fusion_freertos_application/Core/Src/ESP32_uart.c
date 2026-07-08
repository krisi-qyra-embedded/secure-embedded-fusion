/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "ESP32_uart.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "ecdh.h"
#include <string.h>

extern crypto_ctx_t g_crypto_ctx;

gcm_sender_status_t ecdh_encrypt_uart_payload_once(
    crypto_ctx_t *ctx,
    const uint8_t *uart_payload,
    size_t uart_payload_len,
    uint8_t nonce_out[AES_GCM_AEAD_IV_SIZE],
    uint8_t *ciphertext_and_tag_out,
    size_t output_capacity,
    size_t *output_len);

/* ========================================================================
 * Bare-metal STM32 <-> ESP32 UART link test layer
 *
 * USART2:
 *   debug logs through log_printf()
 *
 * USART6:
 *   PC6 = USART6_TX -> ESP32 RX
 *   PC7 = USART6_RX <- ESP32 TX
 *
 * This layer is only for first bring-up.
 * Later this gets replaced by:
 *   binary protocol parser -> CRC32 -> ACK/NACK -> AES-GCM AEAD
 * ======================================================================== */

#define TEST_MSG_TYPE_HEARTBEAT       0x10U
#define TEST_MSG_TYPE_ACK             0x20U
#define TEST_MSG_TYPE_NACK            0x21U
#define TEST_RESPONSE_PAYLOAD_LEN     1U
#define TEST_RESPONSE_FRAME_LEN       11U
#define ESP32_ACK_TIMEOUT_MS          300U
#define ESP32_MAX_RETRIES             3U


#define ESP32_TEST_PERIOD_MS        1000U
#define ESP32_RX_POLL_TIME_MS       100U
#define ESP32_RX_MAX_BYTES          128U

#define TEST_MAGIC_0                0xA5U
#define TEST_MAGIC_1                0x5AU
#define TEST_PROTO_VERSION          0x01U
#define TEST_MSG_TYPE_HEARTBEAT     0x10U

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

static volatile ESP32_UART_Stats_t g_stats;

static uint8_t ESP32_UART_TestChecksum8(const uint8_t *data, uint16_t len);
static void ESP32_UART_TestPrintHex(const uint8_t *data, uint16_t len);
static uint16_t ESP32_UART_BuildTestFrame(uint8_t *buf,
                                          uint16_t buf_size,
                                          uint32_t seq);

typedef enum {
    TEST_NACK_REASON_NONE             = 0x00U,
    TEST_NACK_REASON_BAD_VERSION      = 0x01U,
    TEST_NACK_REASON_BAD_TYPE         = 0x02U,
    TEST_NACK_REASON_BAD_LENGTH       = 0x03U,
    TEST_NACK_REASON_BAD_CHECKSUM     = 0x04U,
    TEST_NACK_REASON_INTERNAL_ERROR   = 0x05U
} test_nack_reason_t;

typedef enum {
    ESP32_LINK_RESPONSE_ACK = 0,
    ESP32_LINK_RESPONSE_NACK,
    ESP32_LINK_RESPONSE_TIMEOUT
} ESP32_LinkResponse_t;

static uint32_t g_tx_attempt_count = 0U;
static uint32_t g_ack_count = 0U;
static uint32_t g_nack_count = 0U;
static uint32_t g_ack_timeout_count = 0U;
static uint32_t g_retry_count = 0U;
static uint32_t g_stale_response_count = 0U;
static uint32_t g_bad_response_count = 0U;

static ESP32_LinkResponse_t ESP32_UART_WaitForResponse(
    uint32_t expected_sequence,
    uint32_t timeout_ms,
    uint8_t *nack_reason);

static const char *ESP32_UART_NackReasonToString(uint8_t reason);

/* ---------------- private helpers ---------------- */

/* Advances an ESP32 UART ring-buffer index. */
static uint16_t rb_next(uint16_t index, uint16_t size)
{
    /* Wrap at the configured buffer size rather than allowing overflow. */
    index++;

    if (index >= size) {
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

/* ---------------- public API ---------------- */

/* Starts the interrupt-driven USART6 ESP32 link. */
ESP32_UART_Status_t ESP32_UART_Init(void)
{
    /* Clear stale status and data before enabling receive and error interrupts. */
    uint32_t dummy;

    if (g_initialized != 0U) {
        return ESP32_UART_STATUS_OK;
    }

    memset((void *)&g_stats, 0, sizeof(g_stats));

    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_busy = 0U;

    esp32_uart_gpio_init();

    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    __DSB();

    USART6->CR1 = 0U;
    USART6->CR2 = 0U;
    USART6->CR3 = 0U;

    dummy = USART6->SR;
    dummy = USART6->DR;
    (void)dummy;

    USART6->BRR = esp32_uart_calc_brr(ESP32_UART_BAUDRATE);

    /*
     * 8 data bits, no parity, 1 stop bit:
     * M = 0, PCE = 0, STOP = 00.
     *
     * Enable:
     * - TX
     * - RX
     * - RXNE interrupt
     */
    USART6->CR1 = USART_CR1_TE |
                  USART_CR1_RE |
                  USART_CR1_RXNEIE;

    USART6->CR3 = USART_CR3_EIE;

    NVIC_SetPriority(USART6_IRQn, ESP32_UART_IRQ_PRIORITY);
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
}

/* Queues ESP32 UART bytes within the supplied timeout. */
ESP32_UART_Status_t ESP32_UART_Write(const uint8_t *data,
                                     uint16_t len,
                                     uint32_t timeout_ms)
{
    /* Wait for ring space without modifying indices outside the critical section. */
    uint16_t written = 0U;
    uint32_t start_ms;

    if ((data == NULL) || (len == 0U)) {
        return ESP32_UART_STATUS_INVALID_PARAM;
    }

    if (g_initialized == 0U) {
        return ESP32_UART_STATUS_NOT_INITIALIZED;
    }

    start_ms = esp32_uart_now_ms();

    while (written < len) {
        uint32_t primask;
        uint16_t next_head;
        uint8_t queued = 0U;

        primask = esp32_uart_enter_critical();

        next_head = rb_next(g_tx_head, ESP32_UART_TX_BUFFER_SIZE);

        if (next_head != g_tx_tail) {
            g_tx_buffer[g_tx_head] = data[written];
            g_tx_head = next_head;
            written++;
            queued = 1U;

            esp32_uart_start_tx_interrupt();
        }

        esp32_uart_exit_critical(primask);

        if (queued == 0U) {
            if (timeout_ms == 0U) {
                g_stats.tx_overflows++;
                return ESP32_UART_STATUS_TIMEOUT;
            }

            if (esp32_uart_elapsed_ms(start_ms) >= timeout_ms) {
                g_stats.tx_overflows++;
                return ESP32_UART_STATUS_TIMEOUT;
            }

            esp32_uart_delay_1ms();
        }
    }

    return ESP32_UART_STATUS_OK;
}

/* Reads ESP32 UART bytes within the supplied timeout. */
ESP32_UART_Status_t ESP32_UART_Read(uint8_t *data,
                                    uint16_t len,
                                    uint32_t timeout_ms,
                                    uint16_t *bytes_read)
{
    /* Report the actual byte count when a partial receive times out. */
    uint16_t count = 0U;
    uint32_t start_ms;

    if ((data == NULL) || (len == 0U)) {
        return ESP32_UART_STATUS_INVALID_PARAM;
    }

    if (g_initialized == 0U) {
        return ESP32_UART_STATUS_NOT_INITIALIZED;
    }

    if (bytes_read != NULL) {
        *bytes_read = 0U;
    }

    start_ms = esp32_uart_now_ms();

    while (count < len) {
        uint32_t primask;
        uint8_t got_byte = 0U;

        primask = esp32_uart_enter_critical();

        if (g_rx_head != g_rx_tail) {
            data[count] = g_rx_buffer[g_rx_tail];
            g_rx_tail = rb_next(g_rx_tail, ESP32_UART_RX_BUFFER_SIZE);
            count++;
            got_byte = 1U;
        }

        esp32_uart_exit_critical(primask);

        if (got_byte == 0U) {
            if (timeout_ms == 0U) {
                break;
            }

            if (esp32_uart_elapsed_ms(start_ms) >= timeout_ms) {
                break;
            }

            esp32_uart_delay_1ms();
        }
    }

    if (bytes_read != NULL) {
        *bytes_read = count;
    }

    if (count == len) {
        return ESP32_UART_STATUS_OK;
    }

    return ESP32_UART_STATUS_TIMEOUT;
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

    primask = esp32_uart_enter_critical();

    g_rx_head = 0U;
    g_rx_tail = 0U;

    esp32_uart_exit_critical(primask);
}

/* Discards queued ESP32 transmit data. */
void ESP32_UART_FlushTx(void)
{
    /* Disable TXE and TC interrupts with the transmit state reset. */
    uint32_t primask;

    primask = esp32_uart_enter_critical();

    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_busy = 0U;

    USART6->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

    esp32_uart_exit_critical(primask);
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
        log_printf("[FATAL] ESP32_UART_Init failed, status=%d\r\n", (int)st);
        return st;
    }

    log_printf("\r\n\r\n");
    log_printf("============================================================\r\n");
    log_printf("[BOOT] STM32F411 bare-metal ESP32 UART link test\r\n");
    log_printf("[BOOT] USART2 = debug logs\r\n");
    log_printf("[BOOT] USART6 = STM32 <-> ESP32 binary link\r\n");
    log_printf("[BOOT] PC6 USART6_TX -> ESP32_RX\r\n");
    log_printf("[BOOT] PC7 USART6_RX <- ESP32_TX\r\n");
    log_printf("[BOOT] GND must be common between STM32 and ESP32\r\n");
    log_printf("============================================================\r\n");

    ESP32_UART_FlushRx();
    ESP32_UART_FlushTx();
    ESP32_UART_ClearStats();

    g_test_sequence = 0U;
    g_tx_ok_count = 0U;
    g_tx_fail_count = 0U;
    g_rx_packet_count = 0U;
    g_loop_count = 0U;

    log_printf("[OK] ESP32 UART6 initialized\r\n");
    log_printf("[INFO] ESP32 UART baudrate = %lu\r\n",
               (unsigned long)ESP32_UART_BAUDRATE);
    log_printf("[INFO] One AES-GCM encrypted UART packet will be sent at startup\r\n");
    log_printf("[INFO] Test wire format: nonce[12] || ciphertext || tag[16]\r\n");
    log_printf("[INFO] ACK/NACK and receive-side processing are disabled for this test\r\n");

    return ESP32_UART_STATUS_OK;
}

/* ------------------------------------------------------------
 * Public test loop function
 * Call this from while(1) in main.c
 * ------------------------------------------------------------ */
/* Runs one ESP32 request/response transaction. */
void ESP32_UART_LinkTest_RunOnce(void)
{
    /* Advance the message sequence only after the expected ACK is accepted. */
    static bool startup_packet_sent = false;

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
    ESP32_UART_Status_t write_status;
    uint32_t wait_start;

    if (startup_packet_sent)
    {
        log_printf(
            "[TX][STARTUP] Encrypted startup packet was already sent; "
            "second transmission suppressed.\r\n");
        return;
    }

    memset(tx_frame, 0, sizeof(tx_frame));
    memset(nonce, 0, sizeof(nonce));
    memset(ciphertext_and_tag, 0, sizeof(ciphertext_and_tag));
    memset(wire_packet, 0, sizeof(wire_packet));

    tx_len = ESP32_UART_BuildTestFrame(
        tx_frame,
        sizeof(tx_frame),
        g_test_sequence);

    if (tx_len == 0U)
    {
        log_printf(
            "[TX][ERROR] Failed to build UART plaintext test frame\r\n");
        return;
    }

    log_printf("\r\n");
    log_printf(
        "========== STM32 -> ESP32 ENCRYPTED STARTUP TEST ==========\r\n");

    log_printf(
        "[TX][PLAINTEXT] Original UART frame [%u bytes]: ",
        (unsigned int)tx_len);

    ESP32_UART_TestPrintHex(tx_frame, tx_len);
    log_printf("\r\n");

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
        log_printf(
            "[TX][ERROR] UART AES-GCM encryption failed, rc=%d\r\n",
            (int)crypto_status);
        return;
    }

    if ((encrypted_len !=
            ((size_t)tx_len + AES_GCM_TAG_SIZE)) ||
        (encrypted_len > UINT16_MAX))
    {
        log_printf(
            "[TX][ERROR] Invalid encrypted size=%u\r\n",
            (unsigned int)encrypted_len);
        return;
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

    log_printf(
        "[TX][NONCE] IV_GCM [%u bytes]: ",
        (unsigned int)AES_GCM_AEAD_IV_SIZE);
    ESP32_UART_TestPrintHex(
        nonce,
        AES_GCM_AEAD_IV_SIZE);
    log_printf("\r\n");

    log_printf(
        "[TX][CIPHERTEXT] [%u bytes]: ",
        (unsigned int)ciphertext_len);
    ESP32_UART_TestPrintHex(
        ciphertext_and_tag,
        (uint16_t)ciphertext_len);
    log_printf("\r\n");

    log_printf(
        "[TX][TAG] [%u bytes]: ",
        (unsigned int)AES_GCM_TAG_SIZE);
    ESP32_UART_TestPrintHex(
        &ciphertext_and_tag[ciphertext_len],
        AES_GCM_TAG_SIZE);
    log_printf("\r\n");

    log_printf(
        "[TX][SIZES] plaintext=%u nonce=%u ciphertext=%u "
        "tag=%u wire=%u\r\n",
        (unsigned int)tx_len,
        (unsigned int)AES_GCM_AEAD_IV_SIZE,
        (unsigned int)ciphertext_len,
        (unsigned int)AES_GCM_TAG_SIZE,
        (unsigned int)wire_len);

    log_printf(
        "[TX][WIRE] nonce || ciphertext || tag [%u bytes]: ",
        (unsigned int)wire_len);
    ESP32_UART_TestPrintHex(
        wire_packet,
        wire_len);
    log_printf("\r\n");

    ESP32_UART_FlushRx();
    g_tx_attempt_count++;

    write_status =
        ESP32_UART_Write(
            wire_packet,
            wire_len,
            200U);

    if (write_status != ESP32_UART_STATUS_OK)
    {
        g_tx_fail_count++;

        log_printf(
            "[TX][ERROR] USART6 queue failed, status=%d\r\n",
            (int)write_status);
        return;
    }

    wait_start = HAL_GetTick();

    while (ESP32_UART_IsTxBusy())
    {
        if ((HAL_GetTick() - wait_start) >= 500U)
        {
            g_tx_fail_count++;

            log_printf(
                "[TX][ERROR] USART6 transmission did not complete "
                "within 500 ms\r\n");
            return;
        }

        __WFI();
    }

    startup_packet_sent = true;
    g_tx_ok_count++;
    g_loop_count++;
    g_test_sequence++;

    log_printf(
        "[TX][SUCCESS] One encrypted startup packet was sent "
        "to ESP32 over USART6.\r\n");

    log_printf(
        "[TX][INFO] No ACK/NACK was requested and no receive-side "
        "decryption was performed.\r\n");

    log_printf(
        "============================================================\r\n");
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

    log_printf("\r\n");
    log_printf("--------------- UART LINK HEALTH ---------------\r\n");
    log_printf("[HEALTH] loop_count=%lu\r\n",
               (unsigned long)g_loop_count);
    log_printf("[HEALTH] tx_ok=%lu\r\n",
               (unsigned long)g_tx_ok_count);
    log_printf("[HEALTH] tx_fail=%lu\r\n",
               (unsigned long)g_tx_fail_count);
    log_printf("[HEALTH] rx_packets=%lu\r\n",
               (unsigned long)g_rx_packet_count);
    log_printf("[HEALTH] drv_tx_bytes=%lu\r\n",
               (unsigned long)stats.tx_bytes);
    log_printf("[HEALTH] drv_rx_bytes=%lu\r\n",
               (unsigned long)stats.rx_bytes);
    log_printf("[HEALTH] rx_overflows=%lu\r\n",
               (unsigned long)stats.rx_overflows);
    log_printf("[HEALTH] tx_overflows=%lu\r\n",
               (unsigned long)stats.tx_overflows);
    log_printf("[HEALTH] parity_errors=%lu\r\n",
               (unsigned long)stats.parity_errors);
    log_printf("[HEALTH] framing_errors=%lu\r\n",
               (unsigned long)stats.framing_errors);
    log_printf("[HEALTH] noise_errors=%lu\r\n",
               (unsigned long)stats.noise_errors);
    log_printf("[HEALTH] overrun_errors=%lu\r\n",
               (unsigned long)stats.overrun_errors);
    log_printf("[HEALTH] rx_available=%u\r\n",
               (unsigned int)ESP32_UART_RxAvailable());
    log_printf("[HEALTH] tx_free=%u\r\n",
               (unsigned int)ESP32_UART_TxFreeSpace());
    log_printf("[HEALTH] tx_busy=%u\r\n",
               (unsigned int)ESP32_UART_IsTxBusy());
    log_printf("[HEALTH] tx_attempts=%lu\r\n",
               (unsigned long)g_tx_attempt_count);
    log_printf("[HEALTH] ack_received=%lu\r\n",
               (unsigned long)g_ack_count);
    log_printf("[HEALTH] nack_received=%lu\r\n",
               (unsigned long)g_nack_count);
    log_printf("[HEALTH] response_timeouts=%lu\r\n",
               (unsigned long)g_ack_timeout_count);
    log_printf("[HEALTH] retries=%lu\r\n",
               (unsigned long)g_retry_count);
    log_printf("[HEALTH] stale_responses=%lu\r\n",
               (unsigned long)g_stale_response_count);
    log_printf("[HEALTH] bad_responses=%lu\r\n",
               (unsigned long)g_bad_response_count);
    log_printf("------------------------------------------------\r\n");
}

/* ------------------------------------------------------------
 * Public bare-metal delay
 * ------------------------------------------------------------ */
/* Waits between ESP32 link transactions. */
void ESP32_UART_LinkTest_DelayMs(uint32_t delay_ms)
{
    /* Use elapsed HAL ticks to measure the requested interval. */
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < delay_ms)
    {
        __WFI();
    }
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
static void ESP32_UART_TestPrintHex(const uint8_t *data, uint16_t len)
{
    /* Skip a null packet before walking its bytes. */
    uint16_t i;

    if (data == NULL)
    {
        return;
    }

    for (i = 0U; i < len; i++)
    {
        log_printf("%02X", data[i]);

        if ((i + 1U) < len)
        {
            log_printf(" ");
        }
    }
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

/* Receives and validates a plaintext ESP32 ACK or NACK. */
static ESP32_LinkResponse_t ESP32_UART_WaitForResponse(
    uint32_t expected_sequence,
    uint32_t timeout_ms,
    uint8_t *nack_reason)
{
    /* Reject a response with a different sequence or invalid checksum. */
    uint8_t response[TEST_RESPONSE_FRAME_LEN];
    uint16_t response_count = 0U;
    uint32_t start_ms = HAL_GetTick();

    if (nack_reason != NULL)
    {
        *nack_reason = TEST_NACK_REASON_NONE;
    }

    memset(response, 0, sizeof(response));

    while ((HAL_GetTick() - start_ms) < timeout_ms)
    {
        uint8_t byte;

        if (ESP32_UART_RxAvailable() == 0U)
        {
            __WFI();
            continue;
        }

        if (ESP32_UART_ReadByte(&byte, 0U) != ESP32_UART_STATUS_OK)
        {
            continue;
        }

        if (response_count == 0U)
        {
            if (byte == TEST_MAGIC_0)
            {
                response[response_count++] = byte;
            }

            continue;
        }

        if (response_count == 1U)
        {
            if (byte == TEST_MAGIC_1)
            {
                response[response_count++] = byte;
            }
            else if (byte == TEST_MAGIC_0)
            {
                response[0] = byte;
                response_count = 1U;
            }
            else
            {
                response_count = 0U;
            }

            continue;
        }

        response[response_count++] = byte;

        if (response_count == TEST_RESPONSE_FRAME_LEN)
        {
            uint8_t calculated_checksum;
            uint8_t received_checksum;
            uint32_t received_sequence;
            uint8_t message_type;
            uint8_t reason;

            log_printf("[RX] Response from ESP32: ");
            ESP32_UART_TestPrintHex(response, TEST_RESPONSE_FRAME_LEN);
            log_printf("\r\n");

            calculated_checksum =
                ESP32_UART_TestChecksum8(response,
                                         TEST_RESPONSE_FRAME_LEN - 1U);

            received_checksum =
                response[TEST_RESPONSE_FRAME_LEN - 1U];

            if (calculated_checksum != received_checksum)
            {
                g_bad_response_count++;

                log_printf(
                    "[RX] Invalid response checksum: calculated=0x%02X received=0x%02X\r\n",
                    calculated_checksum,
                    received_checksum);

                response_count = 0U;
                continue;
            }

            if ((response[0] != TEST_MAGIC_0) ||
                (response[1] != TEST_MAGIC_1) ||
                (response[2] != TEST_PROTO_VERSION) ||
                (response[4] != TEST_RESPONSE_PAYLOAD_LEN))
            {
                g_bad_response_count++;

                log_printf("[RX] Invalid ACK/NACK frame header\r\n");

                response_count = 0U;
                continue;
            }

            message_type = response[3];

            received_sequence =
                ((uint32_t)response[5] << 0U)  |
                ((uint32_t)response[6] << 8U)  |
                ((uint32_t)response[7] << 16U) |
                ((uint32_t)response[8] << 24U);

            reason = response[9];

            if (received_sequence != expected_sequence)
            {
                g_stale_response_count++;

                log_printf(
                    "[RX] Stale response ignored: expected_seq=%lu received_seq=%lu\r\n",
                    (unsigned long)expected_sequence,
                    (unsigned long)received_sequence);

                response_count = 0U;
                continue;
            }

            g_rx_packet_count++;

            if ((message_type == TEST_MSG_TYPE_ACK) &&
                (reason == TEST_NACK_REASON_NONE))
            {
                log_printf("[RX] ACK accepted for seq=%lu\r\n",
                           (unsigned long)received_sequence);

                return ESP32_LINK_RESPONSE_ACK;
            }

            if (message_type == TEST_MSG_TYPE_NACK)
            {
                if (nack_reason != NULL)
                {
                    *nack_reason = reason;
                }

                log_printf(
                    "[RX] NACK accepted for seq=%lu reason=%u (%s)\r\n",
                    (unsigned long)received_sequence,
                    (unsigned int)reason,
                    ESP32_UART_NackReasonToString(reason));

                return ESP32_LINK_RESPONSE_NACK;
            }

            g_bad_response_count++;

            log_printf("[RX] Unknown response type=0x%02X\r\n",
                       message_type);

            response_count = 0U;
        }
    }

    return ESP32_LINK_RESPONSE_TIMEOUT;
}

/* Returns a diagnostic label for an ESP32 NACK reason. */
static const char *ESP32_UART_NackReasonToString(uint8_t reason)
{
    /* Use the unknown label when the reason is not recognized. */
    switch (reason)
    {
        case TEST_NACK_REASON_NONE:
            return "NONE";

        case TEST_NACK_REASON_BAD_VERSION:
            return "BAD_VERSION";

        case TEST_NACK_REASON_BAD_TYPE:
            return "BAD_TYPE";

        case TEST_NACK_REASON_BAD_LENGTH:
            return "BAD_LENGTH";

        case TEST_NACK_REASON_BAD_CHECKSUM:
            return "BAD_CHECKSUM";

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
    uint32_t sr;

    sr = USART6->SR;

    if ((sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0U) {
        volatile uint32_t dummy;

        dummy = USART6->DR;
        (void)dummy;

        if ((sr & USART_SR_PE) != 0U) {
            g_stats.parity_errors++;
        }

        if ((sr & USART_SR_FE) != 0U) {
            g_stats.framing_errors++;
        }

        if ((sr & USART_SR_NE) != 0U) {
            g_stats.noise_errors++;
        }

        if ((sr & USART_SR_ORE) != 0U) {
            g_stats.overrun_errors++;
        }

        ESP32_UART_ErrorCallback(sr);
    }

    if ((sr & USART_SR_RXNE) != 0U) {
        uint8_t byte;
        uint16_t next_head;

        byte = (uint8_t)(USART6->DR & 0xFFU);

        next_head = rb_next(g_rx_head, ESP32_UART_RX_BUFFER_SIZE);

        if (next_head != g_rx_tail) {
            g_rx_buffer[g_rx_head] = byte;
            g_rx_head = next_head;
            g_stats.rx_bytes++;
        } else {
            g_stats.rx_overflows++;
            ESP32_UART_RxOverflowCallback();
        }
    }

    if (((sr & USART_SR_TXE) != 0U) &&
        ((USART6->CR1 & USART_CR1_TXEIE) != 0U)) {

        if (g_tx_tail != g_tx_head) {
            USART6->DR = g_tx_buffer[g_tx_tail];
            g_tx_tail = rb_next(g_tx_tail, ESP32_UART_TX_BUFFER_SIZE);
            g_stats.tx_bytes++;
            g_tx_busy = 1U;
        } else {
            USART6->CR1 &= ~USART_CR1_TXEIE;
            USART6->CR1 |= USART_CR1_TCIE;
        }
    }

    if (((sr & USART_SR_TC) != 0U) &&
        ((USART6->CR1 & USART_CR1_TCIE) != 0U)) {

        USART6->CR1 &= ~USART_CR1_TCIE;
        USART6->SR &= ~USART_SR_TC;
        g_tx_busy = 0U;
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
