/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include "mbedtls_aes_gcm.h"

#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks_common.h"
#include "STM32_uart.h"

#include "esp_log.h"
#include "esp_log_buffer.h"
#include "esp_random.h"

#include "mbedtls/build_info.h"
#include "mbedtls/platform_util.h"

#if !defined(MBEDTLS_AES_C)
#error "MBEDTLS_AES_C must be enabled"
#endif

#if !defined(MBEDTLS_GCM_C)
#error "MBEDTLS_GCM_C must be enabled"
#endif


static const char *TAG = "ESP32_AES_GCM";

/* Encodes a 32-bit value in nonce byte order. */
static void store_be32(
    uint8_t out[4],
    uint32_t value)
{
    /* Write the most significant byte first for the session field. */
    out[0] = (uint8_t)(value >> 24);
    out[1] = (uint8_t)(value >> 16);
    out[2] = (uint8_t)(value >> 8);
    out[3] = (uint8_t)value;
}

/* Encodes the nonce counter as eight big-endian bytes. */
static void store_be64(
    uint8_t out[8],
    uint64_t value)
{
    /* Keep the wire counter independent of host byte order. */
    out[0] = (uint8_t)(value >> 56);
    out[1] = (uint8_t)(value >> 48);
    out[2] = (uint8_t)(value >> 40);
    out[3] = (uint8_t)(value >> 32);
    out[4] = (uint8_t)(value >> 24);
    out[5] = (uint8_t)(value >> 16);
    out[6] = (uint8_t)(value >> 8);
    out[7] = (uint8_t)value;
}

/* Detects an empty or all-zero byte buffer. */
static bool buffer_is_all_zero(
    const uint8_t *buffer,
    size_t length)
{
    /* OR every byte so the result does not depend on the first nonzero position. */
    uint8_t accumulator = 0U;

    if (buffer == NULL)
    {
        return true;
    }

    for (size_t i = 0U; i < length; ++i)
    {
        accumulator |= buffer[i];
    }

    return accumulator == 0U;
}

/* Checks the pointer required by an optional AAD buffer. */
static bool aad_is_valid(
    const uint8_t *aad,
    size_t aad_len)
{
    /* A null AAD pointer is valid only when its length is zero. */
    return ((aad_len == 0U) || (aad != NULL));
}

/* Logs a failed Mbed TLS operation and error code. */
static void log_mbedtls_error(
    const char *operation,
    int rc)
{
    /* Display the negative library result in hexadecimal. */
    ESP_LOGE(
        TAG,
        "%s failed: rc=-0x%04X",
        (operation != NULL) ? operation : "Mbed TLS operation",
        (unsigned int)(-rc));
}

/* Initializes the application's GCM context. */
esp32_aes_gcm_status_t esp32_aes_gcm_init(
    esp32_aes_gcm_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return ESP32_AES_GCM_ERR_BAD_PARAM;
    }

    /* Reject repeat initialization before resetting context storage. */
    if (ctx->initialized)
    {
        return ESP32_AES_GCM_ERR_ALREADY_INIT;
    }

    memset(
        ctx,
        0,
        sizeof(*ctx));

    mbedtls_gcm_init(
        &ctx->gcm_tx);

    mbedtls_gcm_init(
        &ctx->gcm_rx);

    ctx->initialized = true;

    ESP_LOGI(
        TAG,
        "Independent AES-GCM TX and RX contexts initialized");

    return ESP32_AES_GCM_SUCCESS;
}

/* Releases GCM resources and clears the application context. */
void esp32_aes_gcm_free(
    esp32_aes_gcm_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    if (ctx->initialized)
    {
        mbedtls_gcm_free(
            &ctx->gcm_tx);

        mbedtls_gcm_free(
            &ctx->gcm_rx);
    }

    /* Zeroize the context after releasing the Mbed TLS state. */
    mbedtls_platform_zeroize(
        ctx,
        sizeof(*ctx));
}

/* Initializes the transmit nonce session for the derived key. */
esp32_aes_gcm_status_t esp32_gcm_nonce_session_init(
    esp32_aes_gcm_ctx_t *ctx,
    const esp32_ecdh_ctx_t *ecdh_ctx)
{
    uint32_t session_id = 0U;

    if ((ctx == NULL) ||
        (ecdh_ctx == NULL))
    {
        return ESP32_AES_GCM_ERR_BAD_PARAM;
    }

    if (!ctx->initialized)
    {
        return ESP32_AES_GCM_ERR_NOT_INITIALIZED;
    }

    if ((!ecdh_ctx->keypairs_generated) ||
        (!ecdh_ctx->derived_keys_ready) ||
        (ecdh_ctx->aes_key_len !=
            ESP32_AES_GCM_KEY_SIZE) ||
        buffer_is_all_zero(
            ecdh_ctx->aes_key,
            ESP32_AES_GCM_KEY_SIZE))
    {
        return ESP32_AES_GCM_ERR_BAD_ECDH_STATE;
    }

    /*
     * Generate a fresh nonzero 32-bit session prefix for the
     * ESP32->STM32 transmit direction.
     *
     * app_main keeps bootloader_random_enable() active while this
     * function runs. A fresh session prefix prevents reuse of:
     *
     *     session_id[4] || counter[8]
     *
     * with the same static ECDH-derived AES key after an ESP32 reboot.
     */
    for (unsigned int attempt = 0U;
         attempt < 8U;
         ++attempt)
    {
        /* Try up to eight random draws to obtain a nonzero transmit session. */
        esp_fill_random(
            &session_id,
            sizeof(session_id));

        if (session_id != 0U)
        {
            break;
        }
    }

    if (session_id == 0U)
    {
        ESP_LOGE(
            TAG,
            "Failed to generate a nonzero GCM session identifier");

        return ESP32_AES_GCM_ERR_SESSION_HASH;
    }

    mbedtls_platform_zeroize(
        &ctx->nonce_state,
        sizeof(ctx->nonce_state));

    ctx->nonce_state.session_id =
        session_id;

    ctx->nonce_state.next_counter =
        ESP32_GCM_COUNTER_START;

    ctx->nonce_state.current_counter =
        UINT64_C(0);

    ctx->nonce_state.initialized =
        true;

    mbedtls_platform_zeroize(
        ctx->iv_gcm,
        sizeof(ctx->iv_gcm));

    ESP_LOGI(
        TAG,
        "New ESP32->STM32 GCM nonce session: "
        "session_id=%08lX first_counter=%llu",
        (unsigned long)ctx->nonce_state.session_id,
        (unsigned long long)ctx->nonce_state.next_counter);

    return ESP32_AES_GCM_SUCCESS;
}

/* Allocates the next transmit nonce from the session and counter. */
esp32_aes_gcm_status_t esp32_gcm_nonce_allocate(
    esp32_aes_gcm_ctx_t *ctx)
{
    uint64_t counter;

    if (ctx == NULL)
    {
        return ESP32_AES_GCM_ERR_BAD_PARAM;
    }

    if (!ctx->initialized)
    {
        return ESP32_AES_GCM_ERR_NOT_INITIALIZED;
    }

    if (!ctx->nonce_state.initialized)
    {
        return ESP32_AES_GCM_ERR_NO_SESSION;
    }

    /* Reject counter exhaustion before incrementing the nonce state. */
    if (ctx->nonce_state.next_counter == UINT64_MAX)
    {
        return ESP32_AES_GCM_ERR_COUNTER_EXHAUSTED;
    }

    counter =
        ctx->nonce_state.next_counter;

    ctx->nonce_state.next_counter++;
    ctx->nonce_state.current_counter =
        counter;

    store_be32(
        &ctx->iv_gcm[0],
        ctx->nonce_state.session_id);

    store_be64(
        &ctx->iv_gcm[4],
        counter);

    ESP_LOGI(
        TAG,
        "GCM nonce allocated: session=%08lX counter=%llu",
        (unsigned long)ctx->nonce_state.session_id,
        (unsigned long long)counter);

    return ESP32_AES_GCM_SUCCESS;
}

/* Allocates a nonce for the next GCM transfer. */
esp32_aes_gcm_status_t
esp32_generate_nonce_and_iv_per_transfer(
    esp32_aes_gcm_ctx_t *ctx)
{
    esp32_aes_gcm_status_t status;

    if (ctx == NULL)
    {
        return ESP32_AES_GCM_ERR_BAD_PARAM;
    }

    status =
        esp32_gcm_nonce_allocate(
            ctx);

    if (status != ESP32_AES_GCM_SUCCESS)
    {
        /* Clear the IV if nonce allocation fails. */
        mbedtls_platform_zeroize(
            ctx->iv_gcm,
            sizeof(ctx->iv_gcm));

        return status;
    }

    return ESP32_AES_GCM_SUCCESS;
}

/* Encrypts a payload and appends its GCM authentication tag. */
esp32_aes_gcm_status_t esp32_encrypt_aes_gcm(
    esp32_aes_gcm_ctx_t *ctx,
    const uint8_t key[ESP32_AES_GCM_KEY_SIZE],
    const uint8_t *plaintext,
    size_t plaintext_len,
    const uint8_t *aad,
    size_t aad_len,
    uint8_t *ciphertext_and_tag,
    size_t output_capacity,
    size_t *output_len)
{
    uint8_t tag[ESP32_AES_GCM_TAG_SIZE];
    size_t required_size;
    int rc;

    if (output_len != NULL)
    {
        *output_len = 0U;
    }

    if ((ctx == NULL) ||
        (key == NULL) ||
        (plaintext == NULL) ||
        (ciphertext_and_tag == NULL) ||
        (output_len == NULL) ||
        (!aad_is_valid(aad, aad_len)))
    {
        return ESP32_AES_GCM_ERR_BAD_PARAM;
    }

    if (!ctx->initialized)
    {
        return ESP32_AES_GCM_ERR_NOT_INITIALIZED;
    }

    if ((!ctx->nonce_state.initialized) ||
        (ctx->nonce_state.current_counter == UINT64_C(0)) ||
        buffer_is_all_zero(
            ctx->iv_gcm,
            sizeof(ctx->iv_gcm)))
    {
        return ESP32_AES_GCM_ERR_NO_SESSION;
    }

    if ((plaintext_len == 0U) ||
        (plaintext_len >
            (SIZE_MAX - ESP32_AES_GCM_TAG_SIZE)))
    {
        return ESP32_AES_GCM_ERR_LENGTH;
    }

    required_size =
        plaintext_len +
        ESP32_AES_GCM_TAG_SIZE;

    /* Check output capacity for both ciphertext and tag before encryption. */
    if (output_capacity < required_size)
    {
        return ESP32_AES_GCM_ERR_OUTPUT_TOO_SMALL;
    }

    if (buffer_is_all_zero(
            key,
            ESP32_AES_GCM_KEY_SIZE))
    {
        return ESP32_AES_GCM_ERR_BAD_KEY;
    }

    mbedtls_platform_zeroize(
        ciphertext_and_tag,
        required_size);

    mbedtls_platform_zeroize(
        tag,
        sizeof(tag));

    rc = mbedtls_gcm_setkey(
        &ctx->gcm_tx,
        MBEDTLS_CIPHER_ID_AES,
        key,
        ESP32_AES_GCM_KEY_SIZE * 8U);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_gcm_setkey(gcm_tx)",
            rc);

        mbedtls_platform_zeroize(
            tag,
            sizeof(tag));

        return ESP32_AES_GCM_ERR_SETKEY;
    }

    rc = mbedtls_gcm_crypt_and_tag(
        &ctx->gcm_tx,
        MBEDTLS_GCM_ENCRYPT,
        plaintext_len,
        ctx->iv_gcm,
        sizeof(ctx->iv_gcm),
        aad,
        aad_len,
        plaintext,
        ciphertext_and_tag,
        ESP32_AES_GCM_TAG_SIZE,
        tag);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_gcm_crypt_and_tag(gcm_tx)",
            rc);

        mbedtls_platform_zeroize(
            ciphertext_and_tag,
            required_size);

        mbedtls_platform_zeroize(
            tag,
            sizeof(tag));

        return ESP32_AES_GCM_ERR_ENCRYPT;
    }

    memcpy(
        &ciphertext_and_tag[plaintext_len],
        tag,
        ESP32_AES_GCM_TAG_SIZE);

    mbedtls_platform_zeroize(
        tag,
        sizeof(tag));

    *output_len =
        required_size;

    ESP_LOGI(
        TAG,
        "AES-GCM TX encryption succeeded using dedicated gcm_tx: "
        "plaintext=%u ciphertext=%u tag=%u total=%u",
        (unsigned int)plaintext_len,
        (unsigned int)plaintext_len,
        (unsigned int)ESP32_AES_GCM_TAG_SIZE,
        (unsigned int)*output_len);

    return ESP32_AES_GCM_SUCCESS;
}

/* Allocates a nonce and encrypts with the derived sender key. */
esp32_aes_gcm_status_t
esp32_generate_nonce_iv_encrypt_gcm_sender(
    esp32_aes_gcm_ctx_t *ctx,
    const esp32_ecdh_ctx_t *ecdh_ctx,
    const uint8_t *plaintext,
    size_t plaintext_len,
    const uint8_t *aad,
    size_t aad_len,
    uint8_t nonce_out[ESP32_AES_GCM_NONCE_SIZE],
    uint8_t *ciphertext_and_tag,
    size_t output_capacity,
    size_t *output_len)
{
    esp32_aes_gcm_status_t status;

    if ((ctx == NULL) ||
        (ecdh_ctx == NULL) ||
        (nonce_out == NULL))
    {
        return ESP32_AES_GCM_ERR_BAD_PARAM;
    }

    if ((!ecdh_ctx->derived_keys_ready) ||
        (ecdh_ctx->aes_key_len != ESP32_AES_GCM_KEY_SIZE))
    {
        return ESP32_AES_GCM_ERR_BAD_ECDH_STATE;
    }

    status =
        esp32_generate_nonce_and_iv_per_transfer(
            ctx);

    if (status != ESP32_AES_GCM_SUCCESS)
    {
        return status;
    }

    /* Return the nonce separately from the ciphertext and tag. */
    memcpy(
        nonce_out,
        ctx->iv_gcm,
        ESP32_AES_GCM_NONCE_SIZE);

    status =
        esp32_encrypt_aes_gcm(
            ctx,
            ecdh_ctx->aes_key,
            plaintext,
            plaintext_len,
            aad,
            aad_len,
            ciphertext_and_tag,
            output_capacity,
            output_len);

    if (status != ESP32_AES_GCM_SUCCESS)
    {
        mbedtls_platform_zeroize(
            nonce_out,
            ESP32_AES_GCM_NONCE_SIZE);

        mbedtls_platform_zeroize(
            ctx->iv_gcm,
            sizeof(ctx->iv_gcm));

        return status;
    }

    return ESP32_AES_GCM_SUCCESS;
}

/* Authenticates and decrypts with the derived receiver key. */
esp32_aes_gcm_status_t
esp32_decrypt_verify_aes_gcm_receiver(
    esp32_aes_gcm_ctx_t *ctx,
    const esp32_ecdh_ctx_t *ecdh_ctx,
    const uint8_t nonce[ESP32_AES_GCM_NONCE_SIZE],
    const uint8_t *ciphertext_and_tag,
    size_t input_len,
    const uint8_t *aad,
    size_t aad_len,
    uint8_t *plaintext,
    size_t plaintext_capacity,
    size_t *plaintext_len)
{
    size_t ciphertext_len;
    const uint8_t *tag;
    int rc;

    if (plaintext_len != NULL)
    {
        *plaintext_len = 0U;
    }

    if ((ctx == NULL) ||
        (ecdh_ctx == NULL) ||
        (nonce == NULL) ||
        (ciphertext_and_tag == NULL) ||
        (plaintext == NULL) ||
        (plaintext_len == NULL) ||
        (!aad_is_valid(aad, aad_len)))
    {
        return ESP32_AES_GCM_ERR_BAD_PARAM;
    }

    if (!ctx->initialized)
    {
        return ESP32_AES_GCM_ERR_NOT_INITIALIZED;
    }

    if ((!ecdh_ctx->derived_keys_ready) ||
        (ecdh_ctx->aes_key_receiver_len !=
            ESP32_AES_GCM_KEY_SIZE))
    {
        return ESP32_AES_GCM_ERR_BAD_ECDH_STATE;
    }

    if ((input_len <= ESP32_AES_GCM_TAG_SIZE) ||
        buffer_is_all_zero(
            nonce,
            ESP32_AES_GCM_NONCE_SIZE))
    {
        return ESP32_AES_GCM_ERR_LENGTH;
    }

    ciphertext_len =
        input_len -
        ESP32_AES_GCM_TAG_SIZE;

    if (plaintext_capacity < ciphertext_len)
    {
        return ESP32_AES_GCM_ERR_OUTPUT_TOO_SMALL;
    }

    if (buffer_is_all_zero(
            ecdh_ctx->aes_key_receiver,
            ESP32_AES_GCM_KEY_SIZE))
    {
        return ESP32_AES_GCM_ERR_BAD_KEY;
    }

    tag =
        &ciphertext_and_tag[ciphertext_len];

    mbedtls_platform_zeroize(
        plaintext,
        plaintext_capacity);

    rc = mbedtls_gcm_setkey(
        &ctx->gcm_rx,
        MBEDTLS_CIPHER_ID_AES,
        ecdh_ctx->aes_key_receiver,
        ESP32_AES_GCM_KEY_SIZE * 8U);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_gcm_setkey(gcm_rx)",
            rc);

        return ESP32_AES_GCM_ERR_SETKEY;
    }

    /* Only expose the plaintext length after GCM authentication succeeds. */
    rc = mbedtls_gcm_auth_decrypt(
        &ctx->gcm_rx,
        ciphertext_len,
        nonce,
        ESP32_AES_GCM_NONCE_SIZE,
        aad,
        aad_len,
        tag,
        ESP32_AES_GCM_TAG_SIZE,
        ciphertext_and_tag,
        plaintext);

    if (rc != 0)
    {
        mbedtls_platform_zeroize(
            plaintext,
            plaintext_capacity);

        ESP_LOGE(
            TAG,
            "AES-GCM RX authentication/decryption failed");

        return ESP32_AES_GCM_ERR_DECRYPT_AUTH;
    }

    *plaintext_len =
        ciphertext_len;

    ESP_LOGI(
        TAG,
        "AES-GCM RX authentication/decryption succeeded using "
        "dedicated gcm_rx: plaintext=%u bytes",
        (unsigned int)*plaintext_len);

    return ESP32_AES_GCM_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

static const char *RUNTIME_TAG = "GCM_UART_DUPLEX";

/*
 * Temporary development logging for the ESP32 -> STM32 encrypted
 * ACK/NACK packet.
 *
 * Set to 0 after interoperability testing. It may also be overridden
 * from the build system with:
 *
 *     -DESP32_GCM_DEBUG_PRINT_TX_PACKET=0
 */
#ifndef ESP32_GCM_DEBUG_PRINT_TX_PACKET
#define ESP32_GCM_DEBUG_PRINT_TX_PACKET 1
#endif

#define STM32_GCM_REQUEST_PLAINTEXT_SIZE       14U
#define STM32_GCM_REQUEST_CIPHERTEXT_TAG_SIZE  \
    (STM32_GCM_REQUEST_PLAINTEXT_SIZE + ESP32_AES_GCM_TAG_SIZE)
#define STM32_GCM_REQUEST_WIRE_SIZE            \
    (ESP32_AES_GCM_NONCE_SIZE + STM32_GCM_REQUEST_CIPHERTEXT_TAG_SIZE)

#define ESP32_GCM_RESPONSE_PLAINTEXT_SIZE       11U
#define ESP32_GCM_RESPONSE_CIPHERTEXT_TAG_SIZE  \
    (ESP32_GCM_RESPONSE_PLAINTEXT_SIZE + ESP32_AES_GCM_TAG_SIZE)
#define ESP32_GCM_RESPONSE_WIRE_SIZE            \
    (ESP32_AES_GCM_NONCE_SIZE + ESP32_GCM_RESPONSE_CIPHERTEXT_TAG_SIZE)

#define STM32_TEST_MAGIC_0                      0xA5U
#define STM32_TEST_MAGIC_1                      0x5AU
#define STM32_TEST_PROTOCOL_VERSION             0x01U
#define STM32_TEST_REQUEST_TYPE                 0x10U
#define STM32_TEST_ACK_TYPE                     0x20U
#define STM32_TEST_NACK_TYPE                    0x21U
#define STM32_TEST_REQUEST_PAYLOAD_LENGTH       0x04U
#define STM32_TEST_RESPONSE_PAYLOAD_LENGTH      0x01U
#define STM32_UART_RESPONSE_TIMEOUT_MS          100U


_Static_assert(
    ESP32_AES_GCM_NONCE_SIZE == 12U,
    "Interoperability protocol requires a 12-byte GCM nonce");

_Static_assert(
    ESP32_AES_GCM_TAG_SIZE == 16U,
    "Interoperability protocol requires a 16-byte GCM tag");

_Static_assert(
    STM32_GCM_REQUEST_PLAINTEXT_SIZE == 14U,
    "STM32 request plaintext must be 14 bytes");

_Static_assert(
    STM32_GCM_REQUEST_CIPHERTEXT_TAG_SIZE == 30U,
    "STM32 request ciphertext || tag must be 30 bytes");

_Static_assert(
    STM32_GCM_REQUEST_WIRE_SIZE == 42U,
    "STM32 request wire packet must be 42 bytes");

_Static_assert(
    ESP32_GCM_RESPONSE_PLAINTEXT_SIZE == 11U,
    "ESP32 ACK/NACK plaintext must be 11 bytes");

_Static_assert(
    ESP32_GCM_RESPONSE_CIPHERTEXT_TAG_SIZE == 27U,
    "ESP32 ACK/NACK ciphertext || tag must be 27 bytes");

_Static_assert(
    ESP32_GCM_RESPONSE_WIRE_SIZE == 39U,
    "ESP32 ACK/NACK wire packet must be 39 bytes");

typedef enum
{
    ESP32_RESPONSE_REASON_NONE          = 0x00U,
    ESP32_RESPONSE_REASON_BAD_CHECKSUM  = 0x01U,
    ESP32_RESPONSE_REASON_BAD_MAGIC     = 0x02U,
    ESP32_RESPONSE_REASON_BAD_FORMAT    = 0x03U,
    ESP32_RESPONSE_REASON_GCM_AUTH      = 0x04U

} esp32_response_reason_t;

extern bool STM32_UART_ReceiveEncryptedPacket(
    uint8_t *packet,
    size_t packet_capacity,
    size_t *packet_len,
    uint32_t timeout_ms);

typedef struct
{
    esp32_aes_gcm_ctx_t *gcm_ctx;
    esp32_ecdh_ctx_t *ecdh_ctx;
    TaskHandle_t task_handle;

} esp32_aes_gcm_runtime_state_t;

static esp32_aes_gcm_runtime_state_t s_runtime_state;

/* Reads the session ID from a wire nonce. */
static uint32_t runtime_load_be32(
    const uint8_t input[4])
{
    /* The nonce session is transmitted most significant byte first. */
    return ((uint32_t)input[0] << 24U) |
           ((uint32_t)input[1] << 16U) |
           ((uint32_t)input[2] << 8U)  |
           ((uint32_t)input[3]);
}

/* Reads the 64-bit counter from a wire nonce. */
static uint64_t runtime_load_be64(
    const uint8_t input[8])
{
    /* Assemble all eight bytes in big-endian order. */
    return ((uint64_t)input[0] << 56U) |
           ((uint64_t)input[1] << 48U) |
           ((uint64_t)input[2] << 40U) |
           ((uint64_t)input[3] << 32U) |
           ((uint64_t)input[4] << 24U) |
           ((uint64_t)input[5] << 16U) |
           ((uint64_t)input[6] << 8U)  |
           ((uint64_t)input[7]);
}

/* Calculates the frame's additive byte checksum. */
static uint8_t runtime_checksum8(
    const uint8_t *data,
    size_t len)
{
    /* The eight-bit cast preserves the protocol's modulo-256 sum. */
    uint8_t sum = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    for (size_t i = 0U; i < len; ++i)
    {
        sum =
            (uint8_t)(sum + data[i]);
    }

    return sum;
}

/* Reads the transaction sequence from the plaintext frame. */
static uint32_t runtime_load_sequence_le(
    const uint8_t *frame)
{
    if (frame == NULL)
    {
        return 0U;
    }

    /* Decode the four sequence bytes at their fixed frame offsets. */
    return ((uint32_t)frame[5] << 0U)  |
           ((uint32_t)frame[6] << 8U)  |
           ((uint32_t)frame[7] << 16U) |
           ((uint32_t)frame[8] << 24U);
}

/* Checks the decrypted request's protocol fields and checksum. */
static esp32_response_reason_t runtime_validate_plaintext_frame(
    const uint8_t *frame,
    size_t frame_len,
    uint32_t *sequence_out)
{
    uint32_t sequence = 0U;

    if (sequence_out != NULL)
    {
        *sequence_out = 0U;
    }

    if ((frame == NULL) ||
        (frame_len != STM32_GCM_REQUEST_PLAINTEXT_SIZE))
    {
        return ESP32_RESPONSE_REASON_BAD_FORMAT;
    }

    sequence =
        runtime_load_sequence_le(frame);

    if (sequence_out != NULL)
    {
        *sequence_out = sequence;
    }

    /* Validate frame magic before accepting its type and payload length. */
    if ((frame[0] != STM32_TEST_MAGIC_0) ||
        (frame[1] != STM32_TEST_MAGIC_1))
    {
        return ESP32_RESPONSE_REASON_BAD_MAGIC;
    }

    if ((frame[2] != STM32_TEST_PROTOCOL_VERSION) ||
        (frame[3] != STM32_TEST_REQUEST_TYPE) ||
        (frame[4] != STM32_TEST_REQUEST_PAYLOAD_LENGTH))
    {
        return ESP32_RESPONSE_REASON_BAD_FORMAT;
    }

    if (runtime_checksum8(
            frame,
            frame_len - 1U) !=
        frame[frame_len - 1U])
    {
        return ESP32_RESPONSE_REASON_BAD_CHECKSUM;
    }

    return ESP32_RESPONSE_REASON_NONE;
}

/* Builds the ACK or NACK frame for a transaction. */
static void runtime_build_response_plaintext(
    uint8_t response[
        ESP32_GCM_RESPONSE_PLAINTEXT_SIZE],
    uint8_t message_type,
    uint32_t sequence,
    esp32_response_reason_t reason)
{
    if (response == NULL)
    {
        return;
    }

    mbedtls_platform_zeroize(
        response,
        ESP32_GCM_RESPONSE_PLAINTEXT_SIZE);

    response[0] = STM32_TEST_MAGIC_0;
    response[1] = STM32_TEST_MAGIC_1;
    response[2] = STM32_TEST_PROTOCOL_VERSION;
    response[3] = message_type;
    response[4] = STM32_TEST_RESPONSE_PAYLOAD_LENGTH;

    /* Echo the sequence in little-endian order before adding the checksum. */
    response[5] = (uint8_t)(sequence >> 0U);
    response[6] = (uint8_t)(sequence >> 8U);
    response[7] = (uint8_t)(sequence >> 16U);
    response[8] = (uint8_t)(sequence >> 24U);

    response[9] = (uint8_t)reason;

    response[10] =
        runtime_checksum8(
            response,
            ESP32_GCM_RESPONSE_PLAINTEXT_SIZE - 1U);
}

/* Encrypts and transmits the transaction ACK or NACK. */
static bool runtime_send_encrypted_response(
    esp32_aes_gcm_runtime_state_t *runtime,
    uint32_t sequence,
    esp32_response_reason_t reason)
{
    const uint8_t *aad = NULL;
    const size_t aad_len = 0U;

    uint8_t plaintext[
        ESP32_GCM_RESPONSE_PLAINTEXT_SIZE];

    uint8_t nonce[
        ESP32_AES_GCM_NONCE_SIZE];

    uint8_t ciphertext_and_tag[
        ESP32_GCM_RESPONSE_CIPHERTEXT_TAG_SIZE];

    uint8_t wire_packet[
        ESP32_GCM_RESPONSE_WIRE_SIZE];

    const uint8_t message_type =
        (reason == ESP32_RESPONSE_REASON_NONE) ?
            STM32_TEST_ACK_TYPE :
            STM32_TEST_NACK_TYPE;

    size_t encrypted_len = 0U;
    esp32_aes_gcm_status_t status;
    STM32_UART_Status_t uart_status;
    bool success = false;

    if ((runtime == NULL) ||
        (runtime->gcm_ctx == NULL) ||
        (runtime->ecdh_ctx == NULL))
    {
        return false;
    }

    mbedtls_platform_zeroize(
        plaintext,
        sizeof(plaintext));

    mbedtls_platform_zeroize(
        nonce,
        sizeof(nonce));

    mbedtls_platform_zeroize(
        ciphertext_and_tag,
        sizeof(ciphertext_and_tag));

    mbedtls_platform_zeroize(
        wire_packet,
        sizeof(wire_packet));

    runtime_build_response_plaintext(
        plaintext,
        message_type,
        sequence,
        reason);

    /* Use the sender key and a fresh nonce for every response. */
    status =
        esp32_generate_nonce_iv_encrypt_gcm_sender(
            runtime->gcm_ctx,
            runtime->ecdh_ctx,
            plaintext,
            sizeof(plaintext),
            aad,
            aad_len,
            nonce,
            ciphertext_and_tag,
            sizeof(ciphertext_and_tag),
            &encrypted_len);

    if ((status != ESP32_AES_GCM_SUCCESS) ||
        (encrypted_len !=
            ESP32_GCM_RESPONSE_CIPHERTEXT_TAG_SIZE))
    {
        ESP_LOGE(
            RUNTIME_TAG,
            "Encrypted ACK/NACK generation failed: "
            "status=%d encrypted_len=%u",
            (int)status,
            (unsigned int)encrypted_len);

        goto cleanup;
    }

    memcpy(
        &wire_packet[0],
        nonce,
        ESP32_AES_GCM_NONCE_SIZE);

    memcpy(
        &wire_packet[ESP32_AES_GCM_NONCE_SIZE],
        ciphertext_and_tag,
        encrypted_len);

    ESP_LOGI(
        RUNTIME_TAG,
        "========== ESP32->STM32 ENCRYPTED %s ==========",
        (message_type == STM32_TEST_ACK_TYPE) ?
            "ACK" :
            "NACK");

    ESP_LOGI(
        RUNTIME_TAG,
        "Response metadata: sequence=%lu reason=%u "
        "nonce_counter=%llu",
        (unsigned long)sequence,
        (unsigned int)reason,
        (unsigned long long)
            runtime->gcm_ctx->nonce_state.current_counter);

#if ESP32_GCM_DEBUG_PRINT_TX_PACKET

    ESP_LOGI(
        RUNTIME_TAG,
        "----- TEMP TX PACKET DEBUG BEGIN -----");

    ESP_LOGI(
        RUNTIME_TAG,
        "Response plaintext ACK/NACK [%u bytes]:",
        (unsigned int)sizeof(plaintext));

    ESP_LOG_BUFFER_HEX_LEVEL(
        RUNTIME_TAG,
        plaintext,
        sizeof(plaintext),
        ESP_LOG_INFO);

    ESP_LOGI(
        RUNTIME_TAG,
        "Outgoing nonce [%u bytes]:",
        (unsigned int)sizeof(nonce));

    ESP_LOG_BUFFER_HEX_LEVEL(
        RUNTIME_TAG,
        nonce,
        sizeof(nonce),
        ESP_LOG_INFO);

    ESP_LOGI(
        RUNTIME_TAG,
        "Internal IV_GCM [%u bytes]:",
        (unsigned int)sizeof(runtime->gcm_ctx->iv_gcm));

    ESP_LOG_BUFFER_HEX_LEVEL(
        RUNTIME_TAG,
        runtime->gcm_ctx->iv_gcm,
        sizeof(runtime->gcm_ctx->iv_gcm),
        ESP_LOG_INFO);

    ESP_LOGI(
        RUNTIME_TAG,
        "Response ciphertext only [%u bytes]:",
        (unsigned int)ESP32_GCM_RESPONSE_PLAINTEXT_SIZE);

    ESP_LOG_BUFFER_HEX_LEVEL(
        RUNTIME_TAG,
        ciphertext_and_tag,
        ESP32_GCM_RESPONSE_PLAINTEXT_SIZE,
        ESP_LOG_INFO);

    ESP_LOGI(
        RUNTIME_TAG,
        "Response authentication tag only [%u bytes]:",
        (unsigned int)ESP32_AES_GCM_TAG_SIZE);

    ESP_LOG_BUFFER_HEX_LEVEL(
        RUNTIME_TAG,
        &ciphertext_and_tag[
            ESP32_GCM_RESPONSE_PLAINTEXT_SIZE],
        ESP32_AES_GCM_TAG_SIZE,
        ESP_LOG_INFO);

    ESP_LOGI(
        RUNTIME_TAG,
        "Combined ciphertext || tag [%u bytes]:",
        (unsigned int)encrypted_len);

    ESP_LOG_BUFFER_HEX_LEVEL(
        RUNTIME_TAG,
        ciphertext_and_tag,
        encrypted_len,
        ESP_LOG_INFO);

    ESP_LOGI(
        RUNTIME_TAG,
        "Complete TX wire packet: nonce || ciphertext || tag "
        "[%u bytes]:",
        (unsigned int)sizeof(wire_packet));

    ESP_LOG_BUFFER_HEX_LEVEL(
        RUNTIME_TAG,
        wire_packet,
        sizeof(wire_packet),
        ESP_LOG_INFO);

    ESP_LOGI(
        RUNTIME_TAG,
        "TX layout: nonce=%u ciphertext=%u tag=%u wire=%u",
        (unsigned int)ESP32_AES_GCM_NONCE_SIZE,
        (unsigned int)ESP32_GCM_RESPONSE_PLAINTEXT_SIZE,
        (unsigned int)ESP32_AES_GCM_TAG_SIZE,
        (unsigned int)sizeof(wire_packet));

    ESP_LOGI(
        RUNTIME_TAG,
        "----- TEMP TX PACKET DEBUG END -----");

#endif /* ESP32_GCM_DEBUG_PRINT_TX_PACKET */

    uart_status =
        STM32_UART_Write(
            wire_packet,
            sizeof(wire_packet),
            STM32_UART_RESPONSE_TIMEOUT_MS);

    if (uart_status != STM32_UART_STATUS_OK)
    {
        ESP_LOGE(
            RUNTIME_TAG,
            "Encrypted ACK/NACK UART transmission failed: "
            "status=%d",
            (int)uart_status);

        goto cleanup;
    }

    ESP_LOGI(
        RUNTIME_TAG,
        "Encrypted %s sent successfully: wire=%u bytes",
        (message_type == STM32_TEST_ACK_TYPE) ?
            "ACK" :
            "NACK",
        (unsigned int)sizeof(wire_packet));

    success = true;

cleanup:

    mbedtls_platform_zeroize(
        plaintext,
        sizeof(plaintext));

    mbedtls_platform_zeroize(
        nonce,
        sizeof(nonce));

    mbedtls_platform_zeroize(
        ciphertext_and_tag,
        sizeof(ciphertext_and_tag));

    mbedtls_platform_zeroize(
        wire_packet,
        sizeof(wire_packet));

    return success;
}

/* Authenticates UART requests and returns encrypted ACK/NACK frames. */
static void esp32_aes_gcm_runtime_task(void *param)
{
    esp32_aes_gcm_runtime_state_t *runtime =
        (esp32_aes_gcm_runtime_state_t *)param;

    const uint8_t *aad = NULL;
    const size_t aad_len = 0U;

    uint8_t wire_packet[
        STM32_GCM_REQUEST_WIRE_SIZE];

    uint8_t nonce[
        ESP32_AES_GCM_NONCE_SIZE];

    uint8_t ciphertext_and_tag[
        STM32_GCM_REQUEST_CIPHERTEXT_TAG_SIZE];

    uint8_t decrypted_plaintext[
        STM32_GCM_REQUEST_PLAINTEXT_SIZE];

    size_t wire_len = 0U;
    size_t decrypted_len = 0U;

    uint32_t sequence = 0U;
    uint32_t session_id = 0U;
    uint64_t nonce_counter = UINT64_C(0);

    uint32_t packet_counter = 0U;
    uint32_t auth_failure_counter = 0U;
    uint32_t ack_counter = 0U;
    uint32_t nack_counter = 0U;

    esp32_response_reason_t response_reason;
    esp32_aes_gcm_status_t status;

    if ((runtime == NULL) ||
        (runtime->gcm_ctx == NULL) ||
        (runtime->ecdh_ctx == NULL))
    {
        ESP_LOGE(
            RUNTIME_TAG,
            "Runtime task received invalid crypto context");

        s_runtime_state.task_handle = NULL;
        s_runtime_state.gcm_ctx = NULL;
        s_runtime_state.ecdh_ctx = NULL;

        vTaskDelete(NULL);

        return;
    }

    ESP_LOGI(
        RUNTIME_TAG,
        "Full-duplex encrypted UART task started");

    ESP_LOGI(
        RUNTIME_TAG,
        "RX request: nonce[12] || ciphertext[14] || tag[16] "
        "= 42 bytes");

    ESP_LOGI(
        RUNTIME_TAG,
        "TX response: nonce[12] || ciphertext[11] || tag[16] "
        "= 39 bytes");

    for (;;)
    {
        mbedtls_platform_zeroize(
            wire_packet,
            sizeof(wire_packet));

        mbedtls_platform_zeroize(
            nonce,
            sizeof(nonce));

        mbedtls_platform_zeroize(
            ciphertext_and_tag,
            sizeof(ciphertext_and_tag));

        mbedtls_platform_zeroize(
            decrypted_plaintext,
            sizeof(decrypted_plaintext));

        wire_len = 0U;
        decrypted_len = 0U;
        sequence = 0U;
        session_id = 0U;
        nonce_counter = UINT64_C(0);
        response_reason =
            ESP32_RESPONSE_REASON_BAD_FORMAT;

        if (!STM32_UART_ReceiveEncryptedPacket(
                wire_packet,
                sizeof(wire_packet),
                &wire_len,
                1000U))
        {
            continue;
        }

        if (wire_len !=
            STM32_GCM_REQUEST_WIRE_SIZE)
        {
            ESP_LOGW(
                RUNTIME_TAG,
                "Discarding request candidate with invalid "
                "length: got=%u expected=%u",
                (unsigned int)wire_len,
                (unsigned int)STM32_GCM_REQUEST_WIRE_SIZE);

            continue;
        }

        memcpy(
            nonce,
            &wire_packet[0],
            ESP32_AES_GCM_NONCE_SIZE);

        memcpy(
            ciphertext_and_tag,
            &wire_packet[ESP32_AES_GCM_NONCE_SIZE],
            STM32_GCM_REQUEST_CIPHERTEXT_TAG_SIZE);

        session_id =
            runtime_load_be32(
                &nonce[0]);

        nonce_counter =
            runtime_load_be64(
                &nonce[4]);

        packet_counter++;

        ESP_LOGI(
            RUNTIME_TAG,
            "========== STM32->ESP32 ENCRYPTED REQUEST %lu ==========",
            (unsigned long)packet_counter);

        ESP_LOGI(
            RUNTIME_TAG,
            "Request nonce metadata: session_id=%08lX counter=%llu",
            (unsigned long)session_id,
            (unsigned long long)nonce_counter);

        ESP_LOGI(
            RUNTIME_TAG,
            "Complete request wire packet [%u bytes]:",
            (unsigned int)wire_len);

        ESP_LOG_BUFFER_HEX_LEVEL(
            RUNTIME_TAG,
            wire_packet,
            wire_len,
            ESP_LOG_INFO);

        /* Authenticate the encrypted bytes before inspecting plaintext fields. */
        status =
            esp32_decrypt_verify_aes_gcm_receiver(
                runtime->gcm_ctx,
                runtime->ecdh_ctx,
                nonce,
                ciphertext_and_tag,
                STM32_GCM_REQUEST_CIPHERTEXT_TAG_SIZE,
                aad,
                aad_len,
                decrypted_plaintext,
                sizeof(decrypted_plaintext),
                &decrypted_len);

        if (status != ESP32_AES_GCM_SUCCESS)
        {
            auth_failure_counter++;

            ESP_LOGE(
                RUNTIME_TAG,
                "STM32 request authentication/decryption failed: "
                "status=%d failures=%lu",
                (int)status,
                (unsigned long)auth_failure_counter);

            if (runtime_send_encrypted_response(
                    runtime,
                    0U,
                    ESP32_RESPONSE_REASON_GCM_AUTH))
            {
                nack_counter++;
            }

            continue;
        }

        ESP_LOGI(
            RUNTIME_TAG,
            "Authenticated request plaintext [%u bytes]:",
            (unsigned int)decrypted_len);

        ESP_LOG_BUFFER_HEX_LEVEL(
            RUNTIME_TAG,
            decrypted_plaintext,
            decrypted_len,
            ESP_LOG_INFO);

        response_reason =
            runtime_validate_plaintext_frame(
                decrypted_plaintext,
                decrypted_len,
                &sequence);

        if (response_reason !=
            ESP32_RESPONSE_REASON_NONE)
        {
            ESP_LOGW(
                RUNTIME_TAG,
                "Authenticated STM32 frame is invalid: "
                "sequence=%lu reason=%u",
                (unsigned long)sequence,
                (unsigned int)response_reason);

            if (runtime_send_encrypted_response(
                    runtime,
                    sequence,
                    response_reason))
            {
                nack_counter++;
            }

            continue;
        }

        ESP_LOGI(
            RUNTIME_TAG,
            "Valid STM32 request: sequence=%lu "
            "payload=%02X %02X %02X %02X",
            (unsigned long)sequence,
            decrypted_plaintext[9],
            decrypted_plaintext[10],
            decrypted_plaintext[11],
            decrypted_plaintext[12]);

        if (runtime_send_encrypted_response(
                runtime,
                sequence,
                ESP32_RESPONSE_REASON_NONE))
        {
            ack_counter++;

            ESP_LOGI(
                RUNTIME_TAG,
                "Full encrypted request/ACK cycle completed: "
                "sequence=%lu ack_total=%lu nack_total=%lu",
                (unsigned long)sequence,
                (unsigned long)ack_counter,
                (unsigned long)nack_counter);
        }
        else
        {
            ESP_LOGE(
                RUNTIME_TAG,
                "Request was valid but encrypted ACK transmission "
                "failed: sequence=%lu",
                (unsigned long)sequence);
        }

        ESP_LOGI(
            RUNTIME_TAG,
            "======================================================");
    }
}

/* Starts UART GCM processing with validated crypto contexts. */
esp32_aes_gcm_status_t esp32_aes_gcm_runtime_task_start(
    esp32_aes_gcm_ctx_t *gcm_ctx,
    esp32_ecdh_ctx_t *ecdh_ctx)
{
    BaseType_t task_result;

    if ((gcm_ctx == NULL) ||
        (ecdh_ctx == NULL))
    {
        return ESP32_AES_GCM_ERR_BAD_PARAM;
    }

    if (s_runtime_state.task_handle != NULL)
    {
        return ESP32_AES_GCM_ERR_TASK_ALREADY_RUNNING;
    }

    if (!gcm_ctx->initialized)
    {
        return ESP32_AES_GCM_ERR_NOT_INITIALIZED;
    }

    /* Reject startup if the receiver key material is incomplete. */
    if ((!ecdh_ctx->derived_keys_ready) ||
        (ecdh_ctx->aes_key_receiver_len !=
            ESP32_AES_GCM_KEY_SIZE) ||
        (ecdh_ctx->aes_key_len !=
            ESP32_AES_GCM_KEY_SIZE) ||
        buffer_is_all_zero(
            ecdh_ctx->aes_key_receiver,
            ESP32_AES_GCM_KEY_SIZE) ||
        buffer_is_all_zero(
            ecdh_ctx->aes_key,
            ESP32_AES_GCM_KEY_SIZE))
    {
        return ESP32_AES_GCM_ERR_BAD_ECDH_STATE;
    }

    if (!gcm_ctx->nonce_state.initialized)
    {
        return ESP32_AES_GCM_ERR_NO_SESSION;
    }

    s_runtime_state.gcm_ctx = gcm_ctx;
    s_runtime_state.ecdh_ctx = ecdh_ctx;
    s_runtime_state.task_handle = NULL;

    task_result =
        xTaskCreatePinnedToCore(
            esp32_aes_gcm_runtime_task,
            "aes_gcm_uart_duplex",
            AES_GCM_RUNTIME_TASK_STACK_SIZE,
            &s_runtime_state,
            AES_GCM_RUNTIME_TASK_PRIORITY,
            &s_runtime_state.task_handle,
            AES_GCM_RUNTIME_TASK_CORE_ID);

    if (task_result != pdPASS)
    {
        s_runtime_state.gcm_ctx = NULL;
        s_runtime_state.ecdh_ctx = NULL;
        s_runtime_state.task_handle = NULL;

        return ESP32_AES_GCM_ERR_TASK_CREATE;
    }

    ESP_LOGI(
        RUNTIME_TAG,
        "AES-GCM UART full-duplex task started on core=%u",
        (unsigned int)AES_GCM_RUNTIME_TASK_CORE_ID);

    return ESP32_AES_GCM_SUCCESS;
}

/* Stops UART GCM processing and clears its context pointers. */
void esp32_aes_gcm_runtime_task_stop(void)
{
    /* Clear the shared task state before deleting the worker. */
    TaskHandle_t task_handle =
        s_runtime_state.task_handle;

    s_runtime_state.task_handle = NULL;
    s_runtime_state.gcm_ctx = NULL;
    s_runtime_state.ecdh_ctx = NULL;

    if (task_handle != NULL)
    {
        vTaskDelete(task_handle);
    }
}

/* Reports whether the UART GCM worker exists. */
bool esp32_aes_gcm_runtime_task_is_running(void)
{
    /* The stored task handle tracks whether runtime processing is active. */
    return s_runtime_state.task_handle != NULL;
}

