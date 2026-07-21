/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
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

#include "mbedtls/build_info.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/sha256.h"

#if !defined(MBEDTLS_AES_C)
#error "MBEDTLS_AES_C must be enabled"
#endif

#if !defined(MBEDTLS_GCM_C)
#error "MBEDTLS_GCM_C must be enabled"
#endif

#if !defined(MBEDTLS_SHA256_C)
#error "MBEDTLS_SHA256_C must be enabled"
#endif

static const char *TAG = "ESP32_AES_GCM";

static const uint8_t s_nonce_domain_label[] =
{
    'S', 'T', 'M', '3', '2',
    '-', 'G', 'C', 'M',
    '-', 'N', 'O', 'N', 'C', 'E',
    '-', 'V', '1'
};

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

/* Decodes a four-byte big-endian value. */
static uint32_t load_be32(
    const uint8_t in[4])
{
    /* Rebuild the value from explicit byte shifts. */
    return ((uint32_t)in[0] << 24) |
           ((uint32_t)in[1] << 16) |
           ((uint32_t)in[2] << 8)  |
           ((uint32_t)in[3]);
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
        &ctx->gcm);

    ctx->initialized = true;

    ESP_LOGI(
        TAG,
        "AES-GCM context initialized");

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
            &ctx->gcm);
    }

    /* Zeroize the context after releasing the Mbed TLS state. */
    mbedtls_platform_zeroize(
        ctx,
        sizeof(*ctx));
}

/* Derives a deterministic nonce session from public-key material. */
esp32_aes_gcm_status_t esp32_gcm_nonce_session_init(
    esp32_aes_gcm_ctx_t *ctx,
    const esp32_ecdh_ctx_t *ecdh_ctx)
{
    uint8_t material[
        sizeof(s_nonce_domain_label) +
        1U +
        ESP32_ECDH_PUBLIC_KEY_SIZE +
        1U +
        ESP32_ECDH_PUBLIC_KEY_SIZE];

    uint8_t digest[32];

    size_t material_len = 0U;
    uint32_t session_id = 0U;
    int rc;

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
        (ecdh_ctx->sender.public_key_len !=
            ESP32_ECDH_PUBLIC_KEY_SIZE) ||
        (ecdh_ctx->receiver.public_key_len !=
            ESP32_ECDH_PUBLIC_KEY_SIZE))
    {
        return ESP32_AES_GCM_ERR_BAD_ECDH_STATE;
    }

    mbedtls_platform_zeroize(
        material,
        sizeof(material));

    mbedtls_platform_zeroize(
        digest,
        sizeof(digest));

    memcpy(
        &material[material_len],
        s_nonce_domain_label,
        sizeof(s_nonce_domain_label));

    material_len +=
        sizeof(s_nonce_domain_label);

    material[material_len++] =
        (uint8_t)ecdh_ctx->sender.public_key_len;

    memcpy(
        &material[material_len],
        ecdh_ctx->sender.public_key_bytes,
        ecdh_ctx->sender.public_key_len);

    material_len +=
        ecdh_ctx->sender.public_key_len;

    material[material_len++] =
        (uint8_t)ecdh_ctx->receiver.public_key_len;

    memcpy(
        &material[material_len],
        ecdh_ctx->receiver.public_key_bytes,
        ecdh_ctx->receiver.public_key_len);

    material_len +=
        ecdh_ctx->receiver.public_key_len;

    /* Hash the domain label and public points before selecting a nonzero session. */
    rc = mbedtls_sha256(
        material,
        material_len,
        digest,
        0);

    mbedtls_platform_zeroize(
        material,
        sizeof(material));

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_sha256(nonce session)",
            rc);

        mbedtls_platform_zeroize(
            digest,
            sizeof(digest));

        return ESP32_AES_GCM_ERR_SESSION_HASH;
    }

    for (size_t offset = 0U;
         offset < sizeof(digest);
         offset += 4U)
    {
        session_id =
            load_be32(
                &digest[offset]);

        if (session_id != 0U)
        {
            break;
        }
    }

    mbedtls_platform_zeroize(
        digest,
        sizeof(digest));

    if (session_id == 0U)
    {
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
        "New GCM nonce session: session_id=%08lX "
        "first_counter=%llu",
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
        (plaintext_len > (SIZE_MAX - ESP32_AES_GCM_TAG_SIZE)))
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

    rc = mbedtls_gcm_setkey(
        &ctx->gcm,
        MBEDTLS_CIPHER_ID_AES,
        key,
        ESP32_AES_GCM_KEY_SIZE * 8U);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_gcm_setkey(encrypt)",
            rc);

        return ESP32_AES_GCM_ERR_SETKEY;
    }

    rc = mbedtls_gcm_crypt_and_tag(
        &ctx->gcm,
        MBEDTLS_GCM_ENCRYPT,
        plaintext_len,
        ctx->iv_gcm,
        sizeof(ctx->iv_gcm),
        aad,
        aad_len,
        plaintext,
        ciphertext_and_tag,
        ESP32_AES_GCM_TAG_SIZE,
        &ciphertext_and_tag[plaintext_len]);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_gcm_crypt_and_tag",
            rc);

        mbedtls_platform_zeroize(
            ciphertext_and_tag,
            required_size);

        return ESP32_AES_GCM_ERR_ENCRYPT;
    }

    *output_len =
        required_size;

    ESP_LOGI(
        TAG,
        "AES-GCM encryption succeeded: plaintext=%u "
        "ciphertext=%u tag=%u total=%u",
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

    rc = mbedtls_gcm_setkey(
        &ctx->gcm,
        MBEDTLS_CIPHER_ID_AES,
        ecdh_ctx->aes_key_receiver,
        ESP32_AES_GCM_KEY_SIZE * 8U);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_gcm_setkey(decrypt)",
            rc);

        return ESP32_AES_GCM_ERR_SETKEY;
    }

    /* Only expose the plaintext length after GCM authentication succeeds. */
    rc = mbedtls_gcm_auth_decrypt(
        &ctx->gcm,
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
            "AES-GCM authentication/decryption failed");

        return ESP32_AES_GCM_ERR_DECRYPT_AUTH;
    }

    *plaintext_len =
        ciphertext_len;

    ESP_LOGI(
        TAG,
        "AES-GCM authentication and decryption succeeded: "
        "plaintext=%u bytes",
        (unsigned int)*plaintext_len);

    return ESP32_AES_GCM_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

static const char *RUNTIME_TAG = "GCM_UART_RX";

#define STM32_GCM_TEST_PLAINTEXT_SIZE       14U
#define STM32_GCM_TEST_CIPHERTEXT_TAG_SIZE  \
    (STM32_GCM_TEST_PLAINTEXT_SIZE + ESP32_AES_GCM_TAG_SIZE)
#define STM32_GCM_TEST_WIRE_SIZE            \
    (ESP32_AES_GCM_NONCE_SIZE + STM32_GCM_TEST_CIPHERTEXT_TAG_SIZE)


_Static_assert(
    ESP32_AES_GCM_NONCE_SIZE == 12U,
    "STM32 interoperability test requires a 12-byte GCM nonce");

_Static_assert(
    ESP32_AES_GCM_TAG_SIZE == 16U,
    "STM32 interoperability test requires a 16-byte GCM tag");

_Static_assert(
    STM32_GCM_TEST_PLAINTEXT_SIZE == 14U,
    "STM32 plaintext UART frame must be 14 bytes");

_Static_assert(
    STM32_GCM_TEST_CIPHERTEXT_TAG_SIZE == 30U,
    "STM32 ciphertext || tag must be 30 bytes");

_Static_assert(
    STM32_GCM_TEST_WIRE_SIZE == 42U,
    "STM32 encrypted UART wire packet must be 42 bytes");

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

/* Checks the decrypted request's protocol fields and checksum. */
static bool runtime_validate_plaintext_frame(
    const uint8_t *frame,
    size_t frame_len,
    uint32_t *sequence_out)
{
    uint32_t sequence;

    if ((frame == NULL) ||
        (frame_len != STM32_GCM_TEST_PLAINTEXT_SIZE))
    {
        return false;
    }

    /* Check the fixed magic, version and request fields before the checksum. */
    if ((frame[0] != 0xA5U) ||
        (frame[1] != 0x5AU) ||
        (frame[2] != 0x01U) ||
        (frame[3] != 0x10U) ||
        (frame[4] != 4U))
    {
        return false;
    }

    if (runtime_checksum8(
            frame,
            frame_len - 1U) != frame[frame_len - 1U])
    {
        return false;
    }

    sequence =
        ((uint32_t)frame[5] << 0U)  |
        ((uint32_t)frame[6] << 8U)  |
        ((uint32_t)frame[7] << 16U) |
        ((uint32_t)frame[8] << 24U);

    if (sequence_out != NULL)
    {
        *sequence_out = sequence;
    }

    return true;
}

/* Authenticates received STM32 test frames. */
static void esp32_aes_gcm_runtime_task(void *param)
{
    esp32_aes_gcm_runtime_state_t *runtime =
        (esp32_aes_gcm_runtime_state_t *)param;

    const uint8_t *aad = NULL;
    const size_t aad_len = 0U;

    uint8_t wire_packet[STM32_GCM_TEST_WIRE_SIZE];
    uint8_t nonce[ESP32_AES_GCM_NONCE_SIZE];
    uint8_t ciphertext_and_tag[
        STM32_GCM_TEST_CIPHERTEXT_TAG_SIZE];
    uint8_t decrypted_plaintext[
        STM32_GCM_TEST_PLAINTEXT_SIZE];

    size_t wire_len = 0U;
    size_t decrypted_len = 0U;
    uint32_t sequence = 0U;
    uint32_t session_id = 0U;
    uint64_t nonce_counter = UINT64_C(0);
    uint32_t packet_counter = 0U;
    uint32_t auth_failure_counter = 0U;
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
        "Waiting continuously for STM32 encrypted UART packets");

    ESP_LOGI(
        RUNTIME_TAG,
        "Expected wire layout: nonce[12] || ciphertext[14] || tag[16]");

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

        if (!STM32_UART_ReceiveEncryptedPacket(
                wire_packet,
                sizeof(wire_packet),
                &wire_len,
                1000U))
        {
            continue;
        }

        if (wire_len != STM32_GCM_TEST_WIRE_SIZE)
        {
            ESP_LOGW(
                RUNTIME_TAG,
                "Discarding encrypted UART packet with invalid "
                "length: got=%u expected=%u",
                (unsigned int)wire_len,
                (unsigned int)STM32_GCM_TEST_WIRE_SIZE);

            continue;
        }

        memcpy(
            nonce,
            &wire_packet[0],
            ESP32_AES_GCM_NONCE_SIZE);

        memcpy(
            ciphertext_and_tag,
            &wire_packet[ESP32_AES_GCM_NONCE_SIZE],
            STM32_GCM_TEST_CIPHERTEXT_TAG_SIZE);

        session_id =
            runtime_load_be32(
                &nonce[0]);

        nonce_counter =
            runtime_load_be64(
                &nonce[4]);

        packet_counter++;

        ESP_LOGI(
            RUNTIME_TAG,
            "========== STM32->ESP32 ENCRYPTED PACKET %lu ==========",
            (unsigned long)packet_counter);

        ESP_LOGI(
            RUNTIME_TAG,
            "Complete UART wire packet [%u bytes]:",
            (unsigned int)wire_len);

        ESP_LOG_BUFFER_HEX_LEVEL(
            RUNTIME_TAG,
            wire_packet,
            wire_len,
            ESP_LOG_INFO);

        ESP_LOGI(
            RUNTIME_TAG,
            "Nonce from STM32 [%u bytes]:",
            (unsigned int)ESP32_AES_GCM_NONCE_SIZE);

        ESP_LOG_BUFFER_HEX_LEVEL(
            RUNTIME_TAG,
            nonce,
            sizeof(nonce),
            ESP_LOG_INFO);

        ESP_LOGI(
            RUNTIME_TAG,
            "Nonce metadata: session_id=%08lX counter=%llu",
            (unsigned long)session_id,
            (unsigned long long)nonce_counter);

        ESP_LOGI(
            RUNTIME_TAG,
            "Ciphertext from STM32 [%u bytes]:",
            (unsigned int)STM32_GCM_TEST_PLAINTEXT_SIZE);

        ESP_LOG_BUFFER_HEX_LEVEL(
            RUNTIME_TAG,
            ciphertext_and_tag,
            STM32_GCM_TEST_PLAINTEXT_SIZE,
            ESP_LOG_INFO);

        ESP_LOGI(
            RUNTIME_TAG,
            "Authentication tag from STM32 [%u bytes]:",
            (unsigned int)ESP32_AES_GCM_TAG_SIZE);

        ESP_LOG_BUFFER_HEX_LEVEL(
            RUNTIME_TAG,
            &ciphertext_and_tag[
                STM32_GCM_TEST_PLAINTEXT_SIZE],
            ESP32_AES_GCM_TAG_SIZE,
            ESP_LOG_INFO);

        /* Authenticate the encrypted bytes before inspecting plaintext fields. */
        status =
            esp32_decrypt_verify_aes_gcm_receiver(
                runtime->gcm_ctx,
                runtime->ecdh_ctx,
                nonce,
                ciphertext_and_tag,
                STM32_GCM_TEST_CIPHERTEXT_TAG_SIZE,
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
                "STM32 packet authentication/decryption failed: "
                "status=%d failures=%lu",
                (int)status,
                (unsigned long)auth_failure_counter);

            continue;
        }

        ESP_LOGI(
            RUNTIME_TAG,
            "Recovered plaintext UART frame [%u bytes]:",
            (unsigned int)decrypted_len);

        ESP_LOG_BUFFER_HEX_LEVEL(
            RUNTIME_TAG,
            decrypted_plaintext,
            decrypted_len,
            ESP_LOG_INFO);

        if (!runtime_validate_plaintext_frame(
                decrypted_plaintext,
                decrypted_len,
                &sequence))
        {
            ESP_LOGE(
                RUNTIME_TAG,
                "AES-GCM succeeded but recovered UART frame "
                "format/checksum is invalid");

            continue;
        }

        ESP_LOGI(
            RUNTIME_TAG,
            "Recovered frame valid: sequence=%lu "
            "payload=%02X %02X %02X %02X",
            (unsigned long)sequence,
            decrypted_plaintext[9],
            decrypted_plaintext[10],
            decrypted_plaintext[11],
            decrypted_plaintext[12]);

        ESP_LOGI(
            RUNTIME_TAG,
            "STM32 packet authenticated and decrypted successfully");

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
        buffer_is_all_zero(
            ecdh_ctx->aes_key_receiver,
            ESP32_AES_GCM_KEY_SIZE))
    {
        return ESP32_AES_GCM_ERR_BAD_ECDH_STATE;
    }

    s_runtime_state.gcm_ctx = gcm_ctx;
    s_runtime_state.ecdh_ctx = ecdh_ctx;
    s_runtime_state.task_handle = NULL;

    task_result =
        xTaskCreatePinnedToCore(
            esp32_aes_gcm_runtime_task,
            "aes_gcm_uart_rx",
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
        "AES-GCM UART receiver task started on core=%u",
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

