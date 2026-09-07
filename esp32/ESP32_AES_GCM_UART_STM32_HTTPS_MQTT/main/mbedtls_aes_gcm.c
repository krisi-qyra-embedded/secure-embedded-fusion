/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "mbedtls_aes_gcm.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "stm32_telemetry_public.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks_common.h"
#include "STM32_uart.h"

#include "esp_log.h"
#include "esp_log_timestamp.h"
#include "esp_log_buffer.h"
#include "esp_random.h"
#include "nvs.h"
#include "esp_err.h"

#include "mbedtls/build_info.h"
#include "mbedtls/platform_util.h"

#if !defined(MBEDTLS_AES_C)
#error "MBEDTLS_AES_C must be enabled"
#endif

#if !defined(MBEDTLS_GCM_C)
#error "MBEDTLS_GCM_C must be enabled"
#endif


static const char *TAG = "ESP32_AES_GCM";
static const char *RUNTIME_TAG = "GCM_UART_DUPLEX";
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* NVS is initialized by app_main. TX sessions are persisted; counters stay in RAM.
 * The zero RX seed is stored once. Incoming packets update only the RAM watermark.
 * Erasing NVS allows the seed to be provisioned again. */
#define ESP32_GCM_NVS_NAMESPACE        "gcmsecv1"
#define ESP32_GCM_NVS_STATE_KEY        "state"
#define ESP32_GCM_NVS_MAGIC            UINT32_C(0x47434D31)
#define ESP32_GCM_NVS_VERSION          UINT32_C(1)

#define ESP32_GCM_NVS_RX_SEED_KEY      "rxseed0v1"
#define ESP32_GCM_NVS_RX_SEED_MAGIC    UINT32_C(0x52585330)
#define ESP32_GCM_NVS_RX_SEED_VERSION  UINT32_C(1)

typedef struct
{
    uint32_t magic;  
    uint32_t version;  
    uint32_t generation;  

    uint32_t tx_session_id;  

    /* Legacy RX fields stay zero; live RX replay state is held in RAM. */
    uint32_t rx_session_id;  
    uint32_t rx_counter_hi;  
    uint32_t rx_counter_lo;  

    uint32_t crc32;  

} esp32_gcm_nvs_record_t;

typedef struct
{
    uint32_t magic;  
    uint32_t version;  
    uint32_t session_id;  
    uint32_t counter_hi;  
    uint32_t counter_lo;  
    uint32_t crc32;  

} esp32_gcm_rx_seed_record_t;

typedef enum
{
    ESP32_GCM_REPLAY_ACCEPT_NEW = 0, /**< New nonce; RAM watermark advanced. */
    ESP32_GCM_REPLAY_DUPLICATE_LAST,  
    ESP32_GCM_REPLAY_REJECT_OLD,  
    ESP32_GCM_REPLAY_STORAGE_ERROR,  
    ESP32_GCM_REPLAY_BAD_NONCE  

} esp32_gcm_replay_result_t;

static nvs_handle_t s_gcm_nvs_handle;
static bool s_gcm_nvs_open;
static bool s_gcm_nvs_loaded;
static esp32_gcm_nvs_record_t s_gcm_nvs_state;

static uint32_t s_gcm_rx_session_id_ram;
static uint64_t s_gcm_rx_counter_ram;
static bool s_gcm_rx_ram_initialized;

/* Computes the CRC for a persistent GCM state record. */
static uint32_t esp32_gcm_crc32(
    const uint8_t *data,
    size_t length)
{
    uint32_t crc = UINT32_C(0xFFFFFFFF);

    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }

    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];

        for (unsigned int bit = 0U; bit < 8U; ++bit)
        {
            /* Apply the reflected CRC polynomial to each record byte. */
            const uint32_t mask =
                (uint32_t)(0U - (crc & 1U));

            crc =
                (crc >> 1U) ^
                (UINT32_C(0xEDB88320) & mask);
        }
    }

    return ~crc;
}

/* Reconstructs the RX seed's 64-bit counter. */
static uint64_t esp32_gcm_rx_seed_counter(
    const esp32_gcm_rx_seed_record_t *record)
{
    if (record == NULL)
    {
        /* Combine the stored high and low words without signed extension. */
        return UINT64_C(0);
    }

    return
        ((uint64_t)record->counter_hi << 32U) |
        (uint64_t)record->counter_lo;
}

/* Checks the persistent TX record header and CRC. */
static bool esp32_gcm_nvs_record_valid(
    const esp32_gcm_nvs_record_t *record)
{
    uint32_t expected_crc;

    if (record == NULL)
    {
        return false;
    }

    if ((record->magic != ESP32_GCM_NVS_MAGIC) ||
        (record->version != ESP32_GCM_NVS_VERSION))
    {
        return false;
    }

    /* Exclude the CRC field itself when calculating the expected checksum. */
    expected_crc =
        esp32_gcm_crc32(
            (const uint8_t *)record,
            offsetof(
                esp32_gcm_nvs_record_t,
                crc32));

    return record->crc32 == expected_crc;
}

/* Validates the one-time RX seed record. */
static bool esp32_gcm_rx_seed_record_valid(
    const esp32_gcm_rx_seed_record_t *record)
{
    uint32_t expected_crc;

    if (record == NULL)
    {
        return false;
    }

    /* Check the seed format before accepting its CRC. */
    if ((record->magic != ESP32_GCM_NVS_RX_SEED_MAGIC) ||
        (record->version != ESP32_GCM_NVS_RX_SEED_VERSION))
    {
        return false;
    }

    expected_crc =
        esp32_gcm_crc32(
            (const uint8_t *)record,
            offsetof(
                esp32_gcm_rx_seed_record_t,
                crc32));

    return record->crc32 == expected_crc;
}

/* Loads or creates the fixed zero RX seed in NVS. */
static bool esp32_gcm_nvs_load_or_provision_rx_seed_once(void)
{
    esp32_gcm_rx_seed_record_t seed;
    esp_err_t err;
    size_t blob_size;

    if (!s_gcm_nvs_open)
    {
        return false;
    }

    mbedtls_platform_zeroize(
        &seed,
        sizeof(seed));

    blob_size =
        sizeof(seed);

    err =
        nvs_get_blob(
            s_gcm_nvs_handle,
            ESP32_GCM_NVS_RX_SEED_KEY,
            &seed,
            &blob_size);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        seed.magic =
            ESP32_GCM_NVS_RX_SEED_MAGIC;

        seed.version =
            ESP32_GCM_NVS_RX_SEED_VERSION;

        seed.session_id = 0U;
        seed.counter_hi = 0U;
        seed.counter_lo = 0U;

        seed.crc32 =
            esp32_gcm_crc32(
                (const uint8_t *)&seed,
                offsetof(
                    esp32_gcm_rx_seed_record_t,
                    crc32));

        err =
            nvs_set_blob(
                s_gcm_nvs_handle,
                ESP32_GCM_NVS_RX_SEED_KEY,
                &seed,
                sizeof(seed));

        if (err == ESP_OK)
        {
            err =
                nvs_commit(
                    s_gcm_nvs_handle);
        }

        if (err != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "One-time RX seed provisioning failed: %s",
                esp_err_to_name(err));

            mbedtls_platform_zeroize(
                &seed,
                sizeof(seed));

            return false;
        }

        ESP_LOGI(
            TAG,
            "One-time NVS RX seed programmed: "
            "session=00000000 counter=0 key=%s",
            ESP32_GCM_NVS_RX_SEED_KEY);
    }
    else
    {
        if (err != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "nvs_get_blob(%s) failed: %s",
                ESP32_GCM_NVS_RX_SEED_KEY,
                esp_err_to_name(err));

            mbedtls_platform_zeroize(
                &seed,
                sizeof(seed));

            return false;
        }

        /* Existing seeds must remain zero; runtime RX progress stays in RAM. */
        if ((blob_size != sizeof(seed)) ||
            (!esp32_gcm_rx_seed_record_valid(
                &seed)) ||
            (seed.session_id != 0U) ||
            (esp32_gcm_rx_seed_counter(
                &seed) != UINT64_C(0)))
        {
            ESP_LOGE(
                TAG,
                "One-time RX seed is invalid or not {session=0,counter=0}; "
                "failing closed");

            mbedtls_platform_zeroize(
                &seed,
                sizeof(seed));

            return false;
        }

        ESP_LOGI(
            TAG,
            "One-time NVS RX seed already provisioned: "
            "session=00000000 counter=0 (no NVS rewrite)");
    }

    s_gcm_rx_session_id_ram = 0U;
    s_gcm_rx_counter_ram = UINT64_C(0);
    s_gcm_rx_ram_initialized = true;

    mbedtls_platform_zeroize(
        &seed,
        sizeof(seed));

    return true;
}

/* Closes GCM storage and clears its cached state. */
static void esp32_gcm_nvs_close(void)
{
    if (s_gcm_nvs_open)
    {
        nvs_close(
            s_gcm_nvs_handle);
    }

    s_gcm_nvs_handle = 0U;
    s_gcm_nvs_open = false;
    s_gcm_nvs_loaded = false;

    /* Clear RAM replay state when the storage context is released. */
    s_gcm_rx_session_id_ram = 0U;
    s_gcm_rx_counter_ram = UINT64_C(0);
    s_gcm_rx_ram_initialized = false;

    mbedtls_platform_zeroize(
        &s_gcm_nvs_state,
        sizeof(s_gcm_nvs_state));
}

/* Loads and validates persistent GCM session state. */
static bool esp32_gcm_nvs_load_once(void)
{
    esp_err_t err;
    size_t blob_size;

    if (s_gcm_nvs_loaded)
    {
        return true;
    }

    if (!s_gcm_nvs_open)
    {
        err =
            nvs_open(
                ESP32_GCM_NVS_NAMESPACE,
                NVS_READWRITE,
                &s_gcm_nvs_handle);

        if (err != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "nvs_open(%s) failed: %s",
                ESP32_GCM_NVS_NAMESPACE,
                esp_err_to_name(err));

            return false;
        }

        s_gcm_nvs_open = true;
    }

    mbedtls_platform_zeroize(
        &s_gcm_nvs_state,
        sizeof(s_gcm_nvs_state));

    blob_size =
        sizeof(s_gcm_nvs_state);

    err =
        nvs_get_blob(
            s_gcm_nvs_handle,
            ESP32_GCM_NVS_STATE_KEY,
            &s_gcm_nvs_state,
            &blob_size);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        s_gcm_nvs_state.magic =
            ESP32_GCM_NVS_MAGIC;

        s_gcm_nvs_state.version =
            ESP32_GCM_NVS_VERSION;

        s_gcm_nvs_state.rx_session_id = 0U;
        s_gcm_nvs_state.rx_counter_hi = 0U;
        s_gcm_nvs_state.rx_counter_lo = 0U;

        ESP_LOGI(
            TAG,
            "Persistent TX GCM state is blank; first TX reserve "
            "will create generation 1");
    }
    else
    {
        if (err != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "nvs_get_blob(%s) failed: %s",
                ESP32_GCM_NVS_STATE_KEY,
                esp_err_to_name(err));

            esp32_gcm_nvs_close();
            return false;
        }

        /* Fail closed on a corrupt record instead of resetting its session counter. */
        if ((blob_size != sizeof(s_gcm_nvs_state)) ||
            (!esp32_gcm_nvs_record_valid(
                &s_gcm_nvs_state)))
        {
            ESP_LOGE(
                TAG,
                "Persistent GCM TX state is invalid or corrupted; "
                "failing closed");

            esp32_gcm_nvs_close();
            return false;
        }

        ESP_LOGI(
            TAG,
            "Persistent GCM TX state loaded: generation=%lu "
            "tx_session=%lu",
            (unsigned long)s_gcm_nvs_state.generation,
            (unsigned long)s_gcm_nvs_state.tx_session_id);
    }

    if (!esp32_gcm_nvs_load_or_provision_rx_seed_once())
    {
        esp32_gcm_nvs_close();
        return false;
    }

    s_gcm_nvs_loaded = true;

    ESP_LOGI(
        TAG,
        "STM32->ESP32 replay policy: "
        "NVS seed={session=00000000,counter=0} "
        "runtime_high_water=RAM_ONLY");

    return true;
}

/* Commits a candidate TX session record to NVS. */
static bool esp32_gcm_nvs_commit(
    const esp32_gcm_nvs_record_t *candidate)
{
    esp32_gcm_nvs_record_t record;
    esp_err_t err;

    if ((candidate == NULL) ||
        (!esp32_gcm_nvs_load_once()))
    {
        return false;
    }

    record =
        *candidate;

    record.magic =
        ESP32_GCM_NVS_MAGIC;

    record.version =
        ESP32_GCM_NVS_VERSION;

    /* Keep persistent RX fields zero; RX high-water marks are RAM-only. */
    record.rx_session_id = 0U;
    record.rx_counter_hi = 0U;
    record.rx_counter_lo = 0U;

    record.crc32 =
        esp32_gcm_crc32(
            (const uint8_t *)&record,
            offsetof(
                esp32_gcm_nvs_record_t,
                crc32));

    err =
        nvs_set_blob(
            s_gcm_nvs_handle,
            ESP32_GCM_NVS_STATE_KEY,
            &record,
            sizeof(record));

    if (err == ESP_OK)
    {
        err =
            nvs_commit(
                s_gcm_nvs_handle);
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Persistent GCM state commit failed: %s",
            esp_err_to_name(err));

        mbedtls_platform_zeroize(
            &record,
            sizeof(record));

        return false;
    }

    s_gcm_nvs_state =
        record;

    mbedtls_platform_zeroize(
        &record,
        sizeof(record));

    return true;
}

/* Reserves the next persistent transmit session ID. */
static bool esp32_gcm_nvs_reserve_tx_session(
    uint32_t *session_id_out)
{
    esp32_gcm_nvs_record_t next;

    if ((session_id_out == NULL) ||
        (!esp32_gcm_nvs_load_once()))
    {
        return false;
    }

    if (s_gcm_nvs_state.tx_session_id == UINT32_MAX)
    {
        ESP_LOGE(
            TAG,
            "Persistent TX session counter is exhausted");

        return false;
    }

    next =
        s_gcm_nvs_state;

    if (next.generation == UINT32_MAX)
    {
        ESP_LOGE(
            TAG,
            "Persistent GCM generation counter is exhausted");

        return false;
    }

    next.generation++;
    next.tx_session_id++;

    /* I commit the new session before allowing it to be used for transmission. */
    if (!esp32_gcm_nvs_commit(
            &next))
    {
        return false;
    }

    *session_id_out =
        next.tx_session_id;

    return true;
}

/* Returns the current RAM replay high-water mark. */
static bool esp32_gcm_get_rx_replay_state(
    uint32_t *session_id_out,
    uint64_t *counter_out)
{
    if ((session_id_out == NULL) ||
        (counter_out == NULL))
    {
        return false;
    }

    /* Require a loaded zero seed before exposing runtime RX state. */
    if ((!esp32_gcm_nvs_load_once()) ||
        (!s_gcm_rx_ram_initialized))
    {
        return false;
    }

    *session_id_out =
        s_gcm_rx_session_id_ram;

    *counter_out =
        s_gcm_rx_counter_ram;

    return true;
}

/* Classifies an authenticated nonce and updates RAM replay state. */
static esp32_gcm_replay_result_t
esp32_gcm_replay_check_and_commit(
    uint32_t session_id,
    uint64_t counter)
{
    if ((session_id == 0U) ||
        (counter == UINT64_C(0)))
    {
        return ESP32_GCM_REPLAY_BAD_NONCE;
    }

#if ESP32_TEST_FAULT_REPLAY_DUPLICATE_LAST
    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][FAULT_INJECT][REPLAY] Forcing "
        "ESP32_GCM_REPLAY_DUPLICATE_LAST");

    return ESP32_GCM_REPLAY_DUPLICATE_LAST;
#endif

#if ESP32_TEST_FAULT_REPLAY_REJECT_OLD
    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][FAULT_INJECT][REPLAY] Forcing "
        "ESP32_GCM_REPLAY_REJECT_OLD");

    return ESP32_GCM_REPLAY_REJECT_OLD;
#endif

#if ESP32_TEST_FAULT_REPLAY_BAD_NONCE
    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][FAULT_INJECT][REPLAY] Forcing "
        "ESP32_GCM_REPLAY_BAD_NONCE");

    return ESP32_GCM_REPLAY_BAD_NONCE;
#endif

#if ESP32_TEST_FAULT_REPLAY_STORAGE_ERROR
    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][FAULT_INJECT][REPLAY] Forcing "
        "ESP32_GCM_REPLAY_STORAGE_ERROR");

    return ESP32_GCM_REPLAY_STORAGE_ERROR;
#endif

    if ((!esp32_gcm_nvs_load_once()) ||
        (!s_gcm_rx_ram_initialized))
    {
        return ESP32_GCM_REPLAY_STORAGE_ERROR;
    }

    if (s_gcm_rx_session_id_ram != 0U)
    {
        if (session_id <
            s_gcm_rx_session_id_ram)
        {
            return ESP32_GCM_REPLAY_REJECT_OLD;
        }

        if (session_id ==
            s_gcm_rx_session_id_ram)
        {
            if (counter <
                s_gcm_rx_counter_ram)
            {
                return ESP32_GCM_REPLAY_REJECT_OLD;
            }

            /* Reject older counters and identify an exact repeat of the last nonce. */
            if (counter ==
                s_gcm_rx_counter_ram)
            {
                return ESP32_GCM_REPLAY_DUPLICATE_LAST;
            }
        }
    }

    s_gcm_rx_session_id_ram =
        session_id;

    s_gcm_rx_counter_ram =
        counter;

    return ESP32_GCM_REPLAY_ACCEPT_NEW;
}

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

    esp32_gcm_nvs_close();

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

    /* Reserve the persistent TX session before initializing the RAM counter. */
    if (!esp32_gcm_nvs_reserve_tx_session(
            &session_id))
    {
        return ESP32_AES_GCM_ERR_NVS;
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
        "Reset-safe ESP32->STM32 GCM nonce session reserved: "
        "persistent_session=%08lX first_counter=%llu",
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

#if ESP32_TEST_FAULT_GCM_RX_AUTH_FAILURE
    ESP_LOGW(
        TAG,
        "[WARN][FAULT_INJECT][GCM_RX] Forcing "
        "ESP32_AES_GCM_ERR_DECRYPT_AUTH");

    mbedtls_platform_zeroize(
        plaintext,
        plaintext_capacity);

    return ESP32_AES_GCM_ERR_DECRYPT_AUTH;
#endif

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

    return ESP32_AES_GCM_SUCCESS;
}

/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


#ifndef ESP32_GCM_DEBUG_PRINT_TX_PACKET
#define ESP32_GCM_DEBUG_PRINT_TX_PACKET 0
#endif

/* Request: nonce[12] || ciphertext[169] || tag[16] = 197 bytes.
 * Response: nonce[12] || ciphertext[11] || tag[16] = 39 bytes. */
#define STM32_GCM_REQUEST_PLAINTEXT_SIZE       STM32_TELEMETRY_PLAINTEXT_SIZE
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
#define STM32_TEST_REQUEST_TYPE                 0x11U
#define STM32_TEST_ACK_TYPE                     0x20U
#define STM32_TEST_NACK_TYPE                    0x21U
#define STM32_TEST_REQUEST_PAYLOAD_LENGTH       STM32_TELEMETRY_PAYLOAD_SIZE
#define STM32_TEST_RESPONSE_PAYLOAD_LENGTH      0x01U
#define STM32_UART_RESPONSE_TIMEOUT_MS          100U


_Static_assert(
    ESP32_AES_GCM_NONCE_SIZE == 12U,
    "Interoperability protocol requires a 12-byte GCM nonce");

_Static_assert(
    ESP32_AES_GCM_TAG_SIZE == 16U,
    "Interoperability protocol requires a 16-byte GCM tag");

_Static_assert(
    STM32_GCM_REQUEST_PLAINTEXT_SIZE == 169U,
    "STM32 telemetry plaintext must be 169 bytes");

_Static_assert(
    STM32_GCM_REQUEST_CIPHERTEXT_TAG_SIZE == 185U,
    "STM32 telemetry ciphertext || tag must be 185 bytes");

_Static_assert(
    STM32_GCM_REQUEST_WIRE_SIZE == 197U,
    "STM32 telemetry wire packet must be 197 bytes");

_Static_assert(
    ESP32_GCM_RESPONSE_PLAINTEXT_SIZE == 11U,
    "ESP32 ACK/NACK plaintext must be 11 bytes");

_Static_assert(
    ESP32_GCM_RESPONSE_CIPHERTEXT_TAG_SIZE == 27U,
    "ESP32 ACK/NACK ciphertext || tag must be 27 bytes");

_Static_assert(
    ESP32_GCM_RESPONSE_WIRE_SIZE == 39U,
    "ESP32 ACK/NACK wire packet must be 39 bytes");

_Static_assert(
    sizeof(float) == 4U,
    "STM32 telemetry protocol requires 32-bit float values");

typedef enum
{
    ESP32_RESPONSE_REASON_NONE          = 0x00U, /**< Successful request; ACK. */
    ESP32_RESPONSE_REASON_BAD_CHECKSUM  = 0x01U,  
    ESP32_RESPONSE_REASON_BAD_MAGIC     = 0x02U,  
    ESP32_RESPONSE_REASON_BAD_FORMAT    = 0x03U,  
    ESP32_RESPONSE_REASON_GCM_AUTH      = 0x04U,  
    ESP32_RESPONSE_REASON_REPLAY        = 0x05U,  
    ESP32_RESPONSE_REASON_STORAGE       = 0x06U  

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

static stm32_telemetry_packet_t s_latest_telemetry = {0};
static portMUX_TYPE s_latest_telemetry_mux = portMUX_INITIALIZER_UNLOCKED;

/* Publishes a validated STM32 snapshot to the shared cache. */
static void runtime_store_latest_telemetry(
    const stm32_telemetry_packet_t *telemetry)
{
    if ((telemetry == NULL) || (!telemetry->valid))
    {
        return;
    }

    /* Update the complete snapshot under the telemetry lock. */
    portENTER_CRITICAL(&s_latest_telemetry_mux);

    s_latest_telemetry = *telemetry;

    portEXIT_CRITICAL(&s_latest_telemetry_mux);
}

/* Copies the latest validated STM32 snapshot. */
bool stm32_telemetry_get_latest(stm32_telemetry_packet_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    /* Hold the cache lock while copying data and validity together. */
    portENTER_CRITICAL(&s_latest_telemetry_mux);

    *out = s_latest_telemetry;
    bool valid = s_latest_telemetry.valid;

    portEXIT_CRITICAL(&s_latest_telemetry_mux);

    return valid;
}

static stm32_telemetry_packet_t s_latest_telemetry;
;

static bool s_last_app_transaction_valid = false;
static uint32_t s_last_app_session_id = 0U;
static uint32_t s_last_app_sequence = 0U;
static uint32_t s_app_duplicate_counter = 0U;

#if ESP32_TEST_DROP_ONE_SUCCESS_ACK
static bool s_test_success_ack_dropped = false;
#endif


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

/* Decodes an unsigned 16-bit telemetry field. */
static uint16_t runtime_load_u16_le(
    const uint8_t *data)
{
    if (data == NULL)
    {
        return 0U;
    }

    /* Use byte shifts so the source buffer need not be aligned. */
    return (uint16_t)(
        ((uint16_t)data[0] << 0U) |
        ((uint16_t)data[1] << 8U));
}

/* Decodes a signed 16-bit telemetry field. */
static int16_t runtime_load_i16_le(
    const uint8_t *data)
{
    /* Reuse the unsigned byte decode before applying the signed interpretation. */
    return (int16_t)runtime_load_u16_le(data);
}

/* Decodes an unsigned 32-bit telemetry field. */
static uint32_t runtime_load_u32_le(
    const uint8_t *data)
{
    if (data == NULL)
    {
        return 0U;
    }

    /* Assemble little-endian bytes without an unaligned word access. */
    return ((uint32_t)data[0] << 0U)  |
           ((uint32_t)data[1] << 8U)  |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/* Decodes a little-endian telemetry float. */
static float runtime_load_float_le(
    const uint8_t *data)
{
    uint32_t bits;
    float value = 0.0f;

    if (data == NULL)
    {
        return 0.0f;
    }

    bits = runtime_load_u32_le(data);

    /* Copy decoded bits into the float without pointer aliasing. */
    memcpy(
        &value,
        &bits,
        sizeof(value));

    return value;
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

#if ESP32_TEST_FAULT_FRAME_BAD_MAGIC
    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][FAULT_INJECT][FRAME] Forcing "
        "ESP32_RESPONSE_REASON_BAD_MAGIC");

    return ESP32_RESPONSE_REASON_BAD_MAGIC;
#endif

#if ESP32_TEST_FAULT_FRAME_BAD_FORMAT
    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][FAULT_INJECT][FRAME] Forcing "
        "ESP32_RESPONSE_REASON_BAD_FORMAT");

    return ESP32_RESPONSE_REASON_BAD_FORMAT;
#endif

#if ESP32_TEST_FAULT_FRAME_BAD_CHECKSUM
    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][FAULT_INJECT][FRAME] Forcing "
        "ESP32_RESPONSE_REASON_BAD_CHECKSUM");

    return ESP32_RESPONSE_REASON_BAD_CHECKSUM;
#endif

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

/* Decodes the fixed STM32 sensor snapshot layout. */
static bool runtime_parse_telemetry_frame(
    const uint8_t *frame,
    size_t frame_len,
    uint32_t sequence,
    stm32_telemetry_packet_t *out)
{
    if ((frame == NULL) ||
        (out == NULL) ||
        (frame_len != STM32_TELEMETRY_PLAINTEXT_SIZE))
    {
        return false;
    }

    /* Verify each sensor identifier before reading its indexed fields. */
    if ((frame[STM32_TELEM_BME680_NAME_LEN_INDEX] !=
            STM32_TELEM_BME680_NAME_LENGTH) ||
        (memcmp(
            &frame[STM32_TELEM_BME680_NAME_INDEX],
            "bme680",
            STM32_TELEM_BME680_NAME_LENGTH) != 0))
    {
        return false;
    }

    if ((frame[STM32_TELEM_MAX31865_NAME_LEN_INDEX] !=
            STM32_TELEM_MAX31865_NAME_LENGTH) ||
        (memcmp(
            &frame[STM32_TELEM_MAX31865_NAME_INDEX],
            "max31865",
            STM32_TELEM_MAX31865_NAME_LENGTH) != 0))
    {
        return false;
    }

    if ((frame[STM32_TELEM_DS18B20_NAME_LEN_INDEX] !=
            STM32_TELEM_DS18B20_NAME_LENGTH) ||
        (memcmp(
            &frame[STM32_TELEM_DS18B20_NAME_INDEX],
            "ds18b20x5",
            STM32_TELEM_DS18B20_NAME_LENGTH) != 0))
    {
        return false;
    }

    if ((frame[STM32_TELEM_PM25_NAME_LEN_INDEX] !=
            STM32_TELEM_PM25_NAME_LENGTH) ||
        (memcmp(
            &frame[STM32_TELEM_PM25_NAME_INDEX],
            "pm2.5",
            STM32_TELEM_PM25_NAME_LENGTH) != 0))
    {
        return false;
    }

    if ((frame[STM32_TELEM_MPU9250_NAME_LEN_INDEX] !=
            STM32_TELEM_MPU9250_NAME_LENGTH) ||
        (memcmp(
            &frame[STM32_TELEM_MPU9250_NAME_INDEX],
            "mpu9250",
            STM32_TELEM_MPU9250_NAME_LENGTH) != 0))
    {
        return false;
    }

    memset(out, 0, sizeof(*out));

    out->sequence = sequence;
    out->sequence_index = STM32_TELEMETRY_SEQUENCE_INDEX;
    out->checksum_index = STM32_TELEMETRY_CHECKSUM_INDEX;

    memcpy(out->bme680.name, "bme680", sizeof("bme680"));
    out->bme680.name_index = STM32_TELEM_BME680_NAME_INDEX;

    out->bme680.temperature =
        runtime_load_float_le(
            &frame[STM32_TELEM_BME680_TEMP_INDEX]);
    out->bme680.temperature_index = STM32_TELEM_BME680_TEMP_INDEX;
    memcpy(out->bme680.temperature_unit, "C", sizeof("C"));

    out->bme680.pressure =
        runtime_load_float_le(
            &frame[STM32_TELEM_BME680_PRESSURE_INDEX]);
    out->bme680.pressure_index = STM32_TELEM_BME680_PRESSURE_INDEX;
    memcpy(out->bme680.pressure_unit, "hPa", sizeof("hPa"));

    out->bme680.humidity =
        runtime_load_float_le(
            &frame[STM32_TELEM_BME680_HUMIDITY_INDEX]);
    out->bme680.humidity_index = STM32_TELEM_BME680_HUMIDITY_INDEX;
    memcpy(out->bme680.humidity_unit, "%RH", sizeof("%RH"));

    out->bme680.iaq_score =
        runtime_load_i16_le(
            &frame[STM32_TELEM_BME680_IAQ_SCORE_INDEX]);
    out->bme680.iaq_score_index = STM32_TELEM_BME680_IAQ_SCORE_INDEX;
    memcpy(out->bme680.iaq_unit, "index", sizeof("index"));

    memcpy(
        out->bme680.iaq_label,
        &frame[STM32_TELEM_BME680_IAQ_LABEL_INDEX],
        STM32_TELEM_BME680_IAQ_LABEL_LENGTH);
    out->bme680.iaq_label[
        STM32_TELEMETRY_IAQ_LABEL_BUFFER_SIZE - 1U] = '\0';
    out->bme680.iaq_label_index = STM32_TELEM_BME680_IAQ_LABEL_INDEX;

    memcpy(out->max31865.name, "max31865", sizeof("max31865"));
    out->max31865.name_index = STM32_TELEM_MAX31865_NAME_INDEX;

    out->max31865.temperature =
        runtime_load_float_le(
            &frame[STM32_TELEM_MAX31865_TEMP_INDEX]);
    out->max31865.temperature_index = STM32_TELEM_MAX31865_TEMP_INDEX;
    memcpy(out->max31865.temperature_unit, "C", sizeof("C"));

    memcpy(out->ds18b20.name, "ds18b20x5", sizeof("ds18b20x5"));
    out->ds18b20.name_index = STM32_TELEM_DS18B20_NAME_INDEX;
    memcpy(out->ds18b20.temperature_unit, "C", sizeof("C"));

    out->ds18b20.temperature[0] =
        runtime_load_float_le(&frame[STM32_TELEM_DS18B20_TEMP1_INDEX]);
    out->ds18b20.temperature_index[0] = STM32_TELEM_DS18B20_TEMP1_INDEX;

    out->ds18b20.temperature[1] =
        runtime_load_float_le(&frame[STM32_TELEM_DS18B20_TEMP2_INDEX]);
    out->ds18b20.temperature_index[1] = STM32_TELEM_DS18B20_TEMP2_INDEX;

    out->ds18b20.temperature[2] =
        runtime_load_float_le(&frame[STM32_TELEM_DS18B20_TEMP3_INDEX]);
    out->ds18b20.temperature_index[2] = STM32_TELEM_DS18B20_TEMP3_INDEX;

    out->ds18b20.temperature[3] =
        runtime_load_float_le(&frame[STM32_TELEM_DS18B20_TEMP4_INDEX]);
    out->ds18b20.temperature_index[3] = STM32_TELEM_DS18B20_TEMP4_INDEX;

    out->ds18b20.temperature[4] =
        runtime_load_float_le(&frame[STM32_TELEM_DS18B20_TEMP5_INDEX]);
    out->ds18b20.temperature_index[4] = STM32_TELEM_DS18B20_TEMP5_INDEX;

    memcpy(out->pm25.name, "pm2.5", sizeof("pm2.5"));
    out->pm25.name_index = STM32_TELEM_PM25_NAME_INDEX;
    memcpy(out->pm25.concentration_unit, "ug/m3", sizeof("ug/m3"));
    memcpy(out->pm25.particle_count_unit, "count", sizeof("count"));

    out->pm25.pm1_0_standard =
        runtime_load_u16_le(&frame[STM32_TELEM_PM1_STD_INDEX]);
    out->pm25.pm1_0_standard_index = STM32_TELEM_PM1_STD_INDEX;

    out->pm25.pm2_5_standard =
        runtime_load_u16_le(&frame[STM32_TELEM_PM25_STD_INDEX]);
    out->pm25.pm2_5_standard_index = STM32_TELEM_PM25_STD_INDEX;

    out->pm25.pm10_standard =
        runtime_load_u16_le(&frame[STM32_TELEM_PM10_STD_INDEX]);
    out->pm25.pm10_standard_index = STM32_TELEM_PM10_STD_INDEX;

    out->pm25.pm1_0_atmospheric =
        runtime_load_u16_le(&frame[STM32_TELEM_PM1_ATM_INDEX]);
    out->pm25.pm1_0_atmospheric_index = STM32_TELEM_PM1_ATM_INDEX;

    out->pm25.pm2_5_atmospheric =
        runtime_load_u16_le(&frame[STM32_TELEM_PM25_ATM_INDEX]);
    out->pm25.pm2_5_atmospheric_index = STM32_TELEM_PM25_ATM_INDEX;

    out->pm25.pm10_atmospheric =
        runtime_load_u16_le(&frame[STM32_TELEM_PM10_ATM_INDEX]);
    out->pm25.pm10_atmospheric_index = STM32_TELEM_PM10_ATM_INDEX;

    out->pm25.particles_0_3um =
        runtime_load_u16_le(&frame[STM32_TELEM_PARTICLE_03_INDEX]);
    out->pm25.particles_0_3um_index = STM32_TELEM_PARTICLE_03_INDEX;

    out->pm25.particles_0_5um =
        runtime_load_u16_le(&frame[STM32_TELEM_PARTICLE_05_INDEX]);
    out->pm25.particles_0_5um_index = STM32_TELEM_PARTICLE_05_INDEX;

    out->pm25.particles_1_0um =
        runtime_load_u16_le(&frame[STM32_TELEM_PARTICLE_10_INDEX]);
    out->pm25.particles_1_0um_index = STM32_TELEM_PARTICLE_10_INDEX;

    out->pm25.particles_2_5um =
        runtime_load_u16_le(&frame[STM32_TELEM_PARTICLE_25_INDEX]);
    out->pm25.particles_2_5um_index = STM32_TELEM_PARTICLE_25_INDEX;

    out->pm25.particles_5_0um =
        runtime_load_u16_le(&frame[STM32_TELEM_PARTICLE_50_INDEX]);
    out->pm25.particles_5_0um_index = STM32_TELEM_PARTICLE_50_INDEX;

    out->pm25.particles_10um =
        runtime_load_u16_le(&frame[STM32_TELEM_PARTICLE_100_INDEX]);
    out->pm25.particles_10um_index = STM32_TELEM_PARTICLE_100_INDEX;

    memcpy(out->mpu9250.name, "mpu9250", sizeof("mpu9250"));
    out->mpu9250.name_index = STM32_TELEM_MPU9250_NAME_INDEX;
    memcpy(out->mpu9250.acceleration_unit, "g", sizeof("g"));
    memcpy(out->mpu9250.gyroscope_unit, "dps", sizeof("dps"));

    out->mpu9250.ax =
        runtime_load_float_le(&frame[STM32_TELEM_MPU9250_AX_INDEX]);
    out->mpu9250.ax_index = STM32_TELEM_MPU9250_AX_INDEX;

    out->mpu9250.ay =
        runtime_load_float_le(&frame[STM32_TELEM_MPU9250_AY_INDEX]);
    out->mpu9250.ay_index = STM32_TELEM_MPU9250_AY_INDEX;

    out->mpu9250.az =
        runtime_load_float_le(&frame[STM32_TELEM_MPU9250_AZ_INDEX]);
    out->mpu9250.az_index = STM32_TELEM_MPU9250_AZ_INDEX;

    out->mpu9250.gx =
        runtime_load_float_le(&frame[STM32_TELEM_MPU9250_GX_INDEX]);
    out->mpu9250.gx_index = STM32_TELEM_MPU9250_GX_INDEX;

    out->mpu9250.gy =
        runtime_load_float_le(&frame[STM32_TELEM_MPU9250_GY_INDEX]);
    out->mpu9250.gy_index = STM32_TELEM_MPU9250_GY_INDEX;

    out->mpu9250.gz =
        runtime_load_float_le(&frame[STM32_TELEM_MPU9250_GZ_INDEX]);
    out->mpu9250.gz_index = STM32_TELEM_MPU9250_GZ_INDEX;

    out->valid = true;

    return true;
}

/* Logs the decoded STM32 sensor measurements. */
static void runtime_log_telemetry(
    const stm32_telemetry_packet_t *t)
{
    /* Log only snapshots marked valid by the telemetry parser. */
    if ((t == NULL) || (!t->valid))
    {
        return;
    }

    ESP_LOGI(
        RUNTIME_TAG,
        "[TELEMETRY][RX_OK] seq=%lu STM32->ESP32 complete sensor snapshot transferred successfully",
        (unsigned long)t->sequence);

    ESP_LOGI(
        RUNTIME_TAG,
        "[TELEMETRY][%s] temperature=%.2f %s | pressure=%.2f %s | humidity=%.2f %s | IAQ=%d %s | class=%s",
        t->bme680.name,
        (double)t->bme680.temperature,
        t->bme680.temperature_unit,
        (double)t->bme680.pressure,
        t->bme680.pressure_unit,
        (double)t->bme680.humidity,
        t->bme680.humidity_unit,
        (int)t->bme680.iaq_score,
        t->bme680.iaq_unit,
        t->bme680.iaq_label);

    ESP_LOGI(
        RUNTIME_TAG,
        "[TELEMETRY][%s] temperature=%.4f %s",
        t->max31865.name,
        (double)t->max31865.temperature,
        t->max31865.temperature_unit);

    ESP_LOGI(
        RUNTIME_TAG,
        "[TELEMETRY][%s] T1=%.4f %s | T2=%.4f %s | T3=%.4f %s | T4=%.4f %s | T5=%.4f %s",
        t->ds18b20.name,
        (double)t->ds18b20.temperature[0], t->ds18b20.temperature_unit,
        (double)t->ds18b20.temperature[1], t->ds18b20.temperature_unit,
        (double)t->ds18b20.temperature[2], t->ds18b20.temperature_unit,
        (double)t->ds18b20.temperature[3], t->ds18b20.temperature_unit,
        (double)t->ds18b20.temperature[4], t->ds18b20.temperature_unit);

    ESP_LOGI(
        RUNTIME_TAG,
        "[TELEMETRY][%s][MASS] STD={PM1.0:%u PM2.5:%u PM10:%u} %s | ATM={PM1.0:%u PM2.5:%u PM10:%u} %s",
        t->pm25.name,
        (unsigned)t->pm25.pm1_0_standard,
        (unsigned)t->pm25.pm2_5_standard,
        (unsigned)t->pm25.pm10_standard,
        t->pm25.concentration_unit,
        (unsigned)t->pm25.pm1_0_atmospheric,
        (unsigned)t->pm25.pm2_5_atmospheric,
        (unsigned)t->pm25.pm10_atmospheric,
        t->pm25.concentration_unit);

    ESP_LOGI(
        RUNTIME_TAG,
        "[TELEMETRY][%s][PARTICLES] >0.3um=%u | >0.5um=%u | >1.0um=%u | >2.5um=%u | >5.0um=%u | >10um=%u | unit=%s",
        t->pm25.name,
        (unsigned)t->pm25.particles_0_3um,
        (unsigned)t->pm25.particles_0_5um,
        (unsigned)t->pm25.particles_1_0um,
        (unsigned)t->pm25.particles_2_5um,
        (unsigned)t->pm25.particles_5_0um,
        (unsigned)t->pm25.particles_10um,
        t->pm25.particle_count_unit);

    ESP_LOGI(
        RUNTIME_TAG,
        "[TELEMETRY][%s] ACCEL={X:%.3f Y:%.3f Z:%.3f} %s | GYRO={X:%.3f Y:%.3f Z:%.3f} %s",
        t->mpu9250.name,
        (double)t->mpu9250.ax,
        (double)t->mpu9250.ay,
        (double)t->mpu9250.az,
        t->mpu9250.acceleration_unit,
        (double)t->mpu9250.gx,
        (double)t->mpu9250.gy,
        (double)t->mpu9250.gz,
        t->mpu9250.gyroscope_unit);
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

/* Authenticates STM32 telemetry and returns encrypted responses. */
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

    uint32_t stored_rx_session_id = 0U;
    uint64_t stored_rx_counter = UINT64_C(0);
    bool stored_rx_state_valid = false;

    uint32_t packet_counter = 0U;
    uint32_t success_cycle_counter = 0U;
    uint32_t auth_failure_counter = 0U;
    uint32_t ack_counter = 0U;
    uint32_t nack_counter = 0U;
    uint32_t duplicate_counter = 0U;
    uint32_t replay_reject_counter = 0U;

    esp32_response_reason_t response_reason;
    esp32_gcm_replay_result_t replay_result;
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
        "[INFO][LINK][INIT] uptime_ms=%lu AES-GCM UART rx=197B tx=39B plaintext_rx=169B auth-before-parse",
        (unsigned long)esp_log_timestamp());

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
        stored_rx_session_id = 0U;
        stored_rx_counter = UINT64_C(0);
        stored_rx_state_valid = false;
        response_reason =
            ESP32_RESPONSE_REASON_BAD_FORMAT;

        replay_result =
            ESP32_GCM_REPLAY_BAD_NONCE;

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
                "[ERROR][LINK][AUTH] uptime_ms=%lu status=%d total=%lu",
                (unsigned long)esp_log_timestamp(),
                (int)status,
                (unsigned long)auth_failure_counter);

            /* Authentication failed; send the GCM NACK with sequence zero. */
            if (runtime_send_encrypted_response(
                    runtime,
                    0U,
                    ESP32_RESPONSE_REASON_GCM_AUTH))
            {
                nack_counter++;
            }

            continue;
        }

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
                "[WARN][LINK][FRAME_REJECT] uptime_ms=%lu seq=%lu reason=%u",
                (unsigned long)esp_log_timestamp(),
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

        stored_rx_state_valid =
            esp32_gcm_get_rx_replay_state(
                &stored_rx_session_id,
                &stored_rx_counter);

        if (!stored_rx_state_valid)
        {
            ESP_LOGW(
                RUNTIME_TAG,
                "[WARN][LINK][REPLAY_STATE] Could not read ESP32 RAM RX replay "
                "state | STM32_INCOMING: session=%08lX counter=%llu",
                (unsigned long)session_id,
                (unsigned long long)nonce_counter);
        }

        replay_result =
            esp32_gcm_replay_check_and_commit(
                session_id,
                nonce_counter);

        if (replay_result ==
            ESP32_GCM_REPLAY_DUPLICATE_LAST)
        {
            duplicate_counter++;

            if (stored_rx_state_valid)
            {
                ESP_LOGW(
                    RUNTIME_TAG,
                    "[WARN][LINK][REPLAY_DUP] Authenticated duplicate request suppressed: "
                    "ESP32_RAM_RX={session=%08lX counter=%llu} "
                    "STM32_INCOMING={session=%08lX counter=%llu} "
                    "sequence=%lu duplicates=%lu",
                    (unsigned long)stored_rx_session_id,
                    (unsigned long long)stored_rx_counter,
                    (unsigned long)session_id,
                    (unsigned long long)nonce_counter,
                    (unsigned long)sequence,
                    (unsigned long)duplicate_counter);
            }
            else
            {
                ESP_LOGW(
                    RUNTIME_TAG,
                    "[WARN][LINK][REPLAY_DUP] Authenticated duplicate request suppressed: "
                    "STM32_INCOMING={session=%08lX counter=%llu} "
                    "sequence=%lu duplicates=%lu "
                    "(ESP32 RAM RX state unavailable)",
                    (unsigned long)session_id,
                    (unsigned long long)nonce_counter,
                    (unsigned long)sequence,
                    (unsigned long)duplicate_counter);
            }

            if (runtime_send_encrypted_response(
                    runtime,
                    sequence,
                    ESP32_RESPONSE_REASON_NONE))
            {
                ack_counter++;
            }

            continue;
        }

        if ((replay_result ==
                ESP32_GCM_REPLAY_REJECT_OLD) ||
            (replay_result ==
                ESP32_GCM_REPLAY_BAD_NONCE))
        {
            replay_reject_counter++;

            if (stored_rx_state_valid)
            {
                ESP_LOGW(
                    RUNTIME_TAG,
                    "[WARN][LINK][REPLAY_REJECT] Authenticated replay rejected: result=%s "
                    "ESP32_RAM_RX={session=%08lX counter=%llu} "
                    "STM32_INCOMING={session=%08lX counter=%llu} "
                    "sequence=%lu rejections=%lu",
                    (replay_result == ESP32_GCM_REPLAY_REJECT_OLD) ?
                        "REJECT_OLD" :
                        "BAD_NONCE",
                    (unsigned long)stored_rx_session_id,
                    (unsigned long long)stored_rx_counter,
                    (unsigned long)session_id,
                    (unsigned long long)nonce_counter,
                    (unsigned long)sequence,
                    (unsigned long)replay_reject_counter);
            }
            else
            {
                ESP_LOGW(
                    RUNTIME_TAG,
                    "[WARN][LINK][REPLAY_REJECT] Authenticated replay rejected: result=%s "
                    "STM32_INCOMING={session=%08lX counter=%llu} "
                    "sequence=%lu rejections=%lu "
                    "(ESP32 RAM RX state unavailable)",
                    (replay_result == ESP32_GCM_REPLAY_REJECT_OLD) ?
                        "REJECT_OLD" :
                        "BAD_NONCE",
                    (unsigned long)session_id,
                    (unsigned long long)nonce_counter,
                    (unsigned long)sequence,
                    (unsigned long)replay_reject_counter);
            }

            if (runtime_send_encrypted_response(
                    runtime,
                    sequence,
                    ESP32_RESPONSE_REASON_REPLAY))
            {
                nack_counter++;
            }

            continue;
        }

        if (replay_result ==
            ESP32_GCM_REPLAY_STORAGE_ERROR)
        {
            ESP_LOGE(
                RUNTIME_TAG,
                "[ERROR][LINK][REPLAY_STATE] uptime_ms=%lu state=UNAVAILABLE",
                (unsigned long)esp_log_timestamp());

            if (runtime_send_encrypted_response(
                    runtime,
                    sequence,
                    ESP32_RESPONSE_REASON_STORAGE))
            {
                nack_counter++;
            }

            continue;
        }

if (s_last_app_transaction_valid &&
    (s_last_app_session_id == session_id) &&
    (s_last_app_sequence == sequence))
{
    s_app_duplicate_counter++;

    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][LINK][APP_DUP] "
        "Authenticated telemetry retry suppressed | "
        "session=%08lX seq=%lu nonce_counter=%llu "
        "app_duplicates=%lu | "
        "payload NOT processed again; ACK will be resent",
        (unsigned long)session_id,
        (unsigned long)sequence,
        (unsigned long long)nonce_counter,
        (unsigned long)s_app_duplicate_counter);

    if (runtime_send_encrypted_response(
            runtime,
            sequence,
            ESP32_RESPONSE_REASON_NONE))
    {
        ack_counter++;
        success_cycle_counter++;

        ESP_LOGI(
            RUNTIME_TAG,
            "[INFO][LINK][APP_DUP_ACK] "
            "seq=%lu duplicate transaction ACK resent",
            (unsigned long)sequence);
    }

    continue;
}

stm32_telemetry_packet_t parsed_telemetry = {0};

if (!runtime_parse_telemetry_frame(
        decrypted_plaintext,
        decrypted_len,
        sequence,
        &parsed_telemetry))
{
    ESP_LOGW(
        RUNTIME_TAG,
        "[WARN][TELEMETRY][PARSE] "
        "Authenticated telemetry payload structure invalid seq=%lu",
        (unsigned long)sequence);

    if (runtime_send_encrypted_response(
            runtime,
            sequence,
            ESP32_RESPONSE_REASON_BAD_FORMAT))
    {
        nack_counter++;
    }

    continue;
}

runtime_store_latest_telemetry(&parsed_telemetry);

runtime_log_telemetry(&parsed_telemetry);

			s_last_app_session_id = session_id;
			s_last_app_sequence = sequence;
			s_last_app_transaction_valid = true;  

        ESP_LOGI(
    RUNTIME_TAG,
    "[INFO][LINK][RX] uptime_ms=%lu seq=%lu source=STM32 | "
    "payload=REDACTED | wire=%uB | "
    "auth=OK | decrypt=OK | replay=NEW",
    (unsigned long)esp_log_timestamp(),
    (unsigned long)sequence,
    (unsigned int)wire_len);

#if ESP32_TEST_DROP_ONE_SUCCESS_ACK

if (!s_test_success_ack_dropped)
{
    s_test_success_ack_dropped = true;

    ESP_LOGW(
        RUNTIME_TAG,
        "[FAULT_INJECT][ACK_DROP] "
        "Suppressing one successful ACK after processing | "
        "session=%08lX seq=%lu nonce_counter=%llu",
        (unsigned long)session_id,
        (unsigned long)sequence,
        (unsigned long long)nonce_counter);

    continue;
}

#endif /* ESP32_TEST_DROP_ONE_SUCCESS_ACK */


if (runtime_send_encrypted_response(
        runtime,
        sequence,
        ESP32_RESPONSE_REASON_NONE))
{
    ack_counter++;
    success_cycle_counter++;

    ESP_LOGI(
        RUNTIME_TAG,
        "[INFO][LINK][TX] uptime_ms=%lu seq=%lu ack=ENCRYPTED | wire=%uB | "
        "route=UART->STM32 | status=SENT",
        (unsigned long)esp_log_timestamp(),
        (unsigned long)sequence,
        (unsigned int)ESP32_GCM_RESPONSE_WIRE_SIZE);

    if ((ESP32_GCM_HEALTH_EVERY_N_OK != 0U) &&
        ((success_cycle_counter %
          ESP32_GCM_HEALTH_EVERY_N_OK) == 0U))
    {
        ESP_LOGI(
            RUNTIME_TAG,
            "[INFO][LINK][HEALTH] uptime_ms=%lu cycles=%lu rx_packets=%lu last_seq=%lu "
            "rx={session=%08lX counter=%llu} "
            "tx={session=%08lX counter=%llu} "
            "ack=%lu nack=%lu auth_fail=%lu "
            "duplicate=%lu app_duplicate=%lu replay_reject=%lu",
            (unsigned long)esp_log_timestamp(),
            (unsigned long)success_cycle_counter,
            (unsigned long)packet_counter,
            (unsigned long)sequence,
            (unsigned long)session_id,
            (unsigned long long)nonce_counter,
            (unsigned long)runtime->gcm_ctx->nonce_state.session_id,
            (unsigned long long)
                runtime->gcm_ctx->nonce_state.current_counter,
            (unsigned long)ack_counter,
            (unsigned long)nack_counter,
            (unsigned long)auth_failure_counter,
            (unsigned long)duplicate_counter,
            (unsigned long)s_app_duplicate_counter,
            (unsigned long)replay_reject_counter);
    }
}
else
{
    ESP_LOGE(
        RUNTIME_TAG,
        "[ERROR][LINK][TX] uptime_ms=%lu "
        "encrypted ACK transmission failed seq=%lu",
        (unsigned long)esp_log_timestamp(),
        (unsigned long)sequence);
}
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
