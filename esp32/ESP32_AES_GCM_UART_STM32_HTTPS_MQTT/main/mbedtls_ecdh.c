/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#define ESP32_ECDH_DEFINE_PROVISIONED_KEYS 1
#include "mbedtls_ecdh.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_log_buffer.h"

#include "mbedtls/build_info.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/sha256.h"

#if !defined(MBEDTLS_ENTROPY_C)
#error "MBEDTLS_ENTROPY_C must be enabled"
#endif

#if !defined(MBEDTLS_CTR_DRBG_C)
#error "MBEDTLS_CTR_DRBG_C must be enabled"
#endif

#if !defined(MBEDTLS_ECP_C)
#error "MBEDTLS_ECP_C must be enabled"
#endif

#if !defined(MBEDTLS_ECDH_C)
#error "MBEDTLS_ECDH_C must be enabled"
#endif

#if !defined(MBEDTLS_SHA256_C)
#error "MBEDTLS_SHA256_C must be enabled"
#endif

#if !defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED)
#error "MBEDTLS_ECP_DP_SECP256R1_ENABLED must be enabled"
#endif

static const char *TAG = "ESP32_ECDH";

static const unsigned char s_drbg_personalization[] =
    "ESP32-LOCAL-ECDH-SECP256R1-V1";

/* Logs a failed Mbed TLS operation and error code. */
static void log_mbedtls_error(
    const char *operation,
    int rc)
{
    if (operation == NULL)
    {
        operation = "Mbed TLS operation";
    }

    /* Display the negative library result in hexadecimal. */
    ESP_LOGE(
        TAG,
        "%s failed: rc=-0x%04X",
        operation,
        (unsigned int)(-rc));
}

/* Logs a labeled byte buffer for crypto diagnostics. */
static void log_hex_buffer(
    const char *label,
    const uint8_t *buffer,
    size_t length)
{
    /* Skip empty buffers before calling the hexadecimal log helper. */
    if ((label == NULL) ||
        (buffer == NULL) ||
        (length == 0U))
    {
        return;
    }

    ESP_LOGI(
        TAG,
        "%s (%u bytes):",
        label,
        (unsigned int)length);

    ESP_LOG_BUFFER_HEX_LEVEL(
        TAG,
        buffer,
        length,
        ESP_LOG_INFO);
}

/* Compares two byte buffers without an early mismatch exit. */
static bool secure_buffers_equal(
    const uint8_t *first,
    const uint8_t *second,
    size_t length)
{
    uint8_t difference = 0U;

    if ((first == NULL) ||
        (second == NULL))
    {
        return false;
    }

    /* Accumulate all byte differences before deciding equality. */
    for (size_t i = 0U; i < length; ++i)
    {
        difference |=
            (uint8_t)(first[i] ^ second[i]);
    }

    return difference == 0U;
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

/* Initializes one ECDH party's scalar and point objects. */
static void ecdh_party_init(
    esp32_ecdh_party_t *party)
{
    if (party == NULL)
    {
        return;
    }

    mbedtls_mpi_init(
        &party->private_key);

    mbedtls_ecp_point_init(
        &party->public_key);

    /* Clear exported key buffers when creating the party state. */
    mbedtls_platform_zeroize(
        party->private_key_bytes,
        sizeof(party->private_key_bytes));

    mbedtls_platform_zeroize(
        party->public_key_bytes,
        sizeof(party->public_key_bytes));

    party->public_key_len = 0U;
}

/* Clears an ECDH party's exported key bytes. */
static void ecdh_party_wipe_exported(
    esp32_ecdh_party_t *party)
{
    if (party == NULL)
    {
        return;
    }

    /* Zeroize the exported private scalar and public coordinates. */
    mbedtls_platform_zeroize(
        party->private_key_bytes,
        sizeof(party->private_key_bytes));

    mbedtls_platform_zeroize(
        party->public_key_bytes,
        sizeof(party->public_key_bytes));

    party->public_key_len = 0U;
}

/* Recreates an ECDH party's Mbed TLS objects. */
static void ecdh_party_reset(
    esp32_ecdh_party_t *party)
{
    if (party == NULL)
    {
        return;
    }

    /* Release old key objects and exported bytes before reinitializing them. */
    mbedtls_mpi_free(
        &party->private_key);

    mbedtls_ecp_point_free(
        &party->public_key);

    ecdh_party_wipe_exported(
        party);

    mbedtls_mpi_init(
        &party->private_key);

    mbedtls_ecp_point_init(
        &party->public_key);
}

/* Releases an ECDH party and clears exported keys. */
static void ecdh_party_free(
    esp32_ecdh_party_t *party)
{
    if (party == NULL)
    {
        return;
    }

    mbedtls_mpi_free(
        &party->private_key);

    mbedtls_ecp_point_free(
        &party->public_key);

    /* Wipe exported buffers after freeing the scalar and point objects. */
    ecdh_party_wipe_exported(
        party);
}

/* Clears computed ECDH secrets and their readiness flag. */
static void ecdh_clear_shared_state(
    esp32_ecdh_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    /* Zeroize both directional secret buffers. */
    mbedtls_platform_zeroize(
        ctx->computed_secret,
        sizeof(ctx->computed_secret));

    mbedtls_platform_zeroize(
        ctx->computed_secret_receiver,
        sizeof(ctx->computed_secret_receiver));

    ctx->computed_secret_len = 0U;
    ctx->computed_secret_receiver_len = 0U;

    ctx->shared_secrets_computed = false;
}

/* Clears derived AES/MAC keys and their readiness flag. */
static void ecdh_clear_derived_state(
    esp32_ecdh_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    mbedtls_platform_zeroize(
        ctx->aes_key,
        sizeof(ctx->aes_key));

    mbedtls_platform_zeroize(
        ctx->aes_key_receiver,
        sizeof(ctx->aes_key_receiver));

    mbedtls_platform_zeroize(
        ctx->mac_key,
        sizeof(ctx->mac_key));

    mbedtls_platform_zeroize(
        ctx->mac_key_receiver,
        sizeof(ctx->mac_key_receiver));

    /* Invalidate key lengths after wiping all directional key buffers. */
    ctx->aes_key_len = 0U;
    ctx->aes_key_receiver_len = 0U;
    ctx->mac_key_len = 0U;
    ctx->mac_key_receiver_len = 0U;

    ctx->derived_keys_ready = false;
}

/* Releases ECDH, entropy and DRBG resources. */
void esp32_ecdh_free(
    esp32_ecdh_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    /* Free library objects only when their initialization flag is set. */
    if (ctx->objects_initialized)
    {
        ecdh_party_free(
            &ctx->sender);

        ecdh_party_free(
            &ctx->receiver);

        mbedtls_ecp_group_free(
            &ctx->group);

        mbedtls_ctr_drbg_free(
            &ctx->ctr_drbg);

        mbedtls_entropy_free(
            &ctx->entropy);
    }

    mbedtls_platform_zeroize(
        ctx,
        sizeof(*ctx));
}

/* Initializes P-256 ECDH and seeds the platform DRBG. */
esp32_ecdh_status_t esp32_ecdh_init(
    esp32_ecdh_ctx_t *ctx)
{
    int rc;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (ctx->objects_initialized)
    {
        return ESP32_ECDH_ERR_ALREADY_INIT;
    }

    memset(
        ctx,
        0,
        sizeof(*ctx));

    mbedtls_entropy_init(
        &ctx->entropy);

    mbedtls_ctr_drbg_init(
        &ctx->ctr_drbg);

    mbedtls_ecp_group_init(
        &ctx->group);

    ecdh_party_init(
        &ctx->sender);

    ecdh_party_init(
        &ctx->receiver);

    ctx->objects_initialized = true;

    /* Reject unsupported secp256r1 support before loading the curve. */
    if (mbedtls_ecdh_can_do(
            MBEDTLS_ECP_DP_SECP256R1) != 1)
    {
        ESP_LOGE(
            TAG,
            "This Mbed TLS build does not support "
            "ECDH with secp256r1");

        esp32_ecdh_free(ctx);

        return ESP32_ECDH_ERR_CURVE_UNSUPPORTED;
    }

    rc = mbedtls_ecp_group_load(
        &ctx->group,
        MBEDTLS_ECP_DP_SECP256R1);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_group_load",
            rc);

        esp32_ecdh_free(ctx);

        return ESP32_ECDH_ERR_CURVE_LOAD;
    }

    rc = mbedtls_ctr_drbg_seed(
        &ctx->ctr_drbg,
        mbedtls_entropy_func,
        &ctx->entropy,
        s_drbg_personalization,
        sizeof(s_drbg_personalization) - 1U);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ctr_drbg_seed",
            rc);

        esp32_ecdh_free(ctx);

        return ESP32_ECDH_ERR_DRBG_SEED;
    }

    ctx->drbg_seeded = true;

    ESP_LOGI(
        TAG,
        "Mbed TLS entropy context initialized");

    ESP_LOGI(
        TAG,
        "CTR-DRBG seeded from ESP32 platform entropy");

    ESP_LOGI(
        TAG,
        "secp256r1 curve initialized");

    return ESP32_ECDH_SUCCESS;
}

/* Validates a keypair and exports its fixed-size byte representation. */
static esp32_ecdh_status_t export_and_validate_keypair(
    esp32_ecdh_ctx_t *ctx,
    esp32_ecdh_party_t *party)
{
    uint8_t public_key_uncompressed[
        ESP32_ECDH_PUBLIC_KEY_UNCOMPRESSED_SIZE];

    size_t uncompressed_len = 0U;
    int rc;

    if ((ctx == NULL) ||
        (party == NULL))
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    mbedtls_platform_zeroize(
        public_key_uncompressed,
        sizeof(public_key_uncompressed));

    rc = mbedtls_ecp_check_privkey(
        &ctx->group,
        &party->private_key);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_check_privkey",
            rc);

        return ESP32_ECDH_ERR_PRIVATE_KEY_CHECK;
    }

    rc = mbedtls_ecp_check_pubkey(
        &ctx->group,
        &party->public_key);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_check_pubkey",
            rc);

        return ESP32_ECDH_ERR_PUBLIC_KEY_CHECK;
    }

    rc = mbedtls_mpi_write_binary(
        &party->private_key,
        party->private_key_bytes,
        sizeof(party->private_key_bytes));

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_mpi_write_binary(private)",
            rc);

        ecdh_party_wipe_exported(
            party);

        return ESP32_ECDH_ERR_PRIVATE_EXPORT;
    }

    rc = mbedtls_ecp_point_write_binary(
        &ctx->group,
        &party->public_key,
        MBEDTLS_ECP_PF_UNCOMPRESSED,
        &uncompressed_len,
        public_key_uncompressed,
        sizeof(public_key_uncompressed));

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_point_write_binary",
            rc);

        mbedtls_platform_zeroize(
            public_key_uncompressed,
            sizeof(public_key_uncompressed));

        ecdh_party_wipe_exported(
            party);

        return ESP32_ECDH_ERR_PUBLIC_EXPORT;
    }

    /* Check the uncompressed point prefix before copying raw X and Y. */
    if ((uncompressed_len !=
            ESP32_ECDH_PUBLIC_KEY_UNCOMPRESSED_SIZE) ||
        (public_key_uncompressed[0] != 0x04U))
    {
        ESP_LOGE(
            TAG,
            "Unexpected public-key format: "
            "length=%u prefix=0x%02X",
            (unsigned int)uncompressed_len,
            public_key_uncompressed[0]);

        mbedtls_platform_zeroize(
            public_key_uncompressed,
            sizeof(public_key_uncompressed));

        ecdh_party_wipe_exported(
            party);

        return ESP32_ECDH_ERR_PUBLIC_LENGTH;
    }

    memcpy(
        party->public_key_bytes,
        &public_key_uncompressed[1],
        ESP32_ECDH_PUBLIC_KEY_SIZE);

    party->public_key_len =
        ESP32_ECDH_PUBLIC_KEY_SIZE;

    mbedtls_platform_zeroize(
        public_key_uncompressed,
        sizeof(public_key_uncompressed));

    return ESP32_ECDH_SUCCESS;
}

#define ESP32_ECDH_EXPORT_BYTES_PER_LINE  8U

/* Prints key bytes in provisioning-array format. */
static void print_key_as_c_array(
    const char *name,
    const uint8_t *key,
    size_t key_len)
{
    if ((name == NULL) ||
        (key == NULL) ||
        (key_len == 0U))
    {
        return;
    }

    printf(
        "\r\n[KEY EXPORT] %s [%u bytes]\r\n",
        name,
        (unsigned int)key_len);

    printf(
        "static const uint8_t %s[%u] = {\r\n",
        name,
        (unsigned int)key_len);

    for (size_t i = 0U; i < key_len; ++i)
    {
        /* Wrap output at the configured byte count per line. */
        if ((i % ESP32_ECDH_EXPORT_BYTES_PER_LINE) == 0U)
        {
            printf("    ");
        }

        printf(
            "0x%02X%s",
            (unsigned int)key[i],
            ((i + 1U) < key_len) ? ", " : "");

        if ((((i + 1U) % ESP32_ECDH_EXPORT_BYTES_PER_LINE) == 0U) ||
            ((i + 1U) == key_len))
        {
            printf("\r\n");
        }
    }

    printf("};\r\n");
    fflush(stdout);
}

/*
 * Import one fixed secp256r1 keypair from raw big-endian arrays.
 *
 * private_key[32]
 * public_key[64] = X[32] || Y[32]
 *
 * The function imports and validates both values. The existing
 * sender/receiver shared-secret comparison later detects a mismatched
 * private/public pair before derived keys are accepted.
 */
/* Imports and validates a provisioned scalar and public point. */
static esp32_ecdh_status_t import_provisioned_keypair(
    esp32_ecdh_ctx_t *ctx,
    esp32_ecdh_party_t *party,
    const uint8_t private_key[ESP32_ECDH_PRIVATE_KEY_SIZE],
    const uint8_t public_key[ESP32_ECDH_PUBLIC_KEY_SIZE],
    const char *party_name)
{
    uint8_t public_key_uncompressed[
        ESP32_ECDH_PUBLIC_KEY_UNCOMPRESSED_SIZE];

    esp32_ecdh_status_t status = ESP32_ECDH_SUCCESS;
    int rc;

    if ((ctx == NULL) ||
        (party == NULL) ||
        (private_key == NULL) ||
        (public_key == NULL) ||
        (party_name == NULL))
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (buffer_is_all_zero(
            private_key,
            ESP32_ECDH_PRIVATE_KEY_SIZE) ||
        buffer_is_all_zero(
            public_key,
            ESP32_ECDH_PUBLIC_KEY_SIZE))
    {
        ESP_LOGE(
            TAG,
            "%s provisioned keypair contains an all-zero value",
            party_name);

        return ESP32_ECDH_ERR_PROVISIONED_KEY_EMPTY;
    }

    ecdh_party_reset(party);

    mbedtls_platform_zeroize(
        public_key_uncompressed,
        sizeof(public_key_uncompressed));

    rc = mbedtls_mpi_read_binary(
        &party->private_key,
        private_key,
        ESP32_ECDH_PRIVATE_KEY_SIZE);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_mpi_read_binary(provisioned private)",
            rc);

        status = ESP32_ECDH_ERR_PROVISIONED_KEY_IMPORT;
        goto cleanup;
    }

    /* Add the uncompressed point prefix before importing raw coordinates. */
    public_key_uncompressed[0] = 0x04U;

    memcpy(
        &public_key_uncompressed[1],
        public_key,
        ESP32_ECDH_PUBLIC_KEY_SIZE);

    rc = mbedtls_ecp_point_read_binary(
        &ctx->group,
        &party->public_key,
        public_key_uncompressed,
        sizeof(public_key_uncompressed));

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_point_read_binary(provisioned public)",
            rc);

        status = ESP32_ECDH_ERR_PROVISIONED_KEY_IMPORT;
        goto cleanup;
    }

    rc = mbedtls_ecp_check_privkey(
        &ctx->group,
        &party->private_key);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_check_privkey(provisioned)",
            rc);

        status = ESP32_ECDH_ERR_PRIVATE_KEY_CHECK;
        goto cleanup;
    }

    rc = mbedtls_ecp_check_pubkey(
        &ctx->group,
        &party->public_key);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_check_pubkey(provisioned)",
            rc);

        status = ESP32_ECDH_ERR_PUBLIC_KEY_CHECK;
        goto cleanup;
    }

    memcpy(
        party->private_key_bytes,
        private_key,
        ESP32_ECDH_PRIVATE_KEY_SIZE);

    memcpy(
        party->public_key_bytes,
        public_key,
        ESP32_ECDH_PUBLIC_KEY_SIZE);

    party->public_key_len =
        ESP32_ECDH_PUBLIC_KEY_SIZE;

cleanup:

    mbedtls_platform_zeroize(
        public_key_uncompressed,
        sizeof(public_key_uncompressed));

    if (status != ESP32_ECDH_SUCCESS)
    {
        ecdh_party_reset(party);
    }

    return status;
}

/* Loads the provisioned STM32-to-ESP32 receive material. */
static esp32_ecdh_status_t import_cross_device_receiver_material(
    esp32_ecdh_ctx_t *ctx)
{
    uint8_t peer_public_uncompressed[
        ESP32_ECDH_PUBLIC_KEY_UNCOMPRESSED_SIZE];

    esp32_ecdh_status_t status = ESP32_ECDH_SUCCESS;
    int rc;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

#if !ESP32_ECDH_PROVISIONED_KEYS_PRESENT
    ESP_LOGE(
        TAG,
        "Cross-device receiver test requires provisioned keys");

    return ESP32_ECDH_ERR_PROVISIONED_KEY_EMPTY;
#else

    if (buffer_is_all_zero(
            esp32_provisioned_receiver_private_key,
            ESP32_ECDH_PRIVATE_KEY_SIZE) ||
        buffer_is_all_zero(
            stm32_provisioned_sender_public_key,
            ESP32_ECDH_PUBLIC_KEY_SIZE))
    {
        ESP_LOGE(
            TAG,
            "ESP32 receiver private key or STM32 sender public key "
            "is all zero");

        return ESP32_ECDH_ERR_PROVISIONED_KEY_EMPTY;
    }

    ecdh_party_reset(&ctx->receiver);
    ecdh_party_reset(&ctx->sender);

    mbedtls_platform_zeroize(
        peer_public_uncompressed,
        sizeof(peer_public_uncompressed));

    /* Pair the ESP32 receiver scalar with the STM32 sender public point. */
    rc = mbedtls_mpi_read_binary(
        &ctx->receiver.private_key,
        esp32_provisioned_receiver_private_key,
        ESP32_ECDH_PRIVATE_KEY_SIZE);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_mpi_read_binary(ESP32 receiver private)",
            rc);

        status = ESP32_ECDH_ERR_PROVISIONED_KEY_IMPORT;
        goto cleanup;
    }

    rc = mbedtls_ecp_check_privkey(
        &ctx->group,
        &ctx->receiver.private_key);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_check_privkey(ESP32 receiver)",
            rc);

        status = ESP32_ECDH_ERR_PRIVATE_KEY_CHECK;
        goto cleanup;
    }

    peer_public_uncompressed[0] = 0x04U;

    memcpy(
        &peer_public_uncompressed[1],
        stm32_provisioned_sender_public_key,
        ESP32_ECDH_PUBLIC_KEY_SIZE);

    rc = mbedtls_ecp_point_read_binary(
        &ctx->group,
        &ctx->sender.public_key,
        peer_public_uncompressed,
        sizeof(peer_public_uncompressed));

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_point_read_binary(STM32 sender public)",
            rc);

        status = ESP32_ECDH_ERR_PROVISIONED_KEY_IMPORT;
        goto cleanup;
    }

    rc = mbedtls_ecp_check_pubkey(
        &ctx->group,
        &ctx->sender.public_key);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_check_pubkey(STM32 sender)",
            rc);

        status = ESP32_ECDH_ERR_PUBLIC_KEY_CHECK;
        goto cleanup;
    }

    memcpy(
        ctx->receiver.private_key_bytes,
        esp32_provisioned_receiver_private_key,
        ESP32_ECDH_PRIVATE_KEY_SIZE);

    memcpy(
        ctx->sender.public_key_bytes,
        stm32_provisioned_sender_public_key,
        ESP32_ECDH_PUBLIC_KEY_SIZE);

    ctx->sender.public_key_len =
        ESP32_ECDH_PUBLIC_KEY_SIZE;

    ctx->receiver.public_key_len = 0U;

cleanup:

    mbedtls_platform_zeroize(
        peer_public_uncompressed,
        sizeof(peer_public_uncompressed));

    if (status != ESP32_ECDH_SUCCESS)
    {
        ecdh_party_reset(&ctx->receiver);
        ecdh_party_reset(&ctx->sender);
    }

    return status;

#endif /* ESP32_ECDH_PROVISIONED_KEYS_PRESENT */
}

/* Generates and validates a local P-256 keypair. */
static esp32_ecdh_status_t generate_one_keypair(
    esp32_ecdh_ctx_t *ctx,
    esp32_ecdh_party_t *party,
    esp32_ecdh_status_t keygen_error)
{
    esp32_ecdh_status_t status;
    int rc;

    if ((ctx == NULL) ||
        (party == NULL))
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    /* Use the seeded CTR-DRBG for elliptic-curve key generation. */
    rc = mbedtls_ecdh_gen_public(
        &ctx->group,
        &party->private_key,
        &party->public_key,
        mbedtls_ctr_drbg_random,
        &ctx->ctr_drbg);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecdh_gen_public",
            rc);

        ecdh_party_reset(
            party);

        return keygen_error;
    }

    status = export_and_validate_keypair(
        ctx,
        party);

    if (status != ESP32_ECDH_SUCCESS)
    {
        ecdh_party_reset(
            party);

        return status;
    }

    return ESP32_ECDH_SUCCESS;
}

/* Prepares the provisioned cross-device ECDH key material. */
esp32_ecdh_status_t esp32_ecdh_generate_keypairs(
    esp32_ecdh_ctx_t *ctx)
{
    esp32_ecdh_status_t status;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if ((!ctx->objects_initialized) ||
        (!ctx->drbg_seeded))
    {
        return ESP32_ECDH_ERR_NOT_INITIALIZED;
    }

    if (ctx->keypairs_generated)
    {
        return ESP32_ECDH_ERR_ALREADY_GENERATED;
    }

    /* Clear stale shared secrets and derived keys before loading provisioning. */
    ecdh_clear_shared_state(ctx);
    ecdh_clear_derived_state(ctx);

    status =
        import_cross_device_receiver_material(ctx);

    if (status != ESP32_ECDH_SUCCESS)
    {
        ESP_LOGE(
            TAG,
            "Cross-device receiver key import failed: status=%d",
            (int)status);

        return status;
    }

    ctx->keypairs_generated = true;

    ESP_LOGI(
        TAG,
        "Cross-device RX material loaded");

    ESP_LOGI(
        TAG,
        "ECDH direction: ESP32 receiver private x "
        "STM32 sender public");

    ESP_LOGI(
        TAG,
        "No ESP32 sender keypair was generated for this test");

    ESP_LOGI(
        TAG,
        "Provisioned private/public material will not be printed");

    return ESP32_ECDH_SUCCESS;
}

/* Reports keypair state with provisioning-aware logging. */
esp32_ecdh_status_t esp32_ecdh_print_keypairs(
    const esp32_ecdh_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->keypairs_generated)
    {
        return ESP32_ECDH_ERR_KEYS_NOT_READY;
    }

/* Suppress provisioned key bytes in the configured cross-device mode. */
#if ESP32_ECDH_PROVISIONED_KEYS_PRESENT

    ESP_LOGI(
        TAG,
        "All four provisioned ECDH keys are already loaded");

    ESP_LOGI(
        TAG,
        "Keypair printing is suppressed for provisioned keys");

    return ESP32_ECDH_SUCCESS;

#else

    ESP_LOGI(
        TAG,
        "========== LOCAL ECDH KEYPAIRS ==========");

    ESP_LOGI(
        TAG,
        "----- SENDER KEYPAIR -----");

#if ESP32_ECDH_DEBUG_PRINT_PRIVATE_KEYS

    ESP_LOGW(
        TAG,
        "DEBUG ONLY: printing sender private key");

    log_hex_buffer(
        "Sender private scalar, big-endian",
        ctx->sender.private_key_bytes,
        sizeof(ctx->sender.private_key_bytes));

#else

    ESP_LOGI(
        TAG,
        "Sender private-key printing disabled");

#endif

    log_hex_buffer(
        "Sender public key, raw X||Y",
        ctx->sender.public_key_bytes,
        ctx->sender.public_key_len);

    ESP_LOGI(
        TAG,
        "----- RECEIVER KEYPAIR -----");

#if ESP32_ECDH_DEBUG_PRINT_PRIVATE_KEYS

    ESP_LOGW(
        TAG,
        "DEBUG ONLY: printing receiver private key");

    log_hex_buffer(
        "Receiver private scalar, big-endian",
        ctx->receiver.private_key_bytes,
        sizeof(ctx->receiver.private_key_bytes));

#else

    ESP_LOGI(
        TAG,
        "Receiver private-key printing disabled");

#endif

    log_hex_buffer(
        "Receiver public key, raw X||Y",
        ctx->receiver.public_key_bytes,
        ctx->receiver.public_key_len);

    ESP_LOGI(
        TAG,
        "=========================================");

    return ESP32_ECDH_SUCCESS;

#endif /* ESP32_ECDH_PROVISIONED_KEYS_PRESENT */
}

/* Computes and exports a validated P-256 shared secret. */
static esp32_ecdh_status_t compute_shared_internal(
    esp32_ecdh_ctx_t *ctx,
    const mbedtls_mpi *own_private_key,
    const mbedtls_ecp_point *peer_public_key,
    uint8_t output_secret[
        ESP32_ECDH_SHARED_SECRET_SIZE],
    size_t *output_secret_len,
    esp32_ecdh_status_t compute_error,
    const char *side_name)
{
    mbedtls_mpi shared_secret_mpi;
    esp32_ecdh_status_t status = ESP32_ECDH_SUCCESS;
    size_t mpi_size;
    int rc;

    if ((ctx == NULL) ||
        (own_private_key == NULL) ||
        (peer_public_key == NULL) ||
        (output_secret == NULL) ||
        (output_secret_len == NULL) ||
        (side_name == NULL))
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    mbedtls_mpi_init(&shared_secret_mpi);

    mbedtls_platform_zeroize(
        output_secret,
        ESP32_ECDH_SHARED_SECRET_SIZE);

    *output_secret_len = 0U;

    rc = mbedtls_ecp_check_privkey(
        &ctx->group,
        own_private_key);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_check_privkey",
            rc);

        status = ESP32_ECDH_ERR_PRIVATE_KEY_CHECK;
        goto cleanup;
    }

    /* Check the peer public point before performing scalar multiplication. */
    rc = mbedtls_ecp_check_pubkey(
        &ctx->group,
        peer_public_key);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecp_check_pubkey",
            rc);

        status = ESP32_ECDH_ERR_PUBLIC_KEY_CHECK;
        goto cleanup;
    }

    rc = mbedtls_ecdh_compute_shared(
        &ctx->group,
        &shared_secret_mpi,
        peer_public_key,
        own_private_key,
        mbedtls_ctr_drbg_random,
        &ctx->ctr_drbg);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_ecdh_compute_shared",
            rc);

        status = compute_error;
        goto cleanup;
    }

    mpi_size = mbedtls_mpi_size(
        &shared_secret_mpi);

    if ((mpi_size == 0U) ||
        (mpi_size > ESP32_ECDH_SHARED_SECRET_SIZE))
    {
        ESP_LOGE(
            TAG,
            "%s ECDH shared-secret size invalid: "
            "got=%u maximum=%u",
            side_name,
            (unsigned int)mpi_size,
            (unsigned int)ESP32_ECDH_SHARED_SECRET_SIZE);

        status = ESP32_ECDH_ERR_SHARED_LENGTH;
        goto cleanup;
    }

    rc = mbedtls_mpi_write_binary(
        &shared_secret_mpi,
        output_secret,
        ESP32_ECDH_SHARED_SECRET_SIZE);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_mpi_write_binary(shared)",
            rc);

        status = ESP32_ECDH_ERR_SHARED_EXPORT;
        goto cleanup;
    }

    if (buffer_is_all_zero(
            output_secret,
            ESP32_ECDH_SHARED_SECRET_SIZE))
    {
        ESP_LOGE(
            TAG,
            "%s ECDH shared secret is all zero",
            side_name);

        status = ESP32_ECDH_ERR_SHARED_LENGTH;
        goto cleanup;
    }

    *output_secret_len =
        ESP32_ECDH_SHARED_SECRET_SIZE;

cleanup:

    if (status != ESP32_ECDH_SUCCESS)
    {
        mbedtls_platform_zeroize(
            output_secret,
            ESP32_ECDH_SHARED_SECRET_SIZE);

        *output_secret_len = 0U;
    }

    mbedtls_mpi_free(
        &shared_secret_mpi);

    return status;
}

/* Computes the sender-side shared secret. */
esp32_ecdh_status_t esp32_ecdh_compute_shared_as_sender(
    esp32_ecdh_ctx_t *ctx)
{
    esp32_ecdh_status_t status;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->keypairs_generated)
    {
        return ESP32_ECDH_ERR_KEYS_NOT_READY;
    }

    /* Combine the sender scalar with the receiver public point. */
    status = compute_shared_internal(
        ctx,
        &ctx->sender.private_key,
        &ctx->receiver.public_key,
        ctx->computed_secret,
        &ctx->computed_secret_len,
        ESP32_ECDH_ERR_SENDER_SHARED,
        "Sender");

    if (status != ESP32_ECDH_SUCCESS)
    {
        return status;
    }

    ESP_LOGI(
        TAG,
        "Sender shared secret computed: length=%u bytes",
        (unsigned int)ctx->computed_secret_len);

    return ESP32_ECDH_SUCCESS;
}

/* Computes the receiver-side shared secret. */
esp32_ecdh_status_t esp32_ecdh_compute_shared_as_receiver(
    esp32_ecdh_ctx_t *ctx)
{
    esp32_ecdh_status_t status;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->keypairs_generated)
    {
        return ESP32_ECDH_ERR_KEYS_NOT_READY;
    }

    /* Combine the receiver scalar with the sender public point. */
    status = compute_shared_internal(
        ctx,
        &ctx->receiver.private_key,
        &ctx->sender.public_key,
        ctx->computed_secret_receiver,
        &ctx->computed_secret_receiver_len,
        ESP32_ECDH_ERR_RECEIVER_SHARED,
        "Receiver");

    if (status != ESP32_ECDH_SUCCESS)
    {
        return status;
    }

    ESP_LOGI(
        TAG,
        "Receiver shared secret computed: length=%u bytes",
        (unsigned int)
            ctx->computed_secret_receiver_len);

    return ESP32_ECDH_SUCCESS;
}

/* Computes the required shared secrets and validates their lengths. */
esp32_ecdh_status_t
esp32_ecdh_compute_and_verify_shared_secrets(
    esp32_ecdh_ctx_t *ctx)
{
    esp32_ecdh_status_t status;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->keypairs_generated)
    {
        return ESP32_ECDH_ERR_KEYS_NOT_READY;
    }

    /* Clear previous secret and key state before computing new material. */
    ecdh_clear_shared_state(ctx);
    ecdh_clear_derived_state(ctx);

    status =
        esp32_ecdh_compute_shared_as_receiver(ctx);

    if (status != ESP32_ECDH_SUCCESS)
    {
        ESP_LOGE(
            TAG,
            "Cross-device receiver shared-secret computation "
            "failed: status=%d",
            (int)status);

        ecdh_clear_shared_state(ctx);
        return status;
    }

    if (ctx->computed_secret_receiver_len !=
        ESP32_ECDH_SHARED_SECRET_SIZE)
    {
        ESP_LOGE(
            TAG,
            "Unexpected receiver shared-secret length: "
            "got=%u expected=%u",
            (unsigned int)ctx->computed_secret_receiver_len,
            (unsigned int)ESP32_ECDH_SHARED_SECRET_SIZE);

        ecdh_clear_shared_state(ctx);
        return ESP32_ECDH_ERR_SHARED_LENGTH;
    }

    ctx->shared_secrets_computed = true;

    ESP_LOGI(
        TAG,
        "Cross-device STM32->ESP32 ECDH shared secret ready: "
        "%u bytes",
        (unsigned int)ctx->computed_secret_receiver_len);

    return ESP32_ECDH_SUCCESS;
}

/* Reports whether complete cross-device crypto material is ready. */
esp32_ecdh_status_t esp32_ecdh_print_all_material(
    const esp32_ecdh_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->keypairs_generated)
    {
        return ESP32_ECDH_ERR_KEYS_NOT_READY;
    }

    if (!ctx->shared_secrets_computed)
    {
        return ESP32_ECDH_ERR_SHARED_NOT_READY;
    }

    /* Check key, secret and derived-key readiness before reporting material. */
    if (!ctx->derived_keys_ready)
    {
        return ESP32_ECDH_ERR_DERIVED_KEYS_NOT_READY;
    }

#if ESP32_ECDH_PROVISIONED_KEYS_PRESENT

    ESP_LOGI(
        TAG,
        "All four provisioned ECDH keys are already loaded");

    ESP_LOGI(
        TAG,
        "Complete crypto-material logging is suppressed");

    return ESP32_ECDH_SUCCESS;

#endif /* ESP32_ECDH_PROVISIONED_KEYS_PRESENT */

    if ((ctx->computed_secret_len !=
            ESP32_ECDH_SHARED_SECRET_SIZE) ||
        (ctx->computed_secret_receiver_len !=
            ESP32_ECDH_SHARED_SECRET_SIZE))
    {
        ESP_LOGE(
            TAG,
            "Shared-secret length state is invalid: "
            "sender=%u receiver=%u",
            (unsigned int)
                ctx->computed_secret_len,
            (unsigned int)
                ctx->computed_secret_receiver_len);

        return ESP32_ECDH_ERR_SHARED_LENGTH;
    }

    if (!secure_buffers_equal(
            ctx->computed_secret,
            ctx->computed_secret_receiver,
            ESP32_ECDH_SHARED_SECRET_SIZE))
    {
        ESP_LOGE(
            TAG,
            "Shared-secret state is inconsistent");

        return ESP32_ECDH_ERR_SHARED_MISMATCH;
    }

    ESP_LOGI(
        TAG,
        "========== COMPLETE CRYPTO MATERIAL ==========");

    ESP_LOGI(TAG, "----- SENDER -----");

    log_hex_buffer(
        "private_key_sender [32]",
        ctx->sender.private_key_bytes,
        sizeof(ctx->sender.private_key_bytes));

    log_hex_buffer(
        "public_key_sender [64]",
        ctx->sender.public_key_bytes,
        ctx->sender.public_key_len);

    ESP_LOGI(
        TAG,
        "computed_secret valid=%u bytes",
        (unsigned int)ctx->computed_secret_len);

    log_hex_buffer(
        "computed_secret [32]",
        ctx->computed_secret,
        ctx->computed_secret_len);

    log_hex_buffer(
        "aes_key [32]",
        ctx->aes_key,
        sizeof(ctx->aes_key));

    log_hex_buffer(
        "mac_key [32]",
        ctx->mac_key,
        sizeof(ctx->mac_key));

    ESP_LOGI(TAG, "----- RECEIVER -----");

    log_hex_buffer(
        "private_key_receiver [32]",
        ctx->receiver.private_key_bytes,
        sizeof(ctx->receiver.private_key_bytes));

    log_hex_buffer(
        "public_key_receiver [64]",
        ctx->receiver.public_key_bytes,
        ctx->receiver.public_key_len);

    ESP_LOGI(
        TAG,
        "computed_secret_receiver valid=%u bytes",
        (unsigned int)ctx->computed_secret_receiver_len);

    log_hex_buffer(
        "computed_secret_receiver [32]",
        ctx->computed_secret_receiver,
        ctx->computed_secret_receiver_len);

    log_hex_buffer(
        "aes_key_receiver [32]",
        ctx->aes_key_receiver,
        sizeof(ctx->aes_key_receiver));

    log_hex_buffer(
        "mac_key_receiver [32]",
        ctx->mac_key_receiver,
        sizeof(ctx->mac_key_receiver));

    ESP_LOGI(
        TAG,
        "Sender/receiver shared secrets match");

    ESP_LOGI(
        TAG,
        "Sender/receiver AES keys match");

    ESP_LOGI(
        TAG,
        "Sender/receiver MAC keys match");

    ESP_LOGI(
        TAG,
        "==============================================");

    return ESP32_ECDH_SUCCESS;
}

/* Reports shared-secret state with optional debug output. */
esp32_ecdh_status_t esp32_ecdh_print_shared_secrets(
    const esp32_ecdh_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->shared_secrets_computed)
    {
        return ESP32_ECDH_ERR_SHARED_NOT_READY;
    }

    ESP_LOGI(
        TAG,
        "========== ECDH SHARED SECRETS ==========");

/* Gate raw secret output behind the configured debug switch. */
#if ESP32_ECDH_DEBUG_PRINT_SHARED_SECRETS

    ESP_LOGW(
        TAG,
        "DEBUG ONLY: printing ECDH shared secrets");

    ESP_LOGI(
        TAG,
        "Sender secret length=%u",
        (unsigned int)ctx->computed_secret_len);

    log_hex_buffer(
        "computed_secret",
        ctx->computed_secret,
        ctx->computed_secret_len);

    ESP_LOGI(
        TAG,
        "Receiver secret length=%u",
        (unsigned int)ctx->computed_secret_receiver_len);

    log_hex_buffer(
        "computed_secret_receiver",
        ctx->computed_secret_receiver,
        ctx->computed_secret_receiver_len);

#else

    ESP_LOGI(
        TAG,
        "Shared-secret printing disabled");

#endif

    ESP_LOGI(
        TAG,
        "Sender and receiver shared secrets match");

    ESP_LOGI(
        TAG,
        "=========================================");

    return ESP32_ECDH_SUCCESS;
}

/*
 * Exact compatibility KDF used by your STM32 code:
 *
 * AES key = SHA256(shared || 0x01)
 * MAC key = SHA256(shared || 0x02)
 */
/* Derives AES and MAC keys from a shared secret using SHA-256. */
static esp32_ecdh_status_t derive_keys_internal(
    const uint8_t *shared_secret,
    size_t shared_secret_len,
    uint8_t aes_key[
        ESP32_ECDH_DERIVED_KEY_SIZE],
    size_t *aes_key_len,
    uint8_t mac_key[
        ESP32_ECDH_DERIVED_KEY_SIZE],
    size_t *mac_key_len,
    esp32_ecdh_status_t side_error,
    const char *side_name)
{
    uint8_t kdf_input[
        ESP32_ECDH_SHARED_SECRET_SIZE + 1U];

    int rc;

    if ((shared_secret == NULL) ||
        (aes_key == NULL) ||
        (aes_key_len == NULL) ||
        (mac_key == NULL) ||
        (mac_key_len == NULL) ||
        (side_name == NULL))
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (shared_secret_len !=
        ESP32_ECDH_SHARED_SECRET_SIZE)
    {
        ESP_LOGE(
            TAG,
            "%s KDF received invalid shared-secret "
            "length: %u",
            side_name,
            (unsigned int)shared_secret_len);

        return side_error;
    }

    mbedtls_platform_zeroize(
        kdf_input,
        sizeof(kdf_input));

    mbedtls_platform_zeroize(
        aes_key,
        ESP32_ECDH_DERIVED_KEY_SIZE);

    mbedtls_platform_zeroize(
        mac_key,
        ESP32_ECDH_DERIVED_KEY_SIZE);

    *aes_key_len = 0U;
    *mac_key_len = 0U;

    memcpy(
        kdf_input,
        shared_secret,
        shared_secret_len);

    /* Use separate trailing domain bytes for the AES and MAC hashes. */
    kdf_input[shared_secret_len] =
        0x01U;

    rc = mbedtls_sha256(
        kdf_input,
        shared_secret_len + 1U,
        aes_key,
        0);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_sha256(AES KDF)",
            rc);

        mbedtls_platform_zeroize(
            kdf_input,
            sizeof(kdf_input));

        mbedtls_platform_zeroize(
            aes_key,
            ESP32_ECDH_DERIVED_KEY_SIZE);

        mbedtls_platform_zeroize(
            mac_key,
            ESP32_ECDH_DERIVED_KEY_SIZE);

        return ESP32_ECDH_ERR_KDF_AES_HASH;
    }

    *aes_key_len =
        ESP32_ECDH_DERIVED_KEY_SIZE;

    kdf_input[shared_secret_len] =
        0x02U;

    rc = mbedtls_sha256(
        kdf_input,
        shared_secret_len + 1U,
        mac_key,
        0);

    if (rc != 0)
    {
        log_mbedtls_error(
            "mbedtls_sha256(MAC KDF)",
            rc);

        mbedtls_platform_zeroize(
            kdf_input,
            sizeof(kdf_input));

        mbedtls_platform_zeroize(
            aes_key,
            ESP32_ECDH_DERIVED_KEY_SIZE);

        mbedtls_platform_zeroize(
            mac_key,
            ESP32_ECDH_DERIVED_KEY_SIZE);

        *aes_key_len = 0U;

        return ESP32_ECDH_ERR_KDF_MAC_HASH;
    }

    *mac_key_len =
        ESP32_ECDH_DERIVED_KEY_SIZE;

    mbedtls_platform_zeroize(
        kdf_input,
        sizeof(kdf_input));

    ESP_LOGI(
        TAG,
        "%s AES and MAC keys derived successfully",
        side_name);

    return ESP32_ECDH_SUCCESS;
}

/* Derives the sender AES and MAC keys. */
esp32_ecdh_status_t esp32_ecdh_derive_sender_keys(
    esp32_ecdh_ctx_t *ctx)
{
    esp32_ecdh_status_t status;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    /* Require the shared-secret readiness flag before applying the KDF. */
    if (!ctx->shared_secrets_computed)
    {
        return ESP32_ECDH_ERR_SHARED_NOT_READY;
    }

    status = derive_keys_internal(
        ctx->computed_secret,
        ctx->computed_secret_len,
        ctx->aes_key,
        &ctx->aes_key_len,
        ctx->mac_key,
        &ctx->mac_key_len,
        ESP32_ECDH_ERR_SENDER_KDF,
        "Sender");

    if (status != ESP32_ECDH_SUCCESS)
    {
        ESP_LOGE(
            TAG,
            "Sender key derivation failed: status=%d",
            (int)status);

        return status;
    }

    return ESP32_ECDH_SUCCESS;
}

/* Derives the receiver AES and MAC keys. */
esp32_ecdh_status_t esp32_ecdh_derive_receiver_keys(
    esp32_ecdh_ctx_t *ctx)
{
    esp32_ecdh_status_t status;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->shared_secrets_computed)
    {
        return ESP32_ECDH_ERR_SHARED_NOT_READY;
    }

    /* Use the receiver secret buffer for this direction's KDF. */
    status = derive_keys_internal(
        ctx->computed_secret_receiver,
        ctx->computed_secret_receiver_len,
        ctx->aes_key_receiver,
        &ctx->aes_key_receiver_len,
        ctx->mac_key_receiver,
        &ctx->mac_key_receiver_len,
        ESP32_ECDH_ERR_RECEIVER_KDF,
        "Receiver");

    if (status != ESP32_ECDH_SUCCESS)
    {
        ESP_LOGE(
            TAG,
            "Receiver key derivation failed: status=%d",
            (int)status);

        return status;
    }

    return ESP32_ECDH_SUCCESS;
}

/* Derives the required keys and validates their readiness state. */
esp32_ecdh_status_t esp32_ecdh_derive_and_verify_keys(
    esp32_ecdh_ctx_t *ctx)
{
    esp32_ecdh_status_t status;

    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->shared_secrets_computed)
    {
        return ESP32_ECDH_ERR_SHARED_NOT_READY;
    }

    /* Clear stale derived material before starting the KDF. */
    ecdh_clear_derived_state(ctx);

    /*
     * Derive only the receive-direction keys:
     *
     * AES key = SHA256(shared || 0x01)
     * MAC key = SHA256(shared || 0x02)
     */
    status =
        esp32_ecdh_derive_receiver_keys(ctx);

    if (status != ESP32_ECDH_SUCCESS)
    {
        ecdh_clear_derived_state(ctx);
        return status;
    }

    if ((ctx->aes_key_receiver_len !=
            ESP32_ECDH_DERIVED_KEY_SIZE) ||
        (ctx->mac_key_receiver_len !=
            ESP32_ECDH_DERIVED_KEY_SIZE) ||
        buffer_is_all_zero(
            ctx->aes_key_receiver,
            ESP32_ECDH_DERIVED_KEY_SIZE) ||
        buffer_is_all_zero(
            ctx->mac_key_receiver,
            ESP32_ECDH_DERIVED_KEY_SIZE))
    {
        ESP_LOGE(
            TAG,
            "Invalid receive-direction derived-key state");

        ecdh_clear_derived_state(ctx);
        return ESP32_ECDH_ERR_DERIVED_KEY_MISMATCH;
    }

    ctx->derived_keys_ready = true;

    ESP_LOGI(
        TAG,
        "STM32->ESP32 receiver AES key derived successfully");

    ESP_LOGI(
        TAG,
        "Receiver AES KDF compatibility: "
        "SHA256(shared_secret || 0x01)");

    return ESP32_ECDH_SUCCESS;
}

/* Reports derived-key state with optional debug output. */
esp32_ecdh_status_t esp32_ecdh_print_derived_keys(
    const esp32_ecdh_ctx_t *ctx)
{
    if (ctx == NULL)
    {
        return ESP32_ECDH_ERR_BAD_PARAM;
    }

    if (!ctx->derived_keys_ready)
    {
        return ESP32_ECDH_ERR_DERIVED_KEYS_NOT_READY;
    }

    ESP_LOGI(
        TAG,
        "========== DERIVED AES/MAC KEYS =========");

/* Only expose raw derived keys when their debug switch is enabled. */
#if ESP32_ECDH_DEBUG_PRINT_DERIVED_KEYS

    ESP_LOGW(
        TAG,
        "DEBUG ONLY: printing derived keys");

    log_hex_buffer(
        "aes_key [32]",
        ctx->aes_key,
        ctx->aes_key_len);

    log_hex_buffer(
        "aes_key_receiver [32]",
        ctx->aes_key_receiver,
        ctx->aes_key_receiver_len);

    log_hex_buffer(
        "mac_key [32]",
        ctx->mac_key,
        ctx->mac_key_len);

    log_hex_buffer(
        "mac_key_receiver [32]",
        ctx->mac_key_receiver,
        ctx->mac_key_receiver_len);

#else

    ESP_LOGI(
        TAG,
        "Derived-key printing disabled");

#endif

    ESP_LOGI(
        TAG,
        "Sender and receiver AES keys match");

    ESP_LOGI(
        TAG,
        "Sender and receiver MAC keys match");

    ESP_LOGI(
        TAG,
        "=========================================");

    return ESP32_ECDH_SUCCESS;
}