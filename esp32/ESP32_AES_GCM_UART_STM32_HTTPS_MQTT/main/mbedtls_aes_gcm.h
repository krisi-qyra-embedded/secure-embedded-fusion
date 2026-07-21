/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#ifndef MAIN_MBEDTLS_AES_GCM_H_
#define MAIN_MBEDTLS_AES_GCM_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mbedtls/gcm.h"
#include "mbedtls_ecdh.h"

#define ESP32_AES_GCM_KEY_SIZE       32U
#define ESP32_AES_GCM_NONCE_SIZE     12U
#define ESP32_AES_GCM_TAG_SIZE       16U
#define ESP32_GCM_COUNTER_START      UINT64_C(1)

_Static_assert(
    ESP32_AES_GCM_KEY_SIZE == ESP32_ECDH_DERIVED_KEY_SIZE,
    "AES-GCM key size must match the ECDH-derived AES key");

_Static_assert(
    ESP32_AES_GCM_NONCE_SIZE == 12U,
    "AES-GCM nonce must be 12 bytes");

_Static_assert(
    ESP32_AES_GCM_TAG_SIZE == 16U,
    "AES-GCM tag must be 16 bytes");

typedef enum
{
    ESP32_AES_GCM_SUCCESS = 0,

    ESP32_AES_GCM_ERR_BAD_PARAM              = -1,
    ESP32_AES_GCM_ERR_ALREADY_INIT           = -2,
    ESP32_AES_GCM_ERR_NOT_INITIALIZED        = -3,
    ESP32_AES_GCM_ERR_BAD_ECDH_STATE         = -4,
    ESP32_AES_GCM_ERR_SESSION_HASH           = -5,
    ESP32_AES_GCM_ERR_NO_SESSION             = -6,
    ESP32_AES_GCM_ERR_COUNTER_EXHAUSTED      = -7,
    ESP32_AES_GCM_ERR_BAD_KEY                = -8,
    ESP32_AES_GCM_ERR_OUTPUT_TOO_SMALL       = -9,
    ESP32_AES_GCM_ERR_SETKEY                 = -10,
    ESP32_AES_GCM_ERR_ENCRYPT                = -11,
    ESP32_AES_GCM_ERR_DECRYPT_AUTH           = -12,
    ESP32_AES_GCM_ERR_LENGTH                 = -13,
    ESP32_AES_GCM_ERR_SELF_TEST_MISMATCH     = -14,
    ESP32_AES_GCM_ERR_TASK_ALREADY_RUNNING   = -15,
    ESP32_AES_GCM_ERR_TASK_CREATE            = -16

} esp32_aes_gcm_status_t;

typedef struct
{
    uint32_t session_id;
    uint64_t next_counter;
    uint64_t current_counter;
    bool initialized;

} esp32_gcm_nonce_state_t;

typedef struct
{
    mbedtls_gcm_context gcm;

    esp32_gcm_nonce_state_t nonce_state;

    uint8_t iv_gcm[
        ESP32_AES_GCM_NONCE_SIZE];

    bool initialized;

} esp32_aes_gcm_ctx_t;

esp32_aes_gcm_status_t esp32_aes_gcm_init(
    esp32_aes_gcm_ctx_t *ctx);

void esp32_aes_gcm_free(
    esp32_aes_gcm_ctx_t *ctx);

esp32_aes_gcm_status_t esp32_gcm_nonce_session_init(
    esp32_aes_gcm_ctx_t *ctx,
    const esp32_ecdh_ctx_t *ecdh_ctx);

esp32_aes_gcm_status_t esp32_gcm_nonce_allocate(
    esp32_aes_gcm_ctx_t *ctx);

esp32_aes_gcm_status_t
esp32_generate_nonce_and_iv_per_transfer(
    esp32_aes_gcm_ctx_t *ctx);

esp32_aes_gcm_status_t esp32_encrypt_aes_gcm(
    esp32_aes_gcm_ctx_t *ctx,
    const uint8_t key[ESP32_AES_GCM_KEY_SIZE],
    const uint8_t *plaintext,
    size_t plaintext_len,
    const uint8_t *aad,
    size_t aad_len,
    uint8_t *ciphertext_and_tag,
    size_t output_capacity,
    size_t *output_len);

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
    size_t *output_len);

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
    size_t *plaintext_len);

esp32_aes_gcm_status_t esp32_aes_gcm_runtime_task_start(
    esp32_aes_gcm_ctx_t *gcm_ctx,
    esp32_ecdh_ctx_t *ecdh_ctx);

void esp32_aes_gcm_runtime_task_stop(void);

bool esp32_aes_gcm_runtime_task_is_running(void);

#endif /* MAIN_MBEDTLS_AES_GCM_H_ */
