/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#ifndef MAIN_MBEDTLS_ECDH_H_
#define MAIN_MBEDTLS_ECDH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#include "mbedtls/entropy.h"

#define ESP32_ECDH_PRIVATE_KEY_SIZE              32U
#define ESP32_ECDH_PUBLIC_KEY_RAW_SIZE           64U
#define ESP32_ECDH_PUBLIC_KEY_SIZE               ESP32_ECDH_PUBLIC_KEY_RAW_SIZE
#define ESP32_ECDH_PUBLIC_KEY_UNCOMPRESSED_SIZE  65U
#define ESP32_ECDH_SHARED_SECRET_SIZE            32U
#define ESP32_ECDH_DERIVED_KEY_SIZE              32U

#ifndef ESP32_ECDH_PROVISIONED_KEYS_PRESENT
#define ESP32_ECDH_PROVISIONED_KEYS_PRESENT      1
#endif

#if ESP32_ECDH_PROVISIONED_KEYS_PRESENT && \
    defined(ESP32_ECDH_DEFINE_PROVISIONED_KEYS)


static const uint8_t esp32_provisioned_sender_private_key[
    ESP32_ECDH_PRIVATE_KEY_SIZE
] = {
    0xF0, 0xEC, 0x2D, 0xF6, 0x5A, 0xD9, 0xA4, 0xD4,
    0xFA, 0x55, 0xEB, 0x57, 0x85, 0xFB, 0x7A, 0x09,
    0xA5, 0x1D, 0xD2, 0xEA, 0x9E, 0x87, 0xED, 0xDC,
    0xAD, 0x3E, 0x99, 0xD2, 0x62, 0x6C, 0x77, 0x68
};

static const uint8_t esp32_provisioned_sender_public_key[
    ESP32_ECDH_PUBLIC_KEY_SIZE
] = {
    0x3F, 0x1D, 0xAA, 0x91, 0xD9, 0x88, 0xEE, 0x7A,
    0x33, 0x15, 0xFE, 0xE4, 0x81, 0xF7, 0x35, 0x6F,
    0xBA, 0x73, 0x09, 0xF8, 0xF9, 0x30, 0x64, 0x31,
    0x76, 0xBD, 0x3F, 0xD8, 0x79, 0x10, 0xC0, 0xBF,
    0xE9, 0x18, 0x51, 0x09, 0x9A, 0x96, 0x43, 0x85,
    0xC8, 0xD3, 0xC4, 0xF5, 0x78, 0x6B, 0xF7, 0xA5,
    0x9E, 0x24, 0xF6, 0xA2, 0x50, 0x3C, 0x93, 0xCA,
    0xB6, 0x12, 0x83, 0xA6, 0xFA, 0x83, 0xF6, 0xD5
};

static const uint8_t esp32_provisioned_receiver_private_key[
    ESP32_ECDH_PRIVATE_KEY_SIZE
] = {
    0x02, 0x39, 0x53, 0x83, 0x4E, 0x16, 0x4E, 0x0B,
    0xF9, 0xDE, 0x13, 0x0F, 0xCB, 0x88, 0x2B, 0xCA,
    0xA0, 0xA4, 0x5F, 0xD5, 0x16, 0xBF, 0x8D, 0xDC,
    0xFC, 0x31, 0x98, 0xBA, 0x6C, 0x2F, 0x51, 0x18
};

static const uint8_t esp32_provisioned_receiver_public_key[
    ESP32_ECDH_PUBLIC_KEY_SIZE
] = {
    0x09, 0x00, 0xB5, 0x43, 0x4F, 0xA6, 0x71, 0xBD,
    0xBC, 0x49, 0xC9, 0xA3, 0xCB, 0xA4, 0x58, 0x43,
    0x28, 0x12, 0x94, 0xD5, 0x48, 0x2C, 0x01, 0x69,
    0xBE, 0x27, 0xDD, 0xB4, 0x12, 0x04, 0xD4, 0xA9,
    0x42, 0x2B, 0xA7, 0x3E, 0xFF, 0x8A, 0xE6, 0x13,
    0x3C, 0x10, 0x7E, 0x4D, 0x87, 0x5E, 0xCB, 0xB1,
    0x23, 0xC7, 0x18, 0x8A, 0xA0, 0x27, 0xEE, 0x9D,
    0x49, 0xD4, 0x3F, 0xB2, 0x1C, 0xCD, 0xA0, 0x9C
};

static const uint8_t stm32_provisioned_sender_public_key[
    ESP32_ECDH_PUBLIC_KEY_SIZE
] = {
    0x0B, 0x5E, 0xC4, 0xE1, 0x69, 0xEE, 0xB5, 0xBF,
    0x0F, 0x2F, 0xC3, 0x0A, 0xD8, 0xD3, 0xD3, 0x9E,
    0x7A, 0x7F, 0x94, 0x2C, 0xF5, 0x0D, 0xA8, 0x81,
    0x87, 0x50, 0xF8, 0xE4, 0x3E, 0x2F, 0xC3, 0xE7,
    0xE7, 0x28, 0x95, 0xFF, 0x94, 0x1F, 0x29, 0x77,
    0x85, 0x42, 0x4C, 0x88, 0xAC, 0x9F, 0x68, 0x56,
    0xA3, 0x64, 0xAE, 0x3F, 0xF6, 0xC6, 0xBF, 0xC1,
    0x68, 0x37, 0xCF, 0xD1, 0xAE, 0x29, 0x96, 0xD2
};

static const uint8_t stm32_provisioned_receiver_public_key[
    ESP32_ECDH_PUBLIC_KEY_SIZE
] = {
    0x51, 0xEE, 0x6E, 0x4B, 0xF3, 0x8E, 0x36, 0x85,
    0xF8, 0x06, 0x4F, 0xAB, 0x3C, 0x5E, 0x35, 0xA2,
    0x55, 0x1D, 0xED, 0x22, 0x8E, 0x2E, 0xCC, 0xD6,
    0xE7, 0x1A, 0xEE, 0x0B, 0x47, 0x39, 0x53, 0x39,
    0xEE, 0xED, 0x29, 0xA4, 0x48, 0x31, 0x83, 0x5A,
    0x2C, 0x4E, 0x18, 0x6B, 0x2B, 0x00, 0x05, 0xB3,
    0xAD, 0x5D, 0x47, 0xDA, 0xEE, 0x38, 0x87, 0x25,
    0x6D, 0xAE, 0x5A, 0x6F, 0x47, 0x8D, 0x06, 0x2B
};

#endif /* provisioned-key definitions */

_Static_assert(
    ESP32_ECDH_PRIVATE_KEY_SIZE == 32U,
    "secp256r1 private key must be 32 bytes");

_Static_assert(
    ESP32_ECDH_PUBLIC_KEY_SIZE == 64U,
    "secp256r1 raw public key must be X[32] || Y[32]");

_Static_assert(
    ESP32_ECDH_SHARED_SECRET_SIZE == 32U,
    "canonical secp256r1 ECDH secret must be 32 bytes");

_Static_assert(
    ESP32_ECDH_DERIVED_KEY_SIZE == 32U,
    "SHA-256 derived key must be 32 bytes");

#ifndef ESP32_ECDH_DEBUG_PRINT_PRIVATE_KEYS
#define ESP32_ECDH_DEBUG_PRINT_PRIVATE_KEYS      1
#endif

#ifndef ESP32_ECDH_DEBUG_PRINT_SHARED_SECRETS
#define ESP32_ECDH_DEBUG_PRINT_SHARED_SECRETS    1
#endif

#ifndef ESP32_ECDH_DEBUG_PRINT_DERIVED_KEYS
#define ESP32_ECDH_DEBUG_PRINT_DERIVED_KEYS      1
#endif

typedef enum
{
    ESP32_ECDH_SUCCESS = 0,

    ESP32_ECDH_ERR_BAD_PARAM               = -1,
    ESP32_ECDH_ERR_ALREADY_INIT            = -2,
    ESP32_ECDH_ERR_NOT_INITIALIZED         = -3,
    ESP32_ECDH_ERR_DRBG_SEED               = -4,
    ESP32_ECDH_ERR_CURVE_UNSUPPORTED       = -5,
    ESP32_ECDH_ERR_CURVE_LOAD              = -6,
    ESP32_ECDH_ERR_SENDER_KEYGEN           = -7,
    ESP32_ECDH_ERR_RECEIVER_KEYGEN         = -8,
    ESP32_ECDH_ERR_PRIVATE_KEY_CHECK       = -9,
    ESP32_ECDH_ERR_PUBLIC_KEY_CHECK        = -10,
    ESP32_ECDH_ERR_PRIVATE_EXPORT          = -11,
    ESP32_ECDH_ERR_PUBLIC_EXPORT           = -12,
    ESP32_ECDH_ERR_PUBLIC_LENGTH           = -13,
    ESP32_ECDH_ERR_ALREADY_GENERATED       = -14,
    ESP32_ECDH_ERR_KEYS_NOT_READY          = -15,
    ESP32_ECDH_ERR_SENDER_SHARED           = -16,
    ESP32_ECDH_ERR_RECEIVER_SHARED         = -17,
    ESP32_ECDH_ERR_SHARED_EXPORT           = -18,
    ESP32_ECDH_ERR_SHARED_LENGTH           = -19,
    ESP32_ECDH_ERR_SHARED_MISMATCH         = -20,
    ESP32_ECDH_ERR_SHARED_NOT_READY        = -21,
    ESP32_ECDH_ERR_SENDER_KDF              = -22,
    ESP32_ECDH_ERR_RECEIVER_KDF            = -23,
    ESP32_ECDH_ERR_KDF_AES_HASH            = -24,
    ESP32_ECDH_ERR_KDF_MAC_HASH            = -25,
    ESP32_ECDH_ERR_DERIVED_KEY_MISMATCH    = -26,
    ESP32_ECDH_ERR_DERIVED_KEYS_NOT_READY  = -27,
    ESP32_ECDH_ERR_PROVISIONED_KEY_EMPTY   = -28,
    ESP32_ECDH_ERR_PROVISIONED_KEY_IMPORT  = -29,
    ESP32_ECDH_ERR_PROVISIONED_KEY_MISMATCH = -30

} esp32_ecdh_status_t;

typedef struct
{
    mbedtls_mpi private_key;
    mbedtls_ecp_point public_key;

    uint8_t private_key_bytes[
        ESP32_ECDH_PRIVATE_KEY_SIZE];

    uint8_t public_key_bytes[
        ESP32_ECDH_PUBLIC_KEY_SIZE];

    size_t public_key_len;

} esp32_ecdh_party_t;

typedef struct
{
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    mbedtls_ecp_group group;

    esp32_ecdh_party_t sender;
    esp32_ecdh_party_t receiver;

    uint8_t computed_secret[
        ESP32_ECDH_SHARED_SECRET_SIZE];

    size_t computed_secret_len;

    uint8_t computed_secret_receiver[
        ESP32_ECDH_SHARED_SECRET_SIZE];

    size_t computed_secret_receiver_len;

    uint8_t aes_key[
        ESP32_ECDH_DERIVED_KEY_SIZE];

    uint8_t aes_key_receiver[
        ESP32_ECDH_DERIVED_KEY_SIZE];

    uint8_t mac_key[
        ESP32_ECDH_DERIVED_KEY_SIZE];

    uint8_t mac_key_receiver[
        ESP32_ECDH_DERIVED_KEY_SIZE];

    size_t aes_key_len;
    size_t aes_key_receiver_len;
    size_t mac_key_len;
    size_t mac_key_receiver_len;

    bool objects_initialized;
    bool drbg_seeded;
    bool keypairs_generated;
    bool shared_secrets_computed;
    bool derived_keys_ready;

} esp32_ecdh_ctx_t;

esp32_ecdh_status_t esp32_ecdh_init(
    esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_generate_keypairs(
    esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_print_keypairs(
    const esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_compute_shared_as_sender(
    esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_compute_shared_as_receiver(
    esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_compute_and_verify_shared_secrets(
    esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_print_shared_secrets(
    const esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_derive_sender_keys(
    esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_derive_receiver_keys(
    esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_derive_and_verify_keys(
    esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_print_derived_keys(
    const esp32_ecdh_ctx_t *ctx);

esp32_ecdh_status_t esp32_ecdh_print_all_material(
    const esp32_ecdh_ctx_t *ctx);

void esp32_ecdh_free(
    esp32_ecdh_ctx_t *ctx);

#endif /* MAIN_MBEDTLS_ECDH_H_ */
