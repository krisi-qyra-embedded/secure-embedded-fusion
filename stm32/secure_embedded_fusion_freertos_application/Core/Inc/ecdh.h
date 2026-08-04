/* Author: i fiksuari pas prog
 * Documentation date: 2026-08-18
 */

#ifndef ECDH_H_
#define ECDH_H_
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "uart.h"
#include <nonce.h>
#include "stm32f4xx.h"
#include "cmox_crypto.h"
#include <entropy_personalization_nonce_gen.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#define AES_512_KEY_SIZE            64
#define AES_256_KEY_SIZE            32  // 32 bytes = 256 bits
#define AES_256_IV_SIZE             16  // 16 bytes (CTR counter block)
#define AES_GCM_AEAD_IV_SIZE        12  // 12-byte AES-GCM nonce/IV
#define GCM_GCM_TAG_SIZE            16  /* pro recommended tag size (128-bit) */
#define ADC_SAMPLE_COUNT            8
#define BYTES_PER_SAMPLE            sizeof(uint32_t)
#define AES_GCM_TAG_SIZE       	    16
/*
 * STM32 -> ESP32 telemetry AES-GCM capacity.
 *
 * Plaintext frame:
 *     header    =   9 bytes
 *     telemetry = 159 bytes
 *     checksum  =   1 byte
 *                ---------
 *                 169 bytes
 *
 * AES-GCM output:
 *     ciphertext = 169 bytes
 *     tag        =  16 bytes
 *                  ---------
 *                  185 bytes
 *
 * UART wire packet adds the existing 12-byte nonce:
 *     12 + 185 = 197 bytes.
 *
 * These sizes are intentionally independent of the legacy ADC payload size.
 */
#define UART_GCM_PLAINTEXT_SIZE          169U
#define UART_GCM_CIPHERTEXT_TAG_SIZE     \
    (UART_GCM_PLAINTEXT_SIZE + AES_GCM_TAG_SIZE)
#define GCM_NONCE_SIZE_BYTES  		12U
#define GCM_COUNTER_START     		1ULL
#define ECDH_CANONICAL_SECRET_LEN   32U
extern SemaphoreHandle_t xSerialSemaphore;
#define ECDH_LOG_PRINTF(...)                                                 \
    do {                                                                     \
        if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&           \
            (xSerialSemaphore != NULL)) {                                    \
            if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) ==     \
                pdTRUE) {                                                    \
                log_printf(__VA_ARGS__);                                     \
                (void)xSemaphoreGive(xSerialSemaphore);                      \
            }                                                                \
        } else {                                                             \
            log_printf(__VA_ARGS__);                                         \
        }                                                                    \
    } while (0)
/*
 * ESP32 -> STM32 encrypted ACK/NACK response format.
 *
 * Plaintext:
 *   magic[2] || version[1] || type[1] || payload_len[1] ||
 *   sequence_le[4] || reason[1] || checksum8[1]
 *
 * Encrypted UART wire packet:
 *   nonce[12] || ciphertext[11] || tag[16]
 */
#define GCM_UART_RESPONSE_PLAINTEXT_SIZE      11U
#define GCM_UART_RESPONSE_CIPHERTEXT_SIZE     11U
#define GCM_UART_RESPONSE_WIRE_SIZE           \
    (AES_GCM_AEAD_IV_SIZE +                   \
     GCM_UART_RESPONSE_CIPHERTEXT_SIZE +      \
     AES_GCM_TAG_SIZE)
#define GCM_UART_MAGIC_0                      0xA5U
#define GCM_UART_MAGIC_1                      0x5AU
#define GCM_UART_PROTOCOL_VERSION             0x01U
#define GCM_UART_MSG_TYPE_ACK                 0x20U
#define GCM_UART_MSG_TYPE_NACK                0x21U
#define GCM_UART_RESPONSE_PAYLOAD_LEN         0x01U
#ifndef ECDH_DEBUG_PRINT_RX_DECRYPTED_PACKET
#define ECDH_DEBUG_PRINT_RX_DECRYPTED_PACKET  1U
#endif
#ifndef ECDH_DEBUG_PRINT_RX_DERIVED_MATERIAL
#define ECDH_DEBUG_PRINT_RX_DERIVED_MATERIAL  0U
#endif
#if GCM_UART_RESPONSE_WIRE_SIZE != 39U
#error "ESP32 encrypted ACK/NACK wire packet must be 39 bytes"
#endif
#define CRYPTO_UART_LOG_DELAY_MS     20U
#define ECDH_PROVISIONED_KEYS_PRESENT    1
#if ECDH_PROVISIONED_KEYS_PRESENT
static const uint8_t ecdh_provisioned_sender_private_key[
    AES_256_KEY_SIZE
] = {
    0xEA, 0xA6, 0xC0, 0x09, 0xD5, 0xD7, 0xFE, 0x76,
    0x16, 0xC2, 0xD7, 0xAA, 0x56, 0x1C, 0x69, 0x8B,
    0xFF, 0x99, 0xFF, 0x7A, 0xC6, 0xA3, 0x97, 0x39,
    0x5C, 0xB3, 0xFD, 0x31, 0xAA, 0x36, 0xF4, 0x8C
};
static const uint8_t ecdh_provisioned_sender_public_key[
    AES_512_KEY_SIZE
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
static const uint8_t ecdh_provisioned_receiver_private_key[
    AES_256_KEY_SIZE
] = {
    0x6D, 0xD3, 0x72, 0xA1, 0x7C, 0x47, 0x38, 0xDE,
    0xFE, 0x64, 0x95, 0x1D, 0x89, 0xB7, 0xD9, 0xFA,
    0x1C, 0x08, 0x1C, 0x94, 0x4B, 0xEC, 0xA5, 0x9A,
    0x8D, 0x21, 0x9A, 0x7A, 0x45, 0x08, 0xA0, 0x8D
};
static const uint8_t ecdh_provisioned_receiver_public_key[
    AES_512_KEY_SIZE
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
static const uint8_t esp32_provisioned_receiver_public_key[
    AES_512_KEY_SIZE
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
static const uint8_t esp32_provisioned_sender_public_key[
    AES_512_KEY_SIZE
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
#endif /* ECDH_PROVISIONED_KEYS_PRESENT */
#if ECDH_CANONICAL_SECRET_LEN != AES_256_KEY_SIZE
#error "Canonical secp256r1 ECDH secret must be 32 bytes"
#endif
typedef enum
{
    GCM_NONCE_OK = 0,
    GCM_NONCE_ERR_BAD_PARAM      = -1,
    GCM_NONCE_ERR_NO_SESSION     = -2,
    GCM_NONCE_ERR_BAD_KEY_STATE  = -3,
    GCM_NONCE_ERR_HASH           = -4,
    GCM_NONCE_ERR_EXHAUSTED      = -5,
    GCM_NONCE_ERR_NVM            = -6
} gcm_nonce_status_t;
typedef struct
{
    uint32_t session_id;
    uint64_t next_counter;
    uint64_t current_counter;
    bool initialized;
} gcm_nonce_state_t;
typedef enum {
    GEN_PRIV_SUCCESS            =  0,
    GEN_PRIV_ERR_CONSTRUCT      = -1,
    GEN_PRIV_ERR_INIT           = -2,
    GEN_PRIV_ERR_GEN_SENDER     = -3,
    GEN_PRIV_ERR_GEN_RECEIVER   = -4,
    GEN_PRIV_ERR_CLEANUP        = -5,
	GEN_PRIV_ERR_BAD_PARAM      = -6
} gen_priv_status_t;
typedef enum {
    GEN_IV_SUCCESS        =  0,
    GEN_IV_ERR_CONSTRUCT  = -1,
    GEN_IV_ERR_INIT       = -2,
    GEN_IV_ERR_GENERATE   = -3,
    GEN_IV_ERR_CLEANUP    = -4,
	GEN_IV_ERR_BAD_PARAM  = -5
} gen_iv_status_t;
typedef enum {
    GEN_KEYS_SUCCESS               =  0,
    GEN_KEYS_ERR_DRBG_CONSTRUCT    = -1,
    GEN_KEYS_ERR_DRBG_INIT         = -2,
    GEN_KEYS_ERR_DRBG_GENERATE     = -3,
    GEN_KEYS_ERR_RANDOM_TOO_LARGE  = -4,
    GEN_KEYS_ERR_KEYGEN_SENDER     = -5,
    GEN_KEYS_ERR_KEYGEN_RECEIVER   = -6,
    GEN_KEYS_ERR_DRBG_CLEANUP      = -7,
	GEN_KEYS_ERR_BAD_PARAM         = -8
} gen_keys_status_t;
typedef enum {
    ECDH_SUCCESS          =  0,
    ECDH_ERR_ECDH_FAILED  = -1
} ecdh_status_t;
typedef enum {
    KDF_SUCCESS          =  0,
    KDF_ERR_HASH_FAILED  = -1,
    KDF_ERR_BAD_INPUT    = -2
} kdf_status_t;
typedef enum {
    AES_CTR_ENC_SUCCESS              =  0,
    AES_CTR_ENC_ERR_BAD_INPUT        = -1,
    AES_CTR_ENC_ERR_KEY_OR_IV        = -2,
    AES_CTR_ENC_ERR_CIPHER_FAIL      = -3,
    AES_CTR_ENC_ERR_LENGTH_MISMATCH  = -4
} aes_ctr_enc_status_t;
typedef enum {
    AES_CTR_DEC_SUCCESS              =  0,
    AES_CTR_DEC_ERR_BAD_INPUT        = -1,
    AES_CTR_DEC_ERR_KEY_OR_IV        = -2,
    AES_CTR_DEC_ERR_CIPHER_FAIL      = -3,
    AES_CTR_DEC_ERR_LENGTH_MISMATCH  = -4
} aes_ctr_dec_status_t;
typedef enum {
    HMAC_IVC_SUCCESS            =  0,
    HMAC_IVC_ERR_NULL_INPUT     = -1,
    HMAC_IVC_ERR_NO_CIPHERTEXT  = -2,
    HMAC_IVC_ERR_SIZE_INVALID   = -3,
    HMAC_IVC_ERR_SIZE_OVERFLOW  = -4,
    HMAC_IVC_ERR_ALLOC_FAIL     = -5,
    HMAC_IVC_ERR_MAC_FAILED     = -6
} hmac_ivc_status_t;
typedef enum {
    SENDER_INIT_SUCCESS     =  0,
    SENDER_INIT_ERR_GENKEY  = -1,
    SENDER_INIT_ERR_ECDH    = -2,
    SENDER_INIT_ERR_KDF     = -3,
	SENDER_INIT_ERR_BAD_PARAM = -4
} sender_init_status_t;
typedef enum {
    RECEIVER_INIT_SUCCESS      =  0,
    RECEIVER_INIT_ERR_PUBKEY   = -1,
    RECEIVER_INIT_ERR_ECDH     = -2,
    RECEIVER_INIT_ERR_KDF      = -3,
	RECEIVER_INIT_ERR_BAD_PARAM = -4
} receiver_init_status_t;
typedef enum {
    GCM_ENC_SUCCESS          =   0,
    GCM_ENC_ERR_BAD_INPUT    = -10,
    GCM_ENC_ERR_KEY_OR_IV    = -11,
    GCM_ENC_ERR_CONSTRUCT    = -12,
    GCM_ENC_ERR_INIT         = -13,
    GCM_ENC_ERR_SETKEY       = -14,
    GCM_ENC_ERR_SETIV        = -15,
    GCM_ENC_ERR_SETTAGLEN    = -16,
    GCM_ENC_ERR_APPEND       = -17,
    GCM_ENC_ERR_GENERATETAG  = -18,
    GCM_ENC_ERR_LENGTH_MISM  = -19,
    GCM_ENC_ERR_CLEANUP      = -20
} gcm_enc_status_t;
typedef enum {
    GCM_DEC_SUCCESS        =   0,
    GCM_DEC_ERR_BAD_INPUT  = -40,
    GCM_DEC_ERR_KEY_OR_IV  = -41,
    GCM_DEC_ERR_CONSTRUCT  = -42,
    GCM_DEC_ERR_INIT       = -43,
    GCM_DEC_ERR_SETKEY    = -44,
    GCM_DEC_ERR_SETIV     = -45,
    GCM_DEC_ERR_SETTAGLEN = -46,
    GCM_DEC_ERR_APPEND    = -47,
    GCM_DEC_ERR_VERIFY    = -48,
    GCM_DEC_ERR_CLEANUP   = -49
} gcm_dec_status_t;
typedef enum {
    GCM_AEAD_SINGLE_SUCCESS        =   0,
    GCM_AEAD_SINGLE_ERR_BAD_INPUT  = -30,
    GCM_AEAD_SINGLE_ERR_KEY_OR_IV  = -31,
    GCM_AEAD_SINGLE_ERR_AEAD_FAIL  = -32,
    GCM_AEAD_SINGLE_ERR_LENGTH     = -33
} gcm_aead_single_status_t;
typedef enum {
    GCM_AEAD_DEC_SUCCESS        =   0,
    GCM_AEAD_DEC_ERR_BAD_INPUT  = -60,
    GCM_AEAD_DEC_ERR_KEY_OR_IV  = -61,
    GCM_AEAD_DEC_ERR_AEAD_FAIL  = -62,
    GCM_AEAD_DEC_ERR_LENGTH     = -63
} gcm_aead_dec_status_t;
typedef enum {
    ENCRYPT_GCM_MODE_STREAM_ONLY            = 0,
    ENCRYPT_GCM_MODE_SINGLE_ONLY            = 1,
    ENCRYPT_GCM_MODE_TRY_STREAM_THEN_SINGLE = 2
} encrypt_gcm_mode_t;
typedef enum {
    GCM_MASTER_SUCCESS           =   0,
    GCM_MASTER_ERR_INVALID_MODE  = -100,
    GCM_MASTER_ERR_ALL_FAILED    = -101
    //GEN_IV_ERR_BAD_PARAM         = -201
} gcm_master_status_t;
#ifndef AES_GCM_TAG_SIZE
#define AES_GCM_TAG_SIZE 16 /* recommended 128-bit tag */
#endif
/* Public-key format validator for your codebase.
 *
 * Accepts:
 *   - raw X||Y (64 bytes)
 *   - uncompressed form 0x04 || X || Y (65 bytes)
 *
 * Rejects:
 *   - NULL pointer
 *   - lengths other than 64 or 65
 *   - 65-byte value whose first byte != 0x04
 *   - keys that are all-zero (obvious invalid)
 *
 * Return codes:
 *   0  => valid
 *  -1  => NULL input
 *  -2  => invalid length/format
 *  -3  => key is all-zero (invalid)
 */
typedef enum {
    PUBKEY_OK           =  0,
    PUBKEY_ERR_NULL     = -1,
    PUBKEY_ERR_FORMAT   = -2,
    PUBKEY_ERR_ALLZERO  = -3
} pubkey_status_t;
typedef enum {
    HMAC_SENDER_SUCCESS        =  0,
    HMAC_SENDER_ERR_GEN_IV     = -1,
    HMAC_SENDER_ERR_AES_CTR    = -2,
    HMAC_SENDER_ERR_HMAC       = -3,
	HMAC_SENDER_ERR_BAD_PARAM  = -4
} hmac_sender_status_t;
typedef enum {
    VERIFY_DEC_SUCCESS              =  0,
    VERIFY_DEC_ERR_HMAC_COMPUTE     = -1,
    VERIFY_DEC_ERR_MAC_LEN          = -2,
    VERIFY_DEC_ERR_MAC_MISMATCH     = -3,
    VERIFY_DEC_ERR_AES_CTR_DEC      = -4,
    VERIFY_DEC_ERR_LENGTH_MISMATCH  = -5,
    VERIFY_DEC_ERR_ADC_MISMATCH     = -6,
	VERIFY_DEC_ERR_BAD_PARAM        = -7
} verify_dec_status_t;
typedef enum {
    SECURE_CLEAR_SUCCESS           =  0,
    SECURE_CLEAR_ERR_INVALID_SIZE  = -1,
    SECURE_CLEAR_ERR_VERIFY_FAIL   = -3
} secure_clear_status_t;
typedef enum {
    GCM_SENDER_SUCCESS             =  0,
    GCM_SENDER_ERR_GEN_IV          = -1,
    GCM_SENDER_ERR_GCM_STREAM      = -2,
    GCM_SENDER_ERR_GCM_SINGLECALL  = -3,
	GCM_SENDER_ERR_BAD_PARAM       = -4
} gcm_sender_status_t;
typedef enum {
    GCM_RECEIVER_SUCCESS              =   0,
    GCM_RECEIVER_ERR_GCM_STREAM       =  -1,
    GCM_RECEIVER_ERR_GCM_SINGLECALL   =  -2,
    GCM_RECEIVER_ERR_EMPTY_PLAINTEXT  =  -3,
    GCM_RECEIVER_ERR_ADC_MISMATCH     =  -4,
    GCM_RECEIVER_ERR_BAD_PARAM        =  -5,
    GCM_RECEIVER_ERR_FRAME_LENGTH     =  -6,
    GCM_RECEIVER_ERR_FRAME_CHECKSUM   =  -7,
    GCM_RECEIVER_ERR_FRAME_HEADER     =  -8,
    GCM_RECEIVER_ERR_STALE_SEQUENCE   =  -9,
    GCM_RECEIVER_ERR_FRAME_TYPE       = -10,
    GCM_RECEIVER_ERR_REPLAY           = -11,
    GCM_RECEIVER_ERR_NVM              = -12
} gcm_receiver_status_t;
typedef struct
{
    uint8_t message_type;
    uint8_t reason;
    uint32_t sequence;
} gcm_uart_response_t;
typedef enum {
    RECOMBINE_OK = 0,
    RECOMBINE_ERR_NULL_CTX,
    RECOMBINE_ERR_BAD_CONFIG,
    RECOMBINE_ERR_LENGTH_MISMATCH
} recombine_status_t;
typedef enum {
    DECOMP_ADC_OK = 0,
    DECOMP_ADC_NULL_CTX,
    DECOMP_ADC_NULL_BUFFER,
    DECOMP_ADC_INVALID_CFG
} decomp_adc_ret_t;
typedef struct {
    struct { size_t adc_sample_count; size_t bytes_per_sample; } cfg;
    struct {
        uint32_t adc_values[ADC_SAMPLE_COUNT];
        uint32_t decrypted_adc_values[ADC_SAMPLE_COUNT];
        uint8_t  plain_text_bytes[UART_GCM_PLAINTEXT_SIZE];
        uint8_t  decrypted_bytes[UART_GCM_PLAINTEXT_SIZE];
        uint8_t  cipher_text_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE + AES_256_IV_SIZE];
        /*
         * AES-GCM combined ciphertext || tag buffers.
         * 169-byte ciphertext + 16-byte tag = 185 bytes.
         */
        uint8_t  cipher_text_gcm_aead[UART_GCM_CIPHERTEXT_TAG_SIZE];
        uint8_t  cipher_text_gcm_aead_receiver[UART_GCM_CIPHERTEXT_TAG_SIZE];
    } buffers;
    struct {
        uint8_t private_key_sender[AES_256_KEY_SIZE];
        uint8_t private_key_receiver[AES_256_KEY_SIZE];
        uint8_t public_key_sender[AES_512_KEY_SIZE];
        uint8_t public_key_receiver[AES_512_KEY_SIZE];
        size_t priv_len_sender, pub_len_sender;
        size_t priv_len_receiver, pub_len_receiver;
    } keys;
    struct {
        uint8_t aes_key[32];
        uint8_t aes_key_receiver[32];
        uint8_t mac_key[32];
        uint8_t mac_key_receiver[32];
    } derived;
    cmox_ctr_drbg_handle_t drbg;
    cmox_drbg_handle_t *drbg_ctx;
    cmox_ecc_handle_t ecc;
    uint8_t working_buffer[2000];
    struct {
        uint8_t mac[32];
        uint8_t mac_check[32];
        uint8_t tag_sender[AES_GCM_TAG_SIZE];
        uint8_t ciphertext_sender[UART_GCM_PLAINTEXT_SIZE];
        size_t  ciphertext_len;
        size_t  mac_len;
        size_t  mac_check_len;
    } auth;
    struct {
        uint8_t computed_secret[ECDH_CANONICAL_SECRET_LEN];
        size_t computed_secret_len;
        uint8_t computed_secret_receiver[ECDH_CANONICAL_SECRET_LEN];
        size_t computed_secret_receiver_len;
        uint8_t local_tag[GCM_GCM_TAG_SIZE];
        size_t computed_size;
        size_t plain_len;
    } tmp;
    uint8_t Key[AES_256_KEY_SIZE];
    uint8_t IV[AES_256_IV_SIZE];
    uint8_t IV_GCM[AES_GCM_AEAD_IV_SIZE];
    uint8_t IV_GCM_receiver[AES_GCM_AEAD_IV_SIZE];
    gcm_nonce_state_t gcm_nonce;
} crypto_ctx_t;
/*
 * Compile-time guards for the STM32 -> ESP32 telemetry transaction.
 * These catch any future accidental regression to the old 32-byte ADC-sized
 * AES-GCM buffers.
 */
_Static_assert(
    sizeof(((crypto_ctx_t *)0)->buffers.plain_text_bytes) >=
        UART_GCM_PLAINTEXT_SIZE,
    "AES-GCM plaintext buffer too small for telemetry");
_Static_assert(
    sizeof(((crypto_ctx_t *)0)->buffers.cipher_text_gcm_aead) >=
        UART_GCM_CIPHERTEXT_TAG_SIZE,
    "AES-GCM ciphertext/tag buffer too small for telemetry");
_Static_assert(
    sizeof(((crypto_ctx_t *)0)->buffers.cipher_text_gcm_aead_receiver) >=
        UART_GCM_CIPHERTEXT_TAG_SIZE,
    "AES-GCM receiver ciphertext/tag buffer too small for telemetry");
_Static_assert(
    sizeof(((crypto_ctx_t *)0)->auth.ciphertext_sender) >=
        UART_GCM_PLAINTEXT_SIZE,
    "AES-GCM sender ciphertext buffer too small for telemetry");
void handle_error(const char *message);
void crypto_ctx_init(crypto_ctx_t *ctx);
decomp_adc_ret_t decompose_adc_values(crypto_ctx_t *ctx);
recombine_status_t recombine_adc_values(crypto_ctx_t *ctx);
gen_iv_status_t generate_iv(crypto_ctx_t *ctx);
gen_priv_status_t generate_private_key(crypto_ctx_t *ctx);
gen_keys_status_t generate_keypairs_cmox(crypto_ctx_t *ctx);
ecdh_status_t compute_ecdh_shared_as_sender(crypto_ctx_t *ctx, uint8_t *out_secret, size_t *out_len);
pubkey_status_t validate_public_key_sender_format_ctx(const crypto_ctx_t *ctx,
                                                      const uint8_t *pub,
                                                      size_t pub_len);
ecdh_status_t compute_ecdh_shared_as_receiver(crypto_ctx_t *ctx,
                                              uint8_t *out_secret,
                                              size_t *out_len);
kdf_status_t derive_keys_from_shared_secret(crypto_ctx_t *ctx, bool receiver);
kdf_status_t derive_keys_from_shared_secret_receiver(crypto_ctx_t *ctx);
aes_ctr_enc_status_t encrypt_aes_ctr(crypto_ctx_t *ctx,
                                    size_t *out_computed_size);
aes_ctr_dec_status_t decrypt_aes_ctr(crypto_ctx_t *ctx,
                                    size_t *out_decrypted_size);
hmac_ivc_status_t compute_hmac_over_iv_cipher(crypto_ctx_t *ctx,
                                              uint8_t *mac_out,
                                              size_t mac_out_len,
                                              size_t *out_mac_len);
hmac_ivc_status_t compute_hmac_over_iv_cipher_receiver(crypto_ctx_t *ctx,
                                                       uint8_t *mac_out,
                                                       size_t mac_out_len,
                                                       size_t *out_mac_len);
gen_iv_status_t generate_nonce_and_iv_per_transfer(crypto_ctx_t *ctx);
sender_init_status_t sender_init_ecdh_crypto(crypto_ctx_t *ctx);
receiver_init_status_t receiver_init_ecdh_crypto(crypto_ctx_t *ctx);
hmac_sender_status_t generate_nonce_iv_encrypt_hmac_sender(crypto_ctx_t *ctx);
verify_dec_status_t verify_mac_decrypt_recombine_receiver(crypto_ctx_t *ctx);
secure_clear_status_t secure_clear_decrypted_adc_values(crypto_ctx_t *ctx);
gcm_enc_status_t encrypt_aes_gcm_aead(crypto_ctx_t *ctx);
gcm_aead_single_status_t encrypt_aead_gcm_singlecall(crypto_ctx_t *ctx);
gcm_dec_status_t decrypt_verify_aes_gcm_receiver(crypto_ctx_t *ctx);
gcm_aead_dec_status_t decrypt_aead_gcm_singlecall(crypto_ctx_t *ctx);
gcm_sender_status_t generate_nonce_iv_encrypt_gcm_sender(crypto_ctx_t *ctx);
gcm_sender_status_t ecdh_encrypt_uart_payload_once(
    crypto_ctx_t *ctx,
    const uint8_t *uart_payload,
    size_t uart_payload_len,
    uint8_t nonce_out[AES_GCM_AEAD_IV_SIZE],
    uint8_t *ciphertext_and_tag_out,
    size_t output_capacity,
    size_t *output_len);
gcm_receiver_status_t decrypt_verify_recombine_gcm_receiver(
    crypto_ctx_t *ctx,
    uint32_t expected_sequence,
    gcm_uart_response_t *response_out);
gcm_master_status_t derive_gcm_iv_from_drbg(crypto_ctx_t *ctx);
gcm_nonce_status_t gcm_nonce_allocate(
    crypto_ctx_t *ctx);
gcm_nonce_status_t gcm_nonce_session_init(crypto_ctx_t *ctx);
#endif /* ECDH_H_ */
