/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */

#ifndef ECDH_H_
#define ECDH_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "uart.h"
#include <nonce.h>
#include "stm32f4xx.h"
#include "cmox_crypto.h"
#include <entropy_personalization_nonce_gen.h>
#include "digital_signature.h"

#define AES_512_KEY_SIZE        64
#define AES_256_KEY_SIZE        32  // 32 bytes = 256 bits
#define AES_256_IV_SIZE         16  // 16 bytes (CTR counter block)
#define AES_GCM_AEAD_IV_SIZE    12  // 16 bytes (CTR counter block)
#define GCM_GCM_TAG_SIZE        16  /* pro recommended tag size (128-bit) */
#define ADC_SAMPLE_COUNT        8
#define BYTES_PER_SAMPLE        sizeof(uint32_t)
#define AES_GCM_TAG_SIZE        16
/* error codes */
typedef enum {
    GEN_PRIV_SUCCESS            =  0,
    GEN_PRIV_ERR_CONSTRUCT      = -1,
    GEN_PRIV_ERR_INIT           = -2,
    GEN_PRIV_ERR_GEN_SENDER     = -3,
    GEN_PRIV_ERR_GEN_RECEIVER   = -4,
    GEN_PRIV_ERR_CLEANUP        = -5,
	GEN_PRIV_ERR_BAD_PARAM      = -6
} gen_priv_status_t;

/* error codes */
typedef enum {
    GEN_IV_SUCCESS        =  0,
    GEN_IV_ERR_CONSTRUCT  = -1,
    GEN_IV_ERR_INIT       = -2,
    GEN_IV_ERR_GENERATE   = -3,
    GEN_IV_ERR_CLEANUP    = -4,
	GEN_IV_ERR_BAD_PARAM  = -5
} gen_iv_status_t;


/* return codes */
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

/* return codes */
typedef enum {
    ECDH_SUCCESS          =  0,
    ECDH_ERR_ECDH_FAILED  = -1
} ecdh_status_t;


/* return codes */
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
    GCM_SENDER_ERR_GCM_SINGLECALL  = -3
} gcm_sender_status_t;

typedef enum {
    GCM_RECEIVER_SUCCESS            =  0,
    GCM_RECEIVER_ERR_GCM_STREAM     = -1,
    GCM_RECEIVER_ERR_GCM_SINGLECALL = -2,
    GCM_RECEIVER_ERR_EMPTY_PLAINTEXT= -3,
    GCM_RECEIVER_ERR_ADC_MISMATCH   = -4
} gcm_receiver_status_t;

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

 /*computed_size  must have been set by encrypt_aes_ctr() earlier */
typedef struct {
    /* config (read-only) */
    struct { size_t adc_sample_count; size_t bytes_per_sample; } cfg;

    /* buffers / IO */
    struct {
        uint32_t adc_values[ADC_SAMPLE_COUNT];
        uint32_t decrypted_adc_values[ADC_SAMPLE_COUNT];
        uint8_t  plain_text_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE];
        uint8_t  decrypted_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE];
        uint8_t  cipher_text_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE + AES_256_IV_SIZE];
        uint8_t  cipher_text_gcm_aead[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE + AES_GCM_TAG_SIZE];
        uint8_t  cipher_text_gcm_aead_receiver[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE + AES_GCM_TAG_SIZE];
    } buffers;

    /* asymmetric keys */
    struct {
        uint8_t private_key_sender[AES_256_KEY_SIZE];
        uint8_t private_key_receiver[AES_256_KEY_SIZE];
        uint8_t public_key_sender[AES_512_KEY_SIZE];
        uint8_t public_key_receiver[AES_512_KEY_SIZE];
        size_t priv_len_sender, pub_len_sender;
        size_t priv_len_receiver, pub_len_receiver;
    } keys;

    /* derived symmetric keys */
    struct {
        uint8_t aes_key[32];
        uint8_t aes_key_receiver[32];
        uint8_t mac_key[32];
        uint8_t mac_key_receiver[32];
    } derived;

    /* DRBG / ECC handles and working buffer */
    cmox_ctr_drbg_handle_t drbg;
    cmox_drbg_handle_t *drbg_ctx;
    cmox_ecc_handle_t ecc;
    uint8_t working_buffer[2000];

    /* AEAD / MAC state */
    struct {
        uint8_t mac[32];
        uint8_t mac_check[32];
        uint8_t tag_sender[AES_GCM_TAG_SIZE];
        uint8_t ciphertext_sender[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE];
        size_t  ciphertext_len;
        size_t  mac_len;
        size_t  mac_check_len;
    } auth;

    /* temporary / computed secrets */
    struct {
        uint8_t computed_secret[CMOX_ECC_SECP256R1_SECRET_LEN];
        uint8_t computed_secret_receiver[CMOX_ECC_SECP256R1_SECRET_LEN];
        uint8_t local_tag[GCM_GCM_TAG_SIZE];
        size_t computed_size;
        size_t plain_len;
    } tmp;

    /* IVs */
    uint8_t Key[AES_256_KEY_SIZE];
    uint8_t IV[AES_256_IV_SIZE];
    uint8_t IV_GCM[AES_GCM_AEAD_IV_SIZE];
} crypto_ctx_t;



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
sender_init_status_t generate_private_key_digital_signature(sig_keys_ctx_t *ctx);
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
gcm_receiver_status_t decrypt_verify_recombine_gcm_receiver(crypto_ctx_t *ctx);
gcm_master_status_t derive_gcm_iv_from_drbg(crypto_ctx_t *ctx);
gen_keys_status_t generate_ed25519_signing_keypair(sig_keys_ctx_t *ctx);


#endif /* ECDH_H_ */
