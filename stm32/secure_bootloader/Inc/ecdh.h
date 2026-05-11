/* Author: i fiksuari pas prog | Documentation date: 11 May 2026 */

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


#define AES_512_KEY_SIZE        64
#define AES_256_KEY_SIZE        32  // 32 bytes = 256 bits
#define AES_256_IV_SIZE         16  // 16 bytes (CTR counter block)
#define AES_GCM_AEAD_IV_SIZE    12  // 16 bytes (CTR counter block)

#define ADC_SAMPLE_COUNT        8
#define BYTES_PER_SAMPLE        sizeof(uint32_t)
#define AES_GCM_TAG_SIZE        16
/* error codes */
#define GEN_PRIV_SUCCESS           0
#define GEN_PRIV_ERR_CONSTRUCT    -1
#define GEN_PRIV_ERR_INIT         -2
#define GEN_PRIV_ERR_GEN_SENDER   -3
#define GEN_PRIV_ERR_GEN_RECEIVER -4
#define GEN_PRIV_ERR_CLEANUP      -5
/* error codes */
#define GEN_IV_SUCCESS          0
#define GEN_IV_ERR_CONSTRUCT   -1
#define GEN_IV_ERR_INIT        -2
#define GEN_IV_ERR_GENERATE    -3
#define GEN_IV_ERR_CLEANUP     -4

/* return codes */
#define GEN_KEYS_SUCCESS               0
#define GEN_KEYS_ERR_DRBG_CONSTRUCT   -1
#define GEN_KEYS_ERR_DRBG_INIT        -2
#define GEN_KEYS_ERR_DRBG_GENERATE    -3
#define GEN_KEYS_ERR_RANDOM_TOO_LARGE -4
#define GEN_KEYS_ERR_KEYGEN_SENDER    -5
#define GEN_KEYS_ERR_KEYGEN_RECEIVER  -6
#define GEN_KEYS_ERR_DRBG_CLEANUP     -7

/* return codes */
#define ECDH_SUCCESS            0
#define ECDH_ERR_ECDH_FAILED   -1

/* return codes */
#define KDF_SUCCESS          0
#define KDF_ERR_HASH_FAILED -1
#define KDF_ERR_BAD_INPUT   -2

/* ================= AES-CTR ENCRYPTION ================= */
#define AES_CTR_ENC_SUCCESS              0
#define AES_CTR_ENC_ERR_BAD_INPUT       -1
#define AES_CTR_ENC_ERR_KEY_OR_IV       -2
#define AES_CTR_ENC_ERR_CIPHER_FAIL     -3
#define AES_CTR_ENC_ERR_LENGTH_MISMATCH -4

/* ================= AES-CTR DECRYPTION ================= */
#define AES_CTR_DEC_SUCCESS              0
#define AES_CTR_DEC_ERR_BAD_INPUT       -1
#define AES_CTR_DEC_ERR_KEY_OR_IV       -2
#define AES_CTR_DEC_ERR_CIPHER_FAIL     -3
#define AES_CTR_DEC_ERR_LENGTH_MISMATCH -4

/* ---------- HMAC over IV || Ciphertext return codes ---------- */
#define HMAC_IVC_SUCCESS              0
#define HMAC_IVC_ERR_NULL_INPUT      -1
#define HMAC_IVC_ERR_NO_CIPHERTEXT   -2
#define HMAC_IVC_ERR_SIZE_INVALID    -3
#define HMAC_IVC_ERR_SIZE_OVERFLOW   -4
#define HMAC_IVC_ERR_ALLOC_FAIL      -5
#define HMAC_IVC_ERR_MAC_FAILED      -6

/* Return codes */
#define SENDER_INIT_SUCCESS        0
#define SENDER_INIT_ERR_GENKEY    -1
#define SENDER_INIT_ERR_ECDH      -2
#define SENDER_INIT_ERR_KDF       -3

/* Return codes */
#define RECEIVER_INIT_SUCCESS        0
#define RECEIVER_INIT_ERR_PUBKEY    -1
#define RECEIVER_INIT_ERR_ECDH      -2
#define RECEIVER_INIT_ERR_KDF       -3


#define GCM_ENC_SUCCESS            0
#define GCM_ENC_ERR_BAD_INPUT     -10
#define GCM_ENC_ERR_KEY_OR_IV     -11
#define GCM_ENC_ERR_CONSTRUCT     -12
#define GCM_ENC_ERR_INIT          -13
#define GCM_ENC_ERR_SETKEY        -14
#define GCM_ENC_ERR_SETIV         -15
#define GCM_ENC_ERR_SETTAGLEN     -16
#define GCM_ENC_ERR_APPEND        -17
#define GCM_ENC_ERR_GENERATETAG   -18
#define GCM_ENC_ERR_LENGTH_MISM   -19
#define GCM_ENC_ERR_CLEANUP       -20
#define GCM_GCM_TAG_SIZE          16  /* pro recommended tag size (128-bit) */

#define GCM_DEC_SUCCESS             0
#define GCM_DEC_ERR_BAD_INPUT     -40
#define GCM_DEC_ERR_KEY_OR_IV     -41
#define GCM_DEC_ERR_CONSTRUCT     -42
#define GCM_DEC_ERR_INIT          -43
#define GCM_DEC_ERR_SETKEY        -44
#define GCM_DEC_ERR_SETIV         -45
#define GCM_DEC_ERR_SETTAGLEN     -46
#define GCM_DEC_ERR_APPEND        -47
#define GCM_DEC_ERR_VERIFY        -48
#define GCM_DEC_ERR_CLEANUP       -49

#define GCM_AEAD_SINGLE_SUCCESS        0
#define GCM_AEAD_SINGLE_ERR_BAD_INPUT -30
#define GCM_AEAD_SINGLE_ERR_KEY_OR_IV -31
#define GCM_AEAD_SINGLE_ERR_AEAD_FAIL -32
#define GCM_AEAD_SINGLE_ERR_LENGTH    -33

#define GCM_AEAD_DEC_SUCCESS         0
#define GCM_AEAD_DEC_ERR_BAD_INPUT  -60
#define GCM_AEAD_DEC_ERR_KEY_OR_IV  -61
#define GCM_AEAD_DEC_ERR_AEAD_FAIL  -62
#define GCM_AEAD_DEC_ERR_LENGTH     -63

#define ENCRYPT_GCM_MODE_STREAM_ONLY               0
#define ENCRYPT_GCM_MODE_SINGLE_ONLY               1
#define ENCRYPT_GCM_MODE_TRY_STREAM_THEN_SINGLE    2

#define GCM_MASTER_SUCCESS           0
#define GCM_MASTER_ERR_INVALID_MODE -100
#define GCM_MASTER_ERR_ALL_FAILED   -101
#define GEN_IV_ERR_BAD_PARAM        -201

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
#define PUBKEY_OK            0
#define PUBKEY_ERR_NULL     -1
#define PUBKEY_ERR_FORMAT   -2
#define PUBKEY_ERR_ALLZERO  -3

 /* computed_size must have been set by encrypt_aes_ctr() earlier */
extern uint8_t mac[32];
extern size_t mac_len;
extern uint8_t mac_check[32];
extern size_t mac_check_len;
extern size_t dec_len;
extern uint8_t received_mac[32];
extern size_t received_mac_len;

void handle_error(const char *message);
void decompose_adc_values(void);
void recombine_adc_values(void);
int validate_public_key_sender_format(const uint8_t *pub, size_t pub_len);
int generate_iv(void);
int generate_private_key(void);
int generate_keypairs_cmox(void);
int compute_ecdh_shared_as_sender(uint8_t *out_secret, size_t *out_len);
int compute_ecdh_shared_as_receiver(uint8_t *out_secret, size_t *out_len);
int derive_keys_from_shared_secret(uint8_t *aes_key, size_t aes_key_len,
                                   uint8_t *mac_key, size_t mac_key_len);
int derive_keys_from_shared_secret_receiver(uint8_t *aes_key, size_t aes_key_len,
                                            uint8_t *mac_key, size_t mac_key_len);
int encrypt_aes_ctr(size_t *out_computed_size);
int decrypt_aes_ctr(size_t *out_decrypted_size);
int compute_hmac_over_iv_cipher(uint8_t *mac_out, size_t mac_out_len, size_t *out_mac_len);

int compute_hmac_over_iv_cipher_receiver(uint8_t *mac_out,
                                        size_t mac_out_len,
                                        size_t *out_mac_len);
int generate_nonce_and_iv_per_transfer(void);
int sender_init_ecdh_crypto(void);
int receiver_init_ecdh_crypto(void);
int generate_nonce_iv_encrypt_hmac_sender(void);
int verify_mac_decrypt_recombine_receiver(void);
int secure_clear_decrypted_adc_values(void);
int encrypt_aes_gcm_aead(void);
int encrypt_aead_gcm_singlecall(void);
int decrypt_verify_aes_gcm_receiver(void);
int decrypt_aead_gcm_singlecall(void);
int generate_nonce_iv_encrypt_gcm_sender(void);
int decrypt_verify_recombine_gcm_receiver(void);
int derive_gcm_iv_from_drbg(void);



#endif /* ECDH_H_ */
