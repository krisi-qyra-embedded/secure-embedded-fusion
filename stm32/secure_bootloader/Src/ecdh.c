/*
 * ecdh_refactored.c
 *
 * Refactored from original ecdh.c — context-based, safer,
 * better error handling, constant-time checks, fewer globals.
 *
 * NOTES / REQUIREMENTS (provided elsewhere in your project):
 *  - macros and constants: ADC_SAMPLE_COUNT, BYTES_PER_SAMPLE,
 *    AES_256_KEY_SIZE, AES_256_IV_SIZE, AES_512_KEY_SIZE,
 *    CMOX_* algorithm constants, CMOX_ECC_SECP256R1_SECRET_LEN, etc.
 *  - functions: secure_zero(void *p, size_t n), log_printf(const char*, ...),
 *               nonce_generation(void), entropy_hashed[], device_uid_buffer_hashed[],
 *               nonce_hashed[], cmox_* library symbols, and platform-specific APIs.
 *
 * Usage:
 *  - create a crypto_ctx_t (static or allocated)
 *  - call crypto_ctx_init(&ctx)
 *  - fill ctx.adc_values (or let DMA write into ctx.dma_adc_buffer)
 *  - call sender_init_ecdh_crypto(&ctx) and/or receiver_init_ecdh_crypto(&ctx)
 *  - call generate_nonce_iv_encrypt_hmac_sender(&ctx)
 *  - transfer (IV || ciphertext || MAC) to peer
 *  - receiver calls receiver_init_ecdh_crypto(&ctx) and verify_mac_decrypt_recombine_receiver(&ctx)
 */
/* Author: i fiksuari pas prog | Documentation date: 8 May 2026 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>

#include "ecdh.h" /* your project header with macros & prototypes */



 uint32_t adc_values[ADC_SAMPLE_COUNT];
 uint32_t decrypted_adc_values[ADC_SAMPLE_COUNT];
 uint8_t plain_text_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE];
/* same size as plain_text_bytes */
 uint8_t decrypted_bytes[sizeof(plain_text_bytes)];
 uint8_t cipher_text_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE + AES_256_IV_SIZE]; // extra slack
/*uint8_t decrypted_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE + AES_256_IV_SIZE];*/

/* ---------- DRBG-based key/IV generation (kept from your original) ---------- */
 uint8_t Key[AES_256_KEY_SIZE] = {0};
 uint8_t IV[AES_256_IV_SIZE] = {0};
/* ---------- DRBG-based private key ---------- */
uint8_t private_key_sender[AES_256_KEY_SIZE] = {0};
uint8_t private_key_receiver[AES_256_KEY_SIZE] = {0};
/* ---------- ecdsa-based public key ---------- */
uint8_t public_key_sender[AES_512_KEY_SIZE]   = {0};
uint8_t public_key_receiver[AES_512_KEY_SIZE] = {0};

/* store produced lengths so caller can use exact pub/priv sizes */
 size_t priv_len_sender = 0;
 size_t pub_len_sender  = 0;
 size_t priv_len_receiver = 0;
 size_t pub_len_receiver  = 0;

 uint8_t buf[CMOX_ECC_SECP256R1_SECRET_LEN + 1];
 uint8_t aes_key[32];
 uint8_t aes_key_receiver[32];
 uint8_t mac_key[32];
 uint8_t mac_key_receiver[32];
 size_t out_len;

/* Computed data buffer */
 uint8_t computed_secret[CMOX_ECC_SECP256R1_SECRET_LEN];
 uint8_t computed_secret_receiver[CMOX_ECC_SECP256R1_SECRET_LEN];
 uint8_t my_secret[CMOX_ECC_SECP256R1_SECRET_LEN];
 uint8_t my_secret_receiver[CMOX_ECC_SECP256R1_SECRET_LEN];
 size_t my_len = sizeof(computed_secret);
 size_t my_len_receiver = sizeof(computed_secret_receiver);
 size_t plain_len = sizeof(plain_text_bytes); /* exactly ADC_SAMPLE_COUNT * 4 */
 size_t computed_size;

 cmox_ctr_drbg_handle_t con_drbg_ctx;
/*ECC context handle*/
 cmox_ecc_handle_t ecc_ctx;
/*ECC working buffer*/
 uint8_t working_buffer[2000];

 /* computed_size must have been set by encrypt_aes_ctr() earlier */
 uint8_t mac[32];
 size_t mac_len = 0;
 uint8_t mac_check[32];
 size_t mac_check_len = 0;
 size_t dec_len = 0;
 uint8_t received_mac[32];
 size_t received_mac_len = 32;
 /* global, next to your other globals in ecdh.c */
 static uint8_t mac_sender_input[ sizeof(IV) + sizeof(cipher_text_bytes)];
 /* preallocated global for receiver HMAC input (IV || ciphertext) */
 static uint8_t mac_receiver_input[ sizeof(IV) + sizeof(cipher_text_bytes)];

// const uint8_t entropy[] = {
//     0x4C, 0xFB, 0x21, 0x86, 0x73, 0x34, 0x6D, 0x9D,
//     0x50, 0xC9, 0x22, 0xE4, 0x9B, 0x0D, 0xFC, 0xD0,
//     0x90, 0xAD, 0xF0, 0x4F, 0x5C, 0x3B, 0xA4, 0x73,
//     0x27, 0xDF, 0xCD, 0x6F, 0xA6, 0x3A, 0x78, 0x5C
// };
//
// const uint8_t nonce[] = {
//     0x01, 0x69, 0x62, 0xA7, 0xFD, 0x27, 0x87, 0xA2,
//     0x4B, 0xF6, 0xBE, 0x47, 0xEF, 0x37, 0x83, 0xF1
// };
//
// const uint8_t personalization[] = {
//     0x88, 0xEE, 0xB8, 0xE0, 0xE8, 0x3B, 0xF3, 0x29,
//     0x4B, 0xDA, 0xCD, 0x60, 0x99, 0xEB, 0xE4, 0xBF,
//     0x55, 0xEC, 0xD9, 0x11, 0x3F, 0x71, 0xE5, 0xEB,
//     0xCB, 0x45, 0x75, 0xF3, 0xD6, 0xA6, 0x8A, 0x6B
// };



void handle_error(const char *message) {

    // Print the error message to the debug console
    log_printf("ERROR: %s\r\n", message);

    while (1) {
        // Toggle an LED to indicate an error (assuming LED_PIN is configured)
        //GPIOA->ODR ^= LED_PIN;  // Toggle LED
        for (volatile int i = 0; i < 1000000; i++);  // Delay loop
    }

}



/* Converts ADC sample words into bytes for encryption. */
void decompose_adc_values(void) {
    /* Extract the bytes in the intended order for encryption. */
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        uint32_t adc_value = adc_values[i];
        plain_text_bytes[i * 4]     = (adc_value >> 24) & 0xFF;
        plain_text_bytes[i * 4 + 1] = (adc_value >> 16) & 0xFF;
        plain_text_bytes[i * 4 + 2] = (adc_value >> 8) & 0xFF;
        plain_text_bytes[i * 4 + 3] = adc_value & 0xFF;
    }
}

/* Reconstructs ADC sample words from decrypted bytes. */
void recombine_adc_values(void) {
    /* Recombine bytes in the same order as the original samples. */
    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        decrypted_adc_values[i] = (decrypted_bytes[i * 4] << 24) |
                                  (decrypted_bytes[i * 4 + 1] << 16) |
                                  (decrypted_bytes[i * 4 + 2] << 8) |
                                  decrypted_bytes[i * 4 + 3];
    }
}



/* Clears sensitive or temporary data from memory. */
static int is_all_zero(const uint8_t *buf, size_t len)
{
    /* Clear the buffers before releasing the key material. */
    uint8_t acc = 0;
    for (size_t i = 0; i < len; ++i) acc |= buf[i];
    return (acc == 0);
}

/* Validates the supplied data or peripheral state. */
int validate_public_key_sender_format(const uint8_t *pub, size_t pub_len)
{
    if (pub == NULL) return PUBKEY_ERR_NULL;

    /* Accept raw X||Y (64) or uncompressed 0x04||X||Y (65) */
    if (pub_len == AES_512_KEY_SIZE) {
        if (is_all_zero(pub, AES_512_KEY_SIZE)) return PUBKEY_ERR_ALLZERO;
        return PUBKEY_OK;
    }

    if (pub_len == (AES_512_KEY_SIZE + 1)) {
        /* first byte must be 0x04 (uncompressed) */
        if (pub[0] != 0x04) return PUBKEY_ERR_FORMAT;
        if (is_all_zero(&pub[1], AES_512_KEY_SIZE)) return PUBKEY_ERR_ALLZERO;
        return PUBKEY_OK;
    }

    /* otherwise unsupported/invalid length */
    return PUBKEY_ERR_FORMAT;
}

/**
 * Generate a DRBG-based IV.
 * Returns 0 on success, negative error code on failure.
 * On failure the IV buffer is zeroed.
 */
int generate_iv(void)
{
    cmox_drbg_retval_t retval;
    cmox_drbg_handle_t *drbg_ctx = NULL;

    /* clear IV upfront to avoid reuse on failure */
    memset(IV, 0, sizeof(IV));

    drbg_ctx = cmox_ctr_drbg_construct(&con_drbg_ctx, CMOX_CTR_DRBG_AES256);
    if (drbg_ctx == NULL) {
        return GEN_IV_ERR_CONSTRUCT;
    }

    retval = cmox_drbg_init(drbg_ctx,
    						entropy_hashed, sizeof(entropy_hashed),
    						device_uid_buffer_hashed, sizeof(device_uid_buffer_hashed),
    						nonce_hashed, sizeof(nonce_hashed));
    if (retval != CMOX_DRBG_SUCCESS) {
        (void)cmox_drbg_cleanup(drbg_ctx);
        return GEN_IV_ERR_INIT;
    }

    retval = cmox_drbg_generate(drbg_ctx, NULL, 0, IV, AES_256_IV_SIZE);
    if (retval != CMOX_DRBG_SUCCESS) {
        memset(IV, 0, sizeof(IV));
        (void)cmox_drbg_cleanup(drbg_ctx);
        return GEN_IV_ERR_GENERATE;
    }

    retval = cmox_drbg_cleanup(drbg_ctx);
    if (retval != CMOX_DRBG_SUCCESS) {
        memset(IV, 0, sizeof(IV));
        return GEN_IV_ERR_CLEANUP;
    }

    return GEN_IV_SUCCESS;
}
/**
 * Generate two DRBG-based private keys.
 * Returns 0 on success, negative error code on failure (see defines above).
 * On failure the private_key_* buffers are zeroed.
 */
int generate_private_key(void)
{
    cmox_drbg_retval_t retval;
    cmox_drbg_handle_t *drbg_ctx = NULL;
    int ret = GEN_PRIV_SUCCESS;

    /* zero buffers at start to avoid leaking previous values on failure */
    memset(private_key_sender, 0, sizeof(private_key_sender));
    memset(private_key_receiver, 0, sizeof(private_key_receiver));

    drbg_ctx = cmox_ctr_drbg_construct(&con_drbg_ctx, CMOX_CTR_DRBG_AES256);
    if (drbg_ctx == NULL) {
        return GEN_PRIV_ERR_CONSTRUCT;
    }

    retval = cmox_drbg_init(drbg_ctx,
    						entropy_hashed, sizeof(entropy_hashed),
    					    device_uid_buffer_hashed, sizeof(device_uid_buffer_hashed),
    						nonce_hashed, sizeof(nonce_hashed));
    if (retval != CMOX_DRBG_SUCCESS) {
        /* attempt cleanup, ignore its return value */
        (void)cmox_drbg_cleanup(drbg_ctx);
        return GEN_PRIV_ERR_INIT;
    }

    retval = cmox_drbg_generate(drbg_ctx, NULL, 0, private_key_sender, AES_256_KEY_SIZE);
    if (retval != CMOX_DRBG_SUCCESS) {
        /* zero both keys to be safe */
        memset(private_key_sender, 0, sizeof(private_key_sender));
        memset(private_key_receiver, 0, sizeof(private_key_receiver));
        (void)cmox_drbg_cleanup(drbg_ctx);
        return GEN_PRIV_ERR_GEN_SENDER;
    }

    retval = cmox_drbg_generate(drbg_ctx, NULL, 0, private_key_receiver, AES_256_KEY_SIZE);
    if (retval != CMOX_DRBG_SUCCESS) {
        /* zero both keys to be safe */
        memset(private_key_sender, 0, sizeof(private_key_sender));
        memset(private_key_receiver, 0, sizeof(private_key_receiver));
        (void)cmox_drbg_cleanup(drbg_ctx);
        return GEN_PRIV_ERR_GEN_RECEIVER;
    }

    retval = cmox_drbg_cleanup(drbg_ctx);
    if (retval != CMOX_DRBG_SUCCESS) {
        /* cleanup failed but keys were generated — zero keys to avoid partial state */
        memset(private_key_sender, 0, sizeof(private_key_sender));
        memset(private_key_receiver, 0, sizeof(private_key_receiver));
        return GEN_PRIV_ERR_CLEANUP;
    }

    return ret;
}
/* small helper to fill randomness via DRBG, returns 0 on success or GEN_KEYS_ERR_DRBG_GENERATE */
static int drbg_fill_bytes(cmox_drbg_handle_t *drbg, uint8_t *out, size_t outlen)
{
    /* Check the generator status before using the output. */
    cmox_drbg_retval_t r = cmox_drbg_generate(drbg, NULL, 0, out, outlen);
    return (r == CMOX_DRBG_SUCCESS) ? 0 : GEN_KEYS_ERR_DRBG_GENERATE;
}


/**
 * Generate two ECDSA (secp256r1) keypairs (priv + pub) for sender and receiver.
 * Returns 0 on success, negative error code on failure. On failure both key buffers are zeroed.
 * After success, pub_len_sender/pub_len_receiver contain the actual public-key lengths.
 */
int generate_keypairs_cmox(void)
{
    cmox_drbg_handle_t *drbg_ctx = NULL;
    cmox_ecc_retval_t ecc_ret;
    cmox_drbg_retval_t drbg_ret;
    uint8_t random_buf[128];
    size_t random_len = AES_256_KEY_SIZE; /* initial randomness length */

    /* zero outputs and lens at start */
    memset(private_key_sender, 0, sizeof(private_key_sender));
    memset(private_key_receiver, 0, sizeof(private_key_receiver));
    memset(public_key_sender, 0, sizeof(public_key_sender));
    memset(public_key_receiver, 0, sizeof(public_key_receiver));
    priv_len_sender = pub_len_sender = priv_len_receiver = pub_len_receiver = 0;

    /* construct DRBG */
    drbg_ctx = cmox_ctr_drbg_construct(&con_drbg_ctx, CMOX_CTR_DRBG_AES256);
    if (drbg_ctx == NULL) {
        return GEN_KEYS_ERR_DRBG_CONSTRUCT;
    }

    drbg_ret = cmox_drbg_init(drbg_ctx,
    						  entropy_hashed, sizeof(entropy_hashed),
							  device_uid_buffer_hashed, sizeof(device_uid_buffer_hashed),
							  nonce_hashed, sizeof(nonce_hashed));
    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        (void)cmox_drbg_cleanup(drbg_ctx);
        return GEN_KEYS_ERR_DRBG_INIT;
    }

    /* ---------- GENERATE SENDER KEYPAIR ---------- */
    for (;;) {
        int drbg_rc = drbg_fill_bytes(drbg_ctx, random_buf, random_len);
        if (drbg_rc != 0) {
            (void)cmox_drbg_cleanup(drbg_ctx);
            return drbg_rc;
        }

        cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));

        size_t local_priv_len = sizeof(private_key_sender);
        size_t local_pub_len  = sizeof(public_key_sender);

        ecc_ret = cmox_ecdsa_keyGen(&ecc_ctx,
                                   CMOX_ECC_SECP256R1_LOWMEM,
                                   random_buf, random_len,
                                   private_key_sender, &local_priv_len,
                                   public_key_sender, &local_pub_len);

        cmox_ecc_cleanup(&ecc_ctx);

        if (ecc_ret == CMOX_ECC_SUCCESS) {
            /* store actual lengths */
            priv_len_sender = local_priv_len;
            pub_len_sender  = local_pub_len;
            break;
        }

        if (ecc_ret == CMOX_ECC_ERR_WRONG_RANDOM) {
            /* per docs priv_len contains the minimum random size requested */
            size_t needed = local_priv_len;
            if (needed > sizeof(random_buf)) {
                (void)cmox_drbg_cleanup(drbg_ctx);
                /* zero keys on failure */
                memset(private_key_sender, 0, sizeof(private_key_sender));
                memset(public_key_sender, 0, sizeof(public_key_sender));
                return GEN_KEYS_ERR_RANDOM_TOO_LARGE;
            }
            random_len = needed;
            continue; /* regenerate random with larger size */
        }

        /* other error */
        (void)cmox_drbg_cleanup(drbg_ctx);
        memset(private_key_sender, 0, sizeof(private_key_sender));
        memset(public_key_sender, 0, sizeof(public_key_sender));
        return GEN_KEYS_ERR_KEYGEN_SENDER;
    }

    /* reset random_len preference for receiver generation */
    random_len = AES_256_KEY_SIZE;

    /* ---------- GENERATE RECEIVER KEYPAIR ---------- */
    for (;;) {
        int drbg_rc = drbg_fill_bytes(drbg_ctx, random_buf, random_len);
        if (drbg_rc != 0) {
            /* zero produced sender keys before returning */
            memset(private_key_sender, 0, sizeof(private_key_sender));
            memset(public_key_sender, 0, sizeof(public_key_sender));
            (void)cmox_drbg_cleanup(drbg_ctx);
            return drbg_rc;
        }

        cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));

        size_t local_priv_len = sizeof(private_key_receiver);
        size_t local_pub_len  = sizeof(public_key_receiver);

        ecc_ret = cmox_ecdsa_keyGen(&ecc_ctx,
                                   CMOX_ECC_SECP256R1_LOWMEM,
                                   random_buf, random_len,
                                   private_key_receiver, &local_priv_len,
                                   public_key_receiver, &local_pub_len);

        cmox_ecc_cleanup(&ecc_ctx);

        if (ecc_ret == CMOX_ECC_SUCCESS) {
            priv_len_receiver = local_priv_len;
            pub_len_receiver  = local_pub_len;
            break;
        }

        if (ecc_ret == CMOX_ECC_ERR_WRONG_RANDOM) {
            size_t needed = local_priv_len;
            if (needed > sizeof(random_buf)) {
                /* zero both pairs and cleanup */
                memset(private_key_sender, 0, sizeof(private_key_sender));
                memset(public_key_sender, 0, sizeof(public_key_sender));
                memset(private_key_receiver, 0, sizeof(private_key_receiver));
                memset(public_key_receiver, 0, sizeof(public_key_receiver));
                (void)cmox_drbg_cleanup(drbg_ctx);
                return GEN_KEYS_ERR_RANDOM_TOO_LARGE;
            }
            random_len = needed;
            continue;
        }

        /* other error */
        memset(private_key_sender, 0, sizeof(private_key_sender));
        memset(public_key_sender, 0, sizeof(public_key_sender));
        memset(private_key_receiver, 0, sizeof(private_key_receiver));
        memset(public_key_receiver, 0, sizeof(public_key_receiver));
        (void)cmox_drbg_cleanup(drbg_ctx);
        return GEN_KEYS_ERR_KEYGEN_RECEIVER;
    }

    /* cleanup DRBG */
    drbg_ret = cmox_drbg_cleanup(drbg_ctx);
    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        /* zero all keys on cleanup failure */
        memset(private_key_sender, 0, sizeof(private_key_sender));
        memset(public_key_sender, 0, sizeof(public_key_sender));
        memset(private_key_receiver, 0, sizeof(private_key_receiver));
        memset(public_key_receiver, 0, sizeof(public_key_receiver));
        return GEN_KEYS_ERR_DRBG_CLEANUP;
    }

    return GEN_KEYS_SUCCESS;
}


/**
 * Compute ECDH shared secret as sender using only the previously-generated arrays.
 * Uses global computed_secret[] and my_len as storage.
 *
 * out_secret / out_len:
 *  - If both non-NULL: copy result there and update *out_len
 *  - If out_secret == NULL: result remains in computed_secret[] and my_len
 *
 * Returns 0 on success, negative value on failure.
 */
int compute_ecdh_shared_as_sender(uint8_t *out_secret, size_t *out_len)
{
    cmox_ecc_retval_t retval;

    /* -------- Try 1: raw X||Y public key (64 bytes) -------- */
    my_len = sizeof(computed_secret);

    cmox_ecc_construct(&ecc_ctx,CMOX_ECC256_MATH_FUNCS,working_buffer, sizeof(working_buffer));

    retval = cmox_ecdh(&ecc_ctx,
                       CMOX_ECC_CURVE_SECP256R1,
                       private_key_sender, AES_256_KEY_SIZE,
                       public_key_receiver, AES_512_KEY_SIZE,
                       computed_secret, &my_len);

    if (retval == CMOX_ECC_SUCCESS) {
        cmox_ecc_cleanup(&ecc_ctx);

        if (out_secret && out_len) {
            size_t to_copy = (my_len <= *out_len) ? my_len : *out_len;
            memcpy(out_secret, computed_secret, to_copy);
            *out_len = to_copy;
        } else if (out_len) {
            *out_len = my_len;
        }
        return ECDH_SUCCESS;
    }

    /* -------- Try 2: uncompressed point 0x04 || X || Y -------- */
    uint8_t pub_prefixed[65];
    pub_prefixed[0] = 0x04;
    memcpy(&pub_prefixed[1], public_key_receiver, AES_512_KEY_SIZE);

    my_len = sizeof(computed_secret);

    retval = cmox_ecdh(&ecc_ctx,
                       CMOX_ECC_CURVE_SECP256R1,
                       private_key_sender, AES_256_KEY_SIZE,
                       pub_prefixed, sizeof(pub_prefixed),
                       computed_secret, &my_len);

    cmox_ecc_cleanup(&ecc_ctx);

    if (retval != CMOX_ECC_SUCCESS) {
        /* clear secret on failure */
        memset(computed_secret, 0, sizeof(computed_secret));
        my_len = 0;
        return ECDH_ERR_ECDH_FAILED;
    }

    /* -------- Success with prefixed form -------- */
    if (out_secret && out_len) {
        size_t to_copy = (my_len <= *out_len) ? my_len : *out_len;
        memcpy(out_secret, computed_secret, to_copy);
        *out_len = to_copy;
    } else if (out_len) {
        *out_len = my_len;
    }

    return ECDH_SUCCESS;
}

/* Compute ECDH shared secret as receiver (mirror of sender).
 * Uses global computed_secret[] and my_len.
 */
int compute_ecdh_shared_as_receiver(uint8_t *out_secret, size_t *out_len)
{
    cmox_ecc_retval_t retval;

    /* Try raw X||Y first (64 bytes) */
    my_len = sizeof(computed_secret_receiver);

    cmox_ecc_construct(&ecc_ctx, CMOX_ECC256_MATH_FUNCS, working_buffer, sizeof(working_buffer));

    retval = cmox_ecdh(&ecc_ctx,
                       CMOX_ECC_CURVE_SECP256R1,
                       private_key_receiver, AES_256_KEY_SIZE,
                       public_key_sender, AES_512_KEY_SIZE,
					   computed_secret_receiver, &my_len);

    if (retval == CMOX_ECC_SUCCESS) {
        cmox_ecc_cleanup(&ecc_ctx);
        if (out_secret && out_len) {
            size_t to_copy = (my_len <= *out_len) ? my_len : *out_len;
            memcpy(out_secret, computed_secret_receiver, to_copy);
            *out_len = to_copy;
        } else if (out_len) {
            *out_len = my_len;
        }
        return ECDH_SUCCESS;
    }

    /* Try uncompressed point 0x04 || X || Y */
    uint8_t pub_prefixed[65];
    pub_prefixed[0] = 0x04;
    memcpy(&pub_prefixed[1], public_key_sender, AES_512_KEY_SIZE);

    my_len = sizeof(computed_secret_receiver);

    retval = cmox_ecdh(&ecc_ctx,
                       CMOX_ECC_CURVE_SECP256R1,
                       private_key_receiver, AES_256_KEY_SIZE,
                       pub_prefixed, sizeof(pub_prefixed),
					   computed_secret_receiver, &my_len);

    cmox_ecc_cleanup(&ecc_ctx);

    if (retval != CMOX_ECC_SUCCESS) {
        memset(computed_secret_receiver, 0, sizeof(computed_secret_receiver));
        my_len = 0;
        return ECDH_ERR_ECDH_FAILED;
    }

    if (out_secret && out_len) {
        size_t to_copy = (my_len <= *out_len) ? my_len : *out_len;
        memcpy(out_secret, computed_secret_receiver, to_copy);
        *out_len = to_copy;
    } else if (out_len) {
        *out_len = my_len;
    }

    return ECDH_SUCCESS;
}

/* Example usage:
   size_t outlen = 64;
   if (compute_ecdh_shared_as_sender(NULL, &outlen) == 0) {
       // computed_secret[0..outlen-1] contains the shared secret
   } else {
       handle_error("ECDH failed");
   }
*/

/**
 * Derive AES and MAC keys from ECDH shared secret.
 *
 * AES key = SHA256(shared || 0x01)
 * MAC key = SHA256(shared || 0x02)
 *
 * aes_key_len and mac_key_len must be 32 for SHA-256 output.
 *
 * Returns 0 on success, negative on failure.
 */
int derive_keys_from_shared_secret(uint8_t *aes_key, size_t aes_key_len,
                                   uint8_t *mac_key, size_t mac_key_len)
{
    cmox_hash_retval_t retval;
    uint8_t buf[CMOX_ECC_SECP256R1_SECRET_LEN + 1];
    size_t out_len;

    if (!aes_key || !mac_key) {
        return KDF_ERR_BAD_INPUT;
    }

    if (aes_key_len != 32 || mac_key_len != 32) {
        return KDF_ERR_BAD_INPUT;
    }

    if (my_len == 0 || my_len > CMOX_ECC_SECP256R1_SECRET_LEN) {
        return KDF_ERR_BAD_INPUT;
    }

    /* ---------- AES key derivation: SHA256(shared || 0x01) ---------- */
    memcpy(buf, computed_secret, my_len);
    buf[my_len] = 0x01;

    out_len = aes_key_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf, my_len + 1,
                               aes_key, aes_key_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS || out_len != aes_key_len) {
        memset(aes_key, 0, aes_key_len);
        return KDF_ERR_HASH_FAILED;
    }

    /* ---------- MAC key derivation: SHA256(shared || 0x02) ---------- */
    buf[my_len] = 0x02;

    out_len = mac_key_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf, my_len + 1,
                               mac_key, mac_key_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS || out_len != mac_key_len) {
        memset(aes_key, 0, aes_key_len);
        memset(mac_key, 0, mac_key_len);
        return KDF_ERR_HASH_FAILED;
    }

    /* clear temporary buffer */
    memset(buf, 0, sizeof(buf));

    return KDF_SUCCESS;
}

/**
 * Derive AES and MAC keys from receiver's ECDH shared secret.
 *
 * Uses computed_secret_receiver[] and my_len_receiver (populated by
 * compute_ecdh_shared_as_receiver()).
 *
 * AES key = SHA256(shared || 0x01)
 * MAC key = SHA256(shared || 0x02)
 *
 * aes_key_len and mac_key_len must be 32 for SHA-256 output.
 *
 * Returns 0 on success, negative on failure.
 */
int derive_keys_from_shared_secret_receiver(uint8_t *aes_key, size_t aes_key_len,
                                            uint8_t *mac_key, size_t mac_key_len)
{
    cmox_hash_retval_t retval;
    uint8_t buf[CMOX_ECC_SECP256R1_SECRET_LEN + 1];
    size_t out_len;

    if (!aes_key || !mac_key) {
        return KDF_ERR_BAD_INPUT;
    }

    if (aes_key_len != 32 || mac_key_len != 32) {
        return KDF_ERR_BAD_INPUT;
    }

    if (my_len_receiver == 0 || my_len_receiver > CMOX_ECC_SECP256R1_SECRET_LEN) {
        return KDF_ERR_BAD_INPUT;
    }

    /* ---------- AES key derivation: SHA256(shared || 0x01) ---------- */
    memcpy(buf, computed_secret_receiver, my_len_receiver);
    buf[my_len_receiver] = 0x01;

    out_len = aes_key_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf, my_len_receiver + 1,
                               aes_key, aes_key_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS || out_len != aes_key_len) {
        memset(aes_key, 0, aes_key_len);
        return KDF_ERR_HASH_FAILED;
    }

    /* ---------- MAC key derivation: SHA256(shared || 0x02) ---------- */
    buf[my_len_receiver] = 0x02;

    out_len = mac_key_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf, my_len_receiver + 1,
                               mac_key, mac_key_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS || out_len != mac_key_len) {
        memset(aes_key, 0, aes_key_len);
        memset(mac_key, 0, mac_key_len);
        return KDF_ERR_HASH_FAILED;
    }

    /* clear temporary buffer */
    memset(buf, 0, sizeof(buf));

    return KDF_SUCCESS;
}

/* robust AES-CTR encryption wrapper
 *
 * Behavior:
 *  - Validates inputs and buffer sizes
 *  - Detects obviously-bad key/IV (all-zero) and rejects
 *  - Uses a local output-length variable and only commits the global
 *    `computed_size` on success (avoids partial state)
 *  - Wipes output buffer on any failure
 *  - Returns clear, small-range negative error codes
 *
 * Return codes:
 *   0  => success
 *  -1  => invalid input / plaintext size out of bounds
 *  -2  => invalid key or IV (all zero)
 *  -3  => cipher primitive error
 *  -4  => ciphertext length unexpected / internal sanity
 */


static int is_all_zero_bytes(const uint8_t *p, size_t n)
{
    /* Clear the buffers before releasing the key material. */
    uint8_t acc = 0;
    for (size_t i = 0; i < n; ++i) acc |= p[i];
    return (acc == 0);
}

/* Robust, security-minded AES-CTR decrypt using C-MOX
 *
 * Behavior:
 *  - Strictly uses the dedicated decrypt primitive (CMOX_AESFAST_CTR_DEC_ALGO).
 *  - Performs extensive sanity checks on sizes and buffers.
 *  - Wipes partial output on any failure.
 *  - Returns negative error codes with distinct meanings.
 *
 * Return codes:
 *  0   => success
 * -1   => invalid input / size sanity check failed
 * -2   => cipher primitive returned error
 * -3   => length mismatch (CTR should be 1:1)
 * -4   => decrypted output buffer overflow / internal sanity
 *
 * NOTE: Define ALLOW_CTR_ENCRYPT_FALLBACK to allow the encryption primitive
 *       as a fallback if the dedicated decrypt algorithm macro is not present.
 *       Fallback is NOT recommended for audited code; prefer enabling the
 *       decrypt primitive in the C-MOX config.
 */


/* Uses existing project-wide error-code conventions */
int encrypt_aes_ctr(size_t *out_computed_size)
{
    /* Validate the crypto return value and output length. */
    cmox_cipher_retval_t cipher_ret;
    size_t local_out_len = 0;

    if (out_computed_size) {
        *out_computed_size = 0;
    }

    if (plain_len == 0 || plain_len > sizeof(cipher_text_bytes)) {
        return AES_CTR_ENC_ERR_BAD_INPUT;
    }

    if (is_all_zero_bytes(aes_key, sizeof(aes_key)) ||
        is_all_zero_bytes(IV, sizeof(IV))) {
        return AES_CTR_ENC_ERR_KEY_OR_IV;
    }

    computed_size = 0;

    cipher_ret = cmox_cipher_encrypt(
                    CMOX_AESFAST_CTR_ENC_ALGO,
                    plain_text_bytes, plain_len,
                    aes_key, sizeof(aes_key),
                    IV, sizeof(IV),
                    cipher_text_bytes, &local_out_len
                 );

    if (cipher_ret != CMOX_CIPHER_SUCCESS) {
        secure_zero(cipher_text_bytes, sizeof(cipher_text_bytes));
        return AES_CTR_ENC_ERR_CIPHER_FAIL;
    }

    if (local_out_len != plain_len) {
        secure_zero(cipher_text_bytes, sizeof(cipher_text_bytes));
        return AES_CTR_ENC_ERR_LENGTH_MISMATCH;
    }

    computed_size = local_out_len;

    if (out_computed_size) {
        *out_computed_size = computed_size;
    }

    return AES_CTR_ENC_SUCCESS;
}



/* Return codes:
 *  0  => success
 * -1  => invalid input / size sanity check failed
 * -2  => cipher primitive error
 * -3  => length mismatch (CTR should be 1:1)
 * -4  => decrypted output overflow (internal sanity)
 */
int decrypt_aes_ctr(size_t *out_decrypted_size)
{
    cmox_cipher_retval_t cipher_ret;
    size_t dec_size_local = 0;

    /* clear output size upfront (defensive) */
    if (out_decrypted_size) *out_decrypted_size = 0;

    /* basic sanity checks */
    if (computed_size == 0) {
        return AES_CTR_DEC_ERR_BAD_INPUT;
    }
    if (computed_size > sizeof(cipher_text_bytes)) {
        return AES_CTR_DEC_ERR_BAD_INPUT;
    }
    if (computed_size > sizeof(decrypted_bytes)) {
        return AES_CTR_DEC_ERR_LENGTH_MISMATCH;
    }

    /* Call the dedicated CTR decrypt primitive */
    cipher_ret = cmox_cipher_decrypt(
                    CMOX_AESFAST_CTR_DEC_ALGO,
                    cipher_text_bytes,
                    computed_size,
                    aes_key_receiver,
                    sizeof(aes_key_receiver),
                    IV,
                    sizeof(IV),
                    decrypted_bytes,
                    &dec_size_local
                 );

    if (cipher_ret != CMOX_CIPHER_SUCCESS) {
        /* wipe any partial output */
        secure_zero(decrypted_bytes, sizeof(decrypted_bytes));
        return AES_CTR_DEC_ERR_CIPHER_FAIL;
    }

    /* CTR must be length-preserving: plaintext length == ciphertext length */
    if (dec_size_local != computed_size) {
        secure_zero(decrypted_bytes, sizeof(decrypted_bytes));
        return AES_CTR_DEC_ERR_LENGTH_MISMATCH;
    }

    /* final sanity: ensure dec_size_local fits into buffer */
    if (dec_size_local > sizeof(decrypted_bytes)) {
        secure_zero(decrypted_bytes, sizeof(decrypted_bytes));
        return AES_CTR_DEC_ERR_LENGTH_MISMATCH;
    }

    if (out_decrypted_size) *out_decrypted_size = dec_size_local;
    return AES_CTR_DEC_SUCCESS;
}


/* Computes or verifies a keyed authentication tag. */
int compute_hmac_over_iv_cipher(uint8_t *mac_out,
                               size_t mac_out_len,
                               size_t *out_mac_len)
{
    cmox_mac_retval_t retval;
    size_t mac_input_len;

    /* Validate inputs */
    if (mac_out == NULL || out_mac_len == NULL) {
        return HMAC_IVC_ERR_NULL_INPUT;
    }

    /* Defensive clear */
    *out_mac_len = 0;

    /* Ensure ciphertext exists */
    if (computed_size == 0) {
        return HMAC_IVC_ERR_NO_CIPHERTEXT;
    }

    /* Ensure ciphertext length is sane */
    if (computed_size > sizeof(cipher_text_bytes)) {
        return HMAC_IVC_ERR_SIZE_INVALID;
    }

    /* Prevent size overflow: IV || ciphertext */
    if (computed_size > (SIZE_MAX - sizeof(IV))) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    mac_input_len = sizeof(IV) + computed_size;

    /* Ensure our static global buffer is large enough */
    if (mac_input_len > sizeof(mac_sender_input)) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    /* Build MAC input: IV || ciphertext using preallocated global buffer */
    memcpy(mac_sender_input, IV, sizeof(IV));
    memcpy(mac_sender_input + sizeof(IV), cipher_text_bytes, computed_size);

    /* Compute HMAC-SHA256 */
    retval = cmox_mac_compute(
                CMOX_HMAC_SHA256_ALGO,
                mac_sender_input, mac_input_len,
                mac_key, sizeof(mac_key),
                NULL, 0,
                mac_out, mac_out_len, out_mac_len
             );

    /* Wipe temporary buffer (only touched region) */
    secure_zero(mac_sender_input, mac_input_len);

    if (retval != CMOX_MAC_SUCCESS) {
        /* wipe output on failure */
        secure_zero(mac_out, mac_out_len);
        *out_mac_len = 0;
        return HMAC_IVC_ERR_MAC_FAILED;
    }

    return HMAC_IVC_SUCCESS;
}


/* Computes or verifies a keyed authentication tag. */
int compute_hmac_over_iv_cipher_receiver(uint8_t *mac_out,
                                         size_t mac_out_len,
                                         size_t *out_mac_len)
{
    cmox_mac_retval_t retval;
    size_t mac_input_len;

    /* Validate inputs */
    if (mac_out == NULL || out_mac_len == NULL) {
        return HMAC_IVC_ERR_NULL_INPUT;
    }

    /* Defensive clear */
    *out_mac_len = 0;

    /* Ensure ciphertext exists */
    if (computed_size == 0) {
        return HMAC_IVC_ERR_NO_CIPHERTEXT;
    }

    /* Ensure ciphertext length is sane */
    if (computed_size > sizeof(cipher_text_bytes)) {
        return HMAC_IVC_ERR_SIZE_INVALID;
    }

    /* Prevent IV || ciphertext size overflow */
    if (computed_size > (SIZE_MAX - sizeof(IV))) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    mac_input_len = sizeof(IV) + computed_size;

    /* Ensure our static global buffer is large enough */
    if (mac_input_len > sizeof(mac_receiver_input)) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    /* Build MAC input: IV || ciphertext using preallocated global buffer */
    memcpy(mac_receiver_input, IV, sizeof(IV));
    memcpy(mac_receiver_input + sizeof(IV), cipher_text_bytes, computed_size);

    /* Compute HMAC-SHA256 using receiver MAC key */
    retval = cmox_mac_compute(
                CMOX_HMAC_SHA256_ALGO,
                mac_receiver_input, mac_input_len,
                mac_key_receiver, sizeof(mac_key_receiver),
                NULL, 0,
                mac_out, mac_out_len, out_mac_len
             );

    /* Wipe temporary buffer (only touched region) */
    secure_zero(mac_receiver_input, mac_input_len);

    if (retval != CMOX_MAC_SUCCESS) {
        secure_zero(mac_out, mac_out_len);
        *out_mac_len = 0;
        return HMAC_IVC_ERR_MAC_FAILED;
    }

    return HMAC_IVC_SUCCESS;
}


/*
 * generate_nonce_and_iv_per_transfer
 *
 * Sequence:
 *   1) generate nonce
 *   2) generate IV
 *
 * Returns:
 *   - NONCE_GEN_* on nonce failure
 *   - GEN_IV_* on IV failure
 *   - GEN_IV_SUCCESS on success
 */
int generate_nonce_and_iv_per_transfer(void)
{
    int rc;

    /* 1) generate nonce */
    rc = nonce_generation();
    if (rc != NONCE_GEN_SUCCESS) {
        return rc;
    }

    /* 2) generate IV */
    rc = generate_iv();
    if (rc != GEN_IV_SUCCESS) {
        return rc;
    }

    return GEN_IV_SUCCESS;
}


/*
 * Uses globals declared in your ecdh.c:
 *   uint8_t private_key_sender[], public_key_sender[], my_secret[], aes_key[], mac_key[], size_t my_len;
 *
 * Steps:
 *  1) generate_keypairs_cmox()  -> fills priv/pub (sender & receiver pairs)
 *  2) compute_ecdh_shared_as_sender(my_secret, &my_len)
 *  3) derive_keys_from_shared_secret(aes_key, sizeof(aes_key), mac_key, sizeof(mac_key))
 *
 * On failure the function wipes sensitive outputs and returns a negative code.
 */
int sender_init_ecdh_crypto(void)
{
    int ret;
    /* ---------- 1) generate ECC keypairs ---------- */
    ret = generate_keypairs_cmox();
    if (ret != GEN_KEYS_SUCCESS) {
        log_printf("[ERROR] sender_init_ecdh_crypto: generate_keypairs_cmox failed (rc=%d)\r\n", ret);
        /* make sure nothing sensitive remains */
        secure_zero(private_key_sender, sizeof(private_key_sender));
        secure_zero(public_key_sender,  sizeof(public_key_sender));
        return SENDER_INIT_ERR_GENKEY;
    }

    /* ---------- 2) compute ECDH shared secret (sender side) ---------- */
    ret = compute_ecdh_shared_as_sender(my_secret, &my_len);
    if (ret != ECDH_SUCCESS) {
        log_printf("[ERROR] sender_init_ecdh_crypto: compute_ecdh_shared_as_sender failed (rc=%d)\r\n", ret);
        secure_zero(private_key_sender, sizeof(private_key_sender));
        secure_zero(public_key_sender,  sizeof(public_key_sender));
        secure_zero(my_secret,           sizeof(my_secret));
        my_len = 0;
        return SENDER_INIT_ERR_ECDH;
    }

    /* ---------- 3) derive AES + MAC keys from shared secret ---------- */
    ret = derive_keys_from_shared_secret(aes_key, sizeof(aes_key),
                                         mac_key, sizeof(mac_key));
    if (ret != KDF_SUCCESS) {
        log_printf("[ERROR] sender_init_ecdh_crypto: derive_keys_from_shared_secret failed (rc=%d)\r\n", ret);
        secure_zero(private_key_sender, sizeof(private_key_sender));
        secure_zero(public_key_sender,  sizeof(public_key_sender));
        secure_zero(my_secret,           sizeof(my_secret));
        secure_zero(aes_key,             sizeof(aes_key));
        secure_zero(mac_key,             sizeof(mac_key));
        my_len = 0;
        return SENDER_INIT_ERR_KDF;
    }

    /* success — keep keys in memory for use (caller responsible for later secure wipe) */
    log_printf("[INFO] sender_init_ecdh_crypto: success (shared_len=%u)\r\n", (unsigned)my_len);
    return SENDER_INIT_SUCCESS;
}

/*
 * Uses globals from ecdh.c:
 *   uint8_t public_key_sender[], pub_len_sender,
 *   uint8_t computed_secret_receiver[], my_secret_receiver[], aes_key_receiver[], mac_key_receiver[], size_t my_len_receiver;
 *
 * Steps:
 *  1) validate_public_key_sender_format(public_key_sender, pub_len_sender)
 *  2) compute_ecdh_shared_as_receiver(my_secret_receiver, &my_len_receiver)
 *  3) derive_keys_from_shared_secret_receiver(aes_key_receiver, sizeof(aes_key_receiver), mac_key_receiver, sizeof(mac_key_receiver))
 *
 * On failure the function wipes sensitive outputs and returns a negative code.
 */
int receiver_init_ecdh_crypto(void)
{
    int ret;

    /* ---------- 1) Validate sender public key format ---------- */
    ret = validate_public_key_sender_format(public_key_sender, pub_len_sender);
    if (ret != PUBKEY_OK) {
        if (ret == PUBKEY_ERR_NULL) {
            log_printf("[ERROR] receiver_init_ecdh_crypto: sender public key is NULL\r\n");
        } else if (ret == PUBKEY_ERR_FORMAT) {
            log_printf("[ERROR] receiver_init_ecdh_crypto: sender public key format/length invalid\r\n");
        } else if (ret == PUBKEY_ERR_ALLZERO) {
            log_printf("[ERROR] receiver_init_ecdh_crypto: sender public key is all-zero\r\n");
        } else {
            log_printf("[ERROR] receiver_init_ecdh_crypto: validate_public_key_sender_format returned %d\r\n", ret);
        }
        return RECEIVER_INIT_ERR_PUBKEY;
    }

    /* ---------- 2) compute ECDH shared secret (receiver side) ---------- */
    ret = compute_ecdh_shared_as_receiver(my_secret_receiver, &my_len_receiver);
    if (ret != ECDH_SUCCESS) {
        log_printf("[ERROR] receiver_init_ecdh_crypto: compute_ecdh_shared_as_receiver failed (rc=%d)\r\n", ret);
        secure_zero(computed_secret_receiver, sizeof(computed_secret_receiver));
        secure_zero(my_secret_receiver, sizeof(my_secret_receiver));
        my_len_receiver = 0;
        return RECEIVER_INIT_ERR_ECDH;
    }

    /* ---------- 3) derive AES + MAC keys from shared secret (receiver) ---------- */
    ret = derive_keys_from_shared_secret_receiver(aes_key_receiver, sizeof(aes_key_receiver),
                                                  mac_key_receiver, sizeof(mac_key_receiver));
    if (ret != KDF_SUCCESS) {
        log_printf("[ERROR] receiver_init_ecdh_crypto: derive_keys_from_shared_secret_receiver failed (rc=%d)\r\n", ret);
        secure_zero(computed_secret_receiver, sizeof(computed_secret_receiver));
        secure_zero(my_secret_receiver, sizeof(my_secret_receiver));
        secure_zero(aes_key_receiver, sizeof(aes_key_receiver));
        secure_zero(mac_key_receiver, sizeof(mac_key_receiver));
        my_len_receiver = 0;
        return RECEIVER_INIT_ERR_KDF;
    }

    log_printf("[INFO] receiver_init_ecdh_crypto: success (receiver_shared_len=%u)\r\n", (unsigned)my_len_receiver);
    return RECEIVER_INIT_SUCCESS;
}

/* Implement wrapper */
int generate_nonce_iv_encrypt_hmac_sender(void)
{
    int rc;

    /* 1) generate nonce and IV for this transfer */
    rc = generate_nonce_and_iv_per_transfer();
    if (rc != GEN_IV_SUCCESS) {
        log_printf("[ERROR] generate_nonce_iv_encrypt_hmac_sender: generate_nonce_and_iv_per_transfer failed (rc=%d)\r\n", rc);
        return rc; /* propagate GEN_IV_* */
    }

    /* 2) encrypt (AES-CTR) -> writes ciphertext and sets computed_size */
    rc = encrypt_aes_ctr(&computed_size);
    if (rc != AES_CTR_ENC_SUCCESS) {
        log_printf("[ERROR] generate_nonce_iv_encrypt_hmac_sender: encrypt_aes_ctr failed (rc=%d)\r\n", rc);
        /* wipe IV on failure to avoid reuse */
        secure_zero(IV, sizeof(IV));
        return rc; /* propagate AES_CTR error */
    }

    /* 3) compute HMAC over IV || ciphertext */
    rc = compute_hmac_over_iv_cipher(mac, sizeof(mac), &mac_len);
    if (rc != HMAC_IVC_SUCCESS) {
        log_printf("[ERROR] generate_nonce_iv_encrypt_hmac_sender: compute_hmac_over_iv_cipher failed (rc=%d)\r\n", rc);
        /* wipe sensitive artifacts */
        secure_zero(IV, sizeof(IV));
        return rc; /* propagate HMAC error */
    }

    return 0; /* success */
}



/* Constant-time memory comparison: returns 0 when equal, non-zero otherwise */
static int ct_memcmp(const void *a_, const void *b_, size_t n)
{
    /* Accumulate differences across all bytes before returning. */
    const uint8_t *a = (const uint8_t *)a_;
    const uint8_t *b = (const uint8_t *)b_;
    uint8_t diff = 0;
    size_t i;
    for (i = 0; i < n; ++i) {
        diff |= a[i] ^ b[i];
    }
    return (diff != 0);
}


/* Computes or verifies a keyed authentication tag. */
int verify_mac_decrypt_recombine_receiver(void)
{
    int rc;

    /* 1) Compute local HMAC over IV || ciphertext using receiver MAC key */
    rc = compute_hmac_over_iv_cipher_receiver(mac_check, sizeof(mac_check), &mac_check_len);
    if (rc != HMAC_IVC_SUCCESS) {
        log_printf("[ERROR] verify_mac_decrypt_recombine_receiver: compute_hmac_over_iv_cipher_receiver failed (rc=%d)\r\n", rc);
        secure_zero(mac_check, sizeof(mac_check));
        return -2;
    }

    /* 2) Compare lengths and bytes in constant time */
    if (mac_check_len != mac_len ) {
        log_printf("[ERROR] verify_mac_decrypt_recombine_receiver: mac length mismatch (%zu != %zu)\r\n",
                   mac_check_len, received_mac_len);
        secure_zero(mac_check, sizeof(mac_check));
        return -3;
    }

    if (ct_memcmp(mac_check, mac, mac_check_len) != 0) {
        log_printf("[ERROR] verify_mac_decrypt_recombine_receiver: MAC verification failed (mismatch)\r\n");
        secure_zero(mac_check, sizeof(mac_check));
        return -3;
    }

    /* MAC OK: wipe computed MAC copy before proceeding if desired (but keep until decrypt done) */
    /* secure_zero(mac_check, sizeof(mac_check));  // optional here */

    /* 3) Decrypt AES-CTR (assumes cipher_text_bytes and IV already populated and computed_size set) */
    rc = decrypt_aes_ctr(&dec_len);
    if (rc != AES_CTR_DEC_SUCCESS) {
        log_printf("[ERROR] verify_mac_decrypt_recombine_receiver: decrypt_aes_ctr failed (rc=%d)\r\n", rc);
        secure_zero(mac_check, sizeof(mac_check));
        secure_zero(decrypted_bytes, sizeof(decrypted_bytes));
        return -4;
    }

    /* 4) Sanity: decrypted length must equal transmitted ciphertext length (computed_size global) */
    if (dec_len != computed_size) {
        log_printf("[ERROR] verify_mac_decrypt_recombine_receiver: decrypted length mismatch (dec_len=%zu computed_size=%zu)\r\n",
                   dec_len, computed_size);
        secure_zero(mac_check, sizeof(mac_check));
        secure_zero(decrypted_bytes, sizeof(decrypted_bytes));
        return -5;
    }

    /* 5) Recombine ADC values from decrypted bytes and compare against expected adc_values[] */
    recombine_adc_values();

    for (int i = 0; i < ADC_SAMPLE_COUNT; ++i) {
        if (decrypted_adc_values[i] != adc_values[i]) {
            log_printf("[ERROR] verify_mac_decrypt_recombine_receiver: decrypted ADC mismatch at idx %d (got=0x%08X exp=0x%08X)\r\n",
                       i, (unsigned)decrypted_adc_values[i], (unsigned)adc_values[i]);
            secure_zero(mac_check, sizeof(mac_check));
            secure_zero(decrypted_bytes, sizeof(decrypted_bytes));
            return -6;
        }
    }

    /* Success: wipe ephemeral computed MAC and any other ephemeral buffers */
    secure_zero(mac_check, sizeof(mac_check));

    /* Caller may want to wipe IV and mac received after verification/transmission handling */
    return 0;
}

/* Securely clear the decrypted_adc_values array and verify clearing.
 * Uses secure_zero() then performs a volatile read verification.
 */
int secure_clear_decrypted_adc_values(void)
{
    /* defensive size calculation */
    size_t count = (size_t)ADC_SAMPLE_COUNT;
    if (count == 0) {
        return -1; /* invalid size */
    }

    size_t total_bytes = count * sizeof(decrypted_adc_values[0]);
    if (total_bytes / sizeof(decrypted_adc_values[0]) != count) {
        return -1; /* overflow / invalid */
    }

    /* perform secure wipe */
    secure_zero(decrypted_adc_values, total_bytes);

    /* compiler/memory barrier to ensure wipe is completed before verification */
    __asm__ volatile ("" ::: "memory");

    /* verify using volatile access to avoid compiler optimizing the check away */
    volatile uint8_t *p = (volatile uint8_t *)decrypted_adc_values;
    for (size_t i = 0; i < total_bytes; ++i) {
        if (p[i] != 0U) {
            return -3; /* verification failed */
        }
    }

    return 0; /* success */
}


