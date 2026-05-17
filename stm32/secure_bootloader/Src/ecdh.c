/*
 * ecdh_refactored.c
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
/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>

#include "ecdh.h" /* your project header with macros & prototypes */
#include "digital_signature.h"
/*crypto_ctx_t ctx;*/

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
decomp_adc_ret_t decompose_adc_values(crypto_ctx_t *ctx)
{
    /* Extract the bytes in the intended order for encryption. */
    if (ctx == NULL) {
        return DECOMP_ADC_NULL_CTX;
    }

    if (ctx->buffers.adc_values == NULL ||
        ctx->buffers.plain_text_bytes == NULL) {
        return DECOMP_ADC_NULL_BUFFER;
    }

    if (ctx->cfg.bytes_per_sample < 4) {
        return DECOMP_ADC_INVALID_CFG;
    }

    for (size_t i = 0; i < ctx->cfg.adc_sample_count; i++) {
        uint32_t v = ctx->buffers.adc_values[i];
        size_t off = i * ctx->cfg.bytes_per_sample;

        ctx->buffers.plain_text_bytes[off + 0] = (uint8_t)(v >> 24);
        ctx->buffers.plain_text_bytes[off + 1] = (uint8_t)(v >> 16);
        ctx->buffers.plain_text_bytes[off + 2] = (uint8_t)(v >> 8);
        ctx->buffers.plain_text_bytes[off + 3] = (uint8_t)(v);
    }

    return DECOMP_ADC_OK;
}

/* Clears sensitive or temporary data from memory. */
static int is_all_zero_buf(const crypto_ctx_t *ctx,
                           const uint8_t *buf,
                           size_t len)
{
    (void)ctx; /* kept for symmetry and future policy hooks */
    uint8_t acc = 0;
    for (size_t i = 0; i < len; i++) {
        acc |= buf[i];
    }
    return (acc == 0);
}


/* Validates the supplied data or peripheral state. */
pubkey_status_t validate_public_key_sender_format_ctx(const crypto_ctx_t *ctx,
                                                      const uint8_t *pub,
                                                      size_t pub_len)
{
    (void)ctx; /* kept for symmetry, auditing, and future policy hooks */

    if (pub == NULL) {
        return PUBKEY_ERR_NULL;
    }
    /* Accept raw X||Y (64) */
    if (pub_len == AES_512_KEY_SIZE) {
        if (is_all_zero_buf(ctx, pub, AES_512_KEY_SIZE)) {
            return PUBKEY_ERR_ALLZERO;
        }
        return PUBKEY_OK;
    }
    /* Accept uncompressed 0x04||X||Y (65) */
    if (pub_len == (AES_512_KEY_SIZE + 1)) {
        if (pub[0] != 0x04) {
            return PUBKEY_ERR_FORMAT;
        }
        if (is_all_zero_buf(ctx, &pub[1], AES_512_KEY_SIZE)) {
            return PUBKEY_ERR_ALLZERO;
        }
        return PUBKEY_OK;
    }
    /* Unsupported / invalid length */
    return PUBKEY_ERR_FORMAT;
}

/**
 * Generate a DRBG-based IV.
 * Returns 0 on success, negative error code on failure.
 * On failure the IV buffer is zeroed.
 */
gen_iv_status_t generate_iv(crypto_ctx_t *ctx)
{
    cmox_drbg_retval_t retval;
    //cmox_drbg_handle_t *drbg_ctx = NULL;

    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }

    /* Clear IV upfront to avoid reuse on failure */
    memset(ctx->IV, 0, sizeof(ctx->IV));

    ctx->drbg_ctx = cmox_ctr_drbg_construct(&ctx->drbg,
                                      CMOX_CTR_DRBG_AES256);
    if (ctx->drbg_ctx == NULL) {
        return GEN_IV_ERR_CONSTRUCT;
    }

    retval = cmox_drbg_init(ctx->drbg_ctx,
                            entropy_hashed, sizeof(entropy_hashed),
                            device_uid_buffer_hashed, sizeof(device_uid_buffer_hashed),
                            nonce_hashed, sizeof(nonce_hashed));
    if (retval != CMOX_DRBG_SUCCESS) {
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_IV_ERR_INIT;
    }

    retval = cmox_drbg_generate(ctx->drbg_ctx,
                                NULL, 0,
                                ctx->IV,
                                AES_256_IV_SIZE);
    if (retval != CMOX_DRBG_SUCCESS) {
        memset(ctx->IV, 0, sizeof(ctx->IV));
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_IV_ERR_GENERATE;
    }

    retval = cmox_drbg_cleanup(ctx->drbg_ctx);
    if (retval != CMOX_DRBG_SUCCESS) {
        memset(ctx->IV, 0, sizeof(ctx->IV));
        return GEN_IV_ERR_CLEANUP;
    }

    return GEN_IV_SUCCESS;
}
/**
 * Generate two DRBG-based private keys.
 * Returns 0 on success, negative error code on failure (see defines above).
 * On failure the private_key_* buffers are zeroed.
 */
gen_priv_status_t generate_private_key(crypto_ctx_t *ctx)
{
    cmox_drbg_retval_t retval;

    if (ctx == NULL) {
        return GEN_PRIV_ERR_BAD_PARAM;
    }

    /* zero buffers at start to avoid leaking previous values on failure */
    memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
    memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));

    if (cmox_ctr_drbg_construct(&ctx->drbg,
                               CMOX_CTR_DRBG_AES256) == NULL) {
        return GEN_PRIV_ERR_CONSTRUCT;
    }

    retval = cmox_drbg_init(ctx->drbg_ctx,
                            entropy_hashed, sizeof(entropy_hashed),
                            device_uid_buffer_hashed, sizeof(device_uid_buffer_hashed),
                            nonce_hashed, sizeof(nonce_hashed));
    if (retval != CMOX_DRBG_SUCCESS) {
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_PRIV_ERR_INIT;
    }

    retval = cmox_drbg_generate(ctx->drbg_ctx,
                                NULL, 0,
                                ctx->keys.private_key_sender,
                                AES_256_KEY_SIZE);
    if (retval != CMOX_DRBG_SUCCESS) {
        memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
        memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_PRIV_ERR_GEN_SENDER;
    }

    retval = cmox_drbg_generate(ctx->drbg_ctx,
                                NULL, 0,
                                ctx->keys.private_key_receiver,
                                AES_256_KEY_SIZE);
    if (retval != CMOX_DRBG_SUCCESS) {
        memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
        memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_PRIV_ERR_GEN_RECEIVER;
    }

    retval = cmox_drbg_cleanup(ctx->drbg_ctx);
    if (retval != CMOX_DRBG_SUCCESS) {
        memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
        memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));
        return GEN_PRIV_ERR_CLEANUP;
    }

    return GEN_PRIV_SUCCESS;
}

/* small helper to fill randomness via DRBG, returns 0 on success or GEN_KEYS_ERR_DRBG_GENERATE */
/* small helper to fill randomness via DRBG, returns 0 on success or GEN_KEYS_ERR_DRBG_GENERATE */
static int drbg_fill_bytes(sig_keys_ctx_t *ctx,
                               uint8_t *out,
                               size_t outlen)
{
    /* Check the generator status before using the output. */
    if (ctx == NULL || out == NULL || outlen == 0) {
        return -100;
    }

    cmox_drbg_retval_t r =
        cmox_drbg_generate(ctx->drbg_ctx,
                           NULL,
                           0,
                           out,
                           outlen);

    return (r == CMOX_DRBG_SUCCESS) ? 0 : GEN_KEYS_ERR_DRBG_GENERATE;
}


/**
 * Generate two ECDSA (secp256r1) keypairs (priv + pub) for sender and receiver.
 * Returns 0 on success, negative error code on failure. On failure both key buffers are zeroed.
 * After success, pub_len_sender/pub_len_receiver contain the actual public-key lengths.
 */
gen_keys_status_t generate_keypairs_cmox(crypto_ctx_t *ctx)
{
    cmox_ecc_retval_t  ecc_ret;
    cmox_drbg_retval_t drbg_ret;
    cmox_drbg_handle_t *drbg_ctx = NULL;
    uint8_t random_buf[128];
    size_t  random_len = AES_256_KEY_SIZE;

    if (ctx == NULL) {
        return GEN_KEYS_ERR_BAD_PARAM;
    }

    /* zero outputs and lengths upfront */
    memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
    memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));
    memset(ctx->keys.public_key_sender,    0, sizeof(ctx->keys.public_key_sender));
    memset(ctx->keys.public_key_receiver,  0, sizeof(ctx->keys.public_key_receiver));
    /* ---------- DRBG / ECC ---------- */
    ctx->keys.priv_len_sender   = 0;
    ctx->keys.pub_len_sender    = 0;
    ctx->keys.priv_len_receiver = 0;
    ctx->keys.pub_len_receiver  = 0;

    /* construct + init DRBG */
    ctx->drbg_ctx = cmox_ctr_drbg_construct(&ctx->drbg,CMOX_CTR_DRBG_AES256);
    if (ctx->drbg_ctx == NULL) {
        return GEN_KEYS_ERR_DRBG_CONSTRUCT;
    }

    drbg_ret = cmox_drbg_init(ctx->drbg_ctx,
    						  entropy_hashed, sizeof(entropy_hashed),
							  device_uid_buffer_hashed, sizeof(device_uid_buffer_hashed),
							  nonce_hashed, sizeof(nonce_hashed));
    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_KEYS_ERR_DRBG_INIT;
    }

    /* ---------- GENERATE SENDER KEYPAIR ---------- */
    for (;;) {
        int drbg_rc = drbg_fill_bytes(ctx, random_buf, random_len);
        if (drbg_rc != 0) {
            (void)cmox_drbg_cleanup(ctx->drbg_ctx);
            return drbg_rc;
        }

        cmox_ecc_construct(&ctx->ecc,
                           CMOX_ECC256_MATH_FUNCS,
                           ctx->working_buffer,
                           sizeof(ctx->working_buffer));

        size_t local_priv_len = sizeof(ctx->keys.private_key_sender);
        size_t local_pub_len  = sizeof(ctx->keys.public_key_sender);

        ecc_ret = cmox_ecdsa_keyGen(&ctx->ecc,
                                    CMOX_ECC_SECP256R1_LOWMEM,
                                    random_buf, random_len,
                                    ctx->keys.private_key_sender, &local_priv_len,
                                    ctx->keys.public_key_sender,  &local_pub_len);

        cmox_ecc_cleanup(&ctx->ecc);

        if (ecc_ret == CMOX_ECC_SUCCESS) {
            ctx->keys.priv_len_sender = local_priv_len;
            ctx->keys.pub_len_sender  = local_pub_len;
            break;
        }

        if (ecc_ret == CMOX_ECC_ERR_WRONG_RANDOM) {
            size_t needed = local_priv_len;
            if (needed > sizeof(random_buf)) {
                (void)cmox_drbg_cleanup(ctx->drbg_ctx);
                memset(ctx->keys.private_key_sender, 0, sizeof(ctx->keys.private_key_sender));
                memset(ctx->keys.public_key_sender,  0, sizeof(ctx->keys.public_key_sender));
                return GEN_KEYS_ERR_RANDOM_TOO_LARGE;
            }
            random_len = needed;
            continue;
        }

        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        memset(ctx->keys.private_key_sender, 0, sizeof(ctx->keys.private_key_sender));
        memset(ctx->keys.public_key_sender,  0, sizeof(ctx->keys.public_key_sender));
        return GEN_KEYS_ERR_KEYGEN_SENDER;
    }

    /* reset random preference for receiver */
    random_len = AES_256_KEY_SIZE;

    /* ---------- GENERATE RECEIVER KEYPAIR ---------- */
    for (;;) {
        int drbg_rc = drbg_fill_bytes(ctx, random_buf, random_len);
        if (drbg_rc != 0) {
            memset(ctx->keys.private_key_sender, 0, sizeof(ctx->keys.private_key_sender));
            memset(ctx->keys.public_key_sender,  0, sizeof(ctx->keys.public_key_sender));
            (void)cmox_drbg_cleanup(ctx->drbg_ctx);
            return drbg_rc;
        }

        cmox_ecc_construct(&ctx->ecc,
                           CMOX_ECC256_MATH_FUNCS,
                           ctx->working_buffer,
                           sizeof(ctx->working_buffer));

        size_t local_priv_len = sizeof(ctx->keys.private_key_receiver);
        size_t local_pub_len  = sizeof(ctx->keys.public_key_receiver);

        ecc_ret = cmox_ecdsa_keyGen(&ctx->ecc,
                                    CMOX_ECC_SECP256R1_LOWMEM,
                                    random_buf, random_len,
                                    ctx->keys.private_key_receiver, &local_priv_len,
                                    ctx->keys.public_key_receiver,  &local_pub_len);

        cmox_ecc_cleanup(&ctx->ecc);

        if (ecc_ret == CMOX_ECC_SUCCESS) {
            ctx->keys.priv_len_receiver = local_priv_len;
            ctx->keys.pub_len_receiver  = local_pub_len;
            break;
        }

        if (ecc_ret == CMOX_ECC_ERR_WRONG_RANDOM) {
            size_t needed = local_priv_len;
            if (needed > sizeof(random_buf)) {
                memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
                memset(ctx->keys.public_key_sender,    0, sizeof(ctx->keys.public_key_sender));
                memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));
                memset(ctx->keys.public_key_receiver,  0, sizeof(ctx->keys.public_key_receiver));
                (void)cmox_drbg_cleanup(ctx->drbg_ctx);
                return GEN_KEYS_ERR_RANDOM_TOO_LARGE;
            }
            random_len = needed;
            continue;
        }

        memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
        memset(ctx->keys.public_key_sender,    0, sizeof(ctx->keys.public_key_sender));
        memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));
        memset(ctx->keys.public_key_receiver,  0, sizeof(ctx->keys.public_key_receiver));
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_KEYS_ERR_KEYGEN_RECEIVER;
    }

    /* cleanup DRBG */
    drbg_ret = cmox_drbg_cleanup(ctx->drbg_ctx);
    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
        memset(ctx->keys.public_key_sender,    0, sizeof(ctx->keys.public_key_sender));
        memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));
        memset(ctx->keys.public_key_receiver,  0, sizeof(ctx->keys.public_key_receiver));
        return GEN_KEYS_ERR_DRBG_CLEANUP;
    }

    return GEN_KEYS_SUCCESS;
}

/*
 * Generate a single Ed25519 keypair and store it into sig_keys_ctx_t->sig.
 * Behavior mirrors your existing generate_keypairs_cmox(): uses CTR-DRBG, loops if
 * random length was insufficient, zeros outputs on error.
 */
gen_keys_status_t generate_ed25519_signing_keypair(sig_keys_ctx_t *ctx)
{
    cmox_ecc_retval_t  ecc_ret;
    cmox_drbg_retval_t drbg_ret;
    uint8_t random_buf[64];
    size_t  random_len = 32; /* initial preference */

    if (ctx == NULL) {
        return GEN_KEYS_ERR_BAD_PARAM;
    }

    /* zero outputs and lengths upfront */
    memset(ctx->sig.private_key, 0, sizeof(ctx->sig.private_key));
    memset(ctx->sig.public_key,  0, sizeof(ctx->sig.public_key));
    ctx->sig.priv_len = 0;
    ctx->sig.pub_len  = 0;
    ctx->sig.provisioned = false;
    ctx->sig.storage_flags = SIG_KEY_STORAGE_NONE;

    /* construct + init DRBG in ctx (reuse same pattern as generate_keypairs_cmox) */
    ctx->drbg_ctx = cmox_ctr_drbg_construct(&ctx->drbg, CMOX_CTR_DRBG_AES256);
    if (ctx->drbg_ctx == NULL) {
        return GEN_KEYS_ERR_DRBG_CONSTRUCT;
    }

    drbg_ret = cmox_drbg_init(ctx->drbg_ctx,
                              entropy_hashed, sizeof(entropy_hashed),
                              device_uid_buffer_hashed, sizeof(device_uid_buffer_hashed),
                              nonce_hashed, sizeof(nonce_hashed));
    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_KEYS_ERR_DRBG_INIT;
    }

    /* generate ED25519 keypair into ctx->sig */
    for (;;) {
        int drbg_rc = drbg_fill_bytes(ctx, random_buf, random_len);
        if (drbg_rc != 0) {
            (void)cmox_drbg_cleanup(ctx->drbg_ctx);
            return drbg_rc;
        }

        /* construct ECC context for Ed25519 using ctx->ecc and working buffer */
        cmox_ecc_construct(&ctx->ecc,
        				   CMOX_ECC256_MATH_FUNCS,
                           ctx->working_buffer,
                           sizeof(ctx->working_buffer));

        size_t local_priv_len = sizeof(ctx->sig.private_key); /* expected 64 */
        size_t local_pub_len  = sizeof(ctx->sig.public_key);  /* expected 32 */

        ecc_ret = cmox_eddsa_keyGen(&ctx->ecc,
        							CMOX_ECC_ED25519_HIGHMEM,
                                    random_buf, random_len,
                                    ctx->sig.private_key, &local_priv_len,
                                    ctx->sig.public_key,  &local_pub_len);

        /*cmox_ecc_cleanup(&ctx->ecc);*/

        if (ecc_ret == CMOX_ECC_SUCCESS) {
            ctx->sig.priv_len = local_priv_len;
            ctx->sig.pub_len  = local_pub_len;
            break;
        }

        if (ecc_ret == CMOX_ECC_ERR_WRONG_RANDOM) {
            size_t needed = local_priv_len;
            if (needed > sizeof(random_buf)) {
                (void)cmox_drbg_cleanup(ctx->drbg_ctx);
                memset(ctx->sig.private_key, 0, sizeof(ctx->sig.private_key));
                memset(ctx->sig.public_key,  0, sizeof(ctx->sig.public_key));
                return GEN_KEYS_ERR_RANDOM_TOO_LARGE;
            }
            random_len = needed;
            continue;
        }

        /* other errors: cleanup and zero outputs */
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        memset(ctx->sig.private_key, 0, sizeof(ctx->sig.private_key));
        memset(ctx->sig.public_key,  0, sizeof(ctx->sig.public_key));
        return GEN_KEYS_ERR_KEYGEN_SENDER;
    }

    /* cleanup DRBG stored in ctx */
    drbg_ret = cmox_drbg_cleanup(ctx->drbg_ctx);
    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        memset(ctx->sig.private_key, 0, sizeof(ctx->sig.private_key));
        memset(ctx->sig.public_key,  0, sizeof(ctx->sig.public_key));
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
ecdh_status_t compute_ecdh_shared_as_sender(crypto_ctx_t *ctx, uint8_t *out_secret, size_t *out_len)
{
    cmox_ecc_retval_t retval;
    size_t pub_raw_len;

    if (ctx == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }

    /* Determine receiver public-key raw length (X||Y). Fall back to AES_512_KEY_SIZE if unknown. */
    pub_raw_len = (ctx->keys.pub_len_receiver != 0) ? ctx->keys.pub_len_receiver : AES_512_KEY_SIZE;

    /* Try 1: raw X||Y public key */
    ctx->tmp.computed_size = sizeof(ctx->tmp.computed_secret);

    cmox_ecc_construct(&ctx->ecc,
                       CMOX_ECC256_MATH_FUNCS,
                       ctx->working_buffer, sizeof(ctx->working_buffer));

    retval = cmox_ecdh(&ctx->ecc,
                       CMOX_ECC_CURVE_SECP256R1,
                       ctx->keys.private_key_sender, AES_256_KEY_SIZE,
                       ctx->keys.public_key_receiver, pub_raw_len,
                       ctx->tmp.computed_secret, &ctx->tmp.computed_size);

    if (retval == CMOX_ECC_SUCCESS) {
        cmox_ecc_cleanup(&ctx->ecc);

        if (out_secret && out_len) {
            size_t to_copy = (ctx->tmp.computed_size <= *out_len) ? ctx->tmp.computed_size : *out_len;
            memcpy(out_secret, ctx->tmp.computed_secret, to_copy);
            *out_len = to_copy;
        } else if (out_len) {
            *out_len = ctx->tmp.computed_size;
        }
        return ECDH_SUCCESS;
    }

    /* Try 2: uncompressed point 0x04 || X || Y */
    {
        uint8_t pub_prefixed[1 + AES_512_KEY_SIZE];
        size_t prefixed_len;

        /* ensure we don't read past the available public key array */
        if (pub_raw_len > AES_512_KEY_SIZE) {
            /* invalid public key size */
            cmox_ecc_cleanup(&ctx->ecc);
            memset(ctx->tmp.computed_secret, 0, sizeof(ctx->tmp.computed_secret));
            ctx->tmp.computed_size = 0;
            return ECDH_ERR_ECDH_FAILED;
        }

        pub_prefixed[0] = 0x04;
        memcpy(&pub_prefixed[1], ctx->keys.public_key_receiver, pub_raw_len);
        prefixed_len = 1 + pub_raw_len;

        ctx->tmp.computed_size = sizeof(ctx->tmp.computed_secret);

        retval = cmox_ecdh(&ctx->ecc,
                           CMOX_ECC_CURVE_SECP256R1,
                           ctx->keys.private_key_sender, AES_256_KEY_SIZE,
                           pub_prefixed, prefixed_len,
                           ctx->tmp.computed_secret, &ctx->tmp.computed_size);

        cmox_ecc_cleanup(&ctx->ecc);

        if (retval != CMOX_ECC_SUCCESS) {
            /* clear secret on failure */
            memset(ctx->tmp.computed_secret, 0, sizeof(ctx->tmp.computed_secret));
            ctx->tmp.computed_size = 0;
            return ECDH_ERR_ECDH_FAILED;
        }

        /* Success with prefixed form */
        if (out_secret && out_len) {
            size_t to_copy = (ctx->tmp.computed_size <= *out_len) ? ctx->tmp.computed_size : *out_len;
            memcpy(out_secret, ctx->tmp.computed_secret, to_copy);
            *out_len = to_copy;
        } else if (out_len) {
            *out_len = ctx->tmp.computed_size;
        }

        return ECDH_SUCCESS;
    }
}
/* Compute ECDH shared secret as receiver (mirror of sender).
 * Uses global computed_secret[] and my_len.
 */
ecdh_status_t compute_ecdh_shared_as_receiver(crypto_ctx_t *ctx,
                                              uint8_t *out_secret,
                                              size_t *out_len)
{
    cmox_ecc_retval_t retval;
    size_t pub_raw_len;

    if (ctx == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }

    /* Determine sender public-key raw length (X||Y) */
    pub_raw_len = (ctx->keys.pub_len_sender != 0)
                    ? ctx->keys.pub_len_sender
                    : AES_512_KEY_SIZE;

    /* ---------- Try 1: raw X||Y ---------- */
    ctx->tmp.computed_size = sizeof(ctx->tmp.computed_secret_receiver);

    cmox_ecc_construct(&ctx->ecc,
                       CMOX_ECC256_MATH_FUNCS,
                       ctx->working_buffer,
                       sizeof(ctx->working_buffer));

    retval = cmox_ecdh(&ctx->ecc,
                       CMOX_ECC_CURVE_SECP256R1,
                       ctx->keys.private_key_receiver, AES_256_KEY_SIZE,
                       ctx->keys.public_key_sender, pub_raw_len,
                       ctx->tmp.computed_secret_receiver,
                       &ctx->tmp.computed_size);

    if (retval == CMOX_ECC_SUCCESS) {
        cmox_ecc_cleanup(&ctx->ecc);

        if (out_secret && out_len) {
            size_t to_copy = (ctx->tmp.computed_size <= *out_len)
                               ? ctx->tmp.computed_size
                               : *out_len;
            memcpy(out_secret, ctx->tmp.computed_secret_receiver, to_copy);
            *out_len = to_copy;
        } else if (out_len) {
            *out_len = ctx->tmp.computed_size;
        }

        return ECDH_SUCCESS;
    }

    /* ---------- Try 2: uncompressed point 0x04 || X || Y ---------- */
    {
        uint8_t pub_prefixed[1 + AES_512_KEY_SIZE];
        size_t  prefixed_len;

        if (pub_raw_len > AES_512_KEY_SIZE) {
            cmox_ecc_cleanup(&ctx->ecc);
            memset(ctx->tmp.computed_secret_receiver, 0,
                   sizeof(ctx->tmp.computed_secret_receiver));
            ctx->tmp.computed_size = 0;
            return ECDH_ERR_ECDH_FAILED;
        }

        pub_prefixed[0] = 0x04;
        memcpy(&pub_prefixed[1],
               ctx->keys.public_key_sender,
               pub_raw_len);
        prefixed_len = 1 + pub_raw_len;

        ctx->tmp.computed_size = sizeof(ctx->tmp.computed_secret_receiver);

        retval = cmox_ecdh(&ctx->ecc,
                           CMOX_ECC_CURVE_SECP256R1,
                           ctx->keys.private_key_receiver, AES_256_KEY_SIZE,
                           pub_prefixed, prefixed_len,
                           ctx->tmp.computed_secret_receiver,
                           &ctx->tmp.computed_size);

        cmox_ecc_cleanup(&ctx->ecc);

        if (retval != CMOX_ECC_SUCCESS) {
            memset(ctx->tmp.computed_secret_receiver, 0,
                   sizeof(ctx->tmp.computed_secret_receiver));
            ctx->tmp.computed_size = 0;
            return ECDH_ERR_ECDH_FAILED;
        }

        if (out_secret && out_len) {
            size_t to_copy = (ctx->tmp.computed_size <= *out_len)
                               ? ctx->tmp.computed_size
                               : *out_len;
            memcpy(out_secret, ctx->tmp.computed_secret_receiver, to_copy);
            *out_len = to_copy;
        } else if (out_len) {
            *out_len = ctx->tmp.computed_size;
        }

        return ECDH_SUCCESS;
    }
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
kdf_status_t derive_keys_from_shared_secret(crypto_ctx_t *ctx, bool receiver)
{
    cmox_hash_retval_t retval;
    uint8_t buf[CMOX_ECC_SECP256R1_SECRET_LEN + 1];
    size_t out_len;

    uint8_t *shared;
    uint8_t *aes_key;
    uint8_t *mac_key;

    if (ctx == NULL) {
        return KDF_ERR_BAD_INPUT;
    }

    if (ctx->tmp.computed_size == 0 ||
        ctx->tmp.computed_size > CMOX_ECC_SECP256R1_SECRET_LEN) {
        return KDF_ERR_BAD_INPUT;
    }

    /* select sender vs receiver material */
    if (receiver) {
        shared  = ctx->tmp.computed_secret_receiver;
        aes_key = ctx->derived.aes_key_receiver;
        mac_key = ctx->derived.mac_key_receiver;
    } else {
        shared  = ctx->tmp.computed_secret;
        aes_key = ctx->derived.aes_key;
        mac_key = ctx->derived.mac_key;
    }

    /* ---------- AES key: SHA256(shared || 0x01) ---------- */
    memcpy(buf, shared, ctx->tmp.computed_size);
    buf[ctx->tmp.computed_size] = 0x01;

    out_len = sizeof(ctx->derived.aes_key);
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf, ctx->tmp.computed_size + 1,
                               aes_key, out_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS || out_len != sizeof(ctx->derived.aes_key)) {
        memset(aes_key, 0, sizeof(ctx->derived.aes_key));
        return KDF_ERR_HASH_FAILED;
    }

    /* ---------- MAC key: SHA256(shared || 0x02) ---------- */
    buf[ctx->tmp.computed_size] = 0x02;

    out_len = sizeof(ctx->derived.mac_key);
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf, ctx->tmp.computed_size + 1,
                               mac_key, out_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS || out_len != sizeof(ctx->derived.mac_key)) {
        memset(aes_key, 0, sizeof(ctx->derived.aes_key));
        memset(mac_key, 0, sizeof(ctx->derived.mac_key));
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
kdf_status_t derive_keys_from_shared_secret_receiver(crypto_ctx_t *ctx)
{
    cmox_hash_retval_t retval;
    uint8_t buf[CMOX_ECC_SECP256R1_SECRET_LEN + 1];
    size_t out_len;

    if (ctx == NULL) {
        return KDF_ERR_BAD_INPUT;
    }

    if (ctx->tmp.computed_size == 0 ||
        ctx->tmp.computed_size > CMOX_ECC_SECP256R1_SECRET_LEN) {
        return KDF_ERR_BAD_INPUT;
    }

    /* ---------- AES key derivation: SHA256(shared || 0x01) ---------- */
    memcpy(buf,
           ctx->tmp.computed_secret_receiver,
           ctx->tmp.computed_size);
    buf[ctx->tmp.computed_size] = 0x01;

    out_len = sizeof(ctx->derived.aes_key_receiver);
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf,
                               ctx->tmp.computed_size + 1,
                               ctx->derived.aes_key_receiver,
                               out_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS ||
        out_len != sizeof(ctx->derived.aes_key_receiver)) {
        memset(ctx->derived.aes_key_receiver, 0,
               sizeof(ctx->derived.aes_key_receiver));
        return KDF_ERR_HASH_FAILED;
    }

    /* ---------- MAC key derivation: SHA256(shared || 0x02) ---------- */
    buf[ctx->tmp.computed_size] = 0x02;

    out_len = sizeof(ctx->derived.mac_key_receiver);
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf,
                               ctx->tmp.computed_size + 1,
                               ctx->derived.mac_key_receiver,
                               out_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS ||
        out_len != sizeof(ctx->derived.mac_key_receiver)) {
        memset(ctx->derived.aes_key_receiver, 0,
               sizeof(ctx->derived.aes_key_receiver));
        memset(ctx->derived.mac_key_receiver, 0,
               sizeof(ctx->derived.mac_key_receiver));
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
aes_ctr_enc_status_t encrypt_aes_ctr(crypto_ctx_t *ctx,
                                    size_t *out_computed_size)
{
    /* Validate the crypto return value and output length. */
    cmox_cipher_retval_t cipher_ret;
    size_t local_out_len = 0;

    if (out_computed_size) {
        *out_computed_size = 0;
    }

    if (ctx == NULL) {
        return AES_CTR_ENC_ERR_BAD_INPUT;
    }

    if (ctx->tmp.plain_len == 0 ||
        ctx->tmp.plain_len > sizeof(ctx->buffers.cipher_text_bytes)) {
        return AES_CTR_ENC_ERR_BAD_INPUT;
    }

    if (is_all_zero_bytes(ctx->derived.aes_key,
                          sizeof(ctx->derived.aes_key)) ||
        is_all_zero_bytes(ctx->IV,
                          sizeof(ctx->IV))) {
        return AES_CTR_ENC_ERR_KEY_OR_IV;
    }

    ctx->tmp.computed_size = 0;

    cipher_ret = cmox_cipher_encrypt(
                    CMOX_AESFAST_CTR_ENC_ALGO,
                    ctx->buffers.plain_text_bytes,
                    ctx->tmp.plain_len,
                    ctx->derived.aes_key,
                    sizeof(ctx->derived.aes_key),
                    ctx->IV,
                    sizeof(ctx->IV),
                    ctx->buffers.cipher_text_bytes,
                    &local_out_len
                 );

    if (cipher_ret != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.cipher_text_bytes,
                    sizeof(ctx->buffers.cipher_text_bytes));
        return AES_CTR_ENC_ERR_CIPHER_FAIL;
    }

    if (local_out_len != ctx->tmp.plain_len) {
        secure_zero(ctx->buffers.cipher_text_bytes,
                    sizeof(ctx->buffers.cipher_text_bytes));
        return AES_CTR_ENC_ERR_LENGTH_MISMATCH;
    }

    ctx->tmp.computed_size = local_out_len;

    if (out_computed_size) {
        *out_computed_size = ctx->tmp.computed_size;
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
aes_ctr_dec_status_t decrypt_aes_ctr(crypto_ctx_t *ctx,
                                    size_t *out_decrypted_size)
{
    cmox_cipher_retval_t cipher_ret;
    size_t dec_size_local = 0;

    /* clear output size upfront (defensive) */
    if (out_decrypted_size) {
        *out_decrypted_size = 0;
    }

    if (ctx == NULL) {
        return AES_CTR_DEC_ERR_BAD_INPUT;
    }

    /* basic sanity checks */
    if (ctx->tmp.computed_size == 0) {
        return AES_CTR_DEC_ERR_BAD_INPUT;
    }

    if (ctx->tmp.computed_size >
        sizeof(ctx->buffers.cipher_text_bytes)) {
        return AES_CTR_DEC_ERR_BAD_INPUT;
    }

    if (ctx->tmp.computed_size >
        sizeof(ctx->buffers.decrypted_bytes)) {
        return AES_CTR_DEC_ERR_LENGTH_MISMATCH;
    }

    cipher_ret = cmox_cipher_decrypt(
                    CMOX_AESFAST_CTR_DEC_ALGO,
                    ctx->buffers.cipher_text_bytes,
                    ctx->tmp.computed_size,
                    ctx->derived.aes_key_receiver,
                    sizeof(ctx->derived.aes_key_receiver),
                    ctx->IV,
                    sizeof(ctx->IV),
                    ctx->buffers.decrypted_bytes,
                    &dec_size_local
                 );

    if (cipher_ret != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        return AES_CTR_DEC_ERR_CIPHER_FAIL;
    }

    /* CTR is length-preserving */
    if (dec_size_local != ctx->tmp.computed_size) {
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        return AES_CTR_DEC_ERR_LENGTH_MISMATCH;
    }

    /* final defensive bound check */
    if (dec_size_local >
        sizeof(ctx->buffers.decrypted_bytes)) {
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        return AES_CTR_DEC_ERR_LENGTH_MISMATCH;
    }

    if (out_decrypted_size) {
        *out_decrypted_size = dec_size_local;
    }

    return AES_CTR_DEC_SUCCESS;
}

/* Compute HMAC over IV || ciphertext (sender-side).
 * Writes tag to mac_out and sets *out_mac_len.
 * Uses ctx->derived.mac_key and ctx->buffers.cipher_text_bytes.
 */
hmac_ivc_status_t compute_hmac_over_iv_cipher(crypto_ctx_t *ctx,
                                              uint8_t *mac_out,
                                              size_t mac_out_len,
                                              size_t *out_mac_len)
{
    cmox_mac_retval_t retval;
    size_t mac_input_len;

    if (ctx == NULL || mac_out == NULL || out_mac_len == NULL) {
        return HMAC_IVC_ERR_NULL_INPUT;
    }

    /* Defensive clear */
    *out_mac_len = 0;

    /* Ensure ciphertext exists and size is sane */
    if (ctx->tmp.computed_size == 0) {
        return HMAC_IVC_ERR_NO_CIPHERTEXT;
    }
    if (ctx->tmp.computed_size > sizeof(ctx->buffers.cipher_text_bytes)) {
        return HMAC_IVC_ERR_SIZE_INVALID;
    }

    /* Prevent size overflow: IV || ciphertext */
    if (ctx->tmp.computed_size > (SIZE_MAX - sizeof(ctx->IV))) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    mac_input_len = sizeof(ctx->IV) + ctx->tmp.computed_size;

    /* Use an existing preallocated buffer as temporary MAC input.
     * Ensure it is large enough: cipher_text_gcm_aead is ADC_SAMPLE_COUNT*BYTES_PER_SAMPLE + AES_GCM_TAG_SIZE
     * which is expected to be >= sizeof(IV) + ciphertext length in this codebase.
     */
    if (mac_input_len > sizeof(ctx->buffers.cipher_text_gcm_aead)) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    /* Build MAC input: IV || ciphertext */
    memcpy(ctx->buffers.cipher_text_gcm_aead, ctx->IV, sizeof(ctx->IV));
    memcpy(ctx->buffers.cipher_text_gcm_aead + sizeof(ctx->IV),
           ctx->buffers.cipher_text_bytes,
           ctx->tmp.computed_size);

    /* Compute HMAC-SHA256 using derived MAC key (sender-side) */
    retval = cmox_mac_compute(
                CMOX_HMAC_SHA256_ALGO,
                ctx->buffers.cipher_text_gcm_aead, mac_input_len,
                ctx->derived.mac_key, sizeof(ctx->derived.mac_key),
                NULL, 0,
                mac_out, mac_out_len, out_mac_len
             );

    /* Wipe temporary buffer (only touched region) */
    secure_zero(ctx->buffers.cipher_text_gcm_aead, mac_input_len);

    if (retval != CMOX_MAC_SUCCESS) {
        /* wipe output on failure */
        secure_zero(mac_out, mac_out_len);
        *out_mac_len = 0;
        return HMAC_IVC_ERR_MAC_FAILED;
    }

    return HMAC_IVC_SUCCESS;
}


/* Compute HMAC over IV || ciphertext (receiver-side).
 * Writes tag to mac_out and sets *out_mac_len.
 * Uses ctx->derived.mac_key_receiver and ctx->buffers.cipher_text_bytes.
 */
hmac_ivc_status_t compute_hmac_over_iv_cipher_receiver(crypto_ctx_t *ctx,
                                                       uint8_t *mac_out,
                                                       size_t mac_out_len,
                                                       size_t *out_mac_len)
{
    cmox_mac_retval_t retval;
    size_t mac_input_len;

    if (ctx == NULL || mac_out == NULL || out_mac_len == NULL) {
        return HMAC_IVC_ERR_NULL_INPUT;
    }

    /* Defensive clear */
    *out_mac_len = 0;

    /* Ensure ciphertext exists and size is sane */
    if (ctx->tmp.computed_size == 0) {
        return HMAC_IVC_ERR_NO_CIPHERTEXT;
    }
    if (ctx->tmp.computed_size > sizeof(ctx->buffers.cipher_text_bytes)) {
        return HMAC_IVC_ERR_SIZE_INVALID;
    }

    /* Prevent IV || ciphertext size overflow */
    if (ctx->tmp.computed_size > (SIZE_MAX - sizeof(ctx->IV))) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    mac_input_len = sizeof(ctx->IV) + ctx->tmp.computed_size;

    /* Ensure temporary buffer is large enough */
    if (mac_input_len > sizeof(ctx->buffers.cipher_text_gcm_aead_receiver)) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    /* Build MAC input: IV || ciphertext using preallocated receiver scratch buffer */
    memcpy(ctx->buffers.cipher_text_gcm_aead_receiver, ctx->IV, sizeof(ctx->IV));
    memcpy(ctx->buffers.cipher_text_gcm_aead_receiver + sizeof(ctx->IV),
           ctx->buffers.cipher_text_bytes,
           ctx->tmp.computed_size);

    /* Compute HMAC-SHA256 using receiver MAC key */
    retval = cmox_mac_compute(
                CMOX_HMAC_SHA256_ALGO,
                ctx->buffers.cipher_text_gcm_aead_receiver, mac_input_len,
                ctx->derived.mac_key_receiver, sizeof(ctx->derived.mac_key_receiver),
                NULL, 0,
                mac_out, mac_out_len, out_mac_len
             );

    /* Wipe temporary buffer (only touched region) */
    secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, mac_input_len);

    if (retval != CMOX_MAC_SUCCESS) {
        /* wipe output on failure */
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
gen_iv_status_t generate_nonce_and_iv_per_transfer(crypto_ctx_t *ctx)
{
    int rc;

    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }

    /* 1) generate nonce (must be ctx-aware) */
    rc = nonce_generation();
    if (rc != NONCE_GEN_SUCCESS) {
        return rc;
    }

    /* 2) generate DRBG IV (16 bytes) -> ctx->IV */
    rc = generate_iv(ctx);
    if (rc != GEN_IV_SUCCESS) {
        return rc;
    }

    /* 3) derive 12-byte GCM IV -> ctx->IV_GCM */
    rc = derive_gcm_iv_from_drbg(ctx);
    if (rc != GEN_IV_SUCCESS) {
        return rc;
    }

    return GEN_IV_SUCCESS;
}


/* Processes authenticated AES-GCM encryption or decryption. */
gcm_master_status_t derive_gcm_iv_from_drbg(crypto_ctx_t *ctx)
{
    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }

    /* IV must already be generated */
    /* Take first 12 bytes (acceptable: DRBG output is uniform) */
    memcpy(ctx->IV_GCM, ctx->IV, AES_GCM_AEAD_IV_SIZE);

    return GCM_MASTER_SUCCESS;
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
sender_init_status_t sender_init_ecdh_crypto(crypto_ctx_t *ctx)
{
    int ret;

    if (ctx == NULL) {
        return SENDER_INIT_ERR_BAD_PARAM;
    }

    /* ---------- 1) generate ECC keypairs ---------- */
    ret = generate_keypairs_cmox(ctx);
    if (ret != GEN_KEYS_SUCCESS) {
        log_printf("[ERROR] sender_init_ecdh_crypto: generate_keypairs_cmox failed (rc=%d)\r\n", ret);

        secure_zero(ctx->keys.private_key_sender,
                    sizeof(ctx->keys.private_key_sender));
        secure_zero(ctx->keys.public_key_sender,
                    sizeof(ctx->keys.public_key_sender));

        ctx->keys.priv_len_sender = 0;
        ctx->keys.pub_len_sender  = 0;

        return SENDER_INIT_ERR_GENKEY;
    }

    /* ---------- 2) compute ECDH shared secret (sender side) ---------- */
    ret = compute_ecdh_shared_as_sender(ctx,ctx->tmp.computed_secret,&ctx->tmp.computed_size);
    if (ret != ECDH_SUCCESS) {
        log_printf("[ERROR] sender_init_ecdh_crypto: compute_ec-dh_shared_as_sender failed (rc=%d)\r\n", ret);

        secure_zero(ctx->keys.private_key_sender,
                    sizeof(ctx->keys.private_key_sender));
        secure_zero(ctx->keys.public_key_sender,
                    sizeof(ctx->keys.public_key_sender));
        secure_zero(ctx->tmp.computed_secret,
                    sizeof(ctx->tmp.computed_secret));

        ctx->tmp.computed_size = 0;

        return SENDER_INIT_ERR_ECDH;
    }

    /* ---------- 3) derive AES + MAC keys from shared secret ---------- */
    ret = derive_keys_from_shared_secret(ctx,false);
    if (ret != KDF_SUCCESS) {
        log_printf("[ERROR] sender_init_ecdh_crypto: derive_keys_from_shared_secret failed (rc=%d)\r\n", ret);

        secure_zero(ctx->keys.private_key_sender,
                    sizeof(ctx->keys.private_key_sender));
        secure_zero(ctx->keys.public_key_sender,
                    sizeof(ctx->keys.public_key_sender));
        secure_zero(ctx->tmp.computed_secret,
                    sizeof(ctx->tmp.computed_secret));
        secure_zero(ctx->derived.aes_key,
                    sizeof(ctx->derived.aes_key));
        secure_zero(ctx->derived.mac_key,
                    sizeof(ctx->derived.mac_key));

        ctx->tmp.computed_size = 0;

        return SENDER_INIT_ERR_KDF;
    }

    log_printf("[INFO] sender_init_ecdh_crypto: success (shared_len=%u)\r\n",
               (unsigned)ctx->tmp.computed_size);

    return SENDER_INIT_SUCCESS;
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
sender_init_status_t generate_private_key_digital_signature(sig_keys_ctx_t *ctx)
{
    int ret;

    if (ctx == NULL) {
        return SENDER_INIT_ERR_BAD_PARAM;
    }

    /* ---------- 1) generate Ed25519 signing keypair into ctx->sig ---------- */
    ret = generate_ed25519_signing_keypair(ctx);
    if (ret != GEN_KEYS_SUCCESS) {
        log_printf("[ERROR] generate_ed25519_signing_keypair failed (rc=%d)\r\n", ret);

        /* securely wipe outputs */
        secure_zero(ctx->sig.private_key, sizeof(ctx->sig.private_key));
        secure_zero(ctx->sig.public_key,  sizeof(ctx->sig.public_key));

        /* clear lengths and provisioning flags */
        ctx->sig.priv_len     = 0;
        ctx->sig.pub_len      = 0;
        ctx->sig.provisioned  = false;
        ctx->sig.storage_flags = SIG_KEY_STORAGE_NONE;

        return SENDER_INIT_ERR_GENKEY;
    }

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
receiver_init_status_t receiver_init_ecdh_crypto(crypto_ctx_t *ctx)
{
    int ret;

    if (ctx == NULL) {
        return RECEIVER_INIT_ERR_BAD_PARAM;
    }

    /* ---------- 1) Validate sender public key format ---------- */
    ret = validate_public_key_sender_format_ctx(ctx,ctx->keys.public_key_sender, ctx->keys.pub_len_sender);
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
    ret = compute_ecdh_shared_as_receiver(ctx, NULL, NULL);
    if (ret != ECDH_SUCCESS) {
        log_printf("[ERROR] receiver_init_ecdh_crypto: compute_ecdh_shared_as_receiver failed (rc=%d)\r\n", ret);

        secure_zero(ctx->tmp.computed_secret_receiver, sizeof(ctx->tmp.computed_secret_receiver));
        ctx->tmp.computed_size = 0;

        return RECEIVER_INIT_ERR_ECDH;
    }

    /* ---------- 3) derive AES + MAC keys from shared secret (receiver) ---------- */
    ret = derive_keys_from_shared_secret_receiver(ctx);
    if (ret != KDF_SUCCESS) {
        log_printf("[ERROR] receiver_init_ecdh_crypto: derive_keys_from_shared_secret_receiver failed (rc=%d)\r\n", ret);

        secure_zero(ctx->tmp.computed_secret_receiver, sizeof(ctx->tmp.computed_secret_receiver));
        secure_zero(ctx->derived.aes_key_receiver, sizeof(ctx->derived.aes_key_receiver));
        secure_zero(ctx->derived.mac_key_receiver, sizeof(ctx->derived.mac_key_receiver));
        ctx->tmp.computed_size = 0;

        return RECEIVER_INIT_ERR_KDF;
    }

    log_printf("[INFO] receiver_init_ecdh_crypto: success (receiver_shared_len=%u)\r\n",
               (unsigned)ctx->tmp.computed_size);

    return RECEIVER_INIT_SUCCESS;
}

/* Implement wrapper */
hmac_sender_status_t generate_nonce_iv_encrypt_hmac_sender(crypto_ctx_t *ctx)
{
    gen_iv_status_t     iv_rc;
    aes_ctr_enc_status_t enc_rc;
    hmac_ivc_status_t   hmac_rc;

    if (ctx == NULL) {
        return HMAC_SENDER_ERR_BAD_PARAM;
    }

    /* 1) generate nonce and IV for this transfer */
    iv_rc = generate_nonce_and_iv_per_transfer(ctx);
    if (iv_rc != GEN_IV_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_hmac_sender: IV generation failed (rc=%d)\r\n",
            iv_rc
        );
        return HMAC_SENDER_ERR_GEN_IV;
    }

    /* 2) encrypt (AES-CTR) -> writes ciphertext and sets computed_size */
    enc_rc = encrypt_aes_ctr(ctx, &ctx->tmp.computed_size);
    if (enc_rc != AES_CTR_ENC_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_hmac_sender: AES-CTR encrypt failed (rc=%d)\r\n",
            enc_rc
        );
        secure_zero(ctx->IV, sizeof(ctx->IV));
        return HMAC_SENDER_ERR_AES_CTR;
    }

    /* 3) compute HMAC over IV || ciphertext */
    hmac_rc = compute_hmac_over_iv_cipher(
                    ctx,
                    ctx->auth.mac,
                    sizeof(ctx->auth.mac),
                    &ctx->auth.mac_len);
    if (hmac_rc != HMAC_IVC_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_hmac_sender: HMAC computation failed (rc=%d)\r\n",
            hmac_rc
        );
        secure_zero(ctx->IV, sizeof(ctx->IV));
        return HMAC_SENDER_ERR_HMAC;
    }

    return HMAC_SENDER_SUCCESS;
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
verify_dec_status_t verify_mac_decrypt_recombine_receiver(crypto_ctx_t *ctx)
{
    hmac_ivc_status_t    hmac_rc;
    aes_ctr_dec_status_t dec_rc;
    size_t               dec_len = 0;
    size_t               count, i;

    if (ctx == NULL) {
        return VERIFY_DEC_ERR_BAD_PARAM;
    }

    /* 1) Compute local HMAC over IV || ciphertext (receiver-side) */
    hmac_rc = compute_hmac_over_iv_cipher_receiver(
                 ctx,
                 ctx->auth.mac_check,
                 sizeof(ctx->auth.mac_check),
                 &ctx->auth.mac_check_len
             );
    if (hmac_rc != HMAC_IVC_SUCCESS) {
        log_printf(
            "[ERROR] verify_mac_decrypt_recombine_receiver: HMAC compute failed (rc=%d)\r\n",
            hmac_rc
        );
        secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));
        return VERIFY_DEC_ERR_HMAC_COMPUTE;
    }

    /* 2) Constant-time MAC length + value check */
    if (ctx->auth.mac_check_len != ctx->auth.mac_len) {
        log_printf(
            "[ERROR] verify_mac_decrypt_recombine_receiver: MAC length mismatch (%zu != %zu)\r\n",
            ctx->auth.mac_check_len, ctx->auth.mac_len
        );
        secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));
        return VERIFY_DEC_ERR_MAC_LEN;
    }

    if (ct_memcmp(ctx->auth.mac_check, ctx->auth.mac, ctx->auth.mac_check_len) != 0) {
        log_printf(
            "[ERROR] verify_mac_decrypt_recombine_receiver: MAC verification failed\r\n"
        );
        secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));
        return VERIFY_DEC_ERR_MAC_MISMATCH;
    }

    /* 3) Decrypt AES-CTR */
    dec_rc = decrypt_aes_ctr(ctx, &dec_len);
    if (dec_rc != AES_CTR_DEC_SUCCESS) {
        log_printf(
            "[ERROR] verify_mac_decrypt_recombine_receiver: AES-CTR decrypt failed (rc=%d)\r\n",
            dec_rc
        );
        secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));
        secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
        return VERIFY_DEC_ERR_AES_CTR_DEC;
    }

    /* 4) Length sanity check */
    if (dec_len != ctx->tmp.computed_size) {
        log_printf(
            "[ERROR] verify_mac_decrypt_recombine_receiver: length mismatch (dec_len=%zu computed_size=%zu)\r\n",
            dec_len, ctx->tmp.computed_size
        );
        secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));
        secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
        return VERIFY_DEC_ERR_LENGTH_MISMATCH;
    }

    /* 5) Recombine + verify ADC values */
    recombine_adc_values(ctx);

    count = ctx->cfg.adc_sample_count;
    for (i = 0; i < count; ++i) {
        if (ctx->buffers.decrypted_adc_values[i] != ctx->buffers.adc_values[i]) {
            log_printf(
                "[ERROR] verify_mac_decrypt_recombine_receiver: ADC mismatch idx=%u (got=0x%08X exp=0x%08X)\r\n",
                (unsigned)i,
                (unsigned)ctx->buffers.decrypted_adc_values[i],
                (unsigned)ctx->buffers.adc_values[i]
            );
            secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));
            secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
            return VERIFY_DEC_ERR_ADC_MISMATCH;
        }
    }

    /* Success: wipe ephemeral MAC */
    secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));

    return VERIFY_DEC_SUCCESS;
}

/* Securely clear the decrypted_adc_values array and verify clearing.
 * Uses secure_zero() then performs a volatile read verification.
 */
secure_clear_status_t secure_clear_decrypted_adc_values(crypto_ctx_t *ctx)
{
    size_t count;
    size_t total_bytes;
    volatile uint8_t *p;
    size_t i;

    if (ctx == NULL) {
        return SECURE_CLEAR_ERR_INVALID_SIZE;
    }

    /* 1) Sanity: valid element count (runtime-configured) */
    count = ctx->cfg.adc_sample_count;
    if (count == 0U) {
        return SECURE_CLEAR_ERR_INVALID_SIZE;
    }

    /* 2) Defensive size calculation with overflow check */
    total_bytes = count * sizeof(ctx->buffers.decrypted_adc_values[0]);
    if (total_bytes / sizeof(ctx->buffers.decrypted_adc_values[0]) != count) {
        return SECURE_CLEAR_ERR_INVALID_SIZE;
    }

    /* 3) Perform secure wipe */
    secure_zero(ctx->buffers.decrypted_adc_values, total_bytes);

    /* 4) Compiler + memory barrier to prevent reordering */
    __asm__ volatile ("" ::: "memory");

    /* 5) Verify wipe using volatile access */
    p = (volatile uint8_t *)ctx->buffers.decrypted_adc_values;
    for (i = 0; i < total_bytes; ++i) {
        if (p[i] != 0U) {
            return SECURE_CLEAR_ERR_VERIFY_FAIL;
        }
    }

    return SECURE_CLEAR_SUCCESS;
}





/* --- Streaming-style GCM encrypt (no mac[]) --- */
gcm_enc_status_t encrypt_aes_gcm_aead(crypto_ctx_t *ctx)
{
    cmox_cipher_retval_t cres;
    size_t local_out_len = 0;
    size_t tag_len = 0;
    size_t plain_len = ctx->tmp.plain_len;

    /* Basic input sanity */
    if (plain_len == 0) {
        return GCM_ENC_ERR_BAD_INPUT;
    }

    /* Ensure output buffer has space for ciphertext + tag */
    if (plain_len > (sizeof(ctx->buffers.cipher_text_gcm_aead) - AES_GCM_TAG_SIZE)) {
        return GCM_ENC_ERR_BAD_INPUT;
    }

    /* Key / IV checks */
    if (is_all_zero_bytes(ctx->derived.aes_key, sizeof(ctx->derived.aes_key)) ||
        is_all_zero_bytes(ctx->IV_GCM, sizeof(ctx->IV_GCM))) {
        return GCM_ENC_ERR_KEY_OR_IV;
    }

    ctx->tmp.computed_size = 0; /* defensive */

    /* --- construct GCM handle (default fast impl) --- */
    cmox_gcm_handle_t gcm_ctx;
    cmox_cipher_handle_t *h = cmox_gcm_construct(&gcm_ctx, CMOX_AESFAST_GCMFAST_ENC);
    if (h == NULL) {
        return GCM_ENC_ERR_CONSTRUCT;
    }

    /* --- init handle --- */
    cres = cmox_cipher_init(h);
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_INIT;
    }

    /* --- set key --- */
    cres = cmox_cipher_setKey(h,
                              ctx->derived.aes_key,
                              (cmox_cipher_keyLen_t)sizeof(ctx->derived.aes_key));
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_SETKEY;
    }

    /* Preconditions (compile-time/runtime checks) */
    assert(sizeof(ctx->IV_GCM) == AES_GCM_AEAD_IV_SIZE); /* AES-GCM recommended IV size */

    /* --- set IV/nonce --- */
    cres = cmox_cipher_setIV(h, ctx->IV_GCM, sizeof(ctx->IV_GCM));
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_SETIV;
    }

    /* --- set tag length (recommended = 16 bytes) --- */
    cres = cmox_cipher_setTagLen(h, (size_t)AES_GCM_TAG_SIZE);
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_SETTAGLEN;
    }

    /* --- append plaintext (encrypt) --- */
    cres = cmox_cipher_append(h,
                              ctx->buffers.plain_text_bytes,
                              plain_len,
                              ctx->buffers.cipher_text_gcm_aead, /* ciphertext output */
                              &local_out_len);
    if (cres != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_APPEND;
    }

    /* CTR-like property: ciphertext length must equal plaintext length */
    if (local_out_len != plain_len) {
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_LENGTH_MISM;
    }

    /* --- initialise with 0 the authentication tag --- */
    memset(ctx->tmp.local_tag, 0, sizeof(ctx->tmp.local_tag));
    tag_len = 0;
    cres = cmox_cipher_generateTag(h, ctx->tmp.local_tag, &tag_len);
    if (cres != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        secure_zero(ctx->tmp.local_tag, sizeof(ctx->tmp.local_tag));
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_GENERATETAG;
    }

    if (tag_len != AES_GCM_TAG_SIZE) {
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        secure_zero(ctx->tmp.local_tag, sizeof(ctx->tmp.local_tag));
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_LENGTH_MISM;
    }

    /* --- append tag to combined output buffer: ciphertext || tag --- */
    memcpy(ctx->buffers.cipher_text_gcm_aead + local_out_len, ctx->tmp.local_tag, tag_len);

    /* computed_size should represent ciphertext length (without tag) */
    ctx->tmp.computed_size = local_out_len;

    /* --- cleanup internal handle state --- */
    cres = cmox_cipher_cleanup(h);
    if (cres != CMOX_CIPHER_SUCCESS) {
        /* cleanup failed; wipe outputs and report */
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        ctx->tmp.computed_size = 0;
        return GCM_ENC_ERR_CLEANUP;
    }

    /* wipe temporaries */
    secure_zero(ctx->tmp.local_tag, sizeof(ctx->tmp.local_tag));

    return GCM_ENC_SUCCESS;
}




/* Single-call AEAD encrypt function (AES-GCM) - mirrors structure of encrypt_aes_ctr(...)
 *
 * Uses cmox_aead_encrypt(...) (single-call AEAD) instead of the streaming API.
 * Outputs ciphertext||tag into `cipher_text_gcm_aead[]`, then splits the tag
 * into the existing global `mac[]` so your existing packet layout (IV || ciphertext || tag)
 * and receiver logic that expects tag in `received_mac[]` remains compatible.
 *
 * Globals used (must exist in your translation unit):
 *  - uint8_t  plain_text_bytes[]        (input plaintext)
 *  - size_t   plain_len                 (length of plaintext)
 *  - uint8_t  aes_key[]                 (AEAD key, e.g. 32 bytes for AES-256)
 *  - uint8_t  IV[]                      (IV/nonce, recommended 12 bytes)
 *  - uint8_t  cipher_text_gcm_aead[]    (output buffer: ciphertext || tag)
 *  - size_t   computed_size             (global to store ciphertext len w/o tag)
 *  - uint8_t  mac[]                     (global tag buffer)
 *  - size_t   mac_len                   (global tag length)
 *
 * Note: this function uses its own error codes (does not reuse CTR macros).
 */

/* --- Single-call AEAD encrypt (no mac[]) --- */
gcm_aead_single_status_t encrypt_aead_gcm_singlecall(crypto_ctx_t *ctx)
{
    cmox_cipher_retval_t cres;
    size_t aead_out_len = 0;
    size_t expected_out_len;
    const size_t tag_len = (size_t)AES_GCM_TAG_SIZE;
    size_t plain_len = ctx->tmp.plain_len;

    /* Basic sanity */
    if (plain_len == 0) {
        return GCM_AEAD_SINGLE_ERR_BAD_INPUT;
    }

    /* Ensure output buffer has space for ciphertext + tag */
    if (plain_len > (sizeof(ctx->buffers.cipher_text_gcm_aead) - tag_len)) {
        return GCM_AEAD_SINGLE_ERR_BAD_INPUT;
    }

    /* Key / IV must be non-zero */
    if (is_all_zero_bytes(ctx->derived.aes_key, sizeof(ctx->derived.aes_key)) ||
        is_all_zero_bytes(ctx->IV_GCM, sizeof(ctx->IV_GCM))) {
        return GCM_AEAD_SINGLE_ERR_KEY_OR_IV;
    }

    ctx->tmp.computed_size = 0;

    /* Use single-call AEAD: output will be [ciphertext || tag] */
    cres = cmox_aead_encrypt(
                CMOX_AESFAST_GCMFAST_ENC_ALGO,                 /* algorithm */
                ctx->buffers.plain_text_bytes,                 /* P_pInput */
                plain_len,                                     /* P_inputLen */
                tag_len,                                       /* P_tagLen */
                ctx->derived.aes_key,                          /* P_pKey */
                (cmox_cipher_keyLen_t)sizeof(ctx->derived.aes_key), /* P_keyLen */
                ctx->IV_GCM,                                   /* P_pIv */
                sizeof(ctx->IV_GCM),                           /* P_ivLen */
                NULL,                                          /* P_pAddData (AAD) */
                0,                                             /* P_addDataLen */
                ctx->buffers.cipher_text_gcm_aead,            /* P_pOutput (ciphertext||tag) */
                &aead_out_len                                   /* P_pOutputLen */
           );

    if (cres != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        return GCM_AEAD_SINGLE_ERR_AEAD_FAIL;
    }

    expected_out_len = plain_len + tag_len;
    if (aead_out_len != expected_out_len) {
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        return GCM_AEAD_SINGLE_ERR_LENGTH;
    }

    /* computed_size represents ciphertext length without tag */
    ctx->tmp.computed_size = plain_len;

    /* copy ciphertext into ciphertext_sender (ciphertext only) */
    memcpy(ctx->auth.ciphertext_sender,
           ctx->buffers.cipher_text_gcm_aead,
           ctx->tmp.computed_size);
    ctx->auth.ciphertext_len = ctx->tmp.computed_size;

    /* copy tag into tag_sender (last AES_GCM_TAG_SIZE bytes) */
    memcpy(ctx->auth.tag_sender,
           ctx->buffers.cipher_text_gcm_aead + ctx->tmp.computed_size,
           tag_len);
    ctx->auth.mac_len = tag_len;

    return GCM_AEAD_SINGLE_SUCCESS;
}


/* Wrapper / "mother" encrypt function that exposes both implementations
 * (streaming-style and single-call AEAD) and lets caller choose behavior.
 *
 * Behavior modes:
 *   ENCRYPT_GCM_MODE_STREAM_ONLY    -> call streaming implementation only
 *   ENCRYPT_GCM_MODE_SINGLE_ONLY    -> call single-call implementation only
 *   ENCRYPT_GCM_MODE_TRY_STREAM_THEN_SINGLE
 *                                   -> try streaming, if it fails try single-call
 *
 * It returns 0 on success, negative on error. If successful, *out_computed_size
 * holds the ciphertext length (without tag).
 *
 * This wrapper assumes the lower-level functions already exist in the TU:
 *   int encrypt_aes_gcm_aead(size_t *out_computed_size);       // streaming
 *   int encrypt_aead_gcm_singlecall(size_t *out_computed_size);// single-call
 *
 * It also preserves the robust wiping behavior: when an attempt fails, it
 * relies on the called function to have wiped sensitive buffers; if both fail
 * the wrapper also clears the combined buffer.
 */


gcm_sender_status_t generate_nonce_iv_encrypt_gcm_sender(crypto_ctx_t *ctx)
{
    gen_iv_status_t iv_rc;
    gcm_enc_status_t gcm_rc;
    gcm_aead_single_status_t aead_rc;

    /* 1) Generate nonce and IV */
    iv_rc = generate_nonce_and_iv_per_transfer(ctx);
    if (iv_rc != GEN_IV_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_gcm_sender: IV generation failed (rc=%d)\r\n",
            iv_rc
        );
        return GCM_SENDER_ERR_GEN_IV;
    }

    /* 2) Streaming-style AES-GCM encryption */
    gcm_rc = encrypt_aes_gcm_aead(ctx);
    if (gcm_rc != GCM_ENC_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_gcm_sender: GCM stream encrypt failed (rc=%d)\r\n",
            gcm_rc
        );
        secure_zero(ctx->IV, sizeof(ctx->IV));
        return GCM_SENDER_ERR_GCM_STREAM;
    }

    /* 3) Single-call AEAD AES-GCM encryption */
    aead_rc = encrypt_aead_gcm_singlecall(ctx);
    if (aead_rc != GCM_AEAD_SINGLE_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_gcm_sender: GCM AEAD single-call failed (rc=%d)\r\n",
            aead_rc
        );
        secure_zero(ctx->IV, sizeof(ctx->IV));
        secure_zero(ctx->buffers.cipher_text_gcm_aead,
                    sizeof(ctx->buffers.cipher_text_gcm_aead));
        ctx->tmp.computed_size = 0;
        return GCM_SENDER_ERR_GCM_SINGLECALL;
    }

    log_printf(
        "[SECURE] Payload encrypted (confidentiality, integrity, authenticity): "
        "ECDH-derived key + AES-GCM AEAD\r\n"
    );

    return GCM_SENDER_SUCCESS;
}



/* Receiver: streaming-style GCM decrypt + verify (expects separated ciphertext_sender & tag_sender)
 *
 * Globals assumed present in compilation unit:
 *  - uint8_t  ciphertext_sender[]        (ciphertext only)
 *  - uint8_t  tag_sender[AES_GCM_TAG_SIZE]  (tag from sender)
 *  - uint8_t  IV[]                       (received IV/nonce)
 *  - uint8_t  aes_key_receiver[]         (shared AEAD key for receiver)
 *  - uint8_t  decrypted_bytes[]          (output plaintext buffer)
 *  - size_t   computed_size              (will be set to ciphertext length on success)
 *
 * Uses the streaming API: cmox_gcm_construct -> cmox_cipher_init -> setKey -> setIV ->
 * setTagLen -> cmox_cipher_append (ciphertext -> plaintext) -> cmox_cipher_verifyTag -> cleanup.
 */

gcm_dec_status_t decrypt_verify_aes_gcm_receiver(crypto_ctx_t *ctx)
{
    cmox_cipher_retval_t cres;
    size_t out_len = 0;

    /* Defensive init */
    ctx->tmp.computed_size = 0;

    /* --- Basic sanity --- */
    if (ctx->auth.ciphertext_len == 0) {
        return GCM_DEC_ERR_BAD_INPUT;
    }

    if (ctx->auth.ciphertext_len > sizeof(ctx->auth.ciphertext_sender)) {
        return GCM_DEC_ERR_BAD_INPUT;
    }

    /* --- Key / IV sanity --- */
    if (is_all_zero_bytes(ctx->derived.aes_key_receiver, sizeof(ctx->derived.aes_key_receiver)) ||
        is_all_zero_bytes(ctx->IV_GCM, sizeof(ctx->IV_GCM))) {
        return GCM_DEC_ERR_KEY_OR_IV;
    }

    /* --- Construct GCM decrypt handle --- */
    cmox_gcm_handle_t gcm_ctx;
    cmox_cipher_handle_t *h =
        cmox_gcm_construct(&gcm_ctx, CMOX_AESFAST_GCMFAST_DEC);
    if (h == NULL) {
        return GCM_DEC_ERR_CONSTRUCT;
    }

    /* --- Init handle --- */
    cres = cmox_cipher_init(h);
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_INIT;
    }

    /* --- Set key --- */
    cres = cmox_cipher_setKey(h,
                              ctx->derived.aes_key_receiver,
                              (cmox_cipher_keyLen_t)sizeof(ctx->derived.aes_key_receiver));
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETKEY;
    }

    /* --- Set IV (nonce) --- */
    cres = cmox_cipher_setIV(h, ctx->IV_GCM, sizeof(ctx->IV_GCM));
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETIV;
    }

    /* --- Set expected tag length --- */
    cres = cmox_cipher_setTagLen(h, (size_t)AES_GCM_TAG_SIZE);
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETTAGLEN;
    }

    /* --- Decrypt: append ciphertext -> plaintext --- */
    cres = cmox_cipher_append(h,
                              ctx->auth.ciphertext_sender,
                              ctx->auth.ciphertext_len,
                              ctx->buffers.decrypted_bytes,
                              &out_len);
    if (cres != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_APPEND;
    }

    /* --- Verify authentication tag --- */
    uint32_t fault_check = 0;
    cres = cmox_cipher_verifyTag(h, ctx->auth.tag_sender, &fault_check);

    /* --- Cleanup handle (always) --- */
    cmox_cipher_retval_t cleanup_ret = cmox_cipher_cleanup(h);

    /* --- Verify results per CMOX rules --- */
    if (cres != CMOX_CIPHER_AUTH_SUCCESS ||
        fault_check != CMOX_CIPHER_AUTH_SUCCESS) {
        secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
        return GCM_DEC_ERR_VERIFY;
    }

    if (cleanup_ret != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
        return GCM_DEC_ERR_CLEANUP;
    }

    /* --- Success --- */
    ctx->tmp.computed_size = out_len;   /* plaintext length */

    return GCM_DEC_SUCCESS;
}


/* Single-call AEAD decrypt (AES-GCM) - receiver side
 *
 * Globals assumed:
 *  - uint8_t  ciphertext_sender[]      (ciphertext only)
 *  - uint8_t  tag_sender[AES_GCM_TAG_SIZE]
 *  - uint8_t  IV[]                     (received IV/nonce)
 *  - uint8_t  aes_key_receiver[]       (shared AEAD key for receiver)
 *  - uint8_t  decrypted_bytes[]        (output plaintext buffer)
 *  - uint8_t  cipher_text_gcm_aead[]   (scratch/combined buffer; must be >= ciphertext_len + tag)
 *  - size_t   computed_size            (will be set to ciphertext length on success)
 *
 * Returns 0 on success, negative error on failure.
 */


gcm_aead_dec_status_t decrypt_aead_gcm_singlecall(crypto_ctx_t *ctx)
{
    cmox_cipher_retval_t cres;
    size_t in_len = 0;
    size_t plain_len_local = 0;
    const size_t tag_len = (size_t)AES_GCM_TAG_SIZE;

    /* defensive */
    ctx->tmp.computed_size = 0;

    /* --- basic sanity --- */
    if (ctx->auth.ciphertext_len == 0) {
        return GCM_AEAD_DEC_ERR_BAD_INPUT;
    }

    if (ctx->auth.ciphertext_len > sizeof(ctx->auth.ciphertext_sender)) {
        return GCM_AEAD_DEC_ERR_BAD_INPUT;
    }

    if ((ctx->auth.ciphertext_len + tag_len) >
        sizeof(ctx->buffers.cipher_text_gcm_aead_receiver)) {
        return GCM_AEAD_DEC_ERR_BAD_INPUT;
    }

    /* --- key / IV sanity --- */
    if (is_all_zero_bytes(ctx->derived.aes_key_receiver,
                          sizeof(ctx->derived.aes_key_receiver)) ||
        is_all_zero_bytes(ctx->IV_GCM, sizeof(ctx->IV_GCM))) {
        return GCM_AEAD_DEC_ERR_KEY_OR_IV;
    }

    /* --- Build combined input: ciphertext || tag --- */
    memcpy(ctx->buffers.cipher_text_gcm_aead_receiver,
           ctx->auth.ciphertext_sender,
           ctx->auth.ciphertext_len);

    memcpy(ctx->buffers.cipher_text_gcm_aead_receiver +
               ctx->auth.ciphertext_len,
           ctx->auth.tag_sender,
           tag_len);

    /* Initialize plaintext buffer defensively */
    secure_zero(ctx->buffers.decrypted_bytes,
                sizeof(ctx->buffers.decrypted_bytes));

    in_len = ctx->auth.ciphertext_len + tag_len;

    /* --- Single-call AEAD decrypt --- */
    cres = cmox_aead_decrypt(
                CMOX_AESFAST_GCMFAST_DEC_ALGO,          /* algorithm */
                ctx->buffers.cipher_text_gcm_aead_receiver, /* input: ciphertext||tag */
                in_len,                                 /* input length */
                tag_len,                                /* tag length */
                ctx->derived.aes_key_receiver,          /* key */
                (cmox_cipher_keyLen_t)
                    sizeof(ctx->derived.aes_key_receiver),
                ctx->IV_GCM,                             /* IV */
                sizeof(ctx->IV_GCM),                     /* IV length */
                NULL,                                   /* AAD */
                0,                                      /* AAD length */
                ctx->buffers.decrypted_bytes,            /* plaintext output */
                &plain_len_local                         /* plaintext length */
           );

    /* --- authentication / decrypt failure --- */
    if (cres != CMOX_CIPHER_AUTH_SUCCESS) {
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, in_len);
        return GCM_AEAD_DEC_ERR_AEAD_FAIL;
    }

    /* --- length sanity (GCM is length-preserving) --- */
    if (plain_len_local != ctx->auth.ciphertext_len) {
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, in_len);
        return GCM_AEAD_DEC_ERR_LENGTH;
    }

    /* --- success --- */
    ctx->tmp.computed_size = ctx->auth.ciphertext_len;

    /* wipe temporary combined input */
    secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, in_len);

    return GCM_AEAD_DEC_SUCCESS;
}


/* Processes authenticated AES-GCM encryption or decryption. */
gcm_receiver_status_t decrypt_verify_recombine_gcm_receiver(crypto_ctx_t *ctx)
{
    gcm_dec_status_t gcm_rc;
    gcm_aead_dec_status_t aead_rc;

    /* 1) Streaming-style AES-GCM decrypt + tag verify */
    gcm_rc = decrypt_verify_aes_gcm_receiver(ctx);
    if (gcm_rc != GCM_DEC_SUCCESS) {
        log_printf(
            "[ERROR] decrypt_verify_recombine_gcm_receiver: streaming GCM decrypt failed (rc=%d)\r\n",
            gcm_rc
        );
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        return GCM_RECEIVER_ERR_GCM_STREAM;
    }

    /* 2) Single-call AEAD AES-GCM decrypt + tag verify */
    aead_rc = decrypt_aead_gcm_singlecall(ctx);
    if (aead_rc != GCM_AEAD_DEC_SUCCESS) {
        log_printf(
            "[ERROR] decrypt_verify_recombine_gcm_receiver: single-call GCM decrypt failed (rc=%d)\r\n",
            aead_rc
        );
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        return GCM_RECEIVER_ERR_GCM_SINGLECALL;
    }

    /* 3) Sanity: decrypted size must be non-zero */
    if (ctx->tmp.computed_size == 0U) {
        log_printf(
            "[ERROR] decrypt_verify_recombine_gcm_receiver: decrypted size is zero\r\n"
        );
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        return GCM_RECEIVER_ERR_EMPTY_PLAINTEXT;
    }

    /* 4) Recombine ADC values */
    recombine_adc_values(ctx);

    /* 5) Verify recombined ADC values */
    for (size_t i = 0; i < ctx->cfg.adc_sample_count; ++i) {
        if (ctx->buffers.decrypted_adc_values[i] !=
            ctx->buffers.adc_values[i]) {

            log_printf(
                "[ERROR] decrypt_verify_recombine_gcm_receiver: ADC mismatch idx=%u "
                "(got=0x%08X exp=0x%08X)\r\n",
                (unsigned)i,
                (unsigned)ctx->buffers.decrypted_adc_values[i],
                (unsigned)ctx->buffers.adc_values[i]
            );

            secure_zero(ctx->buffers.decrypted_bytes,
                        sizeof(ctx->buffers.decrypted_bytes));
            return GCM_RECEIVER_ERR_ADC_MISMATCH;
        }
    }

    log_printf(
        "[SECURE] Payload decrypted (confidentiality, integrity, authenticity verified): "
        "ECDH-derived key + AES-GCM AEAD\r\n"
    );

    return GCM_RECEIVER_SUCCESS;
}

/* Reconstructs ADC sample words from decrypted bytes. */
recombine_status_t recombine_adc_values(crypto_ctx_t *ctx)
{
    size_t i;
    size_t bps;

    if (ctx == NULL) {
        return RECOMBINE_ERR_NULL_CTX;
    }

    bps = ctx->cfg.bytes_per_sample;

    /* This implementation assumes 32-bit ADC samples */
    if (bps != sizeof(uint32_t)) {
        return RECOMBINE_ERR_BAD_CONFIG;
    }

    /* Ensure decrypted data length is sufficient */
    if (ctx->tmp.computed_size < ctx->cfg.adc_sample_count * bps) {
        return RECOMBINE_ERR_LENGTH_MISMATCH;
    }

    for (i = 0; i < ctx->cfg.adc_sample_count; i++) {
        size_t off = i * bps;

        ctx->buffers.decrypted_adc_values[i] =
              ((uint32_t)ctx->buffers.decrypted_bytes[off]     << 24)
            | ((uint32_t)ctx->buffers.decrypted_bytes[off + 1] << 16)
            | ((uint32_t)ctx->buffers.decrypted_bytes[off + 2] << 8)
            | ((uint32_t)ctx->buffers.decrypted_bytes[off + 3]);
    }

    return RECOMBINE_OK;
}

/* Configures the module for its next operation. */
void crypto_ctx_init(crypto_ctx_t *ctx)
{
    /* ---------- config ---------- */
    ctx->cfg.adc_sample_count  = ADC_SAMPLE_COUNT;
    ctx->cfg.bytes_per_sample  = BYTES_PER_SAMPLE;

    /* ---------- buffers / IO ---------- */
    for (size_t i = 0; i < ADC_SAMPLE_COUNT; i++) {
        ctx->buffers.adc_values[i]           = 0U;
        ctx->buffers.decrypted_adc_values[i] = 0U;
    }

    for (size_t i = 0; i < sizeof(ctx->buffers.plain_text_bytes); i++) {
        ctx->buffers.plain_text_bytes[i] = 0U;
        ctx->buffers.decrypted_bytes[i]  = 0U;
    }

    for (size_t i = 0; i < sizeof(ctx->buffers.cipher_text_bytes); i++) {
        ctx->buffers.cipher_text_bytes[i] = 0U;
    }

    for (size_t i = 0; i < sizeof(ctx->buffers.cipher_text_gcm_aead); i++) {
        ctx->buffers.cipher_text_gcm_aead[i] = 0U;
        ctx->buffers.cipher_text_gcm_aead_receiver[i] = 0U;
    }

    /* ---------- asymmetric keys ---------- */
    for (size_t i = 0; i < AES_256_KEY_SIZE; i++) {
        ctx->keys.private_key_sender[i]   = 0U;
        ctx->keys.private_key_receiver[i] = 0U;
    }

    for (size_t i = 0; i < AES_512_KEY_SIZE; i++) {
        ctx->keys.public_key_sender[i]   = 0U;
        ctx->keys.public_key_receiver[i] = 0U;
    }

    ctx->keys.priv_len_sender     = 0U;
    ctx->keys.pub_len_sender      = 0U;
    ctx->keys.priv_len_receiver   = 0U;
    ctx->keys.pub_len_receiver    = 0U;

    /* ---------- derived symmetric keys ---------- */
    for (size_t i = 0; i < 32; i++) {
        ctx->derived.aes_key[i]          = 0U;
        ctx->derived.aes_key_receiver[i] = 0U;
        ctx->derived.mac_key[i]          = 0U;
        ctx->derived.mac_key_receiver[i] = 0U;
    }

//    /* ---------- DRBG / ECC ---------- */
//    ctx->drbg_ctx = NULL;

    for (size_t i = 0; i < sizeof(ctx->working_buffer); i++) {
        ctx->working_buffer[i] = 0U;
    }

    /* ---------- AEAD / MAC ---------- */
    for (size_t i = 0; i < 32; i++) {
        ctx->auth.mac[i]       = 0U;
        ctx->auth.mac_check[i] = 0U;
    }

    for (size_t i = 0; i < AES_GCM_TAG_SIZE; i++) {
        ctx->auth.tag_sender[i] = 0U;
    }

    for (size_t i = 0; i < sizeof(ctx->auth.ciphertext_sender); i++) {
        ctx->auth.ciphertext_sender[i] = 0U;
    }

    ctx->auth.ciphertext_len  = sizeof(ctx->auth.ciphertext_sender);
    ctx->auth.mac_len         = 0U;
    ctx->auth.mac_check_len   = 0U;

    /* ---------- temporary / computed ---------- */
    for (size_t i = 0; i < CMOX_ECC_SECP256R1_SECRET_LEN; i++) {
        ctx->tmp.computed_secret[i]          = 0U;
        ctx->tmp.computed_secret_receiver[i] = 0U;
    }

    for (size_t i = 0; i < GCM_GCM_TAG_SIZE; i++) {
        ctx->tmp.local_tag[i] = 0U;
    }

    ctx->tmp.computed_size = 0U;
    ctx->tmp.plain_len     = sizeof(ctx->buffers.plain_text_bytes);

    /* ---------- IVs ---------- */
    for (size_t i = 0; i < AES_256_KEY_SIZE; i++) {
        ctx->Key[i] = 0U;
    }

    for (size_t i = 0; i < AES_256_IV_SIZE; i++) {
        ctx->IV[i] = 0U;
    }

    for (size_t i = 0; i < AES_GCM_AEAD_IV_SIZE; i++) {
        ctx->IV_GCM[i] = 0U;
    }
}







