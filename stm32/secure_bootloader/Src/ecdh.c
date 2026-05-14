/*
 * ecdh_refactored.c
 *
 * Refactored from original ecdh.c — context-based, safer,
 * better error handling, constant-time checks, fewer globals.
 *
 *
 */
/* Author: i fiksuari pas prog | Documentation date: 14 May 2026 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>
#include "tim.h"
#include "ecdh.h" /* your project header with macros & prototypes */

#define KEY_PRINT_DELAY_MS       20U
#define KEY_BYTES_PER_LINE        8U

/* Constant-time comparison helper, implemented later in this file. */
static int ct_memcmp(const void *a_, const void *b_, size_t n);

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

/* Handles the store be32 operation. */
static void store_be32(uint8_t out[4], uint32_t value)
{
    /* Check the return status before continuing to the next step. */
    out[0] = (uint8_t)(value >> 24);
    out[1] = (uint8_t)(value >> 16);
    out[2] = (uint8_t)(value >> 8);
    out[3] = (uint8_t)value;
}

/* Handles the store be64 operation. */
static void store_be64(uint8_t out[8], uint64_t value)
{
    /* Check the return status before continuing to the next step. */
    out[0] = (uint8_t)(value >> 56);
    out[1] = (uint8_t)(value >> 48);
    out[2] = (uint8_t)(value >> 40);
    out[3] = (uint8_t)(value >> 32);
    out[4] = (uint8_t)(value >> 24);
    out[5] = (uint8_t)(value >> 16);
    out[6] = (uint8_t)(value >> 8);
    out[7] = (uint8_t)value;
}

/* Handles the load be32 operation. */
static uint32_t load_be32(const uint8_t in[4])
{
    /* Check the return status before continuing to the next step. */
    return ((uint32_t)in[0] << 24) |
           ((uint32_t)in[1] << 16) |
           ((uint32_t)in[2] << 8)  |
           ((uint32_t)in[3]);
}


/*
 * Allow the UART logger/serial monitor to drain after each complete
 * debug line. This is intentionally used only by the verbose AES-GCM
 * bring-up logs.
 */
static void crypto_uart_log_delay(void)
{
    /* Keep the wait bounded to avoid a stalled execution path. */
    tim2_delay_ms(CRYPTO_UART_LOG_DELAY_MS);
}


/*
 * Print a binary buffer in a UART-friendly hexadecimal format.
 *
 * A line break is inserted after every 16 bytes so plaintext,
 * nonce, ciphertext and tag layouts remain easy to inspect.
 */
static void log_hex_buffer(
    const char *label,
    const uint8_t *buffer,
    size_t length)
{
    /* Check the transport state before writing another message. */
    if ((label == NULL) || (buffer == NULL)) {
        return;
    }

    log_printf(
        "%s [%u bytes]:\r\n",
        label,
        (unsigned)length
    );
    crypto_uart_log_delay();

    for (size_t i = 0U; i < length; ++i) {
        log_printf(
            "%02X",
            (unsigned)buffer[i]
        );

        if ((((i + 1U) % 16U) == 0U) ||
            ((i + 1U) == length)) {
            log_printf("\r\n");
            crypto_uart_log_delay();
        } else {
            log_printf(" ");
        }
    }
}


/*
 * Print the logical 32-bit ADC values represented by the encrypted
 * payload. This is useful during the current local STM32 self-test.
 */
static void log_adc_values(
    const char *label,
    const uint32_t *values,
    size_t count)
{
    /* Check the sampling result before using the values. */
    if ((label == NULL) || (values == NULL)) {
        return;
    }

    log_printf(
        "%s [%u samples]:\r\n",
        label,
        (unsigned)count
    );
    crypto_uart_log_delay();

    for (size_t i = 0U; i < count; ++i) {
        log_printf(
            "  sample[%u] = 0x%08lX (%lu)\r\n",
            (unsigned)i,
            (unsigned long)values[i],
            (unsigned long)values[i]
        );
        crypto_uart_log_delay();
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
static int drbg_fill_bytes(crypto_ctx_t *ctx,
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


/*
 * DEVELOPMENT ONLY.
 *
 * Prints a key as a directly copyable C array.
 * A 20 ms TIM2 delay is added after every complete UART line.
 */
static void print_key_as_c_array_uart(
    const char *key_name,
    const uint8_t *key,
    size_t key_len)
{
    /* Check the transport state before writing another message. */
    char line[96];
    size_t index = 0U;

    if ((key_name == NULL) ||
        (key == NULL) ||
        (key_len == 0U)) {
        return;
    }

    log_printf(
        "\r\n[KEY EXPORT] %s [%u bytes]\r\n",
        key_name,
        (unsigned)key_len
    );

    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    log_printf(
        "static const uint8_t %s[%u] = {\r\n",
        key_name,
        (unsigned)key_len
    );

    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    while (index < key_len) {
        size_t line_position = 0U;
        size_t line_end;
        int written;

        written = snprintf(
            &line[line_position],
            sizeof(line) - line_position,
            "    "
        );

        if ((written < 0) ||
            ((size_t)written >=
             (sizeof(line) - line_position))) {
            return;
        }

        line_position += (size_t)written;

        line_end = index + KEY_BYTES_PER_LINE;

        if (line_end > key_len) {
            line_end = key_len;
        }

        while (index < line_end) {
            written = snprintf(
                &line[line_position],
                sizeof(line) - line_position,
                "0x%02X%s",
                (unsigned)key[index],
                ((index + 1U) < key_len) ? ", " : ""
            );

            if ((written < 0) ||
                ((size_t)written >=
                 (sizeof(line) - line_position))) {
                return;
            }

            line_position += (size_t)written;
            index++;
        }

        written = snprintf(
            &line[line_position],
            sizeof(line) - line_position,
            "\r\n"
        );

        if ((written < 0) ||
            ((size_t)written >=
             (sizeof(line) - line_position))) {
            return;
        }

        log_printf("%s", line);
        tim2_delay_ms(KEY_PRINT_DELAY_MS);
    }

    log_printf("};\r\n");
    tim2_delay_ms(KEY_PRINT_DELAY_MS);
}


/**
 * Generate two ECDSA (secp256r1) keypairs (priv + pub) for sender and receiver.
 * Returns 0 on success, negative error code on failure. On failure both key buffers are zeroed.
 * After success, pub_len_sender/pub_len_receiver contain the actual public-key lengths.
 */
gen_keys_status_t generate_keypairs_cmox(crypto_ctx_t *ctx)
{
    if (ctx == NULL) {
        return GEN_KEYS_ERR_BAD_PARAM;
    }

    /*
     * Always start with a known context state.
     */
    memset(
        ctx->keys.private_key_sender,
        0,
        sizeof(ctx->keys.private_key_sender)
    );

    memset(
        ctx->keys.public_key_sender,
        0,
        sizeof(ctx->keys.public_key_sender)
    );

    memset(
        ctx->keys.private_key_receiver,
        0,
        sizeof(ctx->keys.private_key_receiver)
    );

    memset(
        ctx->keys.public_key_receiver,
        0,
        sizeof(ctx->keys.public_key_receiver)
    );

    ctx->keys.priv_len_sender   = 0U;
    ctx->keys.pub_len_sender    = 0U;
    ctx->keys.priv_len_receiver = 0U;
    ctx->keys.pub_len_receiver  = 0U;

#if ECDH_PROVISIONED_KEYS_PRESENT

    /*
     * The four fixed keys are present in ecdh.h.
     *
     * Do not construct the DRBG.
     * Do not generate replacement keypairs.
     * Do not print private or public key bytes.
     */
    if (is_all_zero_buf(
            ctx,
            ecdh_provisioned_sender_private_key,
            sizeof(ecdh_provisioned_sender_private_key)) ||
        is_all_zero_buf(
            ctx,
            ecdh_provisioned_sender_public_key,
            sizeof(ecdh_provisioned_sender_public_key)) ||
        is_all_zero_buf(
            ctx,
            ecdh_provisioned_receiver_private_key,
            sizeof(ecdh_provisioned_receiver_private_key)) ||
        is_all_zero_buf(
            ctx,
            ecdh_provisioned_receiver_public_key,
            sizeof(ecdh_provisioned_receiver_public_key))) {

        log_printf(
            "[KEYS][ERROR] Provisioned key arrays are present "
            "but at least one key is all-zero.\r\n"
        );
        tim2_delay_ms(KEY_PRINT_DELAY_MS);

        return GEN_KEYS_ERR_BAD_PARAM;
    }

    memcpy(
        ctx->keys.private_key_sender,
        ecdh_provisioned_sender_private_key,
        sizeof(ecdh_provisioned_sender_private_key)
    );

    memcpy(
        ctx->keys.public_key_sender,
        ecdh_provisioned_sender_public_key,
        sizeof(ecdh_provisioned_sender_public_key)
    );

    memcpy(
        ctx->keys.private_key_receiver,
        ecdh_provisioned_receiver_private_key,
        sizeof(ecdh_provisioned_receiver_private_key)
    );

    memcpy(
        ctx->keys.public_key_receiver,
        ecdh_provisioned_receiver_public_key,
        sizeof(ecdh_provisioned_receiver_public_key)
    );

    ctx->keys.priv_len_sender =
        sizeof(ecdh_provisioned_sender_private_key);

    ctx->keys.pub_len_sender =
        sizeof(ecdh_provisioned_sender_public_key);

    ctx->keys.priv_len_receiver =
        sizeof(ecdh_provisioned_receiver_private_key);

    ctx->keys.pub_len_receiver =
        sizeof(ecdh_provisioned_receiver_public_key);

    log_printf(
        "[KEYS] All four provisioned secp256r1 keys were found "
        "in ecdh.h.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    log_printf(
        "[KEYS] Key generation is not required. "
        "Existing sender and receiver keypairs loaded.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    log_printf(
        "[KEYS] Key material will not be printed.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    return GEN_KEYS_SUCCESS;

#else

    /*
     * No provisioned keys are enabled in ecdh.h.
     * Generate two new keypairs and export them once for development.
     */
    cmox_ecc_retval_t  ecc_ret;
    cmox_drbg_retval_t drbg_ret;
    uint8_t random_buf[128];
    size_t random_len = AES_256_KEY_SIZE;

    memset(random_buf, 0, sizeof(random_buf));

    log_printf(
        "[KEYS] No provisioned key set is enabled in ecdh.h.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    log_printf(
        "[KEYS] Generating new sender and receiver "
        "secp256r1 keypairs.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    ctx->drbg_ctx = cmox_ctr_drbg_construct(
        &ctx->drbg,
        CMOX_CTR_DRBG_AES256
    );

    if (ctx->drbg_ctx == NULL) {
        return GEN_KEYS_ERR_DRBG_CONSTRUCT;
    }

    drbg_ret = cmox_drbg_init(
        ctx->drbg_ctx,
        entropy_hashed,
        sizeof(entropy_hashed),
        device_uid_buffer_hashed,
        sizeof(device_uid_buffer_hashed),
        nonce_hashed,
        sizeof(nonce_hashed)
    );

    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_KEYS_ERR_DRBG_INIT;
    }

    /* ---------------- Generate sender keypair ---------------- */
    for (;;) {
        int drbg_rc = drbg_fill_bytes(
            ctx,
            random_buf,
            random_len
        );

        if (drbg_rc != 0) {
            secure_zero(random_buf, sizeof(random_buf));
            (void)cmox_drbg_cleanup(ctx->drbg_ctx);
            return (gen_keys_status_t)drbg_rc;
        }

        cmox_ecc_construct(
            &ctx->ecc,
            CMOX_ECC256_MATH_FUNCS,
            ctx->working_buffer,
            sizeof(ctx->working_buffer)
        );

        size_t local_priv_len =
            sizeof(ctx->keys.private_key_sender);

        size_t local_pub_len =
            sizeof(ctx->keys.public_key_sender);

        ecc_ret = cmox_ecdsa_keyGen(
            &ctx->ecc,
            CMOX_ECC_SECP256R1_LOWMEM,
            random_buf,
            random_len,
            ctx->keys.private_key_sender,
            &local_priv_len,
            ctx->keys.public_key_sender,
            &local_pub_len
        );

        cmox_ecc_cleanup(&ctx->ecc);

        if (ecc_ret == CMOX_ECC_SUCCESS) {
            ctx->keys.priv_len_sender = local_priv_len;
            ctx->keys.pub_len_sender  = local_pub_len;
            break;
        }

        if (ecc_ret == CMOX_ECC_ERR_WRONG_RANDOM) {
            size_t needed = local_priv_len;

            if (needed > sizeof(random_buf)) {
                secure_zero(random_buf, sizeof(random_buf));
                (void)cmox_drbg_cleanup(ctx->drbg_ctx);

                memset(
                    ctx->keys.private_key_sender,
                    0,
                    sizeof(ctx->keys.private_key_sender)
                );

                memset(
                    ctx->keys.public_key_sender,
                    0,
                    sizeof(ctx->keys.public_key_sender)
                );

                return GEN_KEYS_ERR_RANDOM_TOO_LARGE;
            }

            random_len = needed;
            continue;
        }

        secure_zero(random_buf, sizeof(random_buf));
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);

        memset(
            ctx->keys.private_key_sender,
            0,
            sizeof(ctx->keys.private_key_sender)
        );

        memset(
            ctx->keys.public_key_sender,
            0,
            sizeof(ctx->keys.public_key_sender)
        );

        return GEN_KEYS_ERR_KEYGEN_SENDER;
    }

    random_len = AES_256_KEY_SIZE;

    /* ---------------- Generate receiver keypair ---------------- */
    for (;;) {
        int drbg_rc = drbg_fill_bytes(
            ctx,
            random_buf,
            random_len
        );

        if (drbg_rc != 0) {
            secure_zero(random_buf, sizeof(random_buf));

            memset(
                ctx->keys.private_key_sender,
                0,
                sizeof(ctx->keys.private_key_sender)
            );

            memset(
                ctx->keys.public_key_sender,
                0,
                sizeof(ctx->keys.public_key_sender)
            );

            (void)cmox_drbg_cleanup(ctx->drbg_ctx);
            return (gen_keys_status_t)drbg_rc;
        }

        cmox_ecc_construct(
            &ctx->ecc,
            CMOX_ECC256_MATH_FUNCS,
            ctx->working_buffer,
            sizeof(ctx->working_buffer)
        );

        size_t local_priv_len =
            sizeof(ctx->keys.private_key_receiver);

        size_t local_pub_len =
            sizeof(ctx->keys.public_key_receiver);

        ecc_ret = cmox_ecdsa_keyGen(
            &ctx->ecc,
            CMOX_ECC_SECP256R1_LOWMEM,
            random_buf,
            random_len,
            ctx->keys.private_key_receiver,
            &local_priv_len,
            ctx->keys.public_key_receiver,
            &local_pub_len
        );

        cmox_ecc_cleanup(&ctx->ecc);

        if (ecc_ret == CMOX_ECC_SUCCESS) {
            ctx->keys.priv_len_receiver = local_priv_len;
            ctx->keys.pub_len_receiver  = local_pub_len;
            break;
        }

        if (ecc_ret == CMOX_ECC_ERR_WRONG_RANDOM) {
            size_t needed = local_priv_len;

            if (needed > sizeof(random_buf)) {
                secure_zero(random_buf, sizeof(random_buf));

                memset(
                    ctx->keys.private_key_sender,
                    0,
                    sizeof(ctx->keys.private_key_sender)
                );

                memset(
                    ctx->keys.public_key_sender,
                    0,
                    sizeof(ctx->keys.public_key_sender)
                );

                memset(
                    ctx->keys.private_key_receiver,
                    0,
                    sizeof(ctx->keys.private_key_receiver)
                );

                memset(
                    ctx->keys.public_key_receiver,
                    0,
                    sizeof(ctx->keys.public_key_receiver)
                );

                (void)cmox_drbg_cleanup(ctx->drbg_ctx);
                return GEN_KEYS_ERR_RANDOM_TOO_LARGE;
            }

            random_len = needed;
            continue;
        }

        secure_zero(random_buf, sizeof(random_buf));

        memset(
            ctx->keys.private_key_sender,
            0,
            sizeof(ctx->keys.private_key_sender)
        );

        memset(
            ctx->keys.public_key_sender,
            0,
            sizeof(ctx->keys.public_key_sender)
        );

        memset(
            ctx->keys.private_key_receiver,
            0,
            sizeof(ctx->keys.private_key_receiver)
        );

        memset(
            ctx->keys.public_key_receiver,
            0,
            sizeof(ctx->keys.public_key_receiver)
        );

        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        return GEN_KEYS_ERR_KEYGEN_RECEIVER;
    }

    secure_zero(random_buf, sizeof(random_buf));

    drbg_ret = cmox_drbg_cleanup(ctx->drbg_ctx);

    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        memset(
            ctx->keys.private_key_sender,
            0,
            sizeof(ctx->keys.private_key_sender)
        );

        memset(
            ctx->keys.public_key_sender,
            0,
            sizeof(ctx->keys.public_key_sender)
        );

        memset(
            ctx->keys.private_key_receiver,
            0,
            sizeof(ctx->keys.private_key_receiver)
        );

        memset(
            ctx->keys.public_key_receiver,
            0,
            sizeof(ctx->keys.public_key_receiver)
        );

        return GEN_KEYS_ERR_DRBG_CLEANUP;
    }

    /*
     * Only newly generated keys are printed.
     */
    log_printf(
        "\r\n"
        "========== SECP256R1 KEY EXPORT BEGIN ==========\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    print_key_as_c_array_uart(
        "sender_private_key",
        ctx->keys.private_key_sender,
        ctx->keys.priv_len_sender
    );

    print_key_as_c_array_uart(
        "sender_public_key",
        ctx->keys.public_key_sender,
        ctx->keys.pub_len_sender
    );

    print_key_as_c_array_uart(
        "receiver_private_key",
        ctx->keys.private_key_receiver,
        ctx->keys.priv_len_receiver
    );

    print_key_as_c_array_uart(
        "receiver_public_key",
        ctx->keys.public_key_receiver,
        ctx->keys.pub_len_receiver
    );

    log_printf(
        "========== SECP256R1 KEY EXPORT END ============\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    return GEN_KEYS_SUCCESS;

#endif /* ECDH_PROVISIONED_KEYS_PRESENT */
}



/**
 * Compute a C-MOX ECDH result and normalize it to the canonical
 * 32-byte representation used by the ESP32 Mbed TLS side.
 *
 * C-MOX is allowed to write its native secp256r1 representation into
 * raw_secret[]. In this project that native result is 64 bytes. Only
 * bytes 0..31 are retained as the canonical ECDH KDF input.
 *
 * The permanent output is always:
 *
 *     out_secret[32] = fixed-width big-endian ECDH value
 */
static ecdh_status_t compute_cmox_ecdh_canonical32(
    crypto_ctx_t *ctx,
    const uint8_t *private_key,
    size_t private_key_len,
    const uint8_t *peer_public_key,
    size_t peer_public_key_len,
    uint8_t out_secret[ECDH_CANONICAL_SECRET_LEN],
    size_t *out_secret_len)
{
    cmox_ecc_retval_t retval;
    uint8_t raw_secret[CMOX_ECC_SECP256R1_SECRET_LEN];
    uint8_t public_key_prefixed[1U + AES_512_KEY_SIZE];
    size_t raw_secret_len;
    size_t prefixed_len;

    if (ctx == NULL ||
        private_key == NULL ||
        peer_public_key == NULL ||
        out_secret == NULL ||
        out_secret_len == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }

    if (private_key_len != AES_256_KEY_SIZE ||
        peer_public_key_len != AES_512_KEY_SIZE) {
        return ECDH_ERR_ECDH_FAILED;
    }

    secure_zero(raw_secret, sizeof(raw_secret));
    secure_zero(public_key_prefixed, sizeof(public_key_prefixed));
    secure_zero(out_secret, ECDH_CANONICAL_SECRET_LEN);
    *out_secret_len = 0U;

    /* ---------- Try 1: raw peer public key X[32] || Y[32] ---------- */
    raw_secret_len = sizeof(raw_secret);

    cmox_ecc_construct(&ctx->ecc,
                       CMOX_ECC256_MATH_FUNCS,
                       ctx->working_buffer,
                       sizeof(ctx->working_buffer));

    retval = cmox_ecdh(&ctx->ecc,
                       CMOX_ECC_CURVE_SECP256R1,
                       private_key,
                       private_key_len,
                       peer_public_key,
                       peer_public_key_len,
                       raw_secret,
                       &raw_secret_len);

    cmox_ecc_cleanup(&ctx->ecc);

    /* ---------- Try 2: 0x04 || X[32] || Y[32] ---------- */
    if (retval != CMOX_ECC_SUCCESS) {
        public_key_prefixed[0] = 0x04U;
        memcpy(&public_key_prefixed[1],
               peer_public_key,
               peer_public_key_len);

        prefixed_len = 1U + peer_public_key_len;

        secure_zero(raw_secret, sizeof(raw_secret));
        raw_secret_len = sizeof(raw_secret);

        cmox_ecc_construct(&ctx->ecc,
                           CMOX_ECC256_MATH_FUNCS,
                           ctx->working_buffer,
                           sizeof(ctx->working_buffer));

        retval = cmox_ecdh(&ctx->ecc,
                           CMOX_ECC_CURVE_SECP256R1,
                           private_key,
                           private_key_len,
                           public_key_prefixed,
                           prefixed_len,
                           raw_secret,
                           &raw_secret_len);

        cmox_ecc_cleanup(&ctx->ecc);
    }

    if (retval != CMOX_ECC_SUCCESS ||
        raw_secret_len < ECDH_CANONICAL_SECRET_LEN) {
        secure_zero(raw_secret, sizeof(raw_secret));
        secure_zero(public_key_prefixed, sizeof(public_key_prefixed));
        secure_zero(out_secret, ECDH_CANONICAL_SECRET_LEN);
        *out_secret_len = 0U;
        return ECDH_ERR_ECDH_FAILED;
    }

    /*
     * Canonical STM32<->ESP32 representation:
     * keep exactly the first 32 bytes returned by C-MOX.
     */
    memcpy(out_secret,
           raw_secret,
           ECDH_CANONICAL_SECRET_LEN);

    if (is_all_zero_buf(ctx,
                        out_secret,
                        ECDH_CANONICAL_SECRET_LEN)) {
        secure_zero(raw_secret, sizeof(raw_secret));
        secure_zero(public_key_prefixed, sizeof(public_key_prefixed));
        secure_zero(out_secret, ECDH_CANONICAL_SECRET_LEN);
        *out_secret_len = 0U;
        return ECDH_ERR_ECDH_FAILED;
    }

    *out_secret_len = ECDH_CANONICAL_SECRET_LEN;

    log_printf(
        "[INFO] C-MOX ECDH native_len=%u canonical_len=%u\r\n",
        (unsigned)raw_secret_len,
        (unsigned)*out_secret_len);

    secure_zero(raw_secret, sizeof(raw_secret));
    secure_zero(public_key_prefixed, sizeof(public_key_prefixed));

    return ECDH_SUCCESS;
}

/**
 * Compute the sender-side ECDH secret:
 *
 *     sender private key x receiver public key
 *
 * The result is normalized to exactly 32 bytes and stored in:
 *
 *     ctx->tmp.computed_secret[32]
 *     ctx->tmp.computed_secret_len = 32
 */
ecdh_status_t compute_ecdh_shared_as_sender(crypto_ctx_t *ctx,
                                            uint8_t *out_secret,
                                            size_t *out_len)
{
    /* Validate the key agreement result before deriving session keys. */
    ecdh_status_t status;
    size_t receiver_public_len;

    if (ctx == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }

    receiver_public_len =
        (ctx->keys.pub_len_receiver != 0U)
        ? ctx->keys.pub_len_receiver
        : AES_512_KEY_SIZE;

    secure_zero(ctx->tmp.computed_secret,
                sizeof(ctx->tmp.computed_secret));
    ctx->tmp.computed_secret_len = 0U;

    status = compute_cmox_ecdh_canonical32(
        ctx,
        ctx->keys.private_key_sender,
        AES_256_KEY_SIZE,
        ctx->keys.public_key_receiver,
        receiver_public_len,
        ctx->tmp.computed_secret,
        &ctx->tmp.computed_secret_len);

    if (status != ECDH_SUCCESS) {
        secure_zero(ctx->tmp.computed_secret,
                    sizeof(ctx->tmp.computed_secret));
        ctx->tmp.computed_secret_len = 0U;
        return status;
    }

    if (out_secret != NULL) {
        if (out_len == NULL ||
            *out_len < ECDH_CANONICAL_SECRET_LEN) {
            secure_zero(ctx->tmp.computed_secret,
                        sizeof(ctx->tmp.computed_secret));
            ctx->tmp.computed_secret_len = 0U;
            return ECDH_ERR_ECDH_FAILED;
        }

        memcpy(out_secret,
               ctx->tmp.computed_secret,
               ECDH_CANONICAL_SECRET_LEN);
    }

    if (out_len != NULL) {
        *out_len = ECDH_CANONICAL_SECRET_LEN;
    }

    return ECDH_SUCCESS;
}

/**
 * Compute the receiver-side ECDH secret:
 *
 *     receiver private key x sender public key
 *
 * The result is normalized to exactly 32 bytes and stored in:
 *
 *     ctx->tmp.computed_secret_receiver[32]
 *     ctx->tmp.computed_secret_receiver_len = 32
 */
ecdh_status_t compute_ecdh_shared_as_receiver(crypto_ctx_t *ctx,
                                              uint8_t *out_secret,
                                              size_t *out_len)
{
    /* Validate the key agreement result before deriving session keys. */
    ecdh_status_t status;
    size_t sender_public_len;

    if (ctx == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }

    sender_public_len =
        (ctx->keys.pub_len_sender != 0U)
        ? ctx->keys.pub_len_sender
        : AES_512_KEY_SIZE;

    secure_zero(ctx->tmp.computed_secret_receiver,
                sizeof(ctx->tmp.computed_secret_receiver));
    ctx->tmp.computed_secret_receiver_len = 0U;

    status = compute_cmox_ecdh_canonical32(
        ctx,
        ctx->keys.private_key_receiver,
        AES_256_KEY_SIZE,
        ctx->keys.public_key_sender,
        sender_public_len,
        ctx->tmp.computed_secret_receiver,
        &ctx->tmp.computed_secret_receiver_len);

    if (status != ECDH_SUCCESS) {
        secure_zero(ctx->tmp.computed_secret_receiver,
                    sizeof(ctx->tmp.computed_secret_receiver));
        ctx->tmp.computed_secret_receiver_len = 0U;
        return status;
    }

    if (out_secret != NULL) {
        if (out_len == NULL ||
            *out_len < ECDH_CANONICAL_SECRET_LEN) {
            secure_zero(ctx->tmp.computed_secret_receiver,
                        sizeof(ctx->tmp.computed_secret_receiver));
            ctx->tmp.computed_secret_receiver_len = 0U;
            return ECDH_ERR_ECDH_FAILED;
        }

        memcpy(out_secret,
               ctx->tmp.computed_secret_receiver,
               ECDH_CANONICAL_SECRET_LEN);
    }

    if (out_len != NULL) {
        *out_len = ECDH_CANONICAL_SECRET_LEN;
    }

    return ECDH_SUCCESS;
}

/**
 * Derive sender or receiver AES/MAC keys from exactly 32 canonical
 * ECDH-secret bytes.
 *
 * AES key = SHA256(shared[32] || 0x01)
 * MAC key = SHA256(shared[32] || 0x02)
 */
kdf_status_t derive_keys_from_shared_secret(crypto_ctx_t *ctx,
                                            bool receiver)
{
    cmox_hash_retval_t retval;
    uint8_t buf[ECDH_CANONICAL_SECRET_LEN + 1U];
    const uint8_t *shared;
    size_t shared_len;
    uint8_t *aes_key;
    uint8_t *mac_key;
    size_t aes_key_len;
    size_t mac_key_len;
    size_t out_len;

    if (ctx == NULL) {
        return KDF_ERR_BAD_INPUT;
    }

    if (receiver) {
        shared = ctx->tmp.computed_secret_receiver;
        shared_len = ctx->tmp.computed_secret_receiver_len;
        aes_key = ctx->derived.aes_key_receiver;
        mac_key = ctx->derived.mac_key_receiver;
        aes_key_len = sizeof(ctx->derived.aes_key_receiver);
        mac_key_len = sizeof(ctx->derived.mac_key_receiver);
    } else {
        shared = ctx->tmp.computed_secret;
        shared_len = ctx->tmp.computed_secret_len;
        aes_key = ctx->derived.aes_key;
        mac_key = ctx->derived.mac_key;
        aes_key_len = sizeof(ctx->derived.aes_key);
        mac_key_len = sizeof(ctx->derived.mac_key);
    }

    if (shared_len != ECDH_CANONICAL_SECRET_LEN ||
        aes_key_len != AES_256_KEY_SIZE ||
        mac_key_len != AES_256_KEY_SIZE ||
        is_all_zero_buf(ctx, shared, shared_len)) {
        return KDF_ERR_BAD_INPUT;
    }

    secure_zero(buf, sizeof(buf));
    secure_zero(aes_key, aes_key_len);
    secure_zero(mac_key, mac_key_len);

    memcpy(buf, shared, ECDH_CANONICAL_SECRET_LEN);

    /* ---------- AES key: SHA256(shared[32] || 0x01) ---------- */
    buf[ECDH_CANONICAL_SECRET_LEN] = 0x01U;
    out_len = aes_key_len;

    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf,
                               ECDH_CANONICAL_SECRET_LEN + 1U,
                               aes_key,
                               aes_key_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS ||
        out_len != aes_key_len) {
        secure_zero(buf, sizeof(buf));
        secure_zero(aes_key, aes_key_len);
        secure_zero(mac_key, mac_key_len);
        return KDF_ERR_HASH_FAILED;
    }

    /* ---------- MAC key: SHA256(shared[32] || 0x02) ---------- */
    buf[ECDH_CANONICAL_SECRET_LEN] = 0x02U;
    out_len = mac_key_len;

    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               buf,
                               ECDH_CANONICAL_SECRET_LEN + 1U,
                               mac_key,
                               mac_key_len,
                               &out_len);

    if (retval != CMOX_HASH_SUCCESS ||
        out_len != mac_key_len) {
        secure_zero(buf, sizeof(buf));
        secure_zero(aes_key, aes_key_len);
        secure_zero(mac_key, mac_key_len);
        return KDF_ERR_HASH_FAILED;
    }

    secure_zero(buf, sizeof(buf));
    return KDF_SUCCESS;
}

/**
 * Compatibility wrapper retained for existing callers.
 */
kdf_status_t derive_keys_from_shared_secret_receiver(crypto_ctx_t *ctx)
{
    /* Check the digest result before using it in verification. */
    return derive_keys_from_shared_secret(ctx, true);
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
gen_iv_status_t generate_nonce_and_iv_per_transfer(
    crypto_ctx_t *ctx)
{
    gcm_nonce_status_t nonce_ret;

    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }

    /*
     * No ADC/timer hashing and no DRBG reconstruction here.
     * Allocate the next guaranteed-unique GCM nonce.
     */
    nonce_ret = gcm_nonce_allocate(ctx);

    if (nonce_ret != GCM_NONCE_OK) {
        secure_zero(ctx->IV_GCM,
                    sizeof(ctx->IV_GCM));

        log_printf(
            "[ERROR] GCM nonce allocation failed (rc=%d)\r\n",
            nonce_ret
        );

        return GEN_IV_ERR_GENERATE;
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

        ctx->keys.priv_len_sender = 0U;
        ctx->keys.pub_len_sender = 0U;

        return SENDER_INIT_ERR_GENKEY;
    }

    /*
     * ---------- 2) compute sender-side ECDH secret ----------
     *
     * sender private key x receiver public key
     *
     * The function stores exactly 32 canonical bytes internally.
     */
    ret = compute_ecdh_shared_as_sender(ctx, NULL, NULL);
    if (ret != ECDH_SUCCESS) {
        log_printf("[ERROR] sender_init_ecdh_crypto: compute_ecdh_shared_as_sender failed (rc=%d)\r\n", ret);

        secure_zero(ctx->keys.private_key_sender,
                    sizeof(ctx->keys.private_key_sender));
        secure_zero(ctx->keys.public_key_sender,
                    sizeof(ctx->keys.public_key_sender));
        secure_zero(ctx->tmp.computed_secret,
                    sizeof(ctx->tmp.computed_secret));

        ctx->tmp.computed_secret_len = 0U;

        return SENDER_INIT_ERR_ECDH;
    }

    /* ---------- 3) derive AES + MAC keys from 32-byte secret ---------- */
    ret = derive_keys_from_shared_secret(ctx, false);
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

        ctx->tmp.computed_secret_len = 0U;

        return SENDER_INIT_ERR_KDF;
    }

    log_printf(
        "[INFO] sender_init_ecdh_crypto: success (shared_len=%u)\r\n",
        (unsigned)ctx->tmp.computed_secret_len);

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
    ret = validate_public_key_sender_format_ctx(
        ctx,
        ctx->keys.public_key_sender,
        ctx->keys.pub_len_sender);

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

    /*
     * ---------- 2) compute receiver-side ECDH secret ----------
     * receiver private key x sender public key
     * The function stores exactly 32 canonical bytes internally.
     */
    ret = compute_ecdh_shared_as_receiver(ctx, NULL, NULL);
    if (ret != ECDH_SUCCESS) {
        log_printf("[ERROR] receiver_init_ecdh_crypto: compute_ecdh_shared_as_receiver failed (rc=%d)\r\n", ret);

        secure_zero(ctx->tmp.computed_secret_receiver,
                    sizeof(ctx->tmp.computed_secret_receiver));
        ctx->tmp.computed_secret_receiver_len = 0U;

        return RECEIVER_INIT_ERR_ECDH;
    }

    /*
     * Local bring-up verification. In the real two-device system each
     * device keeps only its own secret, but both must produce the same
     * 32-byte canonical value.
     */
    if (ctx->tmp.computed_secret_len == ECDH_CANONICAL_SECRET_LEN) {
        if (ctx->tmp.computed_secret_receiver_len !=
                ECDH_CANONICAL_SECRET_LEN ||
            ct_memcmp(ctx->tmp.computed_secret,
                      ctx->tmp.computed_secret_receiver,
                      ECDH_CANONICAL_SECRET_LEN) != 0) {
            log_printf("[ERROR] receiver_init_ecdh_crypto: sender/receiver 32-byte ECDH secrets do not match\r\n");

            secure_zero(ctx->tmp.computed_secret_receiver,
                        sizeof(ctx->tmp.computed_secret_receiver));
            ctx->tmp.computed_secret_receiver_len = 0U;

            return RECEIVER_INIT_ERR_ECDH;
        }

        log_printf("[INFO] sender and receiver canonical 32-byte ECDH secrets match\r\n");
    }

    /* ---------- 3) derive receiver AES + MAC keys ---------- */
    ret = derive_keys_from_shared_secret(ctx, true);
    if (ret != KDF_SUCCESS) {
        log_printf("[ERROR] receiver_init_ecdh_crypto: derive_keys_from_shared_secret failed (rc=%d)\r\n", ret);

        secure_zero(ctx->tmp.computed_secret_receiver,
                    sizeof(ctx->tmp.computed_secret_receiver));
        secure_zero(ctx->derived.aes_key_receiver,
                    sizeof(ctx->derived.aes_key_receiver));
        secure_zero(ctx->derived.mac_key_receiver,
                    sizeof(ctx->derived.mac_key_receiver));
        ctx->tmp.computed_secret_receiver_len = 0U;

        return RECEIVER_INIT_ERR_KDF;
    }

    log_printf(
        "[INFO] receiver_init_ecdh_crypto: success (receiver_shared_len=%u)\r\n",
        (unsigned)ctx->tmp.computed_secret_receiver_len);

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


gcm_sender_status_t generate_nonce_iv_encrypt_gcm_sender(
    crypto_ctx_t *ctx)
{
    gen_iv_status_t iv_rc;
    gcm_enc_status_t gcm_rc;
    size_t ciphertext_len;

    if (ctx == NULL) {
        return GCM_SENDER_ERR_BAD_PARAM;
    }

    /*
     * 1) Generate one nonce/IV for this new logical packet.
     *
     * Important for UART retries:
     * Do not call this function again for each retry.
     * Cache the encrypted frame and resend the exact same bytes.
     */
    iv_rc = generate_nonce_and_iv_per_transfer(ctx);
    if (iv_rc != GEN_IV_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_gcm_sender: "
            "IV generation failed (rc=%d)\r\n",
            iv_rc
        );

        secure_zero(ctx->IV, sizeof(ctx->IV));
        secure_zero(ctx->IV_GCM, sizeof(ctx->IV_GCM));

        return GCM_SENDER_ERR_GEN_IV;
    }

    /*
     * 2) Perform AES-GCM encryption once using the streaming API.
     *
     * Output layout produced by encrypt_aes_gcm_aead():
     *
     * cipher_text_gcm_aead =
     * [ciphertext][GCM authentication tag]
     *
     * ctx->tmp.computed_size contains ciphertext length only.
     */
    gcm_rc = encrypt_aes_gcm_aead(ctx);
    if (gcm_rc != GCM_ENC_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_gcm_sender: "
            "GCM streaming encryption failed (rc=%d)\r\n",
            gcm_rc
        );

        secure_zero(ctx->IV, sizeof(ctx->IV));
        secure_zero(ctx->IV_GCM, sizeof(ctx->IV_GCM));

        secure_zero(ctx->buffers.cipher_text_gcm_aead,
                    sizeof(ctx->buffers.cipher_text_gcm_aead));

        ctx->tmp.computed_size = 0U;
        ctx->auth.ciphertext_len = 0U;
        ctx->auth.mac_len = 0U;

        return GCM_SENDER_ERR_GCM_STREAM;
    }

    ciphertext_len = ctx->tmp.computed_size;

    /*
     * 3) Validate destination buffer sizes before separating
     * ciphertext and authentication tag.
     */
    if ((ciphertext_len == 0U) ||
        (ciphertext_len > sizeof(ctx->auth.ciphertext_sender)) ||
        (AES_GCM_TAG_SIZE > sizeof(ctx->auth.tag_sender)) ||
        (ciphertext_len >
         (sizeof(ctx->buffers.cipher_text_gcm_aead) -
          AES_GCM_TAG_SIZE))) {

        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_gcm_sender: "
            "invalid ciphertext/tag buffer size\r\n"
        );

        secure_zero(ctx->buffers.cipher_text_gcm_aead,
                    sizeof(ctx->buffers.cipher_text_gcm_aead));

        secure_zero(ctx->auth.ciphertext_sender,
                    sizeof(ctx->auth.ciphertext_sender));

        secure_zero(ctx->auth.tag_sender,
                    sizeof(ctx->auth.tag_sender));

        ctx->tmp.computed_size = 0U;
        ctx->auth.ciphertext_len = 0U;
        ctx->auth.mac_len = 0U;

        return GCM_SENDER_ERR_GCM_STREAM;
    }

    /*
     * 4) Separate ciphertext from the combined output.
     */
    memcpy(ctx->auth.ciphertext_sender,
           ctx->buffers.cipher_text_gcm_aead,
           ciphertext_len);

    ctx->auth.ciphertext_len = ciphertext_len;

    /*
     * 5) Copy the GCM authentication tag located immediately
     * after the ciphertext.
     */
    memcpy(ctx->auth.tag_sender,
           ctx->buffers.cipher_text_gcm_aead + ciphertext_len,
           AES_GCM_TAG_SIZE);

    ctx->auth.mac_len = AES_GCM_TAG_SIZE;

    {
        const size_t combined_len =
            ctx->auth.ciphertext_len +
            ctx->auth.mac_len;

        const uint32_t counter_hi =
            (uint32_t)(
                ctx->gcm_nonce.current_counter >> 32);

        const uint32_t counter_lo =
            (uint32_t)(
                ctx->gcm_nonce.current_counter &
                UINT64_C(0xFFFFFFFF));

        log_printf(
            "\r\n"
            "========== STM32 AES-256-GCM ENCRYPT PACKET ==========\r\n"
        );
        crypto_uart_log_delay();

        log_printf(
            "[TX][META] session=%08lX "
            "counter=%08lX%08lX\r\n",
            (unsigned long)ctx->gcm_nonce.session_id,
            (unsigned long)counter_hi,
            (unsigned long)counter_lo
        );
        crypto_uart_log_delay();

        log_printf(
            "[TX][LENGTHS] plaintext=%u "
            "ciphertext=%u tag=%u combined=%u\r\n",
            (unsigned)ctx->tmp.plain_len,
            (unsigned)ctx->auth.ciphertext_len,
            (unsigned)ctx->auth.mac_len,
            (unsigned)combined_len
        );
        crypto_uart_log_delay();

        log_hex_buffer(
            "[TX][ORIGINAL PLAINTEXT PAYLOAD]",
            ctx->buffers.plain_text_bytes,
            ctx->tmp.plain_len
        );

        log_adc_values(
            "[TX][ORIGINAL ADC VALUES]",
            ctx->buffers.adc_values,
            ctx->cfg.adc_sample_count
        );

        log_hex_buffer(
            "[TX][NONCE IV_GCM]",
            ctx->IV_GCM,
            sizeof(ctx->IV_GCM)
        );

        log_hex_buffer(
            "[TX][CIPHERTEXT ONLY]",
            ctx->auth.ciphertext_sender,
            ctx->auth.ciphertext_len
        );

        log_hex_buffer(
            "[TX][AUTHENTICATION TAG]",
            ctx->auth.tag_sender,
            ctx->auth.mac_len
        );

        log_hex_buffer(
            "[TX][CIPHERTEXT || TAG]",
            ctx->buffers.cipher_text_gcm_aead,
            combined_len
        );

        log_printf(
            "[TX][ENCRYPTED SUCCESSFULLY] "
            "AES-256-GCM produced authenticated ciphertext "
            "using a fresh per-packet nonce.\r\n"
        );
        crypto_uart_log_delay();

        log_printf(
            "=======================================================\r\n"
        );
        crypto_uart_log_delay();
        crypto_uart_log_delay();
    }

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
gcm_receiver_status_t decrypt_verify_recombine_gcm_receiver(
    crypto_ctx_t *ctx)
{
    gcm_dec_status_t gcm_rc;

    if (ctx == NULL) {
        return GCM_RECEIVER_ERR_BAD_PARAM;
    }

    /*
     * Before this function is called, the received UART frame
     * must already have populated:
     *
     * ctx->auth.ciphertext_sender
     * ctx->auth.ciphertext_len
     * ctx->auth.tag_sender
     * ctx->auth.mac_len
     * ctx->IV_GCM
     *
     * Later, it must also provide identical AAD to the sender.
     */

    /*
     * 1) Validate received lengths before invoking AES-GCM.
     */
    if ((ctx->auth.ciphertext_len == 0U) ||
        (ctx->auth.ciphertext_len >
         sizeof(ctx->auth.ciphertext_sender)) ||
        (ctx->auth.ciphertext_len >
         sizeof(ctx->buffers.decrypted_bytes)) ||
        (ctx->auth.mac_len != AES_GCM_TAG_SIZE)) {

        log_printf(
            "[ERROR] decrypt_verify_recombine_gcm_receiver: "
            "invalid ciphertext/tag length\r\n"
        );

        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));

        return GCM_RECEIVER_ERR_EMPTY_PLAINTEXT;
    }

    /*
     * 2) Perform streaming AES-GCM authentication and decryption once.
     *
     * decrypt_verify_aes_gcm_receiver() must verify the GCM tag
     * before the resulting plaintext is accepted.
     */
    gcm_rc = decrypt_verify_aes_gcm_receiver(ctx);
    if (gcm_rc != GCM_DEC_SUCCESS) {
        log_printf(
            "[ERROR] decrypt_verify_recombine_gcm_receiver: "
            "GCM authentication/decryption failed (rc=%d)\r\n",
            gcm_rc
        );

        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));

        ctx->tmp.computed_size = 0U;

        return GCM_RECEIVER_ERR_GCM_STREAM;
    }

    /*
     * 3) Successful decryption must produce plaintext.
     */
    if (ctx->tmp.computed_size == 0U) {
        log_printf(
            "[ERROR] decrypt_verify_recombine_gcm_receiver: "
            "decrypted size is zero\r\n"
        );

        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));

        return GCM_RECEIVER_ERR_EMPTY_PLAINTEXT;
    }

    /*
     * 4) Convert decrypted byte representation back into
     * the ADC-value representation.
     */
    recombine_adc_values(ctx);

    /*
     * 5) Local self-test only.
     *
     * This comparison is valid while sender and receiver are running
     * inside the same STM32 test program because the original ADC
     * values are still available.
     *
     * Remove this comparison on the real ESP32 receiver because the
     * ESP32 will not possess the STM32's original adc_values array.
     */
    for (size_t i = 0U;
         i < ctx->cfg.adc_sample_count;
         ++i) {

        if (ctx->buffers.decrypted_adc_values[i] !=
            ctx->buffers.adc_values[i]) {

            log_printf(
                "[ERROR] decrypt_verify_recombine_gcm_receiver: "
                "ADC mismatch idx=%u "
                "(got=0x%08X exp=0x%08X)\r\n",
                (unsigned)i,
                (unsigned)ctx->buffers.decrypted_adc_values[i],
                (unsigned)ctx->buffers.adc_values[i]
            );

            secure_zero(ctx->buffers.decrypted_bytes,
                        sizeof(ctx->buffers.decrypted_bytes));

            secure_zero(ctx->buffers.decrypted_adc_values,
                        ctx->cfg.adc_sample_count *
                        sizeof(ctx->buffers.decrypted_adc_values[0]));

            return GCM_RECEIVER_ERR_ADC_MISMATCH;
        }
    }

    {
        uint8_t received_ciphertext_and_tag[
            sizeof(ctx->buffers.cipher_text_gcm_aead_receiver)
        ];

        const size_t combined_len =
            ctx->auth.ciphertext_len +
            ctx->auth.mac_len;

        const uint32_t counter_hi =
            (uint32_t)(
                ctx->gcm_nonce.current_counter >> 32);

        const uint32_t counter_lo =
            (uint32_t)(
                ctx->gcm_nonce.current_counter &
                UINT64_C(0xFFFFFFFF));

        secure_zero(
            received_ciphertext_and_tag,
            sizeof(received_ciphertext_and_tag)
        );

        memcpy(
            received_ciphertext_and_tag,
            ctx->auth.ciphertext_sender,
            ctx->auth.ciphertext_len
        );

        memcpy(
            received_ciphertext_and_tag +
                ctx->auth.ciphertext_len,
            ctx->auth.tag_sender,
            ctx->auth.mac_len
        );

        log_printf(
            "\r\n"
            "========== STM32 AES-256-GCM DECRYPT PACKET ==========\r\n"
        );
        crypto_uart_log_delay();

        log_printf(
            "[RX][META] session=%08lX "
            "counter=%08lX%08lX\r\n",
            (unsigned long)ctx->gcm_nonce.session_id,
            (unsigned long)counter_hi,
            (unsigned long)counter_lo
        );
        crypto_uart_log_delay();

        log_printf(
            "[RX][LENGTHS] ciphertext=%u "
            "tag=%u combined=%u recovered_plaintext=%u\r\n",
            (unsigned)ctx->auth.ciphertext_len,
            (unsigned)ctx->auth.mac_len,
            (unsigned)combined_len,
            (unsigned)ctx->tmp.computed_size
        );
        crypto_uart_log_delay();

        log_hex_buffer(
            "[RX][RECEIVED NONCE IV_GCM]",
            ctx->IV_GCM,
            sizeof(ctx->IV_GCM)
        );

        log_hex_buffer(
            "[RX][RECEIVED CIPHERTEXT]",
            ctx->auth.ciphertext_sender,
            ctx->auth.ciphertext_len
        );

        log_hex_buffer(
            "[RX][RECEIVED AUTHENTICATION TAG]",
            ctx->auth.tag_sender,
            ctx->auth.mac_len
        );

        log_hex_buffer(
            "[RX][RECEIVED CIPHERTEXT || TAG]",
            received_ciphertext_and_tag,
            combined_len
        );

        log_hex_buffer(
            "[RX][RECOVERED PLAINTEXT PAYLOAD]",
            ctx->buffers.decrypted_bytes,
            ctx->tmp.computed_size
        );

        log_adc_values(
            "[RX][RECOVERED ADC VALUES]",
            ctx->buffers.decrypted_adc_values,
            ctx->cfg.adc_sample_count
        );

        log_printf(
            "[RX][DECRYPTED SUCCESSFULLY] "
            "GCM tag verified, payload authenticated, "
            "plaintext recovered and ADC values matched.\r\n"
        );
        crypto_uart_log_delay();

        log_printf(
            "=======================================================\r\n"
        );

        secure_zero(
            received_ciphertext_and_tag,
            sizeof(received_ciphertext_and_tag)
        );
    }

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
    for (size_t i = 0; i < ECDH_CANONICAL_SECRET_LEN; i++) {
        ctx->tmp.computed_secret[i]          = 0U;
        ctx->tmp.computed_secret_receiver[i] = 0U;
    }

    ctx->tmp.computed_secret_len = 0U;
    ctx->tmp.computed_secret_receiver_len = 0U;

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








/* Processes authenticated AES-GCM encryption or decryption. */
gcm_nonce_status_t gcm_nonce_session_init(crypto_ctx_t *ctx)
{
    static const uint8_t domain_label[] =
    {
        'S', 'T', 'M', '3', '2',
        '-', 'G', 'C', 'M',
        '-', 'N', 'O', 'N', 'C', 'E',
        '-', 'V', '1'
    };

    /*
     * Accommodates two 65-byte uncompressed P-256 public keys,
     * two encoded lengths and the domain label.
     */
    uint8_t material[
        sizeof(domain_label) +
        2U +
        (AES_512_KEY_SIZE + 1U) +
        (AES_512_KEY_SIZE + 1U)
    ];

    uint8_t digest[32];
    size_t material_len = 0U;
    size_t digest_len = sizeof(digest);
    uint32_t session_id = 0U;
    cmox_hash_retval_t hash_ret;

    if (ctx == NULL) {
        return GCM_NONCE_ERR_BAD_PARAM;
    }

    /*
     * The function must be called only after both fresh ECDH
     * public keys have been generated.
     */
    if ((ctx->keys.pub_len_sender == 0U) ||
        (ctx->keys.pub_len_receiver == 0U) ||
        (ctx->keys.pub_len_sender >
            sizeof(ctx->keys.public_key_sender)) ||
        (ctx->keys.pub_len_receiver >
            sizeof(ctx->keys.public_key_receiver))) {

        return GCM_NONCE_ERR_BAD_KEY_STATE;
    }

    secure_zero(material, sizeof(material));
    secure_zero(digest, sizeof(digest));

    /*
     * Domain separation prevents this hash from being confused
     * with another use of the same public keys.
     */
    memcpy(&material[material_len],
           domain_label,
           sizeof(domain_label));

    material_len += sizeof(domain_label);

    /*
     * Include the public-key lengths so that the serialization
     * is unambiguous.
     */
    material[material_len++] =
        (uint8_t)ctx->keys.pub_len_sender;

    memcpy(&material[material_len],
           ctx->keys.public_key_sender,
           ctx->keys.pub_len_sender);

    material_len += ctx->keys.pub_len_sender;

    material[material_len++] =
        (uint8_t)ctx->keys.pub_len_receiver;

    memcpy(&material[material_len],
           ctx->keys.public_key_receiver,
           ctx->keys.pub_len_receiver);

    material_len += ctx->keys.pub_len_receiver;

    hash_ret = cmox_hash_compute(
        CMOX_SHA256_ALGO,
        material,
        material_len,
        digest,
        sizeof(digest),
        &digest_len
    );

    secure_zero(material, sizeof(material));

    if ((hash_ret != CMOX_HASH_SUCCESS) ||
        (digest_len != sizeof(digest))) {

        secure_zero(digest, sizeof(digest));
        return GCM_NONCE_ERR_HASH;
    }

    /*
     * Select the first nonzero 32-bit word from the digest.
     * Zero is not cryptographically forbidden, but rejecting it
     * works with your existing all-zero IV defensive check.
     */
    for (size_t offset = 0U;
         offset < sizeof(digest);
         offset += 4U) {

        session_id = load_be32(&digest[offset]);

        if (session_id != 0U) {
            break;
        }
    }

    secure_zero(digest, sizeof(digest));

    if (session_id == 0U) {
        return GCM_NONCE_ERR_HASH;
    }

    secure_zero(&ctx->gcm_nonce,
                sizeof(ctx->gcm_nonce));

    ctx->gcm_nonce.session_id = session_id;
    ctx->gcm_nonce.next_counter = GCM_COUNTER_START;
    ctx->gcm_nonce.current_counter = 0ULL;
    ctx->gcm_nonce.initialized = true;

    secure_zero(ctx->IV_GCM, sizeof(ctx->IV_GCM));

    log_printf(
        "[NONCE] New GCM session initialized: "
        "session_id=%08lX first_counter=%08lX%08lX\r\n",
        (unsigned long)ctx->gcm_nonce.session_id,
        (unsigned long)(
            ctx->gcm_nonce.next_counter >> 32),
        (unsigned long)(
            ctx->gcm_nonce.next_counter &
            UINT64_C(0xFFFFFFFF))
    );

    return GCM_NONCE_OK;
}






/* Processes authenticated AES-GCM encryption or decryption. */
gcm_nonce_status_t gcm_nonce_allocate(
    crypto_ctx_t *ctx)
{
    uint64_t counter;

    if (ctx == NULL) {
        return GCM_NONCE_ERR_BAD_PARAM;
    }

    if (!ctx->gcm_nonce.initialized) {
        return GCM_NONCE_ERR_NO_SESSION;
    }

    /*
     * Do not permit wraparound. A new ECDH session and new AES key
     * must be created before counter exhaustion.
     */
    if (ctx->gcm_nonce.next_counter == UINT64_MAX) {
        return GCM_NONCE_ERR_EXHAUSTED;
    }

    /*
     * Consume the counter before encryption.
     *
     * If encryption fails, this counter remains consumed and is
     * never reused under the same AES-GCM key.
     */
    counter = ctx->gcm_nonce.next_counter;
    ctx->gcm_nonce.next_counter++;
    ctx->gcm_nonce.current_counter = counter;

    /*
     * Construct the exact 96-bit GCM nonce:
     *
     * bytes 0..3  = session ID
     * bytes 4..11 = monotonically increasing message counter
     */
    store_be32(&ctx->IV_GCM[0],
               ctx->gcm_nonce.session_id);

    store_be64(&ctx->IV_GCM[4],
               counter);

    log_printf(
        "[NONCE] GCM nonce allocated: "
        "session=%08lX counter=%08lX%08lX\r\n",
        (unsigned long)ctx->gcm_nonce.session_id,
        (unsigned long)(counter >> 32),
        (unsigned long)(
            counter &
            UINT64_C(0xFFFFFFFF))
    );

    return GCM_NONCE_OK;
}





