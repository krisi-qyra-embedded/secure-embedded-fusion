/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>
#include <assert.h>
#include "tim.h"
#include "ecdh.h" /* your project header with macros & prototypes */

#define KEY_PRINT_DELAY_MS       20U
#define KEY_BYTES_PER_LINE        8U

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

/* Reports a fatal error and halts normal execution. */
void handle_error(const char *message) {
    /* Remain in the error loop after logging the failure. */

    log_printf("ERROR: %s\r\n", message);

    while (1) {
        // Toggle an LED to indicate an error (assuming LED_PIN is configured)
        //GPIOA->ODR ^= LED_PIN;  // Toggle LED
        for (volatile int i = 0; i < 1000000; i++);   
    }

}

/* Encodes a 32-bit value in big-endian order. */
static void store_be32(uint8_t out[4], uint32_t value)
{
    /* Place the highest-order byte at the start of the output. */
    out[0] = (uint8_t)(value >> 24);
    out[1] = (uint8_t)(value >> 16);
    out[2] = (uint8_t)(value >> 8);
    out[3] = (uint8_t)value;
}

/* Encodes a 64-bit value in big-endian order. */
static void store_be64(uint8_t out[8], uint64_t value)
{
    /* Store all eight bytes from most to least significant. */
    out[0] = (uint8_t)(value >> 56);
    out[1] = (uint8_t)(value >> 48);
    out[2] = (uint8_t)(value >> 40);
    out[3] = (uint8_t)(value >> 32);
    out[4] = (uint8_t)(value >> 24);
    out[5] = (uint8_t)(value >> 16);
    out[6] = (uint8_t)(value >> 8);
    out[7] = (uint8_t)value;
}

/* Decodes a big-endian 32-bit value. */
static uint32_t load_be32(const uint8_t in[4])
{
    /* Widen each input byte before shifting it into the result. */
    return ((uint32_t)in[0] << 24) |
           ((uint32_t)in[1] << 16) |
           ((uint32_t)in[2] << 8)  |
           ((uint32_t)in[3]);
}

/* Decodes an eight-byte big-endian nonce counter. */
static uint64_t load_be64(const uint8_t in[8])
{
    /* Promote each byte before shifting it into the 64-bit value. */
    return ((uint64_t)in[0] << 56) |
           ((uint64_t)in[1] << 48) |
           ((uint64_t)in[2] << 40) |
           ((uint64_t)in[3] << 32) |
           ((uint64_t)in[4] << 24) |
           ((uint64_t)in[5] << 16) |
           ((uint64_t)in[6] << 8)  |
           ((uint64_t)in[7]);
}

/* Pauses between crypto diagnostic messages. */
static void crypto_uart_log_delay(void)
{
    /* Allow buffered UART diagnostics to drain before the next message. */
    tim2_delay_ms(CRYPTO_UART_LOG_DELAY_MS);
}

/* Logs a labeled byte buffer in hexadecimal. */
static void log_hex_buffer(
    const char *label,
    const uint8_t *buffer,
    size_t length)
{
    /* Break output into sixteen-byte lines and pause between lines. */
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

/* Logs a labeled set of ADC words. */
static void log_adc_values(
    const char *label,
    const uint32_t *values,
    size_t count)
{
    /* Pause between sample lines to pace the UART queue. */
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

/* Serializes ADC words into big-endian bytes. */
decomp_adc_ret_t decompose_adc_values(crypto_ctx_t *ctx)
{
    /* Store the most significant byte first for each ADC value. */
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

/* Checks whether a supplied buffer contains only zeros. */
static int is_all_zero_buf(const crypto_ctx_t *ctx,
                           const uint8_t *buf,
                           size_t len)
{
    /* Accumulate all byte values before testing the result. */
    (void)ctx;  
    uint8_t acc = 0;
    for (size_t i = 0; i < len; i++) {
        acc |= buf[i];
    }
    return (acc == 0);
}

/* Checks the sender public key stored in the crypto context. */
pubkey_status_t validate_public_key_sender_format_ctx(const crypto_ctx_t *ctx,
                                                      const uint8_t *pub,
                                                      size_t pub_len)
{
    /* Validate the accepted raw or prefixed key size and reject zero key material. */
    (void)ctx;  

    if (pub == NULL) {
        return PUBKEY_ERR_NULL;
    }
    if (pub_len == AES_512_KEY_SIZE) {
        if (is_all_zero_buf(ctx, pub, AES_512_KEY_SIZE)) {
            return PUBKEY_ERR_ALLZERO;
        }
        return PUBKEY_OK;
    }
    if (pub_len == (AES_512_KEY_SIZE + 1)) {
        if (pub[0] != 0x04) {
            return PUBKEY_ERR_FORMAT;
        }
        if (is_all_zero_buf(ctx, &pub[1], AES_512_KEY_SIZE)) {
            return PUBKEY_ERR_ALLZERO;
        }
        return PUBKEY_OK;
    }
    return PUBKEY_ERR_FORMAT;
}

/* Generates an AES IV through CTR-DRBG. */
gen_iv_status_t generate_iv(crypto_ctx_t *ctx)
{
    /* Initialize the DRBG from conditioned entropy, UID personalization and nonce input. */
    cmox_drbg_retval_t retval;
    //cmox_drbg_handle_t *drbg_ctx = NULL;

    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }

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
/* Generates sender and receiver private-key bytes. */
gen_priv_status_t generate_private_key(crypto_ctx_t *ctx)
{
    /* Clean up DRBG state and clear key buffers when generation fails. */
    cmox_drbg_retval_t retval;

    if (ctx == NULL) {
        return GEN_PRIV_ERR_BAD_PARAM;
    }

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

/* Fills a buffer from the initialized DRBG. */
static int drbg_fill_bytes(crypto_ctx_t *ctx,
                               uint8_t *out,
                               size_t outlen)
{
    /* Map the CMOX generation result to the key-generation status. */
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

/* Logs a key as a C byte-array initializer. */
static void print_key_as_c_array_uart(
    const char *key_name,
    const uint8_t *key,
    size_t key_len)
{
    /* Bound each formatted line and pace the output with TIM2 delays. */
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

/* Loads provisioned P-256 keypairs or generates local pairs. */
gen_keys_status_t generate_keypairs_cmox(crypto_ctx_t *ctx)
{
    /* Clear both keypairs and lengths before loading or generating keys. */
    if (ctx == NULL) {
        return GEN_KEYS_ERR_BAD_PARAM;
    }

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

    const bool complete_provisioned_bundle =
        !is_all_zero_buf(
            ctx,
            ecdh_provisioned_sender_private_key,
            sizeof(ecdh_provisioned_sender_private_key)) &&
        !is_all_zero_buf(
            ctx,
            ecdh_provisioned_sender_public_key,
            sizeof(ecdh_provisioned_sender_public_key)) &&
        !is_all_zero_buf(
            ctx,
            ecdh_provisioned_receiver_private_key,
            sizeof(ecdh_provisioned_receiver_private_key)) &&
        !is_all_zero_buf(
            ctx,
            ecdh_provisioned_receiver_public_key,
            sizeof(ecdh_provisioned_receiver_public_key)) &&
        !is_all_zero_buf(
            ctx,
            esp32_provisioned_receiver_public_key,
            sizeof(esp32_provisioned_receiver_public_key)) &&
        !is_all_zero_buf(
            ctx,
            esp32_provisioned_sender_public_key,
            sizeof(esp32_provisioned_sender_public_key));

    if (complete_provisioned_bundle) {
        /*
         * Only the four local STM32 keys belong in ctx->keys.
         * The two ESP32 peer public keys remain pinned constants in
         * ecdh.h and are used later by cross-device ECDH functions.
         */
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
            "[KEYS] Complete six-key provisioning bundle found "
            "in ecdh.h.\r\n"
        );
        tim2_delay_ms(KEY_PRINT_DELAY_MS);

        log_printf(
            "[KEYS] Four STM32 local keys and two ESP32 peer "
            "public keys are present.\r\n"
        );
        tim2_delay_ms(KEY_PRINT_DELAY_MS);

        log_printf(
            "[KEYS] Local STM32 key generation is not required.\r\n"
        );
        tim2_delay_ms(KEY_PRINT_DELAY_MS);

        log_printf(
            "[KEYS] Provisioned key material will not be printed.\r\n"
        );
        tim2_delay_ms(KEY_PRINT_DELAY_MS);

        return GEN_KEYS_SUCCESS;
    }

    /*
     * The provisioning block exists, but at least one of the six
     * arrays is all-zero. Fall through to local STM32 key generation.
     */
    log_printf(
        "[KEYS][WARNING] Provisioned six-key bundle is incomplete "
        "or contains an all-zero key.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    log_printf(
        "[KEYS][WARNING] Generating new local STM32 sender and "
        "receiver keypairs.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    log_printf(
        "[KEYS][WARNING] ESP32 peer public keys must still be "
        "provisioned separately.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

#else

    log_printf(
        "[KEYS] ECDH_PROVISIONED_KEYS_PRESENT is disabled.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

    log_printf(
        "[KEYS] Generating new local STM32 sender and receiver "
        "keypairs.\r\n"
    );
    tim2_delay_ms(KEY_PRINT_DELAY_MS);

#endif /* ECDH_PROVISIONED_KEYS_PRESENT */

    cmox_ecc_retval_t  ecc_ret;
    cmox_drbg_retval_t drbg_ret;
    uint8_t random_buf[128];
    size_t random_len = AES_256_KEY_SIZE;

    memset(random_buf, 0, sizeof(random_buf));

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

}
/* Computes ECDH and normalizes the result to 32 bytes. */
static ecdh_status_t compute_cmox_ecdh_canonical32(
    crypto_ctx_t *ctx,
    const uint8_t *private_key,
    size_t private_key_len,
    const uint8_t *peer_public_key,
    size_t peer_public_key_len,
    uint8_t out_secret[ECDH_CANONICAL_SECRET_LEN],
    size_t *out_secret_len)
{
    /* Retry with an uncompressed public-key prefix if the raw-key call fails. */
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
 * Compute the STM32 -> ESP32 sender-side ECDH secret:
 *
 *     STM32 provisioned sender private key
 *     x
 *     ESP32 provisioned receiver public key
 *
 * The peer public key is deliberately taken from the pinned ESP32
 * provisioning array. ctx->keys.public_key_receiver remains the local
 * STM32 receiver public key and is not used for this TX direction.
 *
 * The result is normalized to exactly 32 bytes and stored in:
 *
 *     ctx->tmp.computed_secret[32]
 *     ctx->tmp.computed_secret_len = 32
 */
/* Computes the sender's ECDH shared secret. */
ecdh_status_t compute_ecdh_shared_as_sender(crypto_ctx_t *ctx,
                                            uint8_t *out_secret,
                                            size_t *out_len)
{
    /* Use the receiver public key and retain the result for key derivation. */
    ecdh_status_t status;

    if (ctx == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }

#if !ECDH_PROVISIONED_KEYS_PRESENT
    return ECDH_ERR_ECDH_FAILED;
#else

    if (ctx->keys.priv_len_sender != AES_256_KEY_SIZE ||
        is_all_zero_buf(ctx,
                        ctx->keys.private_key_sender,
                        sizeof(ctx->keys.private_key_sender)) ||
        is_all_zero_buf(ctx,
                        esp32_provisioned_receiver_public_key,
                        sizeof(esp32_provisioned_receiver_public_key))) {
        return ECDH_ERR_ECDH_FAILED;
    }

    secure_zero(ctx->tmp.computed_secret,
                sizeof(ctx->tmp.computed_secret));
    ctx->tmp.computed_secret_len = 0U;

    status = compute_cmox_ecdh_canonical32(
        ctx,
        ctx->keys.private_key_sender,
        ctx->keys.priv_len_sender,
        esp32_provisioned_receiver_public_key,
        sizeof(esp32_provisioned_receiver_public_key),
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

    log_printf(
        "[ECDH][TX] STM32 sender private x "
        "ESP32 receiver public succeeded\r\n"
    );

    return ECDH_SUCCESS;

#endif /* ECDH_PROVISIONED_KEYS_PRESENT */
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

/* Computes the receiver's ECDH shared secret. */
ecdh_status_t compute_ecdh_shared_as_receiver(
    crypto_ctx_t *ctx,
    uint8_t *out_secret,
    size_t *out_len)
{
    /* Use the sender public key and clear the secret on a failed exchange. */
    ecdh_status_t status;

    if (ctx == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }

#if !ECDH_PROVISIONED_KEYS_PRESENT

    (void)out_secret;
    (void)out_len;

    log_printf(
        "[ECDH][RX][ERROR] Provisioned ESP32 sender public key "
        "is required\r\n");

    return ECDH_ERR_ECDH_FAILED;

#else

    if ((ctx->keys.priv_len_receiver != AES_256_KEY_SIZE) ||
        is_all_zero_buf(
            ctx,
            ctx->keys.private_key_receiver,
            sizeof(ctx->keys.private_key_receiver)) ||
        is_all_zero_buf(
            ctx,
            esp32_provisioned_sender_public_key,
            sizeof(esp32_provisioned_sender_public_key))) {
        return ECDH_ERR_ECDH_FAILED;
    }

    secure_zero(
        ctx->tmp.computed_secret_receiver,
        sizeof(ctx->tmp.computed_secret_receiver));

    ctx->tmp.computed_secret_receiver_len = 0U;

    status =
        compute_cmox_ecdh_canonical32(
            ctx,
            ctx->keys.private_key_receiver,
            ctx->keys.priv_len_receiver,
            esp32_provisioned_sender_public_key,
            sizeof(esp32_provisioned_sender_public_key),
            ctx->tmp.computed_secret_receiver,
            &ctx->tmp.computed_secret_receiver_len);

    if (status != ECDH_SUCCESS) {
        secure_zero(
            ctx->tmp.computed_secret_receiver,
            sizeof(ctx->tmp.computed_secret_receiver));

        ctx->tmp.computed_secret_receiver_len = 0U;

        return status;
    }

    if (out_secret != NULL) {
        if ((out_len == NULL) ||
            (*out_len < ECDH_CANONICAL_SECRET_LEN)) {
            secure_zero(
                ctx->tmp.computed_secret_receiver,
                sizeof(ctx->tmp.computed_secret_receiver));

            ctx->tmp.computed_secret_receiver_len = 0U;

            return ECDH_ERR_ECDH_FAILED;
        }

        memcpy(
            out_secret,
            ctx->tmp.computed_secret_receiver,
            ECDH_CANONICAL_SECRET_LEN);
    }

    if (out_len != NULL) {
        *out_len = ECDH_CANONICAL_SECRET_LEN;
    }

    log_printf(
        "[ECDH][RX] STM32 receiver private x "
        "ESP32 sender public succeeded\r\n");

    return ECDH_SUCCESS;

#endif /* ECDH_PROVISIONED_KEYS_PRESENT */
}

/**
 * Derive sender or receiver AES/MAC keys from exactly 32 canonical
 * ECDH-secret bytes.
 *
 * AES key = SHA256(shared[32] || 0x01)
 * MAC key = SHA256(shared[32] || 0x02)
 */
/* Derives separate AES and MAC keys from an ECDH secret. */
kdf_status_t derive_keys_from_shared_secret(crypto_ctx_t *ctx,
                                            bool receiver)
{
    /* Hash the secret with different one-byte suffixes for the two outputs. */
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

/* Derives the receiver's AES and MAC keys. */
kdf_status_t derive_keys_from_shared_secret_receiver(crypto_ctx_t *ctx)
{
    /* Select the receiver shared secret for the same derivation scheme. */
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

/* Tests a byte sequence for an all-zero value. */
static int is_all_zero_bytes(const uint8_t *p, size_t n)
{
    /* OR every byte before evaluating the result. */
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

/* Encrypts the serialized payload with AES-CTR. */
aes_ctr_enc_status_t encrypt_aes_ctr(crypto_ctx_t *ctx,
                                    size_t *out_computed_size)
{
    /* Use the sender AES key and IV and retain the resulting byte count. */
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
/* Decrypts the AES-CTR payload into the receiver buffer. */
aes_ctr_dec_status_t decrypt_aes_ctr(crypto_ctx_t *ctx,
                                    size_t *out_decrypted_size)
{
    /* Apply the receiver-derived key with the same transfer IV. */
    cmox_cipher_retval_t cipher_ret;
    size_t dec_size_local = 0;

    if (out_decrypted_size) {
        *out_decrypted_size = 0;
    }

    if (ctx == NULL) {
        return AES_CTR_DEC_ERR_BAD_INPUT;
    }

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

    if (dec_size_local != ctx->tmp.computed_size) {
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        return AES_CTR_DEC_ERR_LENGTH_MISMATCH;
    }

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
/* Computes the sender HMAC over IV and ciphertext. */
hmac_ivc_status_t compute_hmac_over_iv_cipher(crypto_ctx_t *ctx,
                                              uint8_t *mac_out,
                                              size_t mac_out_len,
                                              size_t *out_mac_len)
{
    /* Authenticate the IV together with the encrypted payload. */
    cmox_mac_retval_t retval;
    size_t mac_input_len;

    if (ctx == NULL || mac_out == NULL || out_mac_len == NULL) {
        return HMAC_IVC_ERR_NULL_INPUT;
    }

    *out_mac_len = 0;

    if (ctx->tmp.computed_size == 0) {
        return HMAC_IVC_ERR_NO_CIPHERTEXT;
    }
    if (ctx->tmp.computed_size > sizeof(ctx->buffers.cipher_text_bytes)) {
        return HMAC_IVC_ERR_SIZE_INVALID;
    }

    if (ctx->tmp.computed_size > (SIZE_MAX - sizeof(ctx->IV))) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    mac_input_len = sizeof(ctx->IV) + ctx->tmp.computed_size;

    if (mac_input_len > sizeof(ctx->buffers.cipher_text_gcm_aead)) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    memcpy(ctx->buffers.cipher_text_gcm_aead, ctx->IV, sizeof(ctx->IV));
    memcpy(ctx->buffers.cipher_text_gcm_aead + sizeof(ctx->IV),
           ctx->buffers.cipher_text_bytes,
           ctx->tmp.computed_size);

    retval = cmox_mac_compute(
                CMOX_HMAC_SHA256_ALGO,
                ctx->buffers.cipher_text_gcm_aead, mac_input_len,
                ctx->derived.mac_key, sizeof(ctx->derived.mac_key),
                NULL, 0,
                mac_out, mac_out_len, out_mac_len
             );

    secure_zero(ctx->buffers.cipher_text_gcm_aead, mac_input_len);

    if (retval != CMOX_MAC_SUCCESS) {
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
/* Recomputes the received IV-and-ciphertext HMAC. */
hmac_ivc_status_t compute_hmac_over_iv_cipher_receiver(crypto_ctx_t *ctx,
                                                       uint8_t *mac_out,
                                                       size_t mac_out_len,
                                                       size_t *out_mac_len)
{
    /* Use the receiver MAC key for the tag comparison path. */
    cmox_mac_retval_t retval;
    size_t mac_input_len;

    if (ctx == NULL || mac_out == NULL || out_mac_len == NULL) {
        return HMAC_IVC_ERR_NULL_INPUT;
    }

    *out_mac_len = 0;

    if (ctx->tmp.computed_size == 0) {
        return HMAC_IVC_ERR_NO_CIPHERTEXT;
    }
    if (ctx->tmp.computed_size > sizeof(ctx->buffers.cipher_text_bytes)) {
        return HMAC_IVC_ERR_SIZE_INVALID;
    }

    if (ctx->tmp.computed_size > (SIZE_MAX - sizeof(ctx->IV))) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    mac_input_len = sizeof(ctx->IV) + ctx->tmp.computed_size;

    if (mac_input_len > sizeof(ctx->buffers.cipher_text_gcm_aead_receiver)) {
        return HMAC_IVC_ERR_SIZE_OVERFLOW;
    }

    memcpy(ctx->buffers.cipher_text_gcm_aead_receiver, ctx->IV, sizeof(ctx->IV));
    memcpy(ctx->buffers.cipher_text_gcm_aead_receiver + sizeof(ctx->IV),
           ctx->buffers.cipher_text_bytes,
           ctx->tmp.computed_size);

    retval = cmox_mac_compute(
                CMOX_HMAC_SHA256_ALGO,
                ctx->buffers.cipher_text_gcm_aead_receiver, mac_input_len,
                ctx->derived.mac_key_receiver, sizeof(ctx->derived.mac_key_receiver),
                NULL, 0,
                mac_out, mac_out_len, out_mac_len
             );

    secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, mac_input_len);

    if (retval != CMOX_MAC_SUCCESS) {
        secure_zero(mac_out, mac_out_len);
        *out_mac_len = 0;
        return HMAC_IVC_ERR_MAC_FAILED;
    }

    return HMAC_IVC_SUCCESS;
}

/* Prepares the IV state for the next transfer. */
gen_iv_status_t generate_nonce_and_iv_per_transfer(
    crypto_ctx_t *ctx)
{
    /* Stop immediately if a nonce or IV preparation step fails. */
    gcm_nonce_status_t nonce_ret;

    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }

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

/* Copies the DRBG IV prefix into the GCM nonce buffer. */
gcm_master_status_t derive_gcm_iv_from_drbg(crypto_ctx_t *ctx)
{
    /* Reuse the existing IV bytes rather than generating another DRBG output. */
    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }

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
/* Initializes the sender ECDH secret and derived keys. */
sender_init_status_t sender_init_ecdh_crypto(crypto_ctx_t *ctx)
{
    /* Clear affected key and secret buffers when an initialization stage fails. */
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

/* Initializes the receiver ECDH secret and derived keys. */
receiver_init_status_t receiver_init_ecdh_crypto(
    crypto_ctx_t *ctx)
{
    /* Validate the peer public key before deriving the receive keys. */
    int ret;

    if (ctx == NULL) {
        return RECEIVER_INIT_ERR_BAD_PARAM;
    }

#if !ECDH_PROVISIONED_KEYS_PRESENT

    log_printf(
        "[ERROR] receiver_init_ecdh_crypto: "
        "provisioned reverse-direction keys are required\r\n");

    return RECEIVER_INIT_ERR_PUBKEY;

#else

    /*
     * Validate the actual peer public key used for the reverse
     * direction. Do not validate ctx->keys.public_key_sender here;
     * that field is the local STM32 sender public key.
     */
    ret =
        validate_public_key_sender_format_ctx(
            ctx,
            esp32_provisioned_sender_public_key,
            sizeof(esp32_provisioned_sender_public_key));

    if (ret != PUBKEY_OK) {
        log_printf(
            "[ERROR] receiver_init_ecdh_crypto: "
            "ESP32 sender public key is invalid (rc=%d)\r\n",
            ret);

        return RECEIVER_INIT_ERR_PUBKEY;
    }

    ret =
        compute_ecdh_shared_as_receiver(
            ctx,
            NULL,
            NULL);

    if (ret != ECDH_SUCCESS) {
        log_printf(
            "[ERROR] receiver_init_ecdh_crypto: "
            "reverse-direction ECDH failed (rc=%d)\r\n",
            ret);

        secure_zero(
            ctx->tmp.computed_secret_receiver,
            sizeof(ctx->tmp.computed_secret_receiver));

        ctx->tmp.computed_secret_receiver_len = 0U;

        return RECEIVER_INIT_ERR_ECDH;
    }

    ret =
        derive_keys_from_shared_secret(
            ctx,
            true);

    if (ret != KDF_SUCCESS) {
        log_printf(
            "[ERROR] receiver_init_ecdh_crypto: "
            "receiver KDF failed (rc=%d)\r\n",
            ret);

        secure_zero(
            ctx->tmp.computed_secret_receiver,
            sizeof(ctx->tmp.computed_secret_receiver));

        secure_zero(
            ctx->derived.aes_key_receiver,
            sizeof(ctx->derived.aes_key_receiver));

        secure_zero(
            ctx->derived.mac_key_receiver,
            sizeof(ctx->derived.mac_key_receiver));

        ctx->tmp.computed_secret_receiver_len = 0U;

        return RECEIVER_INIT_ERR_KDF;
    }

    log_printf(
        "[INFO] receiver_init_ecdh_crypto: "
        "ESP32->STM32 RX key ready (shared_len=%u)\r\n",
        (unsigned)ctx->tmp.computed_secret_receiver_len);

    log_printf(
        "[INFO] Receiver KDF compatibility: "
        "AES=SHA256(shared||0x01), "
        "MAC=SHA256(shared||0x02)\r\n");

#if ECDH_DEBUG_PRINT_RX_DERIVED_MATERIAL

    log_printf(
        "[SECURITY WARNING] Printing reverse-direction development "
        "key material\r\n");

    log_hex_buffer(
        "[ECDH][RX][SHARED SECRET]",
        ctx->tmp.computed_secret_receiver,
        ctx->tmp.computed_secret_receiver_len);

    log_hex_buffer(
        "[ECDH][RX][AES KEY]",
        ctx->derived.aes_key_receiver,
        sizeof(ctx->derived.aes_key_receiver));

    log_hex_buffer(
        "[ECDH][RX][MAC KEY]",
        ctx->derived.mac_key_receiver,
        sizeof(ctx->derived.mac_key_receiver));

#endif /* ECDH_DEBUG_PRINT_RX_DERIVED_MATERIAL */

    return RECEIVER_INIT_SUCCESS;

#endif /* ECDH_PROVISIONED_KEYS_PRESENT */
}

/* Prepares an IV and encrypts and authenticates the sender payload. */
hmac_sender_status_t generate_nonce_iv_encrypt_hmac_sender(crypto_ctx_t *ctx)
{
    /* Complete AES-CTR encryption before calculating the IV-and-ciphertext HMAC. */
    gen_iv_status_t     iv_rc;
    aes_ctr_enc_status_t enc_rc;
    hmac_ivc_status_t   hmac_rc;

    if (ctx == NULL) {
        return HMAC_SENDER_ERR_BAD_PARAM;
    }

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

/* Compares byte buffers without an early exit. */
static int ct_memcmp(const void *a_, const void *b_, size_t n)
{
    /* Accumulate every byte difference before returning the comparison result. */
    const uint8_t *a = (const uint8_t *)a_;
    const uint8_t *b = (const uint8_t *)b_;
    uint8_t diff = 0;
    size_t i;
    for (i = 0; i < n; ++i) {
        diff |= a[i] ^ b[i];
    }
    return (diff != 0);
}

/* Authenticates and decrypts the received CTR payload. */
verify_dec_status_t verify_mac_decrypt_recombine_receiver(crypto_ctx_t *ctx)
{
    /* Verify the HMAC before decrypting and reconstructing ADC values. */
    hmac_ivc_status_t    hmac_rc;
    aes_ctr_dec_status_t dec_rc;
    size_t               dec_len = 0;
    size_t               count, i;

    if (ctx == NULL) {
        return VERIFY_DEC_ERR_BAD_PARAM;
    }

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

    if (dec_len != ctx->tmp.computed_size) {
        log_printf(
            "[ERROR] verify_mac_decrypt_recombine_receiver: length mismatch (dec_len=%zu computed_size=%zu)\r\n",
            dec_len, ctx->tmp.computed_size
        );
        secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));
        secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
        return VERIFY_DEC_ERR_LENGTH_MISMATCH;
    }

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

    secure_zero(ctx->auth.mac_check, sizeof(ctx->auth.mac_check));

    return VERIFY_DEC_SUCCESS;
}

/* Clears the recovered ADC values and checks the result. */
secure_clear_status_t secure_clear_decrypted_adc_values(crypto_ctx_t *ctx)
{
    /* Verify that the volatile clearing path leaves each ADC word zero. */
    size_t count;
    size_t total_bytes;
    volatile uint8_t *p;
    size_t i;

    if (ctx == NULL) {
        return SECURE_CLEAR_ERR_INVALID_SIZE;
    }

    count = ctx->cfg.adc_sample_count;
    if (count == 0U) {
        return SECURE_CLEAR_ERR_INVALID_SIZE;
    }

    total_bytes = count * sizeof(ctx->buffers.decrypted_adc_values[0]);
    if (total_bytes / sizeof(ctx->buffers.decrypted_adc_values[0]) != count) {
        return SECURE_CLEAR_ERR_INVALID_SIZE;
    }

    secure_zero(ctx->buffers.decrypted_adc_values, total_bytes);

    __asm__ volatile ("" ::: "memory");

    p = (volatile uint8_t *)ctx->buffers.decrypted_adc_values;
    for (i = 0; i < total_bytes; ++i) {
        if (p[i] != 0U) {
            return SECURE_CLEAR_ERR_VERIFY_FAIL;
        }
    }

    return SECURE_CLEAR_SUCCESS;
}

/* --- Streaming-style GCM encrypt (no mac[]) --- */
/* Encrypts the payload through the streaming AES-GCM API. */
gcm_enc_status_t encrypt_aes_gcm_aead(crypto_ctx_t *ctx)
{
    /* Append the authentication tag after the ciphertext and clean up the cipher handle. */
    cmox_cipher_retval_t cres;
    size_t local_out_len = 0;
    size_t tag_len = 0;
    size_t plain_len = ctx->tmp.plain_len;

    if (plain_len == 0) {
        return GCM_ENC_ERR_BAD_INPUT;
    }

    if (plain_len > (sizeof(ctx->buffers.cipher_text_gcm_aead) - AES_GCM_TAG_SIZE)) {
        return GCM_ENC_ERR_BAD_INPUT;
    }

    if (is_all_zero_bytes(ctx->derived.aes_key, sizeof(ctx->derived.aes_key)) ||
        is_all_zero_bytes(ctx->IV_GCM, sizeof(ctx->IV_GCM))) {
        return GCM_ENC_ERR_KEY_OR_IV;
    }

    ctx->tmp.computed_size = 0;  

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

    assert(sizeof(ctx->IV_GCM) == AES_GCM_AEAD_IV_SIZE);  

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
                              ctx->buffers.cipher_text_gcm_aead,  
                              &local_out_len);
    if (cres != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_APPEND;
    }

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

    ctx->tmp.computed_size = local_out_len;

    /* --- cleanup internal handle state --- */
    cres = cmox_cipher_cleanup(h);
    if (cres != CMOX_CIPHER_SUCCESS) {
        /* cleanup failed; wipe outputs and report */
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        ctx->tmp.computed_size = 0;
        return GCM_ENC_ERR_CLEANUP;
    }

    secure_zero(ctx->tmp.local_tag, sizeof(ctx->tmp.local_tag));

    return GCM_ENC_SUCCESS;
}

/* --- Single-call AEAD encrypt (no mac[]) --- */
/* Encrypts and tags a payload through the single-call GCM API. */
gcm_aead_single_status_t encrypt_aead_gcm_singlecall(crypto_ctx_t *ctx)
{
    /* Use the configured GCM IV and sender AES key for the combined output. */
    cmox_cipher_retval_t cres;
    size_t aead_out_len = 0;
    size_t expected_out_len;
    const size_t tag_len = (size_t)AES_GCM_TAG_SIZE;
    size_t plain_len = ctx->tmp.plain_len;

    if (plain_len == 0) {
        return GCM_AEAD_SINGLE_ERR_BAD_INPUT;
    }

    if (plain_len > (sizeof(ctx->buffers.cipher_text_gcm_aead) - tag_len)) {
        return GCM_AEAD_SINGLE_ERR_BAD_INPUT;
    }

    if (is_all_zero_bytes(ctx->derived.aes_key, sizeof(ctx->derived.aes_key)) ||
        is_all_zero_bytes(ctx->IV_GCM, sizeof(ctx->IV_GCM))) {
        return GCM_AEAD_SINGLE_ERR_KEY_OR_IV;
    }

    ctx->tmp.computed_size = 0;

    cres = cmox_aead_encrypt(
                CMOX_AESFAST_GCMFAST_ENC_ALGO,                  
                ctx->buffers.plain_text_bytes,                  
                plain_len,                                      
                tag_len,                                        
                ctx->derived.aes_key,                           
                (cmox_cipher_keyLen_t)sizeof(ctx->derived.aes_key),  
                ctx->IV_GCM,                                    
                sizeof(ctx->IV_GCM),                            
                NULL,                                           
                0,                                              
                ctx->buffers.cipher_text_gcm_aead,             
                &aead_out_len                                    
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

    ctx->tmp.computed_size = plain_len;

    memcpy(ctx->auth.ciphertext_sender,
           ctx->buffers.cipher_text_gcm_aead,
           ctx->tmp.computed_size);
    ctx->auth.ciphertext_len = ctx->tmp.computed_size;

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

/* Prepares a GCM nonce and the authenticated sender payload. */
gcm_sender_status_t generate_nonce_iv_encrypt_gcm_sender(
    crypto_ctx_t *ctx)
{
    /* Clear IV state if an encryption step fails. */
    gen_iv_status_t iv_rc;
    gcm_enc_status_t gcm_rc;
    size_t ciphertext_len;

    if (ctx == NULL) {
        return GCM_SENDER_ERR_BAD_PARAM;
    }

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

    memcpy(ctx->auth.ciphertext_sender,
           ctx->buffers.cipher_text_gcm_aead,
           ciphertext_len);

    ctx->auth.ciphertext_len = ciphertext_len;

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
            "[TX][UART PLAINTEXT PACKET]",
            ctx->buffers.plain_text_bytes,
            ctx->tmp.plain_len
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

/* Decrypts a GCM payload and verifies its authentication tag. */
gcm_dec_status_t decrypt_verify_aes_gcm_receiver(
    crypto_ctx_t *ctx)
{
    /* Clear recovered plaintext when tag verification or cipher cleanup fails. */
    cmox_cipher_retval_t cres;
    cmox_cipher_retval_t cleanup_ret;
    cmox_gcm_handle_t gcm_ctx;
    cmox_cipher_handle_t *h;
    uint32_t fault_check = 0U;
    size_t out_len = 0U;

    if (ctx == NULL) {
        return GCM_DEC_ERR_BAD_INPUT;
    }

    ctx->tmp.computed_size = 0U;

    if ((ctx->auth.ciphertext_len == 0U) ||
        (ctx->auth.ciphertext_len >
            sizeof(ctx->auth.ciphertext_sender)) ||
        (ctx->auth.ciphertext_len >
            sizeof(ctx->buffers.decrypted_bytes)) ||
        (ctx->auth.mac_len != AES_GCM_TAG_SIZE)) {
        return GCM_DEC_ERR_BAD_INPUT;
    }

    if (is_all_zero_bytes(
            ctx->derived.aes_key_receiver,
            sizeof(ctx->derived.aes_key_receiver)) ||
        is_all_zero_bytes(
            ctx->IV_GCM_receiver,
            sizeof(ctx->IV_GCM_receiver))) {
        return GCM_DEC_ERR_KEY_OR_IV;
    }

    secure_zero(
        ctx->buffers.decrypted_bytes,
        sizeof(ctx->buffers.decrypted_bytes));

    h =
        cmox_gcm_construct(
            &gcm_ctx,
            CMOX_AESFAST_GCMFAST_DEC);

    if (h == NULL) {
        return GCM_DEC_ERR_CONSTRUCT;
    }

    cres = cmox_cipher_init(h);

    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_INIT;
    }

    cres =
        cmox_cipher_setKey(
            h,
            ctx->derived.aes_key_receiver,
            (cmox_cipher_keyLen_t)
                sizeof(ctx->derived.aes_key_receiver));

    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETKEY;
    }

    cres =
        cmox_cipher_setIV(
            h,
            ctx->IV_GCM_receiver,
            sizeof(ctx->IV_GCM_receiver));

    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETIV;
    }

    cres =
        cmox_cipher_setTagLen(
            h,
            (size_t)AES_GCM_TAG_SIZE);

    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETTAGLEN;
    }

    cres =
        cmox_cipher_append(
            h,
            ctx->auth.ciphertext_sender,
            ctx->auth.ciphertext_len,
            ctx->buffers.decrypted_bytes,
            &out_len);

    if (cres != CMOX_CIPHER_SUCCESS) {
        secure_zero(
            ctx->buffers.decrypted_bytes,
            sizeof(ctx->buffers.decrypted_bytes));

        (void)cmox_cipher_cleanup(h);

        return GCM_DEC_ERR_APPEND;
    }

    cres =
        cmox_cipher_verifyTag(
            h,
            ctx->auth.tag_sender,
            &fault_check);

    cleanup_ret =
        cmox_cipher_cleanup(h);

    if ((cres != CMOX_CIPHER_AUTH_SUCCESS) ||
        (fault_check != CMOX_CIPHER_AUTH_SUCCESS)) {
        secure_zero(
            ctx->buffers.decrypted_bytes,
            sizeof(ctx->buffers.decrypted_bytes));

        return GCM_DEC_ERR_VERIFY;
    }

    if (cleanup_ret != CMOX_CIPHER_SUCCESS) {
        secure_zero(
            ctx->buffers.decrypted_bytes,
            sizeof(ctx->buffers.decrypted_bytes));

        return GCM_DEC_ERR_CLEANUP;
    }

    if (out_len != ctx->auth.ciphertext_len) {
        secure_zero(
            ctx->buffers.decrypted_bytes,
            sizeof(ctx->buffers.decrypted_bytes));

        return GCM_DEC_ERR_APPEND;
    }

    ctx->tmp.computed_size = out_len;

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

/* Authenticates and decrypts through the single-call GCM API. */
gcm_aead_dec_status_t decrypt_aead_gcm_singlecall(crypto_ctx_t *ctx)
{
    /* Reject an unsuccessful authentication result before accepting plaintext. */
    cmox_cipher_retval_t cres;
    size_t in_len = 0;
    size_t plain_len_local = 0;
    const size_t tag_len = (size_t)AES_GCM_TAG_SIZE;

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

    secure_zero(ctx->buffers.decrypted_bytes,
                sizeof(ctx->buffers.decrypted_bytes));

    in_len = ctx->auth.ciphertext_len + tag_len;

    /* --- Single-call AEAD decrypt --- */
    cres = cmox_aead_decrypt(
                CMOX_AESFAST_GCMFAST_DEC_ALGO,           
                ctx->buffers.cipher_text_gcm_aead_receiver,  
                in_len,                                  
                tag_len,                                 
                ctx->derived.aes_key_receiver,           
                (cmox_cipher_keyLen_t)
                    sizeof(ctx->derived.aes_key_receiver),
                ctx->IV_GCM,                              
                sizeof(ctx->IV_GCM),                      
                NULL,                                    
                0,                                       
                ctx->buffers.decrypted_bytes,             
                &plain_len_local                          
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

    secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, in_len);

    return GCM_AEAD_DEC_SUCCESS;
}

/*
 * Encrypt one complete UART plaintext frame for the STM32 -> ESP32
 * startup test.
 *
 * Input:
 *     uart_payload[0..uart_payload_len-1]
 *
 * Output:
 *     nonce_out[12]
 *     ciphertext_and_tag_out =
 *         ciphertext[uart_payload_len] || tag[16]
 *
 * The payload is copied into the existing crypto context so the
 * established C-MOX streaming AES-GCM implementation remains the
 * single encryption implementation.
 */
/* Encrypts one UART payload and exports its GCM nonce and tag. */
gcm_sender_status_t ecdh_encrypt_uart_payload_once(
    crypto_ctx_t *ctx,
    const uint8_t *uart_payload,
    size_t uart_payload_len,
    uint8_t nonce_out[AES_GCM_AEAD_IV_SIZE],
    uint8_t *ciphertext_and_tag_out,
    size_t output_capacity,
    size_t *output_len)
{
    /* Check payload and output capacities before clearing and filling the crypto buffers. */
    gcm_sender_status_t status;
    size_t combined_len;

    if (output_len != NULL) {
        *output_len = 0U;
    }

    if ((ctx == NULL) ||
        (uart_payload == NULL) ||
        (nonce_out == NULL) ||
        (ciphertext_and_tag_out == NULL) ||
        (output_len == NULL) ||
        (uart_payload_len == 0U) ||
        (uart_payload_len > sizeof(ctx->buffers.plain_text_bytes)) ||
        (uart_payload_len >
            (sizeof(ctx->buffers.cipher_text_gcm_aead) -
             AES_GCM_TAG_SIZE))) {
        return GCM_SENDER_ERR_BAD_PARAM;
    }

    combined_len =
        uart_payload_len +
        AES_GCM_TAG_SIZE;

    if (output_capacity < combined_len) {
        return GCM_SENDER_ERR_BAD_PARAM;
    }

    secure_zero(ctx->buffers.plain_text_bytes,
                sizeof(ctx->buffers.plain_text_bytes));

    secure_zero(ctx->buffers.cipher_text_gcm_aead,
                sizeof(ctx->buffers.cipher_text_gcm_aead));

    secure_zero(ctx->auth.ciphertext_sender,
                sizeof(ctx->auth.ciphertext_sender));

    secure_zero(ctx->auth.tag_sender,
                sizeof(ctx->auth.tag_sender));

    secure_zero(nonce_out,
                AES_GCM_AEAD_IV_SIZE);

    secure_zero(ciphertext_and_tag_out,
                output_capacity);

    memcpy(ctx->buffers.plain_text_bytes,
           uart_payload,
           uart_payload_len);

    ctx->tmp.plain_len = uart_payload_len;
    ctx->tmp.computed_size = 0U;
    ctx->auth.ciphertext_len = 0U;
    ctx->auth.mac_len = 0U;

    status =
        generate_nonce_iv_encrypt_gcm_sender(ctx);

    if (status != GCM_SENDER_SUCCESS) {
        secure_zero(ctx->buffers.plain_text_bytes,
                    sizeof(ctx->buffers.plain_text_bytes));

        secure_zero(nonce_out,
                    AES_GCM_AEAD_IV_SIZE);

        secure_zero(ciphertext_and_tag_out,
                    output_capacity);

        return status;
    }

    if ((ctx->auth.ciphertext_len != uart_payload_len) ||
        (ctx->auth.mac_len != AES_GCM_TAG_SIZE) ||
        ((ctx->auth.ciphertext_len +
          ctx->auth.mac_len) != combined_len)) {
        secure_zero(nonce_out,
                    AES_GCM_AEAD_IV_SIZE);

        secure_zero(ciphertext_and_tag_out,
                    output_capacity);

        return GCM_SENDER_ERR_GCM_STREAM;
    }

    memcpy(nonce_out,
           ctx->IV_GCM,
           AES_GCM_AEAD_IV_SIZE);

    memcpy(ciphertext_and_tag_out,
           ctx->buffers.cipher_text_gcm_aead,
           combined_len);

    *output_len = combined_len;

    return GCM_SENDER_SUCCESS;
}

/* Calculates the encrypted reply's additive byte checksum. */
static uint8_t gcm_uart_checksum8(
    const uint8_t *data,
    size_t len)
{
    /* Accumulate the sum with eight-bit wraparound. */
    uint8_t sum = 0U;

    if (data == NULL) {
        return 0U;
    }

    for (size_t i = 0U; i < len; ++i) {
        sum =
            (uint8_t)(
                sum +
                data[i]);
    }

    return sum;
}

/* Authenticates and validates the encrypted ESP32 ACK or NACK reply. */
gcm_receiver_status_t decrypt_verify_recombine_gcm_receiver(
    crypto_ctx_t *ctx,
    uint32_t expected_sequence,
    gcm_uart_response_t *response_out)
{
    /* Verify the GCM tag before interpreting reply fields or replay state. */
    const uint8_t *plaintext;
    uint8_t calculated_checksum;
    uint8_t received_checksum;
    uint8_t message_type;
    uint8_t reason;
    uint32_t received_sequence;
    uint32_t session_id;
    uint64_t counter;
    gcm_dec_status_t gcm_rc;

    if ((ctx == NULL) ||
        (response_out == NULL)) {
        return GCM_RECEIVER_ERR_BAD_PARAM;
    }

    memset(
        response_out,
        0,
        sizeof(*response_out));

    if ((ctx->auth.ciphertext_len !=
            GCM_UART_RESPONSE_CIPHERTEXT_SIZE) ||
        (ctx->auth.mac_len != AES_GCM_TAG_SIZE)) {
        log_printf(
            "[RX][ERROR] Invalid encrypted ACK/NACK lengths: "
            "ciphertext=%u tag=%u\r\n",
            (unsigned)ctx->auth.ciphertext_len,
            (unsigned)ctx->auth.mac_len);

        return GCM_RECEIVER_ERR_FRAME_LENGTH;
    }

    gcm_rc =
        decrypt_verify_aes_gcm_receiver(ctx);

    if (gcm_rc != GCM_DEC_SUCCESS) {
        log_printf(
            "[RX][ERROR] ESP32 ACK/NACK GCM authentication failed "
            "(rc=%d)\r\n",
            gcm_rc);

        secure_zero(
            ctx->buffers.decrypted_bytes,
            sizeof(ctx->buffers.decrypted_bytes));

        ctx->tmp.computed_size = 0U;

        return GCM_RECEIVER_ERR_GCM_STREAM;
    }

    if (ctx->tmp.computed_size !=
        GCM_UART_RESPONSE_PLAINTEXT_SIZE) {
        log_printf(
            "[RX][ERROR] Decrypted ACK/NACK size=%u expected=%u\r\n",
            (unsigned)ctx->tmp.computed_size,
            (unsigned)GCM_UART_RESPONSE_PLAINTEXT_SIZE);

        secure_zero(
            ctx->buffers.decrypted_bytes,
            sizeof(ctx->buffers.decrypted_bytes));

        return GCM_RECEIVER_ERR_FRAME_LENGTH;
    }

    plaintext =
        ctx->buffers.decrypted_bytes;

    calculated_checksum =
        gcm_uart_checksum8(
            plaintext,
            GCM_UART_RESPONSE_PLAINTEXT_SIZE - 1U);

    received_checksum =
        plaintext[
            GCM_UART_RESPONSE_PLAINTEXT_SIZE - 1U];

    if (calculated_checksum != received_checksum) {
        log_printf(
            "[RX][ERROR] Authenticated ACK/NACK checksum invalid: "
            "calculated=0x%02X received=0x%02X\r\n",
            calculated_checksum,
            received_checksum);

        return GCM_RECEIVER_ERR_FRAME_CHECKSUM;
    }

    if ((plaintext[0] != GCM_UART_MAGIC_0) ||
        (plaintext[1] != GCM_UART_MAGIC_1) ||
        (plaintext[2] != GCM_UART_PROTOCOL_VERSION) ||
        (plaintext[4] != GCM_UART_RESPONSE_PAYLOAD_LEN)) {
        log_printf(
            "[RX][ERROR] Authenticated ACK/NACK header invalid\r\n");

        return GCM_RECEIVER_ERR_FRAME_HEADER;
    }

    message_type =
        plaintext[3];

    received_sequence =
        ((uint32_t)plaintext[5] << 0U)  |
        ((uint32_t)plaintext[6] << 8U)  |
        ((uint32_t)plaintext[7] << 16U) |
        ((uint32_t)plaintext[8] << 24U);

    reason =
        plaintext[9];

    if (received_sequence != expected_sequence) {
        log_printf(
            "[RX][ERROR] Authenticated stale response: "
            "expected_seq=%lu received_seq=%lu\r\n",
            (unsigned long)expected_sequence,
            (unsigned long)received_sequence);

        return GCM_RECEIVER_ERR_STALE_SEQUENCE;
    }

    if ((message_type != GCM_UART_MSG_TYPE_ACK) &&
        (message_type != GCM_UART_MSG_TYPE_NACK)) {
        log_printf(
            "[RX][ERROR] Authenticated response type invalid: "
            "0x%02X\r\n",
            message_type);

        return GCM_RECEIVER_ERR_FRAME_TYPE;
    }

    if ((message_type == GCM_UART_MSG_TYPE_ACK) &&
        (reason != 0U)) {
        log_printf(
            "[RX][ERROR] ACK carried nonzero reason=%u\r\n",
            (unsigned)reason);

        return GCM_RECEIVER_ERR_FRAME_TYPE;
    }

    response_out->message_type =
        message_type;

    response_out->reason =
        reason;

    response_out->sequence =
        received_sequence;

    session_id =
        load_be32(
            &ctx->IV_GCM_receiver[0]);

    counter =
        load_be64(
            &ctx->IV_GCM_receiver[4]);

#if ECDH_DEBUG_PRINT_RX_DECRYPTED_PACKET

    log_printf(
        "\r\n"
        "========== ESP32 -> STM32 DECRYPTED ACK/NACK ==========\r\n");

    log_printf(
        "[RX][META] session=%08lX counter=%08lX%08lX\r\n",
        (unsigned long)session_id,
        (unsigned long)(counter >> 32),
        (unsigned long)(
            counter &
            UINT64_C(0xFFFFFFFF)));

    log_hex_buffer(
        "[RX][NONCE IV_GCM]",
        ctx->IV_GCM_receiver,
        sizeof(ctx->IV_GCM_receiver));

    log_hex_buffer(
        "[RX][CIPHERTEXT ONLY]",
        ctx->auth.ciphertext_sender,
        ctx->auth.ciphertext_len);

    log_hex_buffer(
        "[RX][AUTHENTICATION TAG]",
        ctx->auth.tag_sender,
        ctx->auth.mac_len);

    log_hex_buffer(
        "[RX][RECOVERED PLAINTEXT ACK/NACK]",
        plaintext,
        ctx->tmp.computed_size);

    log_printf(
        "[RX][PARSED] type=0x%02X sequence=%lu reason=%u "
        "checksum=0x%02X\r\n",
        message_type,
        (unsigned long)received_sequence,
        (unsigned)reason,
        received_checksum);

    log_printf(
        "[RX][AUTHENTICATED] GCM tag verified before protocol "
        "validation\r\n");

    log_printf(
        "=======================================================\r\n");

#endif /* ECDH_DEBUG_PRINT_RX_DECRYPTED_PACKET */

    return GCM_RECEIVER_SUCCESS;
}

/* Reconstructs ADC words from decrypted bytes. */
recombine_status_t recombine_adc_values(crypto_ctx_t *ctx)
{
    /* Combine each four-byte group in the serialization order. */
    size_t i;
    size_t bps;

    if (ctx == NULL) {
        return RECOMBINE_ERR_NULL_CTX;
    }

    bps = ctx->cfg.bytes_per_sample;

    if (bps != sizeof(uint32_t)) {
        return RECOMBINE_ERR_BAD_CONFIG;
    }

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

/* Initializes the crypto context buffers and configuration. */
void crypto_ctx_init(crypto_ctx_t *ctx)
{
    /* Set the sample dimensions and reset payload, key and authentication state. */
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
        ctx->IV_GCM_receiver[i] = 0U;
    }
}

/* Initializes the transmit GCM session and counter state. */
gcm_nonce_status_t gcm_nonce_session_init(crypto_ctx_t *ctx)
{
    /* Include the startup nonce hash and directional public keys in the session digest. */
    static const uint8_t domain_label[] =
    {
        'S', 'T', 'M', '3', '2',
        '-', 'T', 'X',
        '-', 'E', 'S', 'P', '3', '2',
        '-', 'G', 'C', 'M',
        '-', 'V', '1'
    };

    uint8_t material[
        sizeof(domain_label) +
        1U + AES_512_KEY_SIZE +
        1U + AES_512_KEY_SIZE +
        1U + sizeof(nonce_hashed)
    ];

    uint8_t digest[32];
    size_t material_len = 0U;
    size_t digest_len = sizeof(digest);
    uint32_t session_id = 0U;
    cmox_hash_retval_t hash_ret;

    if (ctx == NULL) {
        return GCM_NONCE_ERR_BAD_PARAM;
    }

#if !ECDH_PROVISIONED_KEYS_PRESENT
    return GCM_NONCE_ERR_BAD_KEY_STATE;
#else

    if ((ctx->keys.pub_len_sender != AES_512_KEY_SIZE) ||
        is_all_zero_buf(ctx,
                        ctx->keys.public_key_sender,
                        sizeof(ctx->keys.public_key_sender)) ||
        is_all_zero_buf(ctx,
                        esp32_provisioned_receiver_public_key,
                        sizeof(esp32_provisioned_receiver_public_key)) ||
        is_all_zero_buf(ctx,
                        nonce_hashed,
                        sizeof(nonce_hashed))) {
        return GCM_NONCE_ERR_BAD_KEY_STATE;
    }

    secure_zero(material, sizeof(material));
    secure_zero(digest, sizeof(digest));

    memcpy(&material[material_len],
           domain_label,
           sizeof(domain_label));
    material_len += sizeof(domain_label);

    material[material_len++] =
        (uint8_t)ctx->keys.pub_len_sender;

    memcpy(&material[material_len],
           ctx->keys.public_key_sender,
           ctx->keys.pub_len_sender);
    material_len += ctx->keys.pub_len_sender;

    material[material_len++] =
        (uint8_t)sizeof(esp32_provisioned_receiver_public_key);

    memcpy(&material[material_len],
           esp32_provisioned_receiver_public_key,
           sizeof(esp32_provisioned_receiver_public_key));
    material_len += sizeof(esp32_provisioned_receiver_public_key);

    material[material_len++] =
        (uint8_t)sizeof(nonce_hashed);

    memcpy(&material[material_len],
           nonce_hashed,
           sizeof(nonce_hashed));
    material_len += sizeof(nonce_hashed);

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
    ctx->gcm_nonce.current_counter = UINT64_C(0);
    ctx->gcm_nonce.initialized = true;

    secure_zero(ctx->IV_GCM, sizeof(ctx->IV_GCM));

    log_printf(
        "[NONCE][TX] New STM32->ESP32 GCM session: "
        "session_id=%08lX first_counter=%08lX%08lX\r\n",
        (unsigned long)ctx->gcm_nonce.session_id,
        (unsigned long)(ctx->gcm_nonce.next_counter >> 32),
        (unsigned long)(
            ctx->gcm_nonce.next_counter &
            UINT64_C(0xFFFFFFFF))
    );

    return GCM_NONCE_OK;

#endif /* ECDH_PROVISIONED_KEYS_PRESENT */
}

/* Allocates the next session-and-counter GCM nonce. */
gcm_nonce_status_t gcm_nonce_allocate(
    crypto_ctx_t *ctx)
{
    /* Reject counter exhaustion before encoding the nonce in big-endian order. */
    uint64_t counter;

    if (ctx == NULL) {
        return GCM_NONCE_ERR_BAD_PARAM;
    }

    if (!ctx->gcm_nonce.initialized) {
        return GCM_NONCE_ERR_NO_SESSION;
    }

    if (ctx->gcm_nonce.next_counter == UINT64_MAX) {
        return GCM_NONCE_ERR_EXHAUSTED;
    }

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

