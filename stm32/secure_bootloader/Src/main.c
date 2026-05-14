/* Author: i fiksuari pas prog | Documentation date: 14 May 2026 */
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "fpu.h"
#include "uart.h"
#include "cmox_crypto.h"
#include <string.h>
#include "ecdh.h"
#include "DMA.h"
#include <entropy_personalization_nonce_gen.h>
#include "tim.h"

cmox_cipher_retval_t retval;
int32_t ret = 0, rc;
crypto_ctx_t g_crypto_ctx;



/* Initializes the platform and runs this cryptographic test sequence. */
int main(void)
{
    fpu_enable();
    uart2_tx_rx_init();
    adc_byte_buffers_init();

    /*
     * C-MOX must be initialized before any cmox_hash_compute(),
     * DRBG, ECC or AES-GCM operation.
     */
    if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS)
    {
        handle_error("cmox_initialize failed!");
    }

    rc = entropy_personalisation_HW_init();

    if (rc != ENTROPY_HW_INIT_SUCCESS)
    {
        log_printf(
            "main: entropy_personalisation_HW_init failed "
            "(rc=%d)\r\n",
            rc);

        handle_error("Entropy HW init failed!");
    }

    rc = entropy_personalisation_hash();

    if (rc != ENTROPY_CREATION_SUCCESS)
    {
        log_printf(
            "main: entropy_personalisation_hash failed "
            "(rc=%d)\r\n",
            rc);

        handle_error("Entropy personalisation hash failed!");
    }

    /*
     * Initialize the old nonce subsystem once.
     * This nonce is used only as DRBG/ECDH startup input.
     */
    nonce_status_t nonce_rc = nonce_simple_init();

    if (nonce_rc != NONCE_OK)
    {
        log_printf(
            "nonce_simple_init failed (rc=%d)\r\n",
            (int)nonce_rc);

        handle_error("Initialization of startup nonce failed!");
    }

    /*
     * Populate nonce_hashed once for the ECDH key-generation DRBG.
     */
    rc = nonce_generation();

    if (rc != NONCE_GEN_SUCCESS)
    {
        log_printf(
            "nonce_generation failed (rc=%d)\r\n",
            rc);

        handle_error("Startup DRBG nonce generation failed!");
    }

    crypto_ctx_init(&g_crypto_ctx);

    for (size_t i = 0U;
         i < g_crypto_ctx.cfg.adc_sample_count;
         ++i)
    {
        g_crypto_ctx.buffers.adc_values[i] = 1U;
    }

    ret = decompose_adc_values(&g_crypto_ctx);

    if (ret != DECOMP_ADC_OK)
    {
        handle_error("decompose_adc_values failed");
    }

    /*
     * Generate fresh sender keypair, calculate sender shared secret,
     * and derive the sender AES key.
     */
    ret = sender_init_ecdh_crypto(&g_crypto_ctx);

    if (ret != SENDER_INIT_SUCCESS)
    {
        handle_error("sender_init_ecdh_crypto failed");
    }

    /*
     * Generate fresh receiver keypair, calculate receiver shared secret,
     * and derive the receiver AES key.
     */
    ret = receiver_init_ecdh_crypto(&g_crypto_ctx);

    if (ret != RECEIVER_INIT_SUCCESS)
    {
        handle_error("receiver_init_ecdh_crypto failed");
    }

    /*
     * Both fresh ECDH public keys now exist.
     *
     * Initialize:
     * IV_GCM = 32-bit session ID || 64-bit packet counter
     */
    ret = gcm_nonce_session_init(&g_crypto_ctx);

    if (ret != GCM_NONCE_OK)
    {
        log_printf(
            "gcm_nonce_session_init failed (rc=%d)\r\n",
            ret);

        handle_error("GCM nonce session initialization failed");
    }

    log_printf(
        "\r\n"
        "[CRYPTO] Runtime AES-GCM test started. "
        "One packet will be processed approximately every 2 seconds.\r\n");

    for (;;)
    {
        /*
         * Allocate the next session+counter nonce and perform
         * one streaming AES-GCM encryption.
         *
         * The encryption function prints:
         * - original plaintext payload
         * - original ADC values
         * - nonce
         * - ciphertext
         * - authentication tag
         * - ciphertext || tag
         * - explicit encryption-success message
         */
        ret =
            generate_nonce_iv_encrypt_gcm_sender(
                &g_crypto_ctx);

        if (ret != GCM_SENDER_SUCCESS)
        {
            log_printf(
                "generate_nonce_iv_encrypt_gcm_sender failed "
                "(rc=%d)\r\n",
                ret);

            handle_error("AES-GCM sender processing failed");
        }

        /*
         * Local receiver self-test:
         * verify the authentication tag, decrypt the payload,
         * reconstruct ADC values and print the recovered data.
         */
        ret =
            decrypt_verify_recombine_gcm_receiver(
                &g_crypto_ctx);

        if (ret != GCM_RECEIVER_SUCCESS)
        {
            log_printf(
                "decrypt_verify_recombine_gcm_receiver failed "
                "(rc=%d)\r\n",
                ret);

            handle_error("AES-GCM receiver processing failed");
        }

        /*
         * Clear recovered plaintext and reconstructed ADC values
         * before processing the next logical packet.
         */
        ret =
            secure_clear_decrypted_adc_values(
                &g_crypto_ctx);

        if (ret != SECURE_CLEAR_SUCCESS)
        {
            log_printf(
                "secure_clear_decrypted_adc_values failed "
                "(rc=%d)\r\n",
                ret);

            handle_error("Failed to clear decrypted ADC values");
        }

        log_printf(
            "[CRYPTO] Packet cycle completed. "
            "Waiting approximately 2 seconds...\r\n");


    }
}
