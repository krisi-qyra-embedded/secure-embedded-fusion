/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */
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
#include "digital_signature.h"

//extern crypto_ctx_t g_crypto_ctx; /* declaration provided by ecdh.h, harmless if duplicated */

cmox_cipher_retval_t retval;
int32_t ret = 0, rc;
crypto_ctx_t g_crypto_ctx;
cmox_ecc_handle_t g_crypto_ctx_;
sign_manifest_status_t sret;
/* bring the global signed manifest */
size_t signature_len = sizeof(g_signed_manifest.signature);

/* Initializes the platform and runs this cryptographic test sequence. */
int main(void)
{
    fpu_enable();
    uart2_tx_rx_init();
    adc_byte_buffers_init();

    rc = entropy_personalisation_HW_init();
        if (rc == ENTROPY_HW_INIT_SUCCESS) {
            log_printf("main: entropy_personalisation_HW_init succeeded.\r\n");
        } else {
            log_printf("main: entropy_personalisation_HW_init failed (rc=%d)\r\n", rc);
            handle_error("Entropy HW init failed!");
            while (1) {
                __NOP();
            }
        }

        /* Run entropy hashing/personalisation pipeline */
        rc = entropy_personalisation_hash();
        if (rc == ENTROPY_CREATION_SUCCESS) {
            log_printf("main: entropy_personalisation_hash succeeded.\r\n");
        } else {
            log_printf("main: entropy_personalisation_hash failed (rc=%d)\r\n", rc);
            handle_error("Entropy personalisation hash failed!");
            while (1) {
                __NOP();
            }
        }

        /* Simple usage where failure is fatal for crypto operations */
        nonce_status_t s = nonce_simple_init();
        if (s != NONCE_OK) {
            log_printf("nonce_simple_init failed: %d\n", (int)s);
            /* Actions: halt, fallback to alternative entropy, or enter safe mode */
            handle_error("Initialization of Nonce failed!");
        }

        rc = nonce_generation();
        if (rc == NONCE_GEN_SUCCESS) {
           log_printf("some_init_routine_with_nonce: NONCE_GENERATION_INIT succeeded.\n");
           } else {
           log_printf("some_init_routine_with_nonce: NONCE_GENERATION_INIT failed (rc=%d)\n", rc);
           /* handle error or propagate */
          }

    	 /* Small delay for the initialisation*/
    	 for (volatile int i = 0; i < 2000000; i++) __NOP();
    	 crypto_ctx_init(&g_crypto_ctx);
    	 /* Fill ADC samples into the context buffer */
    	 for (size_t i = 0; i < g_crypto_ctx.cfg.adc_sample_count; ++i) {
    	    g_crypto_ctx.buffers.adc_values[i] = 1U;
    	  }
    	 /* Initialize cryptolib */
    	     if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS) {
    	         handle_error("cmox_initialize failed!");
    	     }

    /* 2) Serialize */
    //SENDER
    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    /* -------- Generate DRBG-based private keys -------- */
#if 0
    /* optional: DRBG-based private key generation */
    ret = generate_private_key(&g_crypto_ctx);
    if (ret != GEN_PRIV_SUCCESS) {
        handle_error("generate_private_key failed");
    }
#endif
        /* -------- Generate  keypairs (priv + pub) -------- */
    	/* platform init... clocks, RNG, crypto libs, peripherals, etc. */
    ret = generate_private_key_digital_signature(&g_sig_keys);
        if (ret != GEN_KEYS_SUCCESS) {
            handle_error("generate_private_key_digital_signature failed");
        }

        /* Call exactly like your other crypto function */
        sret = sign_manifest_and_write_sig(&g_sig_keys,
        								   g_sig_keys.sig.private_key,
										   g_sig_keys.sig.priv_len,
                                           &g_signed_manifest.manifest,
                                           g_signed_manifest.signature,
                                           &signature_len);
        if (sret != SIGN_MANIFEST_SUCCESS) {
                handle_error("sign_manifest_and_write_sig failed");
            }

        /* -------- NOW store to protected flash (AFTER data is ready) -------- */
            manifest_store_status_t store_ret = store_signed_manifest_protected(&g_signed_manifest);
            if (store_ret != MANIFEST_STORE_SUCCESS) {
                log_printf("[FATAL] Failed to store manifest (rc=%d)\r\n", store_ret);
                handle_error("Manifest storage failed");
            }
            log_printf("[SUCCESS] Secure boot manifest provisioned\r\n");

            /* load signed manifest (boot path) */
            manifest_store_status_t load_ret = load_signed_manifest(&g_signed_manifest);
            if (load_ret != MANIFEST_STORE_SUCCESS) {
                log_printf("[FATAL] Failed to load manifest (rc=%d)\r\n", load_ret);
                handle_error("Manifest load failed");
            }
            log_printf("[SUCCESS] Signed manifest loaded from flash\r\n");

    for(;;) {


    }
}
