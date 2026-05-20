/**
 * @file main.c
 * @brief Secure-boot provisioning, manifest verification and application handoff.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
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
/* Shared secure-boot crypto and verification state. */
cmox_cipher_retval_t retval;
int32_t ret = 0, rc;
crypto_ctx_t g_crypto_ctx;
cmox_ecc_handle_t g_crypto_ctx_;
sign_manifest_status_t sret;
otp_status_t oret;
verify_auth_status_t auth_rc;
verify_integrity_status_t integ_rc;
verify_auth_negtest_status_t negtest_rc;
/* Signature length follows the shared signed-manifest buffer. */
size_t signature_len = sizeof(g_signed_manifest.signature);
/**
 * @brief Run the secure-boot provisioning and verification sequence.
 * @return Not reached during the normal boot path.
 */
int main(void)
{
    fpu_enable();
    uart2_tx_rx_init();
    adc_byte_buffers_init();
//    rc = entropy_personalisation_HW_init();
//        if (rc == ENTROPY_HW_INIT_SUCCESS) {
//            log_printf("main: entropy_personalisation_HW_init succeeded.\r\n");
//        } else {
//            log_printf("main: entropy_personalisation_HW_init failed (rc=%d)\r\n", rc);
//            handle_error("Entropy HW init failed!");
//            while (1) {
//                __NOP();
//            }
//        }
//
//
//
//        /* Run entropy hashing/personalisation pipeline */
//        rc = entropy_personalisation_hash();
//        if (rc == ENTROPY_CREATION_SUCCESS) {
//            log_printf("main: entropy_personalisation_hash succeeded.\r\n");
//        } else {
//            log_printf("main: entropy_personalisation_hash failed (rc=%d)\r\n", rc);
//            handle_error("Entropy personalisation hash failed!");
//            while (1) {
//                __NOP();
//            }
//        }
//
//        /* Simple usage where failure is fatal for crypto operations */
//        nonce_status_t s = nonce_simple_init();
//        if (s != NONCE_OK) {
//            log_printf("nonce_simple_init failed: %d\n", (int)s);
//            /* Actions: halt, fallback to alternative entropy, or enter safe mode */
//            handle_error("Initialization of Nonce failed!");
//        }
//
//        rc = nonce_generation();
//        if (rc == NONCE_GEN_SUCCESS) {
//           log_printf("some_init_routine_with_nonce: NONCE_GENERATION_INIT succeeded.\n");
//           } else {
//           log_printf("some_init_routine_with_nonce: NONCE_GENERATION_INIT failed (rc=%d)\n", rc);
//           /* handle error or propagate */
//          }
    	 /* Short startup delay before crypto initialization. */
    	 for (volatile int i = 0; i < 2000000; i++) __NOP();
    	 crypto_ctx_init(&g_crypto_ctx);
//    	 /* Fill ADC samples into the context buffer */
//    	 for (size_t i = 0; i < g_crypto_ctx.cfg.adc_sample_count; ++i) {
//    	    g_crypto_ctx.buffers.adc_values[i] = 1U;
//    	  }
    	 /* Initialize the CMOX crypto library before secure-boot operations. */
    	     if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS) {
    	         handle_error("cmox_initialize failed!");
    	     }
    /* Secure-boot signing and verification flow. */
    //SENDER
    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    /* Optional DRBG private-key generation path remains disabled below. */
#if 0
    /* Generate a private key from the DRBG when this test path is enabled. */
    ret = generate_private_key(&g_crypto_ctx);
    if (ret != GEN_PRIV_SUCCESS) {
        handle_error("generate_private_key failed");
    }
#endif
        /* First boot provisions Ed25519 OTP; later boots load the keypair from OTP. */
    	/* Platform and crypto prerequisites are initialized before this point. */
//    ret = generate_private_key_digital_signature(&g_sig_keys);
//        if (ret != GEN_KEYS_SUCCESS) {
//            handle_error("generate_private_key_digital_signature failed");
//        }
    		ret = assign_private_and_public_key_from_hex(&g_sig_keys);
    			if (ret != SENDER_INIT_SUCCESS) {
    			handle_error("Ed25519 OTP provision/load failed");
    		}
    			/* Validate the loaded Ed25519 keypair before signing the manifest. */
    			ret = generate_ed25519_signing_keypair_once(&g_sig_keys);
    			if (ret != GEN_KEYS_SUCCESS) {
                    secure_zero(g_sig_keys.sig.private_key,
                                sizeof(g_sig_keys.sig.private_key));
                    secure_zero(g_sig_keys.sig.public_key,
                                sizeof(g_sig_keys.sig.public_key));
                    g_sig_keys.sig.priv_len = 0U;
                    g_sig_keys.sig.pub_len  = 0U;
    			    handle_error("generate_ed25519_signing_keypair_once failed");
    			}
        /*ret =   debug_print_ed25519_keys(&g_sig_keys);
        if (ret != SUCCESS_PRINT_KEYS) {
        	handle_error("debug_print_ed25519_keys  failed");
        }*/
        /*
         * assign_private_and_public_key_from_hex() first provisions the exact
         * supplied keypair if OTP is erased, then loads the keypair FROM OTP
         * into g_sig_keys only for the signing operation below.
         */
        /* Sign the manifest with the OTP-backed Ed25519 private key. */
        sret = sign_manifest_and_write_sig(&g_sig_keys,
        								   g_sig_keys.sig.private_key,
										   g_sig_keys.sig.priv_len,
                                           &g_signed_manifest.manifest,
                                           g_signed_manifest.signature,
                                           &signature_len);
        if (sret != SIGN_MANIFEST_SUCCESS) {
                secure_zero(g_sig_keys.sig.private_key,
                            sizeof(g_sig_keys.sig.private_key));
                secure_zero(g_sig_keys.sig.public_key,
                            sizeof(g_sig_keys.sig.public_key));
                g_sig_keys.sig.priv_len = 0U;
                g_sig_keys.sig.pub_len  = 0U;
                handle_error("sign_manifest_and_write_sig failed");
            }
        /*
         * The Ed25519 signing operation is finished.
         * Remove BOTH OTP-derived key copies from RAM immediately.
         *
         * Authenticity verification below does not need these RAM copies:
         * verify_manifest_authenticity_from_flash() reads the public trust
         * anchor directly from OTP into a short-lived local buffer.
         */
        secure_zero(g_sig_keys.sig.private_key,
                    sizeof(g_sig_keys.sig.private_key));
        secure_zero(g_sig_keys.sig.public_key,
                    sizeof(g_sig_keys.sig.public_key));
        g_sig_keys.sig.priv_len = 0U;
        g_sig_keys.sig.pub_len  = 0U;
        log_printf("[SECURITY][OTP] Ed25519 RAM key copies zeroized after signing\r\n");
        /* Store the signed manifest only after signature generation succeeds. */
            manifest_store_status_t store_ret = store_signed_manifest_protected(&g_signed_manifest,&g_sig_keys);
            if (store_ret != MANIFEST_STORE_SUCCESS) {
                log_printf("[FATAL] Failed to store manifest (rc=%d)\r\n", store_ret);
                handle_error("Manifest storage failed");
            }
            log_printf("[SUCCESS] Secure boot manifest provisioned\r\n");
            /* Reload the signed manifest through the normal boot path. */
            manifest_store_status_t load_ret = load_signed_manifest(&g_signed_manifest);
            if (load_ret != MANIFEST_STORE_SUCCESS) {
                log_printf("[FATAL] Failed to load manifest (rc=%d)\r\n", load_ret);
                handle_error("Manifest load failed");
            }
            log_printf("[SUCCESS] Signed manifest loaded from flash\r\n");
            /*
             * The key copies were already zeroized immediately after signing.
             * Remaining CMOX working state is cleaned before jumping to app.
             */
            /* Verify the manifest signature against the OTP-backed trust anchor. */
            auth_rc = verify_manifest_authenticity_from_flash(&g_sig_keys, &g_signed_manifest);
            if (auth_rc != VERIFY_AUTH_SUCCESS) {
                log_printf("[FATAL] Authenticity verification failed (%d)\r\n", (int)auth_rc);
                handle_error("Authenticity verification failed");
            }
//            /* Negative authenticity test: bad public key, correct manifest, correct signature */
//            negtest_rc = test_auth_fail_with_bad_pubkey(&g_sig_keys);
//            if (negtest_rc != VERIFY_AUTH_NEGTEST_PASS) {
//                log_printf("[FATAL] Negative authenticity test (bad pubkey) FAILED unexpectedly (%d)\r\n",
//                           (int)negtest_rc);
//                handle_error("Negative authenticity test (bad pubkey) did not behave as expected");
//            } else {
//                log_printf("[OK] Negative authenticity test (bad pubkey) passed: tampered public key was rejected\r\n");
//            }
            /* Optional negative test: corrupt signature with valid manifest/public key. */
//            negtest_rc = test_auth_fail_with_bad_signature(&g_sig_keys);
//            if (negtest_rc != VERIFY_AUTH_NEGTEST_PASS) {
//                log_printf("[FATAL] Negative authenticity test (bad signature) FAILED unexpectedly (%d)\r\n",
//                           (int)negtest_rc);
//                handle_error("Negative authenticity test (bad signature) did not behave as expected");
//            } else {
//                log_printf("[OK] Negative authenticity test (bad signature) passed: tampered signature was rejected\r\n");
//            }
            /* Optional negative test: corrupt manifest with valid signature/public key. */
//            negtest_rc = test_auth_fail_with_bad_manifest(&g_sig_keys);
//            if (negtest_rc != VERIFY_AUTH_NEGTEST_PASS) {
//                log_printf("[FATAL] Negative authenticity test (bad manifest) FAILED unexpectedly (%d)\r\n",
//                           (int)negtest_rc);
//                handle_error("Negative authenticity test (bad manifest) did not behave as expected");
//            } else {
//                log_printf("[OK] Negative authenticity test (bad manifest) passed: tampered manifest was rejected\r\n");
//            }
            /* Hash the application image and compare it with the signed manifest digest. */
            /* Verify application-image integrity before the jump. */
            integ_rc = verify_sector5_integrity_against_manifest(&g_signed_manifest, true);
            if (integ_rc != VERIFY_INTEGRITY_SUCCESS) {
                log_printf("[FATAL] Integrity verification failed (%d)\r\n", (int)integ_rc);
                handle_error("Integrity verification failed");
            }
            secure_zero_all_keys(&g_sig_keys);
            /* Secure boot passed; transfer control to the verified application. */
            jump_to_app(APP_ADDRESS);
    for(;;) {
    }
}
/**
 * @brief Configure the vector-table base during early system startup.
 */
void SystemInit(void)
{
    /* Initialize the required state before enabling the peripheral. */
	SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
}
