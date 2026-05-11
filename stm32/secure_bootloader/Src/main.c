/* Author: i fiksuari pas prog | Documentation date: 11 May 2026 */
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

extern uint32_t adc_values[ADC_SAMPLE_COUNT];
extern uint8_t my_secret[CMOX_ECC_SECP256R1_SECRET_LEN];
extern  size_t my_len;
extern  uint8_t aes_key[32];
extern  uint8_t mac_key[32];
extern  size_t computed_size;
extern uint8_t public_key_sender[AES_512_KEY_SIZE];
extern  size_t pub_len_sender;
extern  size_t my_len_receiver;
extern  uint8_t my_secret_receiver[CMOX_ECC_SECP256R1_SECRET_LEN];
extern  uint8_t aes_key_receiver[32];
extern  uint8_t mac_key_receiver[32];
extern  uint32_t decrypted_adc_values[ADC_SAMPLE_COUNT];
extern uint8_t *nonce_;

cmox_cipher_retval_t retval;
int32_t ret = 0,rc;


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


    /* 1) Read ADC values */
    for (int i = 0; i < ADC_SAMPLE_COUNT; ++i) {
        adc_values[i] = 1;
        /* Adjust multiplier for your CPU clock */
    }
    /* Initialize cryptolib*/
    	if(cmox_initialize(NULL) !=  CMOX_INIT_SUCCESS)
    	{
    		handle_error("Initialization of Nonce failed!");
    	}
    /* 2) Serialize */
    	decompose_adc_values();

    /* 2) Serialize */
    //SENDER
    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    /* -------- Generate DRBG-based private keys -------- */
	#if 0
    	ret = generate_private_key();
    	if (ret != 0) {
    		handle_error("generate_private_key failed");
    	}
	#endif
        /* -------- Generate ECC keypairs (priv + pub) -------- */
    	/* platform init... clocks, RNG, crypto libs, peripherals, etc. */
    	ret = sender_init_ecdh_crypto();
    	    if (ret != SENDER_INIT_SUCCESS) {
    	        handle_error("sender_init_ecdh_crypto failed");
    	        /* handle_error may not return */
    	    }
        /* on success: mac[0..mac_len-1] contains the HMAC (usually 32 bytes) */
    ///////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////
    //RECEIVER
    	    /* Assume sender's public key has already been received and stored in public_key_sender/pub_len_sender */
    	    ret = receiver_init_ecdh_crypto();
    	        if (ret != RECEIVER_INIT_SUCCESS) {
    	            handle_error("receiver_init_ecdh_crypto failed");
    	            /* handle_error may not return */
    	        }
           ///////////////////////////////////////////////////////////////
           ///////////////////////////////////////////////////////////////

    for(;;) {

    	/* -------- Generate DRBG-based IV -------- */
    	/*SENDER*/
    	 /* Call per transfer */
    	ret = generate_nonce_iv_encrypt_gcm_sender();
    	    if (ret != 0) {
    	        log_printf("generate_nonce_iv_encrypt_hmac_senderGCM_AEAD failed (rc=%d)\r\n", ret);
    	        handle_error("nonce/iv/encrypt GCM AEAD failed");
    	    }

    	     /*RECEIVER*/
    	    /* fill received_mac... and set computed_size, cipher_text_bytes, IV from incoming packet */
    	     ret = decrypt_verify_recombine_gcm_receiver();
    	    if (ret != 0) {
    	        log_printf("decrypt_recombine_receiver failed (rc=%d)\r\n", ret);
    	        handle_error("receive processing failed");
    	    }

    	    /* ... after you're done with decrypted_adc_values ... */
    	         ret = secure_clear_decrypted_adc_values();
    	        if (ret != 0) {
    	            log_printf("secure_clear_decrypted_adc_values failed (rc=%d)\r\n", rc);
    	            handle_error("failed to initialise  decrypted_adc_values to 0. \r\n");
    	        }

    }
}
