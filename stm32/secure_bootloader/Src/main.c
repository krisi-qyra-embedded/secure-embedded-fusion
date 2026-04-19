/* Author: i fiksuari pas prog | Documentation date: 19 April 2026 */
#include <stdio.h>
#include "stm32f4xx.h"
#include "fpu.h"
#include "uart.h"
#include "timebase.h"
#include "bsp.h"
#include "adc.h"
#include "cmox_crypto.h"
#include <string.h>

#define  GPIOAEN        (1U<<0)
#define  PIN5            (1U<<5)
#define  LED_PIN        PIN5

#define AES_256_KEY_SIZE        32  // 32 bytes = 256 bits
#define AES_256_IV_SIZE         16  // 16 bytes (CTR counter block)

#define ADC_SAMPLE_COUNT  8
#define BYTES_PER_SAMPLE  sizeof(uint32_t)

uint32_t adc_values[ADC_SAMPLE_COUNT];
uint32_t decrypted_adc_values[ADC_SAMPLE_COUNT];

uint8_t plain_text_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE];
uint8_t cipher_text_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE + AES_256_IV_SIZE]; // extra slack
uint8_t decrypted_bytes[ADC_SAMPLE_COUNT * BYTES_PER_SAMPLE + AES_256_IV_SIZE];

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

/* Reports a failed operation through the debug output. */
void handle_error(const char *message) {
    /* Log the failure before stopping further processing. */
    printf("ERROR: %s\n\r", message);
    while (1) {
        GPIOA->ODR ^= LED_PIN;
        for (volatile int i = 0; i < 1000000; i++);
    }
}

/* ---------- DRBG-based key/IV generation (kept from your original) ---------- */
uint8_t Key[AES_256_KEY_SIZE] = {0};
uint8_t IV[AES_256_IV_SIZE] = {0};

const uint8_t entropy[] = {
    0x4C, 0xFB, 0x21, 0x86, 0x73, 0x34, 0x6D, 0x9D,
    0x50, 0xC9, 0x22, 0xE4, 0x9B, 0x0D, 0xFC, 0xD0,
    0x90, 0xAD, 0xF0, 0x4F, 0x5C, 0x3B, 0xA4, 0x73,
    0x27, 0xDF, 0xCD, 0x6F, 0xA6, 0x3A, 0x78, 0x5C
};

const uint8_t nonce[] = {
    0x01, 0x69, 0x62, 0xA7, 0xFD, 0x27, 0x87, 0xA2,
    0x4B, 0xF6, 0xBE, 0x47, 0xEF, 0x37, 0x83, 0xF1
};

const uint8_t personalization[] = {
    0x88, 0xEE, 0xB8, 0xE0, 0xE8, 0x3B, 0xF3, 0x29,
    0x4B, 0xDA, 0xCD, 0x60, 0x99, 0xEB, 0xE4, 0xBF,
    0x55, 0xEC, 0xD9, 0x11, 0x3F, 0x71, 0xE5, 0xEB,
    0xCB, 0x45, 0x75, 0xF3, 0xD6, 0xA6, 0x8A, 0x6B
};

cmox_ctr_drbg_handle_t con_drbg_ctx;

/* Prepares or updates cryptographic key material. */
void generate_key_and_iv(void) {
    /* Validate the key data before using it in crypto operations. */
    cmox_drbg_retval_t retval;
    cmox_drbg_handle_t *drbg_ctx;

    drbg_ctx = cmox_ctr_drbg_construct(&con_drbg_ctx, CMOX_CTR_DRBG_AES256);
    if (drbg_ctx == NULL) handle_error("DRBG construct failed");

    retval = cmox_drbg_init(drbg_ctx, entropy, sizeof(entropy), personalization, sizeof(personalization), nonce, sizeof(nonce));
    if (retval != CMOX_DRBG_SUCCESS) handle_error("DRBG init failed");

    retval = cmox_drbg_generate(drbg_ctx, NULL, 0, Key, AES_256_KEY_SIZE);
    if (retval != CMOX_DRBG_SUCCESS) handle_error("Key generation failed");

    retval = cmox_drbg_generate(drbg_ctx, NULL, 0, IV, AES_256_IV_SIZE);
    if (retval != CMOX_DRBG_SUCCESS) handle_error("IV generation failed");

    retval = cmox_drbg_cleanup(drbg_ctx);
    if (retval != CMOX_DRBG_SUCCESS) handle_error("DRBG cleanup failed");
}

/* Initializes the platform and runs this cryptographic test sequence. */
int main(void)
{
    cmox_cipher_retval_t retval;
    size_t computed_size;

    fpu_enable();
    debug_uart_init();
    timebase_init();
    led_init();
    button_init();
    pa1_adc_init();
    start_conversion();

    if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS) handle_error("cmox init failed");

    /* 1) Read ADC values */
    for (int i = 0; i < ADC_SAMPLE_COUNT; ++i) {
        adc_values[i] = adc_read();
        delay(300);
    }

    /* 2) Serialize */
    decompose_adc_values();
    size_t plain_len = sizeof(plain_text_bytes); /* exactly ADC_SAMPLE_COUNT * 4 */

    /* 3) Generate Key and IV (or derive Key from ECDH/KDF in your final design) */
    generate_key_and_iv();

    /* 4) Encrypt using AES-CTR (no padding)
       Use the fast CTR variant constant; CTR produces ciphertext same length as plaintext.
       Note: do NOT reuse IV with same Key across messages. */
    retval = cmox_cipher_encrypt(CMOX_AESFAST_CTR_ENC_ALGO,
                                plain_text_bytes, plain_len,
                                Key, sizeof(Key),
                                IV, sizeof(IV),
                                cipher_text_bytes, &computed_size);
    if (retval != CMOX_CIPHER_SUCCESS) handle_error("Encryption failed");

    /* computed_size should equal plain_len for CTR */
    if (computed_size != plain_len) handle_error("Unexpected cipher length");

    /* 5) Decrypt (CTR): use the CTR decrypt algorithm */
    retval = cmox_cipher_decrypt(CMOX_AESFAST_CTR_DEC_ALGO,
                                cipher_text_bytes, computed_size,
                                Key, sizeof(Key),
                                IV, sizeof(IV),
                                decrypted_bytes, &computed_size);
    if (retval != CMOX_CIPHER_SUCCESS) handle_error("Decryption failed");

    /* 6) Verify */
    if (memcmp(plain_text_bytes, decrypted_bytes, plain_len) != 0) handle_error("Decrypted mismatch");

    /* 7) Recombine */
    recombine_adc_values();

    printf("*************************************************************\r\n");
    for (int i = 0; i < ADC_SAMPLE_COUNT; ++i) {
        printf("Decrypted sensor value[%d]: %d\r\n", i, (int)decrypted_adc_values[i]);
        printf("Original sensor value[%d]: %d\r\n", i, (int)adc_values[i]);
    }
    printf("*************************************************************\r\n");

    while (1) { }
}
