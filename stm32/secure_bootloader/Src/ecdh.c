/**
 * @file ecdh_refactored.c
 * @brief ECDH, Ed25519 provisioning and AES-GCM/CTR crypto handling.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>
#include "ecdh.h"
#include "digital_signature.h"
#include "stm32f4xx.h"
/*crypto_ctx_t ctx;*/
/*
 * Ed25519 OTP record uses four consecutive 32-byte data blocks.
 * The record stores a header, 64-byte private representation and
 * 32-byte public trust anchor. OTP lock bytes are not key storage.
 */
#define STM32F411_OTP_DATA_START              ((uintptr_t)0x1FFF7800U)
#define STM32F411_OTP_DATA_END                ((uintptr_t)0x1FFF79FFU)
#define STM32F411_OTP_LOCK_START              ((uintptr_t)0x1FFF7A00U)
#define STM32F411_OTP_BLOCK_SIZE              ((size_t)32U)
#define STM32F411_OTP_BLOCK_COUNT             ((size_t)16U)
#define ED25519_OTP_RECORD_BLOCKS             ((size_t)4U)
#define ED25519_OTP_RECORD_SIZE               (ED25519_OTP_RECORD_BLOCKS * STM32F411_OTP_BLOCK_SIZE)
#define ED25519_OTP_HEADER_SIZE               STM32F411_OTP_BLOCK_SIZE
#define ED25519_OTP_PRIVATE_OFFSET            ((uintptr_t)0x20U)
#define ED25519_OTP_PUBLIC_OFFSET             ((uintptr_t)0x60U)
#define ED25519_OTP_RECORD_VERSION            ((uint8_t)0x01U)
#define ED25519_OTP_RECORD_MARKER             ((uint8_t)0xA5U)
#define ED25519_OTP_FACTORY_PROVISIONING_ENABLE  1U
/* STM32F411 Flash programming constants used by OTP provisioning. */
#define STM32F411_FLASH_KEY1                  ((uint32_t)0x45670123U)
#define STM32F411_FLASH_KEY2                  ((uint32_t)0xCDEF89ABU)
#define STM32F411_FLASH_CR_PSIZE_MASK         ((uint32_t)0x00000300U)
#define STM32F411_FLASH_WAIT_LIMIT            ((uint32_t)10000000U)
#define STM32F411_FLASH_ERROR_MASK \
    (FLASH_SR_OPERR  | FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
     FLASH_SR_PGPERR | FLASH_SR_PGSERR)
#define STM32F411_FLASH_CLEAR_MASK \
    (STM32F411_FLASH_ERROR_MASK | FLASH_SR_EOP)
/* Commit header used to discover a valid Ed25519 OTP record. */
static const uint8_t g_ed25519_otp_header[ED25519_OTP_HEADER_SIZE] = {
    'E','D','2','5','5','1','9','K',
    ED25519_OTP_RECORD_VERSION,
    (uint8_t)ED25519_PRIVLEN,
    (uint8_t)ED25519_PUBLEN,
    ED25519_OTP_RECORD_MARKER,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
    0xFF,0xFF,0xFF,0xFF
};
/* Reject erased or all-zero OTP key material. */
static bool otp_key_material_is_valid(const uint8_t *key, size_t len)
{
    /* Return a failure status if any validation check fails. */
    uint8_t any_not_ff = 0U;
    uint8_t any_not_00 = 0U;
    if ((key == NULL) || (len == 0U)) {
        return false;
    }
    for (size_t i = 0U; i < len; ++i) {
        any_not_ff |= (uint8_t)(key[i] ^ 0xFFU);
        any_not_00 |= key[i];
    }
    return ((any_not_ff != 0U) && (any_not_00 != 0U));
}
/* Validate OTP record alignment and data-region bounds. */
static bool otp_record_address_is_in_data_region(uintptr_t record_addr)
{
    /* Check the return status before continuing to the next step. */
    if (record_addr < STM32F411_OTP_DATA_START) {
        return false;
    }
    if (record_addr > (STM32F411_OTP_DATA_END + 1U - ED25519_OTP_RECORD_SIZE)) {
        return false;
    }
    return (((record_addr - STM32F411_OTP_DATA_START) % STM32F411_OTP_BLOCK_SIZE) == 0U);
}
/* Validate the header and keypair stored in one OTP record. */
static bool otp_record_is_valid_at(uintptr_t record_addr)
{
    /* Return a failure status if any validation check fails. */
    const uint8_t *header;
    const uint8_t *priv;
    const uint8_t *pub;
    if (!otp_record_address_is_in_data_region(record_addr)) {
        return false;
    }
    header = (const uint8_t *)record_addr;
    priv   = (const uint8_t *)(record_addr + ED25519_OTP_PRIVATE_OFFSET);
    pub    = (const uint8_t *)(record_addr + ED25519_OTP_PUBLIC_OFFSET);
    if (memcmp(header, g_ed25519_otp_header, ED25519_OTP_HEADER_SIZE) != 0) {
        return false;
    }
    if (!otp_key_material_is_valid(priv, ED25519_PRIVLEN) ||
        !otp_key_material_is_valid(pub, ED25519_PUBLEN)) {
        return false;
    }
    if (memcmp(&priv[ED25519_PRIVLEN - ED25519_PUBLEN], pub, ED25519_PUBLEN) != 0) {
        return false;
    }
    return true;
}
/* Scan OTP for an existing committed Ed25519 record. */
static bool otp_find_existing_ed25519_record(uintptr_t *record_addr_out)
{
    /* Check the return status before continuing to the next step. */
    if (record_addr_out == NULL) {
        return false;
    }
    for (size_t first_block = 0U;
         first_block <= (STM32F411_OTP_BLOCK_COUNT - ED25519_OTP_RECORD_BLOCKS);
         ++first_block) {
        const uintptr_t addr =
            STM32F411_OTP_DATA_START + (first_block * STM32F411_OTP_BLOCK_SIZE);
        if (otp_record_is_valid_at(addr)) {
            *record_addr_out = addr;
            return true;
        }
    }
    return false;
}
/* Resolve private/public key addresses from the committed OTP record. */
static bool otp_get_ed25519_key_addresses(uintptr_t *private_addr_out,
                                          uintptr_t *public_addr_out,
                                          uintptr_t *record_addr_out)
{
    /* Validate the key data before using it in crypto operations. */
    uintptr_t record_addr;
    if (!otp_find_existing_ed25519_record(&record_addr)) {
        return false;
    }
    if (private_addr_out != NULL) {
        *private_addr_out = record_addr + ED25519_OTP_PRIVATE_OFFSET;
    }
    if (public_addr_out != NULL) {
        *public_addr_out = record_addr + ED25519_OTP_PUBLIC_OFFSET;
    }
    if (record_addr_out != NULL) {
        *record_addr_out = record_addr;
    }
    return true;
}
#if ED25519_OTP_FACTORY_PROVISIONING_ENABLE
/* Fixed Ed25519 keypair used only by the factory-provisioning build. */
static const uint8_t factory_ed25519_private_key[ED25519_PRIVLEN] = {
    0xBD,0xE0,0x22,0x62,0xB4,0xC0,0xF2,0x6C,
    0x6D,0xB8,0x2F,0x89,0x8F,0x71,0xC4,0xA7,
    0xBB,0x92,0x88,0xD7,0x50,0xB3,0xA6,0xC1,
    0x6A,0x92,0x1A,0xE4,0x3A,0x2F,0x77,0xEC,
    0xA4,0x6F,0xAB,0x93,0xDB,0x0D,0x32,0x01,
    0xE1,0xC2,0xF1,0x68,0x27,0xD7,0x55,0x35,
    0xFD,0x7F,0xE5,0x60,0xF6,0x29,0x8B,0x8B,
    0xD3,0xBC,0x27,0x7C,0xF6,0xA9,0xD4,0x61
};
static const uint8_t factory_ed25519_public_key[ED25519_PUBLEN] = {
    0xA4,0x6F,0xAB,0x93,0xDB,0x0D,0x32,0x01,
    0xE1,0xC2,0xF1,0x68,0x27,0xD7,0x55,0x35,
    0xFD,0x7F,0xE5,0x60,0xF6,0x29,0x8B,0x8B,
    0xD3,0xBC,0x27,0x7C,0xF6,0xA9,0xD4,0x61
};
/* Wait for the STM32 Flash controller to leave the busy state. */
static bool cmsis_flash_wait_ready(void)
{
    /* Check the flash status and address range before continuing. */
    uint32_t remaining = STM32F411_FLASH_WAIT_LIMIT;
    while ((FLASH->SR & FLASH_SR_BSY) != 0U) {
        if (remaining == 0U) {
            return false;
        }
        --remaining;
    }
    return true;
}
/* Clear Flash completion/error status before OTP programming. */
static void cmsis_flash_clear_status_flags(void)
{
    /* Check the flash status and address range before continuing. */
    FLASH->SR = STM32F411_FLASH_CLEAR_MASK;
    __DSB();
}
/* Unlock the Flash controller for OTP programming. */
static bool cmsis_flash_unlock(void)
{
    /* Check the flash status and address range before continuing. */
    if (!cmsis_flash_wait_ready()) {
        return false;
    }
    if ((FLASH->CR & FLASH_CR_LOCK) == 0U) {
        return true;
    }
    FLASH->KEYR = STM32F411_FLASH_KEY1;
    FLASH->KEYR = STM32F411_FLASH_KEY2;
    __DSB();
    return ((FLASH->CR & FLASH_CR_LOCK) == 0U);
}
/* Disable programming and lock the Flash controller again. */
static void cmsis_flash_lock(void)
{
    /* Check the flash status and address range before continuing. */
    (void)cmsis_flash_wait_ready();
    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;
    __DSB();
}
/* Program one previously-erased OTP byte and verify the write. */
static bool cmsis_flash_program_otp_byte(uintptr_t address, uint8_t value)
{
    /* Check the flash status and address range before continuing. */
    volatile uint8_t *dst = (volatile uint8_t *)address;
    uint32_t status;
    if (*dst == value) {
        return true;
    }
    if (*dst != 0xFFU) {
        return false;
    }
    if (!cmsis_flash_wait_ready()) {
        return false;
    }
    cmsis_flash_clear_status_flags();
    FLASH->CR &= ~STM32F411_FLASH_CR_PSIZE_MASK;
    FLASH->CR |= FLASH_CR_PG;
    __DSB();
    *dst = value;
    __DSB();
    if (!cmsis_flash_wait_ready()) {
        FLASH->CR &= ~FLASH_CR_PG;
        return false;
    }
    status = FLASH->SR;
    FLASH->CR &= ~FLASH_CR_PG;
    __DSB();
    if ((status & STM32F411_FLASH_ERROR_MASK) != 0U) {
        FLASH->SR = (status & STM32F411_FLASH_CLEAR_MASK);
        __DSB();
        return false;
    }
    if ((status & FLASH_SR_EOP) != 0U) {
        FLASH->SR = FLASH_SR_EOP;
        __DSB();
    }
    return (*dst == value);
}
/* Check that an OTP span is still fully erased. */
static bool otp_span_is_erased(uintptr_t address, size_t len)
{
    /* Check the return status before continuing to the next step. */
    const volatile uint8_t *p = (const volatile uint8_t *)address;
    for (size_t i = 0U; i < len; ++i) {
        if (p[i] != 0xFFU) {
            return false;
        }
    }
    return true;
}
/* Check that all four OTP blocks remain programmable. */
static bool otp_record_blocks_are_unlocked(size_t first_block)
{
    /* Check the return status before continuing to the next step. */
    const volatile uint8_t *lock =
        (const volatile uint8_t *)STM32F411_OTP_LOCK_START;
    if (first_block > (STM32F411_OTP_BLOCK_COUNT - ED25519_OTP_RECORD_BLOCKS)) {
        return false;
    }
    for (size_t i = 0U; i < ED25519_OTP_RECORD_BLOCKS; ++i) {
        if (lock[first_block + i] != 0xFFU) {
            return false;
        }
    }
    return true;
}
/* Find four erased/unlocked OTP blocks for a new Ed25519 record. */
static bool otp_find_free_ed25519_record_slot(uintptr_t *record_addr_out,
                                               size_t *first_block_out)
{
    /* Check the return status before continuing to the next step. */
    if (record_addr_out == NULL) {
        return false;
    }
    for (size_t first_block = 0U;
         first_block <= (STM32F411_OTP_BLOCK_COUNT - ED25519_OTP_RECORD_BLOCKS);
         ++first_block) {
        const uintptr_t addr =
            STM32F411_OTP_DATA_START + (first_block * STM32F411_OTP_BLOCK_SIZE);
        if (!otp_record_blocks_are_unlocked(first_block)) {
            continue;
        }
        if (!otp_span_is_erased(addr, ED25519_OTP_RECORD_SIZE)) {
            continue;
        }
        *record_addr_out = addr;
        if (first_block_out != NULL) {
            *first_block_out = first_block;
        }
        return true;
    }
    return false;
}
/* Program and read back one OTP byte range. */
static bool otp_program_bytes(uintptr_t address,
                              const uint8_t *expected,
                              size_t len)
{
    /* Check the return status before continuing to the next step. */
    if ((expected == NULL) || (len == 0U)) {
        return false;
    }
    for (size_t i = 0U; i < len; ++i) {
        if (expected[i] == 0xFFU) {
            continue;
        }
        if (!cmsis_flash_program_otp_byte(address + i, expected[i])) {
            log_printf(
                "[SECURITY][OTP][ERROR] OTP programming failed at 0x%08lX "
                "FLASH_SR=0x%08lX\r\n",
                (unsigned long)(address + i),
                (unsigned long)FLASH->SR);
            return false;
        }
    }
    __DSB();
    __ISB();
    if (memcmp((const void *)address, expected, len) != 0) {
        log_printf("[SECURITY][OTP][ERROR] OTP readback mismatch at 0x%08lX\r\n",
                   (unsigned long)address);
        return false;
    }
    return true;
}
/* Provision the factory Ed25519 keypair only when no valid record exists. */
static bool provision_ed25519_keypair_to_otp_if_needed(void)
{
    uintptr_t record_addr = 0U;
    uintptr_t private_addr = 0U;
    uintptr_t public_addr = 0U;
    size_t first_block = 0U;
    if (memcmp(&factory_ed25519_private_key[ED25519_PRIVLEN - ED25519_PUBLEN],
               factory_ed25519_public_key,
               ED25519_PUBLEN) != 0) {
        log_printf("[SECURITY][OTP][ERROR] Compiled Ed25519 keypair is inconsistent\r\n");
        return false;
    }
    /* Existing committed record wins; never provision a second trust identity. */
    if (otp_get_ed25519_key_addresses(&private_addr, &public_addr, &record_addr)) {
        if ((memcmp((const void *)private_addr,
                    factory_ed25519_private_key,
                    ED25519_PRIVLEN) != 0) ||
            (memcmp((const void *)public_addr,
                    factory_ed25519_public_key,
                    ED25519_PUBLEN) != 0)) {
            log_printf(
                "[SECURITY][OTP][ERROR] Existing Ed25519 OTP record contains a "
                "different keypair; refusing to provision another trust identity\r\n");
            return false;
        }
        log_printf(
            "[SECURITY][OTP] Existing Ed25519 record found at 0x%08lX "
            "(priv=0x%08lX pub=0x%08lX); write skipped\r\n",
            (unsigned long)record_addr,
            (unsigned long)private_addr,
            (unsigned long)public_addr);
        return true;
    }
    if (!otp_find_free_ed25519_record_slot(&record_addr, &first_block)) {
        log_printf(
            "[SECURITY][OTP][ERROR] No four-block erased/unlocked OTP slot available "
            "inside 0x%08lX..0x%08lX\r\n",
            (unsigned long)STM32F411_OTP_DATA_START,
            (unsigned long)STM32F411_OTP_DATA_END);
        return false;
    }
    private_addr = record_addr + ED25519_OTP_PRIVATE_OFFSET;
    public_addr  = record_addr + ED25519_OTP_PUBLIC_OFFSET;
    log_printf(
        "[SECURITY][OTP] Free slot found: blocks %lu..%lu record=0x%08lX "
        "priv=0x%08lX pub=0x%08lX\r\n",
        (unsigned long)first_block,
        (unsigned long)(first_block + ED25519_OTP_RECORD_BLOCKS - 1U),
        (unsigned long)record_addr,
        (unsigned long)private_addr,
        (unsigned long)public_addr);
    /* OTP programming starts only after a free record has been selected. */
    if (!cmsis_flash_unlock()) {
        log_printf("[SECURITY][OTP][ERROR] FLASH unlock failed CR=0x%08lX SR=0x%08lX\r\n",
                   (unsigned long)FLASH->CR,
                   (unsigned long)FLASH->SR);
        return false;
    }
    cmsis_flash_clear_status_flags();
    if (!otp_program_bytes(private_addr,
                           factory_ed25519_private_key,
                           ED25519_PRIVLEN) ||
        !otp_program_bytes(public_addr,
                           factory_ed25519_public_key,
                           ED25519_PUBLEN) ||
        !otp_program_bytes(record_addr,
                           g_ed25519_otp_header,
                           ED25519_OTP_HEADER_SIZE)) {
        cmsis_flash_lock();
        return false;
    }
    cmsis_flash_lock();
    if (!otp_record_is_valid_at(record_addr)) {
        log_printf("[SECURITY][OTP][ERROR] Final dynamic OTP record verification failed\r\n");
        return false;
    }
    log_printf(
        "[SECURITY][OTP] Ed25519 keypair programmed and verified at dynamic OTP "
        "record 0x%08lX\r\n",
        (unsigned long)record_addr);
    return true;
}
#endif
/* General crypto helpers and input formatting. */
/**
 * @brief Log a fatal error and stop normal execution.
 * @param message Error message to print.
 */
void handle_error(const char *message) {
    // Print the error message to the debug console
    log_printf("ERROR: %s\r\n", message);
    while (1) {
        // Toggle an LED to indicate an error (assuming LED_PIN is configured)
        //GPIOA->ODR ^= LED_PIN;  // Toggle LED
        for (volatile int i = 0; i < 1000000; i++);  // Delay loop
    }
}
/**
 * @brief Split configured ADC samples into the byte buffer used by the crypto path.
 * @param ctx Crypto context containing ADC input and plaintext buffers.
 * @return ADC decomposition status.
 */
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
/* Check whether a key buffer is entirely zero. */
static int is_all_zero_buf(const crypto_ctx_t *ctx,
                           const uint8_t *buf,
                           size_t len)
{
    /* Clear the buffers before releasing the key material. */
    (void)ctx;
    uint8_t acc = 0;
    for (size_t i = 0; i < len; i++) {
        acc |= buf[i];
    }
    return (acc == 0);
}
/**
 * @brief Validate the sender public-key length, prefix and non-zero content.
 * @param ctx Crypto context used by the validation policy.
 * @param pub Public-key buffer.
 * @param pub_len Public-key length.
 * @return Public-key validation status.
 */
pubkey_status_t validate_public_key_sender_format_ctx(const crypto_ctx_t *ctx,
                                                      const uint8_t *pub,
                                                      size_t pub_len)
{
    /* Return a failure status if any validation check fails. */
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
/* DRBG, key generation and Ed25519 setup. */
/**
 * @brief Generate a fresh IV from the project CTR-DRBG inputs.
 * @param ctx Crypto context receiving the generated IV.
 * @return IV generation status.
 */
gen_iv_status_t generate_iv(crypto_ctx_t *ctx)
{
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
/**
 * @brief Generate sender and receiver private-key material with CTR-DRBG.
 * @param ctx Crypto context receiving both private keys.
 * @return Private-key generation status.
 */
gen_priv_status_t generate_private_key(crypto_ctx_t *ctx)
{
    /* Validate the key data before using it in crypto operations. */
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
/* Fill a caller buffer from the initialized CTR-DRBG. */
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
 * @brief Generate secp256r1 sender and receiver keypairs with CMOX.
 * @param ctx Crypto context receiving both ECDH keypairs.
 * @return Keypair generation status.
 */
gen_keys_status_t generate_keypairs_cmox(crypto_ctx_t *ctx)
{
    /* Validate the key data before using it in crypto operations. */
    cmox_ecc_retval_t  ecc_ret;
    cmox_drbg_retval_t drbg_ret;
    cmox_drbg_handle_t *drbg_ctx = NULL;
    uint8_t random_buf[128];
    size_t  random_len = AES_256_KEY_SIZE;
    if (ctx == NULL) {
        return GEN_KEYS_ERR_BAD_PARAM;
    }
    memset(ctx->keys.private_key_sender,   0, sizeof(ctx->keys.private_key_sender));
    memset(ctx->keys.private_key_receiver, 0, sizeof(ctx->keys.private_key_receiver));
    memset(ctx->keys.public_key_sender,    0, sizeof(ctx->keys.public_key_sender));
    memset(ctx->keys.public_key_receiver,  0, sizeof(ctx->keys.public_key_receiver));
    ctx->keys.priv_len_sender   = 0;
    ctx->keys.pub_len_sender    = 0;
    ctx->keys.priv_len_receiver = 0;
    ctx->keys.pub_len_receiver  = 0;
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
    random_len = AES_256_KEY_SIZE;
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
/**
 * @brief Construct the CMOX ECC context required for Ed25519 signing.
 * @param ctx Ed25519 signing context.
 * @return Key setup status.
 */
gen_keys_status_t init_ecc_for_signing(sig_keys_ctx_t *ctx)
{
    /* Check the signature operation status before using the result. */
    if (ctx == NULL) {
        return GEN_KEYS_ERR_BAD_PARAM;
    }
    cmox_ecc_construct(&ctx->ecc,
                       CMOX_ECC256_MATH_FUNCS,
                       ctx->working_buffer,
                       sizeof(ctx->working_buffer));
    return GEN_KEYS_SUCCESS;
}
/**
 * @brief Reuse an existing Ed25519 keypair or generate one before signing.
 * @param ctx Ed25519 signing context.
 * @return Key generation/setup status.
 */
gen_keys_status_t generate_ed25519_signing_keypair_once(sig_keys_ctx_t *ctx)
{
    /* Check the signature operation status before using the result. */
    if (ctx == NULL) {
        return GEN_KEYS_ERR_BAD_PARAM;
    }
    if (ctx->sig.priv_len == ED25519_PRIVLEN && ctx->sig.pub_len == ED25519_PUBLEN) {
        return init_ecc_for_signing(ctx);
    }
    gen_keys_status_t gret = generate_ed25519_signing_keypair(ctx);
    if (gret != GEN_KEYS_SUCCESS) {
        return gret;
    }
    return init_ecc_for_signing(ctx);
}
/**
 * @brief Generate one Ed25519 signing keypair with the project CTR-DRBG.
 * @param ctx Ed25519 signing context receiving the keypair.
 * @return Key generation status.
 */
gen_keys_status_t generate_ed25519_signing_keypair(sig_keys_ctx_t *ctx)
{
    cmox_ecc_retval_t  ecc_ret;
    cmox_drbg_retval_t drbg_ret;
    uint8_t random_buf[64];
    uint8_t ret = 0;
    size_t  random_len = 32;
    if (ctx == NULL) {
        return GEN_KEYS_ERR_BAD_PARAM;
    }
    memset(ctx->sig.private_key, 0, sizeof(ctx->sig.private_key));
    memset(ctx->sig.public_key,  0, sizeof(ctx->sig.public_key));
    ctx->sig.priv_len = 0;
    ctx->sig.pub_len  = 0;
    ctx->sig.provisioned = false;
    ctx->sig.storage_flags = SIG_KEY_STORAGE_NONE;
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
        size_t local_priv_len = sizeof(ctx->sig.private_key);
        size_t local_pub_len  = sizeof(ctx->sig.public_key);
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
        (void)cmox_drbg_cleanup(ctx->drbg_ctx);
        memset(ctx->sig.private_key, 0, sizeof(ctx->sig.private_key));
        memset(ctx->sig.public_key,  0, sizeof(ctx->sig.public_key));
        return GEN_KEYS_ERR_KEYGEN_SENDER;
    }
    drbg_ret = cmox_drbg_cleanup(ctx->drbg_ctx);
    if (drbg_ret != CMOX_DRBG_SUCCESS) {
        memset(ctx->sig.private_key, 0, sizeof(ctx->sig.private_key));
        memset(ctx->sig.public_key,  0, sizeof(ctx->sig.public_key));
        return GEN_KEYS_ERR_DRBG_CLEANUP;
    }
    return GEN_KEYS_SUCCESS;
}
/**
 * @brief Print a byte buffer as comma-separated hexadecimal values.
 * @param label Label printed before the buffer.
 * @param buf Byte buffer to print.
 * @param len Buffer length.
 * @return Key-print status.
 */
private_public_print print_hex_bytes_comma(const char *label, const uint8_t *buf, size_t len)
{
    /* Check the return status before continuing to the next step. */
    if (label == NULL) label = "";
    if (buf == NULL || len == 0) {
        log_printf("%s: <empty>\r\n", label);
        return FAILED_PRINT_KEYS;
    }
    log_printf("%s: ", label);
    for (size_t i = 0; i < len; ++i) {
        if (i) log_printf(",");
        log_printf("%02X", (unsigned)buf[i]);
        delay_ms(5);
    }
    log_printf("\r\n");
    return SUCCESS_PRINT_KEYS;
}
/**
 * @brief Print the Ed25519 keypair currently stored in the signing context.
 * @param ctx Ed25519 signing context.
 * @return Key-print status.
 */
private_public_print debug_print_ed25519_keys(const sig_keys_ctx_t *ctx)
{
    /* Validate the key data before using it in crypto operations. */
    if (ctx == NULL) {
        log_printf("debug_print_ed25519_keys: NULL ctx\r\n");
        return FAILED_PRINT_KEYS;
    }
    size_t priv_len = ctx->sig.priv_len;
    size_t pub_len  = ctx->sig.pub_len;
    if (priv_len == 0 || priv_len > ED25519_PRIVLEN) priv_len = ED25519_PRIVLEN;
    if (pub_len  == 0 || pub_len  > ED25519_PUBLEN)  pub_len  = ED25519_PUBLEN;
    if(print_hex_bytes_comma("private_key", ctx->sig.private_key, priv_len) != SUCCESS_PRINT_KEYS){
    	return FAILED_PRINT_KEYS;
    }
    if(print_hex_bytes_comma("public_key",  ctx->sig.public_key,  pub_len) != SUCCESS_PRINT_KEYS){
    	return FAILED_PRINT_KEYS;
    }
    return SUCCESS_PRINT_KEYS;
}
/* OTP-backed Ed25519 runtime loading. */
/**
 * @brief Provision Ed25519 OTP when enabled and load the OTP keypair into RAM.
 * @param ctx Signing context receiving the OTP-backed keypair.
 * @return Sender initialization status.
 */
sender_init_status_t assign_private_and_public_key_from_hex(sig_keys_ctx_t *ctx)
{
    uintptr_t private_addr = 0U;
    uintptr_t public_addr = 0U;
    uintptr_t record_addr = 0U;
    const uint8_t *otp_private_key;
    const uint8_t *otp_public_key;
    if (ctx == NULL) {
        return SENDER_INIT_ERR_BAD_PARAM;
    }
#if ED25519_OTP_FACTORY_PROVISIONING_ENABLE
    if (!provision_ed25519_keypair_to_otp_if_needed()) {
        return SENDER_INIT_ERR_GENKEY;
    }
#endif
    if (!otp_get_ed25519_key_addresses(&private_addr, &public_addr, &record_addr)) {
        log_printf(
            "[SECURITY][OTP][ERROR] No valid Ed25519 OTP record found in "
            "0x%08lX..0x%08lX\r\n",
            (unsigned long)STM32F411_OTP_DATA_START,
            (unsigned long)STM32F411_OTP_DATA_END);
        return SENDER_INIT_ERR_GENKEY;
    }
    otp_private_key = (const uint8_t *)private_addr;
    otp_public_key  = (const uint8_t *)public_addr;
    secure_zero(ctx->sig.private_key, sizeof(ctx->sig.private_key));
    secure_zero(ctx->sig.public_key, sizeof(ctx->sig.public_key));
    ctx->sig.priv_len = 0U;
    ctx->sig.pub_len = 0U;
    ctx->sig.provisioned = false;
    ctx->sig.storage_flags = SIG_KEY_STORAGE_NONE;
    if (!otp_key_material_is_valid(otp_private_key, ED25519_PRIVLEN) ||
        !otp_key_material_is_valid(otp_public_key, ED25519_PUBLEN)) {
        log_printf("[SECURITY][OTP][ERROR] Dynamic OTP key material is invalid\r\n");
        return SENDER_INIT_ERR_GENKEY;
    }
    if (memcmp(&otp_private_key[ED25519_PRIVLEN - ED25519_PUBLEN],
               otp_public_key,
               ED25519_PUBLEN) != 0) {
        log_printf("[SECURITY][OTP][ERROR] Dynamic OTP private/public pair mismatch\r\n");
        return SENDER_INIT_ERR_GENKEY;
    }
    /* Copy OTP key material into RAM only for the signing operation. */
    memcpy(ctx->sig.private_key, otp_private_key, ED25519_PRIVLEN);
    memcpy(ctx->sig.public_key, otp_public_key, ED25519_PUBLEN);
    ctx->sig.priv_len = ED25519_PRIVLEN;
    ctx->sig.pub_len = ED25519_PUBLEN;
    ctx->sig.provisioned = true;
    ctx->sig.storage_flags = SIG_KEY_STORAGE_OTP;
    log_printf(
        "[SECURITY][OTP] Ed25519 keypair loaded from dynamic OTP record "
        "0x%08lX (priv=0x%08lX pub=0x%08lX)\r\n",
        (unsigned long)record_addr,
        (unsigned long)private_addr,
        (unsigned long)public_addr);
    return SENDER_INIT_SUCCESS;
}
/* Volatile byte wipe used for temporary crypto material. */
static void secure_memzero(void *p, size_t n)
{
    /* Clear the buffers before releasing the key material. */
    volatile uint8_t *vp = (volatile uint8_t *)p;
    while (n--) *vp++ = 0;
}
/* secp256r1 ECDH shared-secret handling. */
/**
 * @brief Compute the sender-side secp256r1 ECDH shared secret.
 * @param ctx Crypto context containing sender private and receiver public keys.
 * @param out_secret Optional shared-secret output buffer.
 * @param out_len Output capacity and returned secret length.
 * @return ECDH status.
 */
ecdh_status_t compute_ecdh_shared_as_sender(crypto_ctx_t *ctx, uint8_t *out_secret, size_t *out_len)
{
    cmox_ecc_retval_t retval;
    size_t pub_raw_len;
    if (ctx == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }
    pub_raw_len = (ctx->keys.pub_len_receiver != 0) ? ctx->keys.pub_len_receiver : AES_512_KEY_SIZE;
    ctx->tmp.computed_size = sizeof(ctx->tmp.computed_secret);
    cmox_ecc_construct(&ctx->ecc,
                       CMOX_ECC256_MATH_FUNCS,
                       ctx->working_buffer, sizeof(ctx->working_buffer));
    /* First try the raw X||Y peer public-key representation. */
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
    {
        uint8_t pub_prefixed[1 + AES_512_KEY_SIZE];
        size_t prefixed_len;
        if (pub_raw_len > AES_512_KEY_SIZE) {
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
            memset(ctx->tmp.computed_secret, 0, sizeof(ctx->tmp.computed_secret));
            ctx->tmp.computed_size = 0;
            return ECDH_ERR_ECDH_FAILED;
        }
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
/**
 * @brief Compute the receiver-side secp256r1 ECDH shared secret.
 * @param ctx Crypto context containing receiver private and sender public keys.
 * @param out_secret Optional shared-secret output buffer.
 * @param out_len Output capacity and returned secret length.
 * @return ECDH status.
 */
ecdh_status_t compute_ecdh_shared_as_receiver(crypto_ctx_t *ctx,
                                              uint8_t *out_secret,
                                              size_t *out_len)
{
    /* Validate the key agreement result before deriving session keys. */
    cmox_ecc_retval_t retval;
    size_t pub_raw_len;
    if (ctx == NULL) {
        return ECDH_ERR_ECDH_FAILED;
    }
    pub_raw_len = (ctx->keys.pub_len_sender != 0)
                    ? ctx->keys.pub_len_sender
                    : AES_512_KEY_SIZE;
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
/**
 * @brief Derive AES and MAC keys from the selected ECDH shared secret.
 * @param ctx Crypto context containing shared-secret and derived-key buffers.
 * @param receiver Select receiver material when true.
 * @return KDF status.
 */
/* Example usage:

   size_t outlen = 64;

   if (compute_ecdh_shared_as_sender(NULL, &outlen) == 0) {

       // computed_secret[0..outlen-1] contains the shared secret

   } else {

       handle_error("ECDH failed");

   }

*/
/* SHA-256 session-key derivation. */
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
    if (receiver) {
        shared  = ctx->tmp.computed_secret_receiver;
        aes_key = ctx->derived.aes_key_receiver;
        mac_key = ctx->derived.mac_key_receiver;
    } else {
        shared  = ctx->tmp.computed_secret;
        aes_key = ctx->derived.aes_key;
        mac_key = ctx->derived.mac_key;
    }
    memcpy(buf, shared, ctx->tmp.computed_size);
    /* Domain-separate the AES and MAC derivations with one trailing byte. */
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
    memset(buf, 0, sizeof(buf));
    return KDF_SUCCESS;
}
/**
 * @brief Derive receiver AES and MAC keys from the receiver shared secret.
 * @param ctx Crypto context containing receiver shared-secret material.
 * @return KDF status.
 */
kdf_status_t derive_keys_from_shared_secret_receiver(crypto_ctx_t *ctx)
{
    /* Check the digest result before using it in verification. */
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
    memset(buf, 0, sizeof(buf));
    return KDF_SUCCESS;
}
/* Check whether a byte range is entirely zero. */
static int is_all_zero_bytes(const uint8_t *p, size_t n)
{
    /* Clear the buffers before releasing the key material. */
    uint8_t acc = 0;
    for (size_t i = 0; i < n; ++i) acc |= p[i];
    return (acc == 0);
}
/* AES-CTR and HMAC compatibility path. */
/**
 * @brief Encrypt the current plaintext with the sender AES-CTR key and IV.
 * @param ctx Crypto context containing plaintext and sender key material.
 * @param out_computed_size Returned ciphertext length.
 * @return AES-CTR encryption status.
 */
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
/**
 * @brief Decrypt the received AES-CTR ciphertext with receiver key material.
 * @param ctx Crypto context containing ciphertext and receiver key material.
 * @param out_decrypted_size Returned plaintext length.
 * @return AES-CTR decryption status.
 */
aes_ctr_dec_status_t decrypt_aes_ctr(crypto_ctx_t *ctx,
                                    size_t *out_decrypted_size)
{
    /* Check the crypto status before consuming decrypted data. */
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
/**
 * @brief Compute the sender HMAC over IV and ciphertext.
 * @param ctx Crypto context containing sender MAC key and ciphertext.
 * @param mac_out MAC output buffer.
 * @param mac_out_len MAC output capacity.
 * @param out_mac_len Returned MAC length.
 * @return HMAC status.
 */
hmac_ivc_status_t compute_hmac_over_iv_cipher(crypto_ctx_t *ctx,
                                              uint8_t *mac_out,
                                              size_t mac_out_len,
                                              size_t *out_mac_len)
{
    /* Reject the operation when tag verification fails. */
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
/**
 * @brief Compute the receiver HMAC over the received IV and ciphertext.
 * @param ctx Crypto context containing receiver MAC key and ciphertext.
 * @param mac_out MAC output buffer.
 * @param mac_out_len MAC output capacity.
 * @param out_mac_len Returned MAC length.
 * @return HMAC status.
 */
hmac_ivc_status_t compute_hmac_over_iv_cipher_receiver(crypto_ctx_t *ctx,
                                                       uint8_t *mac_out,
                                                       size_t mac_out_len,
                                                       size_t *out_mac_len)
{
    /* Reject the operation when tag verification fails. */
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
/* Per-transfer nonce/IV setup and ECDH session initialization. */
/**
 * @brief Generate the per-transfer nonce state and DRBG-backed IV.
 * @param ctx Crypto context receiving the transfer IV.
 * @return IV generation status.
 */
gen_iv_status_t generate_nonce_and_iv_per_transfer(crypto_ctx_t *ctx)
{
    /* Check the generator status before using the output. */
    int rc;
    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }
    rc = nonce_generation();
    if (rc != NONCE_GEN_SUCCESS) {
        return rc;
    }
    rc = generate_iv(ctx);
    if (rc != GEN_IV_SUCCESS) {
        return rc;
    }
    rc = derive_gcm_iv_from_drbg(ctx);
    if (rc != GEN_IV_SUCCESS) {
        return rc;
    }
    return GEN_IV_SUCCESS;
}
/**
 * @brief Copy the generated DRBG IV material into the AES-GCM IV buffer.
 * @param ctx Crypto context containing IV and GCM IV buffers.
 * @return GCM IV setup status.
 */
gcm_master_status_t derive_gcm_iv_from_drbg(crypto_ctx_t *ctx)
{
    /* Check the GCM result and authentication tag before using plaintext. */
    if (ctx == NULL) {
        return GEN_IV_ERR_BAD_PARAM;
    }
    memcpy(ctx->IV_GCM, ctx->IV, AES_GCM_AEAD_IV_SIZE);
    return GCM_MASTER_SUCCESS;
}
/**
 * @brief Initialize sender ECDH material and derive the sender session keys.
 * @param ctx Crypto context to initialize.
 * @return Sender initialization status.
 */
sender_init_status_t sender_init_ecdh_crypto(crypto_ctx_t *ctx)
{
    /* Validate the key agreement result before deriving session keys. */
    int ret;
    if (ctx == NULL) {
        return SENDER_INIT_ERR_BAD_PARAM;
    }
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
/**
 * @brief Generate and initialize the Ed25519 signing-key context.
 * @param ctx Signing context to initialize.
 * @return Sender initialization status.
 */
sender_init_status_t generate_private_key_digital_signature(sig_keys_ctx_t *ctx)
{
    /* Check the signature operation status before using the result. */
    int ret;
    if (ctx == NULL) {
        return SENDER_INIT_ERR_BAD_PARAM;
    }
    ret = generate_ed25519_signing_keypair(ctx);
    if (ret != GEN_KEYS_SUCCESS) {
        log_printf("[ERROR] generate_ed25519_signing_keypair failed (rc=%d)\r\n", ret);
        secure_zero(ctx->sig.private_key, sizeof(ctx->sig.private_key));
        secure_zero(ctx->sig.public_key,  sizeof(ctx->sig.public_key));
        ctx->sig.priv_len     = 0;
        ctx->sig.pub_len      = 0;
        ctx->sig.provisioned  = false;
        ctx->sig.storage_flags = SIG_KEY_STORAGE_NONE;
        return SENDER_INIT_ERR_GENKEY;
    }
    return SENDER_INIT_SUCCESS;
}
/**
 * @brief Validate peer material, compute receiver ECDH secret and derive session keys.
 * @param ctx Crypto context to initialize.
 * @return Receiver initialization status.
 */
receiver_init_status_t receiver_init_ecdh_crypto(crypto_ctx_t *ctx)
{
    /* Validate the key agreement result before deriving session keys. */
    int ret;
    if (ctx == NULL) {
        return RECEIVER_INIT_ERR_BAD_PARAM;
    }
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
    ret = compute_ecdh_shared_as_receiver(ctx, NULL, NULL);
    if (ret != ECDH_SUCCESS) {
        log_printf("[ERROR] receiver_init_ecdh_crypto: compute_ecdh_shared_as_receiver failed (rc=%d)\r\n", ret);
        secure_zero(ctx->tmp.computed_secret_receiver, sizeof(ctx->tmp.computed_secret_receiver));
        ctx->tmp.computed_size = 0;
        return RECEIVER_INIT_ERR_ECDH;
    }
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
/**
 * @brief Generate transfer IV, encrypt with AES-CTR and authenticate with HMAC.
 * @param ctx Sender crypto context.
 * @return Sender encrypt/authenticate status.
 */
hmac_sender_status_t generate_nonce_iv_encrypt_hmac_sender(crypto_ctx_t *ctx)
{
    /* Check the generator status before using the output. */
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
/* Constant-time byte comparison; zero means equal. */
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
/**
 * @brief Verify HMAC, decrypt AES-CTR data and rebuild the ADC samples.
 * @param ctx Receiver crypto context.
 * @return Verification/decryption status.
 */
verify_dec_status_t verify_mac_decrypt_recombine_receiver(crypto_ctx_t *ctx)
{
    /* Reject the operation when tag verification fails. */
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
/**
 * @brief Securely clear the decrypted ADC sample array and verify the wipe.
 * @param ctx Crypto context containing decrypted ADC samples.
 * @return Secure-clear status.
 */
secure_clear_status_t secure_clear_decrypted_adc_values(crypto_ctx_t *ctx)
{
    /* Check the crypto status before consuming decrypted data. */
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
/* AES-GCM authenticated-encryption path. */
/**
 * @brief Encrypt the current plaintext with the streaming AES-GCM path.
 * @param ctx Crypto context containing GCM key, IV and plaintext.
 * @return Streaming AES-GCM encryption status.
 */
gcm_enc_status_t encrypt_aes_gcm_aead(crypto_ctx_t *ctx)
{
    /* Check the GCM result and authentication tag before using plaintext. */
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
    cmox_gcm_handle_t gcm_ctx;
    cmox_cipher_handle_t *h = cmox_gcm_construct(&gcm_ctx, CMOX_AESFAST_GCMFAST_ENC);
    if (h == NULL) {
        return GCM_ENC_ERR_CONSTRUCT;
    }
    cres = cmox_cipher_init(h);
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_INIT;
    }
    cres = cmox_cipher_setKey(h,
                              ctx->derived.aes_key,
                              (cmox_cipher_keyLen_t)sizeof(ctx->derived.aes_key));
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_SETKEY;
    }
    assert(sizeof(ctx->IV_GCM) == AES_GCM_AEAD_IV_SIZE);
    cres = cmox_cipher_setIV(h, ctx->IV_GCM, sizeof(ctx->IV_GCM));
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_SETIV;
    }
    cres = cmox_cipher_setTagLen(h, (size_t)AES_GCM_TAG_SIZE);
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_ENC_ERR_SETTAGLEN;
    }
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
    memcpy(ctx->buffers.cipher_text_gcm_aead + local_out_len, ctx->tmp.local_tag, tag_len);
    ctx->tmp.computed_size = local_out_len;
    cres = cmox_cipher_cleanup(h);
    if (cres != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.cipher_text_gcm_aead, sizeof(ctx->buffers.cipher_text_gcm_aead));
        ctx->tmp.computed_size = 0;
        return GCM_ENC_ERR_CLEANUP;
    }
    secure_zero(ctx->tmp.local_tag, sizeof(ctx->tmp.local_tag));
    return GCM_ENC_SUCCESS;
}
/**
 * @brief Encrypt the current plaintext with the single-call AES-GCM AEAD path.
 * @param ctx Crypto context containing GCM key, IV and plaintext.
 * @return Single-call AES-GCM encryption status.
 */
gcm_aead_single_status_t encrypt_aead_gcm_singlecall(crypto_ctx_t *ctx)
{
    /* Check the GCM result and authentication tag before using plaintext. */
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
/**
 * @brief Generate the transfer IV and encrypt through the AES-GCM sender path.
 * @param ctx Sender crypto context.
 * @return AES-GCM sender status.
 */
gcm_sender_status_t generate_nonce_iv_encrypt_gcm_sender(crypto_ctx_t *ctx)
{
    /* Check the GCM result and authentication tag before using plaintext. */
    gen_iv_status_t iv_rc;
    gcm_enc_status_t gcm_rc;
    gcm_aead_single_status_t aead_rc;
    iv_rc = generate_nonce_and_iv_per_transfer(ctx);
    if (iv_rc != GEN_IV_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_gcm_sender: IV generation failed (rc=%d)\r\n",
            iv_rc
        );
        return GCM_SENDER_ERR_GEN_IV;
    }
    gcm_rc = encrypt_aes_gcm_aead(ctx);
    if (gcm_rc != GCM_ENC_SUCCESS) {
        log_printf(
            "[ERROR] generate_nonce_iv_encrypt_gcm_sender: GCM stream encrypt failed (rc=%d)\r\n",
            gcm_rc
        );
        secure_zero(ctx->IV, sizeof(ctx->IV));
        return GCM_SENDER_ERR_GCM_STREAM;
    }
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
/* AES-GCM receiver authentication/decryption path. */
/**
 * @brief Decrypt and authenticate the received packet with streaming AES-GCM.
 * @param ctx Receiver crypto context.
 * @return Streaming AES-GCM decryption status.
 */
gcm_dec_status_t decrypt_verify_aes_gcm_receiver(crypto_ctx_t *ctx)
{
    /* Check the GCM result and authentication tag before using plaintext. */
    cmox_cipher_retval_t cres;
    size_t out_len = 0;
    ctx->tmp.computed_size = 0;
    if (ctx->auth.ciphertext_len == 0) {
        return GCM_DEC_ERR_BAD_INPUT;
    }
    if (ctx->auth.ciphertext_len > sizeof(ctx->auth.ciphertext_sender)) {
        return GCM_DEC_ERR_BAD_INPUT;
    }
    if (is_all_zero_bytes(ctx->derived.aes_key_receiver, sizeof(ctx->derived.aes_key_receiver)) ||
        is_all_zero_bytes(ctx->IV_GCM, sizeof(ctx->IV_GCM))) {
        return GCM_DEC_ERR_KEY_OR_IV;
    }
    cmox_gcm_handle_t gcm_ctx;
    cmox_cipher_handle_t *h =
        cmox_gcm_construct(&gcm_ctx, CMOX_AESFAST_GCMFAST_DEC);
    if (h == NULL) {
        return GCM_DEC_ERR_CONSTRUCT;
    }
    cres = cmox_cipher_init(h);
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_INIT;
    }
    cres = cmox_cipher_setKey(h,
                              ctx->derived.aes_key_receiver,
                              (cmox_cipher_keyLen_t)sizeof(ctx->derived.aes_key_receiver));
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETKEY;
    }
    cres = cmox_cipher_setIV(h, ctx->IV_GCM, sizeof(ctx->IV_GCM));
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETIV;
    }
    cres = cmox_cipher_setTagLen(h, (size_t)AES_GCM_TAG_SIZE);
    if (cres != CMOX_CIPHER_SUCCESS) {
        (void)cmox_cipher_cleanup(h);
        return GCM_DEC_ERR_SETTAGLEN;
    }
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
    uint32_t fault_check = 0;
    cres = cmox_cipher_verifyTag(h, ctx->auth.tag_sender, &fault_check);
    cmox_cipher_retval_t cleanup_ret = cmox_cipher_cleanup(h);
    if (cres != CMOX_CIPHER_AUTH_SUCCESS ||
        fault_check != CMOX_CIPHER_AUTH_SUCCESS) {
        secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
        return GCM_DEC_ERR_VERIFY;
    }
    if (cleanup_ret != CMOX_CIPHER_SUCCESS) {
        secure_zero(ctx->buffers.decrypted_bytes, sizeof(ctx->buffers.decrypted_bytes));
        return GCM_DEC_ERR_CLEANUP;
    }
    ctx->tmp.computed_size = out_len;
    return GCM_DEC_SUCCESS;
}
/**
 * @brief Decrypt and authenticate the received packet with single-call AES-GCM.
 * @param ctx Receiver crypto context.
 * @return Single-call AES-GCM decryption status.
 */
gcm_aead_dec_status_t decrypt_aead_gcm_singlecall(crypto_ctx_t *ctx)
{
    /* Check the GCM result and authentication tag before using plaintext. */
    cmox_cipher_retval_t cres;
    size_t in_len = 0;
    size_t plain_len_local = 0;
    const size_t tag_len = (size_t)AES_GCM_TAG_SIZE;
    ctx->tmp.computed_size = 0;
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
    if (is_all_zero_bytes(ctx->derived.aes_key_receiver,
                          sizeof(ctx->derived.aes_key_receiver)) ||
        is_all_zero_bytes(ctx->IV_GCM, sizeof(ctx->IV_GCM))) {
        return GCM_AEAD_DEC_ERR_KEY_OR_IV;
    }
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
    if (cres != CMOX_CIPHER_AUTH_SUCCESS) {
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, in_len);
        return GCM_AEAD_DEC_ERR_AEAD_FAIL;
    }
    if (plain_len_local != ctx->auth.ciphertext_len) {
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, in_len);
        return GCM_AEAD_DEC_ERR_LENGTH;
    }
    ctx->tmp.computed_size = ctx->auth.ciphertext_len;
    secure_zero(ctx->buffers.cipher_text_gcm_aead_receiver, in_len);
    return GCM_AEAD_DEC_SUCCESS;
}
/**
 * @brief Authenticate/decrypt the GCM packet and rebuild the ADC samples.
 * @param ctx Receiver crypto context.
 * @return AES-GCM receiver status.
 */
gcm_receiver_status_t decrypt_verify_recombine_gcm_receiver(crypto_ctx_t *ctx)
{
    /* Check the GCM result and authentication tag before using plaintext. */
    gcm_dec_status_t gcm_rc;
    gcm_aead_dec_status_t aead_rc;
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
    if (ctx->tmp.computed_size == 0U) {
        log_printf(
            "[ERROR] decrypt_verify_recombine_gcm_receiver: decrypted size is zero\r\n"
        );
        secure_zero(ctx->buffers.decrypted_bytes,
                    sizeof(ctx->buffers.decrypted_bytes));
        return GCM_RECEIVER_ERR_EMPTY_PLAINTEXT;
    }
    recombine_adc_values(ctx);
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
/* Decrypted telemetry reconstruction and context initialization. */
/**
 * @brief Rebuild 32-bit ADC values from the decrypted plaintext bytes.
 * @param ctx Crypto context containing decrypted bytes and ADC output.
 * @return ADC recombination status.
 */
recombine_status_t recombine_adc_values(crypto_ctx_t *ctx)
{
    /* Recombine bytes in the same order as the original samples. */
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
/**
 * @brief Reset the crypto context and apply the project default configuration.
 * @param ctx Crypto context to initialize.
 */
void crypto_ctx_init(crypto_ctx_t *ctx)
{
    ctx->cfg.adc_sample_count  = ADC_SAMPLE_COUNT;
    ctx->cfg.bytes_per_sample  = BYTES_PER_SAMPLE;
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
    for (size_t i = 0; i < CMOX_ECC_SECP256R1_SECRET_LEN; i++) {
        ctx->tmp.computed_secret[i]          = 0U;
        ctx->tmp.computed_secret_receiver[i] = 0U;
    }
    for (size_t i = 0; i < GCM_GCM_TAG_SIZE; i++) {
        ctx->tmp.local_tag[i] = 0U;
    }
    ctx->tmp.computed_size = 0U;
    ctx->tmp.plain_len     = sizeof(ctx->buffers.plain_text_bytes);
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
