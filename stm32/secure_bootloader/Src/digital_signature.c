/**
 * @file digital_signature.c
 * @brief Secure-boot manifest hashing, Ed25519 verification and protected storage.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>
#include "digital_signature.h"
#include "flash_protection.h"
#include <string.h>
/* Shared signed-manifest and Ed25519 key contexts. */
signed_manifest_t g_signed_manifest;
sig_keys_ctx_t g_sig_keys;
/*
 * Dynamic Ed25519 OTP record layout.
 * Four 32-byte OTP blocks hold a header, 64-byte private representation
 * and the 32-byte public trust anchor. OTP lock bytes are excluded.
 */
#define STM32F411_OTP_DATA_START          ((uintptr_t)0x1FFF7800U)
#define STM32F411_OTP_DATA_END            ((uintptr_t)0x1FFF79FFU)
#define STM32F411_OTP_BLOCK_SIZE          ((size_t)32U)
#define STM32F411_OTP_BLOCK_COUNT         ((size_t)16U)
#define ED25519_OTP_RECORD_BLOCKS         ((size_t)4U)
#define ED25519_OTP_RECORD_SIZE           (ED25519_OTP_RECORD_BLOCKS * STM32F411_OTP_BLOCK_SIZE)
#define ED25519_OTP_HEADER_SIZE           STM32F411_OTP_BLOCK_SIZE
#define ED25519_OTP_PRIVATE_OFFSET        ((uintptr_t)0x20U)
#define ED25519_OTP_PUBLIC_OFFSET         ((uintptr_t)0x60U)
#define ED25519_OTP_RECORD_VERSION        ((uint8_t)0x01U)
#define ED25519_OTP_RECORD_MARKER         ((uint8_t)0xA5U)
/* Header used to discover a committed Ed25519 record in OTP. */
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
/* Validate one candidate Ed25519 OTP record. */
static bool otp_record_is_valid_at(uintptr_t record_addr)
{
    /* Return a failure status if any validation check fails. */
    const uint8_t *header;
    const uint8_t *priv;
    const uint8_t *pub;
    if ((record_addr < STM32F411_OTP_DATA_START) ||
        (record_addr > (STM32F411_OTP_DATA_END + 1U - ED25519_OTP_RECORD_SIZE)) ||
        (((record_addr - STM32F411_OTP_DATA_START) % STM32F411_OTP_BLOCK_SIZE) != 0U)) {
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
/* Scan the OTP data blocks for the committed Ed25519 record. */
static bool otp_find_ed25519_record(uintptr_t *record_addr_out,
                                    uintptr_t *public_addr_out)
{
    /* Check the return status before continuing to the next step. */
    for (size_t first_block = 0U;
         first_block <= (STM32F411_OTP_BLOCK_COUNT - ED25519_OTP_RECORD_BLOCKS);
         ++first_block) {
        const uintptr_t record_addr =
            STM32F411_OTP_DATA_START + (first_block * STM32F411_OTP_BLOCK_SIZE);
        if (!otp_record_is_valid_at(record_addr)) {
            continue;
        }
        if (record_addr_out != NULL) {
            *record_addr_out = record_addr;
        }
        if (public_addr_out != NULL) {
            *public_addr_out = record_addr + ED25519_OTP_PUBLIC_OFFSET;
        }
        return true;
    }
    return false;
}
/* Locate the provisioned Ed25519 public trust anchor. */
static bool otp_public_key_is_programmed(uintptr_t *public_addr_out)
{
    /* Validate the key data before using it in crypto operations. */
    uintptr_t public_addr = 0U;
    if (!otp_find_ed25519_record(NULL, &public_addr)) {
        return false;
    }
    if (public_addr_out != NULL) {
        *public_addr_out = public_addr;
    }
    return true;
}
/* Check whether both the manifest and OTP trust anchor are already provisioned. */
static manifest_store_status_t check_existing_manifest_and_pubkey(bool *already_present);
/* Word-aligned size used by the Flash read/write helpers. */
static inline uint32_t get_manifest_word_count(void);
/* Load canonical authenticity-test inputs from Flash and OTP. */
static verify_auth_negtest_status_t load_auth_inputs_from_flash(
    signed_manifest_t *sm_local,
    uint8_t *msg,
    uint8_t *pubkey,
    uint32_t *pubkey_read_buf);
/* Firmware hashing and manifest creation. */
/**
 * @brief Hash a contiguous firmware image with SHA-256.
 * @param firmware_addr Start address of the firmware image.
 * @param firmware_size Firmware size in bytes.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size; must match SHA256_HASH_SIZE.
 * @return Firmware hash status.
 */
firmware_hash_status_t hash_firmware_binary(uint32_t firmware_addr,
                                           uint32_t firmware_size,
                                           uint8_t *out_hash,
                                           size_t out_len)
{
    /* Check the digest result before using it in verification. */
    cmox_hash_retval_t retval;
    size_t out_len_local;
    if (out_hash == NULL) {
        log_printf("hash_firmware_binary: NULL output pointer\r\n");
        return FIRMWARE_HASH_ERR_NULL_INPUT;
    }
    if (out_len != SHA256_HASH_SIZE) {
        log_printf("hash_firmware_binary: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, SHA256_HASH_SIZE);
        return FIRMWARE_HASH_ERR_BAD_LENGTH;
    }
    if (firmware_size == 0 || firmware_size > 0x00100000) {
        log_printf("hash_firmware_binary: invalid firmware size (%lu bytes)\r\n",
                   (unsigned long)firmware_size);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }
    const uint8_t *firmware_data = (const uint8_t *)firmware_addr;
    out_len_local = out_len;
    retval = cmox_hash_compute(
        CMOX_SHA256_ALGO,
        firmware_data,
        firmware_size,
        out_hash,
        out_len,
        &out_len_local
    );
    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        log_printf("hash_firmware_binary: SHA-256 computation failed (retval=%d, out_len=%lu)\r\n",
                   retval, (unsigned long)out_len_local);
        memset(out_hash, 0, out_len);
        return FIRMWARE_HASH_ERR_HASH_FAILED;
    }
    log_printf("✓ Firmware hashed successfully: %lu bytes → 32-byte SHA-256\r\n",
               (unsigned long)firmware_size);
    return FIRMWARE_HASH_SUCCESS;
}
/**
 * @brief Hash the configured firmware image and print the digest.
 * @return Firmware hash status.
 */
firmware_hash_status_t example_hash_my_firmware(void)
{
    uint8_t firmware_hash[32];  // SHA-256 output
    firmware_hash_status_t result = hash_firmware_chunked(
        FIRMWARE_START_ADDR,
        FIRMWARE_SIZE,
        firmware_hash,
        sizeof(firmware_hash)
    );
    if (result == FIRMWARE_HASH_SUCCESS)
    {
        log_printf("Firmware hash computed:\r\n");
        for (int i = 0; i < 32; i++) {
            log_printf("%02X", firmware_hash[i]);
        }
        log_printf("\r\n");
        return FIRMWARE_HASH_SUCCESS;
    }
    else
    {
        log_printf("ERROR: Failed to hash firmware (code=%d)\r\n", result);
        return result;   // propagate exact error upward
    }
}
/**
 * @brief Hash a firmware image with streaming SHA-256 updates.
 * @param firmware_addr Start address of the firmware image.
 * @param firmware_size Firmware size in bytes.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @return Firmware hash status.
 */
firmware_hash_status_t hash_firmware_chunked(uint32_t firmware_addr,
                                             uint32_t firmware_size,
                                             uint8_t *out_hash,
                                             size_t out_len)
{
    /* Check the digest result before using it in verification. */
    cmox_hash_retval_t retval;
    cmox_sha256_handle_t sha_ctx;
    cmox_hash_handle_t *hash_handle;
    uint32_t bytes_remaining = firmware_size;
    uint32_t current_addr = firmware_addr;
    size_t hash_len = 0;
    if (out_hash == NULL) {
        log_printf("hash_firmware_chunked: NULL output pointer\r\n");
        return FIRMWARE_HASH_ERR_NULL_INPUT;
    }
    if (out_len != SHA256_HASH_SIZE) {
        log_printf("hash_firmware_chunked: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, SHA256_HASH_SIZE);
        return FIRMWARE_HASH_ERR_BAD_LENGTH;
    }
    if (firmware_size == 0 || firmware_size > 0x00100000U) {
        log_printf("hash_firmware_chunked: invalid firmware size (%lu bytes)\r\n",
                   (unsigned long)firmware_size);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }
    hash_handle = cmox_sha256_construct(&sha_ctx);
    if (hash_handle == NULL) {
        log_printf("hash_firmware_chunked: sha construct failed\r\n");
        return FIRMWARE_HASH_ERR_HASH_FAILED;
    }
    retval = cmox_hash_init(hash_handle);
    if (retval != CMOX_HASH_SUCCESS) {
        cmox_hash_cleanup(hash_handle);
        log_printf("hash_firmware_chunked: hash init failed (%d)\r\n", retval);
        return FIRMWARE_HASH_ERR_HASH_FAILED;
    }
    log_printf("Hashing firmware in chunks...\r\n");
    while (bytes_remaining > 0U) {
        size_t chunk_size = (bytes_remaining > HASH_CHUNK_SIZE) ? HASH_CHUNK_SIZE : (size_t)bytes_remaining;
        const uint8_t *chunk_data = (const uint8_t *)current_addr;
        retval = cmox_hash_append(hash_handle, chunk_data, chunk_size);
        if (retval != CMOX_HASH_SUCCESS) {
            log_printf("hash_firmware_chunked: hash append failed at offset 0x%08lX (ret=%d)\r\n",
                       (unsigned long)(current_addr - firmware_addr), retval);
            cmox_hash_cleanup(hash_handle);
            memset(out_hash, 0, out_len);
            return FIRMWARE_HASH_ERR_HASH_FAILED;
        }
        current_addr  += (uint32_t)chunk_size;
        bytes_remaining -= (uint32_t)chunk_size;
        if (((firmware_size - bytes_remaining) % (16U * 1024U)) == 0U) {
            log_printf("  Hashed %lu / %lu bytes\r\n",
                       (unsigned long)(firmware_size - bytes_remaining),
                       (unsigned long)firmware_size);
        }
    }
    retval = cmox_hash_generateTag(hash_handle, out_hash, &hash_len);
    if (retval != CMOX_HASH_SUCCESS || hash_len != SHA256_HASH_SIZE) {
        log_printf("hash_firmware_chunked: finalization failed (ret=%d, len=%lu)\r\n",
                   retval, (unsigned long)hash_len);
        cmox_hash_cleanup(hash_handle);
        memset(out_hash, 0, out_len);
        return FIRMWARE_HASH_ERR_HASH_FAILED;
    }
    retval = cmox_hash_cleanup(hash_handle);
    if (retval != CMOX_HASH_SUCCESS) {
        log_printf("hash_firmware_chunked: cleanup returned %d (non-fatal)\r\n", retval);
    }
    log_printf("✓ Firmware hashed successfully (chunked mode)\r\n");
    return FIRMWARE_HASH_SUCCESS;
}
/**
 * @brief Populate the firmware manifest and calculate its SHA-256 digest.
 * @param manifest Manifest to populate.
 * @return Firmware hash status.
 */
firmware_hash_status_t create_and_hash_manifest(firmware_manifest_t *manifest)
{
    /* Check the digest result before using it in verification. */
    firmware_hash_status_t rc;
    if (FIRMWARE_SIZE == 0 || FIRMWARE_SIZE > 0x00100000U) {
        log_printf("create_and_hash_manifest: invalid FIRMWARE_SIZE (%lu)\r\n",
                   (unsigned long)FIRMWARE_SIZE);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }
    manifest->version = FIRMWARE_VERSION;
    manifest->firmware_size = FIRMWARE_SIZE;
    manifest->rollback_counter = 5;
    memset(manifest->sha256_hash, 0, sizeof(manifest->sha256_hash));
    rc = hash_firmware_chunked(
        FIRMWARE_START_ADDR,
        manifest->firmware_size,
        manifest->sha256_hash,
        sizeof(manifest->sha256_hash)
    );
    if (rc != FIRMWARE_HASH_SUCCESS) {
        log_printf("create_and_hash_manifest: firmware hashing failed (code=%d)\r\n", rc);
        memset(manifest->sha256_hash, 0, sizeof(manifest->sha256_hash));
        return rc;
    }
    {
        uint8_t allzero = 1;
        for (size_t i = 0; i < sizeof(manifest->sha256_hash); ++i) {
            if (manifest->sha256_hash[i] != 0) { allzero = 0; break; }
        }
        if (allzero) {
            log_printf("create_and_hash_manifest: computed hash is all zeros - abort\r\n");
            return FIRMWARE_HASH_ERR_HASH_FAILED;
        }
    }
    log_printf("✓ Manifest ready:\r\n");
    log_printf("  Version:        0x%08X\r\n", manifest->version);
    log_printf("  Size:           %lu bytes\r\n", (unsigned long)manifest->firmware_size);
    log_printf("  Rollback count: %lu\r\n", (unsigned long)manifest->rollback_counter);
    log_printf("  Hash: ");
    for (int i = 0; i < (int)sizeof(manifest->sha256_hash); ++i) {
        log_printf("%02X", manifest->sha256_hash[i]);
    }
    log_printf("\r\n");
    return FIRMWARE_HASH_SUCCESS;
}
/* Write one 32-bit manifest field in canonical big-endian order. */
static inline bool write_u32_be(uint8_t *buf, size_t buf_len, size_t offset, uint32_t v)
{
    /* Check the write result before continuing. */
    if (!buf || offset > buf_len || (buf_len - offset) < 4u)
        return false;
    buf[offset + 0] = (uint8_t)(v >> 24);
    buf[offset + 1] = (uint8_t)(v >> 16);
    buf[offset + 2] = (uint8_t)(v >> 8);
    buf[offset + 3] = (uint8_t)(v);
    return true;
}
/**
 * @brief Serialize manifest fields into the canonical big-endian buffer.
 * @param m Manifest to serialize.
 * @param out Serialized output buffer.
 * @param out_len Output buffer size.
 * @return Serializer result flag used by the existing boot flow.
 */
bool serialize_manifest(firmware_manifest_t *m,
                        uint8_t *out,
                        size_t out_len)
{
    /* Check the return status before continuing to the next step. */
    if (!m || !out || out_len < MANIFEST_LEN)
        return true;
    memset(out, 0, MANIFEST_LEN);
    if (!write_u32_be(out, out_len, OFF_VERSION, m->version)) return false;
    if (!write_u32_be(out, out_len, OFF_FW_SIZE, m->firmware_size)) return false;
    memcpy(out + OFF_HASH, m->sha256_hash, SHA256_LEN);
    if (!write_u32_be(out, out_len, OFF_ROLLBK, m->rollback_counter)) return false;
    return false;
}
/* Reject erased or dummy-only application contents before signing. */
static sector5_check_status_t check_sector5_has_real_firmware(void)
{
    /* Return a failure status if any validation check fails. */
    const uint32_t *flash_ptr = (const uint32_t *)APP_SECTOR5_START_ADDR;
    const size_t word_count = APP_SECTOR5_SIZE_BYTES / sizeof(uint32_t);
    bool all_ff = true;
    bool all_00 = true;
    bool found_real_word = false;
    for (size_t i = 0; i < word_count; ++i) {
        uint32_t w = flash_ptr[i];
        if (w != 0xFFFFFFFFUL) {
            all_ff = false;
        }
        if (w != 0x00000000UL) {
            all_00 = false;
        }
        if ((w != 0xFFFFFFFFUL) && (w != 0x00000000UL)) {
            found_real_word = true;
            break;
        }
    }
    if (all_ff) {
        return SECTOR5_CHECK_ALL_FF;
    }
    if (all_00) {
        return SECTOR5_CHECK_ALL_00;
    }
    if (!found_real_word) {
        return SECTOR5_CHECK_DUMMY_00_FF_ONLY;
    }
    return SECTOR5_CHECK_OK;
}
/* Manifest signing and boot-time authenticity verification. */
/**
 * @brief Create the firmware manifest and sign its canonical form with Ed25519.
 * @param ecc_ctx Prepared Ed25519 context.
 * @param privkey Private signing-key representation.
 * @param privkey_len Private-key buffer length.
 * @param manifest Manifest to create and sign.
 * @param out_sig Signature output buffer.
 * @param out_sig_len Signature buffer capacity and returned length.
 * @return Manifest-signing status.
 */
sign_manifest_status_t sign_manifest_and_write_sig(sig_keys_ctx_t *ecc_ctx,
                                                   uint8_t *privkey,
                                                   size_t privkey_len,
                                                   firmware_manifest_t *manifest,
                                                   uint8_t *out_sig,
                                                   size_t *out_sig_len)
{
    uint8_t msg[MANIFEST_LEN];
    cmox_ecc_retval_t ret;
    firmware_hash_status_t mrc;
    bool already_present = false;
    manifest_store_status_t precheck_ret;
    if (ecc_ctx == NULL || privkey == NULL || manifest == NULL || out_sig == NULL || out_sig_len == NULL) {
        return SIGN_MANIFEST_ERR_NULL_PTR;
    }
    if (privkey_len < ED25519_PRIVLEN) {
        return SIGN_MANIFEST_ERR_BAD_PRIVKEY_LEN;
    }
        {
            sector5_check_status_t s5_rc = check_sector5_has_real_firmware();
            if (s5_rc != SECTOR5_CHECK_OK) {
            	if (s5_rc == SECTOR5_CHECK_ALL_FF) {
            	    log_printf("[ERROR] sign_manifest_and_write_sig: can't hash firmware because sector 5 is empty/erased (all 0xFF)\r\n");
            	} else if (s5_rc == SECTOR5_CHECK_ALL_00) {
            	    log_printf("[ERROR] sign_manifest_and_write_sig: can't hash firmware because sector 5 is empty/dummy (all 0x00)\r\n");
            	} else {
            	    log_printf("[ERROR] sign_manifest_and_write_sig: can't hash firmware because sector 5 contains only dummy values (0x00/0xFF)\r\n");
            	}
                return SIGN_MANIFEST_ERR_MANIFEST_CREATE;
            }
        }
/* Avoid regenerating a manifest when Flash and the OTP trust anchor are already provisioned. */
        precheck_ret = check_existing_manifest_and_pubkey(&already_present);
        if (precheck_ret != 0) {
            log_printf("[ERROR] sign_manifest_and_write_sig: precheck failed (%d)\r\n",
                       (int)precheck_ret);
            return MANIFEST_PUBKEY_STORED_FLASH_ERR;
        }
        if (already_present) {
            log_printf("[INFO] Already provisioned; skipping manifest/signature generation\r\n");
            *out_sig_len = 0;
            secure_zero(out_sig, ED25519_SIGLEN);
            return SIGN_MANIFEST_SUCCESS;   // or better: SIGN_MANIFEST_ALREADY_PRESENT
        }
        if (*out_sig_len < ED25519_SIGLEN) {
                return SIGN_MANIFEST_ERR_SIGBUF_TOO_SMALL;
            }
    mrc = create_and_hash_manifest_with_padding(manifest,true);
    if (mrc != FIRMWARE_HASH_SUCCESS) {
        log_printf("sign_manifest_and_write_sig: create_and_hash_manifest failed (code=%d)\r\n", mrc);
        return SIGN_MANIFEST_ERR_MANIFEST_CREATE;
    }
    if (serialize_manifest(manifest, msg, MANIFEST_LEN) != FIRMWARE_HASH_SUCCESS) {
        log_printf("sign_manifest_and_write_sig: serialize_manifest failed\r\n");
        memset(msg, 0, MANIFEST_LEN);
        return SIGN_MANIFEST_ERR_SERIALIZE;
    }
    {
        int all_zero = 1;
        for (size_t i = 0; i < MANIFEST_LEN; ++i) {
            if (msg[i] != 0) { all_zero = 0; break; }
        }
        if (all_zero) {
            memset(msg, 0, MANIFEST_LEN);
            return SIGN_MANIFEST_ERR_SERIALIZE;
        }
    }
/* Sign the canonical manifest bytes with the temporary RAM private-key copy. */
    ret = cmox_eddsa_sign(
    	&ecc_ctx->ecc,
		CMOX_ECC_ED25519_HIGHMEM,
        privkey,
        privkey_len,
        msg,
        MANIFEST_LEN,
        out_sig,
        out_sig_len
    );
    {
        volatile uint8_t *p = msg;
        for (size_t i = 0; i < MANIFEST_LEN; ++i) p[i] = 0;
    }
    if (ret != CMOX_ECC_SUCCESS) {
        log_printf("sign_manifest_and_write_sig: cmox_eddsa_sign failed (ret=%d)\r\n", ret);
        return SIGN_MANIFEST_ERR_SIGN_FAIL;
    }
    if (*out_sig_len != ED25519_SIGLEN) {
        if (*out_sig_len > 0 && *out_sig_len <= ED25519_SIGLEN) {
            volatile uint8_t *q = out_sig;
            for (size_t i = 0; i < *out_sig_len; ++i) q[i] = 0;
        }
        log_printf("sign_manifest_and_write_sig: signature length mismatch (%lu)\r\n", (unsigned long)*out_sig_len);
        return SIGN_MANIFEST_ERR_SIGLEN_MISMATCH;
    }
    return SIGN_MANIFEST_SUCCESS;
}
/* Check whether a buffer contains one repeated byte value. */
static bool buffer_is_all_value(const uint8_t *buf, size_t len, uint8_t value)
{
    /* Keep lengths within the allocated buffer capacity. */
    if (buf == NULL || len == 0U) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (buf[i] != value) {
            return false;
        }
    }
    return true;
}
/**
 * @brief Verify the stored manifest signature using the OTP Ed25519 trust anchor.
 * @param ecc_ctx Prepared Ed25519 context.
 * @param out_sm Optional output for the verified signed manifest.
 * @return Authenticity verification status.
 */
verify_auth_status_t verify_manifest_authenticity_from_flash(sig_keys_ctx_t *ecc_ctx,
                                                             signed_manifest_t *out_sm)
{
    signed_manifest_t sm_local;
    uint8_t msg[MANIFEST_LEN];
    uint8_t pubkey[ED25519_PUBLEN];
    const uint8_t *otp_pubkey;
    uintptr_t otp_public_addr = 0U;
    uint32_t fault_check = 0U;
    uint32_t read_ret;
    uint32_t manifest_word_count;
    cmox_ecc_retval_t ret;
    bool already_present = false;
    manifest_store_status_t precheck_ret;
    log_printf("[INFO] Verifying manifest authenticity using OTP public key...\r\n");
    delay_ms(5);
    if (ecc_ctx == NULL) {
        return VERIFY_AUTH_ERR_NULL_PTR;
    }
    memset(&sm_local, 0, sizeof(sm_local));
    memset(msg, 0, sizeof(msg));
    memset(pubkey, 0, sizeof(pubkey));
    manifest_word_count = get_manifest_word_count();
    precheck_ret = check_existing_manifest_and_pubkey(&already_present);
    if (precheck_ret != MANIFEST_STORE_SUCCESS) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "precheck failed (%d)\r\n",
            (int)precheck_ret);
        return VERIFY_AUTH_ERR_FLASH_READ;
    }
    if (!already_present) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "manifest/OTP public key not provisioned\r\n");
        return VERIFY_AUTH_ERR_NOT_PROVISIONED;
    }
    read_ret = flash_read_data(
        MANIFEST_FLASH_ADDR,
        (uint32_t *)&sm_local,
        manifest_word_count);
    if (read_ret != FLASH_READ_OK) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "flash_read_data(manifest) failed: 0x%08lX\r\n",
            (unsigned long)read_ret);
        secure_zero(&sm_local, sizeof(sm_local));
        return VERIFY_AUTH_ERR_FLASH_READ;
    }
    if (!otp_find_ed25519_record(NULL, &otp_public_addr)) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "dynamic OTP Ed25519 record disappeared\r\n");
        secure_zero(&sm_local, sizeof(sm_local));
        return VERIFY_AUTH_ERR_NOT_PROVISIONED;
    }
    otp_pubkey = (const uint8_t *)otp_public_addr;
    memcpy(pubkey, otp_pubkey, ED25519_PUBLEN);
    if (buffer_is_all_value(sm_local.signature, ED25519_SIGLEN, 0x00U) ||
        buffer_is_all_value(sm_local.signature, ED25519_SIGLEN, 0xFFU)) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "signature empty/erased\r\n");
        secure_zero(&sm_local, sizeof(sm_local));
/* The OTP-derived public-key copy is no longer needed after verification. */
        secure_zero(pubkey, sizeof(pubkey));
        return VERIFY_AUTH_ERR_SIG_EMPTY;
    }
    if (buffer_is_all_value(pubkey, ED25519_PUBLEN, 0x00U) ||
        buffer_is_all_value(pubkey, ED25519_PUBLEN, 0xFFU)) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "OTP public key empty/erased\r\n");
        secure_zero(&sm_local, sizeof(sm_local));
        secure_zero(pubkey, sizeof(pubkey));
        return VERIFY_AUTH_ERR_PUBKEY_EMPTY;
    }
    if (serialize_manifest(
            &sm_local.manifest,
            msg,
            MANIFEST_LEN) != FIRMWARE_HASH_SUCCESS) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "serialize_manifest failed\r\n");
        secure_zero(&sm_local, sizeof(sm_local));
        secure_zero(msg, sizeof(msg));
        secure_zero(pubkey, sizeof(pubkey));
        return VERIFY_AUTH_ERR_SERIALIZE;
    }
    if (buffer_is_all_value(msg, MANIFEST_LEN, 0x00U)) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "serialized manifest is all zero\r\n");
        secure_zero(&sm_local, sizeof(sm_local));
        secure_zero(msg, sizeof(msg));
        secure_zero(pubkey, sizeof(pubkey));
        return VERIFY_AUTH_ERR_SERIALIZE;
    }
/* Verify the signature before trusting the manifest digest. */
    ret = cmox_eddsa_verify(
              &ecc_ctx->ecc,
              CMOX_ECC_ED25519_HIGHMEM,
              pubkey,
              ED25519_PUBLEN,
              msg,
              MANIFEST_LEN,
              sm_local.signature,
              ED25519_SIGLEN,
              &fault_check);
    cmox_ecc_cleanup(&ecc_ctx->ecc);
    secure_zero(msg, sizeof(msg));
    secure_zero(pubkey, sizeof(pubkey));
    if (ret != CMOX_ECC_AUTH_SUCCESS) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "cmox_eddsa_verify failed (ret=%d)\r\n",
            (int)ret);
        secure_zero(&sm_local, sizeof(sm_local));
        if ((ret == CMOX_ECC_AUTH_FAIL) ||
            (ret == CMOX_ECC_ERR_INVALID_SIGNATURE) ||
            (ret == CMOX_ECC_ERR_INVALID_PUBKEY)) {
            return VERIFY_AUTH_ERR_AUTH_FAIL;
        }
        return VERIFY_AUTH_ERR_ECC_VERIFY;
    }
    if (fault_check != CMOX_ECC_AUTH_SUCCESS) {
        log_printf(
            "[ERROR] verify_manifest_authenticity_from_flash: "
            "fault_check mismatch (0x%08lX)\r\n",
            (unsigned long)fault_check);
        secure_zero(&sm_local, sizeof(sm_local));
        return VERIFY_AUTH_ERR_FAULT_CHECK;
    }
    log_printf("[OK] Manifest authenticity verified successfully with OTP trust anchor\r\n");
    log_printf("[OK] Authenticity verification passed\r\n");
    delay_ms(5);
    if (out_sm != NULL) {
        memcpy(out_sm, &sm_local, sizeof(signed_manifest_t));
    }
    secure_zero(&sm_local, sizeof(sm_local));
    return VERIFY_AUTH_SUCCESS;
}
/* Check whether a hash buffer contains one repeated byte value. */
static bool hash_buffer_is_all_value(const uint8_t *buf, size_t len, uint8_t value)
{
    /* Check the digest result before using it in verification. */
    if (buf == NULL || len == 0U) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        if (buf[i] != value) {
            return false;
        }
    }
    return true;
}
/* Constant-time digest comparison; zero means equal. */
static int hash_ct_memcmp(const void *a_, const void *b_, size_t n)
{
    /* Check the digest result before using it in verification. */
    const uint8_t *a = (const uint8_t *)a_;
    const uint8_t *b = (const uint8_t *)b_;
    uint8_t diff = 0U;
    for (size_t i = 0; i < n; ++i) {
        diff |= (uint8_t)(a[i] ^ b[i]);
    }
    return (diff != 0U);
}
/**
 * @brief Hash the application image in Sector 5 without rebuilding the manifest.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @param use_padding Include the configured Flash-word padding when true.
 * @return Firmware hash status.
 */
firmware_hash_status_t hash_sector5_firmware_only(uint8_t *out_hash,
                                                  size_t out_len,
                                                  bool use_padding)
{
    /* Check the digest result before using it in verification. */
    firmware_hash_status_t rc;
    uint32_t padded_size;
    uint32_t padding_bytes;
    if (out_hash == NULL) {
        log_printf("[ERROR] hash_sector5_firmware_only: NULL output buffer\r\n");
        return FIRMWARE_HASH_ERR_NULL_INPUT;
    }
    if (out_len != SHA256_HASH_SIZE) {
        log_printf("[ERROR] hash_sector5_firmware_only: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, SHA256_HASH_SIZE);
        return FIRMWARE_HASH_ERR_BAD_LENGTH;
    }
    if (FIRMWARE_SIZE == 0U || FIRMWARE_SIZE > MAX_FIRMWARE_SIZE) {
        log_printf("[ERROR] hash_sector5_firmware_only: invalid FIRMWARE_SIZE (%lu)\r\n",
                   (unsigned long)FIRMWARE_SIZE);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }
    if (FIRMWARE_SIZE > APP_SLOT_SIZE_BYTES) {
        log_printf("[ERROR] hash_sector5_firmware_only: firmware size exceeds Sector 5 slot\r\n");
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }
    if (use_padding) {
        padded_size = ALIGN_UP(FIRMWARE_SIZE, FLASH_WORD_SIZE);
        padding_bytes = padded_size - FIRMWARE_SIZE;
    } else {
        padded_size = FIRMWARE_SIZE;
        padding_bytes = 0U;
    }
    memset(out_hash, 0, out_len);
    log_printf("\r\n");
    log_printf("╔══════════════════════════════════════════════════════════════════╗\r\n");
    log_printf("║         SECTOR 5 FIRMWARE HASHING                               ║\r\n");
    log_printf("╚══════════════════════════════════════════════════════════════════╝\r\n");
    delay_ms(5);
    rc = hash_firmware_with_padding(
             APP_SLOT_START_ADDR,
             FIRMWARE_SIZE,
             out_hash,
             out_len,
             use_padding
         );
    if (rc != FIRMWARE_HASH_SUCCESS) {
        log_printf("[ERROR] hash_sector5_firmware_only: hashing failed: %s (%d)\r\n",
                   firmware_hash_status_str(rc), (int)rc);
        memset(out_hash, 0, out_len);
        return rc;
    }
    log_printf("[OK] Sector 5 firmware hash computed successfully\r\n");
    delay_ms(5);
    return FIRMWARE_HASH_SUCCESS;
}
/* Runtime application-image integrity verification. */
/**
 * @brief Compare the runtime Sector 5 hash with the authenticated manifest digest.
 * @param verified_sm Previously authenticated signed manifest.
 * @param use_padding Hash the application with configured padding when true.
 * @return Firmware integrity verification status.
 */
verify_integrity_status_t verify_sector5_integrity_against_manifest(const signed_manifest_t *verified_sm,
                                                                    bool use_padding)
{
    uint8_t runtime_hash[SHA256_HASH_SIZE];
    firmware_hash_status_t hrc;
    if (verified_sm == NULL) {
        return VERIFY_INTEGRITY_ERR_NULL_PTR;
    }
    memset(runtime_hash, 0, sizeof(runtime_hash));
    if (hash_buffer_is_all_value(verified_sm->manifest.sha256_hash, SHA256_HASH_SIZE, 0x00U) ||
        hash_buffer_is_all_value(verified_sm->manifest.sha256_hash, SHA256_HASH_SIZE, 0xFFU)) {
        log_printf("[ERROR] verify_sector5_integrity_against_manifest: stored manifest hash is empty/erased\r\n");
        return VERIFY_INTEGRITY_ERR_STORED_HASH_EMPTY;
    }
    log_printf("[INFO] Verifying firmware integrity against manifest hash...\r\n");
    delay_ms(5);
    hrc = hash_sector5_firmware_only(runtime_hash, sizeof(runtime_hash), use_padding);
    if (hrc != FIRMWARE_HASH_SUCCESS) {
/* Remove the temporary runtime digest before returning. */
        secure_zero(runtime_hash, sizeof(runtime_hash));
        return VERIFY_INTEGRITY_ERR_HASH_FAILED;
    }
    if (hash_ct_memcmp(runtime_hash,
                       verified_sm->manifest.sha256_hash,
                       SHA256_HASH_SIZE) != 0) {
        log_printf("[ERROR] verify_sector5_integrity_against_manifest: hash mismatch\r\n");
        log_printf("[INFO] Runtime hash:\r\n       ");
        for (size_t i = 0; i < SHA256_HASH_SIZE; i++) {
            log_printf("%02X", runtime_hash[i]);
            if (i == 15U) {
                log_printf("\r\n       ");
            }
        }
        log_printf("\r\n");
        log_printf("[INFO] Stored manifest hash:\r\n       ");
        for (size_t i = 0; i < SHA256_HASH_SIZE; i++) {
            log_printf("%02X", verified_sm->manifest.sha256_hash[i]);
            if (i == 15U) {
                log_printf("\r\n");
            }
        }
        log_printf("\r\n");
        delay_ms(5);
        secure_zero(runtime_hash, sizeof(runtime_hash));
        return VERIFY_INTEGRITY_ERR_HASH_MISMATCH;
    }
    log_printf("[OK] Integrity verification passed: Sector 5 hash matches manifest hash\r\n");
    log_printf("[OK] Secure boot checks passed\r\n");
    delay_ms(5);
    secure_zero(runtime_hash, sizeof(runtime_hash));
    return VERIFY_INTEGRITY_SUCCESS;
}
/* Handles the get manifest word count operation. */
static inline uint32_t get_manifest_word_count(void)
{
    /* Check the return status before continuing to the next step. */
    return (sizeof(signed_manifest_t) + 3U) / 4U;
}
/* Exact signed-manifest size in bytes. */
static inline uint32_t get_manifest_byte_count(void)
{
    /* Check the return status before continuing to the next step. */
    return sizeof(signed_manifest_t);
}
/* Protected manifest storage and reload path. */
/**
 * @brief Store the signed manifest in Flash, verify it and restore write protection.
 * @param sm Signed manifest to store.
 * @param ctx Signing-key context used by the provisioning flow.
 * @return Manifest storage status.
 */
manifest_store_status_t store_signed_manifest_protected(signed_manifest_t *sm, sig_keys_ctx_t *ctx)
{
    uint8_t wrp_before, wrp_after;
    signed_manifest_t readback;
    uint32_t manifest_word_count;
    uint32_t read_ret;
    uint32_t write_ret;
    bool already_present = false;
    manifest_store_status_t precheck_ret;
    if (sm == NULL || ctx == NULL) {
        log_printf("[ERROR] store_signed_manifest_protected: NULL input\r\n");
        return MANIFEST_STORE_ERR_NULL_INPUT;
    }
    precheck_ret = check_existing_manifest_and_pubkey(&already_present);
    if (precheck_ret != MANIFEST_STORE_SUCCESS) {
        log_printf("[ERROR] store_signed_manifest_protected: precheck failed (%d)\r\n",
                   (int)precheck_ret);
        return precheck_ret;
    }
    if (already_present) {
        log_printf("[INFO] Signed manifest exists and OTP public key is provisioned; skipping manifest update\r\n");
        delay_ms(5);
        return MANIFEST_STORE_SUCCESS;
    }
    manifest_word_count = get_manifest_word_count();
    log_printf("[INFO] Storing signed manifest (%lu bytes, %lu words) to 0x%08lX\r\n",
               (unsigned long)sizeof(signed_manifest_t),
               (unsigned long)manifest_word_count,
               (unsigned long)MANIFEST_FLASH_ADDR);
    delay_ms(5);
    wrp_before = flash_obytes_get_wrp();
    if ((wrp_before & (1U << MANIFEST_SECTOR_NUM)) == 0U) {
        log_printf("[INFO] Sector %u currently WRP-protected, disabling WRP for update\r\n",
                   (unsigned)MANIFEST_SECTOR_NUM);
        dis_wrp_all_sectors();
        delay_ms(5);
    }
    log_printf("[INFO] Erasing manifest sector (%u) before write\r\n", MANIFEST_SECTOR_NUM);
    delay_ms(5);
    flash_erase_status_t erase_ret = erase_manifest_sector();
    if (erase_ret != FLASH_ERASE_SUCCESS) {
        log_printf("[FATAL] Manifest sector erase failed: %s (%d)\r\n",
                   flash_erase_status_str(erase_ret), (int)erase_ret);
        return MANIFEST_STORE_ERR_FLASH_ERASE;
    }
    log_printf("[OK] Manifest sector erased successfully\r\n");
    delay_ms(10);
    write_ret = flash_write_data(MANIFEST_FLASH_ADDR, (uint32_t *)sm, manifest_word_count);
    if (write_ret != 0) {
        log_printf("[ERROR] flash_write_data (manifest) failed with error: 0x%08lX\r\n",
                   (unsigned long)write_ret);
        return MANIFEST_STORE_ERR_FLASH_WRITE;
    }
    log_printf("[OK] Manifest written to flash at 0x%08lX\r\n", (unsigned long)MANIFEST_FLASH_ADDR);
    delay_ms(5);
    memset(&readback, 0, sizeof(readback));
    read_ret = flash_read_data(MANIFEST_FLASH_ADDR, (uint32_t *)&readback, manifest_word_count);
    if (read_ret != FLASH_READ_OK) {
        log_printf("[ERROR] flash_read_data (manifest) failed with error: 0x%08lX\r\n",
                   (unsigned long)read_ret);
        return MANIFEST_STORE_ERR_FLASH_READ;
    }
    if (memcmp(sm, &readback, sizeof(signed_manifest_t)) != 0) {
        log_printf("[ERROR] Manifest verification failed\r\n");
        return MANIFEST_STORE_ERR_VERIFY_FAIL;
    }
    log_printf("[OK] Manifest verify successful\r\n");
    delay_ms(5);
    log_printf("[INFO] Enabling WRP on sector %u\r\n", MANIFEST_SECTOR_NUM);
/* Restore write protection after the manifest write/readback check. */
    sector_set_wrp(MANIFEST_SECTOR);
    delay_ms(10);
    wrp_after = flash_obytes_get_wrp();
    log_printf("[INFO] WRP register: before=0x%02X, after=0x%02X\r\n",
               (unsigned)wrp_before, (unsigned)wrp_after);
    delay_ms(5);
    if ((wrp_after & (1U << 7)) != 0U) {
        log_printf("[ERROR] Failed to enable WRP on sector %u\r\n", MANIFEST_SECTOR_NUM);
        return MANIFEST_STORE_ERR_WRP_FAIL;
    }
    log_printf("[OK] Signed manifest stored and write-protected at sector %u; public key remains in OTP\r\n",
               (unsigned)MANIFEST_SECTOR_NUM);
    delay_ms(5);
    log_printf("[INFO] Securely zeroing manifest from RAM...\r\n");
    delay_ms(5);
    secure_zero(sm, sizeof(signed_manifest_t));
    {
        uint8_t *check_ptr = (uint8_t *)sm;
        bool zero_verified = true;
        for (size_t i = 0; i < sizeof(signed_manifest_t); i++) {
            if (check_ptr[i] != 0) {
                zero_verified = false;
                break;
            }
        }
        if (zero_verified) {
            log_printf("[OK] Manifest securely zeroed from RAM (%lu bytes)\r\n",
                       (unsigned long)sizeof(signed_manifest_t));
        } else {
            log_printf("[WARN] Manifest zeroing verification failed!\r\n");
        }
    }
    delay_ms(5);
    secure_zero(&readback, sizeof(readback));
    log_printf("[OK] Readback buffer zeroed\r\n");
    delay_ms(5);
    return MANIFEST_STORE_SUCCESS;
}
/* Print a byte array in compact hexadecimal form. */
static void print_hex_array(const char *label, const uint8_t *data, size_t len)
{
    /* Check the return status before continuing to the next step. */
    log_printf("%s: ", label);
    for (size_t i = 0; i < len; ++i) {
        log_printf("%02X", data[i]);
        if ((i + 1) % 16 == 0 && i + 1 < len) {
            log_printf("\r\n%*s  ", (int)strlen(label), "");
        }
}
}
/**
 * @brief Load and validate the signed manifest from protected Flash.
 * @param sm_out Signed-manifest output.
 * @return Manifest storage/validation status.
 */
manifest_store_status_t load_signed_manifest(signed_manifest_t *sm_out)
{
    /* Validate the stored data before using it for verification. */
    if (sm_out == NULL) {
        return MANIFEST_STORE_ERR_NULL_INPUT;
    }
    uint32_t word_count = get_manifest_word_count();
    flash_read_data(MANIFEST_FLASH_ADDR, (uint32_t *)sm_out, word_count);
    log_printf("[INFO] Loaded signed manifest from 0x%08lX\r\n",
               (unsigned long)MANIFEST_FLASH_ADDR);
    delay_ms(10);
    log_printf("\r\n");
    delay_ms(5);
    log_printf("╔══════════════════════════════════════════════════════════════════╗\r\n");
    delay_ms(5);
    log_printf("║           SIGNED MANIFEST FLASH MEMORY MAP                       ║\r\n");
    delay_ms(5);
    log_printf("╠══════════════════════════════════════════════════════════════════╣\r\n");
    delay_ms(5);
    log_printf("║ Field            │ Offset │ Size │ Start Addr │ End Addr         ║\r\n");
    delay_ms(5);
    log_printf("╠══════════════════════════════════════════════════════════════════╣\r\n");
    delay_ms(5);
    log_printf("║ version          │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX            ║\r\n",
               (unsigned long)offsetof(signed_manifest_t, manifest.version),
               (unsigned long)sizeof(sm_out->manifest.version),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.version)),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.version)
               + sizeof(sm_out->manifest.version) - 1));
    delay_ms(10);
    log_printf("║ firmware_size    │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX       ║\r\n",
               (unsigned long)offsetof(signed_manifest_t, manifest.firmware_size),
               (unsigned long)sizeof(sm_out->manifest.firmware_size),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.firmware_size)),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.firmware_size)
               + sizeof(sm_out->manifest.firmware_size) - 1));
    delay_ms(10);
    log_printf("║ sha256_hash      │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX       ║\r\n",
               (unsigned long)offsetof(signed_manifest_t, manifest.sha256_hash),
               (unsigned long)sizeof(sm_out->manifest.sha256_hash),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.sha256_hash)),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.sha256_hash)
               + sizeof(sm_out->manifest.sha256_hash) - 1));
    delay_ms(10);
    log_printf("║ rollback_counter │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX       ║\r\n",
               (unsigned long)offsetof(signed_manifest_t, manifest.rollback_counter),
               (unsigned long)sizeof(sm_out->manifest.rollback_counter),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.rollback_counter)),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.rollback_counter)
               + sizeof(sm_out->manifest.rollback_counter) - 1));
    delay_ms(10);
    log_printf("║ signature        │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX       ║\r\n",
               (unsigned long)offsetof(signed_manifest_t, signature),
               (unsigned long)sizeof(sm_out->signature),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, signature)),
               (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, signature)
               + sizeof(sm_out->signature) - 1));
    delay_ms(10);
    log_printf("╠══════════════════════════════════════════════════════════════════╣\r\n");
    log_printf("║ TOTAL            │    0   │ %3lu  │ 0x%08lX │ 0x%08lX       ║\r\n",
               (unsigned long)sizeof(signed_manifest_t),
               (unsigned long)MANIFEST_FLASH_ADDR,
               (unsigned long)(MANIFEST_FLASH_ADDR + sizeof(signed_manifest_t) - 1));
    log_printf("╚══════════════════════════════════════════════════════════════════╝\r\n");
    delay_ms(10);
    log_printf("\r\n");
    log_printf("┌──────────────────────────────────────────────────────────────────┐\r\n");
    log_printf("│                    MANIFEST VALUES                               │\r\n");
    log_printf("├──────────────────────────────────────────────────────────────────┤\r\n");
    log_printf("│ version          : 0x%08lX (%lu)\r\n",
               (unsigned long)sm_out->manifest.version,
               (unsigned long)sm_out->manifest.version);
    delay_ms(10);
    log_printf("│ firmware_size    : 0x%08lX (%lu bytes)\r\n",
               (unsigned long)sm_out->manifest.firmware_size,
               (unsigned long)sm_out->manifest.firmware_size);
    delay_ms(10);
    log_printf("│ rollback_counter : 0x%08lX (%lu)\r\n",
               (unsigned long)sm_out->manifest.rollback_counter,
               (unsigned long)sm_out->manifest.rollback_counter);
    delay_ms(10);
    log_printf("├──────────────────────────────────────────────────────────────────┤\r\n");
    log_printf("│ sha256_hash:\r\n│   ");
    delay_ms(5);
    for (size_t i = 0; i < 32; ++i) {
        log_printf("%02X", sm_out->manifest.sha256_hash[i]);
        delay_ms(5);
        if (i == 15) log_printf("\r\n│   ");
    }
    log_printf("\r\n");
    delay_ms(5);
    log_printf("├──────────────────────────────────────────────────────────────────┤\r\n");
    log_printf("│ signature (Ed25519, %d bytes):\r\n│   ", ED25519_SIGLEN);
    delay_ms(5);
    for (size_t i = 0; i < ED25519_SIGLEN; ++i) {
        log_printf("%02X", sm_out->signature[i]);
        delay_ms(5);
        if ((i + 1) % 16 == 0 && i + 1 < ED25519_SIGLEN) {
            log_printf("\r\n│   ");
            delay_ms(1);
        }
    }
    log_printf("\r\n");
    log_printf("└──────────────────────────────────────────────────────────────────┘\r\n");
    delay_ms(5);
    if (sm_out->manifest.firmware_size == 0 ||
        sm_out->manifest.firmware_size > MAX_EXPECTED_FIRMWARE_SIZE) {
        log_printf("[WARN] Loaded manifest has invalid firmware_size=%lu\r\n",
                   (unsigned long)sm_out->manifest.firmware_size);
        delay_ms(5);
        secure_zero(sm_out, word_count * sizeof(uint32_t));
        return MANIFEST_STORE_ERR_INVALID_MANIFEST;
    }
    {
        int all_zero = 1;
        for (size_t i = 0; i < sizeof(sm_out->manifest.sha256_hash); ++i) {
            if (sm_out->manifest.sha256_hash[i] != 0) { all_zero = 0; break; }
        }
        if (all_zero) {
            log_printf("[WARN] Loaded manifest hash is all-zero (likely empty)\r\n");
            delay_ms(5);
            secure_zero(sm_out, word_count * sizeof(uint32_t));
            return MANIFEST_STORE_ERR_EMPTY;
        }
    }
    {
        int sig_all_zero = 1;
        for (size_t i = 0; i < ED25519_SIGLEN; ++i) {
            if (sm_out->signature[i] != 0) { sig_all_zero = 0; break; }
        }
        if (sig_all_zero) {
            log_printf("[WARN] Loaded signature is empty\r\n");
            delay_ms(5);
            secure_zero(sm_out, word_count * sizeof(uint32_t));
            return MANIFEST_STORE_ERR_NO_SIGNATURE;
        }
    }
    log_printf("[OK] Manifest validation passed\r\n");
    delay_ms(5);
    {
        uintptr_t otp_public_addr = 0U;
        if (!otp_public_key_is_programmed(&otp_public_addr)) {
            log_printf(
                "[WARN] Dynamic OTP Ed25519 record/public key not found in "
                "0x%08lX..0x%08lX\r\n",
                (unsigned long)STM32F411_OTP_DATA_START,
                (unsigned long)STM32F411_OTP_DATA_END);
            return MANIFEST_STORE_ERR_NO_SIGNATURE;
        }
        log_printf(
            "[INFO] Dynamic OTP Ed25519 public key present at 0x%08lX "
            "(not copied into persistent RAM context)\r\n",
            (unsigned long)otp_public_addr);
    }
    return MANIFEST_STORE_SUCCESS;
}
/**
 * @brief Convert a firmware hash status to a short diagnostic string.
 * @param status Firmware hash status.
 * @return Status string.
 */
const char* firmware_hash_status_str(firmware_hash_status_t status)
{
    /* Check the digest result before using it in verification. */
    switch (status) {
        case FIRMWARE_HASH_SUCCESS:          return "SUCCESS";
        case FIRMWARE_HASH_ERR_NULL_INPUT:   return "NULL_INPUT";
        case FIRMWARE_HASH_ERR_BAD_LENGTH:   return "BAD_LENGTH";
        case FIRMWARE_HASH_ERR_INVALID_SIZE: return "INVALID_SIZE";
        case FIRMWARE_HASH_ERR_HASH_FAILED:  return "HASH_FAILED";
        case FIRMWARE_HASH_ERR_CONSTRUCT:    return "CONSTRUCT_FAILED";
        case FIRMWARE_HASH_ERR_INIT:         return "INIT_FAILED";
        case FIRMWARE_HASH_ERR_APPEND:       return "APPEND_FAILED";
        case FIRMWARE_HASH_ERR_FINALIZE:     return "FINALIZE_FAILED";
        default:                             return "UNKNOWN_ERROR";
    }
}
/* Padding-aware firmware hashing used by the final manifest flow. */
/**
 * @brief Hash firmware with optional erased-Flash padding included in the digest.
 * @param firmware_addr Start address of the firmware image.
 * @param firmware_size Actual firmware size in bytes.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @param use_padding Include Flash-word padding when true.
 * @return Firmware hash status.
 */
firmware_hash_status_t hash_firmware_with_padding(
    uint32_t firmware_addr,
    uint32_t firmware_size,
    uint8_t *out_hash,
    size_t out_len,
    bool use_padding)
{
    /* Check the digest result before using it in verification. */
    cmox_hash_retval_t retval;
    cmox_sha256_handle_t sha_ctx;
    cmox_hash_handle_t *hash_handle;
    uint32_t bytes_remaining;
    uint32_t current_addr;
    uint32_t hash_size;
    uint32_t padding_bytes;
    size_t hash_len = 0;
    if (out_hash == NULL) {
        log_printf("[ERROR] hash_firmware: NULL output pointer\r\n");
        return FIRMWARE_HASH_ERR_NULL_INPUT;
    }
    if (out_len != SHA256_HASH_SIZE) {
        log_printf("[ERROR] hash_firmware: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, SHA256_HASH_SIZE);
        return FIRMWARE_HASH_ERR_BAD_LENGTH;
    }
    if (firmware_size == 0 || firmware_size > MAX_FIRMWARE_SIZE) {
        log_printf("[ERROR] hash_firmware: invalid firmware size (%lu bytes)\r\n",
                   (unsigned long)firmware_size);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }
    if (use_padding) {
        hash_size = ALIGN_UP(firmware_size, FLASH_WORD_SIZE);
        padding_bytes = hash_size - firmware_size;
    } else {
        hash_size = firmware_size;
        padding_bytes = 0;
    }
    log_printf("\r\n");
    log_printf("┌─────────────────────────────────────────────────────────────┐\r\n");
    log_printf("│         FIRMWARE HASHING (SHA-256)                          │\r\n");
    log_printf("└─────────────────────────────────────────────────────────────┘\r\n");
    delay_ms(5);
    log_printf("[INFO] Firmware address:    0x%08lX\r\n", (unsigned long)firmware_addr);
    log_printf("[INFO] Firmware size:       %lu bytes (0x%lX)\r\n",
               (unsigned long)firmware_size, (unsigned long)firmware_size);
    delay_ms(5);
    if (use_padding) {
        log_printf("[INFO] Padding enabled:     YES\r\n");
        log_printf("[INFO] Padded size:         %lu bytes (0x%lX)\r\n",
                   (unsigned long)hash_size, (unsigned long)hash_size);
        log_printf("[INFO] Padding bytes:       %lu (filled with 0x%02X)\r\n",
                   (unsigned long)padding_bytes, FLASH_ERASED_VALUE);
    } else {
        log_printf("[INFO] Padding enabled:     NO (exact size)\r\n");
    }
    log_printf("[INFO] Word alignment:      %s\r\n",
               (firmware_size % FLASH_WORD_SIZE == 0) ? "YES" : "NO (padding recommended)");
    delay_ms(5);
    hash_handle = cmox_sha256_construct(&sha_ctx);
    if (hash_handle == NULL) {
        log_printf("[ERROR] SHA-256 construct failed\r\n");
        return FIRMWARE_HASH_ERR_CONSTRUCT;
    }
    retval = cmox_hash_init(hash_handle);
    if (retval != CMOX_HASH_SUCCESS) {
        log_printf("[ERROR] SHA-256 init failed (ret=%d)\r\n", retval);
        cmox_hash_cleanup(hash_handle);
        return FIRMWARE_HASH_ERR_INIT;
    }
    log_printf("[INFO] Hashing firmware in %lu-byte chunks...\r\n",
               (unsigned long)HASH_CHUNK_SIZE);
    delay_ms(5);
    bytes_remaining = firmware_size;
    current_addr = firmware_addr;
    while (bytes_remaining > 0U) {
        size_t chunk_size = (bytes_remaining > HASH_CHUNK_SIZE)
                            ? HASH_CHUNK_SIZE
                            : (size_t)bytes_remaining;
        const uint8_t *chunk_data = (const uint8_t *)current_addr;
        retval = cmox_hash_append(hash_handle, chunk_data, chunk_size);
        if (retval != CMOX_HASH_SUCCESS) {
            log_printf("[ERROR] Hash append failed at offset 0x%08lX (ret=%d)\r\n",
                       (unsigned long)(current_addr - firmware_addr), retval);
            cmox_hash_cleanup(hash_handle);
            memset(out_hash, 0, out_len);
            return FIRMWARE_HASH_ERR_APPEND;
        }
        current_addr += (uint32_t)chunk_size;
        bytes_remaining -= (uint32_t)chunk_size;
        uint32_t hashed_so_far = firmware_size - bytes_remaining;
        if ((hashed_so_far % (16U * 1024U)) == 0U || bytes_remaining == 0) {
            log_printf("  Hashed %lu / %lu bytes (%lu%%)\r\n",
                       (unsigned long)hashed_so_far,
                       (unsigned long)firmware_size,
                       (unsigned long)((hashed_so_far * 100) / firmware_size));
            delay_ms(2);
        }
    }
    if (use_padding && padding_bytes > 0) {
        uint8_t padding_buffer[FLASH_WORD_SIZE];
        memset(padding_buffer, FLASH_ERASED_VALUE, sizeof(padding_buffer));
        log_printf("[INFO] Appending %lu padding bytes (0x%02X)...\r\n",
                   (unsigned long)padding_bytes, FLASH_ERASED_VALUE);
        delay_ms(5);
        retval = cmox_hash_append(hash_handle, padding_buffer, padding_bytes);
        if (retval != CMOX_HASH_SUCCESS) {
            log_printf("[ERROR] Hash append (padding) failed (ret=%d)\r\n", retval);
            cmox_hash_cleanup(hash_handle);
            memset(out_hash, 0, out_len);
            return FIRMWARE_HASH_ERR_APPEND;
        }
    }
    retval = cmox_hash_generateTag(hash_handle, out_hash, &hash_len);
    if (retval != CMOX_HASH_SUCCESS || hash_len != SHA256_HASH_SIZE) {
        log_printf("[ERROR] Hash finalization failed (ret=%d, len=%lu)\r\n",
                   retval, (unsigned long)hash_len);
        cmox_hash_cleanup(hash_handle);
        memset(out_hash, 0, out_len);
        return FIRMWARE_HASH_ERR_FINALIZE;
    }
    retval = cmox_hash_cleanup(hash_handle);
    if (retval != CMOX_HASH_SUCCESS) {
        log_printf("[WARN] Hash cleanup returned %d (non-fatal)\r\n", retval);
    }
    log_printf("[OK] Firmware hashed successfully\r\n");
    log_printf("[INFO] SHA-256 Hash:\r\n       ");
    for (size_t i = 0; i < SHA256_HASH_SIZE; i++) {
        log_printf("%02X", out_hash[i]);
        if (i == 15) log_printf("\r\n       ");
    }
    log_printf("\r\n");
    delay_ms(5);
    {
        bool all_zero = true;
        for (size_t i = 0; i < SHA256_HASH_SIZE; i++) {
            if (out_hash[i] != 0) {
                all_zero = false;
                break;
            }
        }
        if (all_zero) {
            log_printf("[ERROR] Computed hash is all zeros - something is wrong!\r\n");
            return FIRMWARE_HASH_ERR_HASH_FAILED;
        }
    }
    return FIRMWARE_HASH_SUCCESS;
}
/**
 * @brief Build the manifest and hash the configured application with optional padding.
 * @param manifest Manifest to populate.
 * @param use_padding Include Flash-word padding when true.
 * @return Firmware hash status.
 */
firmware_hash_status_t create_and_hash_manifest_with_padding(
    firmware_manifest_t *manifest,
    bool use_padding)
{
    firmware_hash_status_t rc;
    uint32_t padded_size;
    uint32_t padding_bytes;
    wait_image_ready_status_t wrc;
    if (manifest == NULL) {
        log_printf("[ERROR] create_manifest: NULL manifest pointer\r\n");
        return FIRMWARE_HASH_ERR_NULL_INPUT;
    }
    if (FIRMWARE_SIZE == 0 || FIRMWARE_SIZE > MAX_FIRMWARE_SIZE) {
        log_printf("[ERROR] create_manifest: invalid FIRMWARE_SIZE (%lu)\r\n",
                   (unsigned long)FIRMWARE_SIZE);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }
    if (FIRMWARE_SIZE > APP_SLOT_SIZE_BYTES) {
        log_printf("[ERROR] create_manifest: firmware size exceeds app slot\r\n");
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }
//   /* -------- Wait until external flasher marks Sector 5 as ready -------- */
//    wrc = wait_for_image_ready_flag(APP_SLOT_START_ADDR,
//                                    FIRMWARE_SIZE,
//                                    FW_READY_POLL_DELAY_MS,
//                                    0U); /* 0 = wait forever */
//
//    if (wrc == WAIT_IMAGE_READY_ERR_BAD_PARAM) {
//        log_printf("[ERROR] create_manifest: wait_for_image_ready_flag bad param\r\n");
//        return FIRMWARE_HASH_ERR_NULL_INPUT;
//    }
//
//    if (wrc == WAIT_IMAGE_READY_ERR_TIMEOUT) {
//        log_printf("[ERROR] create_manifest: timeout waiting for image-ready flag\r\n");
//        return FIRMWARE_HASH_ERR_HASH_FAILED;
//    }
    if (use_padding) {
        padded_size = ALIGN_UP(FIRMWARE_SIZE, FLASH_WORD_SIZE);
        padding_bytes = padded_size - FIRMWARE_SIZE;
    } else {
        padded_size = FIRMWARE_SIZE;
        padding_bytes = 0;
    }
    log_printf("\r\n");
    log_printf("╔══════════════════════════════════════════════════════════════════╗\r\n");
    log_printf("║         MANIFEST CREATION                                        ║\r\n");
    log_printf("╚══════════════════════════════════════════════════════════════════╝\r\n");
    delay_ms(5);
    manifest->version = FIRMWARE_VERSION;
    manifest->firmware_size = FIRMWARE_SIZE;
    manifest->rollback_counter = 0;
    memset(manifest->sha256_hash, 0, sizeof(manifest->sha256_hash));
    log_printf("[INFO] Manifest configuration:\r\n");
    log_printf("       Version:          0x%08lX\r\n", (unsigned long)manifest->version);
    log_printf("       Firmware size:    %lu bytes (0x%lX)\r\n",
               (unsigned long)manifest->firmware_size,
               (unsigned long)manifest->firmware_size);
    log_printf("       Firmware start:   0x%08lX\r\n", (unsigned long)APP_SLOT_START_ADDR);
    log_printf("       Firmware end:     0x%08lX\r\n",
               (unsigned long)(APP_SLOT_START_ADDR + manifest->firmware_size - 1U));
    delay_ms(5);
    if (use_padding) {
        log_printf("       Padded size:      %lu bytes (0x%lX)\r\n",
                   (unsigned long)padded_size,
                   (unsigned long)padded_size);
        log_printf("       Padding bytes:    %lu\r\n",
                   (unsigned long)padding_bytes);
    }
    log_printf("       Rollback counter: %lu\r\n",
               (unsigned long)manifest->rollback_counter);
    delay_ms(5);
    log_printf("[INFO] Computing firmware hash...\r\n");
    delay_ms(5);
    rc = hash_firmware_with_padding(
        APP_SLOT_START_ADDR,
        manifest->firmware_size,
        manifest->sha256_hash,
        sizeof(manifest->sha256_hash),
        use_padding
    );
    if (rc != FIRMWARE_HASH_SUCCESS) {
        log_printf("[ERROR] Firmware hashing failed: %s (%d)\r\n",
                   firmware_hash_status_str(rc), (int)rc);
        memset(manifest->sha256_hash, 0, sizeof(manifest->sha256_hash));
        return rc;
    }
    log_printf("\r\n");
    log_printf("┌──────────────────────────────────────────────────────────────────┐\r\n");
    log_printf("│                    MANIFEST READY                                │\r\n");
    log_printf("├──────────────────────────────────────────────────────────────────┤\r\n");
    log_printf("│ Version:          0x%08lX (%lu)\r\n",
               (unsigned long)manifest->version,
               (unsigned long)manifest->version);
    log_printf("│ Firmware size:    %lu bytes\r\n",
               (unsigned long)manifest->firmware_size);
    log_printf("│ Rollback counter: %lu\r\n",
               (unsigned long)manifest->rollback_counter);
    log_printf("├──────────────────────────────────────────────────────────────────┤\r\n");
    log_printf("│ SHA-256 Hash:\r\n│   ");
    for (size_t i = 0; i < sizeof(manifest->sha256_hash); i++) {
        log_printf("%02X", manifest->sha256_hash[i]);
        if (i == 15) {
            log_printf("\r\n│   ");
        }
    }
    log_printf("\r\n");
    log_printf("└──────────────────────────────────────────────────────────────────┘\r\n");
    delay_ms(5);
    log_printf("\r\n[SUCCESS] Manifest created - ready for signing\r\n");
    delay_ms(5);
    return FIRMWARE_HASH_SUCCESS;
}
/* OTP provisioning and RAM key cleanup. */
/**
 * @brief Provision the Ed25519 public key to OTP and remove its RAM copy.
 * @param ctx Signing-key context.
 * @param lock_otp Request OTP locking after provisioning.
 * @return OTP provisioning status.
 */
otp_status_t provision_public_key_to_otp(sig_keys_ctx_t *ctx, bool lock_otp)
{
    /* Validate the key data before using it in crypto operations. */
    otp_status_t status;
    if (ctx == NULL) {
        log_printf("[ERROR] NULL context pointer\r\n");
        return OTP_ERR_NULL_INPUT;
    }
    log_printf("\r\n");
    log_printf("╔══════════════════════════════════════════════════════════════════╗\r\n");
    log_printf("║         PUBLIC KEY PROVISIONING TO OTP                           ║\r\n");
    log_printf("╚══════════════════════════════════════════════════════════════════╝\r\n");
    delay_ms(5);
    log_printf("[INFO] Storing public key to OTP...\r\n");
    delay_ms(5);
    status = otp_store_public_key(ctx->sig.public_key, lock_otp);
    if (status != OTP_SUCCESS) {
        log_printf("[ERROR] Failed to store public key to OTP: %s (%d)\r\n",
                   otp_status_str(status), (int)status);
        return status;
    }
    log_printf("[OK] Public key stored in OTP successfully\r\n");
    delay_ms(5);
    log_printf("[INFO] Securely zeroing public key from RAM...\r\n");
    delay_ms(5);
    secure_zero(ctx->sig.public_key, sizeof(ctx->sig.public_key));
    ctx->sig.pub_len = 0;
    bool pubkey_zeroed = true;
    for (size_t i = 0; i < sizeof(ctx->sig.public_key); i++) {
        if (ctx->sig.public_key[i] != 0) {
            pubkey_zeroed = false;
            break;
        }
    }
    if (pubkey_zeroed) {
        log_printf("[OK] Public key securely zeroed from RAM (%lu bytes)\r\n",
                   (unsigned long)sizeof(ctx->sig.public_key));
    } else {
        log_printf("[WARN] Public key zeroing verification failed!\r\n");
    }
    delay_ms(5);
    ctx->sig.provisioned = true;
    ctx->sig.storage_flags |= SIG_KEY_STORAGE_OTP;
    log_printf("[SUCCESS] Public key provisioned to OTP and zeroed from RAM\r\n");
    delay_ms(5);
    return OTP_SUCCESS;
}
/**
 * @brief Zero Ed25519 RAM key copies and remaining crypto working state.
 * @param ctx Signing-key context to clear.
 */
void secure_zero_all_keys(sig_keys_ctx_t *ctx)
{
    /* Clear the buffers before releasing the key material. */
    if (ctx == NULL) {
        return;
    }
    log_printf("\r\n");
    log_printf("┌─────────────────────────────────────────────────────────────┐\r\n");
    log_printf("│         SECURE ZEROING ALL SENSITIVE DATA                   │\r\n");
    log_printf("└─────────────────────────────────────────────────────────────┘\r\n");
    delay_ms(5);
    log_printf("[INFO] Zeroing Ed25519 private key RAM copy (%lu bytes)...\r\n",
               (unsigned long)sizeof(ctx->sig.private_key));
    secure_zero(ctx->sig.private_key, sizeof(ctx->sig.private_key));
    ctx->sig.priv_len = 0U;
    log_printf("[OK] Private key RAM copy zeroed\r\n");
    log_printf("[INFO] Zeroing Ed25519 public key RAM copy (%lu bytes)...\r\n",
               (unsigned long)sizeof(ctx->sig.public_key));
    secure_zero(ctx->sig.public_key, sizeof(ctx->sig.public_key));
    ctx->sig.pub_len = 0U;
    log_printf("[OK] Public key RAM copy zeroed\r\n");
    delay_ms(5);
    log_printf("[INFO] Zeroing working buffer (%lu bytes)...\r\n",
               (unsigned long)sizeof(ctx->working_buffer));
    secure_zero(ctx->working_buffer, sizeof(ctx->working_buffer));
    log_printf("[OK] Working buffer zeroed\r\n");
    delay_ms(5);
    log_printf("[INFO] Zeroing DRBG state...\r\n");
    secure_zero(&ctx->drbg, sizeof(ctx->drbg));
    ctx->drbg_ctx = NULL;
    log_printf("[OK] DRBG state zeroed\r\n");
    delay_ms(5);
    log_printf("[INFO] Zeroing ECC context...\r\n");
    secure_zero(&ctx->ecc, sizeof(ctx->ecc));
    log_printf("[OK] ECC context zeroed\r\n");
    delay_ms(5);
    ctx->sig.provisioned = true;
    ctx->sig.storage_flags = SIG_KEY_STORAGE_OTP;
    log_printf(
        "\r\n[SUCCESS] RAM crypto material zeroized; "
        "Ed25519 keypair remains only in OTP\r\n");
    delay_ms(5);
}
/* Optional image-ready synchronization with the external flashing flow. */
/**
 * @brief Validate the external-flasher image-ready flag.
 * @param expected_image_addr Expected application address.
 * @param expected_image_size Expected application size.
 * @return Image-ready flag status.
 */
image_ready_status_t image_ready_flag_check(uint32_t expected_image_addr,
                                            uint32_t expected_image_size)
{
    /* Check the read status before using the returned data. */
    const image_ready_flag_t *flag;
    if (expected_image_addr == 0U || expected_image_size == 0U) {
        return IMAGE_READY_ERR_BAD_PARAM;
    }
    flag = (const image_ready_flag_t *)IMAGE_READY_FLAG_ADDR;
    if ((flag->magic == 0xFFFFFFFFUL) &&
        (flag->magic_inv == 0xFFFFFFFFUL) &&
        (flag->image_addr == 0xFFFFFFFFUL) &&
        (flag->image_size == 0xFFFFFFFFUL) &&
        (flag->state == 0xFFFFFFFFUL)) {
        return IMAGE_READY_NOT_SET;
    }
    if ((flag->magic ^ flag->magic_inv) != 0xFFFFFFFFUL) {
        return IMAGE_READY_INVALID;
    }
    if ((flag->image_addr ^ flag->image_addr_inv) != 0xFFFFFFFFUL) {
        return IMAGE_READY_INVALID;
    }
    if ((flag->image_size ^ flag->image_size_inv) != 0xFFFFFFFFUL) {
        return IMAGE_READY_INVALID;
    }
    if ((flag->state ^ flag->state_inv) != 0xFFFFFFFFUL) {
        return IMAGE_READY_INVALID;
    }
    if (flag->magic != IMAGE_READY_MAGIC) {
        return IMAGE_READY_INVALID;
    }
    if (flag->state != IMAGE_READY_STATE_VALID) {
        return IMAGE_READY_INVALID;
    }
    if (flag->image_addr != expected_image_addr) {
        return IMAGE_READY_INVALID;
    }
    if (flag->image_size != expected_image_size) {
        return IMAGE_READY_INVALID;
    }
    if (flag->image_size > APP_SLOT_SIZE_BYTES) {
        return IMAGE_READY_INVALID;
    }
    return IMAGE_READY_OK;
}
/**
 * @brief Poll until the expected application-ready flag becomes valid.
 * @param expected_image_addr Expected application address.
 * @param expected_image_size Expected application size.
 * @param poll_delay_ms Delay between polls.
 * @param max_polls Maximum polls, or zero for no fixed limit.
 * @return Image-ready wait status.
 */
wait_image_ready_status_t wait_for_image_ready_flag(uint32_t expected_image_addr,
                                                    uint32_t expected_image_size,
                                                    uint32_t poll_delay_ms,
                                                    uint32_t max_polls)
{
    /* Check the read status before using the returned data. */
    uint32_t polls = 0U;
    int last_state = 0x7FFFFFFF;
    if ((expected_image_addr == 0U) ||
        (expected_image_size == 0U) ||
        (poll_delay_ms == 0U)) {
        return WAIT_IMAGE_READY_ERR_BAD_PARAM;
    }
    for (;;) {
        image_ready_status_t st =
            image_ready_flag_check(expected_image_addr, expected_image_size);
        if ((int)st != last_state) {
            last_state = (int)st;
            if (st == IMAGE_READY_NOT_SET) {
                log_printf("[INFO] Firmware ready flag not set yet. Waiting...\r\n");
            } else if (st == IMAGE_READY_INVALID) {
                log_printf("[WARN] Firmware ready flag present but invalid. Waiting...\r\n");
            } else if (st == IMAGE_READY_OK) {
                log_printf("[OK] Firmware ready flag detected.\r\n");
            } else {
                log_printf("[ERROR] Bad parameters while waiting for ready flag.\r\n");
            }
        }
        if (st == IMAGE_READY_OK) {
            return WAIT_IMAGE_READY_OK;
        }
        if (st == IMAGE_READY_ERR_BAD_PARAM) {
            return WAIT_IMAGE_READY_ERR_BAD_PARAM;
        }
        if ((max_polls != 0U) && (polls >= max_polls)) {
            return WAIT_IMAGE_READY_ERR_TIMEOUT;
        }
        polls++;
        delay_ms(poll_delay_ms);
    }
}
/* Build the redundant image-ready record used by the external-flash flow. */
static void image_ready_flag_build(image_ready_flag_t *flag,
                                   uint32_t image_addr,
                                   uint32_t image_size)
{
    /* Check the read status before using the returned data. */
    if (flag == NULL) {
        return;
    }
    flag->magic         = IMAGE_READY_MAGIC;
    flag->magic_inv     = ~IMAGE_READY_MAGIC;
    flag->image_addr    = image_addr;
    flag->image_addr_inv= ~image_addr;
    flag->image_size    = image_size;
    flag->image_size_inv= ~image_size;
    flag->state         = IMAGE_READY_STATE_VALID;
    flag->state_inv     = ~IMAGE_READY_STATE_VALID;
}
/* Check whether a Flash region is erased or contains data. */
static flash_region_state_t flash_region_check_state(uint32_t start_addr, size_t size_bytes)
{
    /* Check the flash status and address range before continuing. */
    const __IO uint8_t *p;
    size_t i;
    if (start_addr == 0U || size_bytes == 0U) {
        return FLASH_REGION_STATE_ERR;
    }
    p = (const __IO uint8_t *)start_addr;
    for (i = 0; i < size_bytes; i++) {
        if (p[i] != 0xFFU) {
            return FLASH_REGION_STATE_HAS_DATA;
        }
    }
    return FLASH_REGION_STATE_EMPTY;
}
/* Validates the supplied data or peripheral state. */
static manifest_store_status_t check_existing_manifest_and_pubkey(bool *already_present)
{
    /* Return a failure status if any validation check fails. */
    flash_region_state_t manifest_state;
    bool otp_pubkey_present;
    uintptr_t otp_public_addr = 0U;
    if (already_present == NULL) {
        log_printf("[ERROR] check_existing_manifest_and_pubkey: NULL output\r\n");
        return MANIFEST_STORE_ERR_NULL_INPUT;
    }
    *already_present = false;
    manifest_state = flash_region_check_state(
        MANIFEST_FLASH_ADDR,
        sizeof(signed_manifest_t));
    if (manifest_state == FLASH_REGION_STATE_ERR) {
        log_printf("[ERROR] Failed to probe manifest region at 0x%08lX\r\n",
                   (unsigned long)MANIFEST_FLASH_ADDR);
        return MANIFEST_STORE_ERR_FLASH_READ;
    }
    otp_pubkey_present = otp_public_key_is_programmed(&otp_public_addr);
    log_printf(
        "[INFO] Manifest region: %s (0x%08lX - 0x%08lX, %lu bytes)\r\n",
        (manifest_state == FLASH_REGION_STATE_HAS_DATA) ? "HAS DATA" : "EMPTY",
        (unsigned long)MANIFEST_FLASH_ADDR,
        (unsigned long)(MANIFEST_FLASH_ADDR + sizeof(signed_manifest_t) - 1U),
        (unsigned long)sizeof(signed_manifest_t));
    delay_ms(5);
    if (otp_pubkey_present) {
        log_printf(
            "[INFO] Dynamic OTP Ed25519 public key: HAS DATA "
            "(0x%08lX - 0x%08lX, %u bytes)\r\n",
            (unsigned long)otp_public_addr,
            (unsigned long)(otp_public_addr + ED25519_PUBLEN - 1U),
            (unsigned)ED25519_PUBLEN);
    } else {
        log_printf(
            "[INFO] Dynamic OTP Ed25519 record: NOT FOUND in 0x%08lX..0x%08lX\r\n",
            (unsigned long)STM32F411_OTP_DATA_START,
            (unsigned long)STM32F411_OTP_DATA_END);
    }
    delay_ms(5);
    if ((manifest_state == FLASH_REGION_STATE_HAS_DATA) && otp_pubkey_present) {
        *already_present = true;
        return MANIFEST_STORE_SUCCESS;
    }
    if ((manifest_state == FLASH_REGION_STATE_HAS_DATA) && !otp_pubkey_present) {
        log_printf("[WARN] Signed manifest exists but dynamic OTP public key is missing\r\n");
    }
    return MANIFEST_STORE_SUCCESS;
}
/* Performs a flash access or protection operation. */
static verify_auth_negtest_status_t load_auth_inputs_from_flash(
    signed_manifest_t *sm_local,
    uint8_t *msg,
    uint8_t *pubkey,
    uint32_t *pubkey_read_buf)
{
    /* Check the flash status and address range before continuing. */
    uint32_t read_ret;
    uint32_t manifest_word_count;
    bool already_present = false;
    manifest_store_status_t precheck_ret;
    uintptr_t otp_public_addr = 0U;
    if ((sm_local == NULL) ||
        (msg == NULL) ||
        (pubkey == NULL) ||
        (pubkey_read_buf == NULL)) {
        return VERIFY_AUTH_NEGTEST_ERR_NULL_PTR;
    }
    memset(sm_local, 0, sizeof(*sm_local));
    memset(msg, 0, MANIFEST_LEN);
    memset(pubkey, 0, ED25519_PUBLEN);
    memset(
        pubkey_read_buf,
        0,
        ((ED25519_PUBLEN + 3U) / 4U) * sizeof(uint32_t));
    manifest_word_count = get_manifest_word_count();
    precheck_ret = check_existing_manifest_and_pubkey(&already_present);
    if (precheck_ret != MANIFEST_STORE_SUCCESS) {
        log_printf(
            "[ERROR] load_auth_inputs_from_flash: precheck failed (%d)\r\n",
            (int)precheck_ret);
        return VERIFY_AUTH_NEGTEST_ERR_FLASH_READ;
    }
    if (!already_present) {
        log_printf(
            "[ERROR] load_auth_inputs_from_flash: "
            "manifest/OTP public key not provisioned\r\n");
        return VERIFY_AUTH_NEGTEST_ERR_NOT_PROVISIONED;
    }
    read_ret = flash_read_data(
        MANIFEST_FLASH_ADDR,
        (uint32_t *)sm_local,
        manifest_word_count);
    if (read_ret != FLASH_READ_OK) {
        log_printf(
            "[ERROR] load_auth_inputs_from_flash: "
            "flash_read_data(manifest) failed: 0x%08lX\r\n",
            (unsigned long)read_ret);
        secure_zero(sm_local, sizeof(*sm_local));
        return VERIFY_AUTH_NEGTEST_ERR_FLASH_READ;
    }
    if (!otp_find_ed25519_record(NULL, &otp_public_addr)) {
        log_printf(
            "[ERROR] load_auth_inputs_from_flash: dynamic OTP Ed25519 record missing\r\n");
        secure_zero(sm_local, sizeof(*sm_local));
        return VERIFY_AUTH_NEGTEST_ERR_NOT_PROVISIONED;
    }
    memcpy(
        pubkey_read_buf,
        (const void *)otp_public_addr,
        ED25519_PUBLEN);
    memcpy(
        pubkey,
        (const uint8_t *)pubkey_read_buf,
        ED25519_PUBLEN);
    if (buffer_is_all_value(sm_local->signature, ED25519_SIGLEN, 0x00U) ||
        buffer_is_all_value(sm_local->signature, ED25519_SIGLEN, 0xFFU)) {
        log_printf(
            "[ERROR] load_auth_inputs_from_flash: signature empty/erased\r\n");
        secure_zero(sm_local, sizeof(*sm_local));
        secure_zero(pubkey, ED25519_PUBLEN);
        secure_zero(
            pubkey_read_buf,
            ((ED25519_PUBLEN + 3U) / 4U) * sizeof(uint32_t));
        return VERIFY_AUTH_NEGTEST_ERR_SIG_EMPTY;
    }
    if (buffer_is_all_value(pubkey, ED25519_PUBLEN, 0x00U) ||
        buffer_is_all_value(pubkey, ED25519_PUBLEN, 0xFFU)) {
        log_printf(
            "[ERROR] load_auth_inputs_from_flash: OTP public key empty/erased\r\n");
        secure_zero(sm_local, sizeof(*sm_local));
        secure_zero(pubkey, ED25519_PUBLEN);
        secure_zero(
            pubkey_read_buf,
            ((ED25519_PUBLEN + 3U) / 4U) * sizeof(uint32_t));
        return VERIFY_AUTH_NEGTEST_ERR_PUBKEY_EMPTY;
    }
    if (serialize_manifest(
            &sm_local->manifest,
            msg,
            MANIFEST_LEN) != FIRMWARE_HASH_SUCCESS) {
        log_printf(
            "[ERROR] load_auth_inputs_from_flash: serialize_manifest failed\r\n");
        secure_zero(sm_local, sizeof(*sm_local));
        secure_zero(msg, MANIFEST_LEN);
        secure_zero(pubkey, ED25519_PUBLEN);
        secure_zero(
            pubkey_read_buf,
            ((ED25519_PUBLEN + 3U) / 4U) * sizeof(uint32_t));
        return VERIFY_AUTH_NEGTEST_ERR_SERIALIZE;
    }
    if (buffer_is_all_value(msg, MANIFEST_LEN, 0x00U)) {
        log_printf(
            "[ERROR] load_auth_inputs_from_flash: "
            "serialized manifest is all zero\r\n");
        secure_zero(sm_local, sizeof(*sm_local));
        secure_zero(msg, MANIFEST_LEN);
        secure_zero(pubkey, ED25519_PUBLEN);
        secure_zero(
            pubkey_read_buf,
            ((ED25519_PUBLEN + 3U) / 4U) * sizeof(uint32_t));
        return VERIFY_AUTH_NEGTEST_ERR_SERIALIZE;
    }
    return VERIFY_AUTH_NEGTEST_PASS;
}
/* Run an Ed25519 verification that is expected to fail. */
static verify_auth_negtest_status_t run_expected_auth_failure(
    sig_keys_ctx_t *ecc_ctx,
    const uint8_t *pubkey,
    const uint8_t *msg,
    const uint8_t *signature,
    const char *test_name)
{
    /* Check the return status before continuing to the next step. */
    cmox_ecc_retval_t ret;
    uint32_t fault_check = 0U;
    if ((ecc_ctx == NULL) || (pubkey == NULL) || (msg == NULL) || (signature == NULL)) {
        return VERIFY_AUTH_NEGTEST_ERR_NULL_PTR;
    }
    log_printf("[TEST] Running authenticity negative test: %s\r\n", test_name);
    ret = cmox_eddsa_verify(
              &ecc_ctx->ecc,
              CMOX_ECC_ED25519_HIGHMEM,
              pubkey,
              ED25519_PUBLEN,
              msg,
              MANIFEST_LEN,
              signature,
              ED25519_SIGLEN,
              &fault_check
          );
    cmox_ecc_cleanup(&ecc_ctx->ecc);
    if ((ret == CMOX_ECC_AUTH_SUCCESS) && (fault_check == CMOX_ECC_AUTH_SUCCESS)) {
        log_printf("[TEST FAIL] %s: verification unexpectedly succeeded\r\n", test_name);
        return VERIFY_AUTH_NEGTEST_ERR_UNEXPECTED_SUCCESS;
    }
    if ((ret == CMOX_ECC_AUTH_FAIL) ||
        (ret == CMOX_ECC_ERR_INVALID_SIGNATURE) ||
        (ret == CMOX_ECC_ERR_INVALID_PUBKEY)) {
        log_printf("[TEST PASS] %s: authenticity check rejected tampered input as expected (ret=%d)\r\n",
                   test_name, (int)ret);
        return VERIFY_AUTH_NEGTEST_PASS;
    }
    if ((ret != CMOX_ECC_AUTH_SUCCESS) && (ret != CMOX_ECC_AUTH_FAIL)) {
        log_printf("[TEST ERROR] %s: ECC verify returned unexpected error (ret=%d, fault_check=0x%08lX)\r\n",
                   test_name, (int)ret, (unsigned long)fault_check);
        return VERIFY_AUTH_NEGTEST_ERR_ECC_VERIFY;
    }
    log_printf("[TEST ERROR] %s: unexpected verify state (ret=%d, fault_check=0x%08lX)\r\n",
               test_name, (int)ret, (unsigned long)fault_check);
    return VERIFY_AUTH_NEGTEST_ERR_FAULT_CHECK;
}
/* Ed25519 authenticity fault-injection tests. */
/**
 * @brief Run the Ed25519 negative test with a locally corrupted public key.
 * @param ecc_ctx Prepared Ed25519 context.
 * @return Negative-test status.
 */
verify_auth_negtest_status_t test_auth_fail_with_bad_pubkey(sig_keys_ctx_t *ecc_ctx)
{
    /* Validate the key data before using it in crypto operations. */
    signed_manifest_t sm_local;
    uint8_t msg[MANIFEST_LEN];
    uint8_t pubkey[ED25519_PUBLEN];
    uint32_t pubkey_read_buf[(ED25519_PUBLEN + 3U) / 4U];
    verify_auth_negtest_status_t st;
    st = load_auth_inputs_from_flash(&sm_local, msg, pubkey, pubkey_read_buf);
    if (st != VERIFY_AUTH_NEGTEST_PASS) {
        return st;
    }
    pubkey[0] ^= 0xA5U;
    st = run_expected_auth_failure(
            ecc_ctx,
            pubkey,
            msg,
            sm_local.signature,
            "bad public key"
         );
    secure_zero(&sm_local, sizeof(sm_local));
    secure_zero(msg, sizeof(msg));
    secure_zero(pubkey, sizeof(pubkey));
    secure_zero(pubkey_read_buf, sizeof(pubkey_read_buf));
    return st;
}
/**
 * @brief Run the Ed25519 negative test with a locally corrupted signature.
 * @param ecc_ctx Prepared Ed25519 context.
 * @return Negative-test status.
 */
verify_auth_negtest_status_t test_auth_fail_with_bad_signature(sig_keys_ctx_t *ecc_ctx)
{
    /* Check the signature operation status before using the result. */
    signed_manifest_t sm_local;
    uint8_t msg[MANIFEST_LEN];
    uint8_t pubkey[ED25519_PUBLEN];
    uint8_t signature[ED25519_SIGLEN];
    uint32_t pubkey_read_buf[(ED25519_PUBLEN + 3U) / 4U];
    verify_auth_negtest_status_t st;
    st = load_auth_inputs_from_flash(&sm_local, msg, pubkey, pubkey_read_buf);
    if (st != VERIFY_AUTH_NEGTEST_PASS) {
        return st;
    }
    memcpy(signature, sm_local.signature, ED25519_SIGLEN);
    signature[0] ^= 0x5AU;
    st = run_expected_auth_failure(
            ecc_ctx,
            pubkey,
            msg,
            signature,
            "bad signature"
         );
    secure_zero(&sm_local, sizeof(sm_local));
    secure_zero(msg, sizeof(msg));
    secure_zero(pubkey, sizeof(pubkey));
    secure_zero(signature, sizeof(signature));
    secure_zero(pubkey_read_buf, sizeof(pubkey_read_buf));
    return st;
}
/**
 * @brief Run the Ed25519 negative test with a locally corrupted manifest message.
 * @param ecc_ctx Prepared Ed25519 context.
 * @return Negative-test status.
 */
verify_auth_negtest_status_t test_auth_fail_with_bad_manifest(sig_keys_ctx_t *ecc_ctx)
{
    /* Reject the firmware if signature verification fails. */
    signed_manifest_t sm_local;
    uint8_t msg[MANIFEST_LEN];
    uint8_t pubkey[ED25519_PUBLEN];
    uint32_t pubkey_read_buf[(ED25519_PUBLEN + 3U) / 4U];
    verify_auth_negtest_status_t st;
    st = load_auth_inputs_from_flash(&sm_local, msg, pubkey, pubkey_read_buf);
    if (st != VERIFY_AUTH_NEGTEST_PASS) {
        return st;
    }
    msg[0] ^= 0x01U;
    st = run_expected_auth_failure(
            ecc_ctx,
            pubkey,
            msg,
            sm_local.signature,
            "bad manifest/message"
         );
    secure_zero(&sm_local, sizeof(sm_local));
    secure_zero(msg, sizeof(msg));
    secure_zero(pubkey, sizeof(pubkey));
    secure_zero(pubkey_read_buf, sizeof(pubkey_read_buf));
    return st;
}
