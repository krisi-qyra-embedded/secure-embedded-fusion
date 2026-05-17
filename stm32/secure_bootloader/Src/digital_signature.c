/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */


#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <limits.h>
#include <stdbool.h>
#include "digital_signature.h"
#include "flash_protection.h"
#include <string.h>

signed_manifest_t g_signed_manifest;
sig_keys_ctx_t g_sig_keys;
/**
 * @brief Hash firmware binary using SHA-256 (mimicking hash_adc_pc4 structure)
 *
 * @param firmware_addr: Start address of firmware in flash
 * @param firmware_size: Size of firmware in bytes
 * @param out_hash: Output buffer for 32-byte SHA-256 hash
 * @param out_len: Expected output length (must be 32)
 * @return firmware_hash_status_t (enum)
 */
firmware_hash_status_t hash_firmware_binary(uint32_t firmware_addr,
                                           uint32_t firmware_size,
                                           uint8_t *out_hash,
                                           size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;

    /* ===============================================
     * STEP 1: VALIDATE INPUTS (like hash_adc_pc4)
     * =============================================== */

    if (out_hash == NULL) {
        log_printf("hash_firmware_binary: NULL output pointer\r\n");
        return FIRMWARE_HASH_ERR_NULL_INPUT;
    }

    if (out_len != SHA256_HASH_SIZE) {
        log_printf("hash_firmware_binary: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, SHA256_HASH_SIZE);
        return FIRMWARE_HASH_ERR_BAD_LENGTH;
    }

    if (firmware_size == 0 || firmware_size > 0x00100000) {  /* Sanity: max 1MB */
        log_printf("hash_firmware_binary: invalid firmware size (%lu bytes)\r\n",
                   (unsigned long)firmware_size);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }

    /* ===============================================
     * STEP 2: POINT TO FIRMWARE IN FLASH
     * (No fopen needed - firmware is already there!)
     * =============================================== */

    const uint8_t *firmware_data = (const uint8_t *)firmware_addr;

    /* ===============================================
     * STEP 3: COMPUTE SHA-256 HASH
     * (Using cmox_hash_compute - mimics your example)
     * =============================================== */

    out_len_local = out_len;

    retval = cmox_hash_compute(
        CMOX_SHA256_ALGO,           /* Hash algorithm */
        firmware_data,              /* ← Firmware bytes (in flash) */
        firmware_size,              /* ← Size of firmware */
        out_hash,                   /* ← Output: 32-byte hash */
        out_len,                    /* ← Expected: 32 */
        &out_len_local              /* ← Actual output length */
    );

    /* ===============================================
     * STEP 4: VALIDATE HASH RESULT (like hash_adc_pc4)
     * =============================================== */

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

/* Computes or validates a cryptographic digest. */
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
 * @brief  Hash large firmware in chunks (streaming mode)
 * @return firmware_hash_status_t
 */
firmware_hash_status_t hash_firmware_chunked(uint32_t firmware_addr,
                                             uint32_t firmware_size,
                                             uint8_t *out_hash,
                                             size_t out_len)
{
    cmox_hash_retval_t retval;
    cmox_sha256_handle_t sha_ctx;
    cmox_hash_handle_t *hash_handle;

    uint32_t bytes_remaining = firmware_size;
    uint32_t current_addr = firmware_addr;
    size_t hash_len = 0;

    /* Validate inputs */
    if (out_hash == NULL) {
        log_printf("hash_firmware_chunked: NULL output pointer\r\n");
        return FIRMWARE_HASH_ERR_NULL_INPUT;
    }
    if (out_len != SHA256_HASH_SIZE) {
        log_printf("hash_firmware_chunked: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, SHA256_HASH_SIZE);
        return FIRMWARE_HASH_ERR_BAD_LENGTH;
    }
    if (firmware_size == 0 || firmware_size > 0x00100000U) { /* sanity limit 1MB */
        log_printf("hash_firmware_chunked: invalid firmware size (%lu bytes)\r\n",
                   (unsigned long)firmware_size);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }

    /* Construct SHA-256 context and init */
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

    /* Stream chunks from flash memory */
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

        /* Optional progress logging */
        if (((firmware_size - bytes_remaining) % (16U * 1024U)) == 0U) {
            log_printf("  Hashed %lu / %lu bytes\r\n",
                       (unsigned long)(firmware_size - bytes_remaining),
                       (unsigned long)firmware_size);
        }
    }

    /* Finalize */
    retval = cmox_hash_generateTag(hash_handle, out_hash, &hash_len);
    if (retval != CMOX_HASH_SUCCESS || hash_len != SHA256_HASH_SIZE) {
        log_printf("hash_firmware_chunked: finalization failed (ret=%d, len=%lu)\r\n",
                   retval, (unsigned long)hash_len);
        cmox_hash_cleanup(hash_handle);
        memset(out_hash, 0, out_len);
        return FIRMWARE_HASH_ERR_HASH_FAILED;
    }

    /* Cleanup (best-effort) */
    retval = cmox_hash_cleanup(hash_handle);
    if (retval != CMOX_HASH_SUCCESS) {
        log_printf("hash_firmware_chunked: cleanup returned %d (non-fatal)\r\n", retval);
    }

    log_printf("✓ Firmware hashed successfully (chunked mode)\r\n");
    return FIRMWARE_HASH_SUCCESS;
}


/**
 * Build the manifest (fill fields + compute firmware SHA-256) and return status.
 * Uses the enum firmware_hash_status_t for return values.
 *
 * Side-effects:
 *  - prints manifest fields (including rollback_counter) on success
 *  - manifest is local (caller can adapt to store/sign it next)
 */
firmware_hash_status_t create_and_hash_manifest(firmware_manifest_t *manifest)
{
    firmware_hash_status_t rc;

    /* --- Basic validation (defensive) --- */
    if (FIRMWARE_SIZE == 0 || FIRMWARE_SIZE > 0x00100000U) {
        log_printf("create_and_hash_manifest: invalid FIRMWARE_SIZE (%lu)\r\n",
                   (unsigned long)FIRMWARE_SIZE);
        return FIRMWARE_HASH_ERR_INVALID_SIZE;
    }

    /* --- Step 1: Fill manifest fields --- */
    manifest->version = FIRMWARE_VERSION;
    manifest->firmware_size = FIRMWARE_SIZE;
    manifest->rollback_counter = 5; /* example; set properly in release process */

    /* Zero hash buffer before use (defensive) */
    memset(manifest->sha256_hash, 0, sizeof(manifest->sha256_hash));

    /* --- Step 2: Hash the firmware (streaming/chunked recommended) --- */
    rc = hash_firmware_chunked(
        FIRMWARE_START_ADDR,
        manifest->firmware_size,
        manifest->sha256_hash,
        sizeof(manifest->sha256_hash)
    );

    if (rc != FIRMWARE_HASH_SUCCESS) {
        log_printf("create_and_hash_manifest: firmware hashing failed (code=%d)\r\n", rc);
        /* ensure no leftover sensitive data */
        memset(manifest->sha256_hash, 0, sizeof(manifest->sha256_hash));
        return rc;
    }

    /* Additional sanity: ensure hash is not all zeros (very unlikely) */
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

    /* --- Step 3: Print manifest (including rollback counter) --- */
    log_printf("✓ Manifest ready:\r\n");
    log_printf("  Version:        0x%08X\r\n", manifest->version);
    log_printf("  Size:           %lu bytes\r\n", (unsigned long)manifest->firmware_size);
    log_printf("  Rollback count: %lu\r\n", (unsigned long)manifest->rollback_counter);
    log_printf("  Hash: ");
    for (int i = 0; i < (int)sizeof(manifest->sha256_hash); ++i) {
        log_printf("%02X", manifest->sha256_hash[i]);
    }
    log_printf("\r\n");

    /* Manifest built and hashed successfully; caller can sign/store it next */
    return FIRMWARE_HASH_SUCCESS;
}


/* ---- Safer BE writer ---- */
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

/* ---- Robust serializer ---- */
bool serialize_manifest(firmware_manifest_t *m,
                        uint8_t *out,
                        size_t out_len)
{
    if (!m || !out || out_len < MANIFEST_LEN)
        return true;

    /* deterministic output */
    memset(out, 0, MANIFEST_LEN);

    if (!write_u32_be(out, out_len, OFF_VERSION, m->version)) return false;
    if (!write_u32_be(out, out_len, OFF_FW_SIZE, m->firmware_size)) return false;

    memcpy(out + OFF_HASH, m->sha256_hash, SHA256_LEN);

    if (!write_u32_be(out, out_len, OFF_ROLLBK, m->rollback_counter)) return false;

    return false;
}


/*
 * Sign the canonical manifest buffer using Ed25519.
 * - ecc_ctx      : prepared cmox ECC context (non-NULL)
 * - privkey      : host-side private key buffer (secret||public)
 * - privkey_len  : length of privkey (must be >= ED25519_PRIVLEN)
 * - manifest     : pointer to manifest struct to serialize and sign
 * - out_sig      : output buffer where signature will be written
 * - out_sig_len  : IN: capacity of out_sig; OUT: signature length written
 *
 * Returns sign_manifest_status_t (enum).
 */
/*
 * Sign the canonical manifest buffer using Ed25519.
 * This version calls create_and_hash_manifest() before serializing the manifest.
 * If create_and_hash_manifest() fails, we log and return an error.
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

    /* Validate pointers */
    if (ecc_ctx == NULL || privkey == NULL || manifest == NULL || out_sig == NULL || out_sig_len == NULL) {
        return SIGN_MANIFEST_ERR_NULL_PTR;
    }
    /* Validate provided buffers/lengths */
    if (privkey_len < ED25519_PRIVLEN) {
        return SIGN_MANIFEST_ERR_BAD_PRIVKEY_LEN;
    }
    if (*out_sig_len < ED25519_SIGLEN) {
        return SIGN_MANIFEST_ERR_SIGBUF_TOO_SMALL;
    }

    /* --- Ensure manifest is fresh and hashed --- */
    /* Call the creation/hashing routine. It may be a no-op if manifest already filled by caller.
       We only check for success here and bail if it failed. */
    mrc = create_and_hash_manifest(manifest);
    if (mrc != FIRMWARE_HASH_SUCCESS) {
        log_printf("sign_manifest_and_write_sig: create_and_hash_manifest failed (code=%d)\r\n", mrc);
        return SIGN_MANIFEST_ERR_MANIFEST_CREATE;
    }

    /* Serialize manifest into canonical byte buffer (big-endian) */
    if (serialize_manifest(manifest, msg, MANIFEST_LEN) != FIRMWARE_HASH_SUCCESS) {
        log_printf("sign_manifest_and_write_sig: serialize_manifest failed\r\n");
        memset(msg, 0, MANIFEST_LEN);
        return SIGN_MANIFEST_ERR_SERIALIZE;
    }

    /* Basic sanity check: manifest bytes must be non-zero (defensive) */
    {
        int all_zero = 1;
        for (size_t i = 0; i < MANIFEST_LEN; ++i) {
            if (msg[i] != 0) { all_zero = 0; break; }
        }
        if (all_zero) {
            /* treat as serialization error */
            memset(msg, 0, MANIFEST_LEN);
            return SIGN_MANIFEST_ERR_SERIALIZE;
        }
    }


    /* Call library signing function */
    ret = cmox_eddsa_sign(
    	&ecc_ctx->ecc,
		CMOX_ECC_ED25519_HIGHMEM,
        privkey,
        privkey_len,
        msg,
        MANIFEST_LEN,
        out_sig,
        out_sig_len   /* library writes actual sig length here */
    );

    /* Zero sensitive temporary buffer immediately */
    {
        volatile uint8_t *p = msg;
        for (size_t i = 0; i < MANIFEST_LEN; ++i) p[i] = 0;
    }

    /* Check library return */
    if (ret != CMOX_ECC_SUCCESS) {
        /* map any library error to a single failure code (caller can log ret if needed) */
        log_printf("sign_manifest_and_write_sig: cmox_eddsa_sign failed (ret=%d)\r\n", ret);
        return SIGN_MANIFEST_ERR_SIGN_FAIL;
    }

    /* Validate signature length */
    if (*out_sig_len != ED25519_SIGLEN) {
        /* wipe partial signature before returning */
        if (*out_sig_len > 0 && *out_sig_len <= ED25519_SIGLEN) {
            volatile uint8_t *q = out_sig;
            for (size_t i = 0; i < *out_sig_len; ++i) q[i] = 0;
        }
        log_printf("sign_manifest_and_write_sig: signature length mismatch (%lu)\r\n", (unsigned long)*out_sig_len);
        return SIGN_MANIFEST_ERR_SIGLEN_MISMATCH;
    }

    return SIGN_MANIFEST_SUCCESS;
}


/* Internal helper: get word-aligned size */
static inline uint32_t get_manifest_word_count(void)
{
    /* Check the return status before continuing to the next step. */
    return (sizeof(signed_manifest_t) + 3U) / 4U;
}

/* returns number of bytes required to hold the struct */
static inline uint32_t get_manifest_byte_count(void)
{
    /* Check the return status before continuing to the next step. */
    return sizeof(signed_manifest_t);
}

/* Stores the signed manifest in flash. */
manifest_store_status_t store_signed_manifest_protected( signed_manifest_t *sm)
{
    uint8_t wrp_before, wrp_after;
    signed_manifest_t readback;
    uint32_t word_count;
    uint32_t read_ret;
    uint32_t write_ret;

    /* -------- Input validation -------- */
    if (sm == NULL) {
        log_printf("[ERROR] store_signed_manifest: NULL input\r\n");
        return MANIFEST_STORE_ERR_NULL_INPUT;
    }

    word_count = get_manifest_word_count();

    log_printf("[INFO] Storing signed manifest (%lu bytes, %lu words) to 0x%08lX\r\n",
               (unsigned long)sizeof(signed_manifest_t),
               (unsigned long)word_count,
               (unsigned long)MANIFEST_FLASH_ADDR);
    delay_ms(5);
    /* -------- Step 1: Disable WRP if previously enabled (for re-provisioning) -------- */
    wrp_before = flash_obytes_get_wrp();
    if ((wrp_before & (1U << 7)) == 0) {  /* Sector 7 protected if bit is 0 */
        log_printf("[INFO] Sector 7 currently protected, disabling WRP for update\r\n");
        dis_wrp_all_sectors();  /* Or targeted: dis_wrp_sector(MANIFEST_SECTOR) if available */
        delay_ms(5);
    }

    /* -------- Step 2: Erase sector before write (flash requires erase first) -------- */
    log_printf("[INFO] Erasing manifest sector (%u) before write\r\n", MANIFEST_SECTOR_NUM);
    delay_ms(5);
    /* Use convenience wrapper that returns flash_erase_status_t and logs details */
    flash_erase_status_t erase_ret = erase_manifest_sector();
    if (erase_ret != FLASH_ERASE_SUCCESS) {
        log_printf("[FATAL] Manifest sector erase failed: %s (%d)\r\n",
                   flash_erase_status_str(erase_ret), (int)erase_ret);
        return MANIFEST_STORE_ERR_FLASH_ERASE; /* make sure this constant exists */
    }
    log_printf("[OK] Manifest sector erased successfully\r\n");
    delay_ms(10);

    /* -------- Step 3: Write signed manifest to flash -------- */
     write_ret = flash_write_data(MANIFEST_FLASH_ADDR, (uint32_t *)sm, word_count);
    if (write_ret != 0) {
        log_printf("[ERROR] flash_write_data failed with error: 0x%08lX\r\n", (unsigned long)write_ret);
        return MANIFEST_STORE_ERR_FLASH_WRITE;
    }
    log_printf("[OK] Manifest written to flash at 0x%08lX\r\n", (unsigned long)MANIFEST_FLASH_ADDR);
    delay_ms(5);
    /* -------- Step 4: Verify write integrity -------- */
    memset(&readback, 0, sizeof(readback));

    read_ret = flash_read_data(MANIFEST_FLASH_ADDR, (uint32_t *)&readback, word_count);
    if (read_ret != FLASH_READ_OK) {
        log_printf("[ERROR] flash_read_data failed with error: 0x%08lX\r\n", (unsigned long)read_ret);
        /* handle error */
        return MANIFEST_STORE_ERR_FLASH_READ;
    }

    if (memcmp(sm, &readback, sizeof(signed_manifest_t)) != 0) {
        log_printf("[ERROR] Manifest verification failed: readback does not match written data\r\n");
        return MANIFEST_STORE_ERR_VERIFY_FAIL;
    }
    log_printf("[OK] Manifest verify successful\r\n");
    delay_ms(5);

    /* -------- Step 5: Enable write protection on sector 7 -------- */
    log_printf("[INFO] Enabling WRP (Flash Write protection) on sector 7\r\n");
    sector_set_wrp(MANIFEST_SECTOR);
    delay_ms(10);

    /* -------- Step 6: Verify WRP is active -------- */
    wrp_after = flash_obytes_get_wrp();
    log_printf("[INFO] WRP register: before=0x%02X, after=0x%02X\r\n",
               (unsigned)wrp_before, (unsigned)wrp_after);
    delay_ms(5);
    /* On STM32F4, WRP bit = 0 means protected, bit = 1 means unprotected */
    if ((wrp_after & (1U << 7)) != 0) {
        log_printf("[ERROR] Failed to enable WRP on sector 7\r\n");
        return MANIFEST_STORE_ERR_WRP_FAIL;
    }

    log_printf("[OK] Signed manifest stored and protected at 0x%08lX\r\n",
               (unsigned long)MANIFEST_FLASH_ADDR);
    delay_ms(5);
    return MANIFEST_STORE_SUCCESS;
}

/* Helper to print byte array as hex */
static void print_hex_array(const char *label, const uint8_t *data, size_t len)
{
    log_printf("%s: ", label);
    for (size_t i = 0; i < len; ++i) {
        log_printf("%02X", data[i]);
        if ((i + 1) % 16 == 0 && i + 1 < len) {
            log_printf("\r\n%*s  ", (int)strlen(label), "");  /* align continuation */
        }
}
}


/* Loads the signed manifest from its flash location. */
manifest_store_status_t load_signed_manifest(signed_manifest_t *sm_out)
{
    if (sm_out == NULL) {
        return MANIFEST_STORE_ERR_NULL_INPUT;
    }

    uint32_t word_count = get_manifest_word_count();
    flash_read_data(MANIFEST_FLASH_ADDR, (uint32_t *)sm_out, word_count);

    log_printf("[INFO] Loaded signed manifest from 0x%08lX\r\n",
               (unsigned long)MANIFEST_FLASH_ADDR);
    delay_ms(10);
             /* ============================================================
                  * FLASH ADDRESS MAP FOR signed_manifest_t (packed)
                  * ============================================================
                  * Base address: MANIFEST_FLASH_ADDR (0x08060000)
                  *
                  * Field             Offset   Size   Start Addr    End Addr
                  * ---------------------------------------------------------
                  * version              0      4     0x08060000    0x08060003
                  * firmware_size        4      4     0x08060004    0x08060007
                  * sha256_hash          8     32     0x08060008    0x08060027
                  * rollback_counter    40      4     0x08060028    0x0806002B
                  * signature           44     64     0x0806002C    0x0806006B
                  * ---------------------------------------------------------
                  * TOTAL:              108 bytes
                  * ============================================================ */
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

                 /* version */
                 log_printf("║ version          │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX            ║\r\n",
                 (unsigned long)offsetof(signed_manifest_t, manifest.version),
                 (unsigned long)sizeof(sm_out->manifest.version),
                 (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.version)),
                 (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.version)
                 + sizeof(sm_out->manifest.version) - 1));
                 delay_ms(10);
                 /* firmware_size */
                 log_printf("║ firmware_size    │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX       ║\r\n",
                 (unsigned long)offsetof(signed_manifest_t, manifest.firmware_size),
                 (unsigned long)sizeof(sm_out->manifest.firmware_size),
                 (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.firmware_size)),
                 (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.firmware_size)
                 + sizeof(sm_out->manifest.firmware_size) - 1));
                 delay_ms(10);
                 /* sha256_hash */
                 log_printf("║ sha256_hash      │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX       ║\r\n",
                 (unsigned long)offsetof(signed_manifest_t, manifest.sha256_hash),
                 (unsigned long)sizeof(sm_out->manifest.sha256_hash),
                 (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.sha256_hash)),
                 (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.sha256_hash)
                 + sizeof(sm_out->manifest.sha256_hash) - 1));
                 delay_ms(10);
                 /* rollback_counter */
                 log_printf("║ rollback_counter │ %4lu   │ %3lu  │ 0x%08lX │ 0x%08lX       ║\r\n",
                 (unsigned long)offsetof(signed_manifest_t, manifest.rollback_counter),
                 (unsigned long)sizeof(sm_out->manifest.rollback_counter),
                 (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.rollback_counter)),
                 (unsigned long)(MANIFEST_FLASH_ADDR + offsetof(signed_manifest_t, manifest.rollback_counter)
                 + sizeof(sm_out->manifest.rollback_counter) - 1));
                 delay_ms(10);
                 /* signature */
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
                 /* ============================================================
                 * PRINT ACTUAL VALUES
                 * ============================================================ */
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

                 /* Print SHA256 hash (32 bytes) */
                 log_printf("│ sha256_hash:\r\n│   ");
                 delay_ms(5);
                 for (size_t i = 0; i < 32; ++i) {
                     log_printf("%02X", sm_out->manifest.sha256_hash[i]);
                     delay_ms(5);
                     if (i == 15) log_printf("\r\n│   ");  /* wrap at 16 bytes */
                 }
                 log_printf("\r\n");
                 delay_ms(5);
                 log_printf("├──────────────────────────────────────────────────────────────────┤\r\n");

                 /* Print Ed25519 signature (64 bytes) */
                 log_printf("│ signature (Ed25519, 64 bytes):\r\n│   ");
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
                     /* ============================================================
                          * VALIDATION CHECKS
                          * ============================================================ */

                         /* 1) Check manifest fields plausibility */
                 if (sm_out->manifest.firmware_size == 0 ||
                     sm_out->manifest.firmware_size > MAX_EXPECTED_FIRMWARE_SIZE) {
                 log_printf("[WARN] Loaded manifest has invalid firmware_size=%lu\r\n",
                 (unsigned long)sm_out->manifest.firmware_size);
                 delay_ms(5);
                 secure_zero(sm_out, word_count * sizeof(uint32_t));
                 return MANIFEST_STORE_ERR_INVALID_MANIFEST;
                 }
                 /* 2) Ensure SHA256 field is not all-zeros */
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

                 /* 3) Ensure signature is present (not all-zero) */
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

    return MANIFEST_STORE_SUCCESS;
}




