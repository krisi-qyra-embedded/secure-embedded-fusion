/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include <entropy_personalization_nonce_gen.h>

uint8_t device_uid_buffer_hashed[DEVICE_UID_HASH_LEN];
extern volatile uint8_t uid_read;
extern uint8_t device_uid_buffer[UID_SIZE_BYTES];
extern volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE]; 
//jitter_status_t rc;
uint8_t buf_pc4_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t buf_pc5_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t buf_pb0_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t jitter_accum_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t sram_entropy_buffer_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t nonce_hashed[NONCE_HASH_LEN] = {0};
uint8_t entropy_hashed[ENTROPY_HASH_LEN] = {0};

/* Computes a SHA-256 digest of the device UID. */
int hash_device_uid_into_hashed_buf(uint8_t *out32, size_t out32_len)
{
    /* Clear the output when hashing fails or returns an unexpected length. */
    cmox_hash_retval_t retval;
    size_t out_len_local;

    if (out32 == NULL) return DEVICE_UID_HASH_ERR_BAD_INPUT;
    if (out32_len != DEVICE_UID_HASH_LEN) return DEVICE_UID_HASH_ERR_BAD_INPUT;

    if (!uid_read) {
        device_uid_read();
    }

    out_len_local = out32_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               device_uid_buffer, sizeof(device_uid_buffer),
                               out32, out32_len,
                               &out_len_local);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out32_len) {
        memset(out32, 0, out32_len);
        return DEVICE_UID_HASH_ERR_HASH_FAILED;
    }

    return DEVICE_UID_HASH_SUCCESS;
}

/* Updates the device UID personalization digest. */
int compute_device_uid_hashed_global(void)
{
    /* Write the digest into the shared UID hash buffer. */
    return hash_device_uid_into_hashed_buf(device_uid_buffer_hashed, DEVICE_UID_HASH_LEN);
}

/* Tests whether a buffer is null or contains only zero bytes. */
static int is_buffer_empty(const uint8_t *buf, size_t len)
{
    /* Return as soon as a nonzero byte is found. */
    if (buf == NULL) return 1;   

    for (size_t i = 0; i < len; i++) {
        if (buf[i] != 0) return 0;   
    }
    return 1;   
}

/* ========== ADC PC4 Hashing ========== */

/* Computes SHA-256 over the PC4 sample bytes. */
int hash_adc_pc4(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    /* Validate the digest length and clear the output on a hash failure. */
    cmox_hash_retval_t retval;
    size_t out_len_local;

    if (in_buffer == NULL || out_hash == NULL) {
        log_printf("hash_adc_pc4: NULL input pointer\r\n");
        return ENTROPY_HASH_ERR_NULL_INPUT;
    }

    if (out_len != ENTROPY_HASH_LEN) {
        log_printf("hash_adc_pc4: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, ENTROPY_HASH_LEN);
        return ENTROPY_HASH_ERR_BAD_LENGTH;
    }

    if (is_buffer_empty(in_buffer, BUF_SZ)) {
        log_printf("hash_adc_pc4: input buffer is empty (all zeros) – aborting\r\n");
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_BUFFER_EMPTY;
    }

    out_len_local = out_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               in_buffer, BUF_SZ,
                               out_hash, out_len,
                               &out_len_local);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        log_printf("hash_adc_pc4: SHA-256 computation failed (retval=%d, out_len=%lu)\r\n",
                   retval, (unsigned long)out_len_local);
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_HASH_FAILED;
    }

    return ENTROPY_HASH_SUCCESS;
}

/* Updates the shared PC4 ADC digest. */
int compute_adc_pc4_hashed_global(void)
{
    /* Condition the PC4 byte buffer into its channel hash buffer. */
    return hash_adc_pc4(buf_pc4, buf_pc4_hashed, ADC_HASH_LEN);
}

/* ========== ADC PC5 Hashing ========== */

/* Computes SHA-256 over the PC5 sample bytes. */
int hash_adc_pc5(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    /* Validate the digest length and clear the output on a hash failure. */
    cmox_hash_retval_t retval;
    size_t out_len_local;

    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    out_len_local = out_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               in_buffer, BUF_SZ,
                               out_hash, out_len,
                               &out_len_local);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_HASH_FAILED;
    }

    return ENTROPY_HASH_SUCCESS;
}

/* Updates the shared PC5 ADC digest. */
int compute_adc_pc5_hashed_global(void)
{
    /* Condition the PC5 byte buffer into its channel hash buffer. */
    return hash_adc_pc5(buf_pc5, buf_pc5_hashed, ADC_HASH_LEN);
}

/* ========== ADC PB0 Hashing ========== */

/* Computes SHA-256 over the PB0 sample bytes. */
int hash_adc_pb0(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    /* Validate the digest length and clear the output on a hash failure. */
    cmox_hash_retval_t retval;
    size_t out_len_local;

    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    out_len_local = out_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               in_buffer, BUF_SZ,
                               out_hash, out_len,
                               &out_len_local);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_HASH_FAILED;
    }

    return ENTROPY_HASH_SUCCESS;
}

/* Updates the shared PB0 ADC digest. */
int compute_adc_pb0_hashed_global(void)
{
    /* Condition the PB0 byte buffer into its channel hash buffer. */
    return hash_adc_pb0(buf_pb0, buf_pb0_hashed, ADC_HASH_LEN);
}

/* Computes the three ADC channel digests with retries. */
int compute_all_adc_hashes(void)
{
    /* Return the channel-specific failure when its retry budget is exhausted. */
    int rv;
    int attempt;
    for (attempt = 0; attempt < ADC_HASH_RETRY_COUNT; ++attempt) {
        rv = compute_adc_pc4_hashed_global();
        if (rv == 0) break;
        log_printf("compute_all_adc_hashes: attempt %d PC4 hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != 0) {
        log_printf("compute_all_adc_hashes: ADC PC4 hashing ultimately failed.\r\n");
        return ADC_PC4_HASH_FAILED;
    }

    for (attempt = 0; attempt < ADC_HASH_RETRY_COUNT; ++attempt) {
        rv = compute_adc_pc5_hashed_global();
        if (rv == 0) break;
        log_printf("compute_all_adc_hashes: attempt %d PC5 hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != 0) {
        log_printf("compute_all_adc_hashes: ADC PC5 hashing ultimately failed.\r\n");
        return ADC_PC5_HASH_FAILED;
    }

    for (attempt = 0; attempt < ADC_HASH_RETRY_COUNT; ++attempt) {
        rv = compute_adc_pb0_hashed_global();
        if (rv == 0) break;
        log_printf("compute_all_adc_hashes: attempt %d PB0 hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != 0) {
        log_printf("compute_all_adc_hashes: ADC PB0 hashing ultimately failed.\r\n");
        return ADC_PB0_HASH_FAILED;
    }

    return ADC_HASH_ALL_SUCCESS;
}

/* ========== Jitter Accumulator Hashing ========== */

/**
 * Hash jitter accumulator array (256 uint32_t values = 1024 bytes) using SHA-256
 */
/* Computes SHA-256 over the timer difference array. */
int hash_jitter_accum(const uint32_t *in_jitter, uint8_t *out_hash, size_t out_len)
{
    /* Hash the full array as bytes and validate the returned digest size. */
    cmox_hash_retval_t retval;
    size_t out_len_local;

    if (in_jitter == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    /* Cast uint32_t array to uint8_t for hashing
       Total size: 256 values * 4 bytes = 1024 bytes */
    const uint8_t *jitter_bytes = (const uint8_t *)in_jitter;
    size_t jitter_bytes_len = JITTER_CAPTURE_SIZE * sizeof(uint32_t);

    out_len_local = out_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               jitter_bytes, jitter_bytes_len,
                               out_hash, out_len,
                               &out_len_local);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_HASH_FAILED;
    }

    return ENTROPY_HASH_SUCCESS;
}

/* Updates the shared timer-jitter digest. */
int compute_jitter_accum_hashed_global(void)
{
    /* Condition the captured differences into the global jitter hash buffer. */
    return hash_jitter_accum(jitter_differences, jitter_accum_hashed, JITTER_HASHED);
}

/* ========== SRAM Entropy Buffer Hashing ========== */

/* Computes SHA-256 over the SRAM sample buffer. */
int hash_sram_entropy(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    /* Clear the output if the digest operation or output length is invalid. */
    cmox_hash_retval_t retval;
    size_t out_len_local;

    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    out_len_local = out_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               in_buffer, SRAM_ENTROPY_BUFFER_SIZE,
                               out_hash, out_len,
                               &out_len_local);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_HASH_FAILED;
    }

    return ENTROPY_HASH_SUCCESS;
}

/* Updates the shared SRAM digest. */
int compute_sram_entropy_hashed_global(void)
{
    /* Pass the retained sample buffer to the SHA-256 helper. */
    return hash_sram_entropy((const uint8_t *)sram_entropy_buffer,
                             sram_entropy_buffer_hashed,
                             SRAM_ENTROPY_HASH_LEN);
}

/* Conditions the ADC, jitter, SRAM and UID inputs. */
int entropy_personalisation_hash(void)
{
    /* Stop on a failed component rather than publishing a partial combined digest. */
    int rv;
    int attempt;

    secure_zero(entropy_hashed, ENTROPY_HASH_LEN);

    /* ---------- Step 1: ADC hashes ---------- */
    log_printf("entropy_personalisation_hash: starting ADC hashes\r\n");
    rv = compute_all_adc_hashes();
    if (rv != ADC_HASH_ALL_SUCCESS) {
        log_printf("entropy_personalisation_hash: compute_all_adc_hashes failed (rv=%d)\r\n", rv);
        return rv;  
    }
    log_printf("entropy_personalisation_hash: ADC hashes succeeded\r\n");

    /* ---------- Step 2: Jitter accumulation hash (with retries) ---------- */
    log_printf("entropy_personalisation_hash: starting jitter accumulation hash\r\n");
    rv = ENTROPY_HASH_ERR_HASH_FAILED;
    for (attempt = 0; attempt < ENTROPY_CREATION_RETRY_COUNT; ++attempt) {
        rv = compute_jitter_accum_hashed_global();
        if (rv == ENTROPY_HASH_SUCCESS) break;
        log_printf("entropy_personalisation_hash: attempt %d jitter hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != ENTROPY_HASH_SUCCESS) {
        log_printf("entropy_personalisation_hash: jitter hashing ultimately failed.\r\n");
        secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
        return JITTER_HASH_FAILED;
    }
    log_printf("entropy_personalisation_hash: jitter hashing succeeded\r\n");

    /* ---------- Step 3: SRAM entropy buffer hash (with retries) ---------- */
    log_printf("entropy_personalisation_hash: starting SRAM entropy hashing\r\n");
    rv = ENTROPY_HASH_ERR_HASH_FAILED;
    for (attempt = 0; attempt < ENTROPY_CREATION_RETRY_COUNT; ++attempt) {
        rv = compute_sram_entropy_hashed_global();
        if (rv == ENTROPY_HASH_SUCCESS) break;
        log_printf("entropy_personalisation_hash: attempt %d sram entropy hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != ENTROPY_HASH_SUCCESS) {
        log_printf("entropy_personalisation_hash: sram entropy hashing ultimately failed.\r\n");
        secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
        return SRAM_HASH_FAILED;
    }
    log_printf("entropy_personalisation_hash: SRAM entropy hashing succeeded\r\n");

    /* ---------- Step 4: Combine all hashed components (XOR + SHA-256) ---------- */
    log_printf("entropy_personalisation_hash: combining hashed components\r\n");
    rv = ENTROPY_HASH_ERR_HASH_FAILED;
    for (attempt = 0; attempt < ENTROPY_CREATION_RETRY_COUNT; ++attempt) {
        rv = compute_combined_entropy_hashed_global();  
        if (rv == ENTROPY_HASH_SUCCESS) break;
        log_printf("entropy_personalisation_hash: attempt %d combined hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != ENTROPY_HASH_SUCCESS) {
        log_printf("entropy_personalisation_hash: combined hashing ultimately failed.\r\n");
        secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
        return COMBINED_HASH_FAILED;
    }

    if (is_buffer_empty(entropy_hashed, ENTROPY_HASH_LEN)) {
        log_printf("entropy_personalisation_hash: final entropy is all zeros (catastrophic)\r\n");
        secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
        return ENTROPY_HASH_ERR_COMBINE_FAILED;
    }
    log_printf("entropy_personalisation_hash: combined entropy ready\r\n");

    /* ---------- Step 5: Device UID hash (personalisation) ---------- */
    log_printf("entropy_personalisation_hash: computing device UID hash (personalisation)\r\n");
    rv = DEVICE_UID_HASH_ERR_HASH_FAILED;
    for (attempt = 0; attempt < ENTROPY_CREATION_RETRY_COUNT; ++attempt) {
        rv = compute_device_uid_hashed_global();
        if (rv == DEVICE_UID_HASH_SUCCESS) break;
        log_printf("entropy_personalisation_hash: attempt %d device UID hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != DEVICE_UID_HASH_SUCCESS) {
        log_printf("entropy_personalisation_hash: device UID hashing ultimately failed.\r\n");
        secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
        return DEVICE_UID_HASH_FAILED;
    }
    log_printf("entropy_personalisation_hash: device UID hashed successfully\r\n");

    log_printf("entropy_personalisation_hash: success\r\n");
    return ENTROPY_CREATION_SUCCESS;
}
/* ========== Combine All Hashed Components ========== */

/* Combines the conditioned hardware entropy inputs. */
int combine_all_hashed_entropy(uint8_t *out_hash, size_t out_len)
{
    /* XOR the ADC, jitter and SRAM digests before hashing the combined buffer. */
    cmox_hash_retval_t retval;
    size_t out_len_local;
    uint8_t combined[ENTROPY_HASH_LEN];

    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    memset(combined, 0, ENTROPY_HASH_LEN);

    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        combined[i] ^= buf_pc4_hashed[i];
        combined[i] ^= buf_pc5_hashed[i];
        combined[i] ^= buf_pb0_hashed[i];
        combined[i] ^= jitter_accum_hashed[i];
        combined[i] ^= sram_entropy_buffer_hashed[i];
    }

    if (is_buffer_empty(combined, ENTROPY_HASH_LEN)) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_COMBINE_FAILED;
    }

    out_len_local = out_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               combined, ENTROPY_HASH_LEN,
                               out_hash, out_len,
                               &out_len_local);

    memset(combined, 0, ENTROPY_HASH_LEN);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_HASH_FAILED;
    }

    return ENTROPY_HASH_SUCCESS;
}

/* Updates the combined hardware entropy digest. */
int compute_combined_entropy_hashed_global(void)
{
    /* Store the combined hash in the shared entropy buffer. */
    return combine_all_hashed_entropy(entropy_hashed, ENTROPY_HASH_LEN);
}

/* ========== Accessor Functions ========== */

/* Returns the combined entropy digest buffer. */
const uint8_t* get_entropy_hashed(void)
{
    /* Expose the existing digest without recomputing its inputs. */
    return entropy_hashed;
}

/* Returns the combined digest size. */
size_t get_entropy_hashed_size(void)
{
    /* Use the configured entropy hash length. */
    return ENTROPY_HASH_LEN;
}

/* ========== Debug/Print Functions ========== */

/* Logs the combined entropy digest. */
void print_entropy_hashed(void)
{
	/* Format each digest byte for diagnostic output. */
	log_printf("entropy_hashed (32 bytes):\r\n");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        if (i % 16 == 0) log_printf("  ");
        log_printf("%02X ", entropy_hashed[i]);
        if ((i + 1) % 16 == 0) log_printf("\r\n");
    }
    if (ENTROPY_HASH_LEN % 16 != 0) log_printf("\r\n");
}

/* Logs the individual conditioned entropy components. */
void print_all_hashed_components(void)
{
    /* Print the component digests separately for collection diagnostics. */
    log_printf("\r\n========== HASHED COMPONENTS ==========\r\n");

    log_printf("\r\n1. buf_pc4_hashed:\r\n  ");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        log_printf("%02X ", buf_pc4_hashed[i]);
        if ((i + 1) % 16 == 0 && i < ENTROPY_HASH_LEN - 1) log_printf("\r\n  ");
    }

    log_printf("\r\n\r\n2. buf_pc5_hashed:\r\n  ");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        log_printf("%02X ", buf_pc5_hashed[i]);
        if ((i + 1) % 16 == 0 && i < ENTROPY_HASH_LEN - 1) log_printf("\r\n  ");
    }

    log_printf("\r\n\r\n3. buf_pb0_hashed:\r\n  ");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        log_printf("%02X ", buf_pb0_hashed[i]);
        if ((i + 1) % 16 == 0 && i < ENTROPY_HASH_LEN - 1) log_printf("\r\n  ");
    }

    log_printf("\r\n\r\n4. jitter_accum_hashed:\r\n  ");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        log_printf("%02X ", jitter_accum_hashed[i]);
        if ((i + 1) % 16 == 0 && i < ENTROPY_HASH_LEN - 1) log_printf("\r\n  ");
    }

    log_printf("\r\n\r\n5. sram_entropy_buffer_hashed:\r\n  ");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        log_printf("%02X ", sram_entropy_buffer_hashed[i]);
        if ((i + 1) % 16 == 0 && i < ENTROPY_HASH_LEN - 1) log_printf("\r\n  ");
    }

    log_printf("\r\n\r\n========== FINAL ENTROPY ==========\r\n");
    log_printf("\r\nentropy_hashed (XOR + Hash of all above):\r\n  ");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        log_printf("%02X ", entropy_hashed[i]);
        if ((i + 1) % 16 == 0 && i < ENTROPY_HASH_LEN - 1) log_printf("\r\n  ");
    }
    log_printf("\r\n\r\n");
}

/* Collects the hardware inputs used for crypto initialization. */
int entropy_personalisation_HW_init(void)
{
    /* Propagate a source-specific failure if ADC, jitter, SRAM or UID collection fails. */
    int rc;

    /* ================= ADC + DMA ================= */
    rc = ADC_DMA_INIT();
    if (rc == ADC_DMA_INIT_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: ADC_DMA_INIT succeeded.\r\n");
    } else {
        log_printf("entropy_personalisation_HW_init: ADC_DMA_INIT failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_ADC_FAILED;
    }

    /* ================= Jitter Capture ================= */
        rc = JITTER_CAPTURE_INIT();
        if (rc == JITTER_OK) {
            log_printf("entropy_personalisation_HW_init: JITTER_CAPTURE_INIT succeeded.\r\n");
        } else {
            log_printf("entropy_personalisation_HW_init: JITTER_CAPTURE_INIT failed (rc=%d)\r\n", rc);
            return ENTROPY_HW_INIT_JITTER_FAILED;
        }

    /* ================= SRAM Runtime Sampling ================= */
    rc = sram_sample_runtime();
    if (rc == SRAM_SAMPLE_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: sram_sample_runtime_safe succeeded.\r\n");
    } else if (rc == SRAM_SAMPLE_ALREADY_DONE) {
        log_printf("entropy_personalisation_HW_init: SRAM already sampled earlier (rc=%d)\r\n", rc);
    } else {
        log_printf("entropy_personalisation_HW_init: sram_sample_runtime_safe failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_SRAM_FAILED;
    }

    /* ================= Device UID Read ================= */
    rc = device_uid_read();
    if (rc == DEVICE_UID_READ_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: device UID read successfully.\r\n");
    } else {
        log_printf("entropy_personalisation_HW_init: device UID read failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_UID_FAILED;
    }

    return ENTROPY_HW_INIT_SUCCESS;
}

/* Generates and hashes a nonce with a bounded retry count. */
int nonce_generation(void)
{
    /* Copy the digest prefix into the nonce input and clear the temporary hash. */
    cmox_hash_retval_t retval;
    size_t out_len_local;
    int attempt;

    /* local hash buffer (SHA-256 -> 32 bytes) */
    uint8_t hash_buf[32];

    for (attempt = 0; attempt < NONCE_GEN_RETRY_COUNT; ++attempt) {
        nonce_increment_message_counter();

        uint8_t *nonce_ = nonce_simple_generate();

        if (nonce_ == NULL) {
            log_printf("NONCE_GENERATION_INIT: attempt %d - nonce_simple_generate returned NULL\r\n",
                       attempt + 1);
            continue;
        }

        /* --- HASH the 16-byte nonce using SHA-256 --- */
        out_len_local = sizeof(hash_buf);
        retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                                   nonce_, (size_t)NONCE_SIZE_BYTES,
                                   hash_buf, sizeof(hash_buf),
                                   &out_len_local);

        if (retval != CMOX_HASH_SUCCESS || out_len_local != sizeof(hash_buf)) {
            log_printf("NONCE_GENERATION_INIT: SHA-256 on nonce failed (retval=%d out_len=%lu)\r\n",
                       (int)retval, (unsigned long)out_len_local);
            secure_zero(hash_buf, sizeof(hash_buf));
            continue;
        }

        memcpy(nonce_hashed, hash_buf, 16);

        secure_zero(hash_buf, sizeof(hash_buf));

        for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
            /*log_printf("%d ", nonce_[i]);*/
        }
        /*log_printf("\r\n");*/

        for (volatile int i = 0; i < 100000; i++) {
            __NOP();
        }

        return NONCE_GEN_SUCCESS;
    }

    log_printf("NONCE_GENERATION_INIT: nonce generation ultimately failed after %d attempts\r\n",
               NONCE_GEN_RETRY_COUNT);
    return NONCE_GEN_FAILED;
}
