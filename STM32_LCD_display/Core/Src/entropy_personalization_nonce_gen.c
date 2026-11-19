/**
 * @file entropy_gen.c
 * @brief Entropy hashing, UID personalisation and nonce preparation.
 */
#include <entropy_personalization_nonce_gen.h>

/* Hashed STM32 UID used for personalisation. */
uint8_t device_uid_buffer_hashed[DEVICE_UID_HASH_LEN];
extern volatile uint8_t uid_read;
extern uint8_t device_uid_buffer[UID_SIZE_BYTES];
extern volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE];

/* Entropy hashes used by crypto init. */

/* PC4 entropy hash. */
uint8_t buf_pc4_hashed[ENTROPY_HASH_LEN] = {0};

/* PC5 entropy hash. */
uint8_t buf_pc5_hashed[ENTROPY_HASH_LEN] = {0};

/* PB0 entropy hash. */
uint8_t buf_pb0_hashed[ENTROPY_HASH_LEN] = {0};

/* Timer-jitter hash. */
uint8_t jitter_accum_hashed[ENTROPY_HASH_LEN] = {0};

/* SRAM entropy hash. */
uint8_t sram_entropy_buffer_hashed[ENTROPY_HASH_LEN] = {0};

/* Truncated nonce hash. */
uint8_t nonce_hashed[NONCE_HASH_LEN] = {0};

/* Final combined entropy hash. */
uint8_t entropy_hashed[ENTROPY_HASH_LEN] = {0};

/* Device UID */

/**
 * @brief Hash the STM32 UID with SHA-256.
 *
 * @param out32 UID hash output.
 * @param out32_len Output size.
 * @return UID hash status.
 */
int hash_device_uid_into_hashed_buf(uint8_t *out32, size_t out32_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    if (out32 == NULL) return DEVICE_UID_HASH_ERR_BAD_INPUT;
    if (out32_len != DEVICE_UID_HASH_LEN) return DEVICE_UID_HASH_ERR_BAD_INPUT;
    /* Read UID if needed. */
    if (!uid_read) {
        device_uid_read();
    }
    /* Hash UID to fixed 32-byte value. */
    out_len_local = out32_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               device_uid_buffer, sizeof(device_uid_buffer),
                               out32, out32_len,
                               &out_len_local);
    if (retval != CMOX_HASH_SUCCESS || out_len_local != out32_len) {
        /* Clear partial hash on failure. */
        memset(out32, 0, out32_len);
        return DEVICE_UID_HASH_ERR_HASH_FAILED;
    }
    return DEVICE_UID_HASH_SUCCESS;
}

/**
 * @brief Update the global UID hash.
 *
 * @return UID hash status.
 */
int compute_device_uid_hashed_global(void)
{
    return hash_device_uid_into_hashed_buf(device_uid_buffer_hashed, DEVICE_UID_HASH_LEN);
}

/* Helpers */

/* Check if buffer is NULL or all zero. */
static int is_buffer_empty(const uint8_t *buf, size_t len)
{
    if (buf == NULL) return 1;  /* NULL is treated as empty. */
    for (size_t i = 0; i < len; i++) {
        if (buf[i] != 0) return 0;  /* Found data. */
    }
    return 1;  /* Buffer is empty. */
}

/* ADC entropy */

/**
 * @brief Hash PC4 ADC entropy data.
 *
 * @param in_buffer ADC input buffer.
 * @param out_hash Hash output.
 * @param out_len Output size.
 * @return Entropy hash status.
 */
int hash_adc_pc4(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    /* Check input buffers. */
    if (in_buffer == NULL || out_hash == NULL) {
        log_printf("hash_adc_pc4: NULL input pointer\r\n");
        return ENTROPY_HASH_ERR_NULL_INPUT;
    }
    if (out_len != ENTROPY_HASH_LEN) {
        log_printf("hash_adc_pc4: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, ENTROPY_HASH_LEN);
        return ENTROPY_HASH_ERR_BAD_LENGTH;
    }
    /* Reject empty PC4 sample. */
    if (is_buffer_empty(in_buffer, BUF_SZ)) {
        log_printf("hash_adc_pc4: input buffer is empty (all zeros) – aborting\r\n");
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_BUFFER_EMPTY;
    }
    /* Hash source data. */
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

/* Update PC4 entropy hash. */
int compute_adc_pc4_hashed_global(void)
{
    return hash_adc_pc4(buf_pc4, buf_pc4_hashed, ADC_HASH_LEN);
}

/**
 * @brief Hash PC5 ADC entropy data.
 *
 * @param in_buffer ADC input buffer.
 * @param out_hash Hash output.
 * @param out_len Output size.
 * @return Entropy hash status.
 */
int hash_adc_pc5(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    /* Check input buffers. */
    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;
    /* Hash source data. */
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

/* Update PC5 entropy hash. */
int compute_adc_pc5_hashed_global(void)
{
    return hash_adc_pc5(buf_pc5, buf_pc5_hashed, ADC_HASH_LEN);
}

/**
 * @brief Hash PB0 ADC entropy data.
 *
 * @param in_buffer ADC input buffer.
 * @param out_hash Hash output.
 * @param out_len Output size.
 * @return Entropy hash status.
 */
int hash_adc_pb0(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    /* Check input buffers. */
    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;
    /* Hash source data. */
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

/* Update PB0 entropy hash. */
int compute_adc_pb0_hashed_global(void)
{
    return hash_adc_pb0(buf_pb0, buf_pb0_hashed, ADC_HASH_LEN);
}

/**
 * @brief Hash all ADC entropy sources.
 *
 * @return ADC hash status.
 */
int compute_all_adc_hashes(void)
{
    int rv;
    int attempt;
    /* Retry PC4 hash. */
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
    /* Retry PC5 hash. */
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
    /* Retry PB0 hash. */
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

/* Timer jitter */

/**
 * @brief Hash timer-jitter samples.
 *
 * @param in_jitter Jitter sample buffer.
 * @param out_hash Hash output.
 * @param out_len Output size.
 * @return Entropy hash status.
 */
int hash_jitter_accum(const uint32_t *in_jitter, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    /* Check input buffers. */
    if (in_jitter == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;
    /* Hash jitter samples as bytes. */
    const uint8_t *jitter_bytes = (const uint8_t *)in_jitter;
    size_t jitter_bytes_len = JITTER_CAPTURE_SIZE * sizeof(uint32_t);
    /* Hash source data. */
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

/* Update timer-jitter hash. */
int compute_jitter_accum_hashed_global(void)
{
    return hash_jitter_accum(jitter_differences, jitter_accum_hashed, JITTER_HASHED);
}

/* SRAM entropy */

/**
 * @brief Hash SRAM entropy data.
 *
 * @param in_buffer SRAM input buffer.
 * @param out_hash Hash output.
 * @param out_len Output size.
 * @return Entropy hash status.
 */
int hash_sram_entropy(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    /* Check input buffers. */
    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;
    /* Hash source data. */
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

/* Update SRAM entropy hash. */
int compute_sram_entropy_hashed_global(void)
{
    return hash_sram_entropy((const uint8_t *)sram_entropy_buffer,
                             sram_entropy_buffer_hashed,
                             SRAM_ENTROPY_HASH_LEN);
}

/* Entropy pipeline */

/**
 * @brief Run entropy hashing and UID personalisation.
 *
 * @return Entropy creation status.
 */
int entropy_personalisation_hash(void)
{
    int rv;
    int attempt;
    /* Clear old entropy first. */
    secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
    /* ADC sources. */
    log_printf("entropy_personalisation_hash: starting ADC hashes\r\n");
    rv = compute_all_adc_hashes();
    if (rv != ADC_HASH_ALL_SUCCESS) {
        log_printf("entropy_personalisation_hash: compute_all_adc_hashes failed (rv=%d)\r\n", rv);
        return rv; /* Keep the source-specific status so the log shows which ADC stage failed. */
    }
    log_printf("entropy_personalisation_hash: ADC hashes succeeded\r\n");
    /* Timer jitter. */
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
    /* SRAM source. */
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
    /* Combine source hashes. */
    log_printf("entropy_personalisation_hash: combining hashed components\r\n");
    rv = ENTROPY_HASH_ERR_HASH_FAILED;
    for (attempt = 0; attempt < ENTROPY_CREATION_RETRY_COUNT; ++attempt) {
        rv = compute_combined_entropy_hashed_global(); /* writes entropy_hashed */
        if (rv == ENTROPY_HASH_SUCCESS) break;
        log_printf("entropy_personalisation_hash: attempt %d combined hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != ENTROPY_HASH_SUCCESS) {
        log_printf("entropy_personalisation_hash: combined hashing ultimately failed.\r\n");
        secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
        return COMBINED_HASH_FAILED;
    }
    /* Reject empty final entropy. */
    if (is_buffer_empty(entropy_hashed, ENTROPY_HASH_LEN)) {
        log_printf("entropy_personalisation_hash: final entropy is all zeros (catastrophic)\r\n");
        secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
        return ENTROPY_HASH_ERR_COMBINE_FAILED;
    }
    log_printf("entropy_personalisation_hash: combined entropy ready\r\n");
    /* UID personalisation. */
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
    /* Entropy path complete. */
    log_printf("entropy_personalisation_hash: success\r\n");
    return ENTROPY_CREATION_SUCCESS;
}

/* Final combine */

/**
 * @brief Combine the prepared entropy hashes.
 *
 * @param out_hash Final entropy hash.
 * @param out_len Output size.
 * @return Entropy hash status.
 */
int combine_all_hashed_entropy(uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    uint8_t combined[ENTROPY_HASH_LEN];
    /* Check input buffers. */
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;
    /* XOR the five source hashes. */
    memset(combined, 0, ENTROPY_HASH_LEN);
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        combined[i] ^= buf_pc4_hashed[i];
        combined[i] ^= buf_pc5_hashed[i];
        combined[i] ^= buf_pb0_hashed[i];
        combined[i] ^= jitter_accum_hashed[i];
        combined[i] ^= sram_entropy_buffer_hashed[i];
    }
    /* Reject empty mixed value. */
    if (is_buffer_empty(combined, ENTROPY_HASH_LEN)) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_COMBINE_FAILED;
    }
    /* Final SHA-256 pass. */
    out_len_local = out_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               combined, ENTROPY_HASH_LEN,
                               out_hash, out_len,
                               &out_len_local);
    /* Clear temporary mix. */
    memset(combined, 0, ENTROPY_HASH_LEN);
    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_HASH_FAILED;
    }
    return ENTROPY_HASH_SUCCESS;
}

/* Update final combined entropy hash. */
int compute_combined_entropy_hashed_global(void)
{
    return combine_all_hashed_entropy(entropy_hashed, ENTROPY_HASH_LEN);
}

/* Accessors */

/**
 * @brief Get the final entropy hash.
 *
 * @return Pointer to entropy hash.
 */
const uint8_t* get_entropy_hashed(void)
{
    return entropy_hashed;
}

/**
 * @brief Get final entropy hash size.
 *
 * @return Entropy hash size in bytes.
 */
size_t get_entropy_hashed_size(void)
{
    return ENTROPY_HASH_LEN;
}

/* Debug */

/* Print final entropy hash for debug. */
void print_entropy_hashed(void)
{
    log_printf("entropy_hashed (32 bytes):\r\n");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        if (i % 16 == 0) log_printf("  ");
        log_printf("%02X ", entropy_hashed[i]);
        if ((i + 1) % 16 == 0) log_printf("\r\n");
    }
    if (ENTROPY_HASH_LEN % 16 != 0) log_printf("\r\n");
}

/* Print all entropy hashes for debug. */
void print_all_hashed_components(void)
{
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

/* Hardware entropy input */

/**
 * @brief Collect hardware data used by the entropy path.
 *
 * @return Hardware entropy init status.
 */
int entropy_personalisation_HW_init(void)
{
    int rc;
    /* ADC/DMA entropy input. */
    rc = ADC_DMA_INIT();
    if (rc == ADC_DMA_INIT_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: ADC_DMA_INIT succeeded.\r\n");
    } else {
        log_printf("entropy_personalisation_HW_init: ADC_DMA_INIT failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_ADC_FAILED;
    }
    /* Timer-jitter input. */
    rc = JITTER_CAPTURE_INIT();
    if (rc == JITTER_OK) {
        log_printf("entropy_personalisation_HW_init: JITTER_CAPTURE_INIT succeeded.\r\n");
    } else {
        log_printf("entropy_personalisation_HW_init: JITTER_CAPTURE_INIT failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_JITTER_FAILED;
    }
    /* SRAM input. */
    rc = sram_sample_runtime();
    if (rc == SRAM_SAMPLE_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: sram_sample_runtime_safe succeeded.\r\n");
    } else if (rc == SRAM_SAMPLE_ALREADY_DONE) {
        log_printf("entropy_personalisation_HW_init: SRAM already sampled earlier (rc=%d)\r\n", rc);
    } else {
        log_printf("entropy_personalisation_HW_init: sram_sample_runtime_safe failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_SRAM_FAILED;
    }
    /* Device UID input. */
    rc = device_uid_read();
    if (rc == DEVICE_UID_READ_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: device UID read successfully.\r\n");
    } else {
        log_printf("entropy_personalisation_HW_init: device UID read failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_UID_FAILED;
    }
    return ENTROPY_HW_INIT_SUCCESS;
}

/* Nonce */

/**
 * @brief Generate and hash the next project nonce.
 *
 * @return Nonce generation status.
 */
int nonce_generation(void)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    int attempt;
    /* Full nonce hash before truncation. */
    uint8_t hash_buf[32];
    for (attempt = 0; attempt < NONCE_GEN_RETRY_COUNT; ++attempt) {
        /* Advance message counter first. */
        nonce_increment_message_counter();
        /* Generate raw nonce. */
        uint8_t *nonce_ = nonce_simple_generate();
        if (nonce_ == NULL) {
            log_printf("NONCE_GENERATION_INIT: attempt %d - nonce_simple_generate returned NULL\r\n",
                       attempt + 1);
            continue;
        }
        /* Hash raw nonce. */
        out_len_local = sizeof(hash_buf);
        retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                                   nonce_, (size_t)NONCE_SIZE_BYTES,
                                   hash_buf, sizeof(hash_buf),
                                   &out_len_local);
        if (retval != CMOX_HASH_SUCCESS || out_len_local != sizeof(hash_buf)) {
            log_printf("NONCE_GENERATION_INIT: SHA-256 on nonce failed (retval=%d out_len=%lu)\r\n",
                       (int)retval, (unsigned long)out_len_local);
            /* Clear temporary hash. */
            secure_zero(hash_buf, sizeof(hash_buf));
            continue;
        }
        /* Keep first 16 bytes. */
        memcpy(nonce_hashed, hash_buf, 16);
        /* Clear full hash. */
        secure_zero(hash_buf, sizeof(hash_buf));
        /* Raw nonce debug disabled. */
        for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
            /*log_printf("%d ", nonce_[i]);*/
        }
        /*log_printf("\r\n");*/
        /* Short delay before return. */
        for (volatile int i = 0; i < 100000; i++) {
            __NOP();
        }
        return NONCE_GEN_SUCCESS;
    }
    log_printf("NONCE_GENERATION_INIT: nonce generation ultimately failed after %d attempts\r\n",
               NONCE_GEN_RETRY_COUNT);
    return NONCE_GEN_FAILED;
}
