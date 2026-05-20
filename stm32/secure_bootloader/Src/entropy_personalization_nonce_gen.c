/**
 * @file entropy_gen.c
 * @brief Entropy collection, SHA-256 conditioning and nonce personalization.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include <entropy_personalization_nonce_gen.h>
/* Conditioned entropy and personalization buffers shared with the crypto path. */
uint8_t device_uid_buffer_hashed[DEVICE_UID_HASH_LEN];
extern volatile uint8_t uid_read;
extern uint8_t device_uid_buffer[UID_SIZE_BYTES];
extern volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE];
//jitter_status_t rc;
/* SHA-256 outputs for each hardware entropy component. */
uint8_t buf_pc4_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t buf_pc5_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t buf_pb0_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t jitter_accum_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t sram_entropy_buffer_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t nonce_hashed[NONCE_HASH_LEN] = {0};
uint8_t entropy_hashed[ENTROPY_HASH_LEN] = {0};
/* Device-identity personalization. */
/**
 * @brief Hash the STM32 device UID into a fixed-size SHA-256 buffer.
 * @param out32 Destination buffer for the UID digest.
 * @param out32_len Destination buffer size.
 * @return Device UID hash status.
 */
int hash_device_uid_into_hashed_buf(uint8_t *out32, size_t out32_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    if (out32 == NULL) return DEVICE_UID_HASH_ERR_BAD_INPUT;
    if (out32_len != DEVICE_UID_HASH_LEN) return DEVICE_UID_HASH_ERR_BAD_INPUT;
    if (!uid_read) {
        device_uid_read();
    }
    out_len_local = out32_len;
    /* Condition the source bytes with SHA-256. */
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
/**
 * @brief Hash the device UID into the global personalization buffer.
 * @return Device UID hash status.
 */
int compute_device_uid_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_device_uid_into_hashed_buf(device_uid_buffer_hashed, DEVICE_UID_HASH_LEN);
}
/* Treat NULL or an all-zero source buffer as empty. */
static int is_buffer_empty(const uint8_t *buf, size_t len)
{
    /* Keep lengths within the allocated buffer capacity. */
    if (buf == NULL) return 1;
    for (size_t i = 0; i < len; i++) {
        if (buf[i] != 0) return 0;
    }
    return 1;
}
/* ADC entropy conditioning. */
/**
 * @brief Hash the captured PC4 ADC entropy buffer with SHA-256.
 * @param in_buffer PC4 entropy sample buffer.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @return Entropy hash status.
 */
int hash_adc_pc4(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    /* Check the digest result before using it in verification. */
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
/**
 * @brief Hash the global PC4 ADC buffer into its global digest.
 * @return Entropy hash status.
 */
int compute_adc_pc4_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_adc_pc4(buf_pc4, buf_pc4_hashed, ADC_HASH_LEN);
}
/**
 * @brief Hash the captured PC5 ADC entropy buffer with SHA-256.
 * @param in_buffer PC5 entropy sample buffer.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @return Entropy hash status.
 */
int hash_adc_pc5(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    /* Check the digest result before using it in verification. */
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
/**
 * @brief Hash the global PC5 ADC buffer into its global digest.
 * @return Entropy hash status.
 */
int compute_adc_pc5_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_adc_pc5(buf_pc5, buf_pc5_hashed, ADC_HASH_LEN);
}
/**
 * @brief Hash the captured PB0 ADC entropy buffer with SHA-256.
 * @param in_buffer PB0 entropy sample buffer.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @return Entropy hash status.
 */
int hash_adc_pb0(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    /* Check the digest result before using it in verification. */
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
/**
 * @brief Hash the global PB0 ADC buffer into its global digest.
 * @return Entropy hash status.
 */
int compute_adc_pb0_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_adc_pb0(buf_pb0, buf_pb0_hashed, ADC_HASH_LEN);
}
/**
 * @brief Condition all three ADC entropy buffers with bounded retries.
 * @return Combined ADC hashing status.
 */
int compute_all_adc_hashes(void)
{
    /* Check the digest result before using it in verification. */
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
/* Timer-jitter entropy conditioning. */
/**
 * @brief Hash the timer-jitter capture buffer with SHA-256.
 * @param in_jitter Captured jitter values.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @return Entropy hash status.
 */
int hash_jitter_accum(const uint32_t *in_jitter, uint8_t *out_hash, size_t out_len)
{
    /* Check the digest result before using it in verification. */
    cmox_hash_retval_t retval;
    size_t out_len_local;
    if (in_jitter == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;
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
/**
 * @brief Hash the global timer-jitter capture into its global digest.
 * @return Entropy hash status.
 */
int compute_jitter_accum_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_jitter_accum(jitter_differences, jitter_accum_hashed, JITTER_HASHED);
}
/* SRAM entropy conditioning. */
/**
 * @brief Hash the SRAM startup/runtime entropy buffer with SHA-256.
 * @param in_buffer SRAM entropy buffer.
 * @param out_hash SHA-256 output buffer.
 * @param out_len Output buffer size.
 * @return Entropy hash status.
 */
int hash_sram_entropy(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    /* Check entropy acquisition before seeding the generator. */
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
/**
 * @brief Hash the global SRAM entropy buffer into its global digest.
 * @return Entropy hash status.
 */
int compute_sram_entropy_hashed_global(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return hash_sram_entropy((const uint8_t *)sram_entropy_buffer,
                             sram_entropy_buffer_hashed,
                             SRAM_ENTROPY_HASH_LEN);
}
/* Full entropy-conditioning pipeline. */
/**
 * @brief Run the complete entropy-conditioning and device-personalization pipeline.
 * @return Entropy creation status.
 */
int entropy_personalisation_hash(void)
{
    int rv;
    int attempt;
    secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
    log_printf("entropy_personalisation_hash: starting ADC hashes\r\n");
    /* ADC sources are conditioned before combining with jitter and SRAM. */
    rv = compute_all_adc_hashes();
    if (rv != ADC_HASH_ALL_SUCCESS) {
        log_printf("entropy_personalisation_hash: compute_all_adc_hashes failed (rv=%d)\r\n", rv);
        return rv;
    }
    log_printf("entropy_personalisation_hash: ADC hashes succeeded\r\n");
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
/* Final entropy combination and conditioning. */
/**
 * @brief Combine the conditioned hardware entropy components and hash the result.
 * @param out_hash Final conditioned entropy output.
 * @param out_len Output buffer size.
 * @return Entropy hash status.
 */
int combine_all_hashed_entropy(uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    uint8_t combined[ENTROPY_HASH_LEN];
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;
    /* XOR the independent component digests before the final SHA-256 pass. */
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
/**
 * @brief Build the final conditioned entropy value in the global entropy buffer.
 * @return Entropy hash status.
 */
int compute_combined_entropy_hashed_global(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return combine_all_hashed_entropy(entropy_hashed, ENTROPY_HASH_LEN);
}
/* Entropy accessors and diagnostic output. */
/**
 * @brief Get the final conditioned entropy buffer.
 * @return Pointer to the global entropy digest.
 */
const uint8_t* get_entropy_hashed(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return entropy_hashed;
}
/**
 * @brief Get the size of the final conditioned entropy buffer.
 * @return Entropy digest size in bytes.
 */
size_t get_entropy_hashed_size(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return ENTROPY_HASH_LEN;
}
/**
 * @brief Print the final conditioned entropy value for diagnostics.
 */
void print_entropy_hashed(void)
{
    /* Check entropy acquisition before seeding the generator. */
	log_printf("entropy_hashed (32 bytes):\r\n");
    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        if (i % 16 == 0) log_printf("  ");
        log_printf("%02X ", entropy_hashed[i]);
        if ((i + 1) % 16 == 0) log_printf("\r\n");
    }
    if (ENTROPY_HASH_LEN % 16 != 0) log_printf("\r\n");
}
/**
 * @brief Print the intermediate conditioned entropy components and final digest.
 */
/**

 * Print all intermediate hashes using log_printf

 */
void print_all_hashed_components(void)
{
    /* Check the digest result before using it in verification. */
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
/* Hardware entropy-source initialization. */
/**
 * @brief Initialize the ADC, timer-jitter, SRAM and UID entropy sources.
 * @return Entropy hardware initialization status.
 */
int entropy_personalisation_HW_init(void)
{
    int rc;
    /* Collect the hardware sources before the hashing stage runs. */
    rc = ADC_DMA_INIT();
    if (rc == ADC_DMA_INIT_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: ADC_DMA_INIT succeeded.\r\n");
    } else {
        log_printf("entropy_personalisation_HW_init: ADC_DMA_INIT failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_ADC_FAILED;
    }
        rc = JITTER_CAPTURE_INIT();
        if (rc == JITTER_OK) {
            log_printf("entropy_personalisation_HW_init: JITTER_CAPTURE_INIT succeeded.\r\n");
        } else {
            log_printf("entropy_personalisation_HW_init: JITTER_CAPTURE_INIT failed (rc=%d)\r\n", rc);
            return ENTROPY_HW_INIT_JITTER_FAILED;
        }
    rc = sram_sample_runtime();
    if (rc == SRAM_SAMPLE_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: sram_sample_runtime_safe succeeded.\r\n");
    } else if (rc == SRAM_SAMPLE_ALREADY_DONE) {
        log_printf("entropy_personalisation_HW_init: SRAM already sampled earlier (rc=%d)\r\n", rc);
    } else {
        log_printf("entropy_personalisation_HW_init: sram_sample_runtime_safe failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_SRAM_FAILED;
    }
    rc = device_uid_read();
    if (rc == DEVICE_UID_READ_SUCCESS) {
        log_printf("entropy_personalisation_HW_init: device UID read successfully.\r\n");
    } else {
        log_printf("entropy_personalisation_HW_init: device UID read failed (rc=%d)\r\n", rc);
        return ENTROPY_HW_INIT_UID_FAILED;
    }
    return ENTROPY_HW_INIT_SUCCESS;
}
/**
 * @brief Generate a nonce, condition it with SHA-256 and update nonce_hashed.
 * @return Nonce generation status.
 */
/*

 * NONCE_GENERATION_INIT

 *

 * Increments the message counter, generates a nonce via nonce_simple_generate(),

 * prints the nonce bytes via log_printf and performs a small delay.

 *

 * Returns:

 *   NONCE_GEN_SUCCESS on success (0)

 *   NONCE_GEN_NULL_NONCE if nonce_simple_generate() returned NULL

 *   NONCE_GEN_FAILED on repeated failure

 */
/* Nonce generation and personalization. */
int nonce_generation(void)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    int attempt;
    uint8_t hash_buf[32];
    for (attempt = 0; attempt < NONCE_GEN_RETRY_COUNT; ++attempt) {
        /* Advance nonce state before requesting the next nonce. */
        nonce_increment_message_counter();
        uint8_t *nonce_ = nonce_simple_generate();
        if (nonce_ == NULL) {
            log_printf("NONCE_GENERATION_INIT: attempt %d - nonce_simple_generate returned NULL\r\n",
                       attempt + 1);
            continue;
        }
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
        /* Keep the first 16 digest bytes as the nonce personalization value. */
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
