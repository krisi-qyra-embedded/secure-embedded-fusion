/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */


#include <entropy_personalization_nonce_gen.h>

uint8_t device_uid_buffer_hashed[DEVICE_UID_HASH_LEN];
extern volatile uint8_t uid_read;
extern uint8_t device_uid_buffer[UID_SIZE_BYTES];
extern volatile uint8_t sram_entropy_buffer[SRAM_ENTROPY_BUFFER_SIZE];/* ========== Global Hash Buffers (32 bytes each) ========== */
//jitter_status_t rc;
/* Individual hashed components */
uint8_t buf_pc4_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t buf_pc5_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t buf_pb0_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t jitter_accum_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t sram_entropy_buffer_hashed[ENTROPY_HASH_LEN] = {0};
uint8_t nonce_hashed[NONCE_HASH_LEN] = {0};
/* Final combined entropy hash */
uint8_t entropy_hashed[ENTROPY_HASH_LEN] = {0};


/* updated function using the macros */
int hash_device_uid_into_hashed_buf(uint8_t *out32, size_t out32_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;

    if (out32 == NULL) return DEVICE_UID_HASH_ERR_BAD_INPUT;
    if (out32_len != DEVICE_UID_HASH_LEN) return DEVICE_UID_HASH_ERR_BAD_INPUT;

    /* Ensure UID is read */
    if (!uid_read) {
        device_uid_read();
    }

    /* Hash the raw UID bytes (device_uid_buffer) with SHA-256 */
    out_len_local = out32_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               device_uid_buffer, sizeof(device_uid_buffer),
                               out32, out32_len,
                               &out_len_local);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out32_len) {
        /* wipe output on failure */
        memset(out32, 0, out32_len);
        return DEVICE_UID_HASH_ERR_HASH_FAILED;
    }

    return DEVICE_UID_HASH_SUCCESS;
}

/* Convenience wrapper that fills the global device_uid_buffer_hashed */
int compute_device_uid_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_device_uid_into_hashed_buf(device_uid_buffer_hashed, DEVICE_UID_HASH_LEN);
}


/* ========== Helper: Check if buffer is all zeros (defensive check) ========== */
static int is_buffer_empty(const uint8_t *buf, size_t len)
{
    if (buf == NULL) return 1;  /* NULL is \"empty\" */

    for (size_t i = 0; i < len; i++) {
        if (buf[i] != 0) return 0;  /* Found non-zero byte */
    }
    return 1;  /* All zeros */
}

/* ========== ADC PC4 Hashing ========== */

/**
 * Hash ADC PC4 buffer (255 bytes) using SHA-256
 */
int hash_adc_pc4(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;

    /* Validate inputs */
    if (in_buffer == NULL || out_hash == NULL) {
        log_printf("hash_adc_pc4: NULL input pointer\r\n");
        return ENTROPY_HASH_ERR_NULL_INPUT;
    }

    if (out_len != ENTROPY_HASH_LEN) {
        log_printf("hash_adc_pc4: bad output length (got %lu, expected %u)\r\n",
                   (unsigned long)out_len, ENTROPY_HASH_LEN);
        return ENTROPY_HASH_ERR_BAD_LENGTH;
    }

    /* Defensive: reject empty buffer */
    if (is_buffer_empty(in_buffer, BUF_SZ)) {
        log_printf("hash_adc_pc4: input buffer is empty (all zeros) – aborting\r\n");
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_BUFFER_EMPTY;
    }

    /* Compute SHA-256 hash */
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

/* Convenience wrapper that fills the global buf_pc4_hashed from buf_pc4 */
int compute_adc_pc4_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_adc_pc4(buf_pc4, buf_pc4_hashed, ADC_HASH_LEN);
}


/* ========== ADC PC5 Hashing ========== */

/**
 * Hash ADC PC5 buffer (255 bytes) using SHA-256
 */
int hash_adc_pc5(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;

    /* Validate inputs */
    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    /* Compute SHA-256 hash */
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

/* Convenience wrapper that fills the global buf_pc5_hashed from buf_pc5 */
int compute_adc_pc5_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_adc_pc5(buf_pc5, buf_pc5_hashed, ADC_HASH_LEN);
}


/* ========== ADC PB0 Hashing ========== */

/**
 * Hash ADC PB0 buffer (255 bytes) using SHA-256
 */
int hash_adc_pb0(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;

    /* Validate inputs */
    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    /* Compute SHA-256 hash */
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

/* Convenience wrapper that fills the global buf_pb0_hashed from buf_pb0 */
int compute_adc_pb0_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_adc_pb0(buf_pb0, buf_pb0_hashed, ADC_HASH_LEN);
}


/*
 * Compute all ADC hashed buffers (PC4, PC5, PB0).
 * On any failure the function logs an explanatory message and returns
 * a negative macro-specific error code.
 *
 * Returns:
 *   ADC_HASH_ALL_SUCCESS (0) on success
 *   ADC_PC4_HASH_FAILED / ADC_PC5_HASH_FAILED / ADC_PB0_HASH_FAILED on failure
 */


int compute_all_adc_hashes(void)
{
    int rv;
    int attempt;
    /* PC4 */
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

    /* PC5 */
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

    /* PB0 */
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
int hash_jitter_accum(const uint32_t *in_jitter, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;

    /* Validate inputs */
    if (in_jitter == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    /* Cast uint32_t array to uint8_t for hashing
       Total size: 256 values * 4 bytes = 1024 bytes */
    const uint8_t *jitter_bytes = (const uint8_t *)in_jitter;
    size_t jitter_bytes_len = JITTER_CAPTURE_SIZE * sizeof(uint32_t);

    /* Compute SHA-256 hash */
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

/* Convenience wrapper that fills the global jitter_accum_hashed from jitter_accum */
int compute_jitter_accum_hashed_global(void)
{
    /* Check the digest result before using it in verification. */
    return hash_jitter_accum(jitter_differences, jitter_accum_hashed, JITTER_HASHED);
}


/* ========== SRAM Entropy Buffer Hashing ========== */

/**
 * Hash SRAM entropy buffer (256 bytes) using SHA-256
 */
int hash_sram_entropy(const uint8_t *in_buffer, uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;

    /* Validate inputs */
    if (in_buffer == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    /* Compute SHA-256 hash */
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

/* Convenience wrapper that fills the global sram_entropy_buffer_hashed from sram_entropy_buffer */
int compute_sram_entropy_hashed_global(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return hash_sram_entropy((const uint8_t *)sram_entropy_buffer,
                             sram_entropy_buffer_hashed,
                             SRAM_ENTROPY_HASH_LEN);
}


/*
 * entropy_init
 *
 * Runs the full entropy pipeline:
 *  1) compute_all_adc_hashes()
 *  2) compute_jitter_accum_hashed_global()
 *  3) compute_sram_entropy_hashed_global()
 *  4) compute_combined_entropy_hashed_global()
 *
 * Any step failure is logged and a negative, step-specific macro is returned.
 */


int entropy_personalisation_hash(void)
{
    int rv;
    int attempt;

    /* Wipe final output early to avoid partial-state exposure */
    secure_zero(entropy_hashed, ENTROPY_HASH_LEN);

    /* ---------- Step 1: ADC hashes ---------- */
    log_printf("entropy_personalisation_hash: starting ADC hashes\r\n");
    rv = compute_all_adc_hashes();
    if (rv != ADC_HASH_ALL_SUCCESS) {
        log_printf("entropy_personalisation_hash: compute_all_adc_hashes failed (rv=%d)\r\n", rv);
        return rv; /* propagate ADC-specific error */
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
        rv = compute_combined_entropy_hashed_global(); /* writes into entropy_hashed */
        if (rv == ENTROPY_HASH_SUCCESS) break;
        log_printf("entropy_personalisation_hash: attempt %d combined hashing failed (rv=%d)\r\n",
                   attempt + 1, rv);
    }
    if (rv != ENTROPY_HASH_SUCCESS) {
        log_printf("entropy_personalisation_hash: combined hashing ultimately failed.\r\n");
        secure_zero(entropy_hashed, ENTROPY_HASH_LEN);
        return COMBINED_HASH_FAILED;
    }

    /* Defensive check: ensure final entropy isn't all zeros */
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

    /* Success */
    log_printf("entropy_personalisation_hash: success\r\n");
    return ENTROPY_CREATION_SUCCESS;
}
/* ========== Combine All Hashed Components ========== */

/**
 * Combine all 5 hashed components (32 bytes each) into final entropy_hashed
 *
 * Strategy:
 * 1. XOR all 5 hashes together byte-by-byte
 * 2. Hash the XORed result to produce final entropy_hashed
 *
 * This ensures:
 * - All components contribute equally
 * - Final output is uniformly random
 * - Collision resistance from SHA-256
 */

int combine_all_hashed_entropy(uint8_t *out_hash, size_t out_len)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    uint8_t combined[ENTROPY_HASH_LEN];

    /* Validate inputs */
    if (out_hash == NULL) return ENTROPY_HASH_ERR_NULL_INPUT;
    if (out_len != ENTROPY_HASH_LEN) return ENTROPY_HASH_ERR_BAD_LENGTH;

    /* Step 1: XOR all 5 hashed components together */
    memset(combined, 0, ENTROPY_HASH_LEN);

    for (size_t i = 0; i < ENTROPY_HASH_LEN; i++) {
        combined[i] ^= buf_pc4_hashed[i];
        combined[i] ^= buf_pc5_hashed[i];
        combined[i] ^= buf_pb0_hashed[i];
        combined[i] ^= jitter_accum_hashed[i];
        combined[i] ^= sram_entropy_buffer_hashed[i];
    }

    /* Defensive: Check if combined result is all zeros (catastrophic failure) */
    if (is_buffer_empty(combined, ENTROPY_HASH_LEN)) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_COMBINE_FAILED;
    }

    /* Step 2: Hash the XORed combination to produce final entropy */
    out_len_local = out_len;
    retval = cmox_hash_compute(CMOX_SHA256_ALGO,
                               combined, ENTROPY_HASH_LEN,
                               out_hash, out_len,
                               &out_len_local);

    /* Wipe intermediate combined buffer (defensive) */
    memset(combined, 0, ENTROPY_HASH_LEN);

    if (retval != CMOX_HASH_SUCCESS || out_len_local != out_len) {
        memset(out_hash, 0, out_len);
        return ENTROPY_HASH_ERR_HASH_FAILED;
    }

    return ENTROPY_HASH_SUCCESS;
}

/* Convenience wrapper that fills the global sram_entropy_buffer_hashed from combined hashed components */
int compute_combined_entropy_hashed_global(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return combine_all_hashed_entropy(entropy_hashed, ENTROPY_HASH_LEN);
}




/* ========== Accessor Functions ========== */

/**
 * Get pointer to final entropy hash
 */
const uint8_t* get_entropy_hashed(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return entropy_hashed;
}


/**
 * Get entropy hash size
 */
size_t get_entropy_hashed_size(void)
{
    /* Check entropy acquisition before seeding the generator. */
    return ENTROPY_HASH_LEN;
}

/* ========== Debug/Print Functions ========== */

/**
 * Print entropy hash in hex format
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


/* Collects or prepares entropy for cryptographic initialization. */
int entropy_personalisation_HW_init(void)
{
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


int nonce_generation(void)
{
    cmox_hash_retval_t retval;
    size_t out_len_local;
    int attempt;

    /* local hash buffer (SHA-256 -> 32 bytes) */
    uint8_t hash_buf[32];

    for (attempt = 0; attempt < NONCE_GEN_RETRY_COUNT; ++attempt) {
        /* Increment counter */
        nonce_increment_message_counter();

        /* Generate nonce (16 bytes expected) */
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
            /* wipe and continue to next attempt */
            secure_zero(hash_buf, sizeof(hash_buf));
            continue;
        }

        /* Copy first 16 bytes of the 32-byte hash into the global nonce_hashed */
        memcpy(nonce_hashed, hash_buf, 16);

        /* Wipe local hash buffer (defensive) */
        secure_zero(hash_buf, sizeof(hash_buf));

        /* OPTIONAL: print the raw nonce bytes (existing behavior) */
        for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
            /*log_printf("%d ", nonce_[i]);*/
        }
        /*log_printf("\r\n");*/

        /* small settle delay as before */
        for (volatile int i = 0; i < 100000; i++) {
            __NOP();
        }

        return NONCE_GEN_SUCCESS;
    }

    log_printf("NONCE_GENERATION_INIT: nonce generation ultimately failed after %d attempts\r\n",
               NONCE_GEN_RETRY_COUNT);
    return NONCE_GEN_FAILED;
}
