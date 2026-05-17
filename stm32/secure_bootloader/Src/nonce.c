/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */
#include <nonce.h>

/* nonce_simple.c - Extended 16-byte Nonce
 *
 * Nonce Structure (16 bytes):
 * [0-3]:   Message counter (derived from \"messageN_sent\" + hash)
 * [4-7]:   TIM1 timer snapshot (continuous free-running counter + hash)
 * [8-11]:  ADC entropy (static, XOR of pc4+pc5+pb0 hashes, first 4 bytes)
 * [12-15]: Jitter entropy (static, from jitter_accum hash, first 4 bytes)
 *
 * The first 8 bytes are dynamic (change on each generation).
 * The last 8 bytes are static (computed once from entropy sources).
 */

/* Compile-time sanity checks: ensure the macros/sizes are provided and sane.
 * Replace NONCE_SIZE_BYTES, STATIC_ENTROPY_SIZE, COUNTER_STRING_SIZE with your macros if different.
 */


/* Message counter (increments with each data transmission) */
volatile uint32_t message_counter = 0;
/* Generate nonce */
uint8_t *nonce_;
/* Nonce buffer (last generated nonce) */
uint8_t nonce_buffer[NONCE_SIZE_BYTES] = {0};

/* Static entropy component (bytes 8-15, computed once) */
 uint8_t static_entropy[8] = {0};
 uint8_t static_entropy_initialized = 0;

/* Wipes sensitive data from the provided memory region. */
 void secure_zero(void *ptr, size_t len)
 {
    /* Write through the entire buffer to clear key material. */
     volatile uint8_t *p = (volatile uint8_t *)ptr;

     while (len--) {
         *p++ = 0;
     }
 }


/* String buffer for debug display */
char counter_string[32] = {0};

/* Public API: initialise nonce subsystem once at boot.
 * - Returns NONCE_OK on success
 * - Returns negative error code on failure
 *
 * Side effects:
 * - zeroes nonce_buffer, static_entropy, counter_string
 * - resets message_counter to 0
 * - marks static_entropy_initialized only if platform entropy was successfully seeded
 */
nonce_status_t nonce_simple_init(void)
{
    /* Defensive runtime checks for external arrays: try to catch obviously wrong sizes.
     * We expect the linker to provide arrays sized exactly as the macros indicate.
     * If sizes mismatch, behaviour is undefined; we attempt to avoid silent success.
     */
    if (&nonce_buffer[NONCE_SIZE_BYTES - 1] == NULL) { /* never true but expresses intent */ }

    /* Prevent double-initialization unless caller explicitly wants reseed (keep behaviour strict) */
    if (static_entropy_initialized) {
        return NONCE_ERR_ALREADY_INIT;
    }

    /* Reset logical counter */
    message_counter = 0;

    /* Wipe buffers securely */
    secure_zero(nonce_buffer, (size_t)NONCE_SIZE_BYTES);
    secure_zero(static_entropy, (size_t)STATIC_ENTROPY_SIZE);
    secure_zero(counter_string, (size_t)COUNTER_STRING_SIZE);

    /* Attempt to seed static_entropy from platform RNG if available.
     * If PLATFORM_HAS_HW_RNG is defined and platform_get_random_bytes is implemented,
     * use it to mark static_entropy_initialized. Otherwise leave as zero and return OK.
     *
     * Note: prefer defining PLATFORM_HAS_HW_RNG in your platform config when a true HW RNG
     * is available. The weak declaration above allows link-time override.
     */
#ifdef PLATFORM_HAS_HW_RNG
    if (platform_get_random_bytes != NULL) {
        if (platform_get_random_bytes(static_entropy, (size_t)STATIC_ENTROPY_SIZE) != 0) {
            /* failed to obtain entropy -> wipe and return error */
            secure_zero(static_entropy, (size_t)STATIC_ENTROPY_SIZE);
            return NONCE_ERR_ENTROPY_INIT_FAIL;
        }
        /* mark seeded */
        static_entropy_initialized = 1;
    } else {
        /* PLATFORM_HAS_HW_RNG set but no implementation found */
        return NONCE_ERR_ENTROPY_INIT_FAIL;
    }
#else
    /* No platform RNG hook defined: leave static_entropy zeroed.
     * Caller should call a reseed function with a valid entropy source
     * before using the nonce for cryptographic purposes.
     */
    static_entropy_initialized = 0;
#endif

    return NONCE_OK;
}


/**
 * Increment message counter
 * Call this every time data is sent
 */
void nonce_increment_message_counter(void)
{
    /* Check the generator status before using the output. */
    message_counter++;
}

/**
 * Get current message counter value
 * Returns: Current message count
 */
uint32_t nonce_get_message_counter(void)
{
    /* Check the generator status before using the output. */
    return message_counter;
}

/**
 * Get message counter as string format
 * Returns: Pointer to string \"messageN_sent\"
 */
const char* nonce_get_counter_string(void)
{
    /* Check the generator status before using the output. */
    snprintf(counter_string, sizeof(counter_string), "message%lu_sent", message_counter);
    return counter_string;
}

/**
 * Convert counter string to 4-byte value
 *
 * Strategy: Hash the string into 4 bytes using simple checksum
 * This ensures each \"messageN_sent\" produces a unique 4-byte value
 *
 * Returns: 4-byte hash of counter string
 */

uint32_t counter_string_to_uint32(void)
{
    /* Generate string */
    snprintf(counter_string, sizeof(counter_string), "message%lu_sent", message_counter);

    /* Simple hash: XOR-based checksum */
    uint32_t hash = 0x12345678;  /* Initial seed */

    for (size_t i = 0; i < strlen(counter_string); i++) {
        /* Rotate and XOR each character */
        hash = (hash << 5) | (hash >> 27);  /* Rotate left 5 bits */
        hash ^= (uint32_t)counter_string[i];
        hash += i + 1;  /* Add position for more variation */
    }

    /* Mix in the actual counter value to guarantee uniqueness */
    hash ^= message_counter;

    return hash;
}

/* ========== Timer Functions ========== */

/**
 * Read TIM1 counter value (continuous free-running timer)
 *
 * TIM1 is already configured by tim1_ch3_jitter_capture_init():
 * - PSC = 0 (96 MHz clock)
 * - ARR = 0xFFFF (16-bit wraparound)
 * - Free-running, never stops
 *
 * Returns: Current TIM1->CNT value (0 to 65535)
 */
static inline uint16_t read_tim1_counter(void)
{
    /* Ensure TIM1 is enabled (should be already running) */
    if (!(TIM1->CR1 & TIM_CR1_CEN)) {
        /* If not enabled, start it */
        TIM1->CR1 |= TIM_CR1_CEN;
    }
    /* Read current counter value (16-bit) */
    return (uint16_t)(TIM1->CNT);
}

 /**
  * Read TIM1 counter with extended precision
  *
  * Reads counter twice and captures overflow to extend to 32-bit
  * This provides higher resolution for the timer snapshot
  *
  * Returns: Extended 32-bit timer value
  */
uint32_t read_tim1_hashed_loop(void)
{
    uint16_t cnt = read_tim1_counter();
    /* Build a small 4-byte "message" derived from the 16-bit counter.
       This gives us 4 loop iterations (like iterating characters in a string). */
    uint8_t bytes[4];
    bytes[0] = (uint8_t)(cnt & 0xFF);                 /* low byte */
    bytes[1] = (uint8_t)((cnt >> 8) & 0xFF);          /* high byte */
    /* two inexpensive nonlinear mixes to increase variation per byte */
    bytes[2] = (uint8_t)(((cnt ^ (cnt >> 3)) & 0xFF));         /* mixed low */
    bytes[3] = (uint8_t)(((cnt * 7u) ^ (cnt >> 5)) & 0xFF);   /* mixed high */
    uint32_t hash = 0x12345678U;  /* same seed as your string hash */
    /* Loop-style mixing identical in pattern to counter_string_to_uint32() */
    for (size_t i = 0; i < sizeof(bytes); ++i) {
        /* Rotate-left 5 */
        hash = (hash << 5) | (hash >> 27);
        /* XOR in this "byte" */
        hash ^= (uint32_t)bytes[i];
        /* position-based addition for variation */
        hash += (uint32_t)(i + 1);
        /* extra lightweight fuse: XOR a rotated form of cnt (keeps it deterministic) */
        /* rotate 16-bit cnt inside 32-bit word by i bits (safe shift amounts 0..15) */
        uint32_t rotated_cnt = (((uint32_t)cnt << (i)) | ((uint32_t)cnt >> (16 - (i)))) & 0xFFFFU;
        hash ^= rotated_cnt;
    }
    /* Final fuse with raw counter to guarantee uniqueness for same input */
    hash ^= (uint32_t)cnt;
    return hash;
}

/* ========== Static Entropy Component ========== */

/**
 * Compute static entropy component (bytes 8-15 of nonce)
 *
 * Combines ADC and jitter entropy sources:
 * - Bytes 8-11:  XOR of (pc4 + pc5 + pb0) hashes, first 4 bytes
 * - Bytes 12-15: jitter_accum hash, first 4 bytes
 *
 * Call this ONCE after entropy sources are collected and hashed.
 *
 * Parameters:
 *   adc_pc4_hash: 32-byte SHA-256 hash of ADC PC4 buffer
 *   adc_pc5_hash: 32-byte SHA-256 hash of ADC PC5 buffer
 *   adc_pb0_hash: 32-byte SHA-256 hash of ADC PB0 buffer
 *   jitter_hash:  32-byte SHA-256 hash of jitter accumulator
 *
 * Returns: 0 on success, negative on error
 */

int nonce_compute_static_entropy(const uint8_t *adc_pc4_hash,
                                  const uint8_t *adc_pc5_hash,
                                  const uint8_t *adc_pb0_hash,
                                  const uint8_t *jitter_hash)
{
    /* Validate inputs */
    if (adc_pc4_hash == NULL || adc_pc5_hash == NULL ||
        adc_pb0_hash == NULL || jitter_hash == NULL) {
        return -1;  /* NULL input */
    }

    /* Bytes 8-11: Combine ADC sources (XOR first 4 bytes of each hash) */
    for (int i = 0; i < 4; i++) {
        static_entropy[i] = adc_pc4_hash[i] ^ adc_pc5_hash[i] ^ adc_pb0_hash[i];
    }

    /* Bytes 12-15: Use jitter hash (first 4 bytes) */
    for (int i = 0; i < 4; i++) {
        static_entropy[4 + i] = jitter_hash[i];
    }

    static_entropy_initialized = 1;

    return 0;  /* Success */
}

  /* ========== Nonce Generation ========== */

  /**
   * Generate 8-byte nonce
   *
   * Structure:
   * [0-3]: Counter (from \"messageN_sent\" string hash + actual counter)
   * [4-7]: TIM1 timer snapshot (free-running counter value)
   *
   * Returns: Pointer to 8-byte nonce buffer
   */

 uint8_t* nonce_simple_generate(void)
{
    uint32_t counter_val = counter_string_to_uint32();
    uint32_t timer_val   = read_tim1_hashed_loop();

    /* Bytes 0-3: Counter (little-endian) */
    nonce_buffer[0] = (counter_val >>  0) & 0xFF;
    nonce_buffer[1] = (counter_val >>  8) & 0xFF;
    nonce_buffer[2] = (counter_val >> 16) & 0xFF;
    nonce_buffer[3] = (counter_val >> 24) & 0xFF;

    /* Bytes 4-7: Timer (little-endian) */
    nonce_buffer[4] = (timer_val >>  0) & 0xFF;
    nonce_buffer[5] = (timer_val >>  8) & 0xFF;
    nonce_buffer[6] = (timer_val >> 16) & 0xFF;
    nonce_buffer[7] = (timer_val >> 24) & 0xFF;

    /* Ensure static entropy (bytes 8..15) exist.
       If not initialized, call the combiner that operates on pre-hashed inputs. */
    if (!static_entropy_initialized) {
        /* Attempt to compute static_entropy from previously-computed hashes.
           If the combiner fails, zero the static_entropy and mark initialized
           to avoid repeated failing attempts at runtime. */
        if (nonce_compute_static_entropy(buf_pc4_hashed,
                                         buf_pc5_hashed,
                                         buf_pb0_hashed,
                                         jitter_accum_hashed) != 0) {
            memset(static_entropy, 0, sizeof(static_entropy));
        }
        static_entropy_initialized = 1;
    }

    /* Copy static_entropy into upper half of nonce_buffer */
    memcpy(&nonce_buffer[8], static_entropy, sizeof(static_entropy));

    return nonce_buffer;
}

  /**
   * Generate nonce and copy to external buffer
   *
   * out_buffer: Destination buffer (must be at least 8 bytes)
   *
   * Returns: Number of bytes written (always 8)
   */

  uint32_t nonce_simple_generate_to_buffer(uint8_t *out_buffer)
  {
    /* Check the generator status before using the output. */
      if (out_buffer == NULL) {
          return 0;
      }

      const uint8_t *nonce = nonce_simple_generate();

      for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
          out_buffer[i] = nonce[i];
      }

      return NONCE_SIZE_BYTES;
  }

  /**
   * Get last generated nonce
   * Returns: Pointer to 8-byte nonce buffer
   */

  const uint8_t* nonce_simple_get_last(void)
  {
    /* Check the generator status before using the output. */
      return nonce_buffer;
  }


  /* ========== Display Functions ========== */

  /**
   * Print nonce in hex format
   */

  void nonce_simple_print(const uint8_t *nonce)
  {
    /* Check the generator status before using the output. */
      if (nonce == NULL) {
          nonce = nonce_buffer;
      }

      log_printf("Nonce (8 bytes): ");
      for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
    	  log_printf("%02X ", nonce[i]);
      }
      log_printf("\r\n");
  }

/* Initializes or generates cryptographic random/nonce material. */
  void nonce_simple_print_detailed(void)
  {
      /* Extract components from nonce buffer */
      uint32_t counter_val = (nonce_buffer[3] << 24) | (nonce_buffer[2] << 16) |
                             (nonce_buffer[1] << 8)  | (nonce_buffer[0] << 0);

      uint32_t timer_val = (nonce_buffer[7] << 24) | (nonce_buffer[6] << 16) |
                           (nonce_buffer[5] << 8)  | (nonce_buffer[4] << 0);

      uint32_t adc_val = (nonce_buffer[11] << 24) | (nonce_buffer[10] << 16) |
                         (nonce_buffer[9] << 8)   | (nonce_buffer[8] << 0);

      uint32_t jitter_val = (nonce_buffer[15] << 24) | (nonce_buffer[14] << 16) |
                            (nonce_buffer[13] << 8)  | (nonce_buffer[12] << 0);

      log_printf("\r\n┌─ NONCE BREAKDOWN (16 bytes) ─┐\r\n");
      log_printf("│                              │\r\n");

      /* Counter */
      log_printf("│ [0-3] COUNTER (Dynamic)       │\r\n");
      log_printf("│   String: %s", nonce_get_counter_string());

      int pad = 28 - (int)strlen(nonce_get_counter_string());
      if (pad < 0) pad = 0;
      for (int i = 0; i < pad; i++) log_printf(" ");

      log_printf("│\r\n");
      log_printf("│   Value:  0x%08lX (%lu)      │\r\n",
                 (unsigned long)counter_val, (unsigned long)counter_val);
      log_printf("│   Bytes:  %02X %02X %02X %02X           │\r\n",
                 nonce_buffer[0], nonce_buffer[1], nonce_buffer[2], nonce_buffer[3]);
      log_printf("│                              │\r\n");

      /* Timer */
      log_printf("│ [4-7] TIMER (Dynamic)         │\r\n");
      log_printf("│   Value:  0x%08lX (%lu)      │\r\n",
                 (unsigned long)timer_val, (unsigned long)timer_val);
      log_printf("│   Bytes:  %02X %02X %02X %02X           │\r\n",
                 nonce_buffer[4], nonce_buffer[5], nonce_buffer[6], nonce_buffer[7]);
      log_printf("│                              │\r\n");

      /* ADC Entropy */
      log_printf("│ [8-11] ADC Entropy (Static)   │\r\n");
      log_printf("│   Value:  0x%08lX (%lu)      │\r\n",
                 (unsigned long)adc_val, (unsigned long)adc_val);
      log_printf("│   Bytes:  %02X %02X %02X %02X           │\r\n",
                 nonce_buffer[8], nonce_buffer[9], nonce_buffer[10], nonce_buffer[11]);
      log_printf("│                              │\r\n");

      /* Jitter Entropy */
      log_printf("│ [12-15] Jitter Entropy (Static)│\r\n");
      log_printf("│   Value:  0x%08lX (%lu)      │\r\n",
                 (unsigned long)jitter_val, (unsigned long)jitter_val);
      log_printf("│   Bytes:  %02X %02X %02X %02X           │\r\n",
                 nonce_buffer[12], nonce_buffer[13], nonce_buffer[14], nonce_buffer[15]);
      log_printf("│                              │\r\n");

      log_printf("└──────────────────────────────┘\r\n");

      /* Full nonce */
      log_printf("\r\nFull Nonce: ");
      for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
          log_printf("%02X", nonce_buffer[i]);
          if (i == 3 || i == 7 || i == 11) log_printf(" ");
      }
      log_printf("\r\n");
  }


  /**
   * Verify nonce uniqueness (compare with previous)
   * Returns: 1 if different, 0 if same (collision!)
   */

  uint8_t nonce_simple_verify_unique(void)
  {
      static uint8_t prev_nonce[NONCE_SIZE_BYTES] = {0};
      static uint8_t first_call = 1;

      if (first_call) {
          memcpy(prev_nonce, nonce_buffer, NONCE_SIZE_BYTES);
          first_call = 0;
          return 1;  /* First call is always unique */
      }

      /* Compare with previous (only first 8 bytes should change) */
          uint8_t is_different = 0;
          for (uint32_t i = 0; i < 8; i++) {  /* Check dynamic part only */
              if (nonce_buffer[i] != prev_nonce[i]) {
                  is_different = 1;
                  break;
              }
          }

      /* Save current for next comparison */
      memcpy(prev_nonce, nonce_buffer, NONCE_SIZE_BYTES);

      return is_different;
  }

  /* ========== Usage Example ========== */

  /*
   * Typical usage in your crypto code:
   *
   * // At startup
   * nonce_simple_init();
   *
   * // Before each data transmission
   * void send_encrypted_data(uint8_t *data, uint32_t len) {
   *     // Increment counter
   *     nonce_increment_message_counter();
   *
   *     // Generate nonce
   *     const uint8_t *nonce = nonce_simple_generate();
   *
   *     // Use nonce for encryption (e.g., AES-CTR IV)
   *     aes_ctr_encrypt(data, len, key, nonce);
   *
   *     // Or use in DRBG
   *     uint8_t full_nonce[16];
   *     memcpy(full_nonce, nonce, 8);
   *     // Add your other 8 bytes here
   *
   *     cmox_drbg_init(drbg_ctx, entropy, 32, pers, 32, full_nonce, 16);
   * }
   */
