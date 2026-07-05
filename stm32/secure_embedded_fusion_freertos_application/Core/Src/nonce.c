/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include <nonce.h>


volatile uint32_t message_counter = 0;
uint8_t *nonce_;
uint8_t nonce_buffer[NONCE_SIZE_BYTES] = {0};

 uint8_t static_entropy[8] = {0};
 uint8_t static_entropy_initialized = 0;

 /* Clears a memory region through volatile byte writes. */
 void secure_zero(void *ptr, size_t len)
 {
     /* Use volatile stores to retain the clearing loop under optimization. */
     volatile uint8_t *p = (volatile uint8_t *)ptr;

     while (len--) {
         *p++ = 0;
     }
 }

char counter_string[32] = {0};

/* Resets the nonce state and initializes optional RNG input. */
nonce_status_t nonce_simple_init(void)
{
    /* Reject reinitialization after static entropy has been marked ready. */
    /* Defensive runtime checks for external arrays: try to catch obviously wrong sizes.
     * We expect the linker to provide arrays sized exactly as the macros indicate.
     * If sizes mismatch, behaviour is undefined; we attempt to avoid silent success.
     */
    if (&nonce_buffer[NONCE_SIZE_BYTES - 1] == NULL) {   }

    if (static_entropy_initialized) {
        return NONCE_ERR_ALREADY_INIT;
    }

    message_counter = 0;

    secure_zero(nonce_buffer, (size_t)NONCE_SIZE_BYTES);
    secure_zero(static_entropy, (size_t)STATIC_ENTROPY_SIZE);
    secure_zero(counter_string, (size_t)COUNTER_STRING_SIZE);

#ifdef PLATFORM_HAS_HW_RNG
    if (platform_get_random_bytes != NULL) {
        if (platform_get_random_bytes(static_entropy, (size_t)STATIC_ENTROPY_SIZE) != 0) {
            /* failed to obtain entropy -> wipe and return error */
            secure_zero(static_entropy, (size_t)STATIC_ENTROPY_SIZE);
            return NONCE_ERR_ENTROPY_INIT_FAIL;
        }
        static_entropy_initialized = 1;
    } else {
        return NONCE_ERR_ENTROPY_INIT_FAIL;
    }
#else
    static_entropy_initialized = 0;
#endif

    return NONCE_OK;
}

/* Advances the runtime nonce message counter. */
void nonce_increment_message_counter(void)
{
    /* The counter is volatile runtime state and is not persisted here. */
    message_counter++;
}

/* Returns the current nonce message counter. */
uint32_t nonce_get_message_counter(void)
{
    /* Read the shared runtime counter without advancing it. */
    return message_counter;
}

/* Formats the current message counter label. */
const char* nonce_get_counter_string(void)
{
    /* Bound the formatting to the counter-string buffer. */
    snprintf(counter_string, sizeof(counter_string), "message%lu_sent", message_counter);
    return counter_string;
}

/* Mixes the formatted message counter into a 32-bit value. */
uint32_t counter_string_to_uint32(void)
{
    /* Rotate and combine the label bytes before mixing the numeric counter. */
    snprintf(counter_string, sizeof(counter_string), "message%lu_sent", message_counter);

    uint32_t hash = 0x12345678;   

    for (size_t i = 0; i < strlen(counter_string); i++) {
        hash = (hash << 5) | (hash >> 27);   
        hash ^= (uint32_t)counter_string[i];
        hash += i + 1;   
    }

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
/* Reads TIM1's current 16-bit counter value. */
static inline uint16_t read_tim1_counter(void)
{
    /* Start TIM1 if counting is disabled before taking the snapshot. */
    if (!(TIM1->CR1 & TIM_CR1_CEN)) {
        TIM1->CR1 |= TIM_CR1_CEN;
    }
    return (uint16_t)(TIM1->CNT);
}

/* Mixes a TIM1 snapshot into a 32-bit value. */
uint32_t read_tim1_hashed_loop(void)
{
    /* Combine derived counter bytes with rotations of the original snapshot. */
    uint16_t cnt = read_tim1_counter();
    uint8_t bytes[4];
    bytes[0] = (uint8_t)(cnt & 0xFF);                  
    bytes[1] = (uint8_t)((cnt >> 8) & 0xFF);           
    bytes[2] = (uint8_t)(((cnt ^ (cnt >> 3)) & 0xFF));          
    bytes[3] = (uint8_t)(((cnt * 7u) ^ (cnt >> 5)) & 0xFF);    
    uint32_t hash = 0x12345678U;   
    for (size_t i = 0; i < sizeof(bytes); ++i) {
        hash = (hash << 5) | (hash >> 27);
        hash ^= (uint32_t)bytes[i];
        hash += (uint32_t)(i + 1);
        uint32_t rotated_cnt = (((uint32_t)cnt << (i)) | ((uint32_t)cnt >> (16 - (i)))) & 0xFFFFU;
        hash ^= rotated_cnt;
    }
    hash ^= (uint32_t)cnt;
    return hash;
}

/* ========== Static Entropy Component ========== */

/* Builds the static ADC and jitter portion of the nonce. */
int nonce_compute_static_entropy(const uint8_t *adc_pc4_hash,
                                  const uint8_t *adc_pc5_hash,
                                  const uint8_t *adc_pb0_hash,
                                  const uint8_t *jitter_hash)
{
    /* XOR the three ADC digest prefixes and append the jitter digest prefix. */
    if (adc_pc4_hash == NULL || adc_pc5_hash == NULL ||
        adc_pb0_hash == NULL || jitter_hash == NULL) {
        return -1;   
    }

    for (int i = 0; i < 4; i++) {
        static_entropy[i] = adc_pc4_hash[i] ^ adc_pc5_hash[i] ^ adc_pb0_hash[i];
    }

    for (int i = 0; i < 4; i++) {
        static_entropy[4 + i] = jitter_hash[i];
    }

    static_entropy_initialized = 1;

    return 0;   
}

  /* ========== Nonce Generation ========== */

 /* Builds a nonce from the counter, timer and static inputs. */
 uint8_t* nonce_simple_generate(void)
{
    /* Initialize the static portion once, then copy it into the final eight bytes. */
    uint32_t counter_val = counter_string_to_uint32();
    uint32_t timer_val   = read_tim1_hashed_loop();

    nonce_buffer[0] = (counter_val >>  0) & 0xFF;
    nonce_buffer[1] = (counter_val >>  8) & 0xFF;
    nonce_buffer[2] = (counter_val >> 16) & 0xFF;
    nonce_buffer[3] = (counter_val >> 24) & 0xFF;

    nonce_buffer[4] = (timer_val >>  0) & 0xFF;
    nonce_buffer[5] = (timer_val >>  8) & 0xFF;
    nonce_buffer[6] = (timer_val >> 16) & 0xFF;
    nonce_buffer[7] = (timer_val >> 24) & 0xFF;

    if (!static_entropy_initialized) {
        if (nonce_compute_static_entropy(buf_pc4_hashed,
                                         buf_pc5_hashed,
                                         buf_pb0_hashed,
                                         jitter_accum_hashed) != 0) {
            memset(static_entropy, 0, sizeof(static_entropy));
        }
        static_entropy_initialized = 1;
    }

    memcpy(&nonce_buffer[8], static_entropy, sizeof(static_entropy));

    return nonce_buffer;
}

  /* Copies a generated nonce to the caller's buffer. */
  uint32_t nonce_simple_generate_to_buffer(uint8_t *out_buffer)
  {
      /* Require a non-null destination before copying the configured nonce length. */
      if (out_buffer == NULL) {
          return 0;
      }

      const uint8_t *nonce = nonce_simple_generate();

      for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
          out_buffer[i] = nonce[i];
      }

      return NONCE_SIZE_BYTES;
  }

  /* Returns the last generated nonce buffer. */
  const uint8_t* nonce_simple_get_last(void)
  {
      /* Expose the existing buffer without generating a new nonce. */
      return nonce_buffer;
  }

  /* ========== Display Functions ========== */

  /* Logs a nonce byte sequence. */
  void nonce_simple_print(const uint8_t *nonce)
  {
      /* Use the current nonce buffer when no input pointer is supplied. */
      if (nonce == NULL) {
          nonce = nonce_buffer;
      }

      log_printf("Nonce (8 bytes): ");
      for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
    	  log_printf("%02X ", nonce[i]);
      }
      log_printf("\r\n");
  }

  /* Logs the decoded counter, timer and static nonce fields. */
  void nonce_simple_print_detailed(void)
  {
      /* Decode each four-byte field from the stored byte order. */
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

      log_printf("│ [4-7] TIMER (Dynamic)         │\r\n");
      log_printf("│   Value:  0x%08lX (%lu)      │\r\n",
                 (unsigned long)timer_val, (unsigned long)timer_val);
      log_printf("│   Bytes:  %02X %02X %02X %02X           │\r\n",
                 nonce_buffer[4], nonce_buffer[5], nonce_buffer[6], nonce_buffer[7]);
      log_printf("│                              │\r\n");

      log_printf("│ [8-11] ADC Entropy (Static)   │\r\n");
      log_printf("│   Value:  0x%08lX (%lu)      │\r\n",
                 (unsigned long)adc_val, (unsigned long)adc_val);
      log_printf("│   Bytes:  %02X %02X %02X %02X           │\r\n",
                 nonce_buffer[8], nonce_buffer[9], nonce_buffer[10], nonce_buffer[11]);
      log_printf("│                              │\r\n");

      log_printf("│ [12-15] Jitter Entropy (Static)│\r\n");
      log_printf("│   Value:  0x%08lX (%lu)      │\r\n",
                 (unsigned long)jitter_val, (unsigned long)jitter_val);
      log_printf("│   Bytes:  %02X %02X %02X %02X           │\r\n",
                 nonce_buffer[12], nonce_buffer[13], nonce_buffer[14], nonce_buffer[15]);
      log_printf("│                              │\r\n");

      log_printf("└──────────────────────────────┘\r\n");

      log_printf("\r\nFull Nonce: ");
      for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
          log_printf("%02X", nonce_buffer[i]);
          if (i == 3 || i == 7 || i == 11) log_printf(" ");
      }
      log_printf("\r\n");
  }

  /* Checks whether the dynamic nonce bytes changed since the last call. */
  uint8_t nonce_simple_verify_unique(void)
  {
      /* Compare only the first eight bytes and retain the current nonce for the next check. */
      static uint8_t prev_nonce[NONCE_SIZE_BYTES] = {0};
      static uint8_t first_call = 1;

      if (first_call) {
          memcpy(prev_nonce, nonce_buffer, NONCE_SIZE_BYTES);
          first_call = 0;
          return 1;   
      }

          uint8_t is_different = 0;
          for (uint32_t i = 0; i < 8; i++) {   
              if (nonce_buffer[i] != prev_nonce[i]) {
                  is_different = 1;
                  break;
              }
          }

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
