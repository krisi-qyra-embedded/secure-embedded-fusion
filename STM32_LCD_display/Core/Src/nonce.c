

/**
 * @file nonce_simple.c
 * @brief Nonce generation and entropy mixing helpers.
 */


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



#include <nonce.h>
volatile uint32_t message_counter = 0;
uint8_t *nonce_;
uint8_t nonce_buffer[NONCE_SIZE_BYTES] = {0};
 uint8_t static_entropy[8] = {0};
 uint8_t static_entropy_initialized = 0;
/**
 * @brief Clear memory using volatile writes.
 * @param ptr Buffer to clear.
 * @param len Number of bytes.
 */
 void secure_zero(void *ptr, size_t len)
 {
     volatile uint8_t *p = (volatile uint8_t *)ptr;
     while (len--) {
         *p++ = 0;
     }
 }
char counter_string[32] = {0};
/**
 * @brief Initialize the nonce state.
 * @return Nonce status.
 */
nonce_status_t nonce_simple_init(void)
{
    if (&nonce_buffer[NONCE_SIZE_BYTES - 1] == NULL) {  }
    /* Keep init one-shot unless the module is reset. */
    if (static_entropy_initialized) {
        return NONCE_ERR_ALREADY_INIT;
    }
    /* Start the logical counter from zero. */
    message_counter = 0;
    /* Clear any previous nonce state. */
    secure_zero(nonce_buffer, (size_t)NONCE_SIZE_BYTES);
    secure_zero(static_entropy, (size_t)STATIC_ENTROPY_SIZE);
    secure_zero(counter_string, (size_t)COUNTER_STRING_SIZE);
#ifdef PLATFORM_HAS_HW_RNG
    if (platform_get_random_bytes != NULL) {
        if (platform_get_random_bytes(static_entropy, (size_t)STATIC_ENTROPY_SIZE) != 0) {
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
/* Increment the message counter. */
void nonce_increment_message_counter(void)
{
    message_counter++;
}
/* Return the current message counter. */
uint32_t nonce_get_message_counter(void)
{
    return message_counter;
}
/* Build the current message counter string. */
const char* nonce_get_counter_string(void)
{
    /* Counter text is used as one input to the mixer. */
    snprintf(counter_string, sizeof(counter_string), "message%lu_sent", message_counter);
    return counter_string;
}
/* Mix the counter string into a 32-bit value. */
uint32_t counter_string_to_uint32(void)
{
    snprintf(counter_string, sizeof(counter_string), "message%lu_sent", message_counter);
    uint32_t hash = 0x12345678;
    for (size_t i = 0; i < strlen(counter_string); i++) {
        hash = (hash << 5) | (hash >> 27);
        hash ^= (uint32_t)counter_string[i];
        hash += i + 1;
    }
    /* Fold the raw counter in after the string mix. */
    hash ^= message_counter;
    return hash;
}
/* Read the current TIM1 counter. */
static inline uint16_t read_tim1_counter(void)
{
    if (!(TIM1->CR1 & TIM_CR1_CEN)) {
        TIM1->CR1 |= TIM_CR1_CEN;
    }
    return (uint16_t)(TIM1->CNT);
}
/* Mix the TIM1 sample into a 32-bit value. */
uint32_t read_tim1_hashed_loop(void)
{
    /* Take one free-running timer sample. */
    uint16_t cnt = read_tim1_counter();
    uint8_t bytes[4];
    /* Expand the timer sample before the 32-bit mix. */
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
/**
 * @brief Build the static nonce entropy from ADC and jitter hashes.
 * @param adc_pc4_hash PC4 hash.
 * @param adc_pc5_hash PC5 hash.
 * @param adc_pb0_hash PB0 hash.
 * @param jitter_hash Timer-jitter hash.
 * @return 0 on success, otherwise negative.
 */
int nonce_compute_static_entropy(const uint8_t *adc_pc4_hash,
                                  const uint8_t *adc_pc5_hash,
                                  const uint8_t *adc_pb0_hash,
                                  const uint8_t *jitter_hash)
{
    /* All four source hashes must be available. */
    if (adc_pc4_hash == NULL || adc_pc5_hash == NULL ||
        adc_pb0_hash == NULL || jitter_hash == NULL) {
        return -1;
    }
    /* First half uses the three ADC source hashes. */
    for (int i = 0; i < 4; i++) {
        static_entropy[i] = adc_pc4_hash[i] ^ adc_pc5_hash[i] ^ adc_pb0_hash[i];
    }
    /* Second half comes from timer jitter. */
    for (int i = 0; i < 4; i++) {
        static_entropy[4 + i] = jitter_hash[i];
    }
    static_entropy_initialized = 1;
    return 0;
}
/**
 * @brief Generate the current 16-byte nonce.
 * @return Pointer to the nonce buffer.
 */
 uint8_t* nonce_simple_generate(void)
{
    /* First 8 bytes are dynamic for each generated nonce. */
    uint32_t counter_val = counter_string_to_uint32();
    uint32_t timer_val   = read_tim1_hashed_loop();
    /* Bytes 0..3: counter-derived value. */
    nonce_buffer[0] = (counter_val >>  0) & 0xFF;
    nonce_buffer[1] = (counter_val >>  8) & 0xFF;
    nonce_buffer[2] = (counter_val >> 16) & 0xFF;
    nonce_buffer[3] = (counter_val >> 24) & 0xFF;
    /* Bytes 4..7: timer-derived value. */
    nonce_buffer[4] = (timer_val >>  0) & 0xFF;
    nonce_buffer[5] = (timer_val >>  8) & 0xFF;
    nonce_buffer[6] = (timer_val >> 16) & 0xFF;
    nonce_buffer[7] = (timer_val >> 24) & 0xFF;
    /* Build the static half once from the entropy hashes. */
    if (!static_entropy_initialized) {
        if (nonce_compute_static_entropy(buf_pc4_hashed,
                                         buf_pc5_hashed,
                                         buf_pb0_hashed,
                                         jitter_accum_hashed) != 0) {
            memset(static_entropy, 0, sizeof(static_entropy));
        }
        static_entropy_initialized = 1;
    }
    /* Bytes 8..15: stored ADC/jitter entropy. */
    memcpy(&nonce_buffer[8], static_entropy, sizeof(static_entropy));
    return nonce_buffer;
}
/**
 * @brief Generate a nonce and copy it to the caller buffer.
 * @param out_buffer Destination buffer.
 * @return Number of bytes written.
 */
  uint32_t nonce_simple_generate_to_buffer(uint8_t *out_buffer)
  {
      if (out_buffer == NULL) {
          return 0;
      }
      const uint8_t *nonce = nonce_simple_generate();
      for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
          out_buffer[i] = nonce[i];
      }
      return NONCE_SIZE_BYTES;
  }
/* Return the last generated nonce. */
  const uint8_t* nonce_simple_get_last(void)
  {
      return nonce_buffer;
  }
/* Print the nonce in hex for debug. */
  void nonce_simple_print(const uint8_t *nonce)
  {
      if (nonce == NULL) {
          nonce = nonce_buffer;
      }
      log_printf("Nonce (8 bytes): ");
      for (uint32_t i = 0; i < NONCE_SIZE_BYTES; i++) {
    	  log_printf("%02X ", nonce[i]);
      }
      log_printf("\r\n");
  }
/* Print the nonce fields for debug. */
  void nonce_simple_print_detailed(void)
  {
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
/**
 * @brief Compare the current nonce with the previous one.
 * @return 1 when different, otherwise 0.
 */
  uint8_t nonce_simple_verify_unique(void)
  {
      static uint8_t prev_nonce[NONCE_SIZE_BYTES] = {0};
      static uint8_t first_call = 1;
      /* First generated nonce becomes the comparison reference. */
      if (first_call) {
          memcpy(prev_nonce, nonce_buffer, NONCE_SIZE_BYTES);
          first_call = 0;
          return 1;
      }
          uint8_t is_different = 0;
          /* Only the dynamic half is checked for change. */
          for (uint32_t i = 0; i < 8; i++) {
              if (nonce_buffer[i] != prev_nonce[i]) {
                  is_different = 1;
                  break;
              }
          }
      memcpy(prev_nonce, nonce_buffer, NONCE_SIZE_BYTES);
      return is_different;
  }
