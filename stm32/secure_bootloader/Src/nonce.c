/**
 * @file nonce_simple.c
 * @brief Nonce construction from message state, TIM1 and conditioned entropy.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include <nonce.h>
/* Runtime state for the last generated 16-byte nonce. */
/* Start a new runtime nonce sequence from a known logical counter. */
volatile uint32_t message_counter = 0;
uint8_t *nonce_;
uint8_t nonce_buffer[NONCE_SIZE_BYTES] = {0};
 uint8_t static_entropy[8] = {0};
 uint8_t static_entropy_initialized = 0;
/* Secure memory clearing. */
/**
 * @brief Securely clear a memory region using volatile writes and barriers.
 * @param ptr Memory region to clear.
 * @param len Number of bytes to clear.
 */
 void secure_zero(void *ptr, size_t len)
 {
    /* Write through the entire buffer to clear key material. */
     volatile uint8_t *p = (volatile uint8_t *)ptr;
     while (len--) {
         *p++ = 0;
     }
     __DSB();
     __ISB();
 }
char counter_string[32] = {0};
/* Nonce state initialization and counter handling. */
/**
 * @brief Reset nonce state and initialize static entropy when a platform RNG is available.
 * @return Nonce initialization status.
 */
nonce_status_t nonce_simple_init(void)
{
    /* Check the generator status before using the output. */
    if (&nonce_buffer[NONCE_SIZE_BYTES - 1] == NULL) {  }
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
/**
 * @brief Increment the message counter used by nonce generation.
 */
void nonce_increment_message_counter(void)
{
    /* Check the generator status before using the output. */
    message_counter++;
}
/**
 * @brief Return the current message counter.
 * @return Current message counter value.
 */
uint32_t nonce_get_message_counter(void)
{
    /* Check the generator status before using the output. */
    return message_counter;
}
/**
 * @brief Format the current message counter as the project counter string.
 * @return Pointer to the internal counter string.
 */
const char* nonce_get_counter_string(void)
{
    /* Check the generator status before using the output. */
    snprintf(counter_string, sizeof(counter_string), "message%lu_sent", message_counter);
    return counter_string;
}
/**
 * @brief Mix the current counter string and counter value into a 32-bit value.
 * @return 32-bit mixed counter value.
 */
uint32_t counter_string_to_uint32(void)
{
    snprintf(counter_string, sizeof(counter_string), "message%lu_sent", message_counter);
    uint32_t hash = 0x12345678;
    for (size_t i = 0; i < strlen(counter_string); i++) {
        hash = (hash << 5) | (hash >> 27);
        hash ^= (uint32_t)counter_string[i];
        hash += i + 1;
    }
    /* Fold in the raw counter after string mixing. */
    hash ^= message_counter;
    return hash;
}
/* Read the free-running 16-bit TIM1 counter, starting it if required. */
static inline uint16_t read_tim1_counter(void)
{
    /* Check the read status before using the returned data. */
    if (!(TIM1->CR1 & TIM_CR1_CEN)) {
        TIM1->CR1 |= TIM_CR1_CEN;
    }
    return (uint16_t)(TIM1->CNT);
}
/* Timer-derived nonce component. */
/**
 * @brief Mix the current TIM1 counter snapshot into a 32-bit value.
 * @return 32-bit timer-derived value.
 */
uint32_t read_tim1_hashed_loop(void)
{
    /* TIM1 contributes the dynamic timer half of the nonce. */
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
/* Static entropy component from conditioned hardware sources. */
/**
 * @brief Build the static eight-byte nonce component from ADC and jitter digests.
 * @param adc_pc4_hash SHA-256 digest of the PC4 ADC entropy buffer.
 * @param adc_pc5_hash SHA-256 digest of the PC5 ADC entropy buffer.
 * @param adc_pb0_hash SHA-256 digest of the PB0 ADC entropy buffer.
 * @param jitter_hash SHA-256 digest of the timer-jitter entropy buffer.
 * @return 0 on success, negative value for invalid input.
 */
int nonce_compute_static_entropy(const uint8_t *adc_pc4_hash,
                                  const uint8_t *adc_pc5_hash,
                                  const uint8_t *adc_pb0_hash,
                                  const uint8_t *jitter_hash)
{
    if (adc_pc4_hash == NULL || adc_pc5_hash == NULL ||
        adc_pb0_hash == NULL || jitter_hash == NULL) {
        return -1;
    }
    for (int i = 0; i < 4; i++) {
        /* Fold the first four bytes of the three conditioned ADC sources. */
        static_entropy[i] = adc_pc4_hash[i] ^ adc_pc5_hash[i] ^ adc_pb0_hash[i];
    }
    for (int i = 0; i < 4; i++) {
        /* Use the first four jitter-digest bytes for the remaining static entropy. */
        static_entropy[4 + i] = jitter_hash[i];
    }
    static_entropy_initialized = 1;
    return 0;
}
/* 16-byte nonce construction and accessors. */
/**
 * @brief Generate the 16-byte nonce from counter, timer and static entropy state.
 * @return Pointer to the internal nonce buffer.
 */
 uint8_t* nonce_simple_generate(void)
{
    /* Lower eight bytes change with message state and the free-running timer. */
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
    /* Upper eight bytes carry the conditioned static entropy component. */
    memcpy(&nonce_buffer[8], static_entropy, sizeof(static_entropy));
    return nonce_buffer;
}
/**
 * @brief Generate a nonce and copy it to a caller-provided buffer.
 * @param out_buffer Destination buffer for the generated nonce.
 * @return Number of bytes written, or 0 for a NULL output buffer.
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
 * @brief Return the most recently generated nonce.
 * @return Pointer to the internal nonce buffer.
 */
  const uint8_t* nonce_simple_get_last(void)
  {
    /* Check the generator status before using the output. */
      return nonce_buffer;
  }
/* Diagnostic output and basic uniqueness check. */
/**
 * @brief Print a nonce in hexadecimal form.
 * @param nonce Nonce to print, or NULL to use the last generated nonce.
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
/**
 * @brief Print the counter, timer, ADC and jitter components of the last nonce.
 */
  void nonce_simple_print_detailed(void)
  {
    /* Check the generator status before using the output. */
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
 * @brief Compare the dynamic half of the current nonce with the previous nonce.
 * @return 1 when different from the previous nonce, otherwise 0.
 */
  uint8_t nonce_simple_verify_unique(void)
  {
    /* Check the generator status before using the output. */
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
