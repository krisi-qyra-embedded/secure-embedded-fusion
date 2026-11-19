/**
 * @file stm32_gcm_nvm.h
 * @brief Flash journal for AES-GCM TX sessions and RAM replay tracking.
 */
#ifndef STM32_GCM_NVM_H_
#define STM32_GCM_NVM_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
/* TX nonce uses flash session ID || RAM packet counter. */
#ifndef STM32_GCM_NVM_FLASH_SECTOR
#define STM32_GCM_NVM_FLASH_SECTOR     FLASH_SECTOR_4
#endif
#ifndef STM32_GCM_NVM_FLASH_BASE
#define STM32_GCM_NVM_FLASH_BASE       UINT32_C(0x080175E0)
#endif
#ifndef STM32_GCM_NVM_FLASH_SIZE
#define STM32_GCM_NVM_FLASH_SIZE       UINT32_C(0x00008A20)
#endif
#ifndef STM32_GCM_NVM_BOOTSTRAP_TX_SESSION_FLOOR
#define STM32_GCM_NVM_BOOTSTRAP_TX_SESSION_FLOOR UINT32_C(0x00010000)
#endif
/** @brief AES-GCM Flash journal status. */
typedef enum
{
   STM32_GCM_NVM_OK = 0,
   STM32_GCM_NVM_ERR_PARAM = -1,
   STM32_GCM_NVM_ERR_CORRUPT = -2,
   STM32_GCM_NVM_ERR_FULL = -3,
   STM32_GCM_NVM_ERR_FLASH = -4,
   STM32_GCM_NVM_ERR_EXHAUSTED = -5
} stm32_gcm_nvm_status_t;
/** @brief RAM replay-window decision for a received GCM nonce. */
typedef enum
{
   STM32_GCM_REPLAY_ACCEPT_NEW = 0,
   STM32_GCM_REPLAY_DUPLICATE_LAST,
   STM32_GCM_REPLAY_REJECT_OLD,
   STM32_GCM_REPLAY_STORAGE_ERROR,
   STM32_GCM_REPLAY_BAD_NONCE
} stm32_gcm_nvm_replay_result_t;
/**
 * @brief Scan and initialize the TX session journal.
 * @return NVM status.
 */
stm32_gcm_nvm_status_t stm32_gcm_nvm_init(void);
/**
 * @brief Reserve the next persistent TX session ID for this boot.
 * @param session_id_out Reserved session ID.
 * @return NVM status.
 */
stm32_gcm_nvm_status_t stm32_gcm_nvm_reserve_tx_session(
   uint32_t *session_id_out);
/**
 * @brief Check and commit the RX replay high-water state in RAM.
 * @param session_id Received session ID.
 * @param counter Received packet counter.
 * @return Replay decision.
 */
stm32_gcm_nvm_replay_result_t stm32_gcm_nvm_check_and_commit_rx(
   uint32_t session_id,
   uint64_t counter);
/**
 * @brief Get the active TX session ID.
 * @return TX session ID.
 */
uint32_t stm32_gcm_nvm_current_tx_session(void);
/**
 * @brief Get the current RX replay session ID.
 * @return RX session ID.
 */
uint32_t stm32_gcm_nvm_current_rx_session(void);
/**
 * @brief Get the current RX replay counter.
 * @return RX packet counter.
 */
uint64_t stm32_gcm_nvm_current_rx_counter(void);
/**
 * @brief Get the remaining Flash journal capacity.
 * @return Number of unused record slots.
 */
uint32_t stm32_gcm_nvm_remaining_record_slots(void);
#ifdef __cplusplus
}
#endif
#endif /* STM32_GCM_NVM_H_ */
