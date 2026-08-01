/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#ifndef STM32_GCM_NVM_H_
#define STM32_GCM_NVM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifndef STM32_GCM_NVM_FLASH_SECTOR
#define STM32_GCM_NVM_FLASH_SECTOR     FLASH_SECTOR_7
#endif

#ifndef STM32_GCM_NVM_FLASH_BASE
#define STM32_GCM_NVM_FLASH_BASE       UINT32_C(0x08060090)
#endif

#ifndef STM32_GCM_NVM_FLASH_SIZE
#define STM32_GCM_NVM_FLASH_SIZE       UINT32_C(0x0001FF70)
#endif

#ifndef STM32_GCM_NVM_BOOTSTRAP_TX_SESSION_FLOOR
#define STM32_GCM_NVM_BOOTSTRAP_TX_SESSION_FLOOR UINT32_C(0x00010000)
#endif

typedef enum
{
    STM32_GCM_NVM_OK = 0,
    STM32_GCM_NVM_ERR_PARAM = -1,
    STM32_GCM_NVM_ERR_CORRUPT = -2,
    STM32_GCM_NVM_ERR_FULL = -3,
    STM32_GCM_NVM_ERR_FLASH = -4,
    STM32_GCM_NVM_ERR_EXHAUSTED = -5

} stm32_gcm_nvm_status_t;

typedef enum
{
    STM32_GCM_REPLAY_ACCEPT_NEW = 0,
    STM32_GCM_REPLAY_DUPLICATE_LAST,
    STM32_GCM_REPLAY_REJECT_OLD,
    STM32_GCM_REPLAY_STORAGE_ERROR,
    STM32_GCM_REPLAY_BAD_NONCE

} stm32_gcm_nvm_replay_result_t;

stm32_gcm_nvm_status_t stm32_gcm_nvm_init(void);

stm32_gcm_nvm_status_t stm32_gcm_nvm_reserve_tx_session(
    uint32_t *session_id_out);

stm32_gcm_nvm_replay_result_t stm32_gcm_nvm_check_and_commit_rx(
    uint32_t session_id,
    uint64_t counter);

uint32_t stm32_gcm_nvm_current_tx_session(void);
uint32_t stm32_gcm_nvm_current_rx_session(void);
uint64_t stm32_gcm_nvm_current_rx_counter(void);
uint32_t stm32_gcm_nvm_remaining_record_slots(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32_GCM_NVM_H_ */
