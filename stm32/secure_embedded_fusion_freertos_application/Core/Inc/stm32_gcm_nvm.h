/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

/*
 * stm32_gcm_nvm.h
 *
 * GCM nonce/replay security-state backend for STM32F411.
 *
 * IMPORTANT ARCHITECTURE CHANGE:
 *   This implementation DOES NOT use internal FLASH.
 *
 *   It stores the small persistent security state in STM32F411 RTC backup
 *   registers instead. This avoids FLASH-sector overlap, erase problems and
 *   append-journal exhaustion.
 *
 * Reserved backup-register range:
 *   BKP4R  .. BKP11R  = redundant slot A
 *   BKP12R .. BKP19R  = redundant slot B
 *
 * BKP0R..BKP3R are deliberately left untouched for RTC/application use.
 *
 * Two slots are used with CRC32 + commit-last semantics. On each update the
 * inactive slot is written and committed last; the newest valid generation is
 * loaded on the next access.
 *
 * The API names retain "nvm" only for source compatibility with the current
 * ECDH/UART code.
 *
 * NOTE:
 *   Backup-register retention across removal of VDD depends on the board's
 *   VBAT/backup-domain supply. Software that resets the backup domain or a
 *   tamper configuration that clears backup registers will also clear this
 *   security state.
 */

#ifndef STM32_GCM_NVM_H_
#define STM32_GCM_NVM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx.h"

#define STM32_GCM_BKP_FIRST_REG            4U
#define STM32_GCM_BKP_REGS_PER_SLOT        8U
#define STM32_GCM_BKP_SLOT_COUNT           2U
#define STM32_GCM_BKP_TOTAL_REGS           \
    (STM32_GCM_BKP_REGS_PER_SLOT * STM32_GCM_BKP_SLOT_COUNT)

#if (STM32_GCM_BKP_FIRST_REG + STM32_GCM_BKP_TOTAL_REGS) > 20U
#error "STM32 GCM backup-register reservation exceeds BKP19R"
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

/*
 * Reserve the next STM32->ESP32 monotonic session epoch and commit it to the
 * RTC backup-register state before returning it to the caller.
 */
stm32_gcm_nvm_status_t stm32_gcm_nvm_reserve_tx_session(
    uint32_t *session_id_out);

/*
 * Check an authenticated ESP32->STM32 nonce against the retained replay
 * high-water mark and commit ACCEPT_NEW state before returning it.
 */
stm32_gcm_nvm_replay_result_t stm32_gcm_nvm_check_and_commit_rx(
    uint32_t session_id,
    uint64_t counter);

uint32_t stm32_gcm_nvm_current_tx_session(void);
uint32_t stm32_gcm_nvm_current_rx_session(void);
uint64_t stm32_gcm_nvm_current_rx_counter(void);

uint32_t stm32_gcm_nvm_remaining_record_slots(void);

/*
 * Explicitly erase ONLY this module's BKP4R..BKP19R state.
 * Do not call this during normal runtime; clearing replay/session state while
 * keeping the same traffic keys weakens reset/replay guarantees.
 */
stm32_gcm_nvm_status_t stm32_gcm_nvm_clear_security_state(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32_GCM_NVM_H_ */
