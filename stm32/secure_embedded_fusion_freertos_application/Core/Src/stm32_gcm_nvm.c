/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "stm32_gcm_nvm.h"
#include <stddef.h>
#include <string.h>
#define STM32_GCM_NVM_MAGIC          UINT32_C(0x47434D32)
#define STM32_GCM_NVM_VERSION        UINT32_C(1)
#define STM32_GCM_NVM_COMMIT         UINT32_C(0xA55A3CC3)
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t generation;
    uint32_t tx_session_id;
    uint32_t rx_session_id;
    uint32_t rx_counter_hi;
    uint32_t rx_counter_lo;
    uint32_t crc32;
    uint32_t commit;
} stm32_gcm_nvm_record_t;
typedef char stm32_gcm_record_word_aligned[
    (sizeof(stm32_gcm_nvm_record_t) % sizeof(uint32_t)) == 0U ? 1 : -1];
static stm32_gcm_nvm_record_t s_state;
static uint32_t s_next_address;
static bool s_initialized;
static uint32_t s_rx_session_id;
static uint64_t s_rx_counter;
static bool s_tx_session_reserved_this_boot;
static uint32_t s_tx_session_id_this_boot;
/* Calculates the integrity CRC for a GCM state record. */
static uint32_t stm32_gcm_crc32(
    const uint8_t *data,
    size_t length)
{
    /* Apply the reflected 0xEDB88320 polynomial over the record bytes. */
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }
    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (unsigned int bit = 0U; bit < 8U; ++bit)
        {
            const uint32_t mask =
                (uint32_t)(0U - (crc & 1U));
            crc =
                (crc >> 1U) ^
                (UINT32_C(0xEDB88320) & mask);
        }
    }
    return ~crc;
}
/* Checks whether a Flash journal slot is erased. */
static bool stm32_gcm_record_erased(
    const stm32_gcm_nvm_record_t *record)
{
    /* Require every record word to contain the erased all-ones value. */
    const uint32_t *words;
    if (record == NULL)
    {
        return false;
    }
    words =
        (const uint32_t *)record;
    for (size_t i = 0U;
         i < (sizeof(*record) / sizeof(uint32_t));
         ++i)
    {
        if (words[i] != UINT32_C(0xFFFFFFFF))
        {
            return false;
        }
    }
    return true;
}
/* Validates a committed Flash GCM state record. */
static bool stm32_gcm_record_valid(
    const stm32_gcm_nvm_record_t *record)
{
    /* Check its magic, version, commit word and CRC. */
    uint32_t expected_crc;
    if (record == NULL)
    {
        return false;
    }
    if ((record->magic != STM32_GCM_NVM_MAGIC) ||
        (record->version != STM32_GCM_NVM_VERSION) ||
        (record->commit != STM32_GCM_NVM_COMMIT))
    {
        return false;
    }
    expected_crc =
        stm32_gcm_crc32(
            (const uint8_t *)record,
            offsetof(
                stm32_gcm_nvm_record_t,
                crc32));
    return record->crc32 == expected_crc;
}
/* Clears available Flash status flags before journal programming. */
static void stm32_gcm_clear_flash_flags(void)
{
    /* Include only flags exposed by the target HAL headers. */
    uint32_t flags = 0U;
#ifdef FLASH_FLAG_EOP
    flags |= FLASH_FLAG_EOP;
#endif
#ifdef FLASH_FLAG_OPERR
    flags |= FLASH_FLAG_OPERR;
#endif
#ifdef FLASH_FLAG_WRPERR
    flags |= FLASH_FLAG_WRPERR;
#endif
#ifdef FLASH_FLAG_PGAERR
    flags |= FLASH_FLAG_PGAERR;
#endif
#ifdef FLASH_FLAG_PGPERR
    flags |= FLASH_FLAG_PGPERR;
#endif
#ifdef FLASH_FLAG_PGSERR
    flags |= FLASH_FLAG_PGSERR;
#endif
    if (flags != 0U)
    {
        __HAL_FLASH_CLEAR_FLAG(flags);
    }
}
/* Appends and verifies a GCM security-state record in Flash. */
static stm32_gcm_nvm_status_t stm32_gcm_append_record(
    const stm32_gcm_nvm_record_t *candidate)
{
    /* I commit the marker last so interrupted writes remain invalid. */
    stm32_gcm_nvm_record_t record;
    const uint32_t *words;
    const stm32_gcm_nvm_record_t *slot;
    const uint32_t sector_end =
        STM32_GCM_NVM_FLASH_BASE +
        STM32_GCM_NVM_FLASH_SIZE;
    HAL_StatusTypeDef hal_status;
    bool unlocked = false;
    if (candidate == NULL)
    {
        return STM32_GCM_NVM_ERR_PARAM;
    }
    if ((!s_initialized) ||
        ((s_next_address + sizeof(record)) > sector_end))
    {
        return STM32_GCM_NVM_ERR_FULL;
    }
    slot =
        (const stm32_gcm_nvm_record_t *)s_next_address;
    if (!stm32_gcm_record_erased(slot))
    {
        return STM32_GCM_NVM_ERR_FULL;
    }
    record =
        *candidate;
    record.magic =
        STM32_GCM_NVM_MAGIC;
    record.version =
        STM32_GCM_NVM_VERSION;
    record.crc32 =
        stm32_gcm_crc32(
            (const uint8_t *)&record,
            offsetof(
                stm32_gcm_nvm_record_t,
                crc32));
    record.commit =
        STM32_GCM_NVM_COMMIT;
    words =
        (const uint32_t *)&record;
    hal_status =
        HAL_FLASH_Unlock();
    if (hal_status != HAL_OK)
    {
        return STM32_GCM_NVM_ERR_FLASH;
    }
    unlocked = true;
    stm32_gcm_clear_flash_flags();
    for (size_t i = 0U;
         i < ((sizeof(record) / sizeof(uint32_t)) - 1U);
         ++i)
    {
        hal_status =
            HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_WORD,
                s_next_address +
                    (uint32_t)(i * sizeof(uint32_t)),
                (uint64_t)words[i]);
        /*if (hal_status != HAL_OK)



        {



            uint32_t flash_error = HAL_FLASH_GetError();



            log_printf(



                "[FLASH][FAIL] "



                "status=%d error=0x%08lX "



                "addr=0x%08lX data=0x%08lX "



                "SR=0x%08lX CR=0x%08lX OPTCR=0x%08lX \r\n",



                (int)hal_status,



                (unsigned long)flash_error,



                (unsigned long)(



                    s_next_address +



                    (uint32_t)(i * sizeof(uint32_t))),



                (unsigned long)words[i],



                (unsigned long)FLASH->SR,



                (unsigned long)FLASH->CR,



                (unsigned long)FLASH->OPTCR);



            app_fault_delay_ms(100U);



            break;



        */
        if (hal_status != HAL_OK)
        {
            break;
        }
    }
    if (hal_status == HAL_OK)
    {
        const size_t commit_index =
            (sizeof(record) / sizeof(uint32_t)) - 1U;
        hal_status =
            HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_WORD,
                s_next_address +
                    (uint32_t)(
                        commit_index *
                        sizeof(uint32_t)),
                (uint64_t)words[commit_index]);
    }
    if (unlocked)
    {
        (void)HAL_FLASH_Lock();
    }
    if (hal_status != HAL_OK)
    {
        memset(
            &record,
            0,
            sizeof(record));
        return STM32_GCM_NVM_ERR_FLASH;
    }
    slot =
        (const stm32_gcm_nvm_record_t *)s_next_address;
    if ((!stm32_gcm_record_valid(slot)) ||
        (memcmp(
            slot,
            &record,
            sizeof(record)) != 0))
    {
        memset(
            &record,
            0,
            sizeof(record));
        return STM32_GCM_NVM_ERR_FLASH;
    }
    s_state =
        record;
    s_next_address +=
        sizeof(record);
    memset(
        &record,
        0,
        sizeof(record));
    return STM32_GCM_NVM_OK;
}
/* Loads the configured GCM security-state storage. */
stm32_gcm_nvm_status_t stm32_gcm_nvm_init(void)
{
    /* Scan committed Flash records before permitting session allocation. */
    const uint32_t sector_end =
        STM32_GCM_NVM_FLASH_BASE +
        STM32_GCM_NVM_FLASH_SIZE;
    uint32_t address;
    uint32_t last_used_end =
        STM32_GCM_NVM_FLASH_BASE;
    bool any_programmed = false;
    bool any_valid = false;
    if (s_initialized)
    {
        return STM32_GCM_NVM_OK;
    }
    if (((STM32_GCM_NVM_FLASH_BASE &
            (sizeof(uint32_t) - 1U)) != 0U) ||
        (STM32_GCM_NVM_FLASH_SIZE <
            sizeof(stm32_gcm_nvm_record_t)))
    {
        return STM32_GCM_NVM_ERR_PARAM;
    }
    memset(
        &s_state,
        0,
        sizeof(s_state));
    s_rx_session_id = 0U;
    s_rx_counter = UINT64_C(0);
    s_tx_session_reserved_this_boot = false;
    s_tx_session_id_this_boot = 0U;
    for (address = STM32_GCM_NVM_FLASH_BASE;
         (address + sizeof(stm32_gcm_nvm_record_t)) <= sector_end;
         address += sizeof(stm32_gcm_nvm_record_t))
    {
        const stm32_gcm_nvm_record_t *record =
            (const stm32_gcm_nvm_record_t *)address;
        if (stm32_gcm_record_erased(record))
        {
            continue;
        }
        any_programmed = true;
        last_used_end =
            address +
            sizeof(stm32_gcm_nvm_record_t);
        if (!stm32_gcm_record_valid(record))
        {
            continue;
        }
        if ((!any_valid) ||
            (record->generation >
                s_state.generation))
        {
            s_state =
                *record;
            any_valid = true;
        }
    }
    if (any_programmed && !any_valid)
    {
        memset(
            &s_state,
            0,
            sizeof(s_state));
        return STM32_GCM_NVM_ERR_CORRUPT;
    }
    s_next_address =
        last_used_end;
    s_initialized =
        true;
    return STM32_GCM_NVM_OK;
}
/* Reserves the transmit session identifier for this boot. */
stm32_gcm_nvm_status_t stm32_gcm_nvm_reserve_tx_session(
    uint32_t *session_id_out)
{
    /* Reject session or generation exhaustion before advancing state. */
    stm32_gcm_nvm_record_t next;
    stm32_gcm_nvm_status_t status;
    uint32_t previous_session;
    if (session_id_out == NULL)
    {
        return STM32_GCM_NVM_ERR_PARAM;
    }
    status =
        stm32_gcm_nvm_init();
    if (status != STM32_GCM_NVM_OK)
    {
        return status;
    }
    if (s_tx_session_reserved_this_boot)
    {
        *session_id_out =
            s_tx_session_id_this_boot;
        return STM32_GCM_NVM_OK;
    }
    previous_session =
        s_state.tx_session_id;
#ifdef STM32_GCM_NVM_BOOTSTRAP_TX_SESSION_FLOOR
    if (previous_session <
        STM32_GCM_NVM_BOOTSTRAP_TX_SESSION_FLOOR)
    {
        previous_session =
            STM32_GCM_NVM_BOOTSTRAP_TX_SESSION_FLOOR;
    }
#endif
    if ((previous_session == UINT32_MAX) ||
        (s_state.generation == UINT32_MAX))
    {
        return STM32_GCM_NVM_ERR_EXHAUSTED;
    }
    next =
        s_state;
    next.generation++;
    next.tx_session_id =
        previous_session + 1U;
    next.rx_session_id = 0U;
    next.rx_counter_hi = 0U;
    next.rx_counter_lo = 0U;
    status =
        stm32_gcm_append_record(
            &next);
    if (status != STM32_GCM_NVM_OK)
    {
        return status;
    }
    s_tx_session_reserved_this_boot =
        true;
    s_tx_session_id_this_boot =
        next.tx_session_id;
    *session_id_out =
        next.tx_session_id;
    return STM32_GCM_NVM_OK;
}
/* Checks an authenticated receive nonce against replay state. */
stm32_gcm_nvm_replay_result_t stm32_gcm_nvm_check_and_commit_rx(
    uint32_t session_id,
    uint64_t counter)
{
    /* Track the accepted receive session and counter in RAM. */
    stm32_gcm_nvm_status_t status;
    if ((session_id == 0U) ||
        (counter == UINT64_C(0)))
    {
        return STM32_GCM_REPLAY_BAD_NONCE;
    }
    status =
        stm32_gcm_nvm_init();
    if (status != STM32_GCM_NVM_OK)
    {
        return STM32_GCM_REPLAY_STORAGE_ERROR;
    }
    if (s_rx_session_id != 0U)
    {
        if (session_id <
            s_rx_session_id)
        {
            return STM32_GCM_REPLAY_REJECT_OLD;
        }
        if (session_id ==
            s_rx_session_id)
        {
            if (counter <
                s_rx_counter)
            {
                return STM32_GCM_REPLAY_REJECT_OLD;
            }
            if (counter ==
                s_rx_counter)
            {
                return STM32_GCM_REPLAY_DUPLICATE_LAST;
            }
        }
    }
    s_rx_session_id =
        session_id;
    s_rx_counter =
        counter;
    return STM32_GCM_REPLAY_ACCEPT_NEW;
}
/* Returns the current GCM transmit session. */
uint32_t stm32_gcm_nvm_current_tx_session(void)
{
    /* Return zero when security-state initialization fails. */
    if (stm32_gcm_nvm_init() != STM32_GCM_NVM_OK)
    {
        return 0U;
    }
    return s_state.tx_session_id;
}
/* Returns the tracked GCM receive session. */
uint32_t stm32_gcm_nvm_current_rx_session(void)
{
    /* Require successful state initialization before reading the session. */
    if (stm32_gcm_nvm_init() != STM32_GCM_NVM_OK)
    {
        return 0U;
    }
    return s_rx_session_id;
}
/* Returns the tracked GCM receive counter. */
uint64_t stm32_gcm_nvm_current_rx_counter(void)
{
    /* Use zero to indicate unavailable receive state. */
    if (stm32_gcm_nvm_init() != STM32_GCM_NVM_OK)
    {
        return UINT64_C(0);
    }
    return s_rx_counter;
}
/* Reports the configured GCM storage capacity remaining. */
uint32_t stm32_gcm_nvm_remaining_record_slots(void)
{
    /* Return no capacity when state initialization fails. */
    const uint32_t sector_end =
        STM32_GCM_NVM_FLASH_BASE +
        STM32_GCM_NVM_FLASH_SIZE;
    if (stm32_gcm_nvm_init() != STM32_GCM_NVM_OK)
    {
        return 0U;
    }
    if (s_next_address >= sector_end)
    {
        return 0U;
    }
    return
        (sector_end - s_next_address) /
        sizeof(stm32_gcm_nvm_record_t);
}
