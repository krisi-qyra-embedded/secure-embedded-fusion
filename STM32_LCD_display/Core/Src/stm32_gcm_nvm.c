/**
 * @file stm32_gcm_nvm.c
 * @brief Flash journal for AES-GCM session state and RAM replay tracking.
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
/* Latest valid persisted journal state. */
static stm32_gcm_nvm_record_t s_state;
/* Next free record slot found during journal scan. */
static uint32_t s_next_address;
static bool s_initialized;
/* ESP32 replay high-water is intentionally RAM-only. */
static uint32_t s_rx_session_id;
static uint64_t s_rx_counter;
/* Prevent more than one TX-session Flash append per MCU boot. */
static bool s_tx_session_reserved_this_boot;
static uint32_t s_tx_session_id_this_boot;
/* Calculate CRC32 for one journal record region. */
static uint32_t stm32_gcm_crc32(
    const uint8_t *data,
    size_t length)
{
    uint32_t crc = UINT32_C(0xFFFFFFFF);
    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }
    /* Standard reflected CRC32 over the selected record bytes. */
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
/* Check whether a journal slot is still fully erased. */
static bool stm32_gcm_record_erased(
    const stm32_gcm_nvm_record_t *record)
{
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
/* Validate record header, commit marker and CRC. */
static bool stm32_gcm_record_valid(
    const stm32_gcm_nvm_record_t *record)
{
    uint32_t expected_crc;
    if (record == NULL)
    {
        return false;
    }
    /* Header and commit marker must match before CRC is trusted. */
    if ((record->magic != STM32_GCM_NVM_MAGIC) ||
        (record->version != STM32_GCM_NVM_VERSION) ||
        (record->commit != STM32_GCM_NVM_COMMIT))
    {
        return false;
    }
    /* CRC stops before the crc32 and commit fields. */
    expected_crc =
        stm32_gcm_crc32(
            (const uint8_t *)record,
            offsetof(
                stm32_gcm_nvm_record_t,
                crc32));
    return record->crc32 == expected_crc;
}
/* Clear the STM32 Flash status flags used by programming. */
static void stm32_gcm_clear_flash_flags(void)
{
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
/* Append one committed record to the Flash journal. */
static stm32_gcm_nvm_status_t stm32_gcm_append_record(
    const stm32_gcm_nvm_record_t *candidate)
{
    stm32_gcm_nvm_record_t record;
    const uint32_t *words;
    const stm32_gcm_nvm_record_t *slot;
    /* Journal never writes past the configured Flash range. */
    const uint32_t sector_end =
        STM32_GCM_NVM_FLASH_BASE +
        STM32_GCM_NVM_FLASH_SIZE;
    HAL_StatusTypeDef hal_status;
    bool unlocked = false;
    if (candidate == NULL)
    {
        return STM32_GCM_NVM_ERR_PARAM;
    }
    /* Appends require an initialized journal and one full free slot. */
    if ((!s_initialized) ||
        ((s_next_address + sizeof(record)) > sector_end))
    {
        return STM32_GCM_NVM_ERR_FULL;
    }
    /* Never program over a slot that is not fully erased. */
    slot =
        (const stm32_gcm_nvm_record_t *)s_next_address;
    if (!stm32_gcm_record_erased(slot))
    {
        return STM32_GCM_NVM_ERR_FULL;
    }
    /* Build the final record in RAM before touching Flash. */
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
    /* Flash is unlocked only for this one append operation. */
    hal_status =
        HAL_FLASH_Unlock();
    if (hal_status != HAL_OK)
    {
        return STM32_GCM_NVM_ERR_FLASH;
    }
    unlocked = true;
    stm32_gcm_clear_flash_flags();
    /* Program all data words first and leave commit for the final write. */
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
    /* Commit word is last so torn writes stay invalid after reset. */
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
    /* Always lock Flash again before returning. */
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
    /* Read back and validate the record before accepting it as state. */
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
    /* Advance in-RAM state only after Flash verification succeeds. */
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
/**
 * @brief Scan the GCM Flash journal and restore the latest valid state.
 * @return NVM status.
 */
stm32_gcm_nvm_status_t stm32_gcm_nvm_init(void)
{
    const uint32_t sector_end =
        STM32_GCM_NVM_FLASH_BASE +
        STM32_GCM_NVM_FLASH_SIZE;
    uint32_t address;
    uint32_t last_used_end =
        STM32_GCM_NVM_FLASH_BASE;
    bool any_programmed = false;
    bool any_valid = false;
    /* Repeated init calls reuse the result of the first journal scan. */
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
    /* Start with empty RAM state before scanning Flash. */
    memset(
        &s_state,
        0,
        sizeof(s_state));
    s_rx_session_id = 0U;
    s_rx_counter = UINT64_C(0);
    s_tx_session_reserved_this_boot = false;
    s_tx_session_id_this_boot = 0U;
    /* Scan every fixed-size slot and keep the newest valid generation. */
    for (address = STM32_GCM_NVM_FLASH_BASE;
         (address + sizeof(stm32_gcm_nvm_record_t)) <= sector_end;
         address += sizeof(stm32_gcm_nvm_record_t))
    {
        const stm32_gcm_nvm_record_t *record =
            (const stm32_gcm_nvm_record_t *)address;
        /* Erased slots are free and do not affect recovery state. */
        if (stm32_gcm_record_erased(record))
        {
            continue;
        }
        any_programmed = true;
        last_used_end =
            address +
            sizeof(stm32_gcm_nvm_record_t);
        /* Ignore incomplete/corrupt entries while looking for a valid state. */
        if (!stm32_gcm_record_valid(record))
        {
            continue;
        }
        /* Highest generation becomes the restored state. */
        if ((!any_valid) ||
            (record->generation >
                s_state.generation))
        {
            s_state =
                *record;
            any_valid = true;
        }
    }
    /* Programmed Flash with no valid record is treated as corruption. */
    if (any_programmed && !any_valid)
    {
        memset(
            &s_state,
            0,
            sizeof(s_state));
        return STM32_GCM_NVM_ERR_CORRUPT;
    }
    /* Append after the highest programmed slot, not after the latest valid one. */
    s_next_address =
        last_used_end;
    s_initialized =
        true;
    return STM32_GCM_NVM_OK;
}
/**
 * @brief Reserve the STM32 TX session for this boot.
 * @param session_id_out Reserved session ID.
 * @return NVM status.
 */
stm32_gcm_nvm_status_t stm32_gcm_nvm_reserve_tx_session(
    uint32_t *session_id_out)
{
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
    /* Same-boot repeated calls reuse the already reserved session. */
    if (s_tx_session_reserved_this_boot)
    {
        *session_id_out =
            s_tx_session_id_this_boot;
        return STM32_GCM_NVM_OK;
    }
    /* New boot session is monotonic relative to the persisted value. */
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
    /* Refuse wraparound of either session ID or generation. */
    if ((previous_session == UINT32_MAX) ||
        (s_state.generation == UINT32_MAX))
    {
        return STM32_GCM_NVM_ERR_EXHAUSTED;
    }
    /* Clone the last state and advance one generation/session. */
    next =
        s_state;
    next.generation++;
    next.tx_session_id =
        previous_session + 1U;
    /* RX fields remain zero in Flash because replay state is RAM-only. */
    next.rx_session_id = 0U;
    next.rx_counter_hi = 0U;
    next.rx_counter_lo = 0U;
    /* This is the single normal Flash append for the current boot. */
    status =
        stm32_gcm_append_record(
            &next);
    if (status != STM32_GCM_NVM_OK)
    {
        return status;
    }
    /* Publish the reserved session only after the append succeeds. */
    s_tx_session_reserved_this_boot =
        true;
    s_tx_session_id_this_boot =
        next.tx_session_id;
    *session_id_out =
        next.tx_session_id;
    return STM32_GCM_NVM_OK;
}
/**
 * @brief Check and update the RAM-only RX replay high-water mark.
 * @param session_id Incoming ESP32 session ID.
 * @param counter Incoming packet counter.
 * @return Replay decision.
 */
stm32_gcm_nvm_replay_result_t stm32_gcm_nvm_check_and_commit_rx(
    uint32_t session_id,
    uint64_t counter)
{
    stm32_gcm_nvm_status_t status;
    /* Zero session/counter values are not valid replay nonces. */
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
    /* Reject older sessions and non-increasing counters in the same session. */
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
    /* Accepted packet becomes the new RAM high-water mark. */
    s_rx_session_id =
        session_id;
    s_rx_counter =
        counter;
    return STM32_GCM_REPLAY_ACCEPT_NEW;
}
/**
 * @brief Get the current persisted STM32 TX session.
 * @return Current TX session ID, or 0 on storage error.
 */
uint32_t stm32_gcm_nvm_current_tx_session(void)
{
    if (stm32_gcm_nvm_init() != STM32_GCM_NVM_OK)
    {
        return 0U;
    }
    return s_state.tx_session_id;
}
/**
 * @brief Get the current RAM-only RX session.
 * @return Current RX session ID, or 0 on storage error.
 */
uint32_t stm32_gcm_nvm_current_rx_session(void)
{
    if (stm32_gcm_nvm_init() != STM32_GCM_NVM_OK)
    {
        return 0U;
    }
    return s_rx_session_id;
}
/**
 * @brief Get the current RAM-only RX packet counter.
 * @return Current RX counter, or 0 on storage error.
 */
uint64_t stm32_gcm_nvm_current_rx_counter(void)
{
    if (stm32_gcm_nvm_init() != STM32_GCM_NVM_OK)
    {
        return UINT64_C(0);
    }
    return s_rx_counter;
}
/**
 * @brief Get the number of unused Flash journal slots.
 * @return Remaining record count.
 */
uint32_t stm32_gcm_nvm_remaining_record_slots(void)
{
    const uint32_t sector_end =
        STM32_GCM_NVM_FLASH_BASE +
        STM32_GCM_NVM_FLASH_SIZE;
    if (stm32_gcm_nvm_init() != STM32_GCM_NVM_OK)
    {
        return 0U;
    }
    /* No complete record fits once the next address reaches sector end. */
    if (s_next_address >= sector_end)
    {
        return 0U;
    }
    return
        (sector_end - s_next_address) /
        sizeof(stm32_gcm_nvm_record_t);
}
