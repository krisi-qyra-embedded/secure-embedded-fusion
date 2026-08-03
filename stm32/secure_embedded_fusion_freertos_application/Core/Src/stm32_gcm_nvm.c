/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

/*
 * stm32_gcm_nvm.c
 *
 * STM32F411 AES-GCM session/replay state using RTC backup registers.
 *
 * NO FLASH IS READ, PROGRAMMED OR ERASED BY THIS FILE.
 *
 * Reserved registers:
 *   slot A = BKP4R  .. BKP11R
 *   slot B = BKP12R .. BKP19R
 *
 * Each slot is eight 32-bit words:
 *   [0] magic
 *   [1] generation
 *   [2] tx_session_id
 *   [3] rx_session_id
 *   [4] rx_counter_hi
 *   [5] rx_counter_lo
 *   [6] crc32(words 0..5)
 *   [7] commit
 *
 * The inactive slot is written first and commit is written last. RAM copies
 * are zeroized before return; only the small retained backup-register state
 * remains available for future replay/session checks.
 */

#include "stm32_gcm_nvm.h"

#include <stddef.h>
#include <string.h>

#define STM32_GCM_BKP_MAGIC      UINT32_C(0x47434D42) /* 'GCMB' */
#define STM32_GCM_BKP_COMMIT     UINT32_C(0xA55A3CC3)
#define STM32_GCM_BKP_NO_SLOT    UINT32_C(0xFFFFFFFF)

typedef struct
{
    uint32_t magic;
    uint32_t generation;
    uint32_t tx_session_id;
    uint32_t rx_session_id;
    uint32_t rx_counter_hi;
    uint32_t rx_counter_lo;
    uint32_t crc32;
    uint32_t commit;

} stm32_gcm_bkp_record_t;

typedef char stm32_gcm_bkp_record_size_check[
    (sizeof(stm32_gcm_bkp_record_t) ==
     (STM32_GCM_BKP_REGS_PER_SLOT * sizeof(uint32_t))) ? 1 : -1];

/* Clears temporary GCM storage state with volatile writes. */
static void stm32_gcm_secure_zero(void *ptr, size_t len)
{
    /* Keep the byte stores observable under optimization. */
    volatile uint8_t *p = (volatile uint8_t *)ptr;

    if (p == NULL)
    {
        return;
    }

    while (len-- != 0U)
    {
        *p++ = 0U;
    }
}

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

/* Decodes the receive counter from its stored word pair. */
static uint64_t stm32_gcm_record_rx_counter(
    const stm32_gcm_bkp_record_t *record)
{
    /* Combine the high and low words into the 64-bit counter. */
    if (record == NULL)
    {
        return UINT64_C(0);
    }

    return
        ((uint64_t)record->rx_counter_hi << 32U) |
        (uint64_t)record->rx_counter_lo;
}

/* Encodes the receive counter into its record word pair. */
static void stm32_gcm_record_set_rx_counter(
    stm32_gcm_bkp_record_t *record,
    uint64_t counter)
{
    /* Store the upper and lower 32-bit halves separately. */
    if (record == NULL)
    {
        return;
    }

    record->rx_counter_hi =
        (uint32_t)(counter >> 32U);

    record->rx_counter_lo =
        (uint32_t)(counter & UINT64_C(0xFFFFFFFF));
}

/* Returns the RTC backup-register array base. */
static volatile uint32_t *stm32_gcm_bkp_base(void)
{
    /* Index the retained registers starting at BKP0R. */
    return &RTC->BKP0R;
}

/* Reads one RTC backup-register word. */
static uint32_t stm32_gcm_bkp_reg_read(uint32_t reg_index)
{
    /* Use the register index within the configured backup area. */
    volatile uint32_t *const bkp =
        stm32_gcm_bkp_base();

    return bkp[reg_index];
}

/* Writes one RTC backup-register word. */
static void stm32_gcm_bkp_reg_write(
    uint32_t reg_index,
    uint32_t value)
{
    /* Complete the retained-register write before proceeding. */
    volatile uint32_t *const bkp =
        stm32_gcm_bkp_base();

    bkp[reg_index] = value;
    __DSB();
}

/* Calculates a GCM record's backup-register slot offset. */
static uint32_t stm32_gcm_bkp_slot_base(uint32_t slot)
{
    /* Keep each record within its reserved group of eight registers. */
    return
        STM32_GCM_BKP_FIRST_REG +
        (slot * STM32_GCM_BKP_REGS_PER_SLOT);
}

/* Validates a committed RTC GCM state record. */
static bool stm32_gcm_bkp_record_valid(
    const stm32_gcm_bkp_record_t *record)
{
    /* Require the expected magic and commit marker with a matching CRC. */
    uint32_t expected_crc;

    if (record == NULL)
    {
        return false;
    }

    if ((record->magic != STM32_GCM_BKP_MAGIC) ||
        (record->commit != STM32_GCM_BKP_COMMIT))
    {
        return false;
    }

    expected_crc =
        stm32_gcm_crc32(
            (const uint8_t *)record,
            offsetof(stm32_gcm_bkp_record_t, crc32));

    return record->crc32 == expected_crc;
}

/* Checks whether a backup record belongs to GCM storage. */
static bool stm32_gcm_bkp_record_looks_owned(
    const stm32_gcm_bkp_record_t *record)
{
    /* Recognize either its magic or commit marker as prior ownership. */
    if (record == NULL)
    {
        return false;
    }

    return
        (record->magic == STM32_GCM_BKP_MAGIC) ||
        (record->commit == STM32_GCM_BKP_COMMIT);
}

/* Reads one RTC GCM state slot into RAM. */
static void stm32_gcm_bkp_read_slot(
    uint32_t slot,
    stm32_gcm_bkp_record_t *record)
{
    /* Reject an invalid slot index before reading its eight words. */
    uint32_t *words;
    const uint32_t base =
        stm32_gcm_bkp_slot_base(slot);

    if ((slot >= STM32_GCM_BKP_SLOT_COUNT) ||
        (record == NULL))
    {
        return;
    }

    words = (uint32_t *)record;

    for (uint32_t i = 0U;
         i < STM32_GCM_BKP_REGS_PER_SLOT;
         ++i)
    {
        words[i] =
            stm32_gcm_bkp_reg_read(base + i);
    }
}

/* Enables RTC backup-register writes while retaining prior settings. */
static void stm32_gcm_backup_access_begin(
    uint32_t *saved_apb1enr,
    uint32_t *saved_pwr_cr)
{
    /* Save the power clock and DBP state before granting access. */
    if ((saved_apb1enr == NULL) ||
        (saved_pwr_cr == NULL))
    {
        return;
    }

    *saved_apb1enr = RCC->APB1ENR;

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();

    *saved_pwr_cr = PWR->CR;

    PWR->CR |= PWR_CR_DBP;
    __DSB();
}

/* Restores the previous RTC backup-write access settings. */
static void stm32_gcm_backup_access_end(
    uint32_t saved_apb1enr,
    uint32_t saved_pwr_cr)
{
    /* Clear DBP and the power clock only if they were previously disabled. */
    if ((saved_pwr_cr & PWR_CR_DBP) == 0U)
    {
        PWR->CR &= ~PWR_CR_DBP;
        __DSB();
    }

    if ((saved_apb1enr & RCC_APB1ENR_PWREN) == 0U)
    {
        RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
        __DSB();
    }
}

/* Selects the newest valid RTC GCM state record. */
static stm32_gcm_nvm_status_t stm32_gcm_bkp_load_newest(
    stm32_gcm_bkp_record_t *record_out,
    uint32_t *slot_out)
{
    /* Report corruption when owned records exist but neither validates. */
    stm32_gcm_bkp_record_t slot0;
    stm32_gcm_bkp_record_t slot1;
    bool valid0;
    bool valid1;
    bool owned0;
    bool owned1;

    if ((record_out == NULL) ||
        (slot_out == NULL))
    {
        return STM32_GCM_NVM_ERR_PARAM;
    }

    stm32_gcm_secure_zero(&slot0, sizeof(slot0));
    stm32_gcm_secure_zero(&slot1, sizeof(slot1));
    stm32_gcm_secure_zero(record_out, sizeof(*record_out));
    *slot_out = STM32_GCM_BKP_NO_SLOT;

    stm32_gcm_bkp_read_slot(0U, &slot0);
    stm32_gcm_bkp_read_slot(1U, &slot1);

    valid0 = stm32_gcm_bkp_record_valid(&slot0);
    valid1 = stm32_gcm_bkp_record_valid(&slot1);
    owned0 = stm32_gcm_bkp_record_looks_owned(&slot0);
    owned1 = stm32_gcm_bkp_record_looks_owned(&slot1);

    if (valid0 && valid1)
    {
        if (slot1.generation > slot0.generation)
        {
            *record_out = slot1;
            *slot_out = 1U;
        }
        else
        {
            *record_out = slot0;
            *slot_out = 0U;
        }
    }
    else if (valid0)
    {
        *record_out = slot0;
        *slot_out = 0U;
    }
    else if (valid1)
    {
        *record_out = slot1;
        *slot_out = 1U;
    }
    else if (owned0 || owned1)
    {
        stm32_gcm_secure_zero(&slot0, sizeof(slot0));
        stm32_gcm_secure_zero(&slot1, sizeof(slot1));
        return STM32_GCM_NVM_ERR_CORRUPT;
    }
    else
    {
        stm32_gcm_secure_zero(record_out, sizeof(*record_out));
        *slot_out = STM32_GCM_BKP_NO_SLOT;
    }

    stm32_gcm_secure_zero(&slot0, sizeof(slot0));
    stm32_gcm_secure_zero(&slot1, sizeof(slot1));

    return STM32_GCM_NVM_OK;
}

/* Commits and verifies GCM state in the alternate RTC slot. */
static stm32_gcm_nvm_status_t stm32_gcm_bkp_commit(
    const stm32_gcm_bkp_record_t *candidate,
    uint32_t current_slot)
{
    /* Invalidate the target commit word before writing the new record. */
    stm32_gcm_bkp_record_t record;
    stm32_gcm_bkp_record_t verify;
    uint32_t target_slot;
    uint32_t base;
    const uint32_t *words;
    uint32_t saved_apb1enr;
    uint32_t saved_pwr_cr;
    uint32_t primask;

    if (candidate == NULL)
    {
        return STM32_GCM_NVM_ERR_PARAM;
    }

    record = *candidate;
    record.magic = STM32_GCM_BKP_MAGIC;
    record.crc32 =
        stm32_gcm_crc32(
            (const uint8_t *)&record,
            offsetof(stm32_gcm_bkp_record_t, crc32));
    record.commit = STM32_GCM_BKP_COMMIT;

    if (current_slot == 0U)
    {
        target_slot = 1U;
    }
    else
    {
        target_slot = 0U;
    }

    base = stm32_gcm_bkp_slot_base(target_slot);
    words = (const uint32_t *)&record;

    primask = __get_PRIMASK();
    __disable_irq();

    stm32_gcm_backup_access_begin(
        &saved_apb1enr,
        &saved_pwr_cr);

    stm32_gcm_bkp_reg_write(base + 7U, 0U);

    for (uint32_t i = 0U; i < 7U; ++i)
    {
        stm32_gcm_bkp_reg_write(
            base + i,
            words[i]);
    }

    stm32_gcm_bkp_reg_write(
        base + 7U,
        words[7U]);

    stm32_gcm_secure_zero(&verify, sizeof(verify));
    stm32_gcm_bkp_read_slot(target_slot, &verify);

    stm32_gcm_backup_access_end(
        saved_apb1enr,
        saved_pwr_cr);

    if (primask == 0U)
    {
        __enable_irq();
    }

    if ((!stm32_gcm_bkp_record_valid(&verify)) ||
        (memcmp(&verify, &record, sizeof(record)) != 0))
    {
        stm32_gcm_secure_zero(&record, sizeof(record));
        stm32_gcm_secure_zero(&verify, sizeof(verify));
        return STM32_GCM_NVM_ERR_FLASH;
    }

    stm32_gcm_secure_zero(&record, sizeof(record));
    stm32_gcm_secure_zero(&verify, sizeof(verify));

    return STM32_GCM_NVM_OK;
}

/* Loads the configured GCM security-state storage. */
stm32_gcm_nvm_status_t stm32_gcm_nvm_init(void)
{
    /* Recover the newest valid backup-register slot before using security state. */
    stm32_gcm_bkp_record_t current;
    uint32_t slot;
    stm32_gcm_nvm_status_t status;

    stm32_gcm_secure_zero(&current, sizeof(current));

    status =
        stm32_gcm_bkp_load_newest(
            &current,
            &slot);

    stm32_gcm_secure_zero(&current, sizeof(current));
    slot = 0U;

    return status;
}

/* Reserves the transmit session identifier for this boot. */
stm32_gcm_nvm_status_t stm32_gcm_nvm_reserve_tx_session(
    uint32_t *session_id_out)
{
    /* Reject session or generation exhaustion before advancing state. */
    stm32_gcm_bkp_record_t current;
    stm32_gcm_bkp_record_t next;
    uint32_t current_slot;
    stm32_gcm_nvm_status_t status;

    if (session_id_out == NULL)
    {
        return STM32_GCM_NVM_ERR_PARAM;
    }

    *session_id_out = 0U;
    stm32_gcm_secure_zero(&current, sizeof(current));
    stm32_gcm_secure_zero(&next, sizeof(next));

    status =
        stm32_gcm_bkp_load_newest(
            &current,
            &current_slot);

    if (status != STM32_GCM_NVM_OK)
    {
        goto cleanup;
    }

    if ((current.tx_session_id == UINT32_MAX) ||
        (current.generation == UINT32_MAX))
    {
        status = STM32_GCM_NVM_ERR_EXHAUSTED;
        goto cleanup;
    }

    next = current;
    next.generation++;
    next.tx_session_id++;

    status =
        stm32_gcm_bkp_commit(
            &next,
            current_slot);

    if (status == STM32_GCM_NVM_OK)
    {
        *session_id_out =
            next.tx_session_id;
    }

cleanup:
    stm32_gcm_secure_zero(&current, sizeof(current));
    stm32_gcm_secure_zero(&next, sizeof(next));
    current_slot = 0U;

    return status;
}

/* Checks an authenticated receive nonce against replay state. */
stm32_gcm_nvm_replay_result_t stm32_gcm_nvm_check_and_commit_rx(
    uint32_t session_id,
    uint64_t counter)
{
    /* Commit authenticated receive state before accepting the newer nonce. */
    stm32_gcm_bkp_record_t current;
    stm32_gcm_bkp_record_t next;
    uint32_t current_slot;
    uint64_t highest_counter;
    stm32_gcm_nvm_status_t status;
    stm32_gcm_nvm_replay_result_t result;

    if ((session_id == 0U) ||
        (counter == UINT64_C(0)))
    {
        return STM32_GCM_REPLAY_BAD_NONCE;
    }

    stm32_gcm_secure_zero(&current, sizeof(current));
    stm32_gcm_secure_zero(&next, sizeof(next));

    status =
        stm32_gcm_bkp_load_newest(
            &current,
            &current_slot);

    if (status != STM32_GCM_NVM_OK)
    {
        result = STM32_GCM_REPLAY_STORAGE_ERROR;
        goto cleanup;
    }

    highest_counter =
        stm32_gcm_record_rx_counter(
            &current);

    if (current.rx_session_id != 0U)
    {
        if (session_id < current.rx_session_id)
        {
            result = STM32_GCM_REPLAY_REJECT_OLD;
            goto cleanup;
        }

        if (session_id == current.rx_session_id)
        {
            if (counter < highest_counter)
            {
                result = STM32_GCM_REPLAY_REJECT_OLD;
                goto cleanup;
            }

            if (counter == highest_counter)
            {
                result = STM32_GCM_REPLAY_DUPLICATE_LAST;
                goto cleanup;
            }
        }
    }

    if (current.generation == UINT32_MAX)
    {
        result = STM32_GCM_REPLAY_STORAGE_ERROR;
        goto cleanup;
    }

    next = current;
    next.generation++;
    next.rx_session_id = session_id;
    stm32_gcm_record_set_rx_counter(&next, counter);

    status =
        stm32_gcm_bkp_commit(
            &next,
            current_slot);

    if (status != STM32_GCM_NVM_OK)
    {
        result = STM32_GCM_REPLAY_STORAGE_ERROR;
        goto cleanup;
    }

    result = STM32_GCM_REPLAY_ACCEPT_NEW;

cleanup:
    stm32_gcm_secure_zero(&current, sizeof(current));
    stm32_gcm_secure_zero(&next, sizeof(next));
    highest_counter = UINT64_C(0);
    current_slot = 0U;

    return result;
}

/* Returns the current GCM transmit session. */
uint32_t stm32_gcm_nvm_current_tx_session(void)
{
    /* Return zero when security-state initialization fails. */
    stm32_gcm_bkp_record_t current;
    uint32_t slot;
    uint32_t value = 0U;

    stm32_gcm_secure_zero(&current, sizeof(current));

    if (stm32_gcm_bkp_load_newest(&current, &slot) ==
        STM32_GCM_NVM_OK)
    {
        value = current.tx_session_id;
    }

    stm32_gcm_secure_zero(&current, sizeof(current));
    slot = 0U;

    return value;
}

/* Returns the tracked GCM receive session. */
uint32_t stm32_gcm_nvm_current_rx_session(void)
{
    /* Require successful state initialization before reading the session. */
    stm32_gcm_bkp_record_t current;
    uint32_t slot;
    uint32_t value = 0U;

    stm32_gcm_secure_zero(&current, sizeof(current));

    if (stm32_gcm_bkp_load_newest(&current, &slot) ==
        STM32_GCM_NVM_OK)
    {
        value = current.rx_session_id;
    }

    stm32_gcm_secure_zero(&current, sizeof(current));
    slot = 0U;

    return value;
}

/* Returns the tracked GCM receive counter. */
uint64_t stm32_gcm_nvm_current_rx_counter(void)
{
    /* Use zero to indicate unavailable receive state. */
    stm32_gcm_bkp_record_t current;
    uint32_t slot;
    uint64_t value = UINT64_C(0);

    stm32_gcm_secure_zero(&current, sizeof(current));

    if (stm32_gcm_bkp_load_newest(&current, &slot) ==
        STM32_GCM_NVM_OK)
    {
        value =
            stm32_gcm_record_rx_counter(
                &current);
    }

    stm32_gcm_secure_zero(&current, sizeof(current));
    slot = 0U;

    return value;
}

/* Reports the backup-register backend capacity indicator. */
uint32_t stm32_gcm_nvm_remaining_record_slots(void)
{
    /* Return the maximum value because this backend does not consume Flash slots. */
    return UINT32_MAX;
}

/* Clears the reserved RTC GCM state registers. */
stm32_gcm_nvm_status_t stm32_gcm_nvm_clear_security_state(void)
{
    /* Restore backup-write access and the prior interrupt state afterward. */
    uint32_t saved_apb1enr;
    uint32_t saved_pwr_cr;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    stm32_gcm_backup_access_begin(
        &saved_apb1enr,
        &saved_pwr_cr);

    for (uint32_t reg = STM32_GCM_BKP_FIRST_REG;
         reg < (STM32_GCM_BKP_FIRST_REG + STM32_GCM_BKP_TOTAL_REGS);
         ++reg)
    {
        stm32_gcm_bkp_reg_write(reg, 0U);
    }

    stm32_gcm_backup_access_end(
        saved_apb1enr,
        saved_pwr_cr);

    if (primask == 0U)
    {
        __enable_irq();
    }

    return STM32_GCM_NVM_OK;
}
