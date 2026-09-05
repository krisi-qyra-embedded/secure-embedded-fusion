/* Author: i fiksuari pas prog
 * Documentation date: 2026-09-05
 */

#include "public_key_fault_injection_patch.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "uart.h"
#include "systick.h"

/*
 * Corrupt ONLY:
 *
 *     PUBLIC KEY
 *     0x08060070 - 0x0806008F
 *
 * No Sector-7 erase is performed.
 *
 * This is important because Sector 7 also contains your manifest.
 *
 * STM32 Flash can program:
 *
 *     1 -> 0
 *
 * without erasing the sector.
 *
 * It cannot program:
 *
 *     0 -> 1
 *
 * without erasing first.
 *
 * Therefore the fake public key is derived by ANDing the currently
 * provisioned public key with fixed masks. This guarantees that this
 * fault-injection operation only clears bits.
 */
/* Corrupts the stored boot public key for an authenticity test. */
bool fault_inject_corrupt_stored_boot_public_key(void)
{
    /* Clear key bits without erasing the sector, then verify every programmed word. */
    static const uint32_t fake_masks[TEST_PUBLIC_KEY_WORDS] =
    {
        UINT32_C(0xA5F3C96D),
        UINT32_C(0x6C5A39B7),
        UINT32_C(0xD3A56F19),
        UINT32_C(0x5AA5C33C),

        UINT32_C(0x9C63A5D2),
        UINT32_C(0x3AD596C5),
        UINT32_C(0xC5A36F92),
        UINT32_C(0x69D3A5C6)
    };

    const volatile uint32_t *const flash_key =
        (const volatile uint32_t *)TEST_PUBLIC_KEY_FLASH_ADDR;

    uint32_t original[TEST_PUBLIC_KEY_WORDS];
    uint32_t fake[TEST_PUBLIC_KEY_WORDS];

    bool changed = false;
    bool fake_has_nonzero = false;

    /*
     * Read the currently stored Ed25519 public key and create
     * a different key using only 1 -> 0 transitions.
     */
    for (uint32_t i = 0U;
         i < TEST_PUBLIC_KEY_WORDS;
         ++i)
    {
        original[i] = flash_key[i];

        fake[i] =
            original[i] &
            fake_masks[i];

        if (fake[i] != original[i])
        {
            changed = true;
        }

        if (fake[i] != 0U)
        {
            fake_has_nonzero = true;
        }
    }

    if (!fake_has_nonzero)
    {
        log_printf(
            "[FAULT_INJECT][AUTH][ERROR] "
            "Fake public key would become all zero; aborting\r\n");

        return false;
    }

    if (!changed)
    {
        log_printf(
            "[FAULT_INJECT][AUTH] "
            "Public-key corruption already present; "
            "no additional Flash write required\r\n");

        return true;
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        log_printf(
            "[FAULT_INJECT][AUTH][ERROR] "
            "HAL_FLASH_Unlock failed\r\n");

        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(
        FLASH_FLAG_EOP    |
        FLASH_FLAG_OPERR  |
        FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR |
        FLASH_FLAG_PGPERR |
        FLASH_FLAG_PGSERR
    );

    for (uint32_t i = 0U;
         i < TEST_PUBLIC_KEY_WORDS;
         ++i)
    {
        if (fake[i] == original[i])
        {
            continue;
        }

        const uint32_t address =
            TEST_PUBLIC_KEY_FLASH_ADDR +
            (i * (uint32_t)sizeof(uint32_t));

        if (HAL_FLASH_Program(
                FLASH_TYPEPROGRAM_WORD,
                address,
                (uint64_t)fake[i]) != HAL_OK)
        {
            const uint32_t flash_error =
                HAL_FLASH_GetError();

            (void)HAL_FLASH_Lock();

            log_printf(
                "[FAULT_INJECT][AUTH][ERROR] "
                "Public-key Flash write failed | "
                "addr=0x%08lX error=0x%08lX\r\n",
                (unsigned long)address,
                (unsigned long)flash_error);

            return false;
        }
    }

    (void)HAL_FLASH_Lock();

    __DSB();
    __ISB();

    for (uint32_t i = 0U;
         i < TEST_PUBLIC_KEY_WORDS;
         ++i)
    {
        if (flash_key[i] != fake[i])
        {
            log_printf(
                "[FAULT_INJECT][AUTH][ERROR] "
                "Public-key readback mismatch | "
                "word=%lu addr=0x%08lX\r\n",
                (unsigned long)i,
                (unsigned long)(
                    TEST_PUBLIC_KEY_FLASH_ADDR +
                    (i * (uint32_t)sizeof(uint32_t))));

            return false;
        }
    }

    log_printf(
        "[FAULT_INJECT][AUTH] "
        "Stored Ed25519 public key corrupted successfully\r\n");

    log_printf(
        "[FAULT_INJECT][AUTH] "
        "Modified region ONLY: "
        "0x08060070-0x0806008F\r\n");

    log_printf(
        "[FAULT_INJECT][AUTH] "
        "Firmware and signed manifest were NOT modified\r\n");

    log_printf(
        "[FAULT_INJECT][AUTH] "
        "Expected next boot: "
        "AUTHENTICITY=FAIL | INTEGRITY=PASS\r\n");

    return true;
}
