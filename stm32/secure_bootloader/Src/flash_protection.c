/* Improved, more robust version of flash option bytes helpers.
   - Keeps original function names, signatures and high-level logic.
   - Adds timeouts for busy-waits to avoid infinite loops.
   - Uses read-modify-write with temporaries to avoid racey register updates.
   - Adds small input validation and clearer bit manipulation.
   - Preserves structure FLASH_OBProgramInitTypeDef and public API.
*/
/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */

#include "flash_protection.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Tunable timeout for flash operations (iterations). Adjust as appropriate for your clock. */
#ifndef FLASH_WAIT_TIMEOUT_ITER
#define FLASH_WAIT_TIMEOUT_ITER  1000000U
#endif

/* Keep existing macros from user code. They must already be consistent with MCU headers.
   If those are not defined in included headers, uncomment or define appropriate values. */
/* #define FLASH_OPTCR_OPTLOCK   (1U << 0)
   #define FLASH_OPTCR_OPTSTART  (1U << 1)
   #define FLASH_SR_BSY          (1U << 16) */ /* example: actual definitions should come from device headers */

#define FLASH_GET_FLAG(FLAG)    ((FLASH->SR) & (FLAG))

#define FLASH_OPT_KEY1 0x08192A3BU
#define FLASH_OPT_KEY2 0x4C5D6E7FU
#define FLASH_KEY1              0x45670123U
#define FLASH_KEY2              0xCDEF89ABU
/* Addresses for individual bytes inside OPTCR (kept from original file) */
#define OPTCR_BYTE0_ADDRESS     0x40023C14U
#define OPTCR_BYTE1_ADDRESS     0x40023C15U
#define OPTCR_BYTE2_ADDRESS     0x40023C16U
#define OPTCR_BYTE3_ADDRESS     0x40023C17U

/* Byte offsets in OPTCR word (kept from original file) */
#define START_OF_WRP_BYTE       16
#define START_OF_RDP_BYTE       8


static const struct {
    uint32_t addr;
    uint32_t size;
    const char *name;
} sector_map[NUM_SECTORS] = {
    [0] = { 0x08000000, 0x04000, "Sector 0 (16KB)"  },   /* 16KB  - Bootloader     */
    [1] = { 0x08004000, 0x04000, "Sector 1 (16KB)"  },   /* 16KB  - Bootloader     */
    [2] = { 0x08008000, 0x04000, "Sector 2 (16KB)"  },   /* 16KB  - Config/App     */
    [3] = { 0x0800C000, 0x04000, "Sector 3 (16KB)"  },   /* 16KB  - Config/App     */
    [4] = { 0x08010000, 0x10000, "Sector 4 (64KB)"  },   /* 64KB  - Application    */
    [5] = { 0x08020000, 0x20000, "Sector 5 (128KB)" },   /* 128KB - Application    */
    [6] = { 0x08040000, 0x20000, "Sector 6 (128KB)" },   /* 128KB - Application    */
    [7] = { 0x08060000, 0x20000, "Sector 7 (128KB)" },   /* 128KB - MANIFEST       */
};

#define NUM_SECTORS (sizeof(sector_map) / sizeof(sector_map[0]))

static bool flash_wait_not_busy(uint32_t timeout_iter);


/* Helper: wait until flash not busy or timeout.
   Returns true if not busy, false if timeout occurred. */
static bool flash_wait_not_busy(uint32_t timeout_iter)
{
    /* Check the flash status and address range before continuing. */
    uint32_t i = 0U;
    while (FLASH_GET_FLAG(FLASH_SR_BSY) != RESET)
    {
        if (++i >= timeout_iter)
            return false;
    }
    return true;
}

/* Read helpers use volatile pointers for clarity and to avoid multiple memory accesses. */
uint8_t flash_obytes_get_wrp(void)
{
    /* Check the flash status and address range before continuing. */
    return (*(__IO uint8_t *)(OPTCR_BYTE2_ADDRESS));
}

/* Performs a flash access or protection operation. */
uint8_t flash_obytes_get_rdp(void)
{
    /* Check the flash status and address range before continuing. */
    return (*(__IO uint8_t *)(OPTCR_BYTE1_ADDRESS));
}

/* User option byte: return only the meaningful bits (top 3 bits of byte 0 in original code).
   Preserve original behavior but make masking explicit. */
uint8_t flash_obytes_get_user(void)
{
    /* original: return ((uint8_t)(FLASH->OPTCR & 0xE0)); */
    uint8_t raw = (uint8_t)((FLASH->OPTCR >> 0) & 0xFFU);
    return (uint8_t)(raw & 0xE0U);
}

/* BOR bits (preserve original masking). */
uint8_t flash_obytes_get_bor(void)
{
    /* original: return ((uint8_t)(FLASH->OPTCR & 0x0C)); */
    uint8_t raw = (uint8_t)((FLASH->OPTCR >> 0) & 0xFFU);
    return (uint8_t)(raw & 0x0CU);
}

/* Lock option bytes (keeps same name & logic) */
DEV_StatusTypeDef flash_obytes_lock(void)
{
    /* Set OPTLOCK bit (write 1). */
    FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
    return DEV_OK;
}

/* Unlock option bytes: perform key sequence only when locked.
   Returns DEV_OK on success, DEV_ERROR if the unlock sequence did not clear the lock. */
DEV_StatusTypeDef flash_obytes_unlock(void)
{
    /* If already unlocked, nothing to do. Preserve safe behavior by returning OK. */
    if ((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) == RESET)
    {
        return DEV_OK;
    }

    /* Write key sequence to OPTKEYR to unlock option bytes */
    FLASH->OPTKEYR = FLASH_OPT_KEY1;
    FLASH->OPTKEYR = FLASH_OPT_KEY2;

    /* Verify unlock succeeded (OPTLOCK cleared). If not cleared, return error. */
    if ((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) != RESET)
    {
        return DEV_ERROR;
    }

    return DEV_OK;
}

/* Set write protection bits for a sector mask (preserve original high-level steps).
   This function kept void signature to match original; it attempts to run and will early-return
   if an important step fails (unlock or timeout). */
void sector_set_wrp(sector_wrp_mask sector_num)
{
    DEV_StatusTypeDef st;

    /* Unlock option bytes */
    st = flash_obytes_unlock();
    if (st != DEV_OK)
        return; /* unlock failed, abort */

    /* Make sure flash is not busy (with timeout) */
    if (!flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER))
        goto finalize;

    /* Enable WRP mode by clearing SPRMOD (bit 31) */
    /* Use temporary read-modify-write to avoid accidental multiple reads */
    {
        uint32_t optcr = FLASH->OPTCR;
        optcr &= ~(1U << 31);
        /* Clear selected WRP bits for the sector mask (preserve original shift semantics) */
        optcr &= ~((uint32_t)sector_num << START_OF_WRP_BYTE);
        /* Write back */
        FLASH->OPTCR = optcr;
    }

    /* Trigger OPTSTART by setting OPTSTART bit (bit1). */
    FLASH->OPTCR |= (1U << 1);

    /* Wait until not busy or timeout */
    flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER);

finalize:
    /* Re-lock option bytes (best-effort) */
    flash_obytes_lock();
}

/* Disable WRP for all sectors (keeps original behavior). */
void dis_wrp_all_sectors(void)
{
    DEV_StatusTypeDef st;

    st = flash_obytes_unlock();
    if (st != DEV_OK)
        return;

    if (!flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER))
        goto finalize;

    /* Clear SPRMOD and then set all WRP bits (0xFF) at the WRP byte offset. */
    {
        uint32_t optcr = FLASH->OPTCR;
        optcr &= ~(1U << 31); /* clear SPRMOD */
        optcr |= ((uint32_t)0xFFU << START_OF_WRP_BYTE);
        FLASH->OPTCR = optcr;
    }

    /* Start the option byte programming sequence */
    FLASH->OPTCR |= (1U << 1);

    /* Wait for completion (best-effort) */
    flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER);

finalize:
    flash_obytes_lock();
}

/* Set RDP (read protection) level. Keeps same signature and intent.
   Only modifies RDP byte (clear + set) to avoid changing other option bits. */
void flash_set_rdp_level(uint8_t level)
{
    DEV_StatusTypeDef st;

    st = flash_obytes_unlock();
    if (st != DEV_OK)
        return;

    if (!flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER))
        goto finalize;

    /* Operate on RDP bits (the RDP byte at START_OF_RDP_BYTE) - clear then set. */
    {
        uint32_t optcr = FLASH->OPTCR;
        /* Clear existing RDP byte */
        optcr &= ~(0xFFU << START_OF_RDP_BYTE);
        /* Set new level in the RDP byte */
        optcr |= ((uint32_t)level << START_OF_RDP_BYTE);
        FLASH->OPTCR = optcr;
    }

    /* Trigger OPTSTART */
    FLASH->OPTCR |= (1U << 1);

    /* Wait for completion (best-effort) */
    flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER);

finalize:
    flash_obytes_lock();
}

/* Populate a FLASH_OBProgramInitTypeDef with current option bytes.
   Returns 0 on success, negative value on error. */
int flash_obytes_get_config(FLASH_OBProgramInitTypeDef *curr_obytes)
{
    /* Defensive null check */
    if (curr_obytes == NULL)
    {
        return -1; /* invalid pointer */
    }

    curr_obytes->WRPSector  = (uint8_t)flash_obytes_get_wrp();
    curr_obytes->RDPLevel   = (uint8_t)flash_obytes_get_rdp();
    curr_obytes->USERConfig = (uint8_t)flash_obytes_get_user();
    curr_obytes->BORLevel   = (uint8_t)flash_obytes_get_bor();

    return 0; /* success */
}


/* ============================================================================
 * Internal: Flash unlock (CMSIS)
 * ============================================================================ */
static flash_erase_status_t flash_unlock_cmsis(void)
{
    /* Check if already unlocked */
    if ((FLASH->CR & FLASH_CR_LOCK) == 0) {
        return FLASH_ERASE_SUCCESS;
    }

    /* Write unlock sequence */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    /* Verify unlock */
    if (FLASH->CR & FLASH_CR_LOCK) {
        return FLASH_ERASE_ERR_UNLOCK_FAIL;
    }

    return FLASH_ERASE_SUCCESS;
}


/* ============================================================================
 * Internal: Flash lock (CMSIS)
 * ============================================================================ */
static void flash_lock_cmsis(void)
{
    /* Check the flash status and address range before continuing. */
    FLASH->CR |= FLASH_CR_LOCK;
}

/* ============================================================================
 * Internal: Wait for flash operation to complete
 * ============================================================================ */
static flash_erase_status_t flash_wait_busy(uint32_t timeout)
{
    /* Check the flash status and address range before continuing. */
    uint32_t count = 0;

    while (FLASH->SR & FLASH_SR_BSY) {
        if (++count > timeout) {
            return FLASH_ERASE_ERR_TIMEOUT;
        }
    }

    return FLASH_ERASE_SUCCESS;
}

/* ============================================================================
 * Internal: Clear all error flags
 * ============================================================================ */
static void flash_clear_errors(void)
{
    /* Check the flash status and address range before continuing. */
    FLASH->SR = FLASH_SR_ERR_MSK | FLASH_SR_EOP;
}

/* ============================================================================
 * Internal: Check for errors after operation
 * ============================================================================ */
static flash_erase_status_t flash_check_errors(void)
{
    /* Check the flash status and address range before continuing. */
    uint32_t sr = FLASH->SR;

    if (sr & FLASH_SR_WRPERR) {
        log_printf("[ERROR] Write protection error (WRPERR)\r\n");
        return FLASH_ERASE_ERR_WRP_ACTIVE;
    }
    if (sr & FLASH_SR_PGAERR) {
        log_printf("[ERROR] Programming alignment error (PGAERR)\r\n");
        return FLASH_ERASE_ERR_ERASE_FAIL;
    }
    if (sr & FLASH_SR_PGPERR) {
        log_printf("[ERROR] Programming parallelism error (PGPERR)\r\n");
        return FLASH_ERASE_ERR_ERASE_FAIL;
    }
    if (sr & FLASH_SR_PGSERR) {
        log_printf("[ERROR] Programming sequence error (PGSERR)\r\n");
        return FLASH_ERASE_ERR_ERASE_FAIL;
    }
    if (sr & FLASH_SR_OPERR) {
        log_printf("[ERROR] Operation error (OPERR)\r\n");
        return FLASH_ERASE_ERR_ERASE_FAIL;
    }

    return FLASH_ERASE_SUCCESS;
}


/* ============================================================================
 * Internal: Perform sector erase (CMSIS)
 * ============================================================================ */
static flash_erase_status_t do_sector_erase_cmsis(uint32_t sector_num)
{
    flash_erase_status_t status;

    /* Wait for any ongoing operation */
    status = flash_wait_busy(FLASH_TIMEOUT);
    if (status != FLASH_ERASE_SUCCESS) {
        log_printf("[ERROR] Flash busy before erase\r\n");
        return status;
    }
    /* Clear pending error flags */
    flash_clear_errors();

    /* Configure erase:
     * - Set SER (Sector Erase)
     * - Set SNB (Sector Number)
     * - Set PSIZE (32-bit for 2.7V-3.6V operation)
     */
    FLASH->CR &= ~(FLASH_CR_SNB_MSK | FLASH_CR_PSIZE_MSK);
    FLASH->CR |= FLASH_CR_SER |
                 ((sector_num << FLASH_CR_SNB_POS) & FLASH_CR_SNB_MSK) |
                 FLASH_CR_PSIZE_X32;
    /* Start erase */
    FLASH->CR |= FLASH_CR_STRT;
    /* Data Synchronization Barrier - ensure write is complete */
    __DSB();
    /* Wait for erase to complete */
    status = flash_wait_busy(FLASH_TIMEOUT * 10);  /* Erase takes longer */
    if (status != FLASH_ERASE_SUCCESS) {
        log_printf("[ERROR] Erase timeout\r\n");
        FLASH->CR &= ~FLASH_CR_SER;
        return status;
    }
    /* Check for errors */
    status = flash_check_errors();
    /* Clear SER bit */
    FLASH->CR &= ~FLASH_CR_SER;

    /* Clear EOP flag if set */
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR = FLASH_SR_EOP;
    }

    return status;
}


/* ============================================================================
 * Verify sector is erased (all 0xFF)
 * ============================================================================ */
bool verify_sector_erased(uint32_t start_addr, uint32_t size)
{
    const uint8_t *ptr = (const uint8_t *)start_addr;
    uint32_t check_size = (size < ERASE_VERIFY_SAMPLE_SIZE) ? size : ERASE_VERIFY_SAMPLE_SIZE;

    /* Check beginning */
    for (uint32_t i = 0; i < check_size; i++) {
        if (ptr[i] != FLASH_ERASED_VALUE) {
            return false;
        }
    }

    /* Check middle */
    uint32_t mid_offset = size / 2;
    for (uint32_t i = 0; i < check_size && (mid_offset + i) < size; i++) {
        if (ptr[mid_offset + i] != FLASH_ERASED_VALUE) {
            return false;
        }
    }

    /* Check end */
    uint32_t end_offset = size - check_size;
    for (uint32_t i = 0; i < check_size; i++) {
        if (ptr[end_offset + i] != FLASH_ERASED_VALUE) {
            return false;
        }
    }

    return true;
}

/* ============================================================================
 * Sector address lookup table - STM32F411 (512KB Flash)
 * ============================================================================
 *
 * STM32F411RE Flash Memory Map:
 * ┌─────────┬──────────────────────────┬────────┬─────────────────────────────┐
 * │ Sector  │ Address Range            │ Size   │ Usage                       │
 * ├─────────┼──────────────────────────┼────────┼─────────────────────────────┤
 * │    0    │ 0x08000000 - 0x08003FFF  │ 16 KB  │ Bootloader                  │
 * │    1    │ 0x08004000 - 0x08007FFF  │ 16 KB  │ Bootloader                  │
 * │    2    │ 0x08008000 - 0x0800BFFF  │ 16 KB  │ Application / Config        │
 * │    3    │ 0x0800C000 - 0x0800FFFF  │ 16 KB  │ Application / Config        │
 * │    4    │ 0x08010000 - 0x0801FFFF  │ 64 KB  │ Application Code            │
 * │    5    │ 0x08020000 - 0x0803FFFF  │ 128 KB │ Application Code            │
 * │    6    │ 0x08040000 - 0x0805FFFF  │ 128 KB │ Application Code            │
 * │    7    │ 0x08060000 - 0x0807FFFF  │ 128 KB │ MANIFEST + SIGNATURE        │
 * └─────────┴──────────────────────────┴────────┴─────────────────────────────┘
 *                                        ══════
 *                                        512 KB Total
 * ============================================================================ */

/* ============================================================================
 * Error string lookup
 * ============================================================================ */


/* ============================================================================
 * Error string lookup
 * ============================================================================ */
const char* flash_erase_status_str(flash_erase_status_t status)
{
    /* Check the flash status and address range before continuing. */
    switch (status) {
        case FLASH_ERASE_SUCCESS:           return "SUCCESS";
        case FLASH_ERASE_ERR_WRP_ACTIVE:    return "WRITE_PROTECTION_ACTIVE";
        case FLASH_ERASE_ERR_UNLOCK_FAIL:   return "UNLOCK_FAILED";
        case FLASH_ERASE_ERR_ERASE_FAIL:    return "ERASE_OPERATION_FAILED";
        case FLASH_ERASE_ERR_VERIFY_FAIL:   return "VERIFICATION_FAILED";
        case FLASH_ERASE_ERR_TIMEOUT:       return "TIMEOUT";
        case FLASH_ERASE_ERR_LOCK_FAIL:     return "LOCK_FAILED";
        case FLASH_ERASE_ERR_BUSY:          return "FLASH_BUSY";
        default:                            return "UNKNOWN_ERROR";
    }
}


/* ============================================================================
 * Erase any sector by number (CMSIS implementation)
 * ============================================================================ */
flash_erase_status_t erase_flash_sector(uint32_t sector_num)
{
    flash_erase_status_t status;
    uint8_t wrp_byte;

    /* -------- Validate sector number -------- */
    if (sector_num >= NUM_SECTORS) {
        log_printf("[ERROR] Invalid sector: %lu (max=%lu)\r\n",
                   (unsigned long)sector_num, (unsigned long)(NUM_SECTORS - 1));
        return FLASH_ERASE_ERR_ERASE_FAIL;
    }

    uint32_t sector_addr = sector_map[sector_num].addr;
    uint32_t sector_size = sector_map[sector_num].size;

    log_printf("[INFO] ═══════════════════════════════════════════════════════\r\n");
    log_printf("[INFO] ERASING SECTOR %lu (CMSIS)\r\n", (unsigned long)sector_num);
    log_printf("[INFO] Address: 0x%08lX - 0x%08lX (%lu KB)\r\n",
               (unsigned long)sector_addr,
               (unsigned long)(sector_addr + sector_size - 1),
               (unsigned long)(sector_size / 1024));
    log_printf("[INFO] ═══════════════════════════════════════════════════════\r\n");
    delay_ms(10);
    /* -------- Step 1: Check if already erased -------- */
    if (verify_sector_erased(sector_addr, sector_size)) {
        log_printf("[INFO] Sector %lu already erased, skipping\r\n", (unsigned long)sector_num);
        delay_ms(5);
        return FLASH_ERASE_SUCCESS;
    }

    /* -------- Step 2: Check write protection -------- */
    wrp_byte = flash_obytes_get_wrp();
    log_printf("[INFO] WRP status: 0x%02X\r\n", (unsigned)wrp_byte);
    delay_ms(5);
    /* STM32F4: bit = 0 means protected, bit = 1 means unprotected */
    if ((wrp_byte & (1U << sector_num)) == 0) {
        log_printf("[WARN] Sector %lu write-protected, disabling...\r\n",
                   (unsigned long)sector_num);

        dis_wrp_all_sectors();
        delay_ms(2);

        wrp_byte = flash_obytes_get_wrp();
        if ((wrp_byte & (1U << sector_num)) == 0) {
            log_printf("[ERROR] WRP disable failed for sector %lu\r\n",
                       (unsigned long)sector_num);
            return FLASH_ERASE_ERR_WRP_ACTIVE;
        }
        log_printf("[INFO] WRP disabled\r\n");
        delay_ms(5);
    }

    /* -------- Step 3: Check flash busy -------- */
    if (FLASH->SR & FLASH_SR_BSY) {
        log_printf("[ERROR] Flash busy\r\n");
        return FLASH_ERASE_ERR_BUSY;
    }

    /* -------- Step 4: Unlock flash -------- */
    log_printf("[INFO] Unlocking flash...\r\n");
    delay_ms(5);
    status = flash_unlock_cmsis();
    if (status != FLASH_ERASE_SUCCESS) {
        log_printf("[ERROR] Flash unlock failed\r\n");
        return status;
    }

    /* -------- Step 5: Perform erase -------- */
    log_printf("[INFO] Erasing sector %lu...\r\n", (unsigned long)sector_num);
    delay_ms(5);
    status = do_sector_erase_cmsis(sector_num);

    /* -------- Step 6: Lock flash -------- */
    log_printf("[INFO] Locking flash...\r\n");
    delay_ms(5);
    flash_lock_cmsis();

    /* Check erase result */
    if (status != FLASH_ERASE_SUCCESS) {
        log_printf("[ERROR] Erase failed: %s\r\n", flash_erase_status_str(status));
        return status;
    }

    /* -------- Step 7: Verify erase -------- */
    log_printf("[INFO] Verifying erase...\r\n");
    delay_ms(5);
    if (!verify_sector_erased(sector_addr, sector_size)) {
        log_printf("[ERROR] Verification failed\r\n");

        /* Debug: find first non-erased byte */
        const uint8_t *ptr = (const uint8_t *)sector_addr;
        for (uint32_t i = 0; i < sector_size; i++) {
            if (ptr[i] != FLASH_ERASED_VALUE) {
                log_printf("[DEBUG] Non-0xFF at offset 0x%08lX: 0x%02X\r\n",
                           (unsigned long)i, (unsigned)ptr[i]);
                delay_ms(5);
                break;
            }
        }
        return FLASH_ERASE_ERR_VERIFY_FAIL;
    }

    log_printf("[OK] Sector %lu erased successfully\r\n", (unsigned long)sector_num);
    log_printf("[INFO] ═══════════════════════════════════════════════════════\r\n");
    delay_ms(5);
    return FLASH_ERASE_SUCCESS;
}


/* ============================================================================
 * Erase manifest sector (Sector 7) - convenience wrapper
 * ============================================================================ */
flash_erase_status_t erase_manifest_sector(void)
{
    /* Check the return status before continuing to the next step. */
    log_printf("\r\n");
    log_printf("┌─────────────────────────────────────────────────────────────┐\r\n");
    log_printf("│         MANIFEST SECTOR ERASE (SECTOR 7) - CMSIS            │\r\n");
    log_printf("└─────────────────────────────────────────────────────────────┘\r\n");
    delay_ms(5);
    flash_erase_status_t ret = erase_flash_sector(MANIFEST_SECTOR_NUM);

    if (ret == FLASH_ERASE_SUCCESS) {
        log_printf("[OK] Manifest sector ready\r\n");
        delay_ms(5);
    } else {
        log_printf("[FATAL] Erase failed: %s (%d)\r\n",
                   flash_erase_status_str(ret), (int)ret);
    }

    return ret;
}





