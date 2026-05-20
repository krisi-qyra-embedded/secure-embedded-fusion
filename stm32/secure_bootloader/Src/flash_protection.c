/**
 * @file flash_protection.c
 * @brief Flash option-byte protection, sector erase, OTP storage and boot handoff helpers.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include "flash_protection.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* Bounded polling limit used by option-byte operations. */
#ifndef FLASH_WAIT_TIMEOUT_ITER
#define FLASH_WAIT_TIMEOUT_ITER  1000000U
#endif
/* #define FLASH_OPTCR_OPTLOCK   (1U << 0)

   \#define FLASH_OPTCR_OPTSTART  (1U << 1)

   \#define FLASH_SR_BSY          (1U << 16) */
#define FLASH_GET_FLAG(FLAG)    ((FLASH->SR) & (FLAG))
/* STM32F4 Flash and option-byte unlock keys. */
#define FLASH_OPT_KEY1 0x08192A3BU
#define FLASH_OPT_KEY2 0x4C5D6E7FU
#define FLASH_KEY1              0x45670123U
#define FLASH_KEY2              0xCDEF89ABU
/* Byte addresses and bit offsets used inside OPTCR. */
#define OPTCR_BYTE0_ADDRESS     0x40023C14U
#define OPTCR_BYTE1_ADDRESS     0x40023C15U
#define OPTCR_BYTE2_ADDRESS     0x40023C16U
#define OPTCR_BYTE3_ADDRESS     0x40023C17U
#define START_OF_WRP_BYTE       16
#define START_OF_RDP_BYTE       8
#define FLASH_TIMEOUT           100000U
/* STM32F411 sector layout used by erase and verification helpers. */
static const struct {
    uint32_t addr;
    uint32_t size;
    const char *name;
} sector_map[NUM_SECTORS] = {
    [0] = { 0x08000000, 0x04000, "Sector 0 (16KB)"  },
    [1] = { 0x08004000, 0x04000, "Sector 1 (16KB)"  },
    [2] = { 0x08008000, 0x04000, "Sector 2 (16KB)"  },
    [3] = { 0x0800C000, 0x04000, "Sector 3 (16KB)"  },
    [4] = { 0x08010000, 0x10000, "Sector 4 (64KB)"  },
    [5] = { 0x08020000, 0x20000, "Sector 5 (128KB)" },
    [6] = { 0x08040000, 0x20000, "Sector 6 (128KB)" },
    [7] = { 0x08060000, 0x20000, "Sector 7 (128KB)" },
};
#define NUM_SECTORS (sizeof(sector_map) / sizeof(sector_map[0]))
static bool flash_wait_not_busy(uint32_t timeout_iter);
/* Wait for the Flash controller to become idle with a bounded poll. */
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
/* Option-byte read, lock and protection helpers. */
/**
 * @brief Read the write-protection option-byte value.
 * @return Current WRP byte.
 */
uint8_t flash_obytes_get_wrp(void)
{
    /* Check the flash status and address range before continuing. */
    return (*(__IO uint8_t *)(OPTCR_BYTE2_ADDRESS));
}
/**
 * @brief Read the read-protection option-byte value.
 * @return Current RDP byte.
 */
uint8_t flash_obytes_get_rdp(void)
{
    /* Check the flash status and address range before continuing. */
    return (*(__IO uint8_t *)(OPTCR_BYTE1_ADDRESS));
}
/**
 * @brief Read the user option-byte bits used by this driver.
 * @return Masked user option-byte value.
 */
uint8_t flash_obytes_get_user(void)
{
    /* original: return ((uint8_t)(FLASH->OPTCR & 0xE0)); */
    uint8_t raw = (uint8_t)((FLASH->OPTCR >> 0) & 0xFFU);
    return (uint8_t)(raw & 0xE0U);
}
/**
 * @brief Read the brown-out reset option-byte bits.
 * @return Masked BOR option-byte value.
 */
uint8_t flash_obytes_get_bor(void)
{
    /* original: return ((uint8_t)(FLASH->OPTCR & 0x0C)); */
    uint8_t raw = (uint8_t)((FLASH->OPTCR >> 0) & 0xFFU);
    return (uint8_t)(raw & 0x0CU);
}
/**
 * @brief Lock option-byte programming.
 * @return Device status.
 */
DEV_StatusTypeDef flash_obytes_lock(void)
{
    /* Check the flash status and address range before continuing. */
    FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
    return DEV_OK;
}
/**
 * @brief Unlock option-byte programming with the STM32 key sequence.
 * @return Device status.
 */
DEV_StatusTypeDef flash_obytes_unlock(void)
{
    /* Check the flash status and address range before continuing. */
    if ((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) == RESET)
    {
        return DEV_OK;
    }
    FLASH->OPTKEYR = FLASH_OPT_KEY1;
    FLASH->OPTKEYR = FLASH_OPT_KEY2;
    if ((FLASH->OPTCR & FLASH_OPTCR_OPTLOCK) != RESET)
    {
        return DEV_ERROR;
    }
    return DEV_OK;
}
/**
 * @brief Enable write protection for the selected sector mask.
 * @param sector_num Sector write-protection mask.
 */
void sector_set_wrp(sector_wrp_mask sector_num)
{
    DEV_StatusTypeDef st;
    st = flash_obytes_unlock();
    if (st != DEV_OK)
        return;
    if (!flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER))
        goto finalize;
    {
        uint32_t optcr = FLASH->OPTCR;
        /* Clear SPRMOD so WRP bits use write-protection semantics. */
        optcr &= ~(1U << 31);
        optcr &= ~((uint32_t)sector_num << START_OF_WRP_BYTE);
        FLASH->OPTCR = optcr;
    }
    /* OPTSTART commits the modified option bytes. */
    FLASH->OPTCR |= (1U << 1);
    flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER);
finalize:
    flash_obytes_lock();
}
/**
 * @brief Disable write protection for all Flash sectors.
 */
void dis_wrp_all_sectors(void)
{
    /* Check the return status before continuing to the next step. */
    DEV_StatusTypeDef st;
    st = flash_obytes_unlock();
    if (st != DEV_OK)
        return;
    if (!flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER))
        goto finalize;
    {
        uint32_t optcr = FLASH->OPTCR;
        optcr &= ~(1U << 31);
        optcr |= ((uint32_t)0xFFU << START_OF_WRP_BYTE);
        FLASH->OPTCR = optcr;
    }
    FLASH->OPTCR |= (1U << 1);
    flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER);
finalize:
    flash_obytes_lock();
}
/**
 * @brief Program the requested read-protection level.
 * @param level RDP option-byte value.
 */
void flash_set_rdp_level(uint8_t level)
{
    /* Check the flash status and address range before continuing. */
    DEV_StatusTypeDef st;
    st = flash_obytes_unlock();
    if (st != DEV_OK)
        return;
    if (!flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER))
        goto finalize;
    {
        uint32_t optcr = FLASH->OPTCR;
        optcr &= ~(0xFFU << START_OF_RDP_BYTE);
        optcr |= ((uint32_t)level << START_OF_RDP_BYTE);
        FLASH->OPTCR = optcr;
    }
    FLASH->OPTCR |= (1U << 1);
    flash_wait_not_busy(FLASH_WAIT_TIMEOUT_ITER);
finalize:
    flash_obytes_lock();
}
/* CMSIS Flash erase support. */
/**
 * @brief Read the current Flash option-byte configuration.
 * @param curr_obytes Destination option-byte configuration structure.
 * @return 0 on success, negative value for invalid input.
 */
int flash_obytes_get_config(FLASH_OBProgramInitTypeDef *curr_obytes)
{
    /* Check the flash status and address range before continuing. */
    if (curr_obytes == NULL)
    {
        return -1;
    }
    curr_obytes->WRPSector  = (uint8_t)flash_obytes_get_wrp();
    curr_obytes->RDPLevel   = (uint8_t)flash_obytes_get_rdp();
    curr_obytes->USERConfig = (uint8_t)flash_obytes_get_user();
    curr_obytes->BORLevel   = (uint8_t)flash_obytes_get_bor();
    return 0;
}
/* Unlock main Flash before a sector erase. */
static flash_erase_status_t flash_unlock_cmsis(void)
{
    /* Check the flash status and address range before continuing. */
    if ((FLASH->CR & FLASH_CR_LOCK) == 0) {
        return FLASH_ERASE_SUCCESS;
    }
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    if (FLASH->CR & FLASH_CR_LOCK) {
        return FLASH_ERASE_ERR_UNLOCK_FAIL;
    }
    return FLASH_ERASE_SUCCESS;
}
/* Re-lock main Flash after the erase path completes. */
static void flash_lock_cmsis(void)
{
    /* Check the flash status and address range before continuing. */
    FLASH->CR |= FLASH_CR_LOCK;
}
/* Wait for a Flash erase/program operation to finish. */
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
/* Clear pending Flash completion and error flags. */
static void flash_clear_errors(void)
{
    /* Check the flash status and address range before continuing. */
    FLASH->SR = FLASH_SR_ERR_MSK | FLASH_SR_EOP;
}
/* Translate Flash status-register error bits into erase status. */
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
/* Execute one sector erase directly through CMSIS registers. */
static flash_erase_status_t do_sector_erase_cmsis(uint32_t sector_num)
{
    flash_erase_status_t status;
    /* Do not reconfigure Flash while a previous operation is active. */
    status = flash_wait_busy(FLASH_TIMEOUT);
    if (status != FLASH_ERASE_SUCCESS) {
        log_printf("[ERROR] Flash busy before erase\r\n");
        return status;
    }
    flash_clear_errors();
    FLASH->CR &= ~(FLASH_CR_SNB_MSK | FLASH_CR_PSIZE_MSK);
    /* Select sector erase, sector number and x32 programming width. */
    FLASH->CR |= FLASH_CR_SER |
                 ((sector_num << FLASH_CR_SNB_POS) & FLASH_CR_SNB_MSK) |
                 FLASH_CR_PSIZE_X32;
    FLASH->CR |= FLASH_CR_STRT;
    __DSB();
    status = flash_wait_busy(FLASH_TIMEOUT * 10);
    if (status != FLASH_ERASE_SUCCESS) {
        log_printf("[ERROR] Erase timeout\r\n");
        FLASH->CR &= ~FLASH_CR_SER;
        return status;
    }
    status = flash_check_errors();
    FLASH->CR &= ~FLASH_CR_SER;
    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR = FLASH_SR_EOP;
    }
    return status;
}
/* Sector erase verification and public erase API. */
/**
 * @brief Check representative regions of a Flash sector for the erased value.
 * @param start_addr Start address of the sector.
 * @param size Sector size in bytes.
 * @return True when all sampled bytes are erased.
 */
bool verify_sector_erased(uint32_t start_addr, uint32_t size)
{
    /* Return a failure status if any validation check fails. */
    const uint8_t *ptr = (const uint8_t *)start_addr;
    uint32_t check_size = (size < ERASE_VERIFY_SAMPLE_SIZE) ? size : ERASE_VERIFY_SAMPLE_SIZE;
    for (uint32_t i = 0; i < check_size; i++) {
        if (ptr[i] != FLASH_ERASED_VALUE) {
            return false;
        }
    }
    uint32_t mid_offset = size / 2;
    for (uint32_t i = 0; i < check_size && (mid_offset + i) < size; i++) {
        if (ptr[mid_offset + i] != FLASH_ERASED_VALUE) {
            return false;
        }
    }
    uint32_t end_offset = size - check_size;
    for (uint32_t i = 0; i < check_size; i++) {
        if (ptr[end_offset + i] != FLASH_ERASED_VALUE) {
            return false;
        }
    }
    return true;
}
/**
 * @brief Convert a Flash erase status to a diagnostic string.
 * @param status Flash erase status.
 * @return Constant status string.
 */
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
/**
 * @brief Erase one STM32F411 Flash sector and verify the result.
 * @param sector_num Sector number to erase.
 * @return Flash erase status.
 */
flash_erase_status_t erase_flash_sector(uint32_t sector_num)
{
    flash_erase_status_t status;
    uint8_t wrp_byte;
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
    /* Skip unnecessary erase cycles when the sector is already blank. */
    if (verify_sector_erased(sector_addr, sector_size)) {
        log_printf("[INFO] Sector %lu already erased, skipping\r\n", (unsigned long)sector_num);
        delay_ms(5);
        return FLASH_ERASE_SUCCESS;
    }
    wrp_byte = flash_obytes_get_wrp();
    log_printf("[INFO] WRP status: 0x%02X\r\n", (unsigned)wrp_byte);
    delay_ms(5);
    /* STM32F4 WRP bits are active-low. */
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
    if (FLASH->SR & FLASH_SR_BSY) {
        log_printf("[ERROR] Flash busy\r\n");
        return FLASH_ERASE_ERR_BUSY;
    }
    log_printf("[INFO] Unlocking flash...\r\n");
    delay_ms(5);
    status = flash_unlock_cmsis();
    if (status != FLASH_ERASE_SUCCESS) {
        log_printf("[ERROR] Flash unlock failed\r\n");
        return status;
    }
    log_printf("[INFO] Erasing sector %lu...\r\n", (unsigned long)sector_num);
    delay_ms(5);
    status = do_sector_erase_cmsis(sector_num);
    log_printf("[INFO] Locking flash...\r\n");
    delay_ms(5);
    flash_lock_cmsis();
    if (status != FLASH_ERASE_SUCCESS) {
        log_printf("[ERROR] Erase failed: %s\r\n", flash_erase_status_str(status));
        return status;
    }
    log_printf("[INFO] Verifying erase...\r\n");
    delay_ms(5);
    if (!verify_sector_erased(sector_addr, sector_size)) {
        log_printf("[ERROR] Verification failed\r\n");
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
/**
 * @brief Erase the configured manifest sector.
 * @return Flash erase status.
 */
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
/* OTP public-key provisioning support. */
/**
 * @brief Convert an OTP operation status to a diagnostic string.
 * @param status OTP operation status.
 * @return Constant status string.
 */
const char* otp_status_str(otp_status_t status)
{
    /* Check the return status before continuing to the next step. */
    switch (status) {
        case OTP_SUCCESS:                return "SUCCESS";
        case OTP_ERR_NULL_INPUT:         return "NULL_INPUT";
        case OTP_ERR_INVALID_BLOCK:      return "INVALID_BLOCK";
        case OTP_ERR_ALREADY_LOCKED:     return "ALREADY_LOCKED";
        case OTP_ERR_ALREADY_PROGRAMMED: return "ALREADY_PROGRAMMED";
        case OTP_ERR_UNLOCK_FAIL:        return "UNLOCK_FAILED";
        case OTP_ERR_WRITE_FAIL:         return "WRITE_FAILED";
        case OTP_ERR_VERIFY_FAIL:        return "VERIFY_FAILED";
        case OTP_ERR_LOCK_FAIL:          return "LOCK_FAILED";
        case OTP_ERR_TIMEOUT:            return "TIMEOUT";
        default:                         return "UNKNOWN_ERROR";
    }
}
/* Unlock main Flash before OTP programming. */
static otp_status_t flash_unlock_for_otp(void)
{
    /* Check the flash status and address range before continuing. */
    if ((FLASH->CR & FLASH_CR_LOCK) == 0) {
        return OTP_SUCCESS;
    }
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    if (FLASH->CR & FLASH_CR_LOCK) {
        return OTP_ERR_UNLOCK_FAIL;
    }
    return OTP_SUCCESS;
}
/* Re-lock Flash after OTP programming. */
static void flash_lock_for_otp(void)
{
    /* Check the flash status and address range before continuing. */
    FLASH->CR |= FLASH_CR_LOCK;
}
/* Wait for an OTP programming operation to complete. */
static otp_status_t flash_wait_for_otp(uint32_t timeout)
{
    /* Check the flash status and address range before continuing. */
    uint32_t count = 0;
    while (FLASH->SR & FLASH_SR_BSY) {
        if (++count > timeout) {
            return OTP_ERR_TIMEOUT;
        }
    }
    return OTP_SUCCESS;
}
/* Program one OTP byte and report controller errors. */
static otp_status_t otp_program_byte(uint32_t address, uint8_t data)
{
    /* Check the return status before continuing to the next step. */
    otp_status_t status;
    status = flash_wait_for_otp(FLASH_TIMEOUT);
    if (status != OTP_SUCCESS) {
        return status;
    }
    FLASH->SR = (FLASH_SR_EOP | FLASH_SR_OPERR | FLASH_SR_WRPERR |
                 FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR);
    FLASH->CR &= ~(FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint8_t *)address = data;
    __DSB();
    status = flash_wait_for_otp(FLASH_TIMEOUT);
    FLASH->CR &= ~FLASH_CR_PG;
    if (FLASH->SR & (FLASH_SR_OPERR | FLASH_SR_WRPERR |
                     FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)) {
        return OTP_ERR_WRITE_FAIL;
    }
    return status;
}
/**
 * @brief Check whether an OTP block is permanently locked.
 * @param block_num OTP block index.
 * @return True when the block is locked or the index is invalid.
 */
bool otp_is_block_locked(uint8_t block_num)
{
    /* Check the return status before continuing to the next step. */
    if (block_num >= OTP_NUM_BLOCKS) {
        return true;
    }
    uint8_t lock_byte = *(__IO uint8_t *)(OTP_LOCK_BASE_ADDR + block_num);
    return (lock_byte == OTP_LOCKED);
}
/**
 * @brief Check whether the configured OTP public-key region contains data.
 * @return True when public-key data is present.
 */
bool otp_is_pubkey_provisioned(void)
{
    /* Validate the key data before using it in crypto operations. */
    const uint8_t *otp_data = (const uint8_t *)OTP_PUBKEY_ADDR;
    for (size_t i = 0; i < ED25519_PUBKEY_SIZE; i++) {
        if (otp_data[i] != OTP_ERASED_VALUE) {
            return true;
        }
    }
    return false;
}
/**
 * @brief Read the configured Ed25519 public key from OTP.
 * @param public_key_out Destination public-key buffer.
 * @return OTP operation status.
 */
otp_status_t otp_read_public_key(uint8_t *public_key_out)
{
    /* Check the read status before using the returned data. */
    if (public_key_out == NULL) {
        return OTP_ERR_NULL_INPUT;
    }
    const uint8_t *otp_data = (const uint8_t *)OTP_PUBKEY_ADDR;
    for (size_t i = 0; i < ED25519_PUBKEY_SIZE; i++) {
        public_key_out[i] = otp_data[i];
    }
    return OTP_SUCCESS;
}
/**
 * @brief Program the Ed25519 public key into OTP and optionally lock its block.
 * @param public_key Public key to provision.
 * @param lock_after Lock the OTP block after a successful write when true.
 * @return OTP operation status.
 */
otp_status_t otp_store_public_key(const uint8_t *public_key, bool lock_after)
{
    otp_status_t status;
    uint8_t readback[ED25519_PUBKEY_SIZE];
    log_printf("\r\n");
    log_printf("┌─────────────────────────────────────────────────────────────┐\r\n");
    log_printf("│         OTP PUBLIC KEY STORAGE (Block 0)                    │\r\n");
    log_printf("└─────────────────────────────────────────────────────────────┘\r\n");
    delay_ms(5);
    if (public_key == NULL) {
        log_printf("[ERROR] NULL public key pointer\r\n");
        return OTP_ERR_NULL_INPUT;
    }
    log_printf("[INFO] Public key to store (%u bytes):\r\n", ED25519_PUBKEY_SIZE);
    log_printf("       ");
    for (size_t i = 0; i < ED25519_PUBKEY_SIZE; i++) {
        log_printf("%02X", public_key[i]);
        if ((i + 1) % 16 == 0 && i + 1 < ED25519_PUBKEY_SIZE) {
            log_printf("\r\n       ");
        }
    }
    log_printf("\r\n");
    delay_ms(10);
    log_printf("[INFO] Checking OTP Block 0 lock status...\r\n");
    delay_ms(5);
    if (otp_is_block_locked(OTP_PUBKEY_BLOCK)) {
        log_printf("[ERROR] OTP Block 0 is PERMANENTLY LOCKED!\r\n");
        log_printf("[ERROR] Public key cannot be modified.\r\n");
        return OTP_ERR_ALREADY_LOCKED;
    }
    log_printf("[OK] OTP Block 0 is unlocked\r\n");
    delay_ms(5);
    log_printf("[INFO] Checking if public key already provisioned...\r\n");
    delay_ms(5);
    if (otp_is_pubkey_provisioned()) {
        otp_read_public_key(readback);
        if (memcmp(public_key, readback, ED25519_PUBKEY_SIZE) == 0) {
            log_printf("[INFO] Public key already provisioned with same value\r\n");
            log_printf("[OK] No write needed\r\n");
            return OTP_SUCCESS;
        } else {
            log_printf("[ERROR] OTP Block 0 already contains different data!\r\n");
            log_printf("[ERROR] OTP can only be written ONCE (bits can only go 1→0)\r\n");
            log_printf("[ERROR] Current OTP content:\r\n");
            log_printf("       ");
            for (size_t i = 0; i < ED25519_PUBKEY_SIZE; i++) {
                log_printf("%02X", readback[i]);
                if ((i + 1) % 16 == 0 && i + 1 < ED25519_PUBKEY_SIZE) {
                    log_printf("\r\n       ");
                }
            }
            log_printf("\r\n");
            return OTP_ERR_ALREADY_PROGRAMMED;
        }
    }
    log_printf("[OK] OTP Block 0 is empty (all 0xFF)\r\n");
    delay_ms(5);
    log_printf("[INFO] Unlocking flash for OTP programming...\r\n");
    delay_ms(5);
    /* OTP writes use the same Flash controller unlock sequence. */
    status = flash_unlock_for_otp();
    if (status != OTP_SUCCESS) {
        log_printf("[ERROR] Flash unlock failed\r\n");
        return status;
    }
    log_printf("[OK] Flash unlocked\r\n");
    delay_ms(5);
    log_printf("[INFO] Programming public key to OTP (0x%08lX)...\r\n",
               (unsigned long)OTP_PUBKEY_ADDR);
    delay_ms(5);
    for (size_t i = 0; i < ED25519_PUBKEY_SIZE; i++) {
        if (public_key[i] != OTP_ERASED_VALUE) {
            /* OTP can only transition programmed bits from 1 to 0. */
            status = otp_program_byte(OTP_PUBKEY_ADDR + i, public_key[i]);
            if (status != OTP_SUCCESS) {
                log_printf("[ERROR] Failed to program byte %u at address 0x%08lX\r\n",
                           (unsigned)i, (unsigned long)(OTP_PUBKEY_ADDR + i));
                flash_lock_for_otp();
                return status;
            }
        }
        if ((i + 1) % 8 == 0) {
            log_printf("  Programmed %u/%u bytes\r\n", (unsigned)(i + 1), ED25519_PUBKEY_SIZE);
            delay_ms(2);
        }
    }
    log_printf("[OK] All %u bytes programmed\r\n", ED25519_PUBKEY_SIZE);
    delay_ms(5);
    log_printf("[INFO] Verifying OTP write...\r\n");
    delay_ms(5);
    memset(readback, 0, sizeof(readback));
    otp_read_public_key(readback);
    if (memcmp(public_key, readback, ED25519_PUBKEY_SIZE) != 0) {
        log_printf("[ERROR] OTP verification FAILED!\r\n");
        log_printf("[ERROR] Written:\r\n       ");
        for (size_t i = 0; i < ED25519_PUBKEY_SIZE; i++) {
            log_printf("%02X", public_key[i]);
            if ((i + 1) % 16 == 0 && i + 1 < ED25519_PUBKEY_SIZE) {
                log_printf("\r\n       ");
            }
        }
        log_printf("\r\n[ERROR] Read back:\r\n       ");
        for (size_t i = 0; i < ED25519_PUBKEY_SIZE; i++) {
            log_printf("%02X", readback[i]);
            if ((i + 1) % 16 == 0 && i + 1 < ED25519_PUBKEY_SIZE) {
                log_printf("\r\n       ");
            }
        }
        log_printf("\r\n");
        flash_lock_for_otp();
        return OTP_ERR_VERIFY_FAIL;
    }
    log_printf("[OK] OTP verification successful\r\n");
    delay_ms(5);
    if (lock_after) {
        log_printf("[INFO] Locking OTP Block 0 PERMANENTLY...\r\n");
        log_printf("[WARN] This is IRREVERSIBLE! Block cannot be modified after this.\r\n");
        delay_ms(10);
        status = otp_program_byte(OTP_PUBKEY_LOCK_ADDR, OTP_LOCKED);
        if (status != OTP_SUCCESS) {
            log_printf("[ERROR] Failed to lock OTP block\r\n");
            flash_lock_for_otp();
            return OTP_ERR_LOCK_FAIL;
        }
        if (!otp_is_block_locked(OTP_PUBKEY_BLOCK)) {
            log_printf("[ERROR] OTP block lock verification failed\r\n");
            flash_lock_for_otp();
            return OTP_ERR_LOCK_FAIL;
        }
        log_printf("[OK] OTP Block 0 is now PERMANENTLY LOCKED\r\n");
    } else {
        log_printf("[INFO] OTP Block 0 left UNLOCKED (can still be modified)\r\n");
        log_printf("[WARN] Consider locking for production devices!\r\n");
    }
    delay_ms(5);
    log_printf("[INFO] Locking flash...\r\n");
    flash_lock_for_otp();
    log_printf("[OK] Flash locked\r\n");
    delay_ms(5);
    log_printf("\r\n");
    log_printf("╔══════════════════════════════════════════════════════════════════╗\r\n");
    log_printf("║           OTP PUBLIC KEY MEMORY MAP                              ║\r\n");
    log_printf("╠══════════════════════════════════════════════════════════════════╣\r\n");
    log_printf("║ Field            │ Address Range             │ Size │ Status     ║\r\n");
    log_printf("╠══════════════════════════════════════════════════════════════════╣\r\n");
    log_printf("║ PUBLIC_KEY       │ 0x%08lX - 0x%08lX │ %2u B │ %-10s ║\r\n",
               (unsigned long)OTP_PUBKEY_ADDR,
               (unsigned long)(OTP_PUBKEY_ADDR + ED25519_PUBKEY_SIZE - 1),
               ED25519_PUBKEY_SIZE,
               lock_after ? "LOCKED" : "UNLOCKED");
    log_printf("║ LOCK_BYTE        │ 0x%08lX                │  1 B │ 0x%02X       ║\r\n",
               (unsigned long)OTP_PUBKEY_LOCK_ADDR,
               (unsigned)*(__IO uint8_t *)OTP_PUBKEY_LOCK_ADDR);
    log_printf("╚══════════════════════════════════════════════════════════════════╝\r\n");
    delay_ms(5);
    log_printf("\r\n[SUCCESS] Public key stored in OTP at 0x%08lX\r\n",
               (unsigned long)OTP_PUBKEY_ADDR);
    delay_ms(5);
    return OTP_SUCCESS;
}
/* Check that the application MSP points into valid STM32 SRAM. */
/* Application handoff validation. */
static int is_valid_msp(uint32_t msp)
{
    /* Return a failure status if any validation check fails. */
    return (msp >= 0x20000000UL) &&
           (msp <= 0x20020000UL) &&
           ((msp & 0x3U) == 0U);
}
/* Check Thumb state and Flash range for the application reset handler. */
static int is_valid_reset_handler(uint32_t reset_handler)
{
    /* Clear the handled interrupt flags and update the transfer state. */
    if ((reset_handler & 0x1U) == 0U) {
        return 0;
    }
    reset_handler &= ~0x1U;
    return (reset_handler >= FLASH_BASE_ADDR) && (reset_handler <= FLASH_END_ADDR);
}
/* Validate application vector-table alignment, MSP and reset handler. */
static int is_valid_app_vector_table(uint32_t app_base)
{
    /* Return a failure status if any validation check fails. */
    uint32_t app_msp   = *(volatile uint32_t *)(app_base + 0U);
    uint32_t app_reset = *(volatile uint32_t *)(app_base + 4U);
    if ((app_base & 0x1FFU) != 0U) {
        return 0;
    }
    if ((app_msp == EMPTY_MEM) || (app_reset == EMPTY_MEM)) {
        return 0;
    }
    if (!is_valid_msp(app_msp)) {
        return 0;
    }
    if (!is_valid_reset_handler(app_reset)) {
        return 0;
    }
    return 1;
}
/**
 * @brief Validate the application vector table and transfer execution to it.
 * @param app_base Application vector-table base address.
 */
__attribute__((noreturn))
/* Transfers control to the validated application firmware. */
 void jump_to_app(uint32_t app_base)
{
    uint32_t app_msp;
    uint32_t app_reset;
    func_ptr_t app_entry;
    /* Refuse handoff unless the vector table passes all structural checks. */
    if (!is_valid_app_vector_table(app_base)) {
        log_printf("[ERROR] No valid application at 0x%08lX\r\n", (unsigned long)app_base);
        while (1) {
        }
    }
    log_printf("[INFO] Preparing jump to application at 0x%08lX\r\n", (unsigned long)app_base);
    delay_ms(5);
    app_msp   = *(volatile uint32_t *)(app_base + 0U);
    app_reset = *(volatile uint32_t *)(app_base + 4U);
    app_entry = (func_ptr_t)app_reset;
    /* Quiesce bootloader interrupt state before changing VTOR/MSP. */
    __disable_irq();
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL  = 0U;
    for (uint32_t i = 0U; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }
    /* HAL_DeInit(); */
    /* HAL_RCC_DeInit(); */
    SCB->ICSR |= SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    SCB->VTOR = app_base;
    __DSB();
    __ISB();
    __set_MSP(app_msp);
    __DSB();
    __ISB();
    log_printf("[INFO] Jumping to application Reset_Handler: 0x%08lX\r\n",
               (unsigned long)app_reset);
    delay_ms(5);
    app_entry();
    while (1) {
    }
}
