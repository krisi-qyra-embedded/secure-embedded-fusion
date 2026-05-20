/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file flash_protection.h
 * @brief Option-byte protection, OTP provisioning and application handoff declarations.
 */
#ifndef FLASH_PROTECTION_H_
#define FLASH_PROTECTION_H_

#include <stdint.h>
#include "flash_driver.h"
#include <stddef.h>

#define OB_RDP_LEVEL_0 ((uint8_t)0xAA)
#define OB_RDP_LEVEL_1 ((uint8_t)0x55)
/* RDP level 2 is irreversible on the target device. */
#define OB_RDP_LEVEL_2 ((uint8_t)0xCC)

/* Flash control/status register fields used by the CMSIS paths. */
#define FLASH_CR_PG             (1U << 0)
#define FLASH_CR_SER            (1U << 1)
#define FLASH_CR_MER            (1U << 2)
#define FLASH_CR_SNB_POS        3U
#define FLASH_CR_SNB_MSK        (0xFU << FLASH_CR_SNB_POS)
#define FLASH_CR_PSIZE_POS      8U
#define FLASH_CR_PSIZE_MSK      (0x3U << FLASH_CR_PSIZE_POS)
#define FLASH_CR_PSIZE_X32      (0x2U << FLASH_CR_PSIZE_POS)
#define FLASH_CR_STRT           (1U << 16)
#define FLASH_CR_LOCK           (1U << 31)

#define FLASH_SR_BSY            (1U << 16)
#define FLASH_SR_EOP            (1U << 0)
#define FLASH_SR_OPERR          (1U << 1)
#define FLASH_SR_WRPERR         (1U << 4)
#define FLASH_SR_PGAERR         (1U << 5)
#define FLASH_SR_PGPERR         (1U << 6)
#define FLASH_SR_PGSERR         (1U << 7)

#define FLASH_SR_ERR_MSK        (FLASH_SR_OPERR | FLASH_SR_WRPERR | \
                                 FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)

#define FLASH_TIMEOUT           100000U

#define FLASH_BASE_ADDR        (0x08000000UL)
#define FLASH_END_ADDR         (0x0807FFFFUL)
#define SRAM_BASE_ADDR         (0x20000000UL)
#define SRAM_END_ADDR          (0x2001FFFFUL)
#define EMPTY_MEM              (0xFFFFFFFFUL)

/* STM32F411 OTP data and lock-byte regions. */
#define OTP_BASE_ADDR           0x1FFF7800U
#define OTP_LOCK_BASE_ADDR      0x1FFF7A00U

#define OTP_BLOCK_SIZE          32U
#define OTP_NUM_BLOCKS          16U
#define OTP_LOCK_BLOCK_SIZE     16U

/* Ed25519 public-key storage uses OTP block 0 in this interface. */
#define OTP_PUBKEY_BLOCK        0U
#define OTP_PUBKEY_ADDR         (OTP_BASE_ADDR + (OTP_PUBKEY_BLOCK * OTP_BLOCK_SIZE))
#define OTP_PUBKEY_LOCK_ADDR    (OTP_LOCK_BASE_ADDR + OTP_PUBKEY_BLOCK)

/* Optional rollback-counter storage. */
#define OTP_ROLLBACK_BLOCK      1U
#define OTP_ROLLBACK_ADDR       (OTP_BASE_ADDR + (OTP_ROLLBACK_BLOCK * OTP_BLOCK_SIZE))
#define OTP_ROLLBACK_LOCK_ADDR  (OTP_LOCK_BASE_ADDR + OTP_ROLLBACK_BLOCK)

#define ED25519_PUBKEY_SIZE     32U

#define OTP_UNLOCKED            0xFFU
#define OTP_LOCKED              0x00U
#define OTP_ERASED_VALUE        0xFFU

/** @brief OTP provisioning status. */
typedef enum {
   OTP_SUCCESS                 =  0,
   OTP_ERR_NULL_INPUT          = -1,
   OTP_ERR_INVALID_BLOCK       = -2,
   OTP_ERR_ALREADY_LOCKED      = -3,
   OTP_ERR_ALREADY_PROGRAMMED  = -4,
   OTP_ERR_UNLOCK_FAIL         = -5,
   OTP_ERR_WRITE_FAIL          = -6,
   OTP_ERR_VERIFY_FAIL         = -7,
   OTP_ERR_LOCK_FAIL           = -8,
   OTP_ERR_TIMEOUT             = -9,
} otp_status_t;

/** @brief Write-protection bit mask for STM32F411 sectors. */
typedef enum
{
	SECTOR_0 = 0x01,  // 0B 0000 0001
	SECTOR_1 = 0x02,  // OB 0000 0010
	SECTOR_2 = 0x04,  // OB 0000 0100
	SECTOR_3 = 0x08,  // OB 0000 1000
	SECTOR_4 = 0x10,  // OB 0001 0000
	SECTOR_5 = 0x20,  // OB 0010 0000
	SECTOR_6 = 0x40,  // OB 0100 0000
	SECTOR_7 = 0x80,  // OB 1000 0000
}sector_wrp_mask;

/** @brief Current option-byte configuration. */
typedef struct
{
	uint32_t WRPSector;
	uint32_t RDPLevel;
	uint32_t BORLevel;
	uint32_t USERConfig;
}FLASH_OBProgramInitTypeDef;

/**
 * @brief Store the Ed25519 public key in OTP block 0.
 * @param public_key 32-byte public key.
 * @param lock_after Permanently lock the block after a successful write when true.
 * @return OTP operation status.
 */
otp_status_t otp_store_public_key(const uint8_t *public_key, bool lock_after);

/**
 * @brief Read the Ed25519 public key from OTP block 0.
 * @param public_key_out Destination 32-byte buffer.
 * @return OTP operation status.
 */
otp_status_t otp_read_public_key(uint8_t *public_key_out);

/**
 * @brief Check whether an OTP block is permanently locked.
 * @param block_num OTP block number.
 * @return True when locked.
 */
bool otp_is_block_locked(uint8_t block_num);

/**
 * @brief Check whether public-key data is present in OTP.
 * @return True when the key area is not erased.
 */
bool otp_is_pubkey_provisioned(void);

/**
 * @brief Convert an OTP status to a diagnostic string.
 * @param status OTP status.
 * @return Constant status string.
 */
const char* otp_status_str(otp_status_t status);

/**
 * @brief Read the active option-byte configuration.
 * @param curr_obytes Destination configuration structure.
 * @return 0 on success, negative value for invalid input.
 */
int flash_obytes_get_config(FLASH_OBProgramInitTypeDef *curr_obytes);

/** @brief Program the requested RDP level. */
void flash_set_rdp_level(uint8_t level);

/** @brief Disable write protection for all sectors. */
void dis_wrp_all_sectors(void);

/** @brief Enable write protection for the selected sector mask. */
void sector_set_wrp(sector_wrp_mask sector_num);

/** @brief Lock option-byte programming. */
DEV_StatusTypeDef flash_obytes_lock(void);

/** @brief Unlock option-byte programming. */
DEV_StatusTypeDef flash_obytes_unlock(void);

/** @brief Read the RDP option byte. */
uint8_t flash_obytes_get_rdp(void);

/** @brief Read the WRP option byte. */
uint8_t flash_obytes_get_wrp(void);

/** @brief Read the user option-byte bits. */
uint8_t flash_obytes_get_user(void);

/** @brief Read the BOR option-byte bits. */
uint8_t flash_obytes_get_bor(void);

/**
 * @brief Validate the application vector table and transfer execution to it.
 * @param app_base Application vector-table base address.
 */
void jump_to_app(uint32_t app_base);

typedef void (*func_ptr_t)(void);

#endif
