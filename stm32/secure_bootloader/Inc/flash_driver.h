/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file flash_driver.h
 * @brief STM32F4 Flash erase, programming and read helper declarations.
 */
#ifndef __FLASH_DRIVER_H
#define __FLASH_DRIVER_H

#include "stm32f4xx.h"
#include "stdbool.h"

/* Programming voltage ranges and parallelism. */
#define FLASH_VOLTAGE_RANGE_1        0x00000000U
#define FLASH_VOLTAGE_RANGE_2        0x00000001U
#define FLASH_VOLTAGE_RANGE_3        0x00000002U
#define FLASH_VOLTAGE_RANGE_4        0x00000003U

#define FLASH_PSIZE_BYTE           0x00000000U
#define FLASH_PSIZE_HALF_WORD      0x00000100U
#define FLASH_PSIZE_WORD           0x00000200U
#define FLASH_PSIZE_DOUBLE_WORD    0x00000300U

#define FLASH_TIMEOUT_VALUE 	   50000

#define FLASH_TYPEPROGRAM_BYTE        0x00000000U
#define FLASH_TYPEPROGRAM_HALFWORD    0x00000001U
#define FLASH_TYPEPROGRAM_WORD        0x00000002U
#define FLASH_TYPEPROGRAM_DOUBLEWORD  0x00000003U

#define FLASH_READ_OK               0
#define FLASH_READ_ERR_NULL_PTR     1
#define FLASH_READ_ERR_ALIGNMENT    2

#define FLASH_TYPEERASE_SECTORS         0x00000000U
#define FLASH_TYPEERASE_MASSERASE       0x00000001U

#define FLASH_KEY1               0x45670123U
#define FLASH_KEY2               0xCDEF89ABU

/* Manifest sector and STM32F411 Flash layout. */
#define MANIFEST_SECTOR_NUM         7U
#define MANIFEST_SECTOR_ADDR        0x08060000U
#define MANIFEST_SECTOR_SIZE        0x20000U
#define MANIFEST_SECTOR_END_ADDR    0x0807FFFFU
#define STM32F411_NUM_SECTORS       8U
#define STM32F411_FLASH_SIZE        0x80000U
#define STM32F411_FLASH_START       0x08000000U
#define STM32F411_FLASH_END         0x0807FFFFU
#define ERASE_VERIFY_SAMPLE_SIZE    256U
#define FLASH_ERASED_VALUE          0xFFU

#define NUM_SECTORS STM32F411_NUM_SECTORS

#define FLASH_SECTOR_0     0U
#define FLASH_SECTOR_1     1U
#define FLASH_SECTOR_2     2U
#define FLASH_SECTOR_3     3U
#define FLASH_SECTOR_4     4U
#define FLASH_SECTOR_5     5U
#define FLASH_SECTOR_6     6U
#define FLASH_SECTOR_7     7U

/** @brief Generic device operation status. */
typedef enum
{
	DEV_OK  = 0x00U,
	DEV_ERROR = 0x01U,
	DEV_BUSY  = 0x02U,
	DEV_TIMEOUT = 0x03U
}DEV_StatusTypeDef;

/** @brief Flash procedure currently in progress. */
typedef enum
{
	FLASH_PROC_NONE = 0U,
	FLASH_PROC_SECTERASE,
	FLASH_PROC_MASSERASE,
	FLASH_PROC_PROGRAM
}FLASH_ProcedureTypeDef;

/** @brief Flash lock state. */
typedef enum
{
	DEV_UNLOCKED = 0x00U,
	DEV_LOCKED   = 0x01U
}DEV_LockType;

/** @brief Runtime state used by the Flash driver. */
typedef struct
{
	__IO FLASH_ProcedureTypeDef  ProcedureOngoing;
	__IO uint32_t NbSectorsToErase;
	__IO uint32_t Sector;
	__IO uint32_t Address;
	DEV_LockType Lock;
	__IO uint32_t ErrorCode;
}FLASH_ProcessTypeDef;

/** @brief Sector/mass-erase configuration. */
typedef struct
{
	uint32_t TypeErase;
	uint32_t Sector;
	uint32_t NbSectors;
	uint32_t VoltageRange;
}FLASH_EraseInitTypeDef;

/** @brief Sector erase and verification status values. */
typedef enum {
   FLASH_ERASE_SUCCESS             =  0,
   FLASH_ERASE_ERR_WRP_ACTIVE      = -1,
   FLASH_ERASE_ERR_UNLOCK_FAIL     = -2,
   FLASH_ERASE_ERR_ERASE_FAIL      = -3,
   FLASH_ERASE_ERR_VERIFY_FAIL     = -4,
   FLASH_ERASE_ERR_TIMEOUT         = -5,
   FLASH_ERASE_ERR_LOCK_FAIL       = -6,
   FLASH_ERASE_ERR_BUSY            = -7,
	MANIFEST_STORE_ERR_FLASH_ERASE  = -8,
} flash_erase_status_t;

/**
 * @brief Program one value using the requested Flash programming width.
 * @param prg_type Programming type.
 * @param address Destination Flash address.
 * @param data Value to program.
 * @return Flash operation status.
 */
DEV_StatusTypeDef flash_program(uint32_t prg_type,  uint32_t address, uint64_t data);

/** @brief Program one byte at a Flash address. */
void flash_program_byte(uint32_t address, uint8_t data);

/** @brief Program one halfword at a Flash address. */
void flash_program_halfword(uint32_t address, uint16_t data);

/** @brief Program one word at a Flash address. */
void flash_program_word(uint32_t address, uint32_t data);

/**
 * @brief Erase and verify the manifest sector.
 * @return Flash erase status.
 */
flash_erase_status_t erase_manifest_sector(void);

/**
 * @brief Erase and verify one Flash sector.
 * @param sector_num Sector number.
 * @return Flash erase status.
 */
flash_erase_status_t erase_flash_sector(uint32_t sector_num);

/**
 * @brief Check representative regions of a sector for the erased value.
 * @param start_addr Sector start address.
 * @param size Sector size in bytes.
 * @return True when the checked bytes are erased.
 */
bool verify_sector_erased(uint32_t start_addr, uint32_t size);

/**
 * @brief Convert a Flash erase status to a diagnostic string.
 * @param status Flash erase status.
 * @return Constant status string.
 */
const char* flash_erase_status_str(flash_erase_status_t status);

/** @brief Program one doubleword at a Flash address. */
void flash_program_doubleword(uint32_t address, uint64_t data);

/** @brief Configure and start one sector erase. */
void flash_sector_erase(uint32_t sector, uint8_t voltage_range);

/** @brief Configure and start a mass erase. */
void flash_mass_erase(uint8_t voltage_range);

/**
 * @brief Unlock the Flash controller.
 * @return Flash operation status.
 */
DEV_StatusTypeDef flash_unlock(void);

/**
 * @brief Read one float from Flash.
 * @param start_sect_addr Source Flash address.
 * @return Stored float value.
 */
float flash_read_num(uint32_t start_sect_addr);

/** @brief Store one float in Flash. */
void flash_write_num(uint32_t start_sect_addr, float num);

/**
 * @brief Read 32-bit words from Flash.
 * @return Flash read status.
 */
int flash_read_data(uint32_t start_sect_addr, uint32_t *rx_buff, uint16_t numberofwords);

/** @brief Copy raw bytes from memory-mapped Flash. */
void flash_read_bytes_memcpy(uint32_t start_addr, uint8_t *rx_buff, uint32_t nbytes);

/**
 * @brief Erase the required sectors and write 32-bit words.
 * @return 0 on success, otherwise the stored Flash error code.
 */
uint32_t flash_write_data(uint32_t start_sect_addr,uint32_t *data, uint16_t numberofwords);

/** @brief Copy packed word data into a character buffer. */
void get_str(uint32_t *src_data, char *dest_buff);

#endif
