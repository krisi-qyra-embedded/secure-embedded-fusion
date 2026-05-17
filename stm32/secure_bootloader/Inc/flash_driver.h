/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */
#ifndef __FLASH_DRIVER_H
#define __FLASH_DRIVER_H

#include "stm32f4xx.h"
#include "stdbool.h"

#define FLASH_VOLTAGE_RANGE_1        0x00000000U  /*!< Device operating range: 1.8V to 2.1V                */
#define FLASH_VOLTAGE_RANGE_2        0x00000001U  /*!< Device operating range: 2.1V to 2.7V                */
#define FLASH_VOLTAGE_RANGE_3        0x00000002U  /*!< Device operating range: 2.7V to 3.6V                */
#define FLASH_VOLTAGE_RANGE_4        0x00000003U  /*!< Device operating range: 2.7V to 3.6V + External Vpp */

#define FLASH_PSIZE_BYTE           0x00000000U
#define FLASH_PSIZE_HALF_WORD      0x00000100U
#define FLASH_PSIZE_WORD           0x00000200U
#define FLASH_PSIZE_DOUBLE_WORD    0x00000300U

#define FLASH_TIMEOUT_VALUE 	   50000 /*50 seconds*/


#define FLASH_TYPEPROGRAM_BYTE        0x00000000U  /*!< Program byte (8-bit) at a specified address           */
#define FLASH_TYPEPROGRAM_HALFWORD    0x00000001U  /*!< Program a half-word (16-bit) at a specified address   */
#define FLASH_TYPEPROGRAM_WORD        0x00000002U  /*!< Program a word (32-bit) at a specified address        */
#define FLASH_TYPEPROGRAM_DOUBLEWORD  0x00000003U  /*!< Program a double word (64-bit) at a specified address */

#define FLASH_READ_OK               0
#define FLASH_READ_ERR_NULL_PTR     1
#define FLASH_READ_ERR_ALIGNMENT    2

#define FLASH_TYPEERASE_SECTORS         0x00000000U  /*!< Sectors erase only          */
#define FLASH_TYPEERASE_MASSERASE       0x00000001U  /*!< Flash Mass erase activation */

#define FLASH_KEY1               0x45670123U
#define FLASH_KEY2               0xCDEF89ABU

/* Sector 7 configuration */
#define MANIFEST_SECTOR_NUM         7U
#define MANIFEST_SECTOR_ADDR        0x08060000U
#define MANIFEST_SECTOR_SIZE        0x20000U        /* 128KB for sector 7 on STM32F4 */
#define MANIFEST_SECTOR_END_ADDR    0x0807FFFFU
/* STM32F411 has 8 sectors (0-7), 512KB total */
#define STM32F411_NUM_SECTORS       8U
#define STM32F411_FLASH_SIZE        0x80000U    /* 512KB */
#define STM32F411_FLASH_START       0x08000000U
#define STM32F411_FLASH_END         0x0807FFFFU
/* Erase verification sample size (check first N bytes) */
#define ERASE_VERIFY_SAMPLE_SIZE    256U

/* Flash erased state */
#define FLASH_ERASED_VALUE          0xFFU

#define NUM_SECTORS STM32F411_NUM_SECTORS

#define FLASH_SECTOR_0     0U /*!< Sector Number 0   */
#define FLASH_SECTOR_1     1U /*!< Sector Number 1   */
#define FLASH_SECTOR_2     2U /*!< Sector Number 2   */
#define FLASH_SECTOR_3     3U /*!< Sector Number 3   */
#define FLASH_SECTOR_4     4U /*!< Sector Number 4   */
#define FLASH_SECTOR_5     5U /*!< Sector Number 5   */
#define FLASH_SECTOR_6     6U /*!< Sector Number 6   */
#define FLASH_SECTOR_7     7U /*!< Sector Number 7   */


typedef enum
{
	DEV_OK  = 0x00U,
	DEV_ERROR = 0x01U,
	DEV_BUSY  = 0x02U,
	DEV_TIMEOUT = 0x03U
}DEV_StatusTypeDef;

typedef enum
{
	FLASH_PROC_NONE = 0U,
	FLASH_PROC_SECTERASE,
	FLASH_PROC_MASSERASE,
	FLASH_PROC_PROGRAM
}FLASH_ProcedureTypeDef;

typedef enum
{
	DEV_UNLOCKED = 0x00U,
	DEV_LOCKED   = 0x01U
}DEV_LockType;


typedef struct
{
	/*Internal variable to indicate currently running procedure*/
	__IO FLASH_ProcedureTypeDef  ProcedureOngoing;
	/*Internal variable to save number of sectors to erase*/
	__IO uint32_t NbSectorsToErase;
	__IO uint32_t Sector;
	__IO uint32_t Address;
	DEV_LockType Lock;
	__IO uint32_t ErrorCode;

}FLASH_ProcessTypeDef;


typedef struct
{
	uint32_t TypeErase;
	uint32_t Sector;
	uint32_t NbSectors;
	uint32_t VoltageRange;


}FLASH_EraseInitTypeDef;

/* Status codes */
typedef enum {
    FLASH_ERASE_SUCCESS             =  0,
    FLASH_ERASE_ERR_WRP_ACTIVE      = -1,   /* Write protection still active */
    FLASH_ERASE_ERR_UNLOCK_FAIL     = -2,   /* Could not unlock flash */
    FLASH_ERASE_ERR_ERASE_FAIL      = -3,   /* HAL erase operation failed */
    FLASH_ERASE_ERR_VERIFY_FAIL     = -4,   /* Post-erase verification failed */
    FLASH_ERASE_ERR_TIMEOUT         = -5,   /* Operation timed out */
    FLASH_ERASE_ERR_LOCK_FAIL       = -6,   /* Could not re-lock flash */
    FLASH_ERASE_ERR_BUSY            = -7,   /* Flash busy */
	MANIFEST_STORE_ERR_FLASH_ERASE  = -8,
} flash_erase_status_t;


DEV_StatusTypeDef flash_program(uint32_t prg_type,  uint32_t address, uint64_t data);

void flash_program_byte(uint32_t address, uint8_t data);

void flash_program_halfword(uint32_t address, uint16_t data);

void flash_program_word(uint32_t address, uint32_t data);
/**
 * @brief  Erase manifest sector (Sector 7) with full verification
 * @note   Automatically handles WRP check, unlock, erase, verify, lock
 * @retval flash_erase_status_t
 */
flash_erase_status_t erase_manifest_sector(void);
/**
 * @brief  Erase any sector by number (0-11 for STM32F4)
 * @param  sector_num  Sector number (0-11)
 * @retval flash_erase_status_t
 */
flash_erase_status_t erase_flash_sector(uint32_t sector_num);
/**
 * @brief  Verify sector is fully erased (all 0xFF)
 * @param  start_addr  Sector start address
 * @param  size        Sector size in bytes
 * @retval true if erased, false otherwise
 */
bool verify_sector_erased(uint32_t start_addr, uint32_t size);
/**
 * @brief  Get human-readable error string
 * @param  status  Status code
 * @retval const char* error description
 */
const char* flash_erase_status_str(flash_erase_status_t status);

void flash_program_doubleword(uint32_t address, uint64_t data);
void flash_sector_erase(uint32_t sector, uint8_t voltage_range);
void flash_mass_erase(uint8_t voltage_range);
DEV_StatusTypeDef flash_unlock(void);
float flash_read_num(uint32_t start_sect_addr);
void flash_write_num(uint32_t start_sect_addr, float num);
int flash_read_data(uint32_t start_sect_addr, uint32_t *rx_buff, uint16_t numberofwords);
void flash_read_bytes_memcpy(uint32_t start_addr, uint8_t *rx_buff, uint32_t nbytes);
uint32_t flash_write_data(uint32_t start_sect_addr,uint32_t *data, uint16_t numberofwords);
void get_str(uint32_t *src_data, char *dest_buff);

#endif
