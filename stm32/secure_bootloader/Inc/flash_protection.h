/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */
#ifndef FLASH_PROTECTION_H_
#define FLASH_PROTECTION_H_

#include <stdint.h>
#include "flash_driver.h"
#include <stddef.h>

#define OB_RDP_LEVEL_0 ((uint8_t)0xAA)
#define OB_RDP_LEVEL_1 ((uint8_t)0x55)
/*WARNING :  Enabling level 2 locks your debugger FOREVER*/
#define OB_RDP_LEVEL_2 ((uint8_t)0xCC)

#define FLASH_CR_PG             (1U << 0)       /* Programming */
#define FLASH_CR_SER            (1U << 1)       /* Sector Erase */
#define FLASH_CR_MER            (1U << 2)       /* Mass Erase */
#define FLASH_CR_SNB_POS        3U              /* Sector number bit position */
#define FLASH_CR_SNB_MSK        (0xFU << FLASH_CR_SNB_POS)
#define FLASH_CR_PSIZE_POS      8U              /* Program size bit position */
#define FLASH_CR_PSIZE_MSK      (0x3U << FLASH_CR_PSIZE_POS)
#define FLASH_CR_PSIZE_X32      (0x2U << FLASH_CR_PSIZE_POS)  /* 32-bit parallelism */
#define FLASH_CR_STRT           (1U << 16)      /* Start */
#define FLASH_CR_LOCK           (1U << 31)      /* Lock */

#define FLASH_SR_BSY            (1U << 16)      /* Busy */
#define FLASH_SR_EOP            (1U << 0)       /* End of operation */
#define FLASH_SR_OPERR          (1U << 1)       /* Operation error */
#define FLASH_SR_WRPERR         (1U << 4)       /* Write protection error */
#define FLASH_SR_PGAERR         (1U << 5)       /* Programming alignment error */
#define FLASH_SR_PGPERR         (1U << 6)       /* Programming parallelism error */
#define FLASH_SR_PGSERR         (1U << 7)       /* Programming sequence error */

#define FLASH_SR_ERR_MSK        (FLASH_SR_OPERR | FLASH_SR_WRPERR | \
                                 FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR)

#define FLASH_TIMEOUT           100000U         /* Timeout for operations */


/* ============================================================================
 * STM32F411 OTP Memory Map
 * ============================================================================
 *
 * OTP Area: 0x1FFF7800 - 0x1FFF7A0F (528 bytes total)
 *
 * ┌─────────┬──────────────────────────┬────────┐
 * │ Block   │ Address Range            │ Size   │
 * ├─────────┼──────────────────────────┼────────┤
 * │ Block 0 │ 0x1FFF7800 - 0x1FFF781F  │ 32B    │ ← PUBLIC KEY HERE
 * │ Block 1 │ 0x1FFF7820 - 0x1FFF783F  │ 32B    │
 * │ Block 2 │ 0x1FFF7840 - 0x1FFF785F  │ 32B    │
 * │ ...     │ ...                      │ ...    │
 * │ Block 15│ 0x1FFF79E0 - 0x1FFF79FF  │ 32B    │
 * ├─────────┼──────────────────────────┼────────┤
 * │ Lock    │ 0x1FFF7A00 - 0x1FFF7A0F  │ 16B    │ ← Lock bytes
 * └─────────┴──────────────────────────┴────────┘
 *
 * Lock byte values:
 *   0xFF = OTP block is unlocked (can still be written)
 *   0x00 = OTP block is PERMANENTLY locked (cannot be modified)
 *
 * ============================================================================ */





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


typedef struct
{
	uint32_t WRPSector;
	uint32_t RDPLevel;
	uint32_t BORLevel;
	uint32_t USERConfig;

}FLASH_OBProgramInitTypeDef;

int flash_obytes_get_config(FLASH_OBProgramInitTypeDef *curr_obytes);
void flash_set_rdp_level(uint8_t level);
void dis_wrp_all_sectors(void);
void sector_set_wrp(sector_wrp_mask sector_num);
DEV_StatusTypeDef flash_obytes_lock(void);
DEV_StatusTypeDef flash_obytes_unlock(void);
uint8_t flash_obytes_get_rdp(void);
uint8_t flash_obytes_get_wrp(void);
uint8_t flash_obytes_get_user(void);
uint8_t flash_obytes_get_bor(void);


#endif
