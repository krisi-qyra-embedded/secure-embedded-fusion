/**
 * @file flash_driver.c
 * @brief STM32F4 Flash erase, program, read and conversion helpers.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include "string.h"
#include "flash_driver.h"
#include "timebase.h"
#define TEMP_BUFF_SZ		4
/* Shared Flash status and temporary conversion buffer. */
FLASH_ProcessTypeDef pFlash;
uint8_t temp_buff[TEMP_BUFF_SZ];
DEV_StatusTypeDef  flash_wait_for_last_operation(uint32_t timeout);
static void flush_caches(void);
/* Flash erase operations. */
/**
 * @brief Erase one or more Flash sectors or perform a mass erase.
 * @param pt_erase_init Erase configuration.
 * @param sect_err Receives the sector index if a sector erase fails.
 * @return Flash operation status.
 */
DEV_StatusTypeDef flash_ex_erase(FLASH_EraseInitTypeDef *pt_erase_init, uint32_t *sect_err)
{
	DEV_StatusTypeDef  status =  DEV_ERROR;
	uint32_t index = 0U;
	/* Never start a new erase/program cycle while Flash is busy. */
	status = flash_wait_for_last_operation(FLASH_TIMEOUT_VALUE);
	if( status  == DEV_OK)
	{
	   /* 0xFFFFFFFF means no sector-specific erase error has been reported yet. */
	   *sect_err =  0xFFFFFFFFU;
	   if(pt_erase_init->TypeErase ==  FLASH_TYPEERASE_MASSERASE)
	   {
		   flash_mass_erase((uint8_t)pt_erase_init->VoltageRange);
			status = flash_wait_for_last_operation(FLASH_TIMEOUT_VALUE);
			FLASH->CR &=~(1U<<2);
	   }
	   else
	   {
		   for(index = pt_erase_init->Sector; index <  (pt_erase_init->NbSectors + pt_erase_init->Sector); index++)
		   {
			   flash_sector_erase(index, (uint8_t)pt_erase_init->VoltageRange);
				status = flash_wait_for_last_operation(FLASH_TIMEOUT_VALUE);
				/* Clear the sector-erase selection after each operation. */
				CLEAR_BIT(FLASH->CR, (FLASH_CR_SER | FLASH_CR_SNB));
				if(status !=DEV_OK)
				{
					*sect_err =  index;
					break;
				}
		   }
	   }
	   flush_caches();
	}
	return status;
}
/**
 * @brief Configure and start an erase for one Flash sector.
 * @param sector Flash sector number.
 * @param voltage_range Voltage range used to select programming width.
 */
void flash_sector_erase(uint32_t sector, uint8_t voltage_range)
{
    /* Check the flash status and address range before continuing. */
	uint32_t tmp_psize = 0U;
	if(voltage_range ==  FLASH_VOLTAGE_RANGE_1)
	{
		tmp_psize = FLASH_PSIZE_BYTE;
	}
	else if(voltage_range ==  FLASH_VOLTAGE_RANGE_2)
	{
		tmp_psize = FLASH_PSIZE_HALF_WORD;
	}
	else if(voltage_range ==  FLASH_VOLTAGE_RANGE_3)
	{
		tmp_psize = FLASH_PSIZE_WORD;
	}
	else
	{
		tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
	}
	FLASH->CR &=~FLASH_CR_PSIZE;
	FLASH->CR |= tmp_psize;
	FLASH->CR |=  FLASH_CR_SER |(sector << FLASH_CR_SNB_Pos);
}
/**
 * @brief Configure and start a mass erase of internal Flash.
 * @param voltage_range Voltage range used for the erase operation.
 */
void flash_mass_erase(uint8_t voltage_range)
{
	FLASH->CR &=~FLASH_CR_PSIZE;
	/* Mass erase starts after MER and STRT are asserted. */
	FLASH->CR |= FLASH_CR_MER;
	FLASH->CR |=  FLASH_CR_STRT |((uint32_t)voltage_range <<8U);
}
/**
 * @brief Wait for the current Flash operation to complete or time out.
 * @param timeout Maximum wait time in system ticks.
 * @return Flash operation status.
 */
DEV_StatusTypeDef  flash_wait_for_last_operation(uint32_t timeout)
{
   uint32_t tickstart = 0U;
   pFlash.ErrorCode = 0;
   /* Bound the busy wait with the project timebase. */
   tickstart = get_tick();
   while((FLASH->SR & FLASH_SR_BSY) != RESET)
   {
	   if(timeout != MAX_DELAY)
	   {
		   if((timeout  == 0) || ((get_tick() -  tickstart) > timeout))
		   {
			   return DEV_TIMEOUT;
		   }
	   }
   }
   if((FLASH->SR & (1U<<0)) != RESET)
   {
	   FLASH->SR = (1U<<0);
   }
   return DEV_OK;
}
/* Flash programming operations. */
/**
 * @brief Program one value using the requested Flash programming width.
 * @param prg_type Programming type/width.
 * @param address Destination Flash address.
 * @param data Value to program.
 * @return Flash operation status.
 */
DEV_StatusTypeDef flash_program(uint32_t prg_type,  uint32_t address, uint64_t data)
{
	DEV_StatusTypeDef status = DEV_ERROR;
	status = flash_wait_for_last_operation(FLASH_TIMEOUT_VALUE);
	if( status == DEV_OK)
	{
		if(prg_type == FLASH_TYPEPROGRAM_BYTE)
		{
			flash_program_byte(address,(uint8_t)data);
		}
		else if(prg_type == FLASH_TYPEPROGRAM_HALFWORD)
		{
			flash_program_halfword(address,(uint16_t)data);
		}
		else if(prg_type == FLASH_TYPEPROGRAM_WORD)
		{
			flash_program_word(address,(uint32_t)data);
		}
		else
		{
			flash_program_doubleword(address,data);
		}
		status = flash_wait_for_last_operation(FLASH_TIMEOUT_VALUE);
		/* Leave programming mode after the write completes. */
		FLASH->CR &=~FLASH_CR_PG;
	}
	return status;
}
/**
 * @brief Program one byte at the requested Flash address.
 * @param address Destination Flash address.
 * @param data Byte to program.
 */
void flash_program_byte(uint32_t address, uint8_t data)
{
    /* Check the flash status and address range before continuing. */
	FLASH->CR &=~FLASH_CR_PSIZE;
	FLASH->CR |=FLASH_PSIZE_BYTE;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint8_t *)address =  data;
}
/**
 * @brief Program one 16-bit halfword at the requested Flash address.
 * @param address Destination Flash address.
 * @param data Halfword to program.
 */
void flash_program_halfword(uint32_t address, uint16_t data)
{
    /* Check the flash status and address range before continuing. */
	FLASH->CR &=~FLASH_CR_PSIZE;
	FLASH->CR |=FLASH_PSIZE_HALF_WORD;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint16_t *)address =  data;
}
/**
 * @brief Program one 32-bit word at the requested Flash address.
 * @param address Destination Flash address.
 * @param data Word to program.
 */
void flash_program_word(uint32_t address, uint32_t data)
{
    /* Check the flash status and address range before continuing. */
	FLASH->CR &=~FLASH_CR_PSIZE;
	FLASH->CR |=FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint32_t *)address =  data;
}
/**
 * @brief Program one 64-bit value as two consecutive Flash words.
 * @param address Destination Flash address.
 * @param data 64-bit value to program.
 */
void flash_program_doubleword(uint32_t address, uint64_t data)
{
    /* Check the flash status and address range before continuing. */
	FLASH->CR &=~FLASH_CR_PSIZE;
	FLASH->CR |=FLASH_PSIZE_DOUBLE_WORD;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint32_t *)address =  (uint32_t)data;
	__ISB();
	*(__IO uint32_t *)(address+4) =  (uint32_t)(data>>32);
}
/* Reset enabled instruction/data caches after Flash erase operations. */
static void flush_caches(void)
{
    /* Check the return status before continuing to the next step. */
	  if (READ_BIT(FLASH->ACR, FLASH_ACR_ICEN) != RESET)
	  {
			  FLASH->ACR &= (~FLASH_ACR_ICEN);
			  FLASH->ACR |= FLASH_ACR_ICRST;
			 FLASH->ACR &= ~FLASH_ACR_ICRST;
			 FLASH->ACR |= FLASH_ACR_ICEN;
	  }
	  if (READ_BIT(FLASH->ACR, FLASH_ACR_DCEN) != RESET)
	  {
		  FLASH->ACR &= (~FLASH_ACR_DCEN);
	    FLASH->ACR |= FLASH_ACR_DCRST;
	    FLASH->ACR &= ~FLASH_ACR_DCRST;
	    FLASH->ACR |= FLASH_ACR_DCEN;
	  }
}
/* Flash controller state and address helpers. */
/**
 * @brief Unlock the STM32 Flash controller for erase/program operations.
 * @return Flash operation status.
 */
DEV_StatusTypeDef flash_unlock(void)
{
    /* Check the flash status and address range before continuing. */
	DEV_StatusTypeDef status = DEV_OK;
	if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
	{
		FLASH->KEYR  =  FLASH_KEY1;
		FLASH->KEYR  =  FLASH_KEY2;
		if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
		{
			status =  DEV_ERROR;
		}
	}
	return status;
}
/**
 * @brief Return the stored Flash driver error code.
 * @return Current Flash error code.
 */
uint32_t flash_get_error(void)
{
    /* Check the flash status and address range before continuing. */
	return pFlash.ErrorCode;
}
/* Map an internal Flash address to its STM32F4 sector number. */
static uint32_t get_sector(uint32_t address)
{
	uint32_t sector = 0;
	if((address < 0x08003FFF) && (address >= 0x08000000))
	 {
	     sector = FLASH_SECTOR_0;
	 }
   else if((address < 0x08007FFF) && (address >= 0x08004000))
   {
	 sector = FLASH_SECTOR_1;
   }
   else if((address < 0x0800BFFF) && (address >= 0x08008000))
   {
	 sector = FLASH_SECTOR_2;
   }
   else if((address < 0x0800FFFF) && (address >= 0x0800C000))
   {
	 sector = FLASH_SECTOR_3;
   }
   else if((address < 0x0801FFFF) && (address >= 0x08010000))
   {
	 sector = FLASH_SECTOR_4;
   }
   else if((address < 0x0803FFFF) && (address >= 0x08020000))
   {
	 sector = FLASH_SECTOR_5;
   }
   else if((address < 0x0805FFFF) && (address >= 0x08040000))
   {
	 sector = FLASH_SECTOR_6;
   }
   else if((address < 0x0807FFFF) && (address >= 0x08060000))
   {
	 sector = FLASH_SECTOR_7;
   }
   else
   {
     //Do something
   }
	return sector;
}
/* Higher-level Flash read/write helpers. */
/**
 * @brief Erase the required sector range and program a sequence of 32-bit words.
 * @param start_sect_addr Start Flash address.
 * @param data Word buffer to program.
 * @param numberofwords Number of 32-bit words to write.
 * @return 0 on success, otherwise the Flash error code.
 */
uint32_t flash_write_data(uint32_t start_sect_addr,uint32_t *data, uint16_t numberofwords)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t sect_err;
	uint16_t write_count  = 0;
	flash_unlock();
	/* Erase every sector touched by the requested word range. */
	uint32_t start_sector =  get_sector(start_sect_addr);
	uint32_t end_sect_addr =  start_sect_addr + numberofwords * 4;
	uint32_t end_sector =  get_sector(end_sect_addr);
	EraseInitStruct.TypeErase =  FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector       =  start_sector;
	EraseInitStruct.NbSectors    =  (end_sector - start_sector) + 1;
	if(flash_ex_erase(&EraseInitStruct, &sect_err) != DEV_OK )
	{
		return flash_get_error();
	}
	while(write_count < numberofwords )
	{
		if(flash_program(FLASH_TYPEPROGRAM_WORD, start_sect_addr,data[write_count]) == DEV_OK)
		{
			start_sect_addr += 4;
			write_count++;
		}
		else
		{
			return flash_get_error();
		}
	}
	return 0;
}
/**
 * @brief Copy raw bytes directly from memory-mapped Flash.
 * @param start_addr Start Flash address.
 * @param rx_buff Destination byte buffer.
 * @param nbytes Number of bytes to copy.
 */
void flash_read_bytes_memcpy(uint32_t start_addr, uint8_t *rx_buff, uint32_t nbytes)
{
    /* Check the flash status and address range before continuing. */
    memcpy(rx_buff, (const void *)start_addr, nbytes);
}
/**
 * @brief Read 32-bit words from memory-mapped Flash with basic validation.
 * @param start_sect_addr Start Flash address.
 * @param rx_buff Destination word buffer.
 * @param numberofwords Number of 32-bit words to read.
 * @return Flash read status.
 */
int flash_read_data(uint32_t start_sect_addr, uint32_t *rx_buff, uint16_t numberofwords)
{
    if (rx_buff == NULL) {
        return FLASH_READ_ERR_NULL_PTR;
    }
    if (numberofwords == 0) {
        return FLASH_READ_OK;
    }
    /* 32-bit reads require a word-aligned Flash address. */
    if (start_sect_addr & 0x3U) {
        return FLASH_READ_ERR_ALIGNMENT;
    }
    for (uint16_t i = 0; i < numberofwords; ++i) {
        rx_buff[i] = *(__IO uint32_t *)(start_sect_addr + ((uint32_t)i * 4U));
    }
    return FLASH_READ_OK;
}
/* Small data-conversion helpers used by Flash storage. */
/**
 * @brief Convert one float into its four-byte representation.
 * @param rsult_buff Destination four-byte buffer.
 * @param value Float value to convert.
 */
void float_to_bytes(uint8_t * rsult_buff, float value)
{
    /* Check the return status before continuing to the next step. */
	union{
		float float_value;
		uint8_t bytes_value[4];
	}data_t;
	data_t.float_value = value;
	for( int i = 0; i < 4; i++)
	{
		rsult_buff[i] = data_t.bytes_value[i];
	}
}
/**
 * @brief Rebuild one float from four raw bytes.
 * @param value Four-byte float representation.
 * @return Decoded float value.
 */
float bytes_to_float(uint8_t * value)
{
    /* Check the return status before continuing to the next step. */
	float result;
	union{
		float float_value;
		uint8_t bytes_value[4];
	}data_t;
	for(int i = 0; i < 4; i++)
	{
		data_t.bytes_value[i] =  value[i];
	}
	result =  data_t.float_value;
	return result;
}
/**
 * @brief Copy packed 32-bit source data into a byte-oriented string buffer.
 * @param src_data Packed source words.
 * @param dest_buff Destination character buffer.
 */
 void get_str(uint32_t *src_data, char *dest_buff)
 {
    /* Check the return status before continuing to the next step. */
	 int numberofbytes =  ((strlen((char *)src_data)/4) + ((strlen((char *)src_data) %4) != 0))*4;
	for( int i = 0 ; i < numberofbytes; i++ )
	{
		int word_idx =  i / sizeof(uint32_t);
		int byte_idx =  i % sizeof(uint32_t);
		dest_buff[i] =  (src_data[word_idx] >> (8 * byte_idx)) & 0xff;
	}
 }
/**
 * @brief Store one float in Flash using the generic word-write path.
 * @param start_sect_addr Destination Flash address.
 * @param num Float value to store.
 */
 void flash_write_num(uint32_t start_sect_addr, float num)
 {
    /* Check the flash status and address range before continuing. */
	 float_to_bytes(temp_buff, num);
	 flash_write_data(start_sect_addr,(uint32_t *)temp_buff, 1);
 }
/**
 * @brief Read one float previously stored in Flash.
 * @param start_sect_addr Source Flash address.
 * @return Decoded float value.
 */
 float flash_read_num(uint32_t start_sect_addr)
 {
    /* Check the flash status and address range before continuing. */
	 uint8_t buff[4];
	 float value;
	 flash_read_data(start_sect_addr,(uint32_t *)buff,1);
	 value =  bytes_to_float(buff);
	 return value;
 }
