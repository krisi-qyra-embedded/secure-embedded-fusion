/**
 * @file onewire.c
 * @brief Blocking 1-Wire bus handling and ROM search.
 */
#include "onewire.h"
#include "tim.h"
#include "tim3.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
/* Check OneWire GPIO configuration. */
static uint8_t OneWire_IsConfigValid(const OneWire_t *OW)
{
    return ((OW != NULL) &&
            (OW->DataPort != NULL) &&
            (OW->DataPin != 0U)) ? 1U : 0U;
}
/* Switch the data pin between input and open-drain output. */
static void OneWire_Pin_Mode(OneWire_t *OW, PinMode Mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = OW->DataPin;
    if (Mode == Input)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    else
    {
        /* Open-drain lets the external pull-up release the bus high. */
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    }
    HAL_GPIO_Init(OW->DataPort, &GPIO_InitStruct);
}
/* Drive the 1-Wire data pin. */
static void OneWire_Pin_Level(OneWire_t *OW, uint8_t Level)
{
    HAL_GPIO_WritePin(OW->DataPort,
                      OW->DataPin,
                      (Level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
/* Read current 1-Wire bus level. */
static uint8_t OneWire_Pin_Read(OneWire_t *OW)
{
    return (HAL_GPIO_ReadPin(OW->DataPort, OW->DataPin) == GPIO_PIN_SET) ? 1U : 0U;
}
/* Protect one timing-sensitive 1-Wire slot. */
static void OneWire_EnterCritical(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        taskENTER_CRITICAL();
    }
    else
    {
        __disable_irq();
    }
}
/* Leave the timing-critical section. */
static void OneWire_ExitCritical(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        taskEXIT_CRITICAL();
    }
    else
    {
        __enable_irq();
    }
}
/* Write one 1-Wire bit using blocking timing. */
static void OneWire_WriteBitBlocking(OneWire_t *OW, uint8_t bit)
{
	OneWire_EnterCritical();
    /* Write-1 uses a short low pulse, write-0 keeps the bus low longer. */
    if (bit != 0U)
    {
        OneWire_Pin_Level(OW, 0U);
        OneWire_Pin_Mode(OW, Output);
        delay_us(6U);
        OneWire_Pin_Mode(OW, Input);
        delay_us(64U);
    }
    else
    {
        OneWire_Pin_Level(OW, 0U);
        OneWire_Pin_Mode(OW, Output);
        delay_us(60U);
        OneWire_Pin_Mode(OW, Input);
        delay_us(10U);
    }
    OneWire_ExitCritical();
}
/**
 * @brief Read one bit from the 1-Wire bus.
 * @param OW OneWire context.
 * @param bit Read bit output.
 * @return OneWire status.
 */
OneWire_Status_t OneWire_ReadBit(OneWire_t *OW, uint8_t *bit)
{
    uint8_t sample;
    if ((OneWire_IsConfigValid(OW) == 0U) || (bit == NULL))
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }
    OneWire_EnterCritical();
    /* Start the read slot, then release the bus before sampling. */
    OneWire_Pin_Level(OW, 0U);
    OneWire_Pin_Mode(OW, Output);
    delay_us(3U);
    OneWire_Pin_Mode(OW, Input);
    delay_us(10U);
    sample = OneWire_Pin_Read(OW);
    delay_us(50U);
    OneWire_ExitCritical();
    *bit = sample;
    return ONEWIRE_STATUS_OK;
}
/**
 * @brief Write one byte on the 1-Wire bus.
 * @param OW OneWire context.
 * @param byte Byte to send.
 * @return OneWire status.
 */
OneWire_Status_t OneWire_WriteByte(OneWire_t *OW, uint8_t byte)
{
    uint8_t bit = 8U;
    if (OneWire_IsConfigValid(OW) == 0U)
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }
    while (bit-- != 0U)
    {
        OneWire_WriteBitBlocking(OW, (uint8_t)(byte & 0x01U));
        byte >>= 1U;
    }
    return ONEWIRE_STATUS_OK;
}
/**
 * @brief Read one byte from the 1-Wire bus.
 * @param OW OneWire context.
 * @param byte Read byte output.
 * @return OneWire status.
 */
OneWire_Status_t OneWire_ReadByte(OneWire_t *OW, uint8_t *byte)
{
    uint8_t bit = 8U;
    uint8_t local = 0U;
    uint8_t sample;
    OneWire_Status_t st;
    if ((OneWire_IsConfigValid(OW) == 0U) || (byte == NULL))
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }
    while (bit-- != 0U)
    {
        local >>= 1U;
        st = OneWire_ReadBit(OW, &sample);
        if (st != ONEWIRE_STATUS_OK)
        {
            return st;
        }
        local |= (uint8_t)(sample << 7U);
    }
    *byte = local;
    return ONEWIRE_STATUS_OK;
}
/**
 * @brief Send reset pulse and check for device presence.
 * @param OW OneWire context.
 * @return OneWire status.
 */
OneWire_Status_t OneWire_Reset(OneWire_t *OW)
{
    uint8_t presence_sample;
    if (OneWire_IsConfigValid(OW) == 0U)
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }
    OneWire_EnterCritical();
    /* Reset pulse holds the bus low long enough for presence detection. */
    OneWire_Pin_Level(OW, 0U);
    OneWire_Pin_Mode(OW, Output);
    delay_us(480U);
    OneWire_Pin_Mode(OW, Input);
    delay_us(70U);
    /* A low level here means at least one device answered. */
    presence_sample = OneWire_Pin_Read(OW);
    delay_us(410U);
    OneWire_ExitCritical();
    return (presence_sample == 0U) ? ONEWIRE_STATUS_OK : ONEWIRE_STATUS_NO_DEVICE;
}
/**
 * @brief Search for the next device ROM on the 1-Wire bus.
 * @param OW OneWire context.
 * @param Cmd Search command.
 * @return OneWire status.
 */
OneWire_Status_t OneWire_Search(OneWire_t *OW, uint8_t Cmd)
{
    uint8_t id_bit_number   = 1U;
    uint8_t last_zero       = 0U;
    uint8_t rom_byte_number = 0U;
    uint8_t search_result   = 0U;
    uint8_t rom_byte_mask   = 1U;
    uint8_t id_bit;
    uint8_t cmp_id_bit;
    uint8_t search_direction;
    OneWire_Status_t st;
    if (OneWire_IsConfigValid(OW) == 0U)
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }
    if (OW->LastDeviceFlag != 0U)
    {
        return ONEWIRE_STATUS_NOT_FOUND;
    }
    /* Search starts from a fresh reset/presence cycle. */
    st = OneWire_Reset(OW);
    if (st != ONEWIRE_STATUS_OK)
    {
        OW->LastDiscrepancy = 0U;
        OW->LastDeviceFlag = 0U;
        OW->LastFamilyDiscrepancy = 0U;
        return ONEWIRE_STATUS_NO_DEVICE;
    }
    st = OneWire_WriteByte(OW, Cmd);
    if (st != ONEWIRE_STATUS_OK)
    {
        return st;
    }
    do
    {
        /* Read ROM bit and its complement to resolve each search branch. */
        st = OneWire_ReadBit(OW, &id_bit);
        if (st != ONEWIRE_STATUS_OK)
        {
            return st;
        }
        st = OneWire_ReadBit(OW, &cmp_id_bit);
        if (st != ONEWIRE_STATUS_OK)
        {
            return st;
        }
        /* 1/1 means no device is participating in this search position. */
        if ((id_bit == 1U) && (cmp_id_bit == 1U))
        {
            break;
        }
        else
        {
            if (id_bit != cmp_id_bit)
            {
                search_direction = id_bit;
            }
            else
            {
                if (id_bit_number < OW->LastDiscrepancy)
                {
                    search_direction = ((OW->RomByte[rom_byte_number] & rom_byte_mask) != 0U) ? 1U : 0U;
                }
                else
                {
                    search_direction = (id_bit_number == OW->LastDiscrepancy) ? 1U : 0U;
                }
                /* Remember the last zero branch so the next search can continue there. */
                if (search_direction == 0U)
                {
                    last_zero = id_bit_number;
                    if (last_zero < 9U)
                    {
                        OW->LastFamilyDiscrepancy = last_zero;
                    }
                }
            }
            if (search_direction == 1U)
            {
                OW->RomByte[rom_byte_number] |= rom_byte_mask;
            }
            else
            {
                OW->RomByte[rom_byte_number] &= (uint8_t)(~rom_byte_mask);
            }
            /* Tell all devices which ROM branch remains active. */
            OneWire_WriteBitBlocking(OW, search_direction);
            id_bit_number++;
            rom_byte_mask <<= 1U;
            if (rom_byte_mask == 0U)
            {
                rom_byte_number++;
                rom_byte_mask = 1U;
            }
        }
    } while (rom_byte_number < 8U);
    /* All 64 ROM bits were resolved. */
    if (id_bit_number >= 65U)
    {
        OW->LastDiscrepancy = last_zero;
        if (OW->LastDiscrepancy == 0U)
        {
            OW->LastDeviceFlag = 1U;
        }
        search_result = 1U;
    }
    if ((search_result == 0U) || (OW->RomByte[0] == 0U))
    {
        OW->LastDiscrepancy = 0U;
        OW->LastDeviceFlag = 0U;
        OW->LastFamilyDiscrepancy = 0U;
        return ONEWIRE_STATUS_NOT_FOUND;
    }
    return ONEWIRE_STATUS_OK;
}
/**
 * @brief Copy the current discovered ROM code.
 * @param OW OneWire context.
 * @param ROM Destination buffer.
 * @return OneWire status.
 */
OneWire_Status_t OneWire_GetDevRom(OneWire_t *OW, uint8_t *ROM)
{
    uint8_t i;
    if ((OW == NULL) || (ROM == NULL))
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }
    for (i = 0U; i < 8U; i++)
    {
        ROM[i] = OW->RomByte[i];
    }
    return ONEWIRE_STATUS_OK;
}
/**
 * @brief Initialize the 1-Wire bus state.
 * @param OW OneWire context.
 * @return OneWire status.
 */
OneWire_Status_t OneWire_Init(OneWire_t *OW)
{
    /* Re-arm the bus and clear previous ROM-search state. */
    if (OneWire_IsConfigValid(OW) == 0U)
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }
    OneWire_Pin_Mode(OW, Output);
    OneWire_Pin_Level(OW, 1U);
    delay_us(1000U);
    OneWire_Pin_Level(OW, 0U);
    delay_us(1000U);
    OneWire_Pin_Level(OW, 1U);
    OneWire_Pin_Mode(OW, Input);
    delay_us(2000U);
    OW->LastDiscrepancy = 0U;
    OW->LastDeviceFlag = 0U;
    OW->LastFamilyDiscrepancy = 0U;
    OW->RomCnt = 0U;
    memset(OW->RomByte, 0, sizeof(OW->RomByte));
    OW->IrqDone = 0U;
    OW->ReadSample = 1U;
    OW->ResetPresence = 1U;
    OW->WriteBitValue = 0U;
    OW->OpPhase = 0U;
    OW->ActiveOp = ONEWIRE_OP_NONE;
    return ONEWIRE_STATUS_OK;
}
/**
 * @brief Select one device by ROM code.
 * @param OW OneWire context.
 * @param ROM Device ROM code.
 * @return OneWire status.
 */
OneWire_Status_t OneWire_SelectWithPointer(OneWire_t *OW, const uint8_t *ROM)
{
    uint8_t i;
    OneWire_Status_t st;
    if ((OW == NULL) || (ROM == NULL))
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }
    /* MATCH ROM selects one device before sending its 64-bit address. */
    st = OneWire_WriteByte(OW, ONEWIRE_CMD_MATCHROM);
    if (st != ONEWIRE_STATUS_OK)
    {
        return st;
    }
    for (i = 0U; i < 8U; i++)
    {
        st = OneWire_WriteByte(OW, ROM[i]);
        if (st != ONEWIRE_STATUS_OK)
        {
            return st;
        }
    }
    return ONEWIRE_STATUS_OK;
}
/* Compatibility callback. Blocking driver does not use timer IRQs. */
void OneWire_TimerElapsedCallback(OneWire_t *OW, TIM_HandleTypeDef *htim)
{
    (void)OW;
    (void)htim;
}
/**
 * @brief Calculate Dallas/Maxim CRC-8.
 * @param Addr Input data.
 * @param Len Number of bytes.
 * @return CRC-8 value.
 */
uint8_t OneWire_CRC8(const uint8_t *Addr, uint8_t Len)
{
    uint8_t crc = 0U;
    uint8_t inbyte;
    uint8_t i;
    uint8_t mix;
    while (Len-- != 0U)
    {
        inbyte = *Addr++;
        for (i = 8U; i != 0U; i--)
        {
            /* Dallas/Maxim CRC uses polynomial 0x8C in reflected form. */
            mix = (uint8_t)((crc ^ inbyte) & 0x01U);
            crc >>= 1U;
            crc ^= (mix != 0U) ? 0x8CU : 0x00U;
            inbyte >>= 1U;
        }
    }
    return crc;
}
