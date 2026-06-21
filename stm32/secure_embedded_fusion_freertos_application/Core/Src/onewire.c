/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#include "onewire.h"
#include "tim.h"
#include "tim3.h"
#include <string.h>

/* Checks the OneWire GPIO configuration. */
static uint8_t OneWire_IsConfigValid(const OneWire_t *OW)
{
    /* Require a bus object, GPIO port and nonzero pin mask. */
    return ((OW != NULL) &&
            (OW->DataPort != NULL) &&
            (OW->DataPin != 0U)) ? 1U : 0U;
}

/* Switches the OneWire pin between input and open-drain output. */
static void OneWire_Pin_Mode(OneWire_t *OW, PinMode Mode)
{
    /* Release the bus by selecting input mode without an internal pull-up. */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = OW->DataPin;

    if (Mode == Input)
    {
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    else
    {
        /* Correct 1-Wire drive mode: MCU only pulls low; external resistor pulls high. */
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    }

    HAL_GPIO_Init(OW->DataPort, &GPIO_InitStruct);
}

/* Sets the OneWire output latch. */
static void OneWire_Pin_Level(OneWire_t *OW, uint8_t Level)
{
    /* Use the supplied port and pin rather than a fixed bus pin. */
    HAL_GPIO_WritePin(OW->DataPort,
                      OW->DataPin,
                      (Level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Samples the OneWire bus level. */
static uint8_t OneWire_Pin_Read(OneWire_t *OW)
{
    /* Map the GPIO reading to the bit value used by the bus protocol. */
    return (HAL_GPIO_ReadPin(OW->DataPort, OW->DataPin) == GPIO_PIN_SET) ? 1U : 0U;
}

/* Writes one OneWire bit with timer-based slot timing. */
static void OneWire_WriteBitBlocking(OneWire_t *OW, uint8_t bit)
{
    /* Use a short low pulse for one and a longer low pulse for zero. */
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
}

/* Reads one OneWire bit with timer-based slot timing. */
OneWire_Status_t OneWire_ReadBit(OneWire_t *OW, uint8_t *bit)
{
    /* Release the line before sampling the device's response. */
    uint8_t sample;

    if ((OneWire_IsConfigValid(OW) == 0U) || (bit == NULL))
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }

    OneWire_Pin_Level(OW, 0U);
    OneWire_Pin_Mode(OW, Output);
    delay_us(3U);

    OneWire_Pin_Mode(OW, Input);
    delay_us(10U);

    sample = OneWire_Pin_Read(OW);

    delay_us(50U);

    *bit = sample;
    return ONEWIRE_STATUS_OK;
}

/* Sends a byte on the OneWire bus. */
OneWire_Status_t OneWire_WriteByte(OneWire_t *OW, uint8_t byte)
{
    /* Transmit the least-significant bit first. */
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

/* Receives a byte from the OneWire bus. */
OneWire_Status_t OneWire_ReadByte(OneWire_t *OW, uint8_t *byte)
{
    /* Insert each successive bit into the byte while shifting the prior bits. */
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

/* Issues a OneWire reset and checks the presence pulse. */
OneWire_Status_t OneWire_Reset(OneWire_t *OW)
{
    /* Sample the released bus after the reset low interval. */
    uint8_t presence_sample;

    if (OneWire_IsConfigValid(OW) == 0U)
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }

    OneWire_Pin_Level(OW, 0U);
    OneWire_Pin_Mode(OW, Output);
    delay_us(480U);

    OneWire_Pin_Mode(OW, Input);
    delay_us(70U);

    presence_sample = OneWire_Pin_Read(OW);

    delay_us(410U);

    return (presence_sample == 0U) ? ONEWIRE_STATUS_OK : ONEWIRE_STATUS_NO_DEVICE;
}

/* Finds the next ROM using the OneWire search tree. */
OneWire_Status_t OneWire_Search(OneWire_t *OW, uint8_t Cmd)
{
    /* Retain the last discrepancy to select the next search branch. */
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

/* Copies the most recently discovered OneWire ROM. */
OneWire_Status_t OneWire_GetDevRom(OneWire_t *OW, uint8_t *ROM)
{
    /* Copy all eight ROM bytes into the caller's destination. */
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

/* Releases the OneWire bus and resets discovery state. */
OneWire_Status_t OneWire_Init(OneWire_t *OW)
{
    /* Clear discrepancy and operation state after the bus settling sequence. */
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

/* Selects a OneWire device by its ROM. */
OneWire_Status_t OneWire_SelectWithPointer(OneWire_t *OW, const uint8_t *ROM)
{
    /* Send MATCH ROM followed by the eight address bytes. */
    uint8_t i;
    OneWire_Status_t st;

    if ((OW == NULL) || (ROM == NULL))
    {
        return ONEWIRE_STATUS_BAD_PARAM;
    }

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

/* Retains the unused OneWire timer callback entry point. */
void OneWire_TimerElapsedCallback(OneWire_t *OW, TIM_HandleTypeDef *htim)
{
    /* The blocking implementation does not consume timer callbacks. */
    (void)OW;
    (void)htim;
}

/* Calculates the OneWire CRC over a byte sequence. */
uint8_t OneWire_CRC8(const uint8_t *Addr, uint8_t Len)
{
    /* Apply the reflected 0x8C polynomial one bit at a time. */
    uint8_t crc = 0U;
    uint8_t inbyte;
    uint8_t i;
    uint8_t mix;

    while (Len-- != 0U)
    {
        inbyte = *Addr++;

        for (i = 8U; i != 0U; i--)
        {
            mix = (uint8_t)((crc ^ inbyte) & 0x01U);
            crc >>= 1U;
            crc ^= (mix != 0U) ? 0x8CU : 0x00U;
            inbyte >>= 1U;
        }
    }

    return crc;
}
