/* pm2_5.c | Author: i fiksuari pas prog | Date: 11 February 2026 */
///*
// * pm2_5.c
// *
// * PMS5003 / PMSA003 style frame reader
// */
//
//#include "pm2_5.h"
//#include "uart.h"
//#include "stm32f4xx_hal.h"
//#include "uart2.h"
//
//#define PMS25_FRAME_SIZE         32U
//#define PMS25_HEADER_BYTE_0      0x42U
//#define PMS25_HEADER_BYTE_1      0x4DU
//#define PMS25_EXPECTED_FRAME_LEN 28U
//
//static uint16_t PMS25_ReadBE16(const uint8_t *p)
//{
//    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
//}
//
//static PMS25_Status_t PMS25_MapUartStatus(UART_Status_t uart_st)
//{
//    switch (uart_st)
//    {
//        case UART_STATUS_TIMEOUT:
//            return PMS25_STATUS_TIMEOUT;
//
//        case UART_STATUS_OK:
//            return PMS25_STATUS_OK;
//
//        default:
//            return PMS25_STATUS_UART_ERROR;
//    }
//}
//
//PMS25_Status_t PMS25_Read(PMS25_Data_t *out, uint32_t timeout_ms)
//{
//    uint8_t frame[PMS25_FRAME_SIZE];
//    uint8_t first_byte;
//    uint16_t checksum_calc = 0U;
//    uint16_t checksum_rx;
//    uint32_t start_tick;
//    uint32_t elapsed;
//    uint32_t remaining;
//    UART_Status_t uart_st;
//    uint8_t i;
//
//    if ((out == NULL) || (timeout_ms == 0U))
//    {
//        return PMS25_STATUS_INVALID_PARAM;
//    }
//
//    start_tick = HAL_GetTick();
//
//    /* Sync on 0x42, then read the remaining 31 bytes */
//    while (1)
//    {
//        elapsed = HAL_GetTick() - start_tick;
//        if (elapsed >= timeout_ms)
//        {
//            return PMS25_STATUS_TIMEOUT;
//        }
//
//        remaining = timeout_ms - elapsed;
//
//        uart_st = UART1_RX_DMA_Receive(&first_byte, 1U, remaining);
//        if (uart_st != UART_STATUS_OK)
//        {
//            return PMS25_MapUartStatus(uart_st);
//        }
//
//        if (first_byte != PMS25_HEADER_BYTE_0)
//        {
//            continue;
//        }
//
//        frame[0] = first_byte;
//
//        elapsed = HAL_GetTick() - start_tick;
//        if (elapsed >= timeout_ms)
//        {
//            return PMS25_STATUS_TIMEOUT;
//        }
//
//        remaining = timeout_ms - elapsed;
//
//        uart_st = UART1_RX_DMA_Receive(&frame[1], 31U, remaining);
//        if (uart_st != UART_STATUS_OK)
//        {
//            return PMS25_MapUartStatus(uart_st);
//        }
//
//        if (frame[1] != PMS25_HEADER_BYTE_1)
//        {
//            continue;
//        }
//
//        break;
//    }
//
//    for (i = 0U; i < 30U; i++)
//    {
//        checksum_calc = (uint16_t)(checksum_calc + frame[i]);
//    }
//
//    checksum_rx = PMS25_ReadBE16(&frame[30]);
//
//    if (checksum_calc != checksum_rx)
//    {
//        return PMS25_STATUS_CHECKSUM_ERROR;
//    }
//
//    out->frame_len = PMS25_ReadBE16(&frame[2]);
//    if (out->frame_len != PMS25_EXPECTED_FRAME_LEN)
//    {
//        return PMS25_STATUS_LENGTH_ERROR;
//    }
//
//    out->pm1_0_standard      = PMS25_ReadBE16(&frame[4]);
//    out->pm2_5_standard      = PMS25_ReadBE16(&frame[6]);
//    out->pm10_standard       = PMS25_ReadBE16(&frame[8]);
//
//    out->pm1_0_atmospheric   = PMS25_ReadBE16(&frame[10]);
//    out->pm2_5_atmospheric   = PMS25_ReadBE16(&frame[12]);
//    out->pm10_atmospheric    = PMS25_ReadBE16(&frame[14]);
//
//    out->particles_0_3um     = PMS25_ReadBE16(&frame[16]);
//    out->particles_0_5um     = PMS25_ReadBE16(&frame[18]);
//    out->particles_1_0um     = PMS25_ReadBE16(&frame[20]);
//    out->particles_2_5um     = PMS25_ReadBE16(&frame[22]);
//    out->particles_5_0um     = PMS25_ReadBE16(&frame[24]);
//    out->particles_10um      = PMS25_ReadBE16(&frame[26]);
//
//    out->reserved            = PMS25_ReadBE16(&frame[28]);
//    out->checksum            = checksum_rx;
//
//    return PMS25_STATUS_OK;
//}
