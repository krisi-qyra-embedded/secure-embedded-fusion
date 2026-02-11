/* pm2_5.h | Author: i fiksuari pas prog | Date: 11 February 2026 */
#ifndef PM2_5_H
#define PM2_5_H

#include <stdint.h>

typedef enum
{
    PMS25_STATUS_OK = 0U,
    PMS25_STATUS_INVALID_PARAM,
    PMS25_STATUS_TIMEOUT,
    PMS25_STATUS_UART_ERROR,
    PMS25_STATUS_SYNC_ERROR,
    PMS25_STATUS_LENGTH_ERROR,
    PMS25_STATUS_CHECKSUM_ERROR
} PMS25_Status_t;

typedef struct
{
    uint16_t frame_len;

    uint16_t pm1_0_standard;
    uint16_t pm2_5_standard;
    uint16_t pm10_standard;

    uint16_t pm1_0_atmospheric;
    uint16_t pm2_5_atmospheric;
    uint16_t pm10_atmospheric;

    uint16_t particles_0_3um;
    uint16_t particles_0_5um;
    uint16_t particles_1_0um;
    uint16_t particles_2_5um;
    uint16_t particles_5_0um;
    uint16_t particles_10um;

    uint16_t reserved;
    uint16_t checksum;
} PMS25_Data_t;

PMS25_Status_t PMS25_Read(PMS25_Data_t *out, uint32_t timeout_ms);

#endif /* PM2_5_H */
