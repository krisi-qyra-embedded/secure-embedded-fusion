/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#ifndef __SPI_DMA_H__
#define __SPI_DMA_H__

#include <stdint.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SPI_DMA_STATUS_OK                     = 0,
    SPI_DMA_STATUS_INVALID_ARG            = -1,
    SPI_DMA_STATUS_STREAM_DISABLE_TIMEOUT = -2,
    SPI_DMA_STATUS_INIT_FAILED            = -3,
    SPI_DMA_STATUS_CONFIG_MISMATCH        = -4,
    SPI_DMA_STATUS_TIMEOUT                = -5,
    SPI_DMA_STATUS_TRANSFER_ERROR         = -6,
    SPI_DMA_STATUS_RECOVERY_FAILED        = -7,
    SPI_DMA_STATUS_DEVICE_REINIT_FAILED   = -8,
    SPI_DMA_STATUS_SELFTEST_FAILED        = -9
} spi_dma_status_t;

typedef enum
{
    SPI_DMA_TRANSFER_IDLE    = 0u,
    SPI_DMA_TRANSFER_SUCCESS = 1u,
    SPI_DMA_TRANSFER_ERROR   = 2u
} spi_dma_transfer_state_t;

typedef struct
{
    volatile uint32_t rx_tc_count;
    volatile uint32_t tx_tc_count;
    volatile uint32_t rx_err_count;
    volatile uint32_t tx_err_count;
    volatile uint32_t rx_ht_count;
    volatile uint32_t tx_ht_count;
} spi_dma_diag_t;

spi_dma_status_t spi4_dma_init(void);

spi_dma_status_t spi4_dma_transmit(uint8_t *tx_buf, uint32_t len);
spi_dma_status_t spi4_dma_receive(uint8_t *rx_buf, uint32_t len);
spi_dma_status_t spi4_dma_transmit_receive(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);

spi_dma_status_t spi4_recover(void);
spi_dma_status_t spi4_restore_and_init(void);

void spi4_flush_rx(void);
void spi4_check_and_clear_ovr(void);

spi_dma_status_t spi_dma_stream0_wait_complete(uint32_t timeout_ms);
spi_dma_status_t spi_dma_stream1_wait_complete(uint32_t timeout_ms);
spi_dma_status_t spi4_wait_bsy_clear_ms(uint32_t timeout_ms);

spi_dma_status_t spi4_set_baudrate_prescaler(uint8_t br_code);

extern volatile spi_dma_transfer_state_t max31865_spi_dma_rx_cmplt;
extern volatile spi_dma_transfer_state_t max31865_spi_dma_tx_cmplt;
extern volatile uint32_t hb_spi_ts;
extern volatile uint8_t tim2_1s_flag;

uint8_t tim2_1s_check_flag(void);

extern spi_dma_diag_t g_spi_dma_diag;

#ifdef __cplusplus
}
#endif

#endif /* __SPI_DMA_H__ */
