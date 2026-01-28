/* adc_dma.c | Author: i fiksuari mbas prog | Date: 28 January 2026 */
#ifndef __I2C_DMA_H__
#define __I2C_DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx.h"

/* Multi-sensor I2C DMA driver: BME680: I2C1_SCL -> PB8 AF4 I2C1_SDA -> PB9 AF4 I2C1_RX -> DMA1 Stream5. */

/* DMA flag helper masks */
/* These are convenience masks for transfer-complete flag checks/clears. */

#define DMA1_STREAM1_TCIF        (1U << 11)
#define DMA1_STREAM2_TCIF        (1U << 21)
#define DMA1_STREAM4_TCIF        (1U << 5)
#define DMA1_STREAM5_TCIF        (1U << 11)
#define DMA1_STREAM6_TCIF        (1U << 21)
#define DMA1_STREAM7_TCIF        (1U << 27)

/* Public status types */

typedef enum
{
    I2C_DMA_OK = 0,
    I2C_DMA_ERR_NULL_BUFFER,
    I2C_DMA_ERR_UNSUPPORTED_INSTANCE,
    I2C_DMA_ERR_ZERO_LEN,
    I2C_DMA_ERR_BUSY_TIMEOUT,
    I2C_DMA_ERR_SB_TIMEOUT,
    I2C_DMA_ERR_ADDR_TIMEOUT,
    I2C_DMA_ERR_TXE_BTF_TIMEOUT,
    I2C_DMA_ERR_DMA_START
} i2c_dma_status_t;

typedef enum
{
    I2C_DMA_INIT_OK              = 0,
    I2C_DMA_INIT_ERR_I2C_SELECT  = -1,
    I2C_DMA_INIT_ERR_DMA_RX_INIT = -2,
    I2C_DMA_INIT_ERR_DMA_TX_INIT = -3
} i2c_dma_init_status_t;

typedef enum
{
    DMA_OK = 0,
    DMA_ERR_DISABLE_TIMEOUT
} dma_status_t;

typedef enum
{
    DMA_DIR_P2M = 0, 
    DMA_DIR_M2P = 1  
} dma_direction_t;

typedef enum
{
    I2C_SELECT_OK              = 0,
    I2C_SELECT_ERR_INVALID_ARG = -1,
    I2C_SELECT_ERR_PE_NOT_SET  = -2
} i2c_select_status_t;

typedef enum
{
    I2C_DMA_TRANSFER_IDLE     = 0U,
    I2C_DMA_TRANSFER_COMPLETE = 1U,
    I2C_DMA_TRANSFER_ERROR    = 2U
} i2c_dma_transfer_state_t;

typedef enum
{
    I2C_RECOVER_FAIL = 0,
    I2C_RECOVER_OK   = 1
} i2c_recover_status_t;

/* BME680 completion flags */
/* BME680 uses I2C1: RX complete/error flag is driven by DMA1_Stream5_IRQHandler. */
extern volatile i2c_dma_transfer_state_t bme680g_rx_cmplt;
extern volatile i2c_dma_transfer_state_t bme680g_tx_cmplt;


/* MPU9250 uses I2C3: RX complete/error flag is driven by DMA1_Stream1_IRQHandler. */
extern volatile i2c_dma_transfer_state_t mpu9250g_rx_cmplt;
extern volatile i2c_dma_transfer_state_t mpu9250g_tx_cmplt;



i2c_select_status_t i2c_init_select(uint8_t i2c_num,
                                    uint8_t scl_pin,
                                    uint8_t sda_pin);

i2c_dma_status_t i2c_dma_read(I2C_TypeDef *I2Cx,
                              uint8_t slave_addr,
                              uint8_t reg_addr,
                              uint8_t *p_read_buff,
                              uint16_t num_of_bytes);

i2c_dma_status_t i2c_dma_write_reg(I2C_TypeDef *I2Cx,
                                   uint8_t slave_addr_7bit,
                                   uint8_t reg_addr,
                                   const uint8_t *data,
                                   uint16_t data_len);

dma_status_t dma_stream_init(DMA_Stream_TypeDef *stream,
                             uint32_t channel,
                             dma_direction_t dir,
                             volatile uint32_t *flag_reg,
                             uint32_t flag_mask,
                             IRQn_Type irq);

/* Sensor-specific init APIs */

i2c_dma_init_status_t i2c_dma_bme680_peripherals_init(void);

i2c_dma_init_status_t i2c_dma_mpu9250_peripherals_init(void);

/* Recovery API */

i2c_recover_status_t i2c_recover(I2C_TypeDef *I2Cx,
                                 volatile uint32_t *status_reg,
                                 uint32_t wait_mask,
                                 uint8_t expect_set,
                                 uint8_t allow_restart);

#ifdef __cplusplus
}
#endif

#endif 
