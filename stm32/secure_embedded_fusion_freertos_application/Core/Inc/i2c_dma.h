/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#ifndef __I2C_DMA_H__
#define __I2C_DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "uart.h"


#define DMA1_STREAM5_TCIF (1U << 11)   // Transfer complete flag for Stream5
#define DMA1_STREAM6_TCIF (1U << 21)   // Transfer complete flag for Stream6
#define DMA1_STREAM7_TCIF (1U << 27)
#define DMA1_STREAM2_TCIF (1U << 21)
#define DMA1_STREAM1_TCIF (1U << 11)
#define DMA1_STREAM4_TCIF (1U << 5)


#define SYSTEM_FAULT_MASK_BME680      (1UL << 0)
#define SYSTEM_FAULT_MASK_MPU9250     (1UL << 1)
#define SYSTEM_FAULT_MASK_MAX31865    (1UL << 2)
#define SYSTEM_FAULT_MASK_PM25        (1UL << 3)
#define SYSTEM_FAULT_MASK_DS18B20     (1UL << 4)

#define SYSTEM_FAULT_OWNER_NONE       0U
#define SYSTEM_FAULT_OWNER_BME680     1U
#define SYSTEM_FAULT_OWNER_MPU9250    2U
#define SYSTEM_FAULT_OWNER_MAX31865   3U
#define SYSTEM_FAULT_OWNER_PM25       4U
#define SYSTEM_FAULT_OWNER_DS18B20    5U

#define SYSTEM_FAULT_MASK_CRITICAL  \
    (SYSTEM_FAULT_MASK_BME680 | SYSTEM_FAULT_MASK_MPU9250 | SYSTEM_FAULT_MASK_MAX31865 | SYSTEM_FAULT_MASK_PM25 | SYSTEM_FAULT_MASK_DS18B20)

extern volatile uint8_t  system_fault_logging_active;
extern volatile uint8_t  system_fault_owner;
extern volatile uint32_t system_fault_active_mask;
extern volatile uint32_t system_fault_completed_mask;
extern volatile uint32_t system_fault_in_progress_mask;

void system_fault_mark_active(uint8_t owner);
void system_fault_mark_completed(uint8_t owner);
void system_fault_clear_active(uint8_t owner);
void system_fault_mark_active_from_isr(uint8_t owner);
void system_fault_mark_in_progress(uint8_t owner);
uint32_t system_fault_mask_from_owner(uint8_t owner);

extern SemaphoreHandle_t xSerialSemaphore;

#define I2C_DMA_LOG_PRINTF(...)                                                    \
    do {                                                                           \
        if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) &&                 \
            (xSerialSemaphore != NULL)) {                                          \
            if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(100U)) == pdTRUE) {  \
                log_printf(__VA_ARGS__);                                           \
                xSemaphoreGive(xSerialSemaphore);                                  \
            }                                                                      \
        } else {                                                                   \
            log_printf(__VA_ARGS__);                                               \
        }                                                                          \
    } while (0)

/* ----------------------------- Public status types ----------------------------- */

typedef enum {
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

typedef enum {
    I2C_DMA_INIT_OK = 0,
    I2C_DMA_INIT_ERR_I2C_SELECT = -1,
    I2C_DMA_INIT_ERR_DMA_RX_INIT = -2,
    I2C_DMA_INIT_ERR_DMA_TX_INIT = -3
} i2c_dma_init_status_t;

typedef enum {
    DMA_OK = 0,
    DMA_ERR_DISABLE_TIMEOUT
} dma_status_t;

typedef enum {
    DMA_DIR_P2M = 0, /* Peripheral -> Memory */
    DMA_DIR_M2P = 1  /* Memory -> Peripheral */
} dma_direction_t;

typedef enum {
    I2C_SELECT_OK = 0,
    I2C_SELECT_ERR_INVALID_ARG = -1,
    I2C_SELECT_ERR_PE_NOT_SET = -2
} i2c_select_status_t;

typedef enum {
    I2C_DMA_TRANSFER_IDLE = 0U,
    I2C_DMA_TRANSFER_COMPLETE = 1U,
    I2C_DMA_TRANSFER_ERROR = 2U
} i2c_dma_transfer_state_t;

typedef enum
{
    I2C_RECOVER_FAIL = 0,
    I2C_RECOVER_OK   = 1
} i2c_recover_status_t;

/* ----------------------------- I2C DMA completion flags ----------------------------- */
/*
 * These flags belong only to the I2C DMA driver.
 *
 * BME680:
 *   RX -> DMA1_Stream5
 *   TX -> DMA1_Stream6
 *
 * MPU9250:
 *   RX -> DMA1_Stream1
 *   TX -> DMA1_Stream4
 *
 * PM2.5 is NOT declared here because PM2.5 uses USART1 RX DMA2_Stream5.
 * Its UART RX event/semaphore live in pm25_uart.c / pm25_uart.h:
 *   pm25_uart_rx_sem
 *
 * DS18B20 is NOT declared here because DS18B20 is 1-Wire, not I2C DMA.
 * Only its shared system fault owner/mask lives here.
 */
extern volatile i2c_dma_transfer_state_t bme680g_rx_cmplt;
extern volatile i2c_dma_transfer_state_t bme680g_tx_cmplt;
extern volatile i2c_dma_transfer_state_t mpu9250g_rx_cmplt;
extern volatile i2c_dma_transfer_state_t mpu9250g_tx_cmplt;

/* ----------------------------- Public API ----------------------------- */

i2c_select_status_t i2c_init_select(uint8_t i2c_num, uint8_t scl_pin, uint8_t sda_pin);

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

i2c_dma_init_status_t i2c_dma_bme680_peripherals_init(void);

i2c_recover_status_t i2c_recover(I2C_TypeDef *I2Cx,
                                 volatile uint32_t *status_reg,
                                 uint32_t wait_mask,
                                 uint8_t expect_set,
                                 uint8_t allow_restart);
int i2c_dma_rtos_sync_init(void);

void app_fault_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_DMA_H__ */
