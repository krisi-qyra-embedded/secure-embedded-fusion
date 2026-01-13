/* i2c_dma.h | Author: i fiksuari mbas prog | Date: 13 January 2026 */
#ifndef __I2C_DMA_H__
#define __I2C_DMA_H__

#include <stdint.h>
#include <stddef.h>
#include "stm32f4xx.h"

int i2c_init_select(uint8_t i2c_num, uint8_t scl_pin, uint8_t sda_pin);
void dma1_stream6_i2c1_transfer(uint8_t *msg_to_send, uint32_t msg_len);
void dma1_stream5_i2c1_receive(uint8_t * received_msg, uint32_t msg_len);
static int i2c_recover(volatile __IO uint32_t *reg_SR, volatile __IO uint32_t *reg_CR, uint32_t SR_BIT, uint32_t CR_SWRST, uint32_t CR_STOP, uint32_t CR_START);
extern volatile uint8_t g_rx_cmplt;
extern volatile uint8_t g_tx_cmplt;
/* Correct prototype (matches implementation) */

#endif
