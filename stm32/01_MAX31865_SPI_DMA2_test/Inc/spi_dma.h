#ifndef __SPI_DMA_H__
#define __SPI_DMA_H__

#include <stdint.h>
#include "stm32f4xx.h"

 
int spi4_dma_init(void);

/* Transmit/receive helpers tx_buf and rx_buf may be same buffer for full-duplex exchanges. */
int spi4_dma_transmit(uint8_t *tx_buf, uint32_t len);
int spi4_dma_receive(uint8_t *rx_buf, uint32_t len);
int spi4_dma_transmit_receive(uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len);

int spi4_recover(void);
void spi4_flush_rx(void);
void spi4_check_and_clear_ovr(void);
void spi4_short_delay_ms(uint32_t ms);

/* Blocking wait with timeout (ms). */
int spi_dma_stream0_wait_complete(uint32_t timeout_ms);
int spi_dma_stream1_wait_complete(uint32_t timeout_ms);

/* completion globals (volatile - set by IRQ handlers) */
extern volatile uint8_t spi_dma_rx_cmplt;  
extern volatile uint8_t spi_dma_tx_cmplt;
extern volatile uint32_t hb_spi_ts;


#endif  
