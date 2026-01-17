/* main.c | Author: i fiksuari pas prog | Date: 17 January 2026 */
#include "stm32f4xx.h"
#include <stdio.h>
#include "dma.h"
#include "adc_dma.h"
#include "mpu9250.h"
#include "i2c_dma.h"
#include <string.h>
#include "uart.h"


typedef enum {
    DMA_OK = 0,
    DMA_ERR_DISABLE_TIMEOUT,
} dma_status_t;

typedef enum {
    DMA_DIR_P2M = 0,   // Peripheral-to-Memory
    DMA_DIR_M2P = 1,   // Memory-to-Peripheral
} dma_direction_t;

MPU9250_t MPU9250;

/* Example flag masks (adjust depending on which stream you target) */
#define DMA1_STREAM5_TCIF (1U << 11)   // Transfer complete flag for Stream5
#define DMA1_STREAM6_TCIF (1U << 21)   // Transfer complete flag for Stream6
#define DMA1_STREAM7_TCIF (1U << 27)
#define DMA1_STREAM2_TCIF (1U << 21)
#define DMA1_STREAM1_TCIF (1U << 11)
#define DMA1_STREAM4_TCIF (1U << 5)

int rc;
uint8_t retries = 3;
dma_status_t status;


/* Initializes the MPU9250 runtime and enters the sensor processing loop. */
int main(void)
{
	/* Initialize the interfaces before starting repeated sensor reads. */
	/* Enable TIM2 */
		tim2_1s_init();
		/* Enable USART */
		uart2_tx_rx_init();
		/* Enable I2C */
		  MPU9250.settings.gFullScaleRange = GFSR_500DPS;
		  MPU9250.settings.aFullScaleRange = AFSR_4G;
		  MPU9250.attitude.tau = 0.98;
		  MPU9250.attitude.dt = 0.004;
		/* Configure I2C1 using PA8 (SCL) and PB4 (SDA) */
		do {
			    rc = i2c_init_select(3U, 8U, 4U);
			    if (rc != 0) {
			        log_printf("[WARN] i2c_init_select failed rc=%d, retries left: %u\r\n", rc, retries-1);
			    }
			} while (rc != 0 && --retries);

			if (rc != 0) {
			    log_printf("[FATAL] i2c_init_select failed after 3 retries -> system reset\r\n");
			    NVIC_SystemReset();
			}
		 
		 
		SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

		do {
			    status = dma_stream_init(DMA1_Stream1,
			                             1U,                     // I2C1_TX on CH1
										 DMA_DIR_P2M,            // Memory → Peripheral
										 &DMA1->LIFCR,           // High IFCR
										 DMA1_STREAM1_TCIF,      // Flag mask
										 DMA1_Stream1_IRQn);     // IRQ line
			    if (status != DMA_OK) {
			        log_printf("[WARN] DMA1_Stream6 init failed, retries left: %u\r\n", retries-1);
			    }
			} while (status != DMA_OK && --retries);
			if (status != DMA_OK) {
			    log_printf("[FATAL] DMA1_Stream6 init failed after 3 retries -> system reset\r\n");
			    NVIC_SystemReset();
			}
		 
		do {
				 status = dma_stream_init(DMA1_Stream4,
					                      3U,                     // I2C1_TX on CH1
										  DMA_DIR_M2P,            // Memory → Peripheral
										  &DMA1->HIFCR,           // High IFCR
										  DMA1_STREAM4_TCIF,      // Flag mask
										  DMA1_Stream4_IRQn);     // IRQ line
				if (status != DMA_OK) {
					log_printf("[WARN] DMA1_Stream6 init failed, retries left: %u\r\n", retries-1);
				}
			  } while (status != DMA_OK && --retries);
				if (status != DMA_OK) {
				log_printf("[FATAL] DMA1_Stream6 init failed after 3 retries -> system reset\r\n");
				NVIC_SystemReset();
			  }
				 
				    if (mpu9250_accel_config(ACC_FULL_SCALE_2_G,&MPU9250) != 0) {
				        log_printf("[ERROR] mpu9250_accel_config failed (rc=%d) -> resetting\r\n");
				        NVIC_SystemReset();  
				    }
				    if (MPU9250_beginGyro(GYRO_FULL_SCALE_250_DPS,&MPU9250) != 0) {
				    	log_printf("[ERROR] MPU9250_beginGyro failed (rc=%d) -> resetting\r\n");
				    	NVIC_SystemReset();  
				    }
					if(mpu9250_calibrateGyro(&MPU9250,5000) != 0){
				    	log_printf("[ERROR] mpu9250_calibrateGyro failed (rc=%d) -> resetting\r\n");
				    	NVIC_SystemReset();  
					}
	while(1)
	{

		if(MPU_readProcessedData(&MPU9250) != 0){
			log_printf("[ERROR] MPU_readProcessedData failed (rc=%d) -> resetting\r\n");
			NVIC_SystemReset();  
		}
		for(int i = 0; i < 1200000; i++){}

	}
}




