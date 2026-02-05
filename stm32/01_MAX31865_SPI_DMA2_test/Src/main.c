#include <max31865.h>
#include "stm32f4xx.h"
#include <stdio.h>
#include "dma.h"
#include "spi_dma.h"
#include <string.h>
#include "uart.h"
#include"tim.h"
#include "iwdg.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "systick.h"


typedef enum {
    DMA_OK = 0,
    DMA_ERR_DISABLE_TIMEOUT,
} dma_status_t;

typedef enum {
    DMA_DIR_P2M = 0,   // Peripheral-to-Memory
    DMA_DIR_M2P = 1,   // Memory-to-Peripheral
} dma_direction_t;

/* DMA2 stream TCIF masks (STM32F4 style: Stream0 -> LISR bit5, Stream1 -> LISR bit11) */
#define DMA2_STREAM0_TCIF   (1U << 5)   /* TC flag for DMA2 Stream0 */
#define DMA2_STREAM1_TCIF   (1U << 11)  /* TC flag for DMA2 Stream1 */
/* Optional: matching LIFCR/HIFCR clear masks (write-1-to-clear) */
#define DMA2_LIFCR_CTCIF0   (1U << 5)   /* clear TCIF0 in DMA2->LIFCR */
#define DMA2_LIFCR_CTCIF1   (1U << 11)  /* clear TCIF1 in DMA2->LIFCR */

int rc;
/* Main loop: read temperature every 500 ms */
float temperatureC;
uint8_t retries = 3;
dma_status_t status;
/* Example: PT100 on 3 wires, filter = 50 Hz (choose values you need) */
uint8_t num_wires = 3;
uint8_t filterHz = 50;
/* globals used by the driver */
Max31865_t max31865_dev;

int main(void)
{
	iwdg_status_t st;
	/*Enable Floating Point Unit*/
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

	/*Enable TIM2*/
	tim2_1s_init();
	/*Enable USART*/
	uart2_tx_rx_init();

//	 if (systick_init(16000000U, 6U) != SYSTICK_OK) {
//	        /* handle failure: choose fallback or halt */
//	        NVIC_SystemReset();
//	    }
//	 if (systick_selftest(200U) != SYSTICK_OK) {
//	         /* diagnostics: SysTick didn't run */
//	        NVIC_SystemReset();
//	     }

//	/* Inspect reset source before enabling watchdog */
//	check_reset_source();

//	 st = iwdg_init(IWDG_PRESCALER_16, IWDG_RELOAD_MAX, 0U);
//	    if (st != IWDG_OK) {
//	        log_printf("iwdg_init() failed: %d\n\r", (int)st);
//	        /* Decision: halt or continue without watchdog. Here we halt to be safe. */
//	        NVIC_SystemReset();
//	    }
		/*Enable GPIOA6 */
		/* 1) Configure PB6 as CS (CMSIS helper you provided) */
	    /* initialize drivers - all errors returned to main and handled here */
	      if (gpio_pb6_cs_init() != 0){
	    	  log_printf("gpio_pb6_cs_init failed");
	    	  NVIC_SystemReset();
	      }

				/* Configure spi4 */
	      uint8_t retries = 3;
	          do {
	              rc = spi4_dma_init();
	              if (rc != 0) {
	                  log_printf("[WARN] spi4_dma_init rc=%d, retrying\n\r", rc);
	                  tim2_delay_ms(50);
	              }
	          } while (rc != 0 && --retries);
	          if (rc != 0) {
	        	  log_printf("FATAL: Max31865_init failed -> escalating to restart ...\n\r");
	        	  NVIC_SystemReset();
	          }

					/* 3) Prepare the Max31865 device struct (fill fields expected by your driver) */
	          	  memset(&max31865_dev, 0, sizeof(max31865_dev));
	              max31865_dev.lock = 0;
	              if (Max31865_init(&max31865_dev, 3, 50) != 0){
	              log_printf("FATAL: Max31865_init failed -> escalating to restart ...\n\r");
	              NVIC_SystemReset();
	              }

	while(1)
	{
		 /* attempt a robust read (driver-level retries/recoveries should take place in driver) */
		        rc = Max31865_readTempC(&max31865_dev, &temperatureC);
		        if (rc != 0) {
		            log_printf("MAX31865 read error rc=%d\n\r", rc);
		            /* don't busy loop here — escalate to supervised reset so the system restarts cleanly */
			        NVIC_SystemReset();
		        }

		        /* optional debug check */
		               rc = Max31865_debug_once(&max31865_dev);
		               if (rc != 0) {
		                   log_printf("Max31865_debug_once rc=%d\n\r", rc);
		       	        NVIC_SystemReset();
		               }

		tim2_delay_ms(1000);
		    /* keep this delay ISR-friendly so SysTick keeps running */
		    //systick_delay_ms(1000); /* print once per second */
	}
}




