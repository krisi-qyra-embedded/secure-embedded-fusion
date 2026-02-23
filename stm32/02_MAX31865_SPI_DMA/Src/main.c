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
#include "cmox_crypto.h"



typedef enum {
    DMA_OK = 0,
    DMA_ERR_DISABLE_TIMEOUT,
} dma_status_t;

typedef enum {
    DMA_DIR_P2M = 0,    
    DMA_DIR_M2P = 1,    
} dma_direction_t;

/* DMA2 stream TCIF masks (STM32F4 style: Stream0 -> LISR bit5, Stream1 -> LISR bit11) */
#define DMA2_STREAM0_TCIF   (1U << 5)    
#define DMA2_STREAM1_TCIF   (1U << 11)   
/* Optional: matching LIFCR/HIFCR clear masks (write-1-to-clear) */
#define DMA2_LIFCR_CTCIF0   (1U << 5)    
#define DMA2_LIFCR_CTCIF1   (1U << 11)   

int rc;
/* Main loop: read temperature every 500 ms */
float temperatureC;
uint8_t retries = 3;
dma_status_t status;
 
uint8_t num_wires = 3;
uint8_t filterHz = 50;
 
Max31865_t max31865_dev;

/* Initializes the application and runs the MAX31865 sampling loop. */
int main(void)
{
    /* Stop initialization if a required peripheral fails. */
	iwdg_status_t st;
	 
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
	 
	sleep_mode_configure_once();
	 
	tim2_1s_init();
	 
	uart2_tx_rx_init();

	 if (systick_init(16000000U, 6U) != SYSTICK_OK) {
	         
	        NVIC_SystemReset();
	    }
	 if (systick_selftest(200U) != SYSTICK_OK) {
	          
	        NVIC_SystemReset();
	     }

		 
		if(cmox_initialize(NULL) !=  CMOX_INIT_SUCCESS)
		{

	     /* handle_error("ERROR: Init failed\r\n"); */


		}


	/* Inspect reset source before enabling watchdog */
	check_reset_source();

	 st = iwdg_init(IWDG_PRESCALER_16, IWDG_RELOAD_MAX, 0U);
	    if (st != IWDG_OK) {
	        log_printf("iwdg_init() failed: %d\n\r", (int)st);
	         
	        NVIC_SystemReset();
	    }
		 
		 
	    /* initialize drivers - all errors returned to main and handled here */
	      if (gpio_pb6_cs_init() != 0) on_fatal_init_error("gpio_pb6_cs_init failed");

				/* Configure spi4 */
	      uint8_t retries = 3;
	          do {
	              rc = spi4_dma_init();
	              if (rc != 0) {
	                  log_printf("[WARN] spi4_dma_init rc=%d, retrying\n\r", rc);
	                  tim2_delay_ms(50);
	              }
	          } while (rc != 0 && --retries);
	          if (rc != 0) on_fatal_init_error("spi4_dma_init failed after retries");

					/* 3) Prepare the Max31865 device struct (fill fields expected by your driver) */
	          	  memset(&max31865_dev, 0, sizeof(max31865_dev));

	              if (Max31865_init(&max31865_dev, 3, 50) != 0) on_fatal_init_error("Max31865_init failed");

	               
	                  uint32_t now = ms_get();
	                  hb_spi_ts = now;
	                  hb_main_ts = now;

	                  /* Main loop: drivers already return errors; main decides escalation */
	                  uint32_t last_supervisor = ms_get();

	while(1)
	{
		 /* attempt a robust read (driver-level retries/recoveries should take place in driver) */
		        rc = Max31865_readTempC(&max31865_dev, &temperatureC);
		        if (rc != 0) {
		            log_printf("MAX31865 read error rc=%d\n\r", rc);
		            /* don't busy loop here — escalate to supervised reset so the system restarts cleanly */
		            escalate_to_iwdg_reset("Max31865_readTempC unrecoverable");
		        }

		         
		               rc = Max31865_debug_once(&max31865_dev);
		               if (rc != 0) {
		                   log_printf("Max31865_debug_once rc=%d\n\r", rc);
		                   escalate_to_iwdg_reset("Max31865_debug failed");
		               }

		               /* run supervisor periodically (it calls iwdg_refresh only if health is OK) */
		               if (supervisor_due) {
		                       supervisor_due = 0;
		                       supervisor_check_and_kick();  
		                   }

		tim2_delay_ms(1000);
		     
		     

	}
}

