/* main.c | Author: i fiksuari mbas prog | Date: 13 January 2026 */
#include "stm32f4xx.h"
#include <stdio.h>
#include "dma.h"
#include "adc_dma.h"
#include "i2c_dma.h"
#include <string.h>
#include "uart.h"
#include "iwdg.h"
#include "pwr.h"
#include "sig_sensors.h"

typedef enum {
    DMA_OK = 0,
    DMA_ERR_DISABLE_TIMEOUT,
} dma_status_t;

typedef enum {
    DMA_DIR_P2M = 0,   
    DMA_DIR_M2P = 1,   
} dma_direction_t;



#define DMA1_STREAM5_TCIF (1U << 11)   /* Transfer complete flag for Stream5 */
#define DMA1_STREAM6_TCIF (1U << 21)   /* Transfer complete flag for Stream6 */
#define DMA1_STREAM7_TCIF (1U << 27)
#define DMA1_STREAM2_TCIF (1U << 21)
#define DMA1_STREAM1_TCIF (1U << 11)
#define DMA1_STREAM4_TCIF (1U << 5)

int rc;
uint8_t retries = 3;
dma_status_t status;

 volatile st_battery_t bat;
 volatile st_bme680_t bme;

/* USER CODE BEGIN PFP */
void fnAPP_Init_Standard_Values(void);

float temp;
float pres;
float hum;


/* Initializes the required peripherals and enters the main application flow. */
int main(void)
{
		/* Abort initialization early if a required peripheral fails. */
	
		tim2_1s_init();
		
		uart2_tx_rx_init();
		/* Enable I2C */
		/* Configure I2C1 using PB8 (SCL) and PB9 (SDA) */
		do {
			    rc = i2c_init_select(1U, 8U, 9U);
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
			    status = dma_stream_init(DMA1_Stream5,
			                             1U,                     
										 DMA_DIR_P2M,            
										 &DMA1->HIFCR,           
										 DMA1_STREAM5_TCIF,      
										 DMA1_Stream5_IRQn);     
			    if (status != DMA_OK) {
			        log_printf("[WARN] DMA1_Stream6 init failed, retries left: %u\r\n", retries-1);
			    }
			} while (status != DMA_OK && --retries);
			if (status != DMA_OK) {
			    log_printf("[FATAL] DMA1_Stream6 init failed after 3 retries -> system reset\r\n");
			    NVIC_SystemReset();
			}
		
		do {
				 status = dma_stream_init(DMA1_Stream6,
					                      1U,                     
										  DMA_DIR_M2P,            
										  &DMA1->HIFCR,           
										  DMA1_STREAM6_TCIF,      
										  DMA1_Stream6_IRQn);     
				if (status != DMA_OK) {
					log_printf("[WARN] DMA1_Stream6 init failed, retries left: %u\r\n", retries-1);
				}
			  } while (status != DMA_OK && --retries);
				if (status != DMA_OK) {
				log_printf("[FATAL] DMA1_Stream6 init failed after 3 retries -> system reset\r\n");
				NVIC_SystemReset();
			  }

				fnAPP_Init_Standard_Values();

				/* retry init_bme680 up to MAX_ATTEMPTS, then NVIC_SystemReset() */

				    const int MAX_ATTEMPTS = 3;
				    int attempts = 0;
				    int rslt;

				start_bme_init:
				    rslt = init_bme680();
				    if (rslt != BME680_OK) {
				        attempts++;
				        log_printf("[ERROR] BME680 initialization failed (code %d) (attempt %d/%d)\r\n",
				                   (int)rslt, attempts, MAX_ATTEMPTS);

				        if (attempts >= MAX_ATTEMPTS) {
				            log_printf("[FATAL] BME680 initialization failed after %d attempts -> system restart\n\r",
				                       attempts);
				            NVIC_SystemReset(); 
				        }

				        /* optional short backoff before retry (uncomment/replace if you have a delay helper) */
				        
				        user_delay_ms(10);

				        goto start_bme_init;
				    }

				/* init succeeded: reset attempts counter so subsequent read failures get fresh retries */
				attempts = 0;
				/* Get the total measurement duration so as to sleep or wait till the measurement is complete */
				struct bme680_field_data data;

	while(1)
	{
		rslt = bme680_get_sensor_data(&data, &bme.gas_sensor);

		if (rslt == BME680_OK) {
		    bme.temperature = data.temperature / 100.0f;
		    bme.pressure    = data.pressure / 100.0f;
		    bme.humidity    = data.humidity / 1000.0f;
		}

		else {
		        
		if (rslt > 0)
		    log_printf("[WARN] bme680_get_sensor_data returned warning code %d. Re-init and retry...\r\n", rslt);
		else
		    log_printf("[ERROR] bme680_get_sensor_data failed with code %d. Re-init and retry...\r\n", rslt);
		    
		    attempts++;
		if (attempts >= MAX_ATTEMPTS) {
		    log_printf("[FATAL] bme680_get_sensor_data failed after %d attempts -> system restart\r\n", attempts);
		    NVIC_SystemReset(); 
		  }
		/* backoff briefly then re-run full init sequence by jumping to label */
		 	user_delay_ms(10);
		    goto start_bme_init;
		}
		attempts = 0;
		/* assume: int8_t rslt = bme680_get_sensor_data(&data, &bme.gas_sensor); */
		
		if (rslt == BME680_OK) {
		    /* convert sensor raw -> human units once */
		     temp = (float)data.temperature / 100.0f;
		     pres = (float)data.pressure    / 100.0f;
		     hum  = (float)data.humidity    / 1000.0f;

		    
		    bme.temperature = temp;
		    bme.pressure    = pres;
		    bme.humidity    = hum;

		    
		    if (data.status & BME680_GASM_VALID_MSK) {
		        bme.gas_resistance = data.gas_resistance;
		    }

		    
		    log_printf("t:%.2f; p:%.0f; h:%.0f, ",
		           bme.temperature, bme.pressure, bme.humidity);

		    /* increment then check every 5 readings (pre-increment avoids post-increment confusion) */
		    if ((++bme.getgasreference_count % 5U) == 0U) {
		       rslt =  bme_GetGasReference();
		       if (rslt == 0) {
		       /* success: reset attempts counter so transient errors don't accumulate */
		    	attempts = 0;
		       }
		       else {
		        /* log appropriate severity (keep parity with other bme680 error handling) */
		        if (rslt > 0)
		          log_printf("[WARN] bme_GetGasReference returned warning code %d. Re-init and retry...\r\n", (int)rslt);
		        else
		          log_printf("[ERROR] bme_GetGasReference failed with code %d. Re-init and retry...\r\n", (int)rslt);
		        /* increment attempts -> re-run full init when exceeded */
		        attempts++;
		        if (attempts >= MAX_ATTEMPTS) {
		            log_printf("[FATAL] bme_GetGasReference failed after %d attempts -> system restart\n\r", attempts);
		            NVIC_SystemReset(); 
		        }
		        
		        user_delay_ms(10);
		        goto start_bme_init;
		       }
		    }
		    
		    int16_t hum_score = bme_GetHumidityScore();
		    int16_t gas_score = bme_GetGasScore();
		    bme_calculateIAQ((int16_t)(hum_score + gas_score));
		}

		attempts = 0;
		/* wait until measurement is ready — prefer measured profile duration if available */
		{
		    uint32_t wait_ms = (bme.meas_period > 0U) ? (uint32_t)bme.meas_period : 2000U;
		    user_delay_ms(wait_ms);
		}
		user_delay_ms(5000);
		
		if (bme.gas_sensor.power_mode == BME680_FORCED_MODE) {
			 int8_t rslt = bme680_set_sensor_mode(&bme.gas_sensor);
			 if (rslt != BME680_OK) {
			        /* log appropriate severity (parallel to other bme680 error handling) */
			        if (rslt > 0)
			            log_printf("[WARN] bme680_set_sensor_mode returned warning %d. Re-init and retry...\r\n", (int)rslt);
			        else
			            log_printf("[ERROR] bme680_set_sensor_mode failed with code %d. Re-init and retry...\r\n", (int)rslt);
			        
			        attempts++;
			         if (attempts >= MAX_ATTEMPTS) {
			           log_printf("[FATAL] bme680_set_sensor_mode failed after %d attempts -> system restart\n\r", attempts);
			           NVIC_SystemReset(); 
			        }
			         /* brief backoff then re-run full init sequence */
			         user_delay_ms(10);
			         goto start_bme_init;
			 }
			 /* success: reset attempts counter so transient errors don't accumulate */
			 attempts = 0;
		}
	}
}

/* USER CODE BEGIN 4 */
void fnAPP_Init_Standard_Values(void){
	/* Configure only the resources required by this module. */

	bme.gas_weighting = 0.75;
	bme.gas_reference = 2500;
	bme.getgasreference_count = 0;
	bme.hum_weighting = 0.25;
	bme.hum_reference = 40;
	bme.gas_lower_limit = 10000;
	bme.gas_upper_limit = 300000;

}


