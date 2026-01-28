/* adc_dma.c | Author: i fiksuari mbas prog | Date: 28 January 2026 */
#include "stm32f4xx.h"
#include <stdio.h>
#include "dma.h"
#include "adc_dma.h"
#include "i2c_dma.h"
#include "bme680.h"
#include <string.h>
#include "uart.h"
#include "iwdg.h"
#include "pwr.h"
#include "sig_sensors.h"

/* app status enum (declare in main.c as requested) */
typedef enum {
    APP_STATUS_OK = 0,
    APP_STATUS_INIT_FAIL = 1,
} app_status_t;

static app_status_t app_bme680_sensor_stack_init_(void);
static app_status_t app_bme680_peripherals_init(void);

int rslt;
uint8_t retries = 3;
volatile st_bme680_t bme;
dma_status_t status;
struct bme680_field_data data;
volatile st_battery_t bat;
app_bme680_runtime_status_t rc;
/* USER CODE BEGIN PFP */
void fnAPP_Init_Standard_Values(void);

float temp;
float pres;
float hum;


/* Initializes the required peripherals and enters the main application flow. */
int main(void)
{
	/* Abort initialization early if a required peripheral fails. */
	clock_100_mhz_init();
	SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
	sleep_mode_configure_once();
	
	tim2_1s_init();
	uart2_tx_rx_init();

	if (systick_init(96000000U, 6U) != SYSTICK_OK) {
	        NVIC_SystemReset();
	    }

	if (systick_selftest(200U) != SYSTICK_OK) {
	        NVIC_SystemReset();
	    }
	
/* if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS) { */
/* NVIC_SystemReset(); */


	check_reset_source();

	 	{
		const iwdg_status_t iwdg_status = iwdg_init(IWDG_PRESCALER_64, 2499U, 0U);
		if (iwdg_status != IWDG_OK) {
		    log_printf("iwdg_init() failed: %d\r\n", (int)iwdg_status);
		    tim2_delay_ms(2);
		    NVIC_SystemReset();
		}
	    }

	 	if (app_bme680_peripherals_init() != APP_STATUS_OK) {
	 	    on_fatal_init_error("app_bme680_peripherals_init failed");
	 	}

	 	if (app_bme680_sensor_stack_init_() != APP_STATUS_OK) {
	 	    on_fatal_init_error("app_bme680_sensor_stack_init failed");
	 	}


	 	{
	 	       const uint32_t now = ms_get();
	 	       hb_bme680_ts = now;
	 	       hb_bme680main_ts = now;
	 	}

	while(1)
	{
		rc = app_bme680_sensor_runtime_process();
		        if (rc != APP_BME680_RUNTIME_OK) {
		            log_printf("[ERROR] main: app_bme680_sensor_runtime_process rc=%d\r\n", (int)rc);
		            tim2_delay_ms(2);
		            escalate_to_iwdg_reset("BME680_read sensor parameters, unrecoverable");
		        }

		        if (supervisor_due) {
		                    supervisor_due = 0;
		                    supervisor_check_and_kick();
		                }
	            user_delay_ms(500);
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

/* static init functions for BME680 (like MAX31865 pattern) */

static app_status_t app_bme680_peripherals_init(void)
{
    /* Configure only the resources required by this module. */
    
    i2c_select_status_t rc = i2c_init_select(1U, 8U, 9U);
    if (rc != I2C_SELECT_OK) {
        log_printf("[ERROR] app_bme680_peripherals_init: i2c_init_select failed rc=%d\r\n", (int)rc);
        return APP_STATUS_INIT_FAIL;
    }

    
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    /* 3) DMA streams for I2C1 BME680 (RX=Stream5, TX=Stream6) */
    {
        dma_status_t st;

        /* RX stream (I2C1_RX -> DMA1 Stream5, Channel 1 in your setup) */
        st = dma_stream_init(DMA1_Stream5,
                             1U,
                             DMA_DIR_P2M,
                             &DMA1->HIFCR,
                             DMA1_STREAM5_TCIF,
                             DMA1_Stream5_IRQn);
        if (st != DMA_OK) {
            log_printf("[ERROR] app_bme680_peripherals_init: DMA1_Stream5 init failed rc=%d\r\n", (int)st);
            return APP_STATUS_INIT_FAIL;
        }

        /* TX stream (I2C1_TX -> DMA1 Stream6, Channel 1 in your setup) */
        st = dma_stream_init(DMA1_Stream6,
                             1U,
                             DMA_DIR_M2P,
                             &DMA1->HIFCR,
                             DMA1_STREAM6_TCIF,
                             DMA1_Stream6_IRQn);
        if (st != DMA_OK) {
            log_printf("[ERROR] app_bme680_peripherals_init: DMA1_Stream6 init failed rc=%d\r\n", (int)st);
            return APP_STATUS_INIT_FAIL;
        }
    }

    /* keep your original call here (as it was placed after DMA init) */
    fnAPP_Init_Standard_Values();

    return APP_STATUS_OK;
}





/* Configures app bme680 sensor stack init for normal operation. */
static app_status_t app_bme680_sensor_stack_init_(void)
{
    /* Configure only the resources required by this module. */
    
    int8_t rslt = init_bme680();
    if (rslt != 0) {
        log_printf("[ERROR] app_bme680_sensor_stack_init: init_bme680 failed rc=%d\r\n", (int)rslt);
        return APP_STATUS_INIT_FAIL;
    }
    return APP_STATUS_OK;
}





