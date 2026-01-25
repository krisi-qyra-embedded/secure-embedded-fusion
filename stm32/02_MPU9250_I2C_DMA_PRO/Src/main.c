/* main.c | Author: i fiksuari pas prog | Date: 25 January 2026 */
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "systick.h"
#include "dma.h"
#include "adc_dma.h"
#include "mpu9250.h"
#include "i2c_dma.h"
#include "uart.h"
#include "tim.h"
#include "iwdg.h"
#include "pwr.h"

app_mpu9250_runtime_status_t mpu_runtime_rc;
/* app status enum */
//typedef enum {
//    APP_STATUS_OK = 0,
//    APP_STATUS_INIT_FAIL = 1,
//} app_status_t;


/* Initializes the MPU9250 runtime and enters the sensor processing loop. */
int main(void)
{
	/* Initialize the interfaces before starting repeated sensor reads. */

	clock_100_mhz_init();
    memset(&MPU9250, 0, sizeof(MPU9250));
    /* Enable FPU */
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

    /* Crypto use only */
    //	if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS) {
    //	        NVIC_SystemReset();
    //	    }

    check_reset_source();

    {
    	const iwdg_status_t iwdg_status = iwdg_init(IWDG_PRESCALER_64, 2499U, 0U);
    	if (iwdg_status != IWDG_OK) {
    	   log_printf("iwdg_init() failed: %d\r\n", (int)iwdg_status);
    	   tim2_delay_ms(2);
    	   NVIC_SystemReset();
    	}
    }

    if (app_mpu9250_peripherals_init() != APP_STATUS_OK) {
        on_fatal_init_error("app_mpu9250_peripherals_init failed");
    }

    if (app_mpu9250_sensor_stack_init() != APP_STATUS_OK) {
        on_fatal_init_error("app_mpu9250_sensor_stack_init failed");
    }

    {
    	const uint32_t now = ms_get();
    	hb_mpu9250_ts = now;
    	hb_mpu9250main_ts = now;
    }

    while (1) {
    	mpu_runtime_rc = app_mpu9250_runtime_process();
    	if (mpu_runtime_rc != APP_MPU9250_RUNTIME_OK) {
    	    log_printf("[ERROR] main: app_mpu9250_runtime_process rc=%d\r\n", (int)mpu_runtime_rc);
    	    tim2_delay_ms(2);
    	    escalate_to_iwdg_reset("MPU9250_read sensor parameters, unrecoverable");
    	}
    	if (supervisor_due) {
    		supervisor_due = 0;
    		supervisor_check_and_kick();
    		}
        tim2_delay_ms(1000);
    }
}




