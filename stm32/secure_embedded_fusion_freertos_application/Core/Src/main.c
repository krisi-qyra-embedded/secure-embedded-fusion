/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "clock.h"
#include <string.h>
#include "systick.h"
#include "dma.h"
#include "adc_dma.h"
#include "mpu9250.h"
#include "i2c_dma.h"
#include "spi_dma.h"
#include "uart.h"
#include "pm25_uart.h"
#include "tim.h"
#include "iwdg.h"
#include "pwr.h"
#include "sig_sensors.h"
#include "bme680.h"
#include "max31865.h"
#include "pm2_5.h"
#include "ds18b20.h"
#include "tim3.h"

/* ---------------- app status enum (declare in main.c as requested) ---------------- */

static PMS25_App_t g_pm25_app;
static DS18B20_App_t g_ds18b20_app;
static app_status_t app_peripherals_init(void);
static app_status_t app_sensor_stack_init(Max31865_t *dev);
static app_status_t app_bme680_sensor_stack_init_(void);
static app_status_t app_bme680_peripherals_init(void);
static void Error_Handler(void);
static void PM25_HandleRuntimeFatal(PMS25_App_t *app);

/* Writes one character through the ITM trace port. */
int __io_putchar(int ch)
{
    /* Return the supplied character after submitting it to ITM. */
    ITM_SendChar(ch);
    return ch;
}

int rslt;
uint8_t retries = 3;
volatile st_bme680_t bme;
dma_status_t status;
struct bme680_field_data data;
volatile st_battery_t bat;
app_bme680_runtime_status_t rc;
float temperatureC = 0.0f;
Max31865_t max31865_dev;
/* USER CODE BEGIN PFP */
void fnAPP_Init_Standard_Values(void);

float temp;
float pres;
float hum;

app_mpu9250_runtime_status_t mpu_runtime_rc;
/* ---------------- app status enum ---------------- */
//typedef enum {
//    APP_STATUS_OK = 0,
//    APP_STATUS_INIT_FAIL = 1,
//} app_status_t;

/* Initializes and polls the five-sensor application. */
int main(void)
{
	/* Refresh the supervisor only while the sensor runtime paths continue making progress. */
	UART_Status_t uart_st;
	tim_status_t tim_st;
	DS18B20_Status_t ds_st;
	DS18B20_AppRuntimeStatus_t app_st_ds18b20;

	HAL_Init();
	clock_100_mhz_init();
    memset(&MPU9250, 0, sizeof(MPU9250));
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
        tim_st = MX_TIM3_Init();
        if (tim_st != TIM_STATUS_OK)
        {
            log_printf("[ERR] MX_TIM3_Init failed\r\n");
            delay_ms(10U);
            Error_Handler();
        }

    //	if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS) {
    //	        NVIC_SystemReset();
    //	    }

    check_reset_source();

    {
        const iwdg_status_t iwdg_status = iwdg_init(IWDG_PRESCALER_128, 2499U, 0U);

        if (iwdg_status != IWDG_OK)
        {
            log_printf("iwdg_init() failed: %d\r\n", (int)iwdg_status);
            tim2_delay_ms(2);
            NVIC_SystemReset();
        }
    }

    ds_st = DS18B20_AppInit(&g_ds18b20_app,
                               GPIOA,
                               GPIO_PIN_5,
                               &htim3,
                               DS18B20_Resolution_12bits,
                               5U);
       if (ds_st != DS18B20_STATUS_OK)
       {
           on_fatal_init_error("DS18B20_AppInit failed after 3 recovery attempts");
           //NVIC_SystemReset();
       }

    if (app_peripherals_init() != APP_STATUS_OK) {
        on_fatal_init_error("app_peripherals_init failed");
    	//NVIC_SystemReset();
    }

    if (app_sensor_stack_init(&max31865_dev) != APP_STATUS_OK) {
        on_fatal_init_error("app_sensor_stack_init failed");
    	//NVIC_SystemReset();
    }

    if (app_bme680_peripherals_init() != APP_STATUS_OK) {
    	 	    on_fatal_init_error("app_bme680_peripherals_init failed");
    			//NVIC_SystemReset();
    	 	}

    	 	if (app_bme680_sensor_stack_init_() != APP_STATUS_OK) {
    	 	    on_fatal_init_error("app_bme680_sensor_stack_init failed");
    	 		//NVIC_SystemReset();
    	 	}

    if (app_mpu9250_peripherals_init() != APP_STATUS_OK) {
        on_fatal_init_error("app_mpu9250_peripherals_init failed");
    	//NVIC_SystemReset();
    }

    if (app_mpu9250_sensor_stack_init() != APP_STATUS_OK) {
        on_fatal_init_error("app_mpu9250_sensor_stack_init failed");
    	//NVIC_SystemReset();
    }

    uart_st = UART1_PM25_Init();
    if (uart_st != UART_STATUS_OK)
    {
        log_printf("[FATAL][PM25] UART1_PM25_Init failed, st=%u\r\n",
                   (unsigned)uart_st);
        on_fatal_init_error("UART1_PM25_Init failed");
        //NVIC_SystemReset();
    }

    if (PMS25_AppInit(&g_pm25_app,
                      PM25_FRAME_POLL_TIMEOUT_MS,
                      PM25_NO_FRAME_FAULT_MS,
                      PM25_SAMPLE_PERIOD_MS,
                      PM25_RECOVERY_DELAY_MS,
                      PM25_MAX_RECOVERY_ATTEMPTS) != PMS25_APP_RUNTIME_OK)
    {
        log_printf("[FATAL][PM25] PMS25_AppInit failed\r\n");
        on_fatal_init_error("PMS25_AppInit failed");
        //NVIC_SystemReset();
    }

    {
    	const uint32_t now = ms_get();
    	hb_spi_ts = now;
    	hb_main_ts = now;
    	hb_mpu9250_ts = now;
    	hb_mpu9250main_ts = now;
    	hb_bme680_ts = now;
    	hb_bme680main_ts = now;
    }

    while (1) {
    	mpu_runtime_rc = app_mpu9250_runtime_process();
    	if (mpu_runtime_rc != APP_MPU9250_RUNTIME_OK) {
    	    log_printf("[ERROR] main: app_mpu9250_runtime_process rc=%d\r\n", (int)mpu_runtime_rc);
    	    tim2_delay_ms(2);
    	    escalate_to_iwdg_reset("MPU9250_read sensor parameters, unrecoverable");
    	    //NVIC_SystemReset();
    	}

    	rc = app_bme680_sensor_runtime_process();
    	if (rc != APP_BME680_RUNTIME_OK) {
    	    log_printf("[ERROR] main: app_bme680_sensor_runtime_process rc=%d\r\n", (int)rc);
    	    tim2_delay_ms(2);
    	    escalate_to_iwdg_reset("BME680_read sensor parameters, unrecoverable");
    	    //NVIC_SystemReset();
    	}

    	const max31865_status_t read_status = Max31865_readTempC(&max31865_dev, &temperatureC);
    	        if (read_status != MAX31865_STATUS_OK) {
    	            log_printf("MAX31865 read error rc=%d\r\n", read_status);
    	            tim2_delay_ms(2);
    	            escalate_to_iwdg_reset("Max31865_readTempC unrecoverable");
    	            //NVIC_SystemReset();
    	        }

    	const PMS25_AppRuntimeStatus_t app_st = PMS25_AppRun(&g_pm25_app);

    	         if (app_st == PMS25_APP_RUNTIME_FATAL)
    	           {
    	            log_printf("[FATAL][PM25] All recovery attempts exhausted, resetting system now\r\n");
    	            tim2_delay_ms(2);
    	            escalate_to_iwdg_reset("pm25 runtime failure after 3 total recovery attempts");
    	            //NVIC_SystemReset();
    	           }
    	           else if (app_st == PMS25_APP_RUNTIME_BAD_PARAM)
    	           {
    	             log_printf("[FATAL][PM25] PMS25_AppRun returned BAD_PARAM\r\n");
    	             tim2_delay_ms(2);
    	             escalate_to_iwdg_reset("pm25 runtime bad parameter");
    	             //NVIC_SystemReset();
    	           }

    	         app_st_ds18b20 = DS18B20_AppRun(&g_ds18b20_app);

    	                 if ((app_st_ds18b20 == DS18B20_APP_RUNTIME_FATAL) ||
    	                     (app_st_ds18b20 == DS18B20_APP_RUNTIME_RECOVERY_FAIL))
    	                 {
    	                     log_printf("[FATAL] One or more DS18B20 sensors failed after 3 recovery attempts -> escalating\r\n");
    	                     delay_ms(2U);
    	                     escalate_to_iwdg_reset("DS18B20 runtime failure after 3 recovery attempts");
    	                     //NVIC_SystemReset();
    	                 }

    	if (supervisor_due) {
    		supervisor_due = 0;
    		supervisor_check_and_kick();
    		}
        tim2_delay_ms(20);
    }
}

/* USER CODE BEGIN 4 */
/* Sets the BME680 air-quality reference and weighting values. */
void fnAPP_Init_Standard_Values(void){
	/* Reserve 75 percent of the score for gas and 25 percent for humidity. */

	bme.gas_weighting = 0.75;
	bme.gas_reference = 2500;
	bme.getgasreference_count = 0;
	bme.hum_weighting = 0.25;
	bme.hum_reference = 40;
	bme.gas_lower_limit = 10000;
	bme.gas_upper_limit = 300000;

}

/* ---------------- static init functions for BME680 (like MAX31865 pattern) ---------------- */

/* Initializes the BME680 I2C1 bus and DMA streams. */
static app_status_t app_bme680_peripherals_init(void)
{
    /* Reject failed bus or stream setup before initializing the sensor. */
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

    fnAPP_Init_Standard_Values();

    return APP_STATUS_OK;
}

/* Applies the BME680 sensor configuration. */
static app_status_t app_bme680_sensor_stack_init_(void)
{
    /* Propagate a failed sensor initialization to application startup. */
    int8_t rslt = init_bme680();
    if (rslt != 0) {
        log_printf("[ERROR] app_bme680_sensor_stack_init: init_bme680 failed rc=%d\r\n", (int)rslt);
        return APP_STATUS_INIT_FAIL;
    }
    return APP_STATUS_OK;
}

/* Initializes MAX31865 chip select and SPI4 DMA. */
static app_status_t app_peripherals_init(void)
{
    /* Limit SPI setup to three attempts with a delay between failures. */
    if (gpio_pb6_cs_init() != 0) {
        return APP_STATUS_INIT_FAIL;
    }

    {
        spi_dma_status_t spi_status = SPI_DMA_STATUS_INIT_FAILED;
        uint8_t retries = 3U;

        do {
            spi_status = spi4_dma_init();
            if (spi_status != SPI_DMA_STATUS_OK) {
                log_printf("[WARN] spi4_dma_init rc=%d, retrying\r\n", spi_status);
                tim2_delay_ms(50U);
            }
        } while ((spi_status != SPI_DMA_STATUS_OK) && (--retries != 0U));

        if (spi_status != SPI_DMA_STATUS_OK) {
            return APP_STATUS_INIT_FAIL;
        }
    }

    return APP_STATUS_OK;
}

/* Halts execution after an unrecoverable application error. */
static void Error_Handler(void)
{
    /* Disable interrupts before entering the terminal wait. */
    __disable_irq();
    while (1)
    {
    }
}

/* Initializes the supplied MAX31865 device state. */
static app_status_t app_sensor_stack_init(Max31865_t *dev)
{
    /* Reject a null device before selecting three-wire, 50 Hz operation. */
    if (dev == NULL) {
        return APP_STATUS_INIT_FAIL;
    }

    memset(dev, 0, sizeof(*dev));

    if (Max31865_init(dev, 3U, 50U) != MAX31865_STATUS_OK) {
        return APP_STATUS_INIT_FAIL;
    }

    return APP_STATUS_OK;
}

