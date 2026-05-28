/* Author: i fiksuari pas prog
 * Documentation date: 2026-05-28
 */

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
#include "sig_sensors.h"
#include "bme680.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"

/* ---------------- app status enum (declare in main.c as requested) ---------------- */

static app_status_t app_bme680_sensor_stack_init_(void);
static app_status_t app_bme680_peripherals_init(void);

/* ---------------- FreeRTOS sensor task configuration ---------------- */
#define BME680_TASK_STACK_WORDS      		(1536U)
#define MPU9250_TASK_STACK_WORDS     		(1536U)
#define SENSOR_TASK_PERIOD_MS        		(1000U)
#define BME680_SENSOR_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)
#define MPU9250_SENSOR_TASK_PRIORITY        (tskIDLE_PRIORITY + 3U)
#define SUPERVISOR_TASK_STACK_WORDS  		(768U)
#define SUPERVISOR_TASK_PERIOD_MS    		(250U)
#define SUPERVISOR_TASK_PRIORITY     		(tskIDLE_PRIORITY + 4U)

static void bme680_task(void *pvParameters);
static void mpu9250_task(void *pvParameters);
static void supervisor_task(void *pvParameters);
static void debug_log_bme680_task_state(void);

static TaskHandle_t bme680_task_handle = NULL;
static TaskHandle_t mpu9250_task_handle = NULL;
static TaskHandle_t supervisor_task_handle = NULL;

extern SemaphoreHandle_t xSerialSemaphore;

/* DMA IRQ binary semaphores are defined/owned in i2c_dma.c.
 * main.c only creates the objects before the scheduler starts.
 *
 * BME680:  RX -> DMA1_Stream5, TX -> DMA1_Stream6
 * MPU9250: RX -> DMA1_Stream1, TX -> DMA1_Stream4
 */
extern SemaphoreHandle_t bme680_rx_sem;
extern SemaphoreHandle_t bme680_tx_sem;
extern SemaphoreHandle_t mpu9250_rx_sem;
extern SemaphoreHandle_t mpu9250_tx_sem;

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

app_mpu9250_runtime_status_t mpu_runtime_rc;
/* ---------------- app status enum ---------------- */
//typedef enum {
//    APP_STATUS_OK = 0,
//    APP_STATUS_INIT_FAIL = 1,
//} app_status_t;


/* Initializes the sensor application and starts FreeRTOS. */
int main(void)
{
	/* Create the sensor and supervisor tasks before starting the scheduler. */

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

    supervisor_heartbeats_init();

    xSerialSemaphore = xSemaphoreCreateMutex();
    if (xSerialSemaphore == NULL) {
        on_fatal_init_error("xSerialSemaphore create failed");
        /*NVIC_SystemReset();*/
    }

    /* Binary semaphores for BME680 I2C1 DMA IRQ completion.
     * These objects are defined in i2c_dma.c and given from:
     *   RX -> DMA1_Stream5_IRQHandler
     *   TX -> DMA1_Stream6_IRQHandler
     */
    bme680_rx_sem = xSemaphoreCreateBinary();
    if (bme680_rx_sem == NULL) {
        on_fatal_init_error("bme680_rx_sem create failed");
        /*NVIC_SystemReset();*/
    }

    bme680_tx_sem = xSemaphoreCreateBinary();
    if (bme680_tx_sem == NULL) {
    	/*NVIC_SystemReset();*/
        on_fatal_init_error("bme680_tx_sem create failed");
    }

    /* Binary semaphores for MPU9250 I2C3 DMA IRQ completion.
     * These objects are defined in i2c_dma.c and given from:
     *   RX -> DMA1_Stream1_IRQHandler
     *   TX -> DMA1_Stream4_IRQHandler
     */
    mpu9250_rx_sem = xSemaphoreCreateBinary();
    if (mpu9250_rx_sem == NULL) {
    	/*NVIC_SystemReset();*/
        on_fatal_init_error("mpu9250_rx_sem create failed");
    }

    mpu9250_tx_sem = xSemaphoreCreateBinary();
    if (mpu9250_tx_sem == NULL) {
    	/*NVIC_SystemReset();*/
        on_fatal_init_error("mpu9250_tx_sem create failed");
    }

    if (xTaskCreate(bme680_task,
                    "bme680_task",
                    BME680_TASK_STACK_WORDS,
                    NULL,
                    BME680_SENSOR_TASK_PRIORITY,
                    &bme680_task_handle) != pdPASS) {
        on_fatal_init_error("bme680_task create failed");
    	/*NVIC_SystemReset();*/
    }

    if (xTaskCreate(mpu9250_task,
                    "mpu9250_task",
                    MPU9250_TASK_STACK_WORDS,
                    NULL,
                    MPU9250_SENSOR_TASK_PRIORITY,
                    &mpu9250_task_handle) != pdPASS) {
        on_fatal_init_error("mpu9250_task create failed");
    	/*NVIC_SystemReset();*/
    }

    if (xTaskCreate(supervisor_task,
                    "supervisor_task",
                    SUPERVISOR_TASK_STACK_WORDS,
                    NULL,
                    SUPERVISOR_TASK_PRIORITY,
                    &supervisor_task_handle) != pdPASS) {
        on_fatal_init_error("supervisor_task create failed");
    	//NVIC_SystemReset();
    }

    vTaskStartScheduler();

    while (1) {
        NVIC_SystemReset();
    }
}

/* ---------------- FreeRTOS runtime tasks ---------------- */

/* Runs periodic BME680 measurements under FreeRTOS. */
static void bme680_task(void *pvParameters)
{
    /* Suspend the task after reporting an unrecoverable measurement failure. */
    (void)pvParameters;

    for (;;) {
        const app_bme680_runtime_status_t bme_rc =
            app_bme680_sensor_runtime_process();

        if (bme_rc != APP_BME680_RUNTIME_OK) {
            BME680_LOG_PRINTF("[ERROR] bme680_task: app_bme680_sensor_runtime_process rc=%d\r\n",
                              (int)bme_rc);
            tim2_delay_ms(20U);

            request_iwdg_reset("BME680 read sensor parameters unrecoverable");
            //NVIC_SystemReset();
            vTaskSuspend(NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/* Runs periodic MPU9250 measurements under FreeRTOS. */
static void mpu9250_task(void *pvParameters)
{
    /* Stop further measurements after reporting a fatal sensor error. */
    (void)pvParameters;

    for (;;) {
        const app_mpu9250_runtime_status_t mpu_rc =
            app_mpu9250_runtime_process();

        /*debug_log_bme680_task_state();*/

        if (mpu_rc != APP_MPU9250_RUNTIME_OK) {
            MPU9250_LOG_PRINTF("[ERROR] mpu9250_task: app_mpu9250_runtime_process rc=%d\r\n",
                               (int)mpu_rc);
            tim2_delay_ms(20U);

            request_iwdg_reset("MPU9250 read sensor parameters unrecoverable");
            //NVIC_SystemReset();
            vTaskSuspend(NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/* Maps a FreeRTOS task state to diagnostic text. */
static const char *task_state_to_str(eTaskState state)
{
    /* Use UNKNOWN for a state outside the listed enum values. */
    switch (state) {
        case eRunning:   return "RUNNING";
        case eReady:     return "READY";
        case eBlocked:   return "BLOCKED";
        case eSuspended: return "SUSPENDED";
        case eDeleted:   return "DELETED";
        case eInvalid:   return "INVALID";
        default:         return "UNKNOWN";
    }
}

/* Reports BME680 task state, heartbeat ages and stack margin. */
static void debug_log_bme680_task_state(void)
{
    /* Limit diagnostic reports to one every two seconds. */
    static TickType_t last_debug_tick = 0U;
    TickType_t now_tick = xTaskGetTickCount();

    uint32_t now_ms = ms_get();
    uint32_t age_bme680_i2c  = now_ms - hb_bme680_ts;
    uint32_t age_bme680_main = now_ms - hb_bme680main_ts;

    eTaskState bme_state;
    UBaseType_t bme_stack_left;

    if ((now_tick - last_debug_tick) < pdMS_TO_TICKS(2000U)) {
        return;
    }

    last_debug_tick = now_tick;

    if (bme680_task_handle == NULL) {
        MPU9250_LOG_PRINTF("[DBG][MPU9250] BME680 task handle is NULL\r\n");
        tim2_delay_ms(20U);
        return;
    }

    bme_state = eTaskGetState(bme680_task_handle);
    bme_stack_left = uxTaskGetStackHighWaterMark(bme680_task_handle);

    MPU9250_LOG_PRINTF("[DBG][MPU9250] BME680 task state=%s main_age=%lu ms i2c_age=%lu ms stack_left=%lu words\r\n",
                       task_state_to_str(bme_state),
                       (unsigned long)age_bme680_main,
                       (unsigned long)age_bme680_i2c,
                       (unsigned long)bme_stack_left);
    tim2_delay_ms(20U);
}

/* Runs periodic heartbeat and watchdog supervision. */
static void supervisor_task(void *pvParameters)
{
    /* Allow a startup interval before the first supervisor check. */
    TickType_t last_wake;

    (void)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(500U));

    last_wake = xTaskGetTickCount();

    for (;;) {
        supervisor_check_and_kick();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SUPERVISOR_TASK_PERIOD_MS));
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

