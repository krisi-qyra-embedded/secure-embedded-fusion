/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-27
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
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
#include "max31865.h"
#include "spi_dma.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "pm2_5.h"
#include "pm25_uart.h"
#include "ds18b20.h"
#include "tim3.h"
#include "ESP32_uart.h"
#include "ecdh.h"
#include "stm32_gcm_nvm.h"
#include "nonce.h"

/*
 * DEBUG ONLY:
 * 1U = do not start the hardware IWDG, do not initialize heartbeat
 *      supervision, and suspend the supervisor task immediately.
 *      This prevents the 10-second ESP32 UART heartbeat miss from resetting
 *      the MCU while the AES-GCM/UART path is being debugged.
 *
 * 0U = normal production/runtime watchdog + supervisor behavior.
 */
#ifndef DEBUG_DISABLE_IWDG_AND_SUPERVISOR
#define DEBUG_DISABLE_IWDG_AND_SUPERVISOR    (1U)
#endif

/* ---------------- app status enum (declare in main.c as requested) ---------------- */
static app_status_t app_bme680_sensor_stack_init_(void);
static app_status_t app_bme680_peripherals_init(void);
static app_status_t app_max31865_sensor_stack_init(void);
static app_status_t app_max31865_peripherals_init(void);
static void app_crypto_and_esp32_link_init(void);
static void app_pre_iwdg_fatal_reset(const char *reason);
void Error_Handler(void);

/* ---------------- FreeRTOS sensor task configuration ---------------- */
#define BME680_TASK_STACK_WORDS      		(1536U)
#define MPU9250_TASK_STACK_WORDS     		(1536U)
#define SENSOR_TASK_PERIOD_MS        		(1000U)
#define BME680_SENSOR_TASK_PRIORITY         (tskIDLE_PRIORITY + 3U)
#define MPU9250_SENSOR_TASK_PRIORITY        (tskIDLE_PRIORITY + 3U)
#define SUPERVISOR_TASK_STACK_WORDS  		(768U)
#define SUPERVISOR_TASK_PERIOD_MS    		(250U)
#define SUPERVISOR_TASK_PRIORITY     		(tskIDLE_PRIORITY + 4U)
#define MAX31865_TASK_STACK_WORDS        	(1536U)
#define MAX31865_SENSOR_TASK_PRIORITY       (tskIDLE_PRIORITY + 3U)
#ifndef PM25_TASK_STACK_WORDS
#define PM25_TASK_STACK_WORDS        		(1536U)
#endif
#ifndef PM25_SENSOR_TASK_PRIORITY
#define PM25_SENSOR_TASK_PRIORITY    		BME680_SENSOR_TASK_PRIORITY
#endif

#ifndef DS18B20_TASK_STACK_WORDS
#define DS18B20_TASK_STACK_WORDS       	(2048U)
#endif
#ifndef DS18B20_SENSOR_TASK_PRIORITY
#define DS18B20_SENSOR_TASK_PRIORITY   	(tskIDLE_PRIORITY + 3U)
#endif

#ifndef ESP32_UART_AES_GCM_TASK_STACK_WORDS
#define ESP32_UART_AES_GCM_TASK_STACK_WORDS   (2048U)
#endif

#ifndef ESP32_UART_AES_GCM_TASK_PRIORITY
#define ESP32_UART_AES_GCM_TASK_PRIORITY      (tskIDLE_PRIORITY + 3U)
#endif

#ifndef ESP32_UART_AES_GCM_TASK_PERIOD_MS
#define ESP32_UART_AES_GCM_TASK_PERIOD_MS     (1000U)
#endif

static void bme680_task(void *pvParameters);
static void mpu9250_task(void *pvParameters);
static void max31865_task(void *argument);
static void pm25_task(void *argument);
static void ds18b20_task(void *argument);
static void esp32_uart_aes_gcm_task(void *argument);
static void supervisor_task(void *pvParameters);
static void debug_log_bme680_task_state(void);

static TaskHandle_t bme680_task_handle = NULL;
static TaskHandle_t mpu9250_task_handle = NULL;
static TaskHandle_t max31865_task_handle = NULL;
static TaskHandle_t pm25_task_handle = NULL;
static TaskHandle_t ds18b20_task_handle = NULL;
static TaskHandle_t esp32_uart_aes_gcm_task_handle = NULL;
static TaskHandle_t supervisor_task_handle = NULL;
static PMS25_App_t  g_pm25_app;
static DS18B20_App_t g_ds18b20_app;

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
extern SemaphoreHandle_t max31865_spi_rx_sem;
extern SemaphoreHandle_t max31865_spi_tx_sem;
extern SemaphoreHandle_t pm25_uart_rx_sem;
extern SemaphoreHandle_t esp32_uart_rx_sem;
extern SemaphoreHandle_t esp32_uart_tx_sem;

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
UART_Status_t uart_st;
tim_status_t tim_st;
DS18B20_Status_t ds_st;
DS18B20_AppRuntimeStatus_t app_st_ds18b20;

crypto_ctx_t g_crypto_ctx;
/* ---------------- app status enum ---------------- */
//typedef enum {
//    APP_STATUS_OK = 0,
//    APP_STATUS_INIT_FAIL = 1,
//} app_status_t;

/* Initializes the sensor application and starts FreeRTOS. */
int main(void)
{
	/* Initialize the directional crypto state before creating the encrypted link task. */

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
        log_printf(
            "[ERR] MX_TIM3_Init failed, status=%d\r\n",
            (int)tim_st);

        delay_ms(10U);

        on_fatal_init_error(
            "TIM3 1 MHz OneWire delay timer initialization failed");
    }

    //	if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS) {
    //	        NVIC_SystemReset();
    //	    }

    check_reset_source();

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

    if (app_max31865_peripherals_init() != APP_STATUS_OK) {
        on_fatal_init_error("app_max31865_peripherals_init failed");
        /*NVIC_SystemReset();*/
    }

    if (app_max31865_sensor_stack_init() != APP_STATUS_OK) {
        on_fatal_init_error("app_max31865_sensor_stack_init failed");
        /*NVIC_SystemReset();*/
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

    app_crypto_and_esp32_link_init();

#if (DEBUG_DISABLE_IWDG_AND_SUPERVISOR == 0U)
    supervisor_heartbeats_init();
#else
    log_printf(
        "[DEBUG] Heartbeat supervision disabled; supervisor will be suspended.\r\n");
#endif

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

    /* Binary semaphores for MAX31865 SPI4 DMA IRQ completion.
     * These objects are defined in spi_dma.c and given from:
     *   RX -> DMA2_Stream0_IRQHandler
     *   TX -> DMA2_Stream1_IRQHandler
     */
    max31865_spi_rx_sem = xSemaphoreCreateBinary();
    if (max31865_spi_rx_sem == NULL) {
        on_fatal_init_error("max31865_spi_rx_sem create failed");
        /*NVIC_SystemReset();*/
    }

    max31865_spi_tx_sem = xSemaphoreCreateBinary();
    if (max31865_spi_tx_sem == NULL) {
        on_fatal_init_error("max31865_spi_tx_sem create failed");
        /*NVIC_SystemReset();*/
    }

    /* Binary semaphore for PM2.5 USART1 RX DMA completion.
     * This object is used by the PM2.5 UART RX DMA path:
     *   RX -> DMA2_Stream5_IRQHandler
     *   USART1 RX complete -> HAL_UART_RxCpltCallback()
     */
    pm25_uart_rx_sem = xSemaphoreCreateBinary();
    if (pm25_uart_rx_sem == NULL) {
        on_fatal_init_error("pm25_uart_rx_sem create failed");
        /*NVIC_SystemReset();*/
    }

    /*
     * Binary semaphores for interrupt-driven USART6 completion.
     * ESP32_uart.c owns the handles and gives them from USART6_IRQHandler:
     *   RX -> complete encrypted 39-byte response or RX/error condition
     *   TX -> final USART6 transmission-complete interrupt
     */
    esp32_uart_rx_sem = xSemaphoreCreateBinary();
    if (esp32_uart_rx_sem == NULL) {
        on_fatal_init_error("esp32_uart_rx_sem create failed");
    }

    esp32_uart_tx_sem = xSemaphoreCreateBinary();
    if (esp32_uart_tx_sem == NULL) {
        on_fatal_init_error("esp32_uart_tx_sem create failed");
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

    if (xTaskCreate(max31865_task,
                    "max31865_task",
                    MAX31865_TASK_STACK_WORDS,
                    NULL,
                    MAX31865_SENSOR_TASK_PRIORITY,
                    &max31865_task_handle) != pdPASS) {
        on_fatal_init_error("max31865_task create failed");
        /*NVIC_SystemReset();*/
    }

    if (xTaskCreate(pm25_task,
                    "pm25_task",
                    PM25_TASK_STACK_WORDS,
                    NULL,
                    PM25_SENSOR_TASK_PRIORITY,
                    &pm25_task_handle) != pdPASS) {
        on_fatal_init_error("pm25_task create failed");
        /*NVIC_SystemReset();*/
    }

    if (xTaskCreate(ds18b20_task,
                    "ds18b20_task",
                    DS18B20_TASK_STACK_WORDS,
                    NULL,
                    DS18B20_SENSOR_TASK_PRIORITY,
                    &ds18b20_task_handle) != pdPASS) {
        on_fatal_init_error("ds18b20_task create failed");
        /*NVIC_SystemReset();*/
    }

    if (xTaskCreate(esp32_uart_aes_gcm_task,
                    "esp32_gcm_task",
                    ESP32_UART_AES_GCM_TASK_STACK_WORDS,
                    NULL,
                    ESP32_UART_AES_GCM_TASK_PRIORITY,
                    &esp32_uart_aes_gcm_task_handle) != pdPASS) {
        on_fatal_init_error("esp32_uart_aes_gcm_task create failed");
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

#if (DEBUG_DISABLE_IWDG_AND_SUPERVISOR == 0U)
    {
        const iwdg_status_t iwdg_status =
            iwdg_init(IWDG_PRESCALER_128, 2499U, 0U);

        if (iwdg_status != IWDG_OK)
        {
            log_printf("iwdg_init() failed: %d\r\n", (int)iwdg_status);
            app_fault_delay_ms(2U);
            NVIC_SystemReset();
        }
    }
#else
    log_printf(
        "[DEBUG] Hardware IWDG disabled for AES-GCM/UART debugging.\r\n");
#endif

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
    TickType_t last_wake;

    (void)pvParameters;

    last_wake = xTaskGetTickCount();

    for (;;) {
        const app_bme680_runtime_status_t bme_rc =
            app_bme680_sensor_runtime_process();

        if (bme_rc != APP_BME680_RUNTIME_OK) {

            /*
             * Defensive task-level fault lifecycle:
             * driver recovery normally marks in_progress earlier,
             * but the task also guarantees active -> completed before reset request.
             */
            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_BME680);

            BME680_LOG_PRINTF("[ERROR] bme680_task: app_bme680_sensor_runtime_process rc=%d -> reporting fatal to supervisor\r\n",
                              (int)bme_rc);
            app_fault_delay_ms(20U);

            system_fault_mark_completed(SYSTEM_FAULT_OWNER_BME680);

            request_iwdg_reset("BME680 read sensor parameters unrecoverable");

            vTaskSuspend(NULL);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/* Runs periodic MPU9250 measurements under FreeRTOS. */
static void mpu9250_task(void *pvParameters)
{
    /* Stop further measurements after reporting a fatal sensor error. */
    TickType_t last_wake;

    (void)pvParameters;

    last_wake = xTaskGetTickCount();

    for (;;) {
        const app_mpu9250_runtime_status_t mpu_rc =
            app_mpu9250_runtime_process();

        if (mpu_rc != APP_MPU9250_RUNTIME_OK) {

            /*
             * Defensive task-level fault lifecycle:
             * driver recovery normally marks in_progress earlier,
             * but the task also guarantees active -> completed before reset request.
             */
            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MPU9250);

            MPU9250_LOG_PRINTF("[ERROR] mpu9250_task: app_mpu9250_runtime_process rc=%d -> reporting fatal to supervisor\r\n",
                               (int)mpu_rc);
            app_fault_delay_ms(20U);

            system_fault_mark_completed(SYSTEM_FAULT_OWNER_MPU9250);

            request_iwdg_reset("MPU9250 read sensor parameters unrecoverable");

            vTaskSuspend(NULL);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/* Processes PM2.5 frames and runtime recovery under FreeRTOS. */
static void pm25_task(void *argument)
{
    /* Suspend the task after reporting fatal or invalid runtime state. */
    (void)argument;

    for (;;) {
        const PMS25_AppRuntimeStatus_t app_st = PMS25_AppRun(&g_pm25_app);

        if (app_st == PMS25_APP_RUNTIME_OK) {
            continue;
        }

        if (app_st == PMS25_APP_RUNTIME_FATAL) {
            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_PM25);

            PM25_LOG_PRINTF("[FATAL][PM25] All recovery attempts exhausted -> reporting fatal to supervisor\r\n");
            app_fault_delay_ms(20U);

            system_fault_mark_completed(SYSTEM_FAULT_OWNER_PM25);

            request_iwdg_reset("PM2.5 runtime failure after 3 total recovery attempts");

            vTaskSuspend(NULL);
        }

        if (app_st == PMS25_APP_RUNTIME_BAD_PARAM) {
            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_PM25);

            PM25_LOG_PRINTF("[FATAL][PM25] PMS25_AppRun returned BAD_PARAM -> reporting fatal to supervisor\r\n");
            app_fault_delay_ms(20U);

            system_fault_mark_completed(SYSTEM_FAULT_OWNER_PM25);

            request_iwdg_reset("PM2.5 runtime bad parameter");

            vTaskSuspend(NULL);
        }

        system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_PM25);

        PM25_LOG_PRINTF("[FATAL][PM25] PMS25_AppRun returned unexpected status=%d -> reporting fatal to supervisor\r\n",
                        (int)app_st);
        app_fault_delay_ms(20U);

        system_fault_mark_completed(SYSTEM_FAULT_OWNER_PM25);

        request_iwdg_reset("PM2.5 runtime unexpected status");

        vTaskSuspend(NULL);
    }
}

/* Runs the DS18B20 conversion and recovery state machine. */
static void ds18b20_task(void *argument)
{
    /* Request a reset and suspend the task after fatal recovery failure. */
    DS18B20_AppRuntimeStatus_t st;

    (void)argument;

    for (;;)
    {
        st = DS18B20_AppRun(&g_ds18b20_app);

        if ((st == DS18B20_APP_RUNTIME_FATAL) ||
            (st == DS18B20_APP_RUNTIME_RECOVERY_FAIL))
        {
            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_DS18B20);

            DS18B20_LOG_PRINTF("[FATAL][DS18B20] Runtime failure after recovery budget exhausted -> reporting fatal to supervisor\r\n");
            app_fault_delay_ms(20U);

            system_fault_mark_completed(SYSTEM_FAULT_OWNER_DS18B20);

            request_iwdg_reset("DS18B20 runtime failure after 3 recovery attempts");

            vTaskSuspend(NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

/* Runs periodic MAX31865 temperature measurements. */
static void max31865_task(void *pvParameters)
{
    /* Suspend sampling when the runtime recovery budget is exhausted. */
    TickType_t last_wake;

    (void)pvParameters;

    last_wake = xTaskGetTickCount();

    for (;;) {
        const app_max31865_runtime_status_t max_rc =
            app_max31865_runtime_process();

        if (max_rc != APP_MAX31865_RUNTIME_OK) {

            /*
             * Defensive task-level fault lifecycle:
             * app_max31865_runtime_process() normally marks fault in-progress,
             * but the task also guarantees active -> completed before reset request.
             */
            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MAX31865);

            MAX31865_LOG_PRINTF("[ERROR] max31865_task: app_max31865_runtime_process rc=%d -> reporting fatal to supervisor\r\n",
                                (int)max_rc);
            app_fault_delay_ms(20U);

            system_fault_mark_completed(SYSTEM_FAULT_OWNER_MAX31865);

            request_iwdg_reset("MAX31865 read temperature unrecoverable");

            vTaskSuspend(NULL);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/* Runs encrypted ESP32 transactions under FreeRTOS. */
static void esp32_uart_aes_gcm_task(void *argument)
{
    /* Suspend the link task after reporting an unrecoverable transaction. */
    TickType_t last_wake;

    (void)argument;

    last_wake = xTaskGetTickCount();

    for (;;)
    {
        const ESP32_UART_RuntimeStatus_t link_rc =
            ESP32_UART_LinkTest_RunOnce();

        if (link_rc != ESP32_UART_RUNTIME_OK)
        {
            /*
             * Defensive task-level fault lifecycle:
             * ESP32_uart.c marks the owner active/in-progress on the first
             * failed attempt. The task guarantees active -> completed before
             * requesting the coordinated watchdog reset.
             */
            system_fault_mark_in_progress(
                SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM);

            ESP32_LOG_PRINTF(
                "[FATAL][ESP32_UART_AES_GCM] "
                "ESP32_UART_LinkTest_RunOnce rc=%d -> "
                "reporting fatal to supervisor\r\n",
                (int)link_rc);

            app_fault_delay_ms(20U);

            system_fault_mark_completed(
                SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM);

            request_iwdg_reset(
                "ESP32 UART AES-GCM transaction unrecoverable");

            vTaskSuspend(NULL);
        }

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(
                ESP32_UART_AES_GCM_TASK_PERIOD_MS));
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
        app_fault_delay_ms(20U);
        return;
    }

    bme_state = eTaskGetState(bme680_task_handle);
    bme_stack_left = uxTaskGetStackHighWaterMark(bme680_task_handle);

    MPU9250_LOG_PRINTF("[DBG][MPU9250] BME680 task state=%s main_age=%lu ms i2c_age=%lu ms stack_left=%lu words\r\n",
                       task_state_to_str(bme_state),
                       (unsigned long)age_bme680_main,
                       (unsigned long)age_bme680_i2c,
                       (unsigned long)bme_stack_left);
    app_fault_delay_ms(20U);
}

/* Runs periodic heartbeat and watchdog supervision. */
static void supervisor_task(void *pvParameters)
{
    /* Allow a startup interval before the first supervisor check. */
    TickType_t last_wake;

    (void)pvParameters;

#if (DEBUG_DISABLE_IWDG_AND_SUPERVISOR == 1U)
    vTaskSuspend(NULL);

    for (;;)
    {
        vTaskDelay(portMAX_DELAY);
    }
#else
    vTaskDelay(pdMS_TO_TICKS(500U));

    last_wake = xTaskGetTickCount();

    for (;;) {
        supervisor_check_and_kick();

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(SUPERVISOR_TASK_PERIOD_MS));
    }
#endif
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

/* Resets the device after a failure before watchdog startup. */
static void app_pre_iwdg_fatal_reset(const char *reason)
{
    /* Drain the diagnostic message before requesting the system reset. */
    log_printf(
        "[FATAL][PRE-IWDG] %s -> NVIC system reset\r\n",
        (reason != NULL) ? reason : "(no reason)");

    app_fault_delay_ms(20U);

    __DSB();
    __ISB();
    NVIC_SystemReset();

    for (;;)
    {
        __NOP();
    }
}

/* Initializes the encrypted ESP32 link and directional keys. */
static void app_crypto_and_esp32_link_init(void)
{
    /* Require usable GCM state and crypto initialization before starting the link task. */
    int32_t crypto_rc;
    int32_t startup_nonce_rc;
    stm32_gcm_nvm_status_t gcm_nvm_rc;

    if (ESP32_UART_LinkTest_Init() !=
        ESP32_UART_STATUS_OK)
    {
        app_pre_iwdg_fatal_reset(
            "ESP32 UART link initialization failed");
    }

    /*
     * Scan and validate the configured append-only GCM journal.
     * The flash sector is selected in stm32_gcm_nvm.h; it is deliberately
     * not hard-coded in this startup routine.
     */
    gcm_nvm_rc =
        stm32_gcm_nvm_init();

    if (gcm_nvm_rc != STM32_GCM_NVM_OK)
    {
        ECDH_LOG_PRINTF(
            "[SECURITY][ERROR] STM32 GCM journal "
            "initialization failed, rc=%d\r\n",
            (int)gcm_nvm_rc);

        app_fault_delay_ms(20U);

        app_pre_iwdg_fatal_reset(
            "Persistent GCM journal initialization failed");
    }

    adc_byte_buffers_init();

    if (cmox_initialize(NULL) !=
        CMOX_INIT_SUCCESS)
    {
        app_pre_iwdg_fatal_reset(
            "cmox_initialize failed");
    }

    crypto_rc =
        (int32_t)entropy_personalisation_HW_init();

    if (crypto_rc !=
        (int32_t)ENTROPY_HW_INIT_SUCCESS)
    {
        ECDH_LOG_PRINTF(
            "[CRYPTO][ERROR] "
            "entropy_personalisation_HW_init failed, rc=%ld\r\n",
            (long)crypto_rc);

        app_fault_delay_ms(20U);

        app_pre_iwdg_fatal_reset(
            "Entropy hardware initialization failed");
    }

    crypto_rc =
        (int32_t)entropy_personalisation_hash();

    if (crypto_rc !=
        (int32_t)ENTROPY_CREATION_SUCCESS)
    {
        ECDH_LOG_PRINTF(
            "[CRYPTO][ERROR] "
            "entropy_personalisation_hash failed, rc=%ld\r\n",
            (long)crypto_rc);

        app_fault_delay_ms(20U);

        app_pre_iwdg_fatal_reset(
            "Entropy personalization hash failed");
    }

    startup_nonce_rc =
        (int32_t)nonce_simple_init();

    if (startup_nonce_rc !=
        (int32_t)NONCE_OK)
    {
        ECDH_LOG_PRINTF(
            "[CRYPTO][ERROR] nonce_simple_init failed, rc=%ld\r\n",
            (long)startup_nonce_rc);

        app_fault_delay_ms(20U);

        app_pre_iwdg_fatal_reset(
            "Startup nonce initialization failed");
    }

    crypto_rc =
        (int32_t)nonce_generation();

    if (crypto_rc !=
        (int32_t)NONCE_GEN_SUCCESS)
    {
        ECDH_LOG_PRINTF(
            "[CRYPTO][ERROR] nonce_generation failed, rc=%ld\r\n",
            (long)crypto_rc);

        app_fault_delay_ms(20U);

        app_pre_iwdg_fatal_reset(
            "Startup nonce hash generation failed");
    }

    crypto_ctx_init(
        &g_crypto_ctx);

    crypto_rc =
        (int32_t)sender_init_ecdh_crypto(
            &g_crypto_ctx);

    if (crypto_rc !=
        (int32_t)SENDER_INIT_SUCCESS)
    {
        ECDH_LOG_PRINTF(
            "[CRYPTO][ERROR] sender_init_ecdh_crypto "
            "failed, rc=%ld\r\n",
            (long)crypto_rc);

        app_fault_delay_ms(20U);

        app_pre_iwdg_fatal_reset(
            "STM32->ESP32 sender crypto initialization failed");
    }

    crypto_rc =
        (int32_t)receiver_init_ecdh_crypto(
            &g_crypto_ctx);

    if (crypto_rc !=
        (int32_t)RECEIVER_INIT_SUCCESS)
    {
        ECDH_LOG_PRINTF(
            "[CRYPTO][ERROR] receiver_init_ecdh_crypto "
            "failed, rc=%ld\r\n",
            (long)crypto_rc);

        app_fault_delay_ms(20U);

        app_pre_iwdg_fatal_reset(
            "ESP32->STM32 receiver crypto initialization failed");
    }

    crypto_rc =
        (int32_t)gcm_nonce_session_init(
            &g_crypto_ctx);

    if (crypto_rc !=
        (int32_t)GCM_NONCE_OK)
    {
        ECDH_LOG_PRINTF(
            "[CRYPTO][ERROR] gcm_nonce_session_init "
            "failed, rc=%ld\r\n",
            (long)crypto_rc);

        app_fault_delay_ms(20U);

        app_pre_iwdg_fatal_reset(
            "STM32 TX GCM nonce session initialization failed");
    }

    ECDH_LOG_PRINTF(
        "\r\n[CRYPTO] Full-duplex directional keys are ready.\r\n");

    ECDH_LOG_PRINTF(
        "[CRYPTO] TX: STM32 sender private x "
        "ESP32 receiver public.\r\n");

    ECDH_LOG_PRINTF(
        "[CRYPTO] RX: STM32 receiver private x "
        "ESP32 sender public.\r\n");

    ECDH_LOG_PRINTF(
        "[CRYPTO] RX nonce is received from ESP32; "
        "the STM32 does not allocate the reverse nonce.\r\n");

    ECDH_LOG_PRINTF(
        "[MAIN] FreeRTOS encrypted UART task will run every %u ms.\r\n",
        (unsigned)ESP32_UART_AES_GCM_TASK_PERIOD_MS);
}

/* Halts execution after an unrecoverable application error. */
void Error_Handler(void)
{
    /* Disable interrupts before entering the terminal wait. */
    __disable_irq();
    while (1)
    {
    }
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

/* Initializes the MAX31865 chip select and SPI4 DMA. */
static app_status_t app_max31865_peripherals_init(void)
{
    /* Stop initialization if either GPIO or SPI setup fails. */
    if (gpio_pb6_cs_init() != 0) {
        MAX31865_LOG_PRINTF("[ERROR] app_max31865_peripherals_init: gpio_pb6_cs_init failed\r\n");
        app_fault_delay_ms(20U);
        return APP_STATUS_INIT_FAIL;
    }

    if (spi4_dma_init() != SPI_DMA_STATUS_OK) {
        MAX31865_LOG_PRINTF("[ERROR] app_max31865_peripherals_init: spi4_dma_init failed\r\n");
        app_fault_delay_ms(20U);
        return APP_STATUS_INIT_FAIL;
    }

    return APP_STATUS_OK;
}

/* Initializes the application's MAX31865 device. */
static app_status_t app_max31865_sensor_stack_init(void)
{
    /* Clear device state before selecting three-wire, 50 Hz operation. */
    memset(&max31865_dev, 0, sizeof(max31865_dev));

    if (Max31865_init(&max31865_dev, 3U, 50U) != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("[ERROR] app_max31865_sensor_stack_init: Max31865_init failed\r\n");
        app_fault_delay_ms(20U);
        return APP_STATUS_INIT_FAIL;
    }

    return APP_STATUS_OK;
}
