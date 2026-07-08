/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
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
#include "cmox_crypto.h"
#include "entropy_personalization_nonce_gen.h"
#include "nonce.h"

/* ---------------- app status enum (declare in main.c as requested) ---------------- */
static app_status_t app_bme680_sensor_stack_init_(void);
static app_status_t app_bme680_peripherals_init(void);
static app_status_t app_max31865_sensor_stack_init(void);
static app_status_t app_max31865_peripherals_init(void);
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


static void bme680_task(void *pvParameters);
static void mpu9250_task(void *pvParameters);
static void max31865_task(void *argument);
static void pm25_task(void *argument);
static void ds18b20_task(void *argument);
static void supervisor_task(void *pvParameters);
static void debug_log_bme680_task_state(void);

static TaskHandle_t bme680_task_handle = NULL;
static TaskHandle_t mpu9250_task_handle = NULL;
static TaskHandle_t max31865_task_handle = NULL;
static TaskHandle_t pm25_task_handle = NULL;
static TaskHandle_t ds18b20_task_handle = NULL;
static TaskHandle_t supervisor_task_handle = NULL;
static PMS25_App_t  g_pm25_app;
static DS18B20_App_t g_ds18b20_app;

extern SemaphoreHandle_t xSerialSemaphore;

crypto_ctx_t g_crypto_ctx;

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
/* ---------------- app status enum ---------------- */
//typedef enum {
//    APP_STATUS_OK = 0,
//    APP_STATUS_INIT_FAIL = 1,
//} app_status_t;

/* Initializes crypto and runs one encrypted startup transaction. */
int main(void)
{
    /* Require successful key and nonce setup before sending the startup frame. */
    int32_t crypto_rc;
    nonce_status_t startup_nonce_rc;

    HAL_Init();
    clock_100_mhz_init();

    memset(&MPU9250, 0, sizeof(MPU9250));

    SCB->CPACR |=
        ((3UL << (10U * 2U)) |
         (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    if (systick_init(96000000U, 6U) != SYSTICK_OK)
    {
        NVIC_SystemReset();
    }

    if (systick_selftest(200U) != SYSTICK_OK)
    {
        NVIC_SystemReset();
    }

    tim2_1s_init();

    /*
     * USART2 is the debug log and USART6 is the STM32->ESP32 link.
     */
    uart2_tx_rx_init();

    if (ESP32_UART_LinkTest_Init() != ESP32_UART_STATUS_OK)
    {
        Error_Handler();
    }

    adc_byte_buffers_init();

    if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS)
    {
        handle_error("cmox_initialize failed");
    }

    crypto_rc =
        entropy_personalisation_HW_init();

    if (crypto_rc != ENTROPY_HW_INIT_SUCCESS)
    {
        log_printf(
            "[CRYPTO][ERROR] entropy_personalisation_HW_init "
            "failed, rc=%ld\r\n",
            (long)crypto_rc);

        handle_error("Entropy hardware initialization failed");
    }

    crypto_rc =
        entropy_personalisation_hash();

    if (crypto_rc != ENTROPY_CREATION_SUCCESS)
    {
        log_printf(
            "[CRYPTO][ERROR] entropy_personalisation_hash "
            "failed, rc=%ld\r\n",
            (long)crypto_rc);

        handle_error("Entropy personalization hash failed");
    }

    startup_nonce_rc =
        nonce_simple_init();

    if (startup_nonce_rc != NONCE_OK)
    {
        log_printf(
            "[CRYPTO][ERROR] nonce_simple_init failed, rc=%d\r\n",
            (int)startup_nonce_rc);

        handle_error("Startup nonce initialization failed");
    }

    crypto_rc =
        nonce_generation();

    if (crypto_rc != NONCE_GEN_SUCCESS)
    {
        log_printf(
            "[CRYPTO][ERROR] nonce_generation failed, rc=%ld\r\n",
            (long)crypto_rc);

        handle_error("Startup nonce hash generation failed");
    }

    crypto_ctx_init(&g_crypto_ctx);

    crypto_rc =
        sender_init_ecdh_crypto(
            &g_crypto_ctx);

    if (crypto_rc != SENDER_INIT_SUCCESS)
    {
        log_printf(
            "[CRYPTO][ERROR] sender_init_ecdh_crypto "
            "failed, rc=%ld\r\n",
            (long)crypto_rc);

        handle_error("STM32->ESP32 sender crypto initialization failed");
    }

    crypto_rc =
        gcm_nonce_session_init(
            &g_crypto_ctx);

    if (crypto_rc != GCM_NONCE_OK)
    {
        log_printf(
            "[CRYPTO][ERROR] gcm_nonce_session_init "
            "failed, rc=%ld\r\n",
            (long)crypto_rc);

        handle_error("GCM nonce session initialization failed");
    }

    log_printf(
        "\r\n"
        "[CRYPTO] STM32->ESP32 TX key and nonce session are ready.\r\n");

    ESP32_UART_LinkTest_RunOnce();

    log_printf(
        "[MAIN] Startup encrypted UART test complete; "
        "entering idle loop.\r\n");

    for (;;)
    {
        __WFI();
    }
}

/* ---------------- FreeRTOS runtime tasks ---------------- */

//static void bme680_task(void *pvParameters)
//{
//    TickType_t last_wake;
//
//    (void)pvParameters;
//
//    last_wake = xTaskGetTickCount();
//
//    for (;;) {
//        const app_bme680_runtime_status_t bme_rc =
//            app_bme680_sensor_runtime_process();
//
//        if (bme_rc != APP_BME680_RUNTIME_OK) {
//
//            /*
//             * Defensive task-level fault lifecycle:
//             * driver recovery normally marks in_progress earlier,
//             * but the task also guarantees active -> completed before reset request.
//             */
//            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_BME680);
//
//            BME680_LOG_PRINTF("[ERROR] bme680_task: app_bme680_sensor_runtime_process rc=%d -> reporting fatal to supervisor\r\n",
//                              (int)bme_rc);
//            app_fault_delay_ms(20U);
//
//            system_fault_mark_completed(SYSTEM_FAULT_OWNER_BME680);
//
//            request_iwdg_reset("BME680 read sensor parameters unrecoverable");
//
//            vTaskSuspend(NULL);
//        }
//
//        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
//    }
//}
//
//static void mpu9250_task(void *pvParameters)
//{
//    TickType_t last_wake;
//
//    (void)pvParameters;
//
//    last_wake = xTaskGetTickCount();
//
//    for (;;) {
//        const app_mpu9250_runtime_status_t mpu_rc =
//            app_mpu9250_runtime_process();
//
//        if (mpu_rc != APP_MPU9250_RUNTIME_OK) {
//
//            /*
//             * Defensive task-level fault lifecycle:
//             * driver recovery normally marks in_progress earlier,
//             * but the task also guarantees active -> completed before reset request.
//             */
//            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MPU9250);
//
//            MPU9250_LOG_PRINTF("[ERROR] mpu9250_task: app_mpu9250_runtime_process rc=%d -> reporting fatal to supervisor\r\n",
//                               (int)mpu_rc);
//            app_fault_delay_ms(20U);
//
//            system_fault_mark_completed(SYSTEM_FAULT_OWNER_MPU9250);
//
//            request_iwdg_reset("MPU9250 read sensor parameters unrecoverable");
//
//            vTaskSuspend(NULL);
//        }
//
//        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
//    }
//}
//
//static void pm25_task(void *argument)
//{
//    (void)argument;
//
//    for (;;) {
//        const PMS25_AppRuntimeStatus_t app_st = PMS25_AppRun(&g_pm25_app);
//
//        if (app_st == PMS25_APP_RUNTIME_OK) {
//            continue;
//        }
//
//        if (app_st == PMS25_APP_RUNTIME_FATAL) {
//            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_PM25);
//
//            PM25_LOG_PRINTF("[FATAL][PM25] All recovery attempts exhausted -> reporting fatal to supervisor\r\n");
//            app_fault_delay_ms(20U);
//
//            system_fault_mark_completed(SYSTEM_FAULT_OWNER_PM25);
//
//            request_iwdg_reset("PM2.5 runtime failure after 3 total recovery attempts");
//
//            vTaskSuspend(NULL);
//        }
//
//        if (app_st == PMS25_APP_RUNTIME_BAD_PARAM) {
//            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_PM25);
//
//            PM25_LOG_PRINTF("[FATAL][PM25] PMS25_AppRun returned BAD_PARAM -> reporting fatal to supervisor\r\n");
//            app_fault_delay_ms(20U);
//
//            system_fault_mark_completed(SYSTEM_FAULT_OWNER_PM25);
//
//            request_iwdg_reset("PM2.5 runtime bad parameter");
//
//            vTaskSuspend(NULL);
//        }
//
//        system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_PM25);
//
//        PM25_LOG_PRINTF("[FATAL][PM25] PMS25_AppRun returned unexpected status=%d -> reporting fatal to supervisor\r\n",
//                        (int)app_st);
//        app_fault_delay_ms(20U);
//
//        system_fault_mark_completed(SYSTEM_FAULT_OWNER_PM25);
//
//        request_iwdg_reset("PM2.5 runtime unexpected status");
//
//        vTaskSuspend(NULL);
//    }
//}
//
//static void ds18b20_task(void *argument)
//{
//    DS18B20_AppRuntimeStatus_t st;
//
//    (void)argument;
//
//    for (;;)
//    {
//        st = DS18B20_AppRun(&g_ds18b20_app);
//
//        if ((st == DS18B20_APP_RUNTIME_FATAL) ||
//            (st == DS18B20_APP_RUNTIME_RECOVERY_FAIL))
//        {
//            /*
//             * DS18B20_AppRun()/recovery already marks:
//             *   active/in_progress when recovery starts
//             *   completed when 3/3 recovery is exhausted
//             *
//             * This task-level sequence is defensive and idempotent:
//             * it guarantees the supervisor sees DS18B20 as active/completed
//             * before the reset request, matching the other sensor tasks.
//             */
//            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_DS18B20);
//
//            DS18B20_LOG_PRINTF("[FATAL][DS18B20] Runtime failure after recovery budget exhausted -> reporting fatal to supervisor\r\n");
//            app_fault_delay_ms(20U);
//
//            system_fault_mark_completed(SYSTEM_FAULT_OWNER_DS18B20);
//
//            /*
//             * Do NOT call escalate_to_iwdg_reset() directly here.
//             * request_iwdg_reset() lets supervisor_check_and_kick() wait for
//             * other captured sensor faults to complete their 3/3 logs first.
//             */
//            request_iwdg_reset("DS18B20 runtime failure after 3 recovery attempts");
//
//            vTaskSuspend(NULL);
//        }
//
//        vTaskDelay(pdMS_TO_TICKS(10U));
//    }
//}
//
//
//static void max31865_task(void *pvParameters)
//{
//    TickType_t last_wake;
//
//    (void)pvParameters;
//
//    last_wake = xTaskGetTickCount();
//
//    for (;;) {
//        const app_max31865_runtime_status_t max_rc =
//            app_max31865_runtime_process();
//
//        if (max_rc != APP_MAX31865_RUNTIME_OK) {
//
//            /*
//             * Defensive task-level fault lifecycle:
//             * app_max31865_runtime_process() normally marks fault in-progress,
//             * but the task also guarantees active -> completed before reset request.
//             */
//            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MAX31865);
//
//            MAX31865_LOG_PRINTF("[ERROR] max31865_task: app_max31865_runtime_process rc=%d -> reporting fatal to supervisor\r\n",
//                                (int)max_rc);
//            app_fault_delay_ms(20U);
//
//            system_fault_mark_completed(SYSTEM_FAULT_OWNER_MAX31865);
//
//            request_iwdg_reset("MAX31865 read temperature unrecoverable");
//
//            vTaskSuspend(NULL);
//        }
//
//        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
//    }
//}
//
//
//static const char *task_state_to_str(eTaskState state)
//{
//    switch (state) {
//        case eRunning:   return "RUNNING";
//        case eReady:     return "READY";
//        case eBlocked:   return "BLOCKED";
//        case eSuspended: return "SUSPENDED";
//        case eDeleted:   return "DELETED";
//        case eInvalid:   return "INVALID";
//        default:         return "UNKNOWN";
//    }
//}
//
//
//static void debug_log_bme680_task_state(void)
//{
//    static TickType_t last_debug_tick = 0U;
//    TickType_t now_tick = xTaskGetTickCount();
//
//    uint32_t now_ms = ms_get();
//    uint32_t age_bme680_i2c  = now_ms - hb_bme680_ts;
//    uint32_t age_bme680_main = now_ms - hb_bme680main_ts;
//
//    eTaskState bme_state;
//    UBaseType_t bme_stack_left;
//
//    /* Print only once every 2 seconds to avoid destroying your UART log */
//    if ((now_tick - last_debug_tick) < pdMS_TO_TICKS(2000U)) {
//        return;
//    }
//
//    last_debug_tick = now_tick;
//
//    if (bme680_task_handle == NULL) {
//        MPU9250_LOG_PRINTF("[DBG][MPU9250] BME680 task handle is NULL\r\n");
//        app_fault_delay_ms(20U);
//        return;
//    }
//
//    bme_state = eTaskGetState(bme680_task_handle);
//    bme_stack_left = uxTaskGetStackHighWaterMark(bme680_task_handle);
//
//    MPU9250_LOG_PRINTF("[DBG][MPU9250] BME680 task state=%s main_age=%lu ms i2c_age=%lu ms stack_left=%lu words\r\n",
//                       task_state_to_str(bme_state),
//                       (unsigned long)age_bme680_main,
//                       (unsigned long)age_bme680_i2c,
//                       (unsigned long)bme_stack_left);
//    app_fault_delay_ms(20U);
//}
//
//
//static void supervisor_task(void *pvParameters)
//{
//    TickType_t last_wake;
//
//    (void)pvParameters;
//
//    vTaskDelay(pdMS_TO_TICKS(500U));
//
//    last_wake = xTaskGetTickCount();
//
//    for (;;) {
//        supervisor_check_and_kick();
//
//        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SUPERVISOR_TASK_PERIOD_MS));
//    }
//}
//
//
//
///* USER CODE BEGIN 4 */
//void fnAPP_Init_Standard_Values(void){
//
//	bme.gas_weighting = 0.75;
//	bme.gas_reference = 2500;
//	bme.getgasreference_count = 0;
//	bme.hum_weighting = 0.25;
//	bme.hum_reference = 40;
//	bme.gas_lower_limit = 10000;
//	bme.gas_upper_limit = 300000;
//
//}
//
//
///* ---------------- static init functions for BME680 (like MAX31865 pattern) ---------------- */
//
//static app_status_t app_bme680_peripherals_init(void)
//{
//    /* 1) I2C1 on PB8/PB9 */
//    i2c_select_status_t rc = i2c_init_select(1U, 8U, 9U);
//    if (rc != I2C_SELECT_OK) {
//        log_printf("[ERROR] app_bme680_peripherals_init: i2c_init_select failed rc=%d\r\n", (int)rc);
//        return APP_STATUS_INIT_FAIL;
//    }
//
//    /* 2) Enable FPU (CP10/CP11 full access) */
//    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
//    __DSB();
//    __ISB();
//
//    /* 3) DMA streams for I2C1 BME680 (RX=Stream5, TX=Stream6) */
//    {
//        dma_status_t st;
//
//        /* RX stream (I2C1_RX -> DMA1 Stream5, Channel 1 in your setup) */
//        st = dma_stream_init(DMA1_Stream5,
//                             1U,
//                             DMA_DIR_P2M,
//                             &DMA1->HIFCR,
//                             DMA1_STREAM5_TCIF,
//                             DMA1_Stream5_IRQn);
//        if (st != DMA_OK) {
//            log_printf("[ERROR] app_bme680_peripherals_init: DMA1_Stream5 init failed rc=%d\r\n", (int)st);
//            return APP_STATUS_INIT_FAIL;
//        }
//
//        /* TX stream (I2C1_TX -> DMA1 Stream6, Channel 1 in your setup) */
//        st = dma_stream_init(DMA1_Stream6,
//                             1U,
//                             DMA_DIR_M2P,
//                             &DMA1->HIFCR,
//                             DMA1_STREAM6_TCIF,
//                             DMA1_Stream6_IRQn);
//        if (st != DMA_OK) {
//            log_printf("[ERROR] app_bme680_peripherals_init: DMA1_Stream6 init failed rc=%d\r\n", (int)st);
//            return APP_STATUS_INIT_FAIL;
//        }
//    }
//
//    /* keep your original call here (as it was placed after DMA init) */
//    fnAPP_Init_Standard_Values();
//
//    return APP_STATUS_OK;
//}
//
//
//void Error_Handler(void)
//{
//    __disable_irq();
//    while (1)
//    {
//    }
//}
//
//
//static app_status_t app_bme680_sensor_stack_init_(void)
//{
//    /* no retries, no MAX_ATTEMPTS, no goto */
//    int8_t rslt = init_bme680();
//    if (rslt != 0) {
//        log_printf("[ERROR] app_bme680_sensor_stack_init: init_bme680 failed rc=%d\r\n", (int)rslt);
//        return APP_STATUS_INIT_FAIL;
//    }
//    return APP_STATUS_OK;
//}
//
//
//static app_status_t app_max31865_peripherals_init(void)
//{
//    if (gpio_pb6_cs_init() != 0) {
//        MAX31865_LOG_PRINTF("[ERROR] app_max31865_peripherals_init: gpio_pb6_cs_init failed\r\n");
//        app_fault_delay_ms(20U);
//        return APP_STATUS_INIT_FAIL;
//    }
//
//    if (spi4_dma_init() != SPI_DMA_STATUS_OK) {
//        MAX31865_LOG_PRINTF("[ERROR] app_max31865_peripherals_init: spi4_dma_init failed\r\n");
//        app_fault_delay_ms(20U);
//        return APP_STATUS_INIT_FAIL;
//    }
//
//    return APP_STATUS_OK;
//}
//
//static app_status_t app_max31865_sensor_stack_init(void)
//{
//    memset(&max31865_dev, 0, sizeof(max31865_dev));
//
//    if (Max31865_init(&max31865_dev, 3U, 50U) != MAX31865_STATUS_OK) {
//        MAX31865_LOG_PRINTF("[ERROR] app_max31865_sensor_stack_init: Max31865_init failed\r\n");
//        app_fault_delay_ms(20U);
//        return APP_STATUS_INIT_FAIL;
//    }
//
//    return APP_STATUS_OK;
//}

/* Halts execution after an unrecoverable application error. */
void Error_Handler(void)
{
    /* Disable interrupts before entering the terminal wait. */
    __disable_irq();

    while (1)
    {
    }
}
