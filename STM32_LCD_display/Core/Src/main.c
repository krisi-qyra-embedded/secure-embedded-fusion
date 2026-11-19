/**
 * @file main.c
 * @brief STM32F411 96 MHz + existing I2C/DMA BME680 + ST7789/LVGL dashboard.
 *
 * Uses your original sensor, watchdog, and timebase modules. This is NOT
 * a standalone port of your much larger project. See README.txt.
 */
#include "stm32f4xx_hal.h"
#include "clock.h"
#include "lvgl.h"
#include "LCDController.h"
#include "bme680_ui.h"

#include "systick.h"
#include "tim.h"
#include "uart.h"
#include "dma.h"
#include "i2c_dma.h"
#include "bme680.h"
#include "sig_sensors.h"
#include "iwdg.h"
#include "pwr.h"

#include <stdint.h>
#include <stdio.h>

#if LVGL_VERSION_MAJOR != 8 || LVGL_VERSION_MINOR != 3
#error "This example requires LVGL 8.3.x."
#endif

#define BME680_DISPLAY_PERIOD_MS 2000U

/* The same application-level setup status used by your previous main.c. */
typedef enum {
    APP_STATUS_OK = 0,
    APP_STATUS_INIT_FAIL = 1
} app_status_t;

static app_status_t app_bme680_peripherals_init(void);
static app_status_t app_bme680_sensor_stack_init_(void);

/* Define these globals only once in the entire project. */
volatile st_bme680_t bme;
volatile st_battery_t bat;
app_bme680_runtime_status_t rc;

/* Set after lv_init() so the existing systick.c can safely tick LVGL. */
volatile uint8_t g_lvgl_tick_enabled = 0U;

static void error_handler(void)
{
    for (;;) {
        /* Debugger breakpoint here if display/LVGL init fails. */
    }
}

int main(void)
{
    uint32_t last_sample_ms;

    if (HAL_Init() != HAL_OK) {
        error_handler();
    }

    /* Despite its legacy name, this wrapper configures 96 MHz. */
    clock_100_mhz_init();

    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    sleep_mode_configure_once();
    tim2_1s_init();
    uart2_tx_rx_init();

    /* Your CUSTOM SysTick already creates a 1ms tick at 96MHz. */
    if (systick_init(96000000U, 6U) != SYSTICK_OK) {
        NVIC_SystemReset();
    }
    if (systick_selftest(200U) != SYSTICK_OK) {
        NVIC_SystemReset();
    }

    /* Do not call HAL_InitTick() again here: it would reconfigure SysTick. */
    check_reset_source();

    {
        const iwdg_status_t iwdg_status =
            iwdg_init(IWDG_PRESCALER_64, 2499U, 0U);
        if (iwdg_status != IWDG_OK) {
            log_printf("iwdg_init() failed: %d\r\n", (int)iwdg_status);
            tim2_delay_ms(2U);
            NVIC_SystemReset();
        }
    }

    /* Your original I2C1 PB8/PB9 and DMA1 Stream5/6 setup. */
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

    lv_init();
    g_lvgl_tick_enabled = 1U;

    /* Keep your already-working LCDController.c and ST7789 SPI driver. */
    if (lv_port_disp_init() != HAL_OK) {
        error_handler();
    }

    if (!bme680_ui_init()) {
        error_handler();
    }

    lv_refr_now(NULL);

    /* First reading now, then at approximately 2-second intervals. */
    last_sample_ms = ms_get() - BME680_DISPLAY_PERIOD_MS;

    for (;;) {
        const uint32_t now = ms_get();

        if ((uint32_t)(now - last_sample_ms) >= BME680_DISPLAY_PERIOD_MS) {
            last_sample_ms = now;

            /* Your existing driver reads sensor over I2C/DMA and updates bme. */
            rc = app_bme680_sensor_runtime_process();

            if (rc == APP_BME680_RUNTIME_OK) {
                /* The runtime can return OK even when a startup sample was
                 * skipped, so keep dashes until compensated data is valid. */
                if ((bme.pressure > 0.0f) &&
                    (bme.humidity >= 0.0f) &&
                    (bme.humidity <= 100.0f) &&
                    (bme.temperature >= -50.0f) &&
                    (bme.temperature <= 100.0f)) {
                    bme680_ui_update((float)bme.temperature,
                                     (float)bme.humidity,
                                     (float)bme.pressure,
                                     (uint32_t)bme.gas_resistance);
                } else {
                    bme680_ui_waiting();
                }
            } else {
                bme680_ui_error();
                lv_refr_now(NULL);
                log_printf("[ERROR] BME680 runtime rc=%d\r\n", (int)rc);
                tim2_delay_ms(2U);
                escalate_to_iwdg_reset("BME680 sensor read unrecoverable");
            }
        }

        /* Preserve your existing supervisor's exclusive watchdog refresh. */
        if (supervisor_due != 0U) {
            supervisor_due = 0U;
            supervisor_check_and_kick();
        }

        /* LVGL keeps running between sensor acquisitions. */
        lv_timer_handler();
        HAL_Delay(5U);
    }
}

/* Original app settings from your working BME680 project. */
void fnAPP_Init_Standard_Values(void)
{
    bme.gas_weighting = 0.75f;
    bme.gas_reference = 2500.0f;
    bme.getgasreference_count = 0U;
    bme.hum_weighting = 0.25f;
    bme.hum_reference = 40.0f;
    bme.gas_lower_limit = 10000;
    bme.gas_upper_limit = 300000;
}

static app_status_t app_bme680_peripherals_init(void)
{
    i2c_select_status_t select_rc = i2c_init_select(1U, 8U, 9U);
    if (select_rc != I2C_SELECT_OK) {
        log_printf("[ERROR] I2C1 setup rc=%d\r\n", (int)select_rc);
        return APP_STATUS_INIT_FAIL;
    }

    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    {
        dma_status_t st;
        st = dma_stream_init(DMA1_Stream5,
                             1U,
                             DMA_DIR_P2M,
                             &DMA1->HIFCR,
                             DMA1_STREAM5_TCIF,
                             DMA1_Stream5_IRQn);
        if (st != DMA_OK) {
            log_printf("[ERROR] DMA RX init rc=%d\r\n", (int)st);
            return APP_STATUS_INIT_FAIL;
        }

        st = dma_stream_init(DMA1_Stream6,
                             1U,
                             DMA_DIR_M2P,
                             &DMA1->HIFCR,
                             DMA1_STREAM6_TCIF,
                             DMA1_Stream6_IRQn);
        if (st != DMA_OK) {
            log_printf("[ERROR] DMA TX init rc=%d\r\n", (int)st);
            return APP_STATUS_INIT_FAIL;
        }
    }

    fnAPP_Init_Standard_Values();
    return APP_STATUS_OK;
}

static app_status_t app_bme680_sensor_stack_init_(void)
{
    const int8_t init_status = init_bme680();
    if (init_status != 0) {
        log_printf("[ERROR] init_bme680 rc=%d\r\n", (int)init_status);
        return APP_STATUS_INIT_FAIL;
    }
    return APP_STATUS_OK;
}

/*
 * IMPORTANT: NO SysTick_Handler() in this main.c.
 * Keep one SysTick_Handler in your existing systick.c and add LVGL's tick
 * there exactly as described in SYSTICK_PATCH.txt.
 */
