
#include "max31865.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "dma.h"
#include "spi_dma.h"
#include <string.h>
#include "uart.h"
#include "tim.h"
#include "iwdg.h"
#include <stdint.h>
#include <stdbool.h>
#include "systick.h"
#include "cmox_crypto.h"

typedef enum
{
    APP_STATUS_OK        = 0,
    APP_STATUS_INIT_FAIL = -1,
    APP_STATUS_RUN_FAIL  = -2
} app_status_t;

static app_status_t app_peripherals_init(void);
static app_status_t app_sensor_stack_init(Max31865_t *dev);

float temperatureC = 0.0f;
Max31865_t max31865_dev;

/* Initializes the application and runs the MAX31865 sampling loop. */
int main(void)
{
    /* Stop initialization if a required peripheral fails. */
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

    if (cmox_initialize(NULL) != CMOX_INIT_SUCCESS) {
        NVIC_SystemReset();
    }

    check_reset_source();

    {
        const iwdg_status_t iwdg_status = iwdg_init(IWDG_PRESCALER_128, 2499U, 0U);

        if (iwdg_status != IWDG_OK)
        {
            log_printf("iwdg_init() failed: %d\r\n", (int)iwdg_status);
            tim2_delay_ms(20);
            NVIC_SystemReset();
        }
    }

    if (app_peripherals_init() != APP_STATUS_OK) {
        on_fatal_init_error("app_peripherals_init failed");
    }

    if (app_sensor_stack_init(&max31865_dev) != APP_STATUS_OK) {
        on_fatal_init_error("app_sensor_stack_init failed");
    }

    {
        const uint32_t now = ms_get();
        hb_spi_ts = now;
        hb_main_ts = now;
    }

    while (1) {
        const max31865_status_t read_status = Max31865_readTempC(&max31865_dev, &temperatureC);
        if (read_status != MAX31865_STATUS_OK) {
            log_printf("MAX31865 read error rc=%d\r\n", read_status);
            tim2_delay_ms(20);
            escalate_to_iwdg_reset("Max31865_readTempC unrecoverable");
        }

        {
            const max31865_status_t dbg_status = Max31865_debug_once(&max31865_dev);
            if (dbg_status != MAX31865_STATUS_OK) {
                log_printf("Max31865_debug_once rc=%d\r\n", dbg_status);
                tim2_delay_ms(20);
                escalate_to_iwdg_reset("Max31865_debug failed");
            }
        }

        if (supervisor_due) {
            supervisor_due = 0;
            supervisor_check_and_kick();
        }

        tim2_delay_ms(1000U);
    }
}

/* Initializes the resources used by app peripherals init. */
static app_status_t app_peripherals_init(void)
{
    /* Set up the required peripheral configuration. */
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

/* Initializes the resources used by app sensor stack init. */
static app_status_t app_sensor_stack_init(Max31865_t *dev)
{
    /* Set up the required peripheral configuration. */
    if (dev == NULL) {
        return APP_STATUS_INIT_FAIL;
    }

    memset(dev, 0, sizeof(*dev));

    if (Max31865_init(dev, 3U, 50U) != MAX31865_STATUS_OK) {
        return APP_STATUS_INIT_FAIL;
    }

    return APP_STATUS_OK;
}
