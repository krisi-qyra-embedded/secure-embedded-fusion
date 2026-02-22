/* main.c | Author: i fiksuari pas prog | Date: 22 February 2026 */
#include "clock.h"
#include "tim.h"
#include "ds18b20.h"
#include "uart.h"
#include "pwr.h"
#include "iwdg.h"
#include "systick.h"

static DS18B20_App_t g_ds18b20_app;

/* Routes a character to the configured debug output. */
int __io_putchar(int ch)
{
    /* Handle the operation and its status locally. */
    ITM_SendChar(ch);
    return ch;
}

/* Initializes the DS18B20 application and runs the sensor acquisition loop. */
int main(void)
{
    tim_status_t tim_st;
    DS18B20_Status_t ds_st;
    DS18B20_AppRuntimeStatus_t app_st;

    HAL_Init();
    clock_100_mhz_init();   /* with your PLL values this is actually 96 MHz */
    sleep_mode_configure_once();
    uart2_tx_rx_init();

    if (systick_init(96000000U, 6U) != SYSTICK_OK) {
        NVIC_SystemReset();
    }

    if (systick_selftest(200U) != SYSTICK_OK) {
        NVIC_SystemReset();
    }

    /* TIM3 is now only a blocking 1 MHz microsecond delay source for OneWire. */
    tim_st = MX_TIM3_Init();
    if (tim_st != TIM_STATUS_OK)
    {
        log_printf("[ERR] MX_TIM3_Init failed\r\n");
        delay_ms(10U);
        Error_Handler();
    }

    check_reset_source();

    {
        const iwdg_status_t iwdg_status = iwdg_init(IWDG_PRESCALER_64, 2499U, 0U);
        if (iwdg_status != IWDG_OK) {
            log_printf("iwdg_init() failed: %d\r\n", (int)iwdg_status);
            delay_ms(2U);
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

    while (1)
    {
        app_st = DS18B20_AppRun(&g_ds18b20_app);

        if ((app_st == DS18B20_APP_RUNTIME_FATAL) ||
            (app_st == DS18B20_APP_RUNTIME_RECOVERY_FAIL))
        {
            log_printf("[FATAL] One or more DS18B20 sensors failed after 3 recovery attempts -> escalating\r\n");
            delay_ms(2U);
            escalate_to_iwdg_reset("DS18B20 runtime failure after 3 recovery attempts");
            //NVIC_SystemReset();
        }

        supervisor_check_and_kick_ds18b20();
    }
}

/* Stops normal execution after an unrecoverable error. */
void Error_Handler(void)
{
    /* Keep execution in the error path after a fatal failure. */
    __disable_irq();
    while (1)
    {
    }
}
