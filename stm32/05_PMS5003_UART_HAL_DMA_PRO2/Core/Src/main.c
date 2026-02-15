/* main.c | Author: i fiksuari pas prog | Date: 15 February 2026 */
#include "stm32f4xx_hal.h"
#include "clock.h"
#include "uart.h"
#include "uart2.h"
#include "pm2_5.h"
#include "pwr.h"
#include "tim.h"
#include "iwdg.h"
#include "systick.h"


static PMS25_App_t g_pm25_app;

static void Error_Handler(void);
static void PM25_HandleRuntimeFatal(PMS25_App_t *app);

/* Initialize the peripherals and run the PMS5003 acquisition loop. */
int main(void)
{
    /* Initialize the UART path before reading sensor frames. */
    UART_Status_t uart_st;
    tim_status_t tim_st;

    HAL_Init();
    clock_100_mhz_init();
    sleep_mode_configure_once();
    uart2_tx_rx_init();

    if (systick_init(PM25_SYSTICK_CORE_HZ, PM25_SYSTICK_DIV) != SYSTICK_OK)
    {
        NVIC_SystemReset();
    }

    if (systick_selftest(200U) != SYSTICK_OK)
    {
        NVIC_SystemReset();
    }

    tim_st = MX_TIM2_Init();
    if (tim_st != TIM_STATUS_OK)
    {
        log_printf("[FATAL][PM25] MX_TIM2_Init failed\r\n");
        Error_Handler();
    }

    check_reset_source();

    {
        const iwdg_status_t iwdg_st = iwdg_init(IWDG_PRESCALER_64,
                                                PM25_IWDG_RELOAD_VALUE,
                                                0U);
        if (iwdg_st != IWDG_OK)
        {
            log_printf("[FATAL][PM25] iwdg_init failed, st=%d\r\n", (int)iwdg_st);
            delay_ms(PM25_LOG_FLUSH_DELAY_MS);
            NVIC_SystemReset();
        }
    }

    uart_st = UART1_PM25_Init();
    if (uart_st != UART_STATUS_OK)
    {
        log_printf("[FATAL][PM25] UART1_PM25_Init failed, st=%u\r\n",
                   (unsigned)uart_st);
        on_fatal_init_error("UART1_PM25_Init failed");
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
    }

    for (;;)
    {

        const PMS25_AppRuntimeStatus_t app_st = PMS25_AppRun(&g_pm25_app);

        if (app_st == PMS25_APP_RUNTIME_FATAL)
        {
            log_printf("[FATAL][PM25] All recovery attempts exhausted, resetting system now\r\n");
            delay_ms(100U);
            escalate_to_iwdg_reset("pm25 runtime failure after 3 total recovery attempts");
        }
        else if (app_st == PMS25_APP_RUNTIME_BAD_PARAM)
        {
            log_printf("[FATAL][PM25] PMS25_AppRun returned BAD_PARAM\r\n");
            delay_ms(100U);
            escalate_to_iwdg_reset("pm25 runtime bad parameter");
        }

        supervisor_check_and_kick_pm2_5();

    }
}


/* Handle interrupt events in Error_Handler. */
static void Error_Handler(void)
{
    /* Clear the handled event and update the transfer state. */
    __disable_irq();
    while (1)
    {
    }
}
