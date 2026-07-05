/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "pm25_uart.h"
#include "uart.h"
#include "pwr.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

UART_HandleTypeDef huart1;
static DMA_HandleTypeDef hdma_usart1_rx;

/*
 * FreeRTOS binary semaphore used by USART1 RX DMA completion.
 *
 * PM2.5 USART1_RX -> PA10 -> DMA2_Stream5 -> pm25_uart_rx_sem
 */
SemaphoreHandle_t pm25_uart_rx_sem = NULL;

static volatile uint32_t g_uart1_last_rx_activity_tick = 0U;

volatile UART1_RxDmaEvent_t g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;

static void UART1_PM25_ResetHandles(void);
static inline void uart1_pm25_give_rx_sem_from_isr(void);
static inline void uart1_pm25_drain_rx_sem(void);

/* Clears the PM2.5 UART and DMA handle state. */
static void UART1_PM25_ResetHandles(void)
{
    /* Reset the receive event with both HAL handles. */
    memset(&huart1, 0, sizeof(huart1));
    memset(&hdma_usart1_rx, 0, sizeof(hdma_usart1_rx));

    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
}

/* Creates and clears the PM2.5 receive semaphore. */
int UART1_PM25_RtosSyncInit(void)
{
    /* Discard a stale completion token after allocation. */
    if (pm25_uart_rx_sem == NULL) {
        pm25_uart_rx_sem = xSemaphoreCreateBinary();
    }

    if (pm25_uart_rx_sem == NULL) {
        return 0;
    }

    (void)xSemaphoreTake(pm25_uart_rx_sem, 0U);

    return 1;
}

/* Wakes the task waiting for PM2.5 receive completion. */
static inline void uart1_pm25_give_rx_sem_from_isr(void)
{
    /* Yield on ISR exit if the receive semaphore wakes a task. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (pm25_uart_rx_sem != NULL) {
        (void)xSemaphoreGiveFromISR(pm25_uart_rx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Clears a stale PM2.5 receive semaphore token. */
static inline void uart1_pm25_drain_rx_sem(void)
{
    /* Take the token without blocking after scheduler startup. */
    if ((pm25_uart_rx_sem != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
        (void)xSemaphoreTake(pm25_uart_rx_sem, 0U);
    }
}

/* Configures the PM2.5 USART1 receive path and DMA. */
UART_Status_t UART1_PM25_Init(void)
{
    /* Use PA10 at 9600 baud with DMA2 Stream5 channel 4. */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /*
     * PM2.5 sensor TX -> STM32 PA10 / USART1_RX
     */
    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        UART1_PM25_ResetHandles();
        return UART_STATUS_INIT_ERROR;
    }

    hdma_usart1_rx.Instance                 = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
        (void)HAL_UART_DeInit(&huart1);
        UART1_PM25_ResetHandles();
        return UART_STATUS_INIT_ERROR;
    }

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5U, 0U);
    NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
    g_uart1_last_rx_activity_tick = HAL_GetTick();

    return UART_STATUS_OK;
}

/* Restores the PM2.5 UART and application after a receive fault. */
UART_Status_t UART1_PM25_Recover(PMS25_App_t *app)
{
    /* Retain the application's timeout and retry settings across reinitialization. */
    UART_Status_t uart_st;
    uint32_t frame_poll_timeout_ms;
    uint32_t no_frame_fault_ms;
    uint32_t sample_period_ms;
    uint32_t recovery_delay_ms;
    uint8_t  max_recovery_attempts;
    uint8_t  recovery_attempt;

    if (app == NULL)
    {
        return UART_STATUS_INVALID_PARAM;
    }

    frame_poll_timeout_ms = app->FramePollTimeoutMs;
    no_frame_fault_ms     = app->NoFrameFaultMs;
    sample_period_ms      = app->SamplePeriodMs;
    recovery_delay_ms     = app->RecoveryDelayMs;
    max_recovery_attempts = app->MaxRecoveryAttempts;
    recovery_attempt      = app->RecoveryAttempt;

    HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
    NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);

    (void)HAL_UART_DMAStop(&huart1);

    if (huart1.hdmarx != NULL)
    {
        (void)HAL_DMA_Abort(huart1.hdmarx);
        (void)HAL_DMA_DeInit(huart1.hdmarx);
    }

    (void)HAL_UART_DeInit(&huart1);

    UART1_PM25_ResetHandles();

    uart_st = UART1_PM25_Init();
    if (uart_st != UART_STATUS_OK)
    {
        return uart_st;
    }

    if (PMS25_AppInit(app,
                      frame_poll_timeout_ms,
                      no_frame_fault_ms,
                      sample_period_ms,
                      recovery_delay_ms,
                      max_recovery_attempts) != PMS25_APP_RUNTIME_OK)
    {
        return UART_STATUS_ERROR;
    }

    app->RecoveryAttempt = recovery_attempt;

    uart1_pm25_drain_rx_sem();

    return UART_STATUS_OK;
}

/* Receives a PM2.5 UART block within the timeout. */
UART_Status_t UART1_RX_DMA_Receive(uint8_t *p_data, uint16_t length, uint32_t timeout_ms)
{
    /* Stop the DMA receive if completion does not arrive in time. */
    HAL_StatusTypeDef hal_st;
    uint32_t start_tick;

    if ((p_data == NULL) || (length == 0U) || (timeout_ms == 0U))
    {
        return UART_STATUS_INVALID_PARAM;
    }

    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;

    uart1_pm25_drain_rx_sem();

    hal_st = HAL_UART_Receive_DMA(&huart1, p_data, length);
    if (hal_st == HAL_BUSY)
    {
        return UART_STATUS_BUSY;
    }

    if (hal_st != HAL_OK)
    {
        return UART_STATUS_DMA_ERROR;
    }

    if ((pm25_uart_rx_sem != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING))
    {
        if (xSemaphoreTake(pm25_uart_rx_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
        {
            (void)HAL_UART_DMAStop(&huart1);
            g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
            return UART_STATUS_TIMEOUT;
        }

        if (g_uart1_rx_dma_event == UART1_RX_DMA_EVENT_DONE)
        {
            g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
            return UART_STATUS_OK;
        }

        g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
        return UART_STATUS_DMA_ERROR;
    }

    start_tick = HAL_GetTick();

    while (g_uart1_rx_dma_event == UART1_RX_DMA_EVENT_IDLE)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            (void)HAL_UART_DMAStop(&huart1);
            g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
            return UART_STATUS_TIMEOUT;
        }

        cpu_sleep_until_irq();
    }

    if (g_uart1_rx_dma_event == UART1_RX_DMA_EVENT_ERROR)
    {
        g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
        return UART_STATUS_DMA_ERROR;
    }

    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
    return UART_STATUS_OK;
}

/* Dispatches the PM2.5 receive DMA interrupt. */
void DMA2_Stream5_IRQHandler(void)
{
    /* Pass the linked USART1 RX DMA handle to the HAL handler. */
    if (huart1.hdmarx != NULL)
    {
        HAL_DMA_IRQHandler(huart1.hdmarx);
    }
}

/* Returns the last PM2.5 receive-completion timestamp. */
uint32_t UART1_PM25_GetLastRxActivityTick(void)
{
    /* Read the activity tick maintained by the UART callback. */
    return g_uart1_last_rx_activity_tick;
}

/* Publishes completion of a USART1 PM2.5 receive. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Ignore callbacks for other UART instances. */
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        g_uart1_last_rx_activity_tick = HAL_GetTick();
        g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_DONE;

        uart1_pm25_give_rx_sem_from_isr();
    }
}

/* Publishes a USART1 PM2.5 receive error. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Set the receive event to error only for the PM2.5 UART. */
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_ERROR;

        uart1_pm25_give_rx_sem_from_isr();
    }
}
