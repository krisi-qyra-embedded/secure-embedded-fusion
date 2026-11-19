/**
 * @file pm25_uart.c
 * @brief USART1 DMA receive path for the PM2.5 sensor.
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
SemaphoreHandle_t pm25_uart_rx_sem = NULL;
static volatile uint32_t g_uart1_last_rx_activity_tick = 0U;
volatile UART1_RxDmaEvent_t g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
static void UART1_PM25_ResetHandles(void);
static inline void uart1_pm25_give_rx_sem_from_isr(void);
static inline void uart1_pm25_drain_rx_sem(void);
/* Clear UART/DMA handles and RX event state. */
static void UART1_PM25_ResetHandles(void)
{
    memset(&huart1, 0, sizeof(huart1));
    memset(&hdma_usart1_rx, 0, sizeof(hdma_usart1_rx));
    /* Start each transfer with a clean completion state. */
    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
}
/**
 * @brief Create the PM2.5 RX DMA completion semaphore.
 * @return 1 on success, otherwise 0.
 */
int UART1_PM25_RtosSyncInit(void)
{
    if (pm25_uart_rx_sem == NULL) {
        pm25_uart_rx_sem = xSemaphoreCreateBinary();
    }
    if (pm25_uart_rx_sem == NULL) {
        return 0;
    }
    /* Keep repeated sync init deterministic. */
    (void)xSemaphoreTake(pm25_uart_rx_sem, 0U);
    return 1;
}
/* Wake the PM2.5 task from the RX DMA IRQ path. */
static inline void uart1_pm25_give_rx_sem_from_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (pm25_uart_rx_sem != NULL) {
        (void)xSemaphoreGiveFromISR(pm25_uart_rx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/* Clear any stale RX semaphore token. */
static inline void uart1_pm25_drain_rx_sem(void)
{
    if ((pm25_uart_rx_sem != NULL) &&
        (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)) {
        (void)xSemaphoreTake(pm25_uart_rx_sem, 0U);
    }
}
/**
 * @brief Initialize USART1 RX and DMA2 Stream5 for PM2.5.
 * @return UART status.
 */
UART_Status_t UART1_PM25_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* Enable GPIO, USART1 and DMA2 clocks. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    /* PM2.5 sensor TX is connected to USART1 RX on PA10. */
    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* PMS5003 uses 9600 baud, RX only. */
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
    /* USART1 RX uses DMA2 Stream5 Channel4. */
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
    /* Priority 5 is used because the IRQ gives a FreeRTOS semaphore. */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5U, 0U);
    NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
    /* Start the RX activity timer from initialization. */
    g_uart1_last_rx_activity_tick = HAL_GetTick();
    return UART_STATUS_OK;
}
/**
 * @brief Recover and reinitialize the PM2.5 UART/DMA path.
 * @param app PM2.5 runtime context.
 * @return UART status.
 */
UART_Status_t UART1_PM25_Recover(PMS25_App_t *app)
{
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
    /* Preserve runtime settings across UART reinitialization. */
    frame_poll_timeout_ms = app->FramePollTimeoutMs;
    no_frame_fault_ms     = app->NoFrameFaultMs;
    sample_period_ms      = app->SamplePeriodMs;
    recovery_delay_ms     = app->RecoveryDelayMs;
    max_recovery_attempts = app->MaxRecoveryAttempts;
    recovery_attempt      = app->RecoveryAttempt;
    /* Stop IRQ and DMA activity before tearing the peripheral down. */
    HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);
    NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    /* Stop any receive still owned by HAL. */
    (void)HAL_UART_DMAStop(&huart1);
    if (huart1.hdmarx != NULL)
    {
        (void)HAL_DMA_Abort(huart1.hdmarx);
        (void)HAL_DMA_DeInit(huart1.hdmarx);
    }
    (void)HAL_UART_DeInit(&huart1);
    UART1_PM25_ResetHandles();
    /* Rebuild UART and DMA from a clean handle state. */
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
    /* Keep the caller recovery budget after AppInit resets the context. */
    app->RecoveryAttempt = recovery_attempt;
    uart1_pm25_drain_rx_sem();
    return UART_STATUS_OK;
}
/**
 * @brief Receive bytes from USART1 using DMA.
 * @param p_data Destination buffer.
 * @param length Number of bytes to receive.
 * @param timeout_ms Transfer timeout.
 * @return UART status.
 */
UART_Status_t UART1_RX_DMA_Receive(uint8_t *p_data, uint16_t length, uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_st;
    uint32_t start_tick;
    if ((p_data == NULL) || (length == 0U) || (timeout_ms == 0U))
    {
        return UART_STATUS_INVALID_PARAM;
    }
    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
    uart1_pm25_drain_rx_sem();
    /* Arm one bounded DMA receive. */
    hal_st = HAL_UART_Receive_DMA(&huart1, p_data, length);
    if (hal_st == HAL_BUSY)
    {
        return UART_STATUS_BUSY;
    }
    if (hal_st != HAL_OK)
    {
        return UART_STATUS_DMA_ERROR;
    }
    /* Runtime path blocks until the DMA callback gives the semaphore. */
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
    /* Before the scheduler starts, wait by polling the DMA event. */
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
/* USART1 RX DMA interrupt. */
void DMA2_Stream5_IRQHandler(void)
{
    if (huart1.hdmarx != NULL)
    {
        /* HAL finishes the DMA state machine and runs the UART callback. */
        HAL_DMA_IRQHandler(huart1.hdmarx);
    }
}
/**
 * @brief Get the last successful USART1 RX activity time.
 * @return HAL tick of the last completed RX.
 */
uint32_t UART1_PM25_GetLastRxActivityTick(void)
{
    return g_uart1_last_rx_activity_tick;
}
/* Record successful PM2.5 DMA receive completion. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        /* Only a completed RX updates the activity timestamp. */
        g_uart1_last_rx_activity_tick = HAL_GetTick();
        g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_DONE;
        uart1_pm25_give_rx_sem_from_isr();
    }
}
/* Record PM2.5 UART/DMA receive error. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        /* Wake the waiter so it can leave with a DMA error. */
        g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_ERROR;
        uart1_pm25_give_rx_sem_from_isr();
    }
}
