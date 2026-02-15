/* uart.c | Author: i fiksuari pas prog | Date: 15 February 2026 */

#include "uart.h"
#include "uart2.h"
#include "pwr.h"
#include "tim.h"

#include <string.h>

UART_HandleTypeDef huart1;
static DMA_HandleTypeDef hdma_usart1_rx;
static volatile uint32_t g_uart1_last_rx_activity_tick = 0U;

typedef enum
{
    UART1_RX_DMA_EVENT_IDLE = 0U,
    UART1_RX_DMA_EVENT_DONE,
    UART1_RX_DMA_EVENT_ERROR
} UART1_RxDmaEvent_t;

static volatile UART1_RxDmaEvent_t g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;

static void UART1_PM25_ResetHandles(void);

/* Reset UART1_PM25_ResetHandles to a known operating state. */
static void UART1_PM25_ResetHandles(void)
{
    /* Clear stale status before restarting the operation. */
    memset(&huart1, 0, sizeof(huart1));
    memset(&hdma_usart1_rx, 0, sizeof(hdma_usart1_rx));
    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
}

/* Configure USART1 and DMA for PMS5003 receiving. */
UART_Status_t UART1_PM25_Init(void)
{
    /* Set the UART and DMA resources needed by the sensor. */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

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

    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0U, 0U);
    NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;
    g_uart1_last_rx_activity_tick = HAL_GetTick();

    return UART_STATUS_OK;
}

/* Recover the PMS5003 UART and DMA receive path. */
UART_Status_t UART1_PM25_Recover(PMS25_App_t *app)
{
    /* Reset the transfer state before receiving another frame. */
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

    return UART_STATUS_OK;
}

/* Receive bytes through the USART1 DMA channel. */
UART_Status_t UART1_RX_DMA_Receive(uint8_t *p_data, uint16_t length, uint32_t timeout_ms)
{
    /* Wait for receive completion and check the error status. */
    HAL_StatusTypeDef hal_st;
    uint32_t start_tick;

    if ((p_data == NULL) || (length == 0U) || (timeout_ms == 0U))
    {
        return UART_STATUS_INVALID_PARAM;
    }

    g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_IDLE;

    hal_st = HAL_UART_Receive_DMA(&huart1, p_data, length);
    if (hal_st == HAL_BUSY)
    {
        return UART_STATUS_BUSY;
    }
    if (hal_st != HAL_OK)
    {
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

/* Handle interrupt events in DMA2_Stream5_IRQHandler. */
void DMA2_Stream5_IRQHandler(void)
{
    /* Clear the handled event and update the transfer state. */
    if (huart1.hdmarx != NULL)
    {
        HAL_DMA_IRQHandler(huart1.hdmarx);
    }
}

/* Read data using UART1_PM25_GetLastRxActivityTick. */
uint32_t UART1_PM25_GetLastRxActivityTick(void)
{
    /* Check the result before using the returned value. */
    return g_uart1_last_rx_activity_tick;
}

/* Handle completion or failure events in HAL_UART_RxCpltCallback. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Record the transfer result for the waiting code. */
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        g_uart1_last_rx_activity_tick = HAL_GetTick();
        g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_DONE;
    }
}

/* Handle completion or failure events in HAL_UART_ErrorCallback. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* Record the transfer result for the waiting code. */
    if ((huart != NULL) && (huart->Instance == USART1))
    {
        g_uart1_rx_dma_event = UART1_RX_DMA_EVENT_ERROR;
    }
}
