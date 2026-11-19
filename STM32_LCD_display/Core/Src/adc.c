#include "adc.h"

ADC_HandleTypeDef hadc1;

HAL_StatusTypeDef ADCI_Init(void)
{
    ADC_ChannelConfTypeDef channel = {0};
    HAL_StatusTypeDef status;

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    status = HAL_ADC_Init(&hadc1);
    if (status != HAL_OK) {
        return status;
    }

    channel.Channel = ADC_CHANNEL_4;
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    return HAL_ADC_ConfigChannel(&hadc1, &channel);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        GPIO_InitTypeDef gpio = {0};

        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_4;
        gpio.Mode = GPIO_MODE_ANALOG;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
        __HAL_RCC_ADC1_CLK_DISABLE();
    }
}
