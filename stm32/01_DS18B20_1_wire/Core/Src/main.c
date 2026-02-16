/* main.c | Author: i fiksuari pas prog | Date: 17 February 2026 */
#include"clock.h"
#include"tim.h"
#include"ds18b20.h"
#include"onewire.h"

DS18B20_Drv_t DS;
OneWire_t OW;
static void MX_GPIO_Init(void);
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
	clock_100_mhz_init();
	HAL_Init();
	MX_GPIO_Init();
	MX_TIM2_Init();
	OW.DataPin = GPIO_PIN_5;
	OW.DataPort = GPIOA;
	DS.Resolution = DS18B20_Resolution_12bits;
	DS18B20_Init(&DS, &OW);
	DS18B20_SetTempAlarm(&OW, DS.DevAddr[0], 0, 31);
  while (1)
  {
	  /* USER CODE END WHILE */
	  /* USER CODE BEGIN 3 */
	  /* Start temperature conversion on all devices on one bus */
	  DS18B20_StartAll(&OW);
	  /* Read temperature from device and store it to DS data structure */
	  for(uint8_t i = 0; i < OW.RomCnt; i++)
	  	{
	  		/* Read Temperature */
	  		DS18B20_Read(&OW, DS.DevAddr[i], &DS.Temperature[i]);
	  	}
	  /* Search Alarm triggered and store in DS data structure */
	  DS18B20_AlarmSearch(&DS, &OW);
	  /* It's recommanded to do not more than once per second */
	  HAL_Delay(1000);
  }
}


/* Initializes the MX GPIO Init resources. */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock for GPIOA since PA5 is on port A
	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Set PA5 to low initially
	/* Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Configure as push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;  // No internal pull-up or pull-down resistor
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // Set speed to low for general-purpose use
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // Initialize PA5 with these settings
}

/* Stops normal execution after an unrecoverable error. */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



