/* main.c | Author: i fiksuari pas prog | Date: 11 February 2026 */
#include "uart.h"
#include "clock.h"

extern UART_HandleTypeDef huart1; // UART handle
extern DMA_HandleTypeDef hdma_usart1_rx;
extern volatile uint8_t uart_dma_flag;

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;
uint16_t pm1_0_standard,pm2_5_standard,pm10_standard;
uint16_t pm1_0_enviromental,pm2_5_enviromental,pm10_enviromental;
bool readPMSdata(DMA_HandleTypeDef *huart_dma,UART_HandleTypeDef *huart);

/* Initialize the peripherals and run the PMS5003 acquisition loop. */
int main(void)
{
	/* Initialize the UART path before reading sensor frames. */
	HAL_Init();
	UART2_Init();
while(1){
	if(readPMSdata(&hdma_usart1_rx,&huart1))
	{
		pm1_0_standard = data.pm10_standard;
		pm2_5_standard = data.pm25_standard;
		pm10_standard = data.pm100_standard;
		pm1_0_enviromental = data.pm10_env;
		pm2_5_enviromental = data.pm25_env;
		pm10_enviromental = data.pm100_env;
	}
	HAL_Delay(1000);
}
}


/* Read a PMS5003 frame and extract particle measurements. */
bool readPMSdata(DMA_HandleTypeDef *huart_dma, UART_HandleTypeDef *huart)
{
    /* Check the sensor header and frame data before accepting a sample. */
    uint8_t  buffer[32];
    uint16_t bytesRead = 0;
    uint16_t sum = 0;
    uint16_t buffer_u16[15];

    (void)huart_dma;   /* Unused for now */

    uart_dma_flag = 0;

    /* Read until start byte 0x42 is found */
    while (1)
    {
        if (HAL_UART_Receive_DMA(huart, &buffer[0], 1) != HAL_OK)
        {
            return false;
        }

        while (uart_dma_flag == RESET)
        {
            /* Optional: sleep / low-power wait here */
        }

        if (uart_dma_flag == SET)
        {
            uart_dma_flag = 0;

            if (buffer[0] == 0x42)
            {
                break;
            }
        }
    }


        uart_dma_flag = 0;

        if (HAL_UART_Receive_DMA(huart, &buffer[1], 31) != HAL_OK)
        {
            return false;
        }

        while (uart_dma_flag == RESET)
        {
            /* Optional: sleep / low-power wait here */
        }

        if (uart_dma_flag == SET)
        {
            uart_dma_flag = 0;
        }


    /* Verify second header byte */
       if (buffer[1] != 0x4DU)
       {
           return false;
       }


    /* Calculate checksum over first 30 bytes */
    for (uint8_t i = 0; i < 30; i++)
    {
        sum += buffer[i];
    }

    /* Convert payload bytes into uint16_t values */
    for (uint8_t i = 0; i < 15; i++)
    {
        buffer_u16[i]  = ((uint16_t)buffer[2 + i * 2] << 8);
        buffer_u16[i] |=  (uint16_t)buffer[2 + i * 2 + 1];
    }

    /* Copy into data structure */
    memcpy((void *)&data, (void *)buffer_u16, 30);

    /* Verify checksum */
    if (sum != data.checksum)
    {
        return false;
    }

    return true;
}



