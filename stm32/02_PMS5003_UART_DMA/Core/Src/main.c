/* main.c | Author: i fiksuari pas prog | Date: 04 February 2026 */
#include "uart.h"
#include "clock.h"

volatile uint8_t g_transfer_complete_rx = 0;
volatile uint8_t g_transfer_complete_tx = 0;
// Define a maximum timeout for transfers in milliseconds
#define DMA_TRANSFER_TIMEOUT 100  // Adjust as necessary
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;
void readPMSdata(void);
void Error_Handler(void);


/* Initialize the peripherals and run the PMS5003 acquisition loop. */
int main(void)
{
	/* Initialize the UART path before reading sensor frames. */
	clock_100_mhz_init();
	HAL_Init();
	uart2_tx_rx_init();
	DMA1_Init();

    while(1){
    	readPMSdata();
    	HAL_Delay(1000);
}
}

/* Read a PMS5003 frame and extract particle measurements. */
readPMSdata(void)
{
	// Variable to hold the received byte
	uint8_t byte;
	uint8_t buffer[32];
	uint16_t bytesRead = 0;
	// Initialize sum to zero
	uint16_t sum = 0;
	// Create an array to hold the converted 16-bit values
	uint16_t buffer_u16[15];
	// Read bytes until we find the special '0x42' start-byte
while (1)
{
    // Wait until data is available
	DMA_channel4_Stream5_UART_RX_config(&byte,1);
	while(!g_transfer_complete_rx){}
	g_transfer_complete_rx = 0;
    // Check for the start-byte
    if (byte == 0x42) {
        break; // Found the start-byte, exit the loop
    }
}

// Read the next 32 bytes with timeout check
	DMA_channel4_Stream5_UART_RX_config(buffer,32);
    while (!g_transfer_complete_rx){}
    g_transfer_complete_rx = 0;

// Calculate checksum over the first 30 bytes of the buffer
for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];  // Accumulate the checksum
}

for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
// put it into a nice struct :)
 memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
	  Error_Handler();
  }
}


/* Handle interrupt events in DMA2_Stream5_IRQHandler. */
void DMA2_Stream5_IRQHandler(void)
{
	//check if transfer is complete
		if((DMA2->HISR) & (1U << 11))
		{
			g_transfer_complete_rx = 1;
			//clear transfer complete interrupt flag
			DMA2->HIFCR |= (1U << 11);
		}
		//check if transfer error has occured
		if((DMA2->HISR) & (1U << 9))
			{
				//do something
			    Error_Handler();
				//clear transfer complete interrupt flag
				DMA2->HIFCR |= (1U << 9);
			}
}

/* Handle interrupt events in DMA2_Stream7_IRQHandler. */
void DMA2_Stream7_IRQHandler(void)
{
	//check if transfer is complete
		if((DMA2->HISR) & (1U << 21))
		{
			g_transfer_complete_tx = 1;
			//clear transfer complete interrupt flag
			DMA2->HIFCR |= (1U << 21);
		}
		//check if transfer error has occured
		if((DMA2->HISR) & (1U << 19))
			{
				//do something
			    Error_Handler();
				//clear transfer complete interrupt flag
				DMA2->HIFCR |= (1U << 19);
			}
}

// Error Handler
void Error_Handler(void)
{
    /* Clear the handled event and update the transfer state. */
    while(1)
    {
        // Error loop
    }
}

//void USART2_IRQHandler(void)
//{
//	if ((USART2->SR) & (1U << 6)) // Check if TC flag is set
//	{
//		g_transfer_complete_tx = 1;
//		USART2->SR &= ~(1U << 6);
//	}
//}



