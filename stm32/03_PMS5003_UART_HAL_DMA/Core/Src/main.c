/* main.c | Author: i fiksuari pas prog | Date: 08 February 2026 */
#include "uart.h"
#include "clock.h"

extern UART_HandleTypeDef huart1; // UART handle

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
bool readPMSdata(UART_HandleTypeDef *huart);

/* Initialize the peripherals and run the PMS5003 acquisition loop. */
int main(void)
{
	/* Initialize the UART path before reading sensor frames. */
	HAL_Init();
	UART2_Init();
while(1){
	readPMSdata(&huart1);
		pm1_0_standard = data.pm10_standard;
		pm2_5_standard = data.pm25_standard;
		pm10_standard = data.pm100_standard;
		pm1_0_enviromental = data.pm10_env;
		pm2_5_enviromental = data.pm25_env;
		pm10_enviromental = data.pm100_env;

	HAL_Delay(1000);
}
}

/* Read a PMS5003 frame and extract particle measurements. */
bool readPMSdata(UART_HandleTypeDef *huart)
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
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_RXNE) == RESET) {
		    return false; // No data available
		}
    // Wait until data is available
	if (HAL_UART_Receive_DMA(huart, &byte, 1) != HAL_OK) {
	        return false; // Return false on failure
	}
    // Check for the start-byte
    if (byte == 0x42) {
        break; // Found the start-byte, exit the loop
    }
}

// Check if at least 32 bytes are available
    // Use HAL_UART_Receive to read bytes into the buffer
    if (HAL_UART_Receive_DMA(huart,buffer, 32) != HAL_OK) {
        return false; // Return false on failure
    }

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
    return false;
  }
  return true;
}




