/* Author: i fiksuari pas prog | Documentation date: 13 April 2026 */
#include <stdio.h>
#include "stm32f4xx.h"
#include "fpu.h"
#include "uart.h"
#include "timebase.h"
#include "bsp.h"
#include "adc.h"
#include "multiplicative_cipher.h"


#define ENCRYPT_TEXT_LEN 15   // +1 for the null-terminator



char encryptedtext[ENCRYPT_TEXT_LEN];
char decryptedtext[ENCRYPT_TEXT_LEN];

char plaintext[] = "EMBEDDED";
int key =  3;

/* Initializes the platform and runs this cryptographic test sequence. */
int main()
{
	/*Enable FPU*/
	fpu_enable();

	/*Initialize debug UART*/
	debug_uart_init();

	/*Initialize timebase*/
	timebase_init();

	/*Initialize LED*/
	led_init();

	/*Initialize Push button*/
	button_init();

	/*Initialize ADC*/
	pa1_adc_init();

	/*Start conversion*/
	start_conversion();


	encryt_multiplicative_cipher(plaintext,key,encryptedtext);
	decryt_multiplicative_cipher(encryptedtext,key,decryptedtext);

	while(1)
	{



	}
}

