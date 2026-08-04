/* Author: i fiksuari pas prog
 * Documentation date: 2026-08-18
 */

#include "bsp.h"
#include "stm32f4xx.h"


#define GPIOAEN		(1U<<0)
#define GPIOCEN		(1U<<2)

#define PIN5		(1U<<5)
#define LED_PIN		PIN5
#define PIN13		(1U<<13)
#define BTN_PIN		PIN13

/* Configures the board LED on PA5. */
void led_init(void)
{
	/* Enable the GPIO clock before selecting output mode. */
	RCC->AHB1ENR |= GPIOAEN;

	GPIOA->MODER |=(1U<<10);
	GPIOA->MODER &=~(1U<<11);
}

/* Turns on the PA5 board LED. */
void led_on(void)
{
	/* Set the LED output bit without changing the other GPIO outputs. */
	GPIOA->ODR |=LED_PIN;
}

/* Turns off the PA5 board LED. */
void led_off(void)
{
	/* Clear only the LED output bit. */
	GPIOA->ODR &=~LED_PIN;

}

/* Configures PC13 for the board button. */
void button_init(void)
{
	/* Keep the button pin in input mode. */
	RCC->AHB1ENR |=GPIOCEN;

	GPIOC->MODER &=~(1U<<26);
	GPIOC->MODER &=~(1U<<27);

}

/* Reads the active-low board button. */
bool get_btn_state(void)
{
	/* A low PC13 input indicates that the button is pressed. */

	if(GPIOC->IDR & BTN_PIN)
	{
		return false;
	}
	else
	{
		return true;
	}

}
