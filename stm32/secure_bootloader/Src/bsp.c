/**
 * @file bsp.c
 * @brief Nucleo LED and user-button GPIO helpers.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
#include "bsp.h"
#include "stm32f4xx.h"

/* GPIO clocks and board pin assignments. */
#define GPIOAEN		(1U<<0)
#define GPIOCEN		(1U<<2)

#define PIN5		(1U<<5)
#define LED_PIN		PIN5
#define PIN13		(1U<<13)
#define BTN_PIN		PIN13

/**
 * @brief Configure PA5 as the board LED output.
 */
void led_init(void)
{
    /* Initialize the required state before enabling the peripheral. */
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |=(1U<<10);
	GPIOA->MODER &=~(1U<<11);
}

/**
 * @brief Drive the board LED on.
 */
void led_on(void)
{
    /* Check the return status before continuing to the next step. */
	GPIOA->ODR |=LED_PIN;
}

/**
 * @brief Drive the board LED off.
 */
void led_off(void)
{
    /* Check the return status before continuing to the next step. */
	GPIOA->ODR &=~LED_PIN;
}

/**
 * @brief Configure PC13 as the user-button input.
 */
void button_init(void)
{
    /* Initialize the required state before enabling the peripheral. */
	RCC->AHB1ENR |=GPIOCEN;
	GPIOC->MODER &=~(1U<<26);
	GPIOC->MODER &=~(1U<<27);
}

/**
 * @brief Read the active-low user button.
 * @return True when the button is pressed.
 */
bool get_btn_state(void)
{
    /* Check the return status before continuing to the next step. */
	if(GPIOC->IDR & BTN_PIN)
	{
		return false;
	}
	else
	{
		return true;
	}
}
