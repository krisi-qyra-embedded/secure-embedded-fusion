/**
 * @file bsp.c
 * @brief Basic board support functions for LED and user button.
 */

#include "bsp.h"
#include "stm32f4xx.h"


#define GPIOAEN     (1U << 0)
#define GPIOCEN     (1U << 2)

#define PIN5        (1U << 5)
#define LED_PIN     PIN5
#define PIN13       (1U << 13)
#define BTN_PIN     PIN13


/**
 * @brief Initialize the board LED.
 *
 * Configures PA5 as a GPIO output.
 */
void led_init(void)
{
    /* Enable clock access to GPIOA */
    RCC->AHB1ENR |= GPIOAEN;

    /* Set PA5 mode to output mode */
    GPIOA->MODER |= (1U << 10);
    GPIOA->MODER &= ~(1U << 11);
}


/**
 * @brief Turn the board LED on.
 */
void led_on(void)
{
    /* Set PA5 high */
    GPIOA->ODR |= LED_PIN;
}


/**
 * @brief Turn the board LED off.
 */
void led_off(void)
{
    /* Set PA5 low */
    GPIOA->ODR &= ~LED_PIN;
}


/**
 * @brief Initialize the user button.
 *
 * Configures PC13 as a GPIO input.
 */
void button_init(void)
{
    /* Enable clock access to GPIOC */
    RCC->AHB1ENR |= GPIOCEN;

    /* Set PC13 as an input pin */
    GPIOC->MODER &= ~(1U << 26);
    GPIOC->MODER &= ~(1U << 27);
}


/**
 * @brief Read the user button state.
 *
 * The button input is active low.
 *
 * @return true if the button is pressed.
 * @return false if the button is released.
 */
bool get_btn_state(void)
{
    /* Button is active low */

    /* Check if button is pressed */
    if (GPIOC->IDR & BTN_PIN)
    {
        return false;
    }
    else
    {
        return true;
    }
}
