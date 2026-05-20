/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file bsp.h
 * @brief Nucleo LED and user-button GPIO declarations.
 */
#ifndef __BSP_H__
#define __BSP_H__

#include <stdint.h>
#include <stdbool.h>

/** @brief Initialize the board LED GPIO. */
void led_init(void);

/** @brief Turn the board LED on. */
void led_on(void);

/** @brief Turn the board LED off. */
void led_off(void);

/** @brief Initialize the user-button GPIO. */
void button_init(void);

/**
 * @brief Read the current user-button state.
 * @return Current button state.
 */
bool get_btn_state(void);

#endif
