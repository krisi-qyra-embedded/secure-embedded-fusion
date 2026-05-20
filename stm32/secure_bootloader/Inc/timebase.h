/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file timebase.h
 * @brief SysTick timebase and blocking delay declarations.
 */
#ifndef __TIMEBASE_H__
#define __TIMEBASE_H__

#include <stdint.h>

/**
 * @brief Read the current system tick count.
 * @return Current tick value.
 */
uint32_t get_tick(void);

/**
 * @brief Block for the requested number of system ticks.
 * @param delay Delay duration in ticks.
 */
void delay(uint32_t delay);

/** @brief Initialize the SysTick timebase. */
void timebase_init(void);

/**
 * @brief Provide an approximate millisecond busy-wait delay.
 * @param ms Delay duration in milliseconds.
 */
void delay_ms(uint32_t ms);

#define MAX_DELAY		 0xFFFFFFFF

#endif
