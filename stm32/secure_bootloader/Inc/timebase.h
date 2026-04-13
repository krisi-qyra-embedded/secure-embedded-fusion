/* Author: i fiksuari pas prog | Documentation date: 13 April 2026 */
#ifndef __TIMEBASE_H__
#define __TIMEBASE_H__

#include <stdint.h>
uint32_t get_tick(void);
void delay(uint32_t delay);
void timebase_init(void);

#endif
