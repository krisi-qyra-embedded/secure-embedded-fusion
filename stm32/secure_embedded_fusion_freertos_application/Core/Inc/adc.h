/* Author: i fiksuari pas prog
 * Documentation date: 2026-08-18
 */

#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>

void pa1_adc_init(void);
void start_conversion(void);
uint32_t adc_read(void);

#endif

