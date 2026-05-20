/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
/**
 * @file adc.h
 * @brief ADC initialization, conversion and read declarations.
 */
#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>

/** @brief Initialize ADC input on PA1. */
void pa1_adc_init(void);

/** @brief Start an ADC conversion. */
void start_conversion(void);

/**
 * @brief Read the latest ADC conversion result.
 * @return ADC conversion value.
 */
uint32_t adc_read(void);

#endif
