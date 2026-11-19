#ifndef BME680_UI_H_
#define BME680_UI_H_

#include <stdbool.h>
#include <stdint.h>

/* Create the 240x320 portrait BME680 screen. Call once after lv_init() and display init. */
bool bme680_ui_init(void);

/* Show current cached readings; call from main only, never from an ISR. */
void bme680_ui_update(float temperature_c,
                      float humidity_percent,
                      float pressure_hpa,
                      uint32_t gas_resistance_ohms);

/* Retain placeholder values until the first valid sample. */
void bme680_ui_waiting(void);

/* Indicate a sensor read failure. */
void bme680_ui_error(void);

#endif /* BME680_UI_H_ */
