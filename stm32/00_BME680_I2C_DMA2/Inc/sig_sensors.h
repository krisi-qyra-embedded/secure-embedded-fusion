/* sig_sensors.h | Author: i fiksuari mbas prog | Date: 13 January 2026 */

#ifndef SIG_SENSORS_H_
#define SIG_SENSORS_H_

#include <bme680_defs.h>
#include <bme680.h>
#include <math.h>
#include <stdint.h>

/* ! Structure with variables related to HT-ADC sensor . */
typedef struct{
	uint16_t voltage;
} st_battery_t;

/* ! Structure with variables related to BME680 sensor . */
typedef struct{
	struct bme680_dev gas_sensor;
	float temperature;
	float pressure;
	float humidity;
	uint32_t gas_resistance;
	uint16_t meas_period;
	float gas_weighting;
	float gas_reference;
	uint8_t getgasreference_count;
	float hum_weighting;
	float hum_reference;
	int32_t gas_lower_limit;
	int32_t gas_upper_limit;
} st_bme680_t;

extern volatile st_battery_t bat;
extern volatile st_bme680_t bme;

int8_t init_bme680(void);
int8_t bme_GetGasReference(void);
int16_t bme_GetGasScore(void);
int16_t bme_GetHumidityScore(void);
void bme_calculateIAQ(int16_t score);


#endif /* SIG_SENSORS_H_ */
