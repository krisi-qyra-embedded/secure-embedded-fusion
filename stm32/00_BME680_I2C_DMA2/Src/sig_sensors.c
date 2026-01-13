/* sig_sensors.c | Author: i fiksuari mbas prog | Date: 13 January 2026 */
#include "sig_sensors.h"
#include "pwr.h"

/* init_bme680 - initialize BME680 device Returns: 0 -> success <0 -> failure (propagated error code from. */
int8_t init_bme680(void)
{
    /* Configure only the resources required by this module. */
    const uint8_t MAX_ATTEMPTS = 3U;
    const uint32_t BACKOFF_MS_BASE = 10U; 

    int8_t rslt = -1;
    uint8_t set_required_settings;

    
    struct bme680_dev *dev = &bme.gas_sensor;

    
    dev->dev_id    = (uint8_t) to7bit((uint32_t) BME680_I2C_ADDR_PRIMARY);
    dev->intf      = BME680_I2C_INTF;
    dev->handle    = (void *) I2C1;          
    dev->read      = user_i2c_read;
    dev->write     = user_i2c_write;
    dev->delay_ms  = user_delay_ms;
    dev->amb_temp  = 25;                     

    /* bme680_init: retry transient failures */
    /* bme680_init: single attempt, return immediately on error */
    /* single attempt, keep short backoff before returning error */
    rslt = bme680_init(dev);
    if (rslt != 0) {
        log_printf("[ERROR] bme680_init failed (err=%d)\r\n", (int)rslt);
        dev->delay_ms(BACKOFF_MS_BASE); 
        return rslt;
    }

    /* Configure sensor settings (OS, filter, gas) */
    dev->tph_sett.os_hum  = BME680_OS_2X;
    dev->tph_sett.os_pres = BME680_OS_4X;
    dev->tph_sett.os_temp = BME680_OS_8X;
    dev->tph_sett.filter  = BME680_FILTER_SIZE_3;

    dev->gas_sett.run_gas   = BME680_ENABLE_GAS_MEAS;
    dev->gas_sett.heatr_temp = 320; 
    dev->gas_sett.heatr_dur  = 150; 

    dev->power_mode = BME680_FORCED_MODE;

    set_required_settings = (uint8_t)(BME680_OST_SEL | BME680_OSP_SEL |
                                      BME680_OSH_SEL | BME680_FILTER_SEL |
                                      BME680_GAS_SENSOR_SEL);

    /* bme680_set_sensor_settings: single attempt, delay then return on error */
    rslt = bme680_set_sensor_settings(set_required_settings, dev);
    if (rslt != 0) {
        
        log_printf("[ERROR] bme680_set_sensor_settings failed (err=%d)\r\n", (int)rslt);
        dev->delay_ms(BACKOFF_MS_BASE);
        return rslt ? rslt : -2;
    }

    /* Set sensor mode (power mode) */
    /* single attempt, keep short backoff before returning error */
    rslt = bme680_set_sensor_mode(dev);
    if (rslt != 0) {
        log_printf("[ERROR] bme680_set_sensor_mode failed (err=%d)\r\n", (int)rslt);
        dev->delay_ms(BACKOFF_MS_BASE); 
        return rslt ? rslt : -3;
    }

    
    /* single attempt, keep short backoff before returning error */
    rslt = bme680_get_profile_dur(&bme.meas_period, dev);
    if (rslt != 0) {
        log_printf("[ERROR] bme680_get_profile_dur failed (err=%d)\r\n", (int)rslt);
        dev->delay_ms(BACKOFF_MS_BASE); 
        return rslt ? rslt : -4;
    }

    
    /* single attempt, keep short backoff before returning error */
    rslt = bme_GetGasReference();
    if (rslt != 0) {
        log_printf("[ERROR] bme_GetGasReference failed (err=%d)\r\n", (int)rslt);
        dev->delay_ms(BACKOFF_MS_BASE); 
        return rslt ? rslt : -5;
    }

    /* log_printf("[INFO] init_bme680: initialization successful (I2C 7-bit addr 0x%02X)\r\n", dev->dev_id); */
    return 0;
}

/* bme_GetGasReference - sample gas resistance and store an averaged reference Returns: 0 -> success. */
int8_t bme_GetGasReference(void)
{
    /* Check the current status before continuing with the operation. */
    
    const uint8_t TARGET_VALID_READS = 10;        
    const uint8_t MAX_ATTEMPTS_TOTAL = 40;        /* absolute cap on attempts to avoid infinite loops */
    const uint8_t MAX_READ_RETRIES   = 3;         /* retry sensor read a few times on transient fail */

    int8_t rslt;
    struct bme680_field_data data;

    /* Safety: null pointer / callbacks check (reuse your SDK helper if available) */
    rslt = null_ptr_check(&bme.gas_sensor);
    if (rslt != BME680_OK) {
        log_printf("[ERROR] bme_GetGasReference: null_ptr_check failed (err=%d)\r\n", (int)rslt);
        return -1;
    }

    
    uint64_t acc = 0ULL;
    uint8_t valid_count = 0;
    uint16_t attempts = 0;

    
    bme.gas_reference = 0;

    while ((valid_count < TARGET_VALID_READS) && (attempts < MAX_ATTEMPTS_TOTAL)) {
        attempts++;

        /* attempt the read with a few quick retries in case of transient I/O glitches */
        for (uint8_t retry = 0; retry < MAX_READ_RETRIES; ++retry) {
            rslt = bme680_get_sensor_data(&data, &bme.gas_sensor);
            if (rslt == BME680_OK) break;
            log_printf("[WARN] bme_GetGasReference: bme680_get_sensor_data transient err=%d (retry %u)\r\n",
                       (int)rslt, (unsigned)(retry + 1));
            
            bme.gas_sensor.delay_ms(2U + retry * 5U);
        }
        if (rslt != BME680_OK) {
            /* If the underlying sensor read failed after retries, bail out as permanent failure */
            log_printf("[ERROR] bme_GetGasReference: bme680_get_sensor_data failed permanently (err=%d)\r\n", (int)rslt);
            return -3;
        }

        /* Check if the gas measurement in this sample is valid */
        if (data.status & BME680_GASM_VALID_MSK) {
            acc += (uint64_t)data.gas_resistance;
            valid_count++;
        } else {
            
            /* log_printf("[DEBUG] bme_GetGasReference: sample %u invalid (status=0x%02X)\r\n", (unsigned)attempts. */
        }

        /* Wait sensor profile duration (use configured delay callback); ensure non-zero period */
        uint32_t delay_ms = (bme.meas_period > 0U) ? bme.meas_period : 100U;
        bme.gas_sensor.delay_ms(delay_ms);
    }

    if (valid_count == 0) {
        log_printf("[ERROR] bme_GetGasReference: no valid gas readings after %u attempts\r\n", (unsigned)attempts);
        return -2;
    }

    
    uint32_t avg = (uint32_t)(acc / (uint64_t)valid_count);
    bme.gas_reference = avg;

    

    return 0;
}

/* Compute gas contribution to IAQ index. */
int16_t bme_GetGasScore(void)
{
    /* Check the current status before continuing with the operation. */

    
    const float EPS = 1e-6f;
    float upper = (float)bme.gas_upper_limit;
    float lower = (float)bme.gas_lower_limit;

    if ((upper - lower) <= EPS) {
        log_printf("[ERROR] bme_GetGasScore: invalid gas limits upper=%f lower=%f\r\n", upper, lower);
        return -1;
    }

    /* Read reference (use float for formula) */
    float ref = (float)bme.gas_reference;

    
    float frac = 0.75f * (ref - lower) / (upper - lower);
    float score_f = frac * 100.0f;

    
    if (! (score_f > -1e9f && score_f < 1e9f) ) { 
        log_printf("[ERROR] bme_GetGasScore: numeric error (score=%f)\r\n", score_f);
        return -1;
    }

    
    if (score_f > 75.0f) score_f = 75.0f;
    if (score_f < 0.0f)  score_f = 0.0f;

    
    int16_t gas_score = (int16_t)(score_f + 0.5f);

    return gas_score;
}



/* Compute humidity contribution to IAQ index. */
int16_t bme_GetHumidityScore(void)
{
    /* Check the current status before continuing with the operation. */
    const float OPT_LOW  = 38.0f;
    const float OPT_HIGH = 42.0f;
    const float MAX_SCORE_F = 25.0f;   
    const float EPS = 1e-6f;

    /* Read globals (cast to float to be explicit) */
    float hum = (float) bme.humidity;           
    float hum_ref = (float) bme.hum_reference;  

    
    if (!isfinite(hum_ref) || hum_ref <= 0.0f || hum_ref >= 100.0f) {
        log_printf("[ERROR] bme_GetHumidityScore: invalid hum_reference=%.3f\r\n", hum_ref);
        return -1;
    }

    
    if (!isfinite(hum) || hum < 0.0f) {
        log_printf("[ERROR] bme_GetHumidityScore: invalid humidity value=%.3f\r\n", hum);
        return -1;
    }

    float score_f = 0.0f;

    
    if (hum >= OPT_LOW && hum <= OPT_HIGH) {
        score_f = MAX_SCORE_F;
    } else if (hum < OPT_LOW) {
        
        /* guard already performed for hum_ref > 0 */
        score_f = (MAX_SCORE_F * hum) / hum_ref;
    } else { 
        
        float denom = 100.0f - hum_ref;
        if (denom <= EPS) {
            log_printf("[ERROR] bme_GetHumidityScore: denom too small (100 - hum_ref = %.6f)\r\n", denom);
            return -1;
        }
        score_f = ((-0.25f / denom) * hum + 0.416666f) * 100.0f;
        
    }

    
    if (!isfinite(score_f)) {
        log_printf("[ERROR] bme_GetHumidityScore: numeric failure (score=%f)\r\n", score_f);
        return -1;
    }

    
    if (score_f < 0.0f) score_f = 0.0f;
    if (score_f > MAX_SCORE_F) score_f = MAX_SCORE_F;

    
    int16_t score = (int16_t)(score_f + 0.5f);
    return score;
}

/* Handles the bme calculateIAQ operation and returns the resulting status. */
void bme_calculateIAQ(int16_t score){
	/* Check the current status before continuing with the operation. */
	score = (100 - score) * 5;
	log_printf("IAQ: %d\r\n", score);

	log_printf("IAQ ");
	if(score >= 301)
		log_printf("Hazardous\r\n");
	else if (score >= 201 && score <= 300 )
		log_printf("Very Unhealthy\r\n");
	else if (score >= 176 && score <= 200 )
		log_printf("Unhealthy\r\n");
	else if (score >= 151 && score <= 175 )
		log_printf("Unhealthy for Sensitive Groups\r\n");
	else if (score >=  51 && score <= 150 )
		log_printf("Moderate\r\n");
	else if (score >=  00 && score <=  50 )
		log_printf("Good\r\n");
}

