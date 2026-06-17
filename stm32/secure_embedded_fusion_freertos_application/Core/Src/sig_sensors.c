/* Author: i fiksuari pas prog
 * Documentation date: 2026-06-17
 */

#include "sig_sensors.h"
#include "pwr.h"
#include "bme680.h"
/**
 * init_bme680 - initialize BME680 device
 *
 * Returns:
 *   0  -> success
 *  <0  -> failure (propagated error code from underlying call or -1 for local errors)
 */

extern volatile uint8_t system_fault_logging_active;
extern volatile uint8_t system_fault_owner;
volatile uint8_t bme680_gas_reference_active;

#ifndef BME680_STARTUP_GAS_REF_REGULATION_SAMPLES
#define BME680_STARTUP_GAS_REF_REGULATION_SAMPLES    (10U)
#endif

static uint8_t bme680_gas_ref_startup_regulation_left =
    BME680_STARTUP_GAS_REF_REGULATION_SAMPLES;

/* Configures BME680 measurement and gas-heater settings. */
int8_t init_bme680(void)
{
    /* Apply forced-mode settings before collecting the gas reference. */
    const uint8_t MAX_ATTEMPTS = 3U;
    const uint32_t BACKOFF_MS_BASE = 10U;  

    int8_t rslt = -1;
    uint8_t set_required_settings;

    struct bme680_dev *dev = &bme.gas_sensor;

    /* --- Configure device structure --- */
    dev->dev_id    = (uint8_t) to7bit((uint32_t) BME680_I2C_ADDR_PRIMARY);
    dev->intf      = BME680_I2C_INTF;
    dev->handle    = (void *) I2C1;           
    dev->read      = user_i2c_read;
    dev->write     = user_i2c_write;
    dev->delay_ms  = user_delay_ms;
    dev->amb_temp  = 25;                      

    /* --- bme680_init: retry transient failures --- */
    /* --- bme680_init: single attempt, return immediately on error --- */
    rslt = bme680_init(dev);
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_init failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);  
        return rslt;
    }

    /* --- Configure sensor settings (OS, filter, gas) --- */
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

    /* --- bme680_set_sensor_settings: single attempt, delay then return on error --- */
    rslt = bme680_set_sensor_settings(set_required_settings, dev);
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_settings failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);
        return rslt ? rslt : -2;
    }

    /* --- Set sensor mode (power mode) --- */
    rslt = bme680_set_sensor_mode(dev);
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_mode failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);  
        return rslt ? rslt : -3;
    }

    /* --- Get profile duration --- */
    rslt = bme680_get_profile_dur(&bme.meas_period, dev);
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_profile_dur failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);  
        return rslt ? rslt : -4;
    }

    /* --- Get gas reference --- */
    rslt = bme_GetGasReference();
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme_GetGasReference failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);  
        return rslt ? rslt : -5;
    }

    /*log_printf("[INFO] init_bme680: initialization successful (I2C 7-bit addr 0x%02X)\r\n", dev->dev_id);*/
    return 0;
}

/**
 * bme_GetGasReference - sample gas resistance and store an averaged reference
 *
 * Returns:
 *   0  -> success (bme.gas_reference updated)
 *  -1  -> device pointer / callbacks invalid
 *  -2  -> no valid readings collected within attempt budget
 *  -3  -> underlying bme680_get_sensor_data() permanent failure
 */
/* Collects valid BME680 gas samples for its reference. */
int8_t bme_GetGasReference(void)
{
    /* Ignore readings without the gas-valid status bit. */
    const uint8_t  TARGET_VALID_READS = 3U;
    const uint8_t  MAX_ATTEMPTS_TOTAL = 10U;
    const uint8_t  MAX_READ_RETRIES   = 2U;
    const uint32_t MAX_RUNTIME_MS     = 1500U;

    int8_t rslt;
    int8_t final_status = 0;
    struct bme680_field_data data;

    uint64_t acc = 0ULL;
    uint8_t valid_count = 0U;
    uint16_t attempts = 0U;
    uint32_t start_ms = ms_get();
    uint32_t last_valid_gas = 0U;

    bme680_gas_reference_active = 1U;
    heartbeat_signal_bme680main();

    rslt = null_ptr_check(&bme.gas_sensor);
    if (rslt != BME680_OK) {
        BME680_LOG_PRINTF("[ERROR] bme_GetGasReference: null_ptr_check failed err=%d\r\n",
                          (int)rslt);
        tim2_delay_ms(20U);
        final_status = -1;
        goto gas_ref_exit;
    }

    while ((valid_count < TARGET_VALID_READS) &&
           (attempts < MAX_ATTEMPTS_TOTAL)) {

        uint32_t elapsed_ms = ms_get() - start_ms;

        if (elapsed_ms > MAX_RUNTIME_MS) {
            break;
        }

        attempts++;
        heartbeat_signal_bme680main();

        if (bme.gas_sensor.power_mode == BME680_FORCED_MODE) {
            rslt = bme680_set_sensor_mode(&bme.gas_sensor);
            if (rslt != BME680_OK) {
                BME680_LOG_PRINTF("[WARN] bme_GetGasReference: set_sensor_mode failed err=%d attempt=%u/%u\r\n",
                                  (int)rslt,
                                  (unsigned)attempts,
                                  (unsigned)MAX_ATTEMPTS_TOTAL);
                tim2_delay_ms(20U);
                continue;
            }

            {
                uint32_t wait_ms = (bme.meas_period > 0U) ? (uint32_t)bme.meas_period : 100U;

                if (wait_ms > 250U) {
                    wait_ms = 250U;
                }

                heartbeat_signal_bme680main();

                if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
                    vTaskDelay(pdMS_TO_TICKS(wait_ms));
                } else {
                    bme.gas_sensor.delay_ms(wait_ms);
                }
            }
        }

        memset(&data, 0, sizeof(data));

        for (uint8_t retry = 0U; retry < MAX_READ_RETRIES; ++retry) {
            heartbeat_signal_bme680main();

            rslt = bme680_get_sensor_data(&data, &bme.gas_sensor);

            if (rslt == BME680_OK) {
                heartbeat_signal_bme680();
                break;
            }

            BME680_LOG_PRINTF("[WARN] bme_GetGasReference: transient read err=%d retry=%u/%u\r\n",
                              (int)rslt,
                              (unsigned)(retry + 1U),
                              (unsigned)MAX_READ_RETRIES);
            tim2_delay_ms(20U);

            if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
                vTaskDelay(pdMS_TO_TICKS(2U + ((uint32_t)retry * 5U)));
            } else {
                bme.gas_sensor.delay_ms(2U + ((uint32_t)retry * 5U));
            }
        }

        if (rslt != BME680_OK) {
            BME680_LOG_PRINTF("[ERROR] bme_GetGasReference: read failed permanently err=%d\r\n",
                              (int)rslt);
            tim2_delay_ms(20U);
            final_status = -3;
            goto gas_ref_exit;
        }

        if ((data.status & BME680_GASM_VALID_MSK) != 0U) {
            last_valid_gas = (uint32_t)data.gas_resistance;
            bme.gas_resistance = last_valid_gas;

            acc += (uint64_t)last_valid_gas;
            valid_count++;
        }

        heartbeat_signal_bme680main();
    }

    heartbeat_signal_bme680main();

    if ((valid_count == 0U) || (last_valid_gas == 0U)) {
        BME680_LOG_PRINTF("[ERROR] bme_GetGasReference: no valid gas readings attempts=%u\r\n",
                          (unsigned)attempts);
        tim2_delay_ms(20U);
        final_status = -2;
        goto gas_ref_exit;
    }

    {
        uint32_t avg = (uint32_t)(acc / (uint64_t)valid_count);

        bme.gas_reference = last_valid_gas;

        BME680_LOG_PRINTF("[INFO] bme_GetGasReference: valid=%u attempts=%u gas_ref=%lu avg=%lu\r\n",
                          (unsigned)valid_count,
                          (unsigned)attempts,
                          (unsigned long)bme.gas_reference,
                          (unsigned long)avg);
        tim2_delay_ms(20U);
    }

    bme680_gas_ref_startup_regulation_left =
        BME680_STARTUP_GAS_REF_REGULATION_SAMPLES;

    heartbeat_signal_bme680main();
    heartbeat_signal_bme680();

    final_status = 0;

gas_ref_exit:
    bme680_gas_reference_active = 0U;
    return final_status;
}
/* Calculates the gas contribution to the air-quality score. */
int16_t bme_GetGasScore(void)
{
    /* Clamp the valid gas score to its 75-point contribution. */
    uint32_t gas_ref;
    uint32_t gas_res;
    uint32_t gas_score;

    gas_ref = (uint32_t)bme.gas_reference;
    gas_res = (uint32_t)bme.gas_resistance;

    if (gas_res == 0U) {
        BME680_LOG_PRINTF("[ERROR] bme_GetGasScore: gas_resistance is zero\r\n");
        tim2_delay_ms(20U);
        return -1;
    }

    if (gas_ref == 0U) {
        bme.gas_reference = gas_res;
        gas_ref = gas_res;
    }

    if (bme680_gas_ref_startup_regulation_left > 0U) {
        if (gas_res < gas_ref) {
            bme.gas_reference = gas_res;
            gas_ref = gas_res;
        }

        bme680_gas_ref_startup_regulation_left--;
    }

    if (gas_ref == 0U) {
        BME680_LOG_PRINTF("[ERROR] bme_GetGasScore: gas_reference is zero after regulation\r\n");
        tim2_delay_ms(20U);
        return -1;
    }

    /*
     * Gas contribution max = 75.
     * If current gas resistance is equal/better than reference, return full gas score.
     */
    if (gas_res >= gas_ref) {
        gas_score = 75U;
    } else {
        gas_score = (uint32_t)(((uint64_t)gas_res * 75ULL) / (uint64_t)gas_ref);
    }

    if (gas_score > 75U) {
        gas_score = 75U;
    }

    return (int16_t)gas_score;
}

/*
 * Compute humidity contribution to IAQ index.
 *
 * Returns:
 *   >=0 : humidity score (0..25)
 *   -1  : error (invalid hum_reference or numeric failure)
 *
 * Notes:
 * - Keeps your original formulas but adds safety checks.
 * - Uses floating point for clarity; a fixed-point variant can be supplied if needed.
 */
/* Calculates the humidity contribution to the air-quality score. */
int16_t bme_GetHumidityScore(void)
{
    /* Limit the valid humidity score to 25 points. */
    const float OPT_LOW  = 38.0f;
    const float OPT_HIGH = 42.0f;
    const float MAX_SCORE_F = 25.0f;   /* your original 0.25 * 100 = 25 */
    const float EPS = 1e-6f;

    float hum = (float) bme.humidity;            
    float hum_ref = (float) bme.hum_reference;   

    if (!isfinite(hum_ref) || hum_ref <= 0.0f || hum_ref >= 100.0f) {
    	BME680_LOG_PRINTF("[ERROR] bme_GetHumidityScore: invalid hum_reference=%.3f\r\n", hum_ref);
    	tim2_delay_ms(20);
        return -1;
    }

    if (!isfinite(hum) || hum < 0.0f) {
    	BME680_LOG_PRINTF("[ERROR] bme_GetHumidityScore: invalid humidity value=%.3f\r\n", hum);
    	tim2_delay_ms(20);
        return -1;
    }

    float score_f = 0.0f;

    /* Optimal zone => fixed score */
    if (hum >= OPT_LOW && hum <= OPT_HIGH) {
        score_f = MAX_SCORE_F;
    } else if (hum < OPT_LOW) {
        /* below optimal: score = 0.25/hum_ref * hum * 100  -> equals 25 * hum / hum_ref */
        score_f = (MAX_SCORE_F * hum) / hum_ref;
    } else {  
        /* above optimal: original formula
         * score = ((-0.25 / (100 - hum_ref) * hum) + 0.416666) * 100
         * keep same algebra but make it clearer
         */
        float denom = 100.0f - hum_ref;
        if (denom <= EPS) {
        	BME680_LOG_PRINTF("[ERROR] bme_GetHumidityScore: denom too small (100 - hum_ref = %.6f)\r\n", denom);
        	tim2_delay_ms(20);
            return -1;
        }
        score_f = ((-0.25f / denom) * hum + 0.416666f) * 100.0f;
    }

    if (!isfinite(score_f)) {
    	BME680_LOG_PRINTF("[ERROR] bme_GetHumidityScore: numeric failure (score=%f)\r\n", score_f);
    	tim2_delay_ms(20);
        return -1;
    }

    if (score_f < 0.0f) score_f = 0.0f;
    if (score_f > MAX_SCORE_F) score_f = MAX_SCORE_F;

    int16_t score = (int16_t)(score_f + 0.5f);
    return score;
}

/* Logs the air-quality category derived from the combined score. */
void bme_calculateIAQ(int16_t score)
{
    /* Convert the input score to the implementation's IAQ scale. */
    const char *iaq_label;

    if (system_fault_logging_active != 0U) {
        return;
    }

    score = (100 - score) * 5;

    if (score >= 301) {
        iaq_label = "Hazardous";
    } else if (score >= 201) {
        iaq_label = "Very Unhealthy";
    } else if (score >= 176) {
        iaq_label = "Unhealthy";
    } else if (score >= 151) {
        iaq_label = "Unhealthy for Sensitive Groups";
    } else if (score >= 51) {
        iaq_label = "Moderate";
    } else {
        iaq_label = "Good";
    }

    BME680_LOG_PRINTF("IAQ: %d IAQ %s\r\n", score, iaq_label);
}
