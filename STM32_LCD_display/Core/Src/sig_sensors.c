/**
 * @file sig_sensors.c
 * @brief BME680 setup, gas reference and IAQ scoring.
 */
#include "sig_sensors.h"
#include "pwr.h"
#include "bme680.h"
extern volatile uint8_t system_fault_logging_active;
extern volatile uint8_t system_fault_owner;
volatile uint8_t bme680_gas_reference_active;
#ifndef BME680_STARTUP_GAS_REF_REGULATION_SAMPLES
#define BME680_STARTUP_GAS_REF_REGULATION_SAMPLES    (10U)
#endif
/* Remaining samples allowed to regulate the startup gas reference. */
static uint8_t bme680_gas_ref_startup_regulation_left =
    BME680_STARTUP_GAS_REF_REGULATION_SAMPLES;
/**
 * @brief Initialize and configure the BME680.
 * @return 0 on success, otherwise sensor error code.
 */
int8_t init_bme680(void)
{
    const uint8_t MAX_ATTEMPTS = 3U;
    const uint32_t BACKOFF_MS_BASE = 10U;
    int8_t rslt = -1;
    uint8_t set_required_settings;
    /* Keep one local handle for the sensor setup path. */
    struct bme680_dev *dev = &bme.gas_sensor;
    /* Bind the Bosch driver to I2C1 and project callbacks. */
    dev->dev_id    = (uint8_t) to7bit((uint32_t) BME680_I2C_ADDR_PRIMARY);
    dev->intf      = BME680_I2C_INTF;
    dev->handle    = (void *) I2C1;
    dev->read      = user_i2c_read;
    dev->write     = user_i2c_write;
    dev->delay_ms  = user_delay_ms;
    dev->amb_temp  = 25;
    /* Sensor init is one attempt here; upper layers own retry policy. */
    rslt = bme680_init(dev);
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_init failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);
        return rslt;
    }
    /* Configure oversampling, filter and gas heater. */
    dev->tph_sett.os_hum  = BME680_OS_2X;
    dev->tph_sett.os_pres = BME680_OS_4X;
    dev->tph_sett.os_temp = BME680_OS_8X;
    dev->tph_sett.filter  = BME680_FILTER_SIZE_3;
    dev->gas_sett.run_gas   = BME680_ENABLE_GAS_MEAS;
    dev->gas_sett.heatr_temp = 320;
    dev->gas_sett.heatr_dur  = 150;
    dev->power_mode = BME680_FORCED_MODE;
    /* Apply all configured sensing options in one settings update. */
    set_required_settings = (uint8_t)(BME680_OST_SEL | BME680_OSP_SEL |
                                      BME680_OSH_SEL | BME680_FILTER_SEL |
                                      BME680_GAS_SENSOR_SEL);
    rslt = bme680_set_sensor_settings(set_required_settings, dev);
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_settings failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);
        return rslt ? rslt : -2;
    }
    /* Forced mode triggers one measurement cycle at a time. */
    rslt = bme680_set_sensor_mode(dev);
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_set_sensor_mode failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);
        return rslt ? rslt : -3;
    }
    /* Keep the profile duration for later forced-mode waits. */
    rslt = bme680_get_profile_dur(&bme.meas_period, dev);
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme680_get_profile_dur failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);
        return rslt ? rslt : -4;
    }
    /* Build the starting gas reference before runtime IAQ. */
    rslt = bme_GetGasReference();
    if (rslt != 0) {
    	BME680_LOG_PRINTF("[ERROR] bme_GetGasReference failed (err=%d)\r\n", (int)rslt);
    	tim2_delay_ms(20);
        dev->delay_ms(BACKOFF_MS_BASE);
        return rslt ? rslt : -5;
    }
    return 0;
}
/**
 * @brief Build the initial gas-resistance reference.
 * @return 0 on success, otherwise negative.
 */
int8_t bme_GetGasReference(void)
{
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
    /* Mark calibration active so supervision can tolerate the longer path. */
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
    /* Collect a few valid gas samples within a bounded attempt/time budget. */
    while ((valid_count < TARGET_VALID_READS) &&
           (attempts < MAX_ATTEMPTS_TOTAL)) {
        uint32_t elapsed_ms = ms_get() - start_ms;
        if (elapsed_ms > MAX_RUNTIME_MS) {
            break;
        }
        attempts++;
        heartbeat_signal_bme680main();
        /* Start a fresh forced-mode measurement before each reference read. */
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
                /* Wait for the measurement profile, capped to keep calibration bounded. */
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
        /* Retry only the sensor-data read on transient failures. */
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
        /* Only gas-valid samples contribute to the reference. */
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
        /* Keep the average for diagnostics, but use the latest valid sample as reference. */
        uint32_t avg = (uint32_t)(acc / (uint64_t)valid_count);
        /* Latest valid gas value gives runtime IAQ a closer starting point. */
        bme.gas_reference = last_valid_gas;
        BME680_LOG_PRINTF("[INFO] bme_GetGasReference: valid=%u attempts=%u gas_ref=%lu avg=%lu\r\n",
                          (unsigned)valid_count,
                          (unsigned)attempts,
                          (unsigned long)bme.gas_reference,
                          (unsigned long)avg);
        tim2_delay_ms(20U);
    }
    /* Allow a short startup window to pull an overly-high reference down. */
    bme680_gas_ref_startup_regulation_left =
        BME680_STARTUP_GAS_REF_REGULATION_SAMPLES;
    heartbeat_signal_bme680main();
    heartbeat_signal_bme680();
    final_status = 0;
gas_ref_exit:
    bme680_gas_reference_active = 0U;
    return final_status;
}
/**
 * @brief Calculate the gas contribution to IAQ.
 * @return Gas score from 0 to 75, otherwise -1.
 */
int16_t bme_GetGasScore(void)
{
    uint32_t gas_ref;
    uint32_t gas_res;
    uint32_t gas_score;
    /* Work with local copies of the current gas state. */
    gas_ref = (uint32_t)bme.gas_reference;
    gas_res = (uint32_t)bme.gas_resistance;
    if (gas_res == 0U) {
        BME680_LOG_PRINTF("[ERROR] bme_GetGasScore: gas_resistance is zero\r\n");
        tim2_delay_ms(20U);
        return -1;
    }
    /* First valid runtime sample can seed an empty reference. */
    if (gas_ref == 0U) {
        bme.gas_reference = gas_res;
        gas_ref = gas_res;
    }
    /* During startup only, follow lower valid gas values. */
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
    /* Equal or better gas resistance gets the full gas score. */
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
/**
 * @brief Calculate the humidity contribution to IAQ.
 * @return Humidity score from 0 to 25, otherwise -1.
 */
int16_t bme_GetHumidityScore(void)
{
    const float OPT_LOW  = 38.0f;
    const float OPT_HIGH = 42.0f;
    const float MAX_SCORE_F = 25.0f;
    const float EPS = 1e-6f;
    /* Use current humidity against the configured reference. */
    float hum = (float) bme.humidity;
    float hum_ref = (float) bme.hum_reference;
    /* Reject a reference that would break the score calculation. */
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
    /* The 38-42% zone receives the full humidity score. */
    if (hum >= OPT_LOW && hum <= OPT_HIGH) {
        score_f = MAX_SCORE_F;
    } else if (hum < OPT_LOW) {
        score_f = (MAX_SCORE_F * hum) / hum_ref;
    } else {
        /* Above the optimal zone, scale against the remaining humidity range. */
        float denom = 100.0f - hum_ref;
        if (denom <= EPS) {
        	BME680_LOG_PRINTF("[ERROR] bme_GetHumidityScore: denom too small (100 - hum_ref = %.6f)\r\n", denom);
        	tim2_delay_ms(20);
            return -1;
        }
        score_f = ((-0.25f / denom) * hum + 0.416666f) * 100.0f;
    }
    /* Reject numeric failure before clamping and conversion. */
    if (!isfinite(score_f)) {
    	BME680_LOG_PRINTF("[ERROR] bme_GetHumidityScore: numeric failure (score=%f)\r\n", score_f);
    	tim2_delay_ms(20);
        return -1;
    }
    /* Keep the humidity contribution inside 0..25. */
    if (score_f < 0.0f) score_f = 0.0f;
    if (score_f > MAX_SCORE_F) score_f = MAX_SCORE_F;
    int16_t score = (int16_t)(score_f + 0.5f);
    return score;
}
/**
 * @brief Convert the combined sensor score to IAQ.
 * @param score Combined gas and humidity score.
 * @param label_out Optional IAQ label output.
 * @return IAQ value.
 */
int16_t bme_calculateIAQ(
    int16_t score,
    const char **label_out)
{
    const char *iaq_label;
    int16_t iaq_score;
    /* Convert the 0..100 combined score into the project IAQ scale. */
    iaq_score =
        (int16_t)((100 - score) * 5);
    /* Map the calculated value to the dashboard/log label. */
    if (iaq_score >= 301)
    {
        iaq_label = "Hazardous";
    }
    else if (iaq_score >= 201)
    {
        iaq_label = "Very Unhealthy";
    }
    else if (iaq_score >= 176)
    {
        iaq_label = "Unhealthy";
    }
    else if (iaq_score >= 151)
    {
        iaq_label =
            "Unhealthy for Sensitive Groups";
    }
    else if (iaq_score >= 51)
    {
        iaq_label = "Moderate";
    }
    else
    {
        iaq_label = "Good";
    }
    /* Caller can keep the label without depending on serial logging. */
    if (label_out != NULL)
    {
        *label_out = iaq_label;
    }
    /* Fault capture can suppress the print without changing IAQ output. */
    if (system_fault_logging_active == 0U)
    {
        BME680_LOG_PRINTF(
            "IAQ: %d IAQ %s\r\n",
            iaq_score,
            iaq_label);
    }
    return iaq_score;
}
