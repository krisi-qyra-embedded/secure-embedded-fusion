#include "max31865.h"
#include "tim.h"
#include "uart.h"
#include "systick.h"
#include "iwdg.h"
#include <math.h>
#include <string.h>


/* VCC → 3.3V GND → GND CS → PB6 CLK → PB13 SDO → PA11 SDI → PA1 */


static void _log_hex(const char *label, const uint8_t *buf, uint16_t len)
{
    /* Capture the available diagnostic values. */
    log_printf("%s:", label);
    for (uint16_t i = 0U; i < len; ++i) {
        log_printf(" %02X", buf[i]);
    }
    log_printf("\r\n");
}

/* Initializes the resources used by gpio pb6 cs init. */
int gpio_pb6_cs_init(void)
{
    /* Set up the required peripheral configuration. */
    const uint32_t pin = 6U;

#ifdef RCC_AHB1ENR_GPIOBEN
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
#else
    RCC->AHB1ENR |= (1U << 1);
#endif
    __DSB();
    __ISB();

    GPIOB->MODER &= ~(3U << (pin * 2U));
    GPIOB->MODER |=  (1U << (pin * 2U));

    GPIOB->OTYPER &= ~(1U << pin);

    GPIOB->OSPEEDR &= ~(3U << (pin * 2U));
    GPIOB->OSPEEDR |=  (1U << (pin * 2U));

    GPIOB->PUPDR &= ~(3U << (pin * 2U));

    GPIOB->ODR |= (1U << pin);

    __DSB();
    __ISB();

    return 0;
}

/* Handles the MPU CS operation. */
int MPU_CS(uint8_t state)
{
    /* Check the current status before proceeding. */
    const uint32_t pin = 6U;

    if (state == CS_SELECT) {
        GPIOB->ODR &= ~(1U << pin);
        __DSB();
        __ISB();
        return 0;
    }

    if (state == CS_DESELECT) {
        GPIOB->ODR |= (1U << pin);
        __DSB();
        __ISB();
        return 0;
    }

    return -1;
}

/* Reads data through Max31865 readRegisterN and returns the operation status. */
max31865_status_t Max31865_readRegisterN(Max31865_t *max31865, uint8_t addr, uint8_t *buffer, uint8_t n)
{
    /* Check the returned status before accepting the sample. */
    if ((max31865 == NULL) || (buffer == NULL) || (n == 0U)) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    const uint16_t total = (uint16_t)(1U + n);
    if (total > MAX_STACK_TOTAL) {
        return MAX31865_STATUS_SIZE_ERROR;
    }

    uint8_t txbuf[MAX_STACK_TOTAL];
    uint8_t rxbuf[MAX_STACK_TOTAL];

    addr &= 0x7Fu;
    txbuf[0] = addr;
    for (uint16_t i = 1U; i < total; ++i) {
        txbuf[i] = 0xFFu;
    }
    memset(rxbuf, 0, total);

    MPU_CS(CS_SELECT);

    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    if (spi4_dma_transmit_receive(txbuf, rxbuf, total) != SPI_DMA_STATUS_OK) {
        MPU_CS(CS_DESELECT);
        return MAX31865_STATUS_DMA_ERROR;
    }

    if (spi_dma_stream1_wait_complete(TRANSFER_TIMEOUT_MS) != SPI_DMA_STATUS_OK) {
        MPU_CS(CS_DESELECT);
        return MAX31865_STATUS_TIMEOUT;
    }

    if (spi_dma_stream0_wait_complete(TRANSFER_TIMEOUT_MS) != SPI_DMA_STATUS_OK) {
        MPU_CS(CS_DESELECT);
        return MAX31865_STATUS_TIMEOUT;
    }

    if (spi4_wait_bsy_clear_ms(TRANSFER_TIMEOUT_MS) != SPI_DMA_STATUS_OK) {
        MPU_CS(CS_DESELECT);
        return MAX31865_STATUS_TIMEOUT;
    }

    for (uint16_t i = 0U; i < n; ++i) {
        buffer[i] = rxbuf[1U + i];
    }

    MPU_CS(CS_DESELECT);
    return MAX31865_STATUS_OK;
}

/* Reads data through Max31865 readRegister8 and returns the operation status. */
max31865_status_t Max31865_readRegister8(Max31865_t *max31865, uint8_t addr, uint8_t *value)
{
    /* Check the returned status before accepting the sample. */
    if (value == NULL) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    return Max31865_readRegisterN(max31865, addr, value, 1U);
}

/* Reads data through Max31865 readRegister16 and returns the operation status. */
max31865_status_t Max31865_readRegister16(Max31865_t *max31865, uint8_t addr, uint16_t *value)
{
    /* Check the returned status before accepting the sample. */
    if (value == NULL) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    uint8_t buffer[2] = {0U, 0U};
    const max31865_status_t status = Max31865_readRegisterN(max31865, addr, buffer, 2U);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    *value = ((uint16_t)buffer[0] << 8) | buffer[1];
    return MAX31865_STATUS_OK;
}

/* Writes data through Max31865 writeRegister8 and reports transfer errors. */
max31865_status_t Max31865_writeRegister8(Max31865_t *dev, uint8_t addr, uint8_t data)
{
    /* Check the transfer result before continuing. */
    if (dev == NULL) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    uint8_t txbuf[2];
    txbuf[0] = (uint8_t)(addr | 0x80U);
    txbuf[1] = data;

    MPU_CS(CS_SELECT);

    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    if (spi4_dma_transmit(txbuf, 2U) != SPI_DMA_STATUS_OK) {
        MPU_CS(CS_DESELECT);
        return MAX31865_STATUS_DMA_ERROR;
    }

    if (spi_dma_stream1_wait_complete(TRANSFER_TIMEOUT_MS) != SPI_DMA_STATUS_OK) {
        MPU_CS(CS_DESELECT);
        return MAX31865_STATUS_TIMEOUT;
    }

    if (spi4_wait_bsy_clear_ms(TRANSFER_TIMEOUT_MS) != SPI_DMA_STATUS_OK) {
        MPU_CS(CS_DESELECT);
        return MAX31865_STATUS_TIMEOUT;
    }

    MPU_CS(CS_DESELECT);
    return MAX31865_STATUS_OK;
}

/* Reads data through Max31865 readFault and returns the operation status. */
max31865_status_t Max31865_readFault(Max31865_t *max31865, uint8_t *out_fault)
{
    /* Check the returned status before accepting the sample. */
    return Max31865_readRegister8(max31865, MAX31856_FAULTSTAT_REG, out_fault);
}

/* Handles the Max31865 clearFault operation. */
max31865_status_t Max31865_clearFault(Max31865_t *max31865)
{
    /* Check the current status before proceeding. */
    uint8_t cfg = 0U;
    max31865_status_t status = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG, &cfg);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    cfg &= (uint8_t)~0x2CU;
    cfg |= MAX31856_CONFIG_FAULTSTAT;

    return Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
}

/* Updates the enable state for Max31865 enableBias. */
max31865_status_t Max31865_enableBias(Max31865_t *max31865, uint8_t enable)
{
    /* Check the current status before proceeding. */
    uint8_t cfg = 0U;
    max31865_status_t status = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG, &cfg);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    if (enable != 0U) {
        cfg |= MAX31856_CONFIG_BIAS;
    } else {
        cfg &= (uint8_t)~MAX31856_CONFIG_BIAS;
    }

    return Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
}

/* Handles the Max31865 autoConvert operation. */
max31865_status_t Max31865_autoConvert(Max31865_t *max31865, uint8_t enable)
{
    /* Check the current status before proceeding. */
    uint8_t cfg = 0U;
    max31865_status_t status = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG, &cfg);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    if (enable != 0U) {
        cfg |= MAX31856_CONFIG_MODEAUTO;
    } else {
        cfg &= (uint8_t)~MAX31856_CONFIG_MODEAUTO;
    }

    return Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
}

/* Accesses the requested setting through Max31865 setWires. */
max31865_status_t Max31865_setWires(Max31865_t *max31865, uint8_t numWires)
{
    /* Check the current status before proceeding. */
    uint8_t cfg = 0U;
    max31865_status_t status = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG, &cfg);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    if (numWires == 3U) {
        cfg |= MAX31856_CONFIG_3WIRE;
    } else {
        cfg &= (uint8_t)~MAX31856_CONFIG_3WIRE;
    }

    return Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
}

/* Accesses the requested setting through Max31865 setFilter. */
max31865_status_t Max31865_setFilter(Max31865_t *max31865, uint8_t filterHz)
{
    /* Check the current status before proceeding. */
    uint8_t cfg = 0U;
    max31865_status_t status = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG, &cfg);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    if (filterHz == 50U) {
        cfg |= MAX31856_CONFIG_FILT50HZ;
    } else {
        cfg &= (uint8_t)~MAX31856_CONFIG_FILT50HZ;
    }

    return Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
}

/* Reads data through Max31865 readRTD and returns the operation status. */
max31865_status_t Max31865_readRTD(Max31865_t *max31865, uint16_t *rtd_value)
{
    /* Check the returned status before accepting the sample. */
    if ((max31865 == NULL) || (rtd_value == NULL)) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    max31865_status_t status = Max31865_clearFault(max31865);
    if (status != MAX31865_STATUS_OK) {
        log_printf("WARN: readRTD clearFault r=%d\r\n", status);
        tim2_delay_ms(20U);
    }

    status = Max31865_enableBias(max31865, 1U);
    if (status != MAX31865_STATUS_OK) {
        log_printf("WARN: readRTD enableBias r=%d\r\n", status);
        tim2_delay_ms(20U);
    }

    uint8_t cfg = 0U;
    status = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG, &cfg);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    cfg |= MAX31856_CONFIG_1SHOT;
    status = Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: readRTD start 1shot failed r=%d\r\n", status);
        tim2_delay_ms(20U);
        return status;
    }

    status = Max31865_readRegister16(max31865, MAX31856_RTDMSB_REG, rtd_value);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    *rtd_value >>= 1U;
    return MAX31865_STATUS_OK;
}

/* Initializes the resources used by Max31865 init. */
max31865_status_t Max31865_init(Max31865_t *max31865, uint8_t numwires, uint8_t filterHz)
{
    /* Set up the required peripheral configuration. */
    if (max31865 == NULL) {
        log_printf("ERR: init invalid args (max31865 == NULL)\r\n");
        tim2_delay_ms(20U);
        return MAX31865_STATUS_INVALID_ARG;
    }

    hb_main_ts = 0U;
    MPU_CS(CS_DESELECT);

    max31865->wires = numwires;
    max31865->filter_hz = filterHz;

    max31865_status_t status = Max31865_setWires(max31865, numwires);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: init setWires failed r=%d\r\n", status);
        tim2_delay_ms(20U);
        return status;
    }

    status = Max31865_enableBias(max31865, 0U);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: init enableBias failed r=%d\r\n", status);
        tim2_delay_ms(20U);
        return status;
    }

    status = Max31865_autoConvert(max31865, 0U);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: init autoConvert failed r=%d\r\n", status);
        tim2_delay_ms(20U);
        return status;
    }

    status = Max31865_clearFault(max31865);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: init clearFault failed r=%d\r\n", status);
        tim2_delay_ms(20U);
        return status;
    }

    status = Max31865_setFilter(max31865, filterHz);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: init setFilter failed r=%d\r\n", status);
        tim2_delay_ms(20U);
        return status;
    }

    log_printf("OK: Max31865 initialized (wires=%u filter=%u)\r\n", (unsigned)numwires, (unsigned)filterHz);
    tim2_delay_ms(1);
    return MAX31865_STATUS_OK;
}

 
/* Attempts recovery of Max31865 log recovery action after a failed transfer. */
static void Max31865_log_recovery_action(int attempt, int max_attempts)
{
    /* Recover the peripheral state before retrying the transfer. */
    log_printf("ACT: sample attempt %d/%d failed -> starting SPI/MAX31865 recovery path\r\n",
               attempt, max_attempts);
}

/* Datasheet-oriented fault logger for 3-wire MAX31865 setups Uses: - raw 15-bit RTD code after shifting. */
static void Max31865_log_fault_diagnosis(uint16_t raw, uint8_t fault)
{
    /* Capture the available diagnostic values. */
    const float rt_ohms = (((float)raw) / 32768.0f) * _MAX31865_RREF;

    log_printf("DBG: MAX31865 context: raw=0x%04X (%lu), approx_Rt=%.3f ohm, fault=0x%02X\r\n",
               raw, (unsigned long)raw, rt_ohms, (unsigned)fault);
    tim2_delay_ms(20U);
    /* Raw-code interpretation from datasheet behavior */
    if (raw == 0x7FFFU) {
        log_printf("DIAG: RTD code is FULL SCALE (32767). Datasheet meaning: saturated-high measurement.\r\n");
        tim2_delay_ms(20U);
    } else if (raw == 0x0000U) {
        log_printf("DIAG: RTD code is NEAR ZERO (0). Datasheet meaning: saturated-low measurement.\r\n");
        tim2_delay_ms(20U);
    }

    /* No MAX31865 fault bits set -> likely not a clean threshold fault */
    if (fault == 0U) {
        if ((raw == 0x7FFFU) || (raw == 0x0000U)) {
            log_printf("DIAG: No MAX31865 fault bits are set. This makes a pure RTD threshold fault less likely.\r\n");
            log_printf("DIAG: Suspect SPI/CS/DMA/timing, bias/conversion timing, stale register read, transient power/noise, or intermittent wiring.\r\n");
            tim2_delay_ms(20U);
        } else {
            log_printf("DIAG: No MAX31865 fault bits are set.\r\n");
            tim2_delay_ms(20U);
        }
        return;
    }

    /* D7: High threshold -> measured resistance > high threshold */
    if ((fault & MAX31865_FAULT_HIGHTHRESH) != 0U) {
        log_printf("DIAG: D7 HIGH-THRESHOLD fault: measured RTD resistance is above the programmed high threshold.\r\n");
        log_printf("DIAG: Possible causes: open RTD element, RTDIN+ shorted high, FORCE+ shorted high, or FORCE+ not connected to RTD.\r\n");
        log_printf("DIAG: Resulting data trend: FULL SCALE.\r\n");
        tim2_delay_ms(20U);
    }

    /* D6: Low threshold -> measured resistance < low threshold */
    if ((fault & MAX31865_FAULT_LOWTHRESH) != 0U) {
        log_printf("DIAG: D6 LOW-THRESHOLD fault: measured RTD resistance is below the programmed low threshold.\r\n");
        log_printf("DIAG: Possible causes: RTDIN+ shorted to RTDIN-, RTDIN+ shorted low and not connected to RTD, or FORCE+ shorted low.\r\n");
        log_printf("DIAG: Resulting data trend: NEAR ZERO.\r\n");
        tim2_delay_ms(20U);
    }

     
    if ((fault & MAX31865_FAULT_REFINLOW) != 0U) {
        log_printf("DIAG: D5 fault: VREFIN- > 0.85 x VBIAS.\r\n");
        log_printf("DIAG: Possible causes: open RTD element, FORCE+ shorted high and connected to RTD, FORCE+ unconnected, or FORCE+ shorted high and not connected to RTD.\r\n");
        log_printf("DIAG: Resulting data trend: FULL SCALE or INDETERMINATE.\r\n");
        tim2_delay_ms(20U);
    }

     
    if ((fault & MAX31865_FAULT_REFINHIGH) != 0U) {
        log_printf("DIAG: D4 fault: VREFIN- < 0.85 x VBIAS.\r\n");
        log_printf("DIAG: Typical cause: FORCE- open / broken return path / low-side wiring issue.\r\n");
        log_printf("DIAG: Resulting data trend: may APPEAR VALID even though the measurement path is wrong.\r\n");
        tim2_delay_ms(20U);
    }

     
    if ((fault & MAX31865_FAULT_RTDINLOW) != 0U) {
        log_printf("DIAG: D3 fault: VRTDIN- < 0.85 x VBIAS.\r\n");
        log_printf("DIAG: Typical cause: FORCE- open or RTDIN- low-side wiring issue.\r\n");
        log_printf("DIAG: Resulting data trend: NEAR ZERO or deceptively VALID-looking.\r\n");
        tim2_delay_ms(20U);
    }

     
    if ((fault & MAX31865_FAULT_OVUV) != 0U) {
        log_printf("DIAG: D2 over/under-voltage fault: a protected input went above VDD or below GND.\r\n");
        log_printf("DIAG: Resulting data trend: INDETERMINATE. Check supply, grounding, noise, and transients.\r\n");
        tim2_delay_ms(20U);
    }

    /* Holistic summary based on raw + fault combination */
    if (raw == 0x7FFFU) {
        if ((fault & (MAX31865_FAULT_HIGHTHRESH | MAX31865_FAULT_REFINLOW)) != 0U) {
            log_printf("DIAG: Holistic summary: this sample looks like an OPEN-CIRCUIT / VERY-HIGH-RESISTANCE style fault.\r\n");
            tim2_delay_ms(20U);
        } else {
            log_printf("DIAG: Holistic summary: full-scale data observed; combine the fault bits above with wiring and power checks.\r\n");
            tim2_delay_ms(20U);
        }
    } else if (raw == 0x0000U) {
        if ((fault & (MAX31865_FAULT_LOWTHRESH | MAX31865_FAULT_RTDINLOW)) != 0U) {
            log_printf("DIAG: Holistic summary: this sample looks like a SHORT / VERY-LOW-RESISTANCE style fault.\r\n");
            tim2_delay_ms(20U);
        } else {
            log_printf("DIAG: Holistic summary: near-zero data observed; combine the fault bits above with wiring and power checks.\r\n");
            tim2_delay_ms(20U);
        }
    } else if ((fault & (MAX31865_FAULT_REFINHIGH | MAX31865_FAULT_RTDINLOW)) != 0U) {
        log_printf("DIAG: Holistic summary: data can look numerically plausible, but the analog return path is suspect. Do not trust the sample.\r\n");
        tim2_delay_ms(20U);
    }
}


/* Reads data through Max31865 readTempC and returns the operation status. */
max31865_status_t Max31865_readTempC(Max31865_t *max31865, float *readTemp)
{
    /* Check the returned status before accepting the sample. */
    if ((max31865 == NULL) || (readTemp == NULL)) {
        log_printf("ERR: readTempC invalid args\r\n");
        tim2_delay_ms(20U);
        return MAX31865_STATUS_INVALID_ARG;
    }

    enum { MAX_ATTEMPTS = 3 };

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
        uint16_t raw = 0U;
        uint8_t fault = 0U;
        bool fault_valid = false;
        max31865_status_t status;

        status = Max31865_readRTD(max31865, &raw);
        if (status != MAX31865_STATUS_OK) {
            log_printf("ERR: readTempC: readRTD failed r=%d (attempt %d/%d)\r\n",
                       status, attempt, MAX_ATTEMPTS);
            log_printf("DIAG: No trustworthy RTD sample was produced. This points more to transaction/control/timing/recovery path issues than to a clean RTD threshold fault.\r\n");
            tim2_delay_ms(20U);
        } else {
            max31865_status_t fault_status = Max31865_readFault(max31865, &fault);
            if (fault_status == MAX31865_STATUS_OK) {
                fault_valid = true;
            } else {
                log_printf("WARN: readTempC: readFault failed r=%d while diagnosing raw=0x%04X\r\n",
                           fault_status, raw);
                tim2_delay_ms(20U);
            }
        }

        if ((status == MAX31865_STATUS_OK) && (raw != 0U) && (raw < 0x7FFFU)) {
            float Z1;
            float Z2;
            float Z3;
            float Z4;
            float Rt;
            float temp;

            Rt = ((float)raw) / 32768.0f;
            Rt *= _MAX31865_RREF;

            Z1 = -RTD_A;
            Z2 = RTD_A * RTD_A - (4.0f * RTD_B);
            Z3 = (4.0f * RTD_B) / _MAX31865_RNOMINAL;
            Z4 = 2.0f * RTD_B;
            temp = Z2 + (Z3 * Rt);

            if (temp >= 0.0f) {
                temp = (sqrtf(temp) + Z1) / Z4;
            } else {
                Rt /= _MAX31865_RNOMINAL;
                Rt *= 100.0f;
                {
                    float rpoly = Rt;
                    temp = -242.02f;
                    temp += 2.2228f * rpoly;
                    rpoly *= Rt;
                    temp += 2.5859e-3f * rpoly;
                    rpoly *= Rt;
                    temp -= 4.8260e-6f * rpoly;
                    rpoly *= Rt;
                    temp -= 2.8183e-8f * rpoly;
                    rpoly *= Rt;
                    temp += 1.5243e-10f * rpoly;
                }
            }

            if (fault_valid && (fault != 0U)) {
                log_printf("ERR: readTempC: sample looked numerically usable, but MAX31865 reports a fault\r\n");
                tim2_delay_ms(20U);
                Max31865_log_fault_diagnosis(raw, fault);
                return MAX31865_STATUS_FAULT;
            }

            *readTemp = temp;
            log_printf("OK: TempC = %.4f °C (raw=0x%04X, dec=%lu, approx_Rt=%.3f ohm)\r\n",
                       temp,
                       raw,
                       (unsigned long)raw,
                       (((float)raw) / 32768.0f) * _MAX31865_RREF);
            heartbeat_signal_main();
            return MAX31865_STATUS_OK;
        }

        if (status == MAX31865_STATUS_OK) {
            log_printf("ERR: readTempC: invalid RTD value raw=0x%04X (%lu) (attempt %d/%d)\r\n",
                       raw, (unsigned long)raw, attempt, MAX_ATTEMPTS);
            tim2_delay_ms(20U);
            if (raw == 0x7FFFU) {
                log_printf("DIAG: Raw code hit FULL SCALE (32767). In the MAX31865 tables this corresponds to a saturated-high result.\r\n");
                tim2_delay_ms(20U);
            } else if (raw == 0x0000U) {
                log_printf("DIAG: Raw code hit NEAR ZERO (0). In the MAX31865 tables this corresponds to a saturated-low result.\r\n");
                tim2_delay_ms(20U);
            } else {
                log_printf("DIAG: Raw code is outside the accepted window for this routine, so the sample is treated as invalid.\r\n");
                tim2_delay_ms(20U);
            }

            if (fault_valid) {
                Max31865_log_fault_diagnosis(raw, fault);
            } else {
                log_printf("WARN: readTempC: fault register unavailable for this invalid sample; diagnosis limited to raw-code behavior.\r\n");
                tim2_delay_ms(20U);
            }
        }

        if (attempt < MAX_ATTEMPTS) {
            Max31865_log_recovery_action(attempt, MAX_ATTEMPTS);

            if (spi4_recover() == SPI_DMA_STATUS_OK) {
                log_printf("INFO: readTempC: spi4_recover() succeeded, doing full restore/init\r\n");

                if (spi4_restore_and_init() == SPI_DMA_STATUS_OK) {
                    log_printf("INFO: readTempC: spi4_restore_and_init() succeeded, reinitializing device\r\n");

                    memset(max31865, 0, sizeof(*max31865));
                    if (Max31865_init(max31865, 3U, 50U) != MAX31865_STATUS_OK) {
                        log_printf("FATAL: Max31865_init failed after recovery -> restarting\r\n");
                        tim2_delay_ms(20U);
                        NVIC_SystemReset();
                    }

                    log_printf("ACT: recovery complete, retrying temperature conversion\r\n");
                    spi4_short_delay_ms(5U);
                    continue;
                }

                log_printf("WARN: readTempC: spi4_restore_and_init() failed (attempt %d/%d)\r\n",
                           attempt, MAX_ATTEMPTS);
                tim2_delay_ms(20U);
            } else {
                log_printf("WARN: readTempC: spi4_recover() failed (attempt %d/%d)\r\n",
                           attempt, MAX_ATTEMPTS);
                tim2_delay_ms(20U);
            }

            log_printf("ACT: short backoff before next retry\r\n");
            spi4_short_delay_ms(10U);
        }
    }

    log_printf("ERR: readTempC: all attempts exhausted; returning INVALID_DATA\r\n");
    tim2_delay_ms(20U);
    return MAX31865_STATUS_INVALID_DATA;
}


/* Reads data through Max31865 readTempF and returns the operation status. */
max31865_status_t Max31865_readTempF(Max31865_t *max31865, float *readTemp)
{
    /* Check the returned status before accepting the sample. */
    if ((max31865 == NULL) || (readTemp == NULL)) {
        log_printf("ERR: readTempF invalid args\r\n");
        return MAX31865_STATUS_INVALID_ARG;
    }

    float temp_c = 0.0f;
    max31865_status_t status = Max31865_readTempC(max31865, &temp_c);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: readTempF -> Max31865_readTempC failed\r\n");
        return status;
    }

    *readTemp = (temp_c * 9.0f / 5.0f) + 32.0f;
    return MAX31865_STATUS_OK;
}

/* Handles the Max31865 Filter operation. */
float Max31865_Filter(float newInput, float lastOutput, float effectiveFactor)
{
    /* Check the current status before proceeding. */
    return (lastOutput * (1.0f - effectiveFactor)) + (newInput * effectiveFactor);
}

/* Handles the Max31865 monitorLoop operation. */
max31865_status_t Max31865_monitorLoop(Max31865_t *max31865, uint32_t interval_ms)
{
    /* Check the current status before proceeding. */
    if (max31865 == NULL) {
        log_printf("ERR: monitorLoop invalid args\r\n");
        return MAX31865_STATUS_INVALID_ARG;
    }

    float tmp = 0.0f;

    while (1) {
        const max31865_status_t status = Max31865_readTempC(max31865, &tmp);
        if (status == MAX31865_STATUS_OK) {
            log_printf("MON: Temp = %.3f °C\r\n", tmp);
        } else {
            uint8_t fault = 0U;
            (void)Max31865_readFault(max31865, &fault);
            log_printf("MON: Read failed code=%d, fault=0x%02X\r\n", status, fault);
        }

        tim2_delay_ms(interval_ms);
    }
}

/* Logs diagnostic information from Max31865 debug once. */
max31865_status_t Max31865_debug_once(Max31865_t *dev)
{
    /* Capture the available diagnostic values. */
    if (dev == NULL) {
        log_printf("ERR: debug_once invalid arg (dev==NULL)\r\n");
        return MAX31865_STATUS_INVALID_ARG;
    }

    uint8_t raw_buf[2] = {0U, 0U};
    max31865_status_t status = Max31865_readRegisterN(dev, MAX31856_RTDMSB_REG, raw_buf, 2U);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: debug_once read raw16 failed r=%d\r\n", status);
        return status;
    }

    const uint16_t raw16 = ((uint16_t)raw_buf[0] << 8) | raw_buf[1];
    const uint16_t raw   = raw16 >> 1U;

    uint8_t cfg = 0U;
    status = Max31865_readRegisterN(dev, MAX31856_CONFIG_REG, &cfg, 1U);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: debug_once read cfg failed r=%d\r\n", status);
        tim2_delay_ms(20U);
        return status;
    }

    uint8_t fault = 0U;
    status = Max31865_readRegisterN(dev, MAX31856_FAULTSTAT_REG, &fault, 1U);
    if (status != MAX31865_STATUS_OK) {
        log_printf("ERR: debug_once read fault failed r=%d\r\n", status);
        tim2_delay_ms(20U);
        return status;
    }

    const uint64_t Rt_milli = ((uint64_t)raw * (uint64_t)RREF_MILLI_OHMS) / 32768ULL;
    const float Rt_f = (float)Rt_milli / 1000.0f;
    const float Rref_ohm = (float)RREF_MILLI_OHMS / 1000.0f;
    const float expected_raw_for_100ohm = (32768.0f * 100.0f) / Rref_ohm;

    (void)cfg;
    (void)fault;
    (void)Rt_f;
    (void)expected_raw_for_100ohm;

    return MAX31865_STATUS_OK;
}
