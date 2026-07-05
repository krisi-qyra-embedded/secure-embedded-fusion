/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "max31865.h"
#include "tim.h"
#include "uart.h"
#include "systick.h"
#include "iwdg.h"
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "spi_dma.h"
#include "i2c_dma.h"   /* for app_fault_delay_ms/system fault helpers if needed */

extern SemaphoreHandle_t max31865_spi_rx_sem;
extern SemaphoreHandle_t max31865_spi_tx_sem;
extern SemaphoreHandle_t xSerialSemaphore;
extern volatile uint8_t system_fault_logging_active;
extern volatile uint8_t system_fault_owner;
extern float temperatureC;

#ifndef MAX31865_BIAS_SETTLE_MS
#define MAX31865_BIAS_SETTLE_MS          10U
#endif

#ifndef MAX31865_ONESHOT_CONVERSION_MS
#define MAX31865_ONESHOT_CONVERSION_MS   70U
#endif

static void app_max31865_log_local_runtime_error(const char *tag,
                                                 int32_t rc,
                                                 int attempt,
                                                 int max_attempts);

/* Drains stale MAX31865 DMA completion tokens. */
static void max31865_clear_dma_semaphore_if_rtos(SemaphoreHandle_t sem)
{
    /* Take pending semaphore tokens without blocking the task. */
    if ((xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) && (sem != NULL)) {
        while (xSemaphoreTake(sem, 0U) == pdTRUE) {
        }
    }
}

/* Logs a labeled sensor byte buffer. */
static void _log_hex(const char *label, const uint8_t *buf, uint16_t len)
{
    /* Print each byte in hexadecimal for transfer diagnostics. */
    MAX31865_LOG_PRINTF("%s:", label);
    for (uint16_t i = 0U; i < len; ++i) {
        MAX31865_LOG_PRINTF(" %02X", buf[i]);
    }
    MAX31865_LOG_PRINTF("\r\n");
}

/* Configures PB6 as the MAX31865 chip-select output. */
int gpio_pb6_cs_init(void)
{
    /* Leave chip select high after configuring the pin. */
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

/* Controls the MAX31865 chip-select line. */
int MPU_CS(uint8_t state)
{
    /* Drive PB6 low for selection and reject unknown states. */
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

/* Reads consecutive MAX31865 registers through SPI DMA. */
max31865_status_t Max31865_readRegisterN(Max31865_t *max31865,
                                         uint8_t addr,
                                         uint8_t *buffer,
                                         uint8_t n)
{
    /* Keep chip select asserted until both DMA streams and SPI complete. */
    if ((max31865 == NULL) || (buffer == NULL) || (n == 0U)) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    const uint16_t total = (uint16_t)(1U + n);
    if (total > MAX_STACK_TOTAL) {
        return MAX31865_STATUS_SIZE_ERROR;
    }

    uint8_t txbuf[MAX_STACK_TOTAL];
    uint8_t rxbuf[MAX_STACK_TOTAL];

    addr &= 0x7FU;

    txbuf[0] = addr;
    for (uint16_t i = 1U; i < total; ++i) {
        txbuf[i] = 0xFFU;
    }

    memset(rxbuf, 0, total);

    /*
     * Clear stale DMA semaphore tokens before starting the new full-duplex SPI transfer.
     * RX -> DMA2_Stream0 -> max31865_spi_rx_sem
     * TX -> DMA2_Stream1 -> max31865_spi_tx_sem
     */
    max31865_clear_dma_semaphore_if_rtos(max31865_spi_rx_sem);
    max31865_clear_dma_semaphore_if_rtos(max31865_spi_tx_sem);

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

/* Reads one MAX31865 register byte. */
max31865_status_t Max31865_readRegister8(Max31865_t *max31865, uint8_t addr, uint8_t *value)
{
    /* Require an output pointer before using the burst-read helper. */
    if (value == NULL) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    return Max31865_readRegisterN(max31865, addr, value, 1U);
}

/* Reads a big-endian MAX31865 register pair. */
max31865_status_t Max31865_readRegister16(Max31865_t *max31865, uint8_t addr, uint16_t *value)
{
    /* Combine the received high and low bytes after a successful transfer. */
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

/* Writes one MAX31865 register through SPI DMA. */
max31865_status_t Max31865_writeRegister8(Max31865_t *dev,
                                          uint8_t addr,
                                          uint8_t data)
{
    /* Set the write bit in the address and release chip select on failure. */
    if (dev == NULL) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    uint8_t txbuf[2];

    txbuf[0] = (uint8_t)(addr | 0x80U);
    txbuf[1] = data;

    /*
     * Clear stale TX DMA semaphore token before starting the new SPI TX transfer.
     * TX -> DMA2_Stream1 -> max31865_spi_tx_sem
     */
    max31865_clear_dma_semaphore_if_rtos(max31865_spi_tx_sem);

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

/* Reads the MAX31865 fault-status register. */
max31865_status_t Max31865_readFault(Max31865_t *max31865, uint8_t *out_fault)
{
    /* Use the byte-read path for the fault register. */
    return Max31865_readRegister8(max31865, MAX31856_FAULTSTAT_REG, out_fault);
}

/* Clears the MAX31865 latched fault state. */
max31865_status_t Max31865_clearFault(Max31865_t *max31865)
{
    /* Set the configuration fault-clear bit after reading the current value. */
    uint8_t cfg = 0U;
    max31865_status_t status = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG, &cfg);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    cfg &= (uint8_t)~0x2CU;
    cfg |= MAX31856_CONFIG_FAULTSTAT;

    return Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
}

/* Updates the MAX31865 RTD bias setting. */
max31865_status_t Max31865_enableBias(Max31865_t *max31865, uint8_t enable)
{
    /* Preserve the other configuration bits while changing BIAS. */
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

/* Controls the MAX31865 automatic conversion mode. */
max31865_status_t Max31865_autoConvert(Max31865_t *max31865, uint8_t enable)
{
    /* Change MODEAUTO through the configuration register. */
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

/* Selects the MAX31865 RTD wire configuration. */
max31865_status_t Max31865_setWires(Max31865_t *max31865, uint8_t numWires)
{
    /* Set the three-wire bit only for a three-wire probe. */
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

/* Selects the MAX31865 mains-frequency rejection filter. */
max31865_status_t Max31865_setFilter(Max31865_t *max31865, uint8_t filterHz)
{
    /* Choose the 50 Hz bit when that frequency is requested. */
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

/* Starts a MAX31865 one-shot and reads its RTD code. */
max31865_status_t Max31865_readRTD(Max31865_t *max31865, uint16_t *rtd_value)
{
    /* Discard the low status bit from the 16-bit register value. */
    if ((max31865 == NULL) || (rtd_value == NULL)) {
        return MAX31865_STATUS_INVALID_ARG;
    }

    max31865_status_t status = Max31865_clearFault(max31865);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("WARN: readRTD clearFault r=%d\r\n", status);
        app_fault_delay_ms(20U);
    }

    status = Max31865_enableBias(max31865, 1U);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("WARN: readRTD enableBias r=%d\r\n", status);
        app_fault_delay_ms(20U);
        return status;
    }

    /*
     * MAX31865 needs bias settling time before a one-shot conversion.
     * Without this, transient raw=0x0000 / 0x7FFF samples can appear.
     */
    app_fault_delay_ms(MAX31865_BIAS_SETTLE_MS);

    uint8_t cfg = 0U;
    status = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG, &cfg);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    cfg |= MAX31856_CONFIG_1SHOT;

    status = Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, cfg);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: readRTD start 1shot failed r=%d\r\n", status);
        app_fault_delay_ms(20U);
        return status;
    }

    app_fault_delay_ms(MAX31865_ONESHOT_CONVERSION_MS);

    status = Max31865_readRegister16(max31865, MAX31856_RTDMSB_REG, rtd_value);
    if (status != MAX31865_STATUS_OK) {
        return status;
    }

    *rtd_value >>= 1U;

    return MAX31865_STATUS_OK;
}

/* Applies the MAX31865 wire and filter settings. */
max31865_status_t Max31865_init(Max31865_t *max31865, uint8_t numwires, uint8_t filterHz)
{
    /* Disable automatic conversion and bias before clearing faults. */
    if (max31865 == NULL) {
        MAX31865_LOG_PRINTF("ERR: init invalid args (max31865 == NULL)\r\n");
        app_fault_delay_ms(20U);
        return MAX31865_STATUS_INVALID_ARG;
    }

    heartbeat_signal_main();
    MPU_CS(CS_DESELECT);

    max31865->wires = numwires;
    max31865->filter_hz = filterHz;

    max31865_status_t status = Max31865_setWires(max31865, numwires);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: init setWires failed r=%d\r\n", status);
        app_fault_delay_ms(20U);
        return status;
    }

    status = Max31865_enableBias(max31865, 0U);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: init enableBias failed r=%d\r\n", status);
        app_fault_delay_ms(20U);
        return status;
    }

    status = Max31865_autoConvert(max31865, 0U);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: init autoConvert failed r=%d\r\n", status);
        app_fault_delay_ms(20U);
        return status;
    }

    status = Max31865_clearFault(max31865);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: init clearFault failed r=%d\r\n", status);
        app_fault_delay_ms(20U);
        return status;
    }

    status = Max31865_setFilter(max31865, filterHz);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: init setFilter failed r=%d\r\n", status);
        app_fault_delay_ms(20U);
        return status;
    }

    MAX31865_LOG_PRINTF("OK: Max31865 initialized (wires=%u filter=%u)\r\n", (unsigned)numwires, (unsigned)filterHz);
    app_fault_delay_ms(1U);
    return MAX31865_STATUS_OK;
}

/* Reports the next MAX31865 recovery attempt. */
static void Max31865_log_recovery_action(int attempt, int max_attempts)
{
    /* Include the current attempt and configured retry limit. */
    MAX31865_LOG_PRINTF("ACT: sample attempt %d/%d failed -> starting SPI/MAX31865 recovery path\r\n",
               attempt, max_attempts);
}

/* --------------------------------------------------------------------------
 * Datasheet-oriented fault logger for 3-wire MAX31865 setups
 * Uses:
 *   - raw 15-bit RTD code after shifting away the LSB fault bit
 *   - fault register bits to print a more human-readable diagnosis
 * -------------------------------------------------------------------------- */
/* Reports MAX31865 fault bits and raw-code diagnostics. */
static void Max31865_log_fault_diagnosis(uint16_t raw, uint8_t fault)
{
    /* Distinguish zero and full-scale RTD codes from valid measurements. */
    const float rt_ohms = (((float)raw) / 32768.0f) * _MAX31865_RREF;

    MAX31865_LOG_PRINTF("DBG: MAX31865 context: raw=0x%04X (%lu), approx_Rt=%.3f ohm, fault=0x%02X\r\n",
               raw, (unsigned long)raw, rt_ohms, (unsigned)fault);
    app_fault_delay_ms(20U);
    if (raw == 0x7FFFU) {
        MAX31865_LOG_PRINTF("DIAG: RTD code is FULL SCALE (32767). Datasheet meaning: saturated-high measurement.\r\n");
        app_fault_delay_ms(20U);
    } else if (raw == 0x0000U) {
        MAX31865_LOG_PRINTF("DIAG: RTD code is NEAR ZERO (0). Datasheet meaning: saturated-low measurement.\r\n");
        app_fault_delay_ms(20U);
    }

    /* No MAX31865 fault bits set -> likely not a clean threshold fault */
    if (fault == 0U) {
        if ((raw == 0x7FFFU) || (raw == 0x0000U)) {
            MAX31865_LOG_PRINTF("DIAG: No MAX31865 fault bits are set. This makes a pure RTD threshold fault less likely.\r\n");
            MAX31865_LOG_PRINTF("DIAG: Suspect SPI/CS/DMA/timing, bias/conversion timing, stale register read, transient power/noise, or intermittent wiring.\r\n");
            app_fault_delay_ms(20U);
        } else {
            MAX31865_LOG_PRINTF("DIAG: No MAX31865 fault bits are set.\r\n");
            app_fault_delay_ms(20U);
        }
        return;
    }

    /* D7: High threshold -> measured resistance > high threshold */
    if ((fault & MAX31865_FAULT_HIGHTHRESH) != 0U) {
        MAX31865_LOG_PRINTF("DIAG: D7 HIGH-THRESHOLD fault: measured RTD resistance is above the programmed high threshold.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Possible causes: open RTD element, RTDIN+ shorted high, FORCE+ shorted high, or FORCE+ not connected to RTD.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Resulting data trend: FULL SCALE.\r\n");
        app_fault_delay_ms(20U);
    }

    /* D6: Low threshold -> measured resistance < low threshold */
    if ((fault & MAX31865_FAULT_LOWTHRESH) != 0U) {
        MAX31865_LOG_PRINTF("DIAG: D6 LOW-THRESHOLD fault: measured RTD resistance is below the programmed low threshold.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Possible causes: RTDIN+ shorted to RTDIN-, RTDIN+ shorted low and not connected to RTD, or FORCE+ shorted low.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Resulting data trend: NEAR ZERO.\r\n");
        app_fault_delay_ms(20U);
    }

    if ((fault & MAX31865_FAULT_REFINLOW) != 0U) {
        MAX31865_LOG_PRINTF("DIAG: D5 fault: VREFIN- > 0.85 x VBIAS.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Possible causes: open RTD element, FORCE+ shorted high and connected to RTD, FORCE+ unconnected, or FORCE+ shorted high and not connected to RTD.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Resulting data trend: FULL SCALE or INDETERMINATE.\r\n");
        app_fault_delay_ms(20U);
    }

    if ((fault & MAX31865_FAULT_REFINHIGH) != 0U) {
        MAX31865_LOG_PRINTF("DIAG: D4 fault: VREFIN- < 0.85 x VBIAS.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Typical cause: FORCE- open / broken return path / low-side wiring issue.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Resulting data trend: may APPEAR VALID even though the measurement path is wrong.\r\n");
        app_fault_delay_ms(20U);
    }

    if ((fault & MAX31865_FAULT_RTDINLOW) != 0U) {
        MAX31865_LOG_PRINTF("DIAG: D3 fault: VRTDIN- < 0.85 x VBIAS.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Typical cause: FORCE- open or RTDIN- low-side wiring issue.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Resulting data trend: NEAR ZERO or deceptively VALID-looking.\r\n");
        app_fault_delay_ms(20U);
    }

    if ((fault & MAX31865_FAULT_OVUV) != 0U) {
        MAX31865_LOG_PRINTF("DIAG: D2 over/under-voltage fault: a protected input went above VDD or below GND.\r\n");
        MAX31865_LOG_PRINTF("DIAG: Resulting data trend: INDETERMINATE. Check supply, grounding, noise, and transients.\r\n");
        app_fault_delay_ms(20U);
    }

    if (raw == 0x7FFFU) {
        if ((fault & (MAX31865_FAULT_HIGHTHRESH | MAX31865_FAULT_REFINLOW)) != 0U) {
            MAX31865_LOG_PRINTF("DIAG: Holistic summary: this sample looks like an OPEN-CIRCUIT / VERY-HIGH-RESISTANCE style fault.\r\n");
            app_fault_delay_ms(20U);
        } else {
            MAX31865_LOG_PRINTF("DIAG: Holistic summary: full-scale data observed; combine the fault bits above with wiring and power checks.\r\n");
            app_fault_delay_ms(20U);
        }
    } else if (raw == 0x0000U) {
        if ((fault & (MAX31865_FAULT_LOWTHRESH | MAX31865_FAULT_RTDINLOW)) != 0U) {
            MAX31865_LOG_PRINTF("DIAG: Holistic summary: this sample looks like a SHORT / VERY-LOW-RESISTANCE style fault.\r\n");
            app_fault_delay_ms(20U);
        } else {
            MAX31865_LOG_PRINTF("DIAG: Holistic summary: near-zero data observed; combine the fault bits above with wiring and power checks.\r\n");
            app_fault_delay_ms(20U);
        }
    } else if ((fault & (MAX31865_FAULT_REFINHIGH | MAX31865_FAULT_RTDINLOW)) != 0U) {
        MAX31865_LOG_PRINTF("DIAG: Holistic summary: data can look numerically plausible, but the analog return path is suspect. Do not trust the sample.\r\n");
        app_fault_delay_ms(20U);
    }
}

/* Reports a MAX31865 runtime failure to fault capture. */
static void app_max31865_log_runtime_error(const char *tag,
                                           int32_t rc,
                                           int attempt,
                                           int max_attempts)
{
    /* Mark this sensor's capture as in progress before logging. */
    system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MAX31865);

    if (rc > 0) {
        MAX31865_LOG_PRINTF("[WARN] %s returned warning code %ld (attempt %d/%d)\r\n",
                            tag ? tag : "MAX31865 runtime",
                            (long)rc,
                            attempt,
                            max_attempts);
    } else {
        MAX31865_LOG_PRINTF("[ERROR] %s failed with code %ld (attempt %d/%d)\r\n",
                            tag ? tag : "MAX31865 runtime",
                            (long)rc,
                            attempt,
                            max_attempts);
    }

    app_fault_delay_ms(20U);
}

/* Reads Celsius temperature with bounded MAX31865 recovery. */
max31865_status_t Max31865_readTempC(Max31865_t *max31865, float *readTemp)
{
    /* Retry failed samples after restoring the SPI and sensor configuration. */
    if ((max31865 == NULL) || (readTemp == NULL)) {
        system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MAX31865);

        MAX31865_LOG_PRINTF("[ERROR] Max31865_readTempC: invalid args\r\n");
        app_fault_delay_ms(20U);

        return MAX31865_STATUS_INVALID_ARG;
    }

    enum { MAX_ATTEMPTS = 3 };

    max31865_status_t final_status = MAX31865_STATUS_INVALID_DATA;

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
        uint16_t raw = 0U;
        uint8_t fault = 0U;
        bool fault_valid = false;
        bool sample_failed = true;
        max31865_status_t status;

        status = Max31865_readRTD(max31865, &raw);
        if (status != MAX31865_STATUS_OK) {
            final_status = status;

            app_max31865_log_runtime_error("Max31865_readRTD",
                                           (int32_t)status,
                                           attempt,
                                           MAX_ATTEMPTS);

            MAX31865_LOG_PRINTF("DIAG: No trustworthy RTD sample was produced. This points more to transaction/control/timing/recovery path issues than to a clean RTD threshold fault.\r\n");
            app_fault_delay_ms(20U);
        } else {
            max31865_status_t fault_status;

            fault_status = Max31865_readFault(max31865, &fault);
            if (fault_status == MAX31865_STATUS_OK) {
                fault_valid = true;
            } else {
                app_max31865_log_runtime_error("Max31865_readFault",
                                               (int32_t)fault_status,
                                               attempt,
                                               MAX_ATTEMPTS);

                MAX31865_LOG_PRINTF("[WARN] Max31865_readTempC: readFault failed r=%d while diagnosing raw=0x%04X\r\n",
                                    (int)fault_status,
                                    raw);
                app_fault_delay_ms(20U);
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
                final_status = MAX31865_STATUS_FAULT;

                app_max31865_log_runtime_error("MAX31865 fault register",
                                               (int32_t)MAX31865_STATUS_FAULT,
                                               attempt,
                                               MAX_ATTEMPTS);

                MAX31865_LOG_PRINTF("[ERROR] Max31865_readTempC: sample looked numerically usable, but MAX31865 reports fault=0x%02X (attempt %d/%d)\r\n",
                                    (unsigned)fault,
                                    attempt,
                                    MAX_ATTEMPTS);
                app_fault_delay_ms(20U);

                Max31865_log_fault_diagnosis(raw, fault);
            } else {
                sample_failed = false;
                *readTemp = temp;

                if (system_fault_logging_active == 0U) {
                    MAX31865_LOG_PRINTF("[MAX31865] OK: TempC = %.4f C (raw=0x%04X, dec=%lu, approx_Rt=%.3f ohm)\r\n",
                                        temp,
                                        raw,
                                        (unsigned long)raw,
                                        (((float)raw) / 32768.0f) * _MAX31865_RREF);
                }

                heartbeat_signal_main();

                system_fault_clear_active(SYSTEM_FAULT_OWNER_MAX31865);

                return MAX31865_STATUS_OK;
            }
        }

        if ((status == MAX31865_STATUS_OK) &&
            (sample_failed != false) &&
            ((raw == 0U) || (raw >= 0x7FFFU))) {

            final_status = MAX31865_STATUS_INVALID_DATA;

            app_max31865_log_local_runtime_error("MAX31865 invalid RTD raw sample",
                                                 (int32_t)MAX31865_STATUS_INVALID_DATA,
                                                 attempt,
                                                 MAX_ATTEMPTS);

            MAX31865_LOG_PRINTF("[ERROR] Max31865_readTempC: invalid RTD value raw=0x%04X (%lu) (attempt %d/%d)\r\n",
                                raw,
                                (unsigned long)raw,
                                attempt,
                                MAX_ATTEMPTS);
            app_fault_delay_ms(20U);

            if (raw == 0x7FFFU) {
                MAX31865_LOG_PRINTF("DIAG: Raw code hit FULL SCALE (32767). In the MAX31865 tables this corresponds to a saturated-high result.\r\n");
                app_fault_delay_ms(20U);
            } else if (raw == 0x0000U) {
                MAX31865_LOG_PRINTF("DIAG: Raw code hit NEAR ZERO (0). In the MAX31865 tables this corresponds to a saturated-low result.\r\n");
                app_fault_delay_ms(20U);
            } else {
                MAX31865_LOG_PRINTF("DIAG: Raw code is outside the accepted window for this routine, so the sample is treated as invalid.\r\n");
                app_fault_delay_ms(20U);
            }

            if (fault_valid) {
                Max31865_log_fault_diagnosis(raw, fault);
            } else {
                MAX31865_LOG_PRINTF("[WARN] Max31865_readTempC: fault register unavailable for this invalid sample; diagnosis limited to raw-code behavior.\r\n");
                app_fault_delay_ms(20U);
            }
        }

        if (attempt >= MAX_ATTEMPTS) {
            system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MAX31865);

            MAX31865_LOG_PRINTF("ACT: MAX31865 sample attempt %d/%d failed -> recovery budget exhausted, no further retry\r\n",
                                attempt,
                                MAX_ATTEMPTS);
            app_fault_delay_ms(20U);

            MAX31865_LOG_PRINTF("[FATAL] MAX31865 failed after retry %d/%d -> reporting fatal to task (last_status=%d)\r\n",
                                attempt,
                                MAX_ATTEMPTS,
                                (int)final_status);
            app_fault_delay_ms(20U);

            return final_status;
        }

        Max31865_log_recovery_action(attempt, MAX_ATTEMPTS);

        if (spi4_recover() == SPI_DMA_STATUS_OK) {
        	heartbeat_signal_main();
            MAX31865_LOG_PRINTF("INFO: Max31865_readTempC: spi4_recover() succeeded, doing full restore/init\r\n");

            if (spi4_restore_and_init() == SPI_DMA_STATUS_OK) {
            	heartbeat_signal_main();
                MAX31865_LOG_PRINTF("INFO: Max31865_readTempC: spi4_restore_and_init() succeeded, reinitializing device\r\n");

                memset(max31865, 0, sizeof(*max31865));

                if (Max31865_init(max31865, 3U, 50U) != MAX31865_STATUS_OK) {
                    system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MAX31865);

                    MAX31865_LOG_PRINTF("[FATAL] MAX31865 init failed after SPI recovery -> reporting fatal to task\r\n");
                    app_fault_delay_ms(20U);

                    return MAX31865_STATUS_INIT_FAILED;
                }
                heartbeat_signal_main();
                MAX31865_LOG_PRINTF("ACT: MAX31865 recovery complete after attempt %d/%d, retrying temperature conversion\r\n",
                                    attempt,
                                    MAX_ATTEMPTS);

                spi4_short_delay_ms(5U);
                continue;
            }

            app_max31865_log_runtime_error("spi4_restore_and_init",
                                           (int32_t)SPI_DMA_STATUS_RECOVERY_FAILED,
                                           attempt,
                                           MAX_ATTEMPTS);

            MAX31865_LOG_PRINTF("[WARN] Max31865_readTempC: spi4_restore_and_init() failed (attempt %d/%d)\r\n",
                                attempt,
                                MAX_ATTEMPTS);
            app_fault_delay_ms(20U);
        } else {
            app_max31865_log_runtime_error("spi4_recover",
                                           (int32_t)SPI_DMA_STATUS_RECOVERY_FAILED,
                                           attempt,
                                           MAX_ATTEMPTS);

            MAX31865_LOG_PRINTF("[WARN] Max31865_readTempC: spi4_recover() failed (attempt %d/%d)\r\n",
                                attempt,
                                MAX_ATTEMPTS);
            app_fault_delay_ms(20U);
        }

        MAX31865_LOG_PRINTF("ACT: MAX31865 short backoff before retry %d/%d\r\n",
                            attempt + 1,
                            MAX_ATTEMPTS);

        spi4_short_delay_ms(10U);
    }

    system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MAX31865);

    MAX31865_LOG_PRINTF("[ERROR] Max31865_readTempC: unexpected fallthrough; returning INVALID_DATA\r\n");
    app_fault_delay_ms(20U);

    return MAX31865_STATUS_INVALID_DATA;
}

/* Reports a local MAX31865 retry failure. */
static void app_max31865_log_local_runtime_error(const char *tag,
                                                 int32_t rc,
                                                 int attempt,
                                                 int max_attempts)
{
    /* Include the attempt count before the diagnostic-drain delay. */
    MAX31865_LOG_PRINTF("[ERROR] %s failed with code %ld (attempt %d/%d)\r\n",
                        tag ? tag : "MAX31865 runtime",
                        (long)rc,
                        attempt,
                        max_attempts);
    app_fault_delay_ms(20U);
}

/* Runs one supervised MAX31865 measurement. */
app_max31865_runtime_status_t app_max31865_runtime_process(void)
{
    /* Clear the sensor fault after a successful temperature read. */
    max31865_status_t max_rc;

    max_rc = Max31865_readTempC(&max31865_dev, &temperatureC);
    if (max_rc == MAX31865_STATUS_OK) {
        heartbeat_signal_main();

        system_fault_clear_active(SYSTEM_FAULT_OWNER_MAX31865);

        return APP_MAX31865_RUNTIME_OK;
    }

    MAX31865_LOG_PRINTF("[ERROR] app_max31865_runtime_process: Max31865_readTempC rc=%d\r\n",
                        (int)max_rc);
    app_fault_delay_ms(20U);

    return APP_MAX31865_RUNTIME_READ_FAIL;
}

/* Reads MAX31865 temperature in Fahrenheit. */
max31865_status_t Max31865_readTempF(Max31865_t *max31865, float *readTemp)
{
    /* Propagate Celsius-read failures before converting the result. */
    if ((max31865 == NULL) || (readTemp == NULL)) {
        MAX31865_LOG_PRINTF("ERR: readTempF invalid args\r\n");
        return MAX31865_STATUS_INVALID_ARG;
    }

    float temp_c = 0.0f;
    max31865_status_t status = Max31865_readTempC(max31865, &temp_c);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: readTempF -> Max31865_readTempC failed\r\n");
        return status;
    }

    *readTemp = (temp_c * 9.0f / 5.0f) + 32.0f;
    return MAX31865_STATUS_OK;
}

/* Applies a weighted filter to an RTD measurement. */
float Max31865_Filter(float newInput, float lastOutput, float effectiveFactor)
{
    /* Blend the new sample with the previous output using the supplied factor. */
    return (lastOutput * (1.0f - effectiveFactor)) + (newInput * effectiveFactor);
}

/* Continuously reports MAX31865 measurements and faults. */
max31865_status_t Max31865_monitorLoop(Max31865_t *max31865, uint32_t interval_ms)
{
    /* Wait the requested interval between sensor reads. */
    if (max31865 == NULL) {
        MAX31865_LOG_PRINTF("ERR: monitorLoop invalid args\r\n");
        return MAX31865_STATUS_INVALID_ARG;
    }

    float tmp = 0.0f;

    while (1) {
        const max31865_status_t status = Max31865_readTempC(max31865, &tmp);
        if (status == MAX31865_STATUS_OK) {
            MAX31865_LOG_PRINTF("MON: Temp = %.3f °C\r\n", tmp);
        } else {
            uint8_t fault = 0U;
            (void)Max31865_readFault(max31865, &fault);
            MAX31865_LOG_PRINTF("MON: Read failed code=%d, fault=0x%02X\r\n", status, fault);
        }

        tim2_delay_ms(interval_ms);
    }
}

/* Reads MAX31865 raw, configuration and fault registers. */
max31865_status_t Max31865_debug_once(Max31865_t *dev)
{
    /* Remove the RTD status bit before calculating diagnostic resistance. */
    if (dev == NULL) {
        MAX31865_LOG_PRINTF("ERR: debug_once invalid arg (dev==NULL)\r\n");
        return MAX31865_STATUS_INVALID_ARG;
    }

    uint8_t raw_buf[2] = {0U, 0U};
    max31865_status_t status = Max31865_readRegisterN(dev, MAX31856_RTDMSB_REG, raw_buf, 2U);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: debug_once read raw16 failed r=%d\r\n", status);
        return status;
    }

    const uint16_t raw16 = ((uint16_t)raw_buf[0] << 8) | raw_buf[1];
    const uint16_t raw   = raw16 >> 1U;

    uint8_t cfg = 0U;
    status = Max31865_readRegisterN(dev, MAX31856_CONFIG_REG, &cfg, 1U);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: debug_once read cfg failed r=%d\r\n", status);
        app_fault_delay_ms(20U);
        return status;
    }

    uint8_t fault = 0U;
    status = Max31865_readRegisterN(dev, MAX31856_FAULTSTAT_REG, &fault, 1U);
    if (status != MAX31865_STATUS_OK) {
        MAX31865_LOG_PRINTF("ERR: debug_once read fault failed r=%d\r\n", status);
        app_fault_delay_ms(20U);
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
