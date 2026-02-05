/* mpu9250_cmsis_spi.c MPU9250 access using CMSIS SPI4 + DMA2 implementation (Stream0 RX, Stream1 TX). */
#include "spi_dma.h"  
#include "tim.h"      
#include "uart.h"     
#include "max31865.h"
#include "systick.h"
#include "iwdg.h"

/* @PINOUT PA5 : SPI1_SCK PA6 : SPI1_MISO PA7 : SPI1_MOSI MPU--------------> STM32 VCC - 3.3V GND - GND NCS. */

 
/* int Max31865_clearFault(Max31865_t *max31865); etc. */
/* Update prototypes in max31865.h to match these return types. */

 
static void _log_hex(const char *label, const uint8_t *buf, uint16_t len)
{
    /* Capture the available diagnostic values. */
    log_printf("%s:", label);
    for (uint16_t i = 0; i < len; ++i) {
        log_printf(" %02X", buf[i]);
    }
    log_printf("\r\n");
}

/* Initialize PB6 as output for MPU CS (CMSIS style) and print registers for verification */
int gpio_pb6_cs_init(void)
{
    /* Set up the required peripheral configuration. */
    const uint32_t pin = 6U;

     
#ifdef RCC_AHB1ENR_GPIOBEN
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
#else
    RCC->AHB1ENR |= (1U << 1);  
#endif
    __DSB(); __ISB();

     
     

     
    uint32_t moder_before = GPIOB->MODER;
    GPIOB->MODER &= ~(3U << (pin * 2));
    GPIOB->MODER |=  (1U << (pin * 2));
    __DSB(); __ISB();
     

     
    uint32_t otyper_before = GPIOB->OTYPER;
    GPIOB->OTYPER &= ~(1U << pin);
    __DSB(); __ISB();
    /* log_printf("DBG: GPIOB->OTYPER before=0x%08X after=0x%08X\r\n", (unsigned)otyper_before. */

     
    uint32_t ospeedr_before = GPIOB->OSPEEDR;
    GPIOB->OSPEEDR &= ~(3U << (pin * 2));
    GPIOB->OSPEEDR |=  (1U << (pin * 2));
    __DSB(); __ISB();
    /* log_printf("DBG: GPIOB->OSPEEDR before=0x%08X after=0x%08X\r\n", (unsigned)ospeedr_before. */

     
    uint32_t pupdr_before = GPIOB->PUPDR;
    GPIOB->PUPDR &= ~(3U << (pin * 2));
    __DSB(); __ISB();
    /* log_printf("DBG: GPIOB->PUPDR before=0x%08X after=0x%08X\r\n", (unsigned)pupdr_before. */

     
    uint32_t odr_before = GPIOB->ODR;
    GPIOB->ODR |= (1U << pin);
    __DSB(); __ISB();
    /* log_printf("DBG: GPIOB->ODR before=0x%08X after=0x%08X\r\n", (unsigned)odr_before, (unsigned)GPIOB->ODR); */

    /* Optional: also print IDR to confirm input readback (if loopback or external visible) */
     

    return 0;
}

 
/* Handles the MPU CS operation. */
int MPU_CS(uint8_t state)
{
    /* Check the current status before proceeding. */
    const uint32_t pin = 6U;  

    if (state == CS_SELECT) {
         
        GPIOB->ODR &= ~(1U << pin);
        __DSB(); __ISB();
        return 0;
    }

    if (state == CS_DESELECT) {
         
        GPIOB->ODR |= (1U << pin);
        __DSB(); __ISB();
        return 0;
    }

    return -1;  
}


/* Read N registers (1 + n transaction), DMA full-duplex robust version Returns MAX31865_OK or negative. */
int Max31865_readRegisterN(Max31865_t *max31865, uint8_t addr, uint8_t *buffer, uint8_t n)
{
    /* Check the returned status before accepting the sample. */
    if (!max31865 || !buffer || n == 0) {
        /* log_printf("ERR: readRegisterN invalid arg (max=%p buf=%p n=%u)\r\n", */
        /* (void*)max31865, (void*)buffer, (unsigned)n); */
        return MAX31865_ERR_ARG;
    }

    uint16_t total = (uint16_t)(1 + n);
    if (total > MAX_STACK_TOTAL) {
        /* log_printf("ERR: readRegisterN too large (total=%u > MAX_STACK_TOTAL=%u)\r\n", */
         
        return MAX31865_ERR_SIZE;
    }

    uint8_t txbuf[MAX_STACK_TOTAL];
    uint8_t rxbuf[MAX_STACK_TOTAL];

    addr &= 0x7F;                
    txbuf[0] = addr;
    for (uint16_t i = 1; i < total; ++i) txbuf[i] = 0xFF;   
    memset(rxbuf, 0, total);

     
    MPU_CS(CS_SELECT);

     
    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    /* start full-duplex DMA transfer (RX must be programmed first inside routine) */
    if (spi4_dma_transmit_receive(txbuf, rxbuf, total) != 0) {
        MPU_CS(CS_DESELECT);
        return MAX31865_ERR_DMA;
    }

    /* Wait for both DMA streams to finish (use same timeout) */
    if (spi_dma_stream1_wait_complete(TRANSFER_TIMEOUT_MS) != 0) {
        MPU_CS(CS_DESELECT);
        return MAX31865_ERR_TIMEOUT;
    }
    if (spi_dma_stream0_wait_complete(TRANSFER_TIMEOUT_MS) != 0) {
        MPU_CS(CS_DESELECT);
        return MAX31865_ERR_TIMEOUT;
    }

    /* ensure SPI peripheral finished shifting last bits */
    if (spi4_wait_bsy_clear_ms(TRANSFER_TIMEOUT_MS) != 0) {
        MPU_CS(CS_DESELECT);
        return MAX31865_ERR_TIMEOUT;
    }

     
    for (uint16_t i = 0; i < n; ++i) buffer[i] = rxbuf[1 + i];

    /* optional debug: _log_hex("DBG: readRegN RX", &rxbuf[1], n); */

     
    MPU_CS(CS_DESELECT);
    return MAX31865_OK;
}

/* Read 1 register (8-bit) */
uint8_t Max31865_readRegister8(Max31865_t *max31865, uint8_t addr)
{
    /* Check the returned status before accepting the sample. */
    uint8_t ret = 0;
    int r = Max31865_readRegisterN(max31865, addr, &ret, 1);
    if (r != MAX31865_OK) {
        log_printf("ERR: readReg8 addr=0x%02X failed code=%d\r\n", addr, r);
        return 0;  
    }
    /* log_printf("OK: readReg8 addr=0x%02X => 0x%02X\r\n", addr, ret); */
    return ret;
}

/* Read 16-bit (MSB then LSB) - returns value (caller can't know error code), so log errors internally. */
uint16_t Max31865_readRegister16(Max31865_t *max31865, uint8_t addr)
{
    /* Check the returned status before accepting the sample. */
    uint8_t buffer[2] = {0, 0};
    int r = Max31865_readRegisterN(max31865, addr, buffer, 2);
    if (r != MAX31865_OK) {
        log_printf("ERR: readReg16 addr=0x%02X failed code=%d\r\n", addr, r);
        return 0;
    }
    uint16_t ret = ((uint16_t)buffer[0] << 8) | buffer[1];
    /* log_printf("OK: readReg16 addr=0x%02X => 0x%04X (%u)\r\n", addr, ret, (unsigned)ret); */
    return ret;
}

/* Write 8-bit register (blocking). */
int Max31865_writeRegister8(Max31865_t *dev, uint8_t addr, uint8_t data)
{
    /* Check the transfer result before continuing. */
    if (!dev) return MAX31865_ERR_ARG;

    addr |= 0x80;  
    uint8_t txbuf[2] = { addr, data };

    MPU_CS(CS_SELECT);

    spi4_flush_rx();
    spi4_check_and_clear_ovr();

    if (spi4_dma_transmit(txbuf, 2) != 0) {
        MPU_CS(CS_DESELECT);
        return MAX31865_ERR_DMA;
    }

    if (spi_dma_stream1_wait_complete(TRANSFER_TIMEOUT_MS) != 0) {
        MPU_CS(CS_DESELECT);
        return MAX31865_ERR_TIMEOUT;
    }

    if (spi4_wait_bsy_clear_ms(TRANSFER_TIMEOUT_MS) != 0) {
        MPU_CS(CS_DESELECT);
        return MAX31865_ERR_TIMEOUT;
    }

    MPU_CS(CS_DESELECT);
    /* log_printf("OK: writeReg8 addr=0x%02X <= 0x%02X\r\n", addr, data); */
    return MAX31865_OK;
}

/* Fault helpers & setters — changed to return int and log */
int Max31865_readFault(Max31865_t *max31865, uint8_t *out_fault)
{
    /* Check the returned status before accepting the sample. */
    if (!out_fault) return MAX31865_ERR_ARG;
    uint8_t f = Max31865_readRegister8(max31865, MAX31856_FAULTSTAT_REG);
    *out_fault = f;
    /* log_printf("DBG: faultReg = 0x%02X\r\n", f); */
    return MAX31865_OK;
}

/* Handles the Max31865 clearFault operation. */
int Max31865_clearFault(Max31865_t *max31865)
{
    /* Check the current status before proceeding. */
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (!t && max31865 == NULL) {   }
    t &= ~0x2C;
    t |= MAX31856_CONFIG_FAULTSTAT;
    int r = Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
    if (r == MAX31865_OK) {
        /* log_printf("OK: clearFault wrote config=0x%02X\r\n", t); */
    } else {
        log_printf("ERR: clearFault write failed code=%d\r\n", r);
    }
    return r;
}

/* Updates the enable state for Max31865 enableBias. */
int Max31865_enableBias(Max31865_t *max31865, uint8_t enable)
{
    /* Check the current status before proceeding. */
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (enable) t |= MAX31856_CONFIG_BIAS; else t &= ~MAX31856_CONFIG_BIAS;
    int r = Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
     
    return r;
}

/* Handles the Max31865 autoConvert operation. */
int Max31865_autoConvert(Max31865_t *max31865, uint8_t enable)
{
    /* Check the current status before proceeding. */
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (enable) t |= MAX31856_CONFIG_MODEAUTO; else t &= ~MAX31856_CONFIG_MODEAUTO;
    int r = Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
     
    return r;
}

/* Accesses the requested setting through Max31865 setWires. */
int Max31865_setWires(Max31865_t *max31865, uint8_t numWires)
{
    /* Check the current status before proceeding. */
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (numWires == 3) t |= MAX31856_CONFIG_3WIRE; else t &= ~MAX31856_CONFIG_3WIRE;
    int r = Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
     
    return r;
}

/* Accesses the requested setting through Max31865 setFilter. */
int Max31865_setFilter(Max31865_t *max31865, uint8_t filterHz)
{
    /* Check the current status before proceeding. */
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    if (filterHz == 50) t |= MAX31856_CONFIG_FILT50HZ; else t &= ~MAX31856_CONFIG_FILT50HZ;
    int r = Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
     
    return r;
}

/* Read RTD (returns value); add logging on error paths */
uint16_t Max31865_readRTD(Max31865_t *max31865)
{
    /* Check the returned status before accepting the sample. */
    int r;
    r = Max31865_clearFault(max31865);
    if (r != MAX31865_OK) log_printf("WARN: readRTD clearFault r=%d\r\n", r);

    r = Max31865_enableBias(max31865, 1);
    if (r != MAX31865_OK) log_printf("WARN: readRTD enableBias r=%d\r\n", r);

    tim2_delay_ms(10);
    uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
    t |= MAX31856_CONFIG_1SHOT;
    r = Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
    if (r != MAX31865_OK) {
        log_printf("ERR: readRTD start 1shot failed r=%d\r\n", r);
    }

    tim2_delay_ms(65);
    uint16_t rtd = Max31865_readRegister16(max31865, MAX31856_RTDMSB_REG);
    rtd >>= 1;
    /* log_printf("OK: readRTD raw=0x%04X => %u\r\n", rtd, (unsigned)rtd); */
    return rtd;
}

 
/* Initializes the resources used by Max31865 init. */
int Max31865_init(Max31865_t *max31865, uint8_t numwires, uint8_t filterHz)
{
    /* Set up the required peripheral configuration. */
    if (!max31865) {
        log_printf("ERR: init invalid args (max31865 == NULL)\r\n");
        return MAX31865_ERR_ARG;
    }
    hb_main_ts = 0;
    if (max31865->lock == 1) tim2_delay_ms(1);
    max31865->lock = 1;

     
    MPU_CS(CS_DESELECT);
    tim2_delay_ms(100);

    int r;

     
    r = Max31865_setWires(max31865, numwires);
    if (r != MAX31865_OK) {
        log_printf("ERR: init setWires failed r=%d\r\n", r);
        max31865->lock = 0;
        return -2;
    }

     
    r = Max31865_enableBias(max31865, 0);
    if (r != MAX31865_OK) {
        log_printf("ERR: init enableBias failed r=%d\r\n", r);
        max31865->lock = 0;
        return -3;
    }

     
    r = Max31865_autoConvert(max31865, 0);
    if (r != MAX31865_OK) {
        log_printf("ERR: init autoConvert failed r=%d\r\n", r);
        max31865->lock = 0;
        return -4;
    }

     
    r = Max31865_clearFault(max31865);
    if (r != MAX31865_OK) {
        log_printf("ERR: init clearFault failed r=%d\r\n", r);
        max31865->lock = 0;
        return -5;
    }

     
    r = Max31865_setFilter(max31865, filterHz);
    if (r != MAX31865_OK) {
        log_printf("ERR: init setFilter failed r=%d\r\n", r);
        max31865->lock = 0;
        return -6;
    }

    max31865->lock = 0;
    log_printf("OK: Max31865 initialized (wires=%u filter=%u)\r\n", (unsigned)numwires, (unsigned)filterHz);
    return MAX31865_OK;
}

/* Read temperature in °C (returns true on OK). */
/* Read temperature in °C (returns 0 on OK, negative on error). */
int Max31865_readTempC(Max31865_t *max31865, float *readTemp)
{
    /* Check the returned status before accepting the sample. */
    if (!max31865 || !readTemp) {
        log_printf("ERR: readTempC invalid args\r\n");
        return -1;
    }

     
    const int MAX_ATTEMPTS = 3;
    int attempts = 0;
    start:
    if (max31865->lock == 1) tim2_delay_ms(1);
    max31865->lock = 1;

    int rc = 0;
    float Z1, Z2, Z3, Z4, Rt, temp;

     
    uint16_t raw = Max31865_readRTD(max31865);
    if ((raw == 0) || (raw >= 0x7FFF)) {
    	attempts++;
    	log_printf("ERR: readTempC: readRTD returned invalid value (0 or saturated) (attempt %d/%d)\r\n",attempts, MAX_ATTEMPTS);
    	 
    	        if (attempts > MAX_ATTEMPTS) {
    	            log_printf("ERR: readTempC: invalid RTD after %d attempts -> giving up\n\r", attempts);
    	            rc = -2;
    	            goto done;
    	        }
    	         
    	        if (spi4_recover() == 0) {
    	            log_printf("INFO: readTempC: spi4_recover() succeeded, attempting full restore/init (attempt %d)\r\n",attempts);

    	        if (spi4_restore_and_init() == 0) {
    	            log_printf("INFO: readTempC: spi4_restore_and_init() succeeded, performing MAX31865 init and retrying (attempt %d)\r\n",attempts);
    	            /* 3) Prepare the Max31865 device struct (fill fields expected by your driver) */
    	            memset(&max31865_dev, 0, sizeof(max31865_dev));
    	            max31865_dev.lock = 0;
    	        if (Max31865_init(&max31865_dev, 3, 50) != 0) {
    	            log_printf("FATAL: Max31865_init failed -> escalating to restart ...\n\r");
    	            NVIC_SystemReset();  
    	        }
    	          
    	          spi4_short_delay_ms(5);
    	          goto start;
    	        } else {
    	          log_printf("WARN: readTempC: spi4_restore_and_init() failed (attempt %d)\r\n", attempts);
    	          }
    	          } else
    	          {
    	           log_printf("WARN: readTempC: spi4_recover() failed (attempt %d)\r\n", attempts);
    	          }
    	           
    	           spi4_short_delay_ms(10);
    	           goto start;
    }
     
    Rt = ((float)raw) / 32768.0f;
    Rt *= _MAX31865_RREF;

     
    Z1 = -RTD_A;
    Z2 = RTD_A * RTD_A - (4.0f * RTD_B);
    Z3 = (4.0f * RTD_B) / _MAX31865_RNOMINAL;
    Z4 = 2.0f * RTD_B;
    temp = Z2 + (Z3 * Rt);

     
    if (temp < 0.0f) {
         
    } else {
        temp = (sqrtf(temp) + Z1) / Z4;

         
        uint8_t fault = 0;
        Max31865_readFault(max31865, &fault);
        if (fault != 0) {
            log_printf("ERR: TempC fault detected fault=0x%02X (raw=0x%04X)\r\n", fault, raw);
            rc = -3;
            goto done;
        }

        *readTemp = temp;
        log_printf("OK: TempC = %.4f °C (raw=0x%04X)\r\n", temp, raw);
        heartbeat_signal_main();
        rc = 0;
        goto done;
    }

     
    {
        Rt /= _MAX31865_RNOMINAL;
        Rt *= 100.0f;
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

    {
        uint8_t fault = 0;
        Max31865_readFault(max31865, &fault);
        if (fault != 0) {
            log_printf("ERR: TempC fault detected (neg poly) fault=0x%02X (raw=0x%04X)\r\n", fault, raw);
            rc = -3;
            goto done;
        }
    }

    *readTemp = temp;
    log_printf("OK: TempC = %.4f °C (raw=0x%04X)\r\n", temp, raw);
    heartbeat_signal_main();
    rc = 0;

done:
    max31865->lock = 0;
    return rc;
}

 
/* Reads data through Max31865 readTempF and returns the operation status. */
int Max31865_readTempF(Max31865_t *max31865, float *readTemp)
{
    /* Check the returned status before accepting the sample. */
    if (!max31865 || !readTemp) {
        log_printf("ERR: readTempF invalid args\r\n");
        return -1;  
    }

    float temp_c = 0.0f;
    if (!Max31865_readTempC(max31865, &temp_c)) {
        log_printf("ERR: readTempF -> Max31865_readTempC failed\r\n");
        return -2;  
    }

    *readTemp = (temp_c * 9.0f / 5.0f) + 32.0f;
    return 0;  
}

 
/* Handles the Max31865 Filter operation. */
float Max31865_Filter(float newInput, float lastOutput, float effectiveFactor)
{
    /* Check the current status before proceeding. */
    return ((float)lastOutput * (1.0f - effectiveFactor)) + ((float)newInput * effectiveFactor);
}

/* Continuous monitor loop (non-blocking caller should run in a task or main loop) - Reads temp and faults. */
int Max31865_monitorLoop(Max31865_t *max31865, uint32_t interval_ms)
{
    /* Check the current status before proceeding. */
    if (!max31865) {
        log_printf("ERR: monitorLoop invalid args\r\n");
        return -1;  
    }

    float tmp;

    /* NOTE: this implementation assumes you have converted Max31865_readTempC to an `int`-returning function. */

    while (1) {
        int rc = Max31865_readTempC(max31865, &tmp);
        if (rc == 0) {
            log_printf("MON: Temp = %.3f °C\r\n", tmp);
        } else {
            uint8_t fault = Max31865_readRegister8(max31865, MAX31856_FAULTSTAT_REG);
            log_printf("MON: Read failed code=%d, fault=0x%02X\r\n", rc, fault);
        }

        tim2_delay_ms(interval_ms);
    }

     
    return 0;
}


/* Logs diagnostic information from Max31865 debug once. */
int Max31865_debug_once(Max31865_t *dev)
{
    /* Capture the available diagnostic values. */
    if (!dev) {
        log_printf("ERR: debug_once invalid arg (dev==NULL)\r\n");
        return -1;
    }

     
    uint8_t raw_buf[2];
    int r = Max31865_readRegisterN(dev, MAX31856_RTDMSB_REG, raw_buf, 2);
    if (r != MAX31865_OK) {
        log_printf("ERR: debug_once read raw16 failed r=%d\r\n", r);
        return -2;
    }
    uint16_t raw16 = ((uint16_t)raw_buf[0] << 8) | raw_buf[1];
    uint16_t raw   = raw16 >> 1;

     
    uint8_t cfg = 0;
    r = Max31865_readRegisterN(dev, MAX31856_CONFIG_REG, &cfg, 1);
    if (r != MAX31865_OK) {
        log_printf("ERR: debug_once read cfg failed r=%d\r\n", r);
        return -3;
    }

     
    uint8_t fault = 0;
    r = Max31865_readRegisterN(dev, MAX31856_FAULTSTAT_REG, &fault, 1);
    if (r != MAX31865_OK) {
        log_printf("ERR: debug_once read fault failed r=%d\r\n", r);
        return -4;
    }

     
    uint64_t Rt_milli = ((uint64_t)raw * (uint64_t)RREF_MILLI_OHMS) / 32768ULL;
    float Rt_f = (float)Rt_milli / 1000.0f;

     
    float Rref_ohm = (float)RREF_MILLI_OHMS / 1000.0f;
    float expected_raw_for_100ohm = (32768.0f * 100.0f) / Rref_ohm;

     
 
/* log_printf("DBG: cfg=0x%02X fault=0x%02X expected_raw_for_100ohm=%.0f\r\n", */
/* (unsigned)cfg, (unsigned)fault, expected_raw_for_100ohm); */

    return 0;
}
/* DMA IRQ Handlers (forward to HAL) Place these in stm32f4xx_it.c or keep here if linked into vector table. */

/* Wait for SPI BSY to clear. */
int spi4_wait_bsy_clear_ms(uint32_t timeout_ms)
{
    /* Keep the wait bounded to avoid blocking indefinitely. */
    if ((SPI4->CR1 & SPI_CR1_SPE) == 0) return -1;  

     
    uint32_t short_loops = 100000U;
    while ((SPI4->SR & SPI_SR_BSY) && short_loops--) __asm__ volatile("nop");
    if ((SPI4->SR & SPI_SR_BSY) == 0) return 0;

    /* fallback to timed wait using your tim2 1s mechanism (coarse ms granularity) */
    tim2_1s_start();
    uint32_t elapsed_ms = 0;
    while ((SPI4->SR & SPI_SR_BSY)) {
        for (volatile uint32_t i = 0; i < 1000; ++i) __asm__ volatile("nop");
        if (tim2_1s_check_flag()) {
            elapsed_ms += 1000;
            if (elapsed_ms >= timeout_ms) {
                tim2_1s_stop();
                return -1;
            }
        }
    }
    tim2_1s_stop();
    return 0;
}



