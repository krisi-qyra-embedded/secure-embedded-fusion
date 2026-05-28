/* Author: i fiksuari pas prog
 * Documentation date: 2026-05-28
 */

#include "i2c_dma.h"
#include "tim.h"
#include "uart.h"
#include "pwr.h" /* not used directly here; low-power is correctly applied in the caller wait loops */
/*#include "sig_sensors.h"*/
#include "iwdg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*
 * Multi-sensor I2C DMA driver:
 *
 * BME680 uses I2C1 with DMA:
 *   I2C1_RX -> DMA1 Stream5, Channel 1
 *   I2C1_TX -> DMA1 Stream6, Channel 1
 *
 * MPU9250 uses I2C3 with DMA:
 *   I2C3_RX -> DMA1 Stream1, Channel 1
 *   I2C3_TX -> DMA1 Stream4, Channel 3
 *
 * Pin mapping in this project:
 *   BME680:
 *     I2C1_SCL -> PB8  (AF4)
 *     I2C1_SDA -> PB9  (AF4)
 *
 *   MPU9250:
 *     I2C3_SCL -> PA8  (AF4)
 *     I2C3_SDA -> PB4  (AF9)
 *
 * IMPORTANT:
 * - bme680g_rx_cmplt and bme680g_tx_cmplt are ONLY modified in:
 *      DMA1_Stream5_IRQHandler (RX complete/error)
 *      DMA1_Stream6_IRQHandler (TX complete/error)
 *
 * - mpu9250g_rx_cmplt and mpu9250g_tx_cmplt are ONLY modified in:
 *      DMA1_Stream1_IRQHandler (RX complete/error)
 *      DMA1_Stream4_IRQHandler (TX complete/error)
 *
 * - All other DMA stream IRQ handlers in this file are intentionally empty.
 *
 * - Centralized I2C timing + CR2(DMAEN/LAST) restore
 * - Recovery restores CR2/CCR/TRISE/PE after SWRST/APB reset
 * - Separate DMA quiesce paths for BME680 and MPU9250
 * - Sensor-specific lightweight reinit + self-test hooks
 * - Strong DMA stream/dir validation per I2C instance
 * - Bounded BTF wait before STOP in TX DMA IRQ
 * - Defensive STOP/SR clear/bus-release helpers
 * - No WFI inside SR polling loops
 */

#define I2C_AF_DEFAULT              (4U)

/*
 * IMPORTANT:
 * I2C timing on STM32F411 is derived from the APB1 peripheral clock (PCLK1),
 * not directly from SYSCLK/HCLK.
 *
 * For a 96 MHz system clock, APB1 must still remain within STM32F411 limits.
 * A practical configuration is:
 *   SYSCLK = 96 MHz
 *   APB1   = 48 MHz  (prescaler /2)
 *
 * Standard-mode I2C timing (~100 kHz):
 *   CCR   = PCLK1 / (2 * FSCL) = 48 MHz / (2 * 100 kHz) = 240
 *   TRISE = FREQ + 1 = 48 + 1 = 49
 */
#define SYSCLK_HZ                   (96000000UL)
#define I2C_PCLK1_HZ                (48000000UL)
#define PERIPH_CLK_MHZ              (I2C_PCLK1_HZ / 1000000UL)
#define I2C_CCR_100KHZ              (I2C_PCLK1_HZ / (2UL * 100000UL))
#define I2C_TRISE_STD               (PERIPH_CLK_MHZ + 1UL)

#if ((PERIPH_CLK_MHZ < 2U) || (PERIPH_CLK_MHZ > 50U))
#error "STM32F411 I2C CR2.FREQ must be programmed from a valid APB1 clock in MHz (2..50)."
#endif

#define SHORT_WAIT_COUNT            (10000U)
#define LONG_WAIT_COUNT             (100000U)
#define BUSY_WAIT_COUNT             (100000U)
#define POST_WAIT_COUNT             (2000U)
#define I2C_WAIT_TIMEOUT_COUNT      (100000U)

#define USE_TIMER_FALLBACK

#define I2C_IRQ_BTF_SPINS           (128U)

volatile i2c_dma_transfer_state_t bme680g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
volatile i2c_dma_transfer_state_t bme680g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;

volatile i2c_dma_transfer_state_t mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
volatile i2c_dma_transfer_state_t mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;

volatile uint8_t system_fault_logging_active = 0U;
volatile uint8_t system_fault_owner = 0U;

/* Routes an I2C fault to its owning sensor. */
static void i2c_dma_mark_fault_owner(I2C_TypeDef *I2Cx)
{
    /* Associate the bus with the corresponding fault-reporting path. */
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        return;
    }

    system_fault_logging_active = 1U;

    if (I2Cx == I2C1) {
        system_fault_owner = SYSTEM_FAULT_OWNER_BME680;
    } else if (I2Cx == I2C3) {
        system_fault_owner = SYSTEM_FAULT_OWNER_MPU9250;
    }
}

/*
 * FreeRTOS binary semaphores used by DMA IRQ handlers.
 *
 * Ownership/mapping:
 *   BME680  I2C1_RX -> DMA1_Stream5 -> bme680_rx_sem
 *   BME680  I2C1_TX -> DMA1_Stream6 -> bme680_tx_sem
 *   MPU9250 I2C3_RX -> DMA1_Stream1 -> mpu9250_rx_sem
 *   MPU9250 I2C3_TX -> DMA1_Stream4 -> mpu9250_tx_sem
 *
 * These semaphores are given only from the DMA IRQ path. The old completion
 * flags are kept for diagnostics and compatibility.
 */
SemaphoreHandle_t bme680_rx_sem  = NULL;
SemaphoreHandle_t bme680_tx_sem  = NULL;
SemaphoreHandle_t mpu9250_rx_sem = NULL;
SemaphoreHandle_t mpu9250_tx_sem = NULL;
SemaphoreHandle_t xSerialSemaphore = NULL;

typedef struct {
    DMA_Stream_TypeDef *stream;
    volatile uint32_t  *ifcr;
    uint32_t            clear_all_mask;
    uint32_t            clear_tc_mask;
    uint32_t            clear_te_mask;
    IRQn_Type           irq;
} dma_stream_hw_t;

/* ----------------------------- Private prototypes ----------------------------- */

static inline void reg_barrier(void);
static inline void short_delay(volatile uint32_t d);

static void gpio_set_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode);
static void gpio_set_af(GPIO_TypeDef *port, uint8_t pin, uint32_t af);
static void gpio_set_otype_od(GPIO_TypeDef *port, uint8_t pin);
static void gpio_set_pullup(GPIO_TypeDef *port, uint8_t pin);
static void gpio_set_highspeed(GPIO_TypeDef *port, uint8_t pin);

static I2C_TypeDef *i2c_instance_from_num(uint8_t i2c_num);
static int  dma1_get_stream_hw(uint8_t dma_stream_num, dma_stream_hw_t *out);

static uint8_t tim2__1s_check_flag(void);
static int wait_reg_mask_state(volatile uint32_t *reg, uint32_t mask, uint8_t want_set, uint32_t max_count);
static int wait_reg_mask_state_long(volatile uint32_t *reg, uint32_t mask, uint8_t want_set);

static int wait_flag_busy_clear(I2C_TypeDef *I2Cx);
static int wait_flag_addr_or_sb(I2C_TypeDef *I2Cx, uint32_t flag_mask);
static int wait_flag_txe_or_btf(I2C_TypeDef *I2Cx, uint32_t flag_mask);

static int wait_dma_stream_disabled(DMA_Stream_TypeDef *stream,
                                    volatile uint32_t *ifcr,
                                    uint32_t en_mask,
                                    uint32_t clear_all_mask);

static void i2c_force_stop_and_clear(I2C_TypeDef *I2Cx);
static void i2c_restore_config(I2C_TypeDef *I2Cx);

//static int i2c_recover(I2C_TypeDef *I2Cx,
//                       volatile uint32_t *status_reg,
//                       uint32_t wait_mask,
//                       uint8_t expect_set,
//                       uint8_t allow_restart);

static int dma1_i2c_transfer_receive(uint8_t dma_stream_num,
                                     I2C_TypeDef *I2Cx,
                                     dma_direction_t dir,
                                     uint8_t *msg,
                                     uint32_t len);

static inline void i2c_irq_wait_btf_short(I2C_TypeDef *I2Cx);
static inline void i2c_dma_give_sem_from_isr(SemaphoreHandle_t sem);

__attribute__((weak)) int i2c1_bme680_device_reinit(void);
__attribute__((weak)) int i2c1_bme680_selftest(void);
__attribute__((weak)) int i2c3_mpu9250_device_reinit(void);
__attribute__((weak)) int i2c3_mpu9250_selftest(void);

/* ----------------------------- Small helpers ----------------------------- */

/* Orders I2C and DMA register updates. */
static inline void reg_barrier(void)
{
    /* Complete memory accesses before allowing subsequent instructions. */
    __DSB();
    __ISB();
}

/* Provides a short register-settling spin delay. */
static inline void short_delay(volatile uint32_t d)
{
    /* Keep the loop count volatile so the NOP delay is retained. */
    while (d--) {
        __asm__ volatile ("nop");
    }
}

/* Creates and clears the I2C DMA completion semaphores. */
int i2c_dma_rtos_sync_init(void)
{
    /* Fail if any RX or TX semaphore could not be allocated. */
    if (bme680_rx_sem == NULL) {
        bme680_rx_sem = xSemaphoreCreateBinary();
    }

    if (bme680_tx_sem == NULL) {
        bme680_tx_sem = xSemaphoreCreateBinary();
    }

    if (mpu9250_rx_sem == NULL) {
        mpu9250_rx_sem = xSemaphoreCreateBinary();
    }

    if (mpu9250_tx_sem == NULL) {
        mpu9250_tx_sem = xSemaphoreCreateBinary();
    }

    if ((bme680_rx_sem == NULL)  ||
        (bme680_tx_sem == NULL)  ||
        (mpu9250_rx_sem == NULL) ||
        (mpu9250_tx_sem == NULL)) {
        return 0;
    }

    (void)xSemaphoreTake(bme680_rx_sem,  0U);
    (void)xSemaphoreTake(bme680_tx_sem,  0U);
    (void)xSemaphoreTake(mpu9250_rx_sem, 0U);
    (void)xSemaphoreTake(mpu9250_tx_sem, 0U);

    return 1;
}

/* Wakes a task waiting for I2C DMA completion. */
static inline void i2c_dma_give_sem_from_isr(SemaphoreHandle_t sem)
{
    /* Request an ISR exit yield when a higher-priority task wakes. */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (sem != NULL) {
        (void)xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Stops the current I2C transaction and clears address state. */
static void i2c_force_stop_and_clear(I2C_TypeDef *I2Cx)
{
    /* Read SR1 and SR2 before disabling acknowledgement. */
    if (I2Cx == NULL) {
        return;
    }

    I2Cx->CR1 |= I2C_CR1_STOP;
    reg_barrier();

    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    I2Cx->CR1 &= ~I2C_CR1_ACK;
    reg_barrier();
}

/* Restores the I2C timing and DMA configuration. */
static void i2c_restore_config(I2C_TypeDef *I2Cx)
{
    /* Disable PE before replacing CR2, CCR and TRISE. */
    if (I2Cx == NULL) {
        return;
    }

    I2Cx->CR1 &= ~I2C_CR1_PE;
    reg_barrier();

    I2Cx->CR2 = (uint32_t)PERIPH_CLK_MHZ;
    I2Cx->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;

    I2Cx->CCR   = (uint32_t)I2C_CCR_100KHZ;
    I2Cx->TRISE = (uint32_t)I2C_TRISE_STD;

    I2Cx->CR1 &= ~I2C_CR1_ACK;

    I2Cx->CR1 |= I2C_CR1_PE;
    reg_barrier();

    (void)I2Cx->SR1;
    (void)I2Cx->SR2;
}

/* Sets the mode of one GPIO pin. */
static void gpio_set_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode)
{
    /* Replace only the selected pin's two MODER bits. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->MODER = (port->MODER & ~(3U << pos)) | ((mode & 3U) << pos);
}

/* Selects a pin's alternate-function mapping. */
static void gpio_set_af(GPIO_TypeDef *port, uint8_t pin, uint32_t af)
{
    /* Choose the AFR bank from the pin number. */
    uint32_t idx = (uint32_t)pin >> 3U;
    uint32_t shift = (uint32_t)(pin & 7U) * 4U;
    port->AFR[idx] = (port->AFR[idx] & ~(0xFU << shift)) | ((af & 0xFU) << shift);
}

/* Selects open-drain output for an I2C pin. */
static void gpio_set_otype_od(GPIO_TypeDef *port, uint8_t pin)
{
    /* Set only the requested OTYPER bit. */
    port->OTYPER |= (1U << pin);
}

/* Enables the pull-up on an I2C pin. */
static void gpio_set_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    /* Replace the pin's two pull-control bits. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->PUPDR = (port->PUPDR & ~(3U << pos)) | (1U << pos);
}

/* Sets the I2C GPIO output speed. */
static void gpio_set_highspeed(GPIO_TypeDef *port, uint8_t pin)
{
    /* Update only the selected pin's speed field. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->OSPEEDR = (port->OSPEEDR & ~(3U << pos)) | (2U << pos);
}

/* Maps a bus number to the I2C peripheral. */
static I2C_TypeDef *i2c_instance_from_num(uint8_t i2c_num)
{
    /* Reject numbers outside the three supported I2C instances. */
    switch (i2c_num) {
        case 1U: return I2C1;
        case 2U: return I2C2;
        case 3U: return I2C3;
        default: return NULL;
    }
}

/* ----------------------------- DMA stream mapping ----------------------------- */

/* Resolves DMA1 stream registers, flags and interrupt routing. */
static int dma1_get_stream_hw(uint8_t dma_stream_num, dma_stream_hw_t *out)
{
    /* Select the matching low or high flag-clear register. */
    if (out == NULL) {
        return 0;
    }

    switch (dma_stream_num) {
        case 1U:
            out->stream = DMA1_Stream1;
            out->ifcr = &DMA1->LIFCR;
            out->clear_all_mask =
                DMA_LIFCR_CFEIF1  |
                DMA_LIFCR_CDMEIF1 |
                DMA_LIFCR_CTEIF1  |
                DMA_LIFCR_CHTIF1  |
                DMA_LIFCR_CTCIF1;
            out->clear_tc_mask = DMA_LIFCR_CTCIF1;
            out->clear_te_mask = DMA_LIFCR_CTEIF1;
            out->irq = DMA1_Stream1_IRQn;
            return 1;

        case 2U:
            out->stream = DMA1_Stream2;
            out->ifcr = &DMA1->LIFCR;
            out->clear_all_mask =
                DMA_LIFCR_CFEIF2  |
                DMA_LIFCR_CDMEIF2 |
                DMA_LIFCR_CTEIF2  |
                DMA_LIFCR_CHTIF2  |
                DMA_LIFCR_CTCIF2;
            out->clear_tc_mask = DMA_LIFCR_CTCIF2;
            out->clear_te_mask = DMA_LIFCR_CTEIF2;
            out->irq = DMA1_Stream2_IRQn;
            return 1;

        case 4U:
            out->stream = DMA1_Stream4;
            out->ifcr = &DMA1->HIFCR;
            out->clear_all_mask =
                DMA_HIFCR_CFEIF4  |
                DMA_HIFCR_CDMEIF4 |
                DMA_HIFCR_CTEIF4  |
                DMA_HIFCR_CHTIF4  |
                DMA_HIFCR_CTCIF4;
            out->clear_tc_mask = DMA_HIFCR_CTCIF4;
            out->clear_te_mask = DMA_HIFCR_CTEIF4;
            out->irq = DMA1_Stream4_IRQn;
            return 1;

        case 5U:
            out->stream = DMA1_Stream5;
            out->ifcr = &DMA1->HIFCR;
            out->clear_all_mask =
                DMA_HIFCR_CFEIF5  |
                DMA_HIFCR_CDMEIF5 |
                DMA_HIFCR_CTEIF5  |
                DMA_HIFCR_CHTIF5  |
                DMA_HIFCR_CTCIF5;
            out->clear_tc_mask = DMA_HIFCR_CTCIF5;
            out->clear_te_mask = DMA_HIFCR_CTEIF5;
            out->irq = DMA1_Stream5_IRQn;
            return 1;

        case 6U:
            out->stream = DMA1_Stream6;
            out->ifcr = &DMA1->HIFCR;
            out->clear_all_mask =
                DMA_HIFCR_CFEIF6  |
                DMA_HIFCR_CDMEIF6 |
                DMA_HIFCR_CTEIF6  |
                DMA_HIFCR_CHTIF6  |
                DMA_HIFCR_CTCIF6;
            out->clear_tc_mask = DMA_HIFCR_CTCIF6;
            out->clear_te_mask = DMA_HIFCR_CTEIF6;
            out->irq = DMA1_Stream6_IRQn;
            return 1;

        case 7U:
            out->stream = DMA1_Stream7;
            out->ifcr = &DMA1->HIFCR;
            out->clear_all_mask =
                DMA_HIFCR_CFEIF7  |
                DMA_HIFCR_CDMEIF7 |
                DMA_HIFCR_CTEIF7  |
                DMA_HIFCR_CHTIF7  |
                DMA_HIFCR_CTCIF7;
            out->clear_tc_mask = DMA_HIFCR_CTCIF7;
            out->clear_te_mask = DMA_HIFCR_CTEIF7;
            out->irq = DMA1_Stream7_IRQn;
            return 1;

        default:
            return 0;
    }
}

/* ----------------------------- Timer fallback helpers ----------------------------- */

/* Consumes the TIM2 one-second completion flag. */
static uint8_t tim2__1s_check_flag(void)
{
    /* Clear the software flag when returning an elapsed interval. */
    if (tim2_1s_flag != 0U) {
        tim2_1s_flag = 0U;
        return 1U;
    }
    return 0U;
}

/* Services the TIM2 timeout interrupt. */
void TIM2_IRQHandler(void)
{
    /* Publish timer completion when an update event is pending. */
    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->DIER &= ~TIM_DIER_UIE;
        tim2_1s_flag = 1U;
    }
}

/* ----------------------------- Generic wait helpers ----------------------------- */

/* Polls register bits within a fixed iteration budget. */
static int wait_reg_mask_state(volatile uint32_t *reg, uint32_t mask, uint8_t want_set, uint32_t max_count)
{
    /* Compare the masked state on every bounded polling pass. */
    if (reg == NULL) {
        return 0;
    }

    while (max_count--) {
        uint8_t is_set = (((*reg) & mask) != 0U) ? 1U : 0U;
        if (is_set == want_set) {
            return 1;
        }
        __asm__ volatile ("nop");
    }

    return 0;
}

/* Waits for a register state using the longer timeout path. */
static int wait_reg_mask_state_long(volatile uint32_t *reg, uint32_t mask, uint8_t want_set)
{
    /* Return only after the requested state or timeout check completes. */
    if (reg == NULL) {
        return 0;
    }

#ifdef USE_TIMER_FALLBACK
    tim2_1s_flag = 0U;
    tim2_1s_start();

    while (tim2__1s_check_flag() == 0U) {
        uint8_t is_set = (((*reg) & mask) != 0U) ? 1U : 0U;
        if (is_set == want_set) {
            tim2_1s_stop();
            return 1;
        }
        __asm__ volatile ("nop");
    }

    tim2_1s_stop();

    /* timer expired -> decide by current state */
    {
        uint8_t is_set = (((*reg) & mask) != 0U) ? 1U : 0U;
        return (is_set == want_set) ? 1 : 0;
    }
#else
    return wait_reg_mask_state(reg, mask, want_set, LONG_WAIT_COUNT);
#endif
}

/* Provides the overridable BME680 reinitialization hook. */
__attribute__((weak)) int i2c1_bme680_device_reinit(void)
{
    /* The default weak hook reports success without touching the sensor. */
    return 0;
}

/* Provides the overridable BME680 recovery self-test. */
__attribute__((weak)) int i2c1_bme680_selftest(void)
{
    /* The default weak hook performs no sensor validation. */
    return 0;
}

/* Provides the overridable MPU9250 reinitialization hook. */
__attribute__((weak)) int i2c3_mpu9250_device_reinit(void)
{
    /* The default weak hook leaves device configuration unchanged. */
    return 0;
}

/* Provides the overridable MPU9250 recovery self-test. */
__attribute__((weak)) int i2c3_mpu9250_selftest(void)
{
    /* A board-specific override can replace the success-only stub. */
    return 0;
}

/* Stops and clears both BME680 DMA streams. */
static void i2c_bme680_dma_quiesce(void)
{
    /* Return RX and TX software states to idle after clearing flags. */
    const uint32_t stream5_clear_mask =
        DMA_HIFCR_CFEIF5  |
        DMA_HIFCR_CDMEIF5 |
        DMA_HIFCR_CTEIF5  |
        DMA_HIFCR_CHTIF5  |
        DMA_HIFCR_CTCIF5;

    const uint32_t stream6_clear_mask =
        DMA_HIFCR_CFEIF6  |
        DMA_HIFCR_CDMEIF6 |
        DMA_HIFCR_CTEIF6  |
        DMA_HIFCR_CHTIF6  |
        DMA_HIFCR_CTCIF6;

    (void)wait_dma_stream_disabled(DMA1_Stream5,
                                   &DMA1->HIFCR,
                                   DMA_SxCR_EN,
                                   stream5_clear_mask);

    (void)wait_dma_stream_disabled(DMA1_Stream6,
                                   &DMA1->HIFCR,
                                   DMA_SxCR_EN,
                                   stream6_clear_mask);

    DMA1->HIFCR = stream5_clear_mask | stream6_clear_mask;
    reg_barrier();

    bme680g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
    bme680g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;
}

/* Stops and clears both MPU9250 DMA streams. */
static void i2c_mpu9250_dma_quiesce(void)
{
    /* Quiesce Stream1 and Stream4 before resetting their completion states. */
    const uint32_t stream1_clear_mask =
        DMA_LIFCR_CFEIF1  |
        DMA_LIFCR_CDMEIF1 |
        DMA_LIFCR_CTEIF1  |
        DMA_LIFCR_CHTIF1  |
        DMA_LIFCR_CTCIF1;

    const uint32_t stream4_clear_mask =
        DMA_HIFCR_CFEIF4  |
        DMA_HIFCR_CDMEIF4 |
        DMA_HIFCR_CTEIF4  |
        DMA_HIFCR_CHTIF4  |
        DMA_HIFCR_CTCIF4;

    (void)wait_dma_stream_disabled(DMA1_Stream1,
                                   &DMA1->LIFCR,
                                   DMA_SxCR_EN,
                                   stream1_clear_mask);

    (void)wait_dma_stream_disabled(DMA1_Stream4,
                                   &DMA1->HIFCR,
                                   DMA_SxCR_EN,
                                   stream4_clear_mask);

    DMA1->LIFCR = stream1_clear_mask;
    DMA1->HIFCR = stream4_clear_mask;
    reg_barrier();

    mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
    mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;
}

/* Pulses the APB reset for the selected I2C peripheral. */
static void i2c_toggle_apb_reset(I2C_TypeDef *I2Cx)
{
    /* Release the reset bit after the peripheral-settling delay. */
    if (I2Cx == I2C1) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        tim2_delay_ms(1);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    } else if (I2Cx == I2C2) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        tim2_delay_ms(1);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    } else if (I2Cx == I2C3) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
        tim2_delay_ms(1);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
    }

    reg_barrier();
}

/* Configures I2C1 and DMA for the BME680. */
i2c_dma_init_status_t i2c_dma_bme680_peripherals_init(void)
{
    /* Use Stream5 for RX and Stream6 for TX on channel 1. */
    i2c_select_status_t rc;
    dma_status_t status;

    rc = i2c_init_select(1U, 8U, 9U);
    if (rc != I2C_SELECT_OK) {
        i2c_dma_mark_fault_owner(I2C1);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][INIT] GPIO/I2C select failed | rc=%d pins=SCL:PB8 SDA:PB9 AF=4\r\n",
                           (int)rc);
        tim2_delay_ms(20U);
        return I2C_DMA_INIT_ERR_I2C_SELECT;
    }

    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    /* 3) DMA1 Stream5 -> I2C1_RX */
    status = dma_stream_init(
        DMA1_Stream5,
        1U,
        DMA_DIR_P2M,
        &DMA1->HIFCR,
        DMA_HIFCR_CFEIF5  |
        DMA_HIFCR_CDMEIF5 |
        DMA_HIFCR_CTEIF5  |
        DMA_HIFCR_CHTIF5  |
        DMA_HIFCR_CTCIF5,
        DMA1_Stream5_IRQn
    );

    if (status != DMA_OK) {
        i2c_dma_mark_fault_owner(I2C1);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][DMA_RX_INIT] DMA stream init failed | stream=DMA1_Stream5 channel=1 rc=%d\r\n",
                           (int)status);
        tim2_delay_ms(20U);
        return I2C_DMA_INIT_ERR_DMA_RX_INIT;
    }

    /* 4) DMA1 Stream6 -> I2C1_TX */
    status = dma_stream_init(
        DMA1_Stream6,
        1U,
        DMA_DIR_M2P,
        &DMA1->HIFCR,
        DMA_HIFCR_CFEIF6  |
        DMA_HIFCR_CDMEIF6 |
        DMA_HIFCR_CTEIF6  |
        DMA_HIFCR_CHTIF6  |
        DMA_HIFCR_CTCIF6,
        DMA1_Stream6_IRQn
    );

    if (status != DMA_OK) {
        i2c_dma_mark_fault_owner(I2C1);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][DMA_TX_INIT] DMA stream init failed | stream=DMA1_Stream6 channel=1 rc=%d\r\n",
                           (int)status);
        tim2_delay_ms(20U);
        return I2C_DMA_INIT_ERR_DMA_TX_INIT;
    }

    fnAPP_Init_Standard_Values();

    return I2C_DMA_INIT_OK;
}

/* Configures I2C3 and DMA for the MPU9250. */
i2c_dma_init_status_t i2c_dma_mpu9250_peripherals_init(void)
{
    /* Use Stream1 channel 1 for RX and Stream4 channel 3 for TX. */
    i2c_select_status_t rc;
    dma_status_t status;

    rc = i2c_init_select(3U, 8U, 4U);
    if (rc != I2C_SELECT_OK) {
        i2c_dma_mark_fault_owner(I2C3);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][INIT] GPIO/I2C select failed | rc=%d pins=SCL:PA8 SDA:PB4 AF_SCL=4 AF_SDA=9\r\n",
                           (int)rc);
        tim2_delay_ms(20U);
        return I2C_DMA_INIT_ERR_I2C_SELECT;
    }

    /* 2) DMA1 Stream1 -> I2C3_RX */
    status = dma_stream_init(
        DMA1_Stream1,
        1U,
        DMA_DIR_P2M,
        &DMA1->LIFCR,
        DMA_LIFCR_CFEIF1  |
        DMA_LIFCR_CDMEIF1 |
        DMA_LIFCR_CTEIF1  |
        DMA_LIFCR_CHTIF1  |
        DMA_LIFCR_CTCIF1,
        DMA1_Stream1_IRQn
    );

    if (status != DMA_OK) {
        i2c_dma_mark_fault_owner(I2C3);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][DMA_RX_INIT] DMA stream init failed | stream=DMA1_Stream1 channel=1 rc=%d\r\n",
                           (int)status);
        tim2_delay_ms(20U);
        return I2C_DMA_INIT_ERR_DMA_RX_INIT;
    }

    /* 3) DMA1 Stream4 -> I2C3_TX */
    status = dma_stream_init(
        DMA1_Stream4,
        3U,
        DMA_DIR_M2P,
        &DMA1->HIFCR,
        DMA_HIFCR_CFEIF4  |
        DMA_HIFCR_CDMEIF4 |
        DMA_HIFCR_CTEIF4  |
        DMA_HIFCR_CHTIF4  |
        DMA_HIFCR_CTCIF4,
        DMA1_Stream4_IRQn
    );

    if (status != DMA_OK) {
        i2c_dma_mark_fault_owner(I2C3);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][DMA_TX_INIT] DMA stream init failed | stream=DMA1_Stream4 channel=3 rc=%d\r\n",
                           (int)status);
        tim2_delay_ms(20U);
        return I2C_DMA_INIT_ERR_DMA_TX_INIT;
    }

    return I2C_DMA_INIT_OK;
}

/* ----------------------------- I2C recovery/waits ----------------------------- */

/* Attempts staged recovery of a stalled I2C transaction. */
i2c_recover_status_t i2c_recover(I2C_TypeDef *I2Cx,
                                 volatile uint32_t *status_reg,
                                 uint32_t wait_mask,
                                 uint8_t expect_set,
                                 uint8_t allow_restart)
{
    /* Quiesce DMA before escalating from STOP to peripheral reset. */
    uint8_t bus_released = 0U;

    if ((I2Cx == NULL) || (status_reg == NULL)) {
        return I2C_RECOVER_FAIL;
    }

    if (I2Cx == I2C1) {
        i2c_bme680_dma_quiesce();
    } else if (I2Cx == I2C3) {
        i2c_mpu9250_dma_quiesce();
    }

    i2c_force_stop_and_clear(I2Cx);
    tim2_delay_ms(2);

    if (wait_reg_mask_state(&I2Cx->SR2, I2C_SR2_BUSY, 0U, BUSY_WAIT_COUNT)) {
        bus_released = 1U;
    } else {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][RECOVER][BUS_RELEASE] STOP/SR clear did not release bus | bus=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
    }

    if ((bus_released != 0U) && (allow_restart != 0U)) {
        I2Cx->CR1 |= I2C_CR1_START;
        reg_barrier();

        if (wait_reg_mask_state(status_reg, wait_mask, expect_set, POST_WAIT_COUNT)) {
            return I2C_RECOVER_OK;
        }

        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][RECOVER][RESTART_CHECK] START after bus-release did not produce expected flag | bus=0x%08lX mask=0x%08lX expect=%u SR1=0x%08lX SR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)wait_mask,
                   (unsigned int)expect_set,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2);
        tim2_delay_ms(20U);
    }

    I2Cx->CR1 |= I2C_CR1_SWRST;
    reg_barrier();
    short_delay(2000U);
    I2Cx->CR1 &= ~I2C_CR1_SWRST;
    reg_barrier();
    short_delay(2000U);

    i2c_restore_config(I2Cx);
    tim2_delay_ms(2);

    if (I2Cx == I2C1) {
        i2c_bme680_dma_quiesce();
    } else if (I2Cx == I2C3) {
        i2c_mpu9250_dma_quiesce();
    }

    if (wait_reg_mask_state(&I2Cx->SR2, I2C_SR2_BUSY, 0U, POST_WAIT_COUNT)) {
    } else {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][RECOVER][SWRST_FAIL] software reset did not release bus; escalating to APB peripheral reset | bus=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX CR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1,
                   (unsigned long)I2Cx->CR2);
        tim2_delay_ms(20U);

        i2c_toggle_apb_reset(I2Cx);

        if (I2Cx == I2C1) {
            i2c_bme680_dma_quiesce();
        } else if (I2Cx == I2C3) {
            i2c_mpu9250_dma_quiesce();
        }

        i2c_force_stop_and_clear(I2Cx);
        i2c_restore_config(I2Cx);
        tim2_delay_ms(2);
    }

    if (I2Cx == I2C1) {
        if (i2c1_bme680_device_reinit() != 0) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][RECOVER] device lightweight reinit failed after bus recovery\r\n");
            tim2_delay_ms(20U);
            return I2C_RECOVER_FAIL;
        }

        if (i2c1_bme680_selftest() != 0) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][RECOVER] post-recovery self-test failed\r\n");
            tim2_delay_ms(20U);
            return I2C_RECOVER_FAIL;
        }

        I2C_DMA_LOG_PRINTF("[INFO][I2C_DMA][I2C1][BME680][RECOVER] lightweight restore completed | DMA=quiesced I2C=config-restored selftest=OK\r\n");
        tim2_delay_ms(20U);
    } else if (I2Cx == I2C3) {
        if (i2c3_mpu9250_device_reinit() != 0) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][RECOVER] device lightweight reinit failed after bus recovery\r\n");
            tim2_delay_ms(20U);
            return I2C_RECOVER_FAIL;
        }

        if (i2c3_mpu9250_selftest() != 0) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][RECOVER] post-recovery self-test failed\r\n");
            tim2_delay_ms(20U);
            return I2C_RECOVER_FAIL;
        }
    }

    if (allow_restart != 0U) {
        I2Cx->CR1 |= I2C_CR1_START;
        reg_barrier();

        if (wait_reg_mask_state(status_reg, wait_mask, expect_set, POST_WAIT_COUNT)) {
            return I2C_RECOVER_OK;
        }

        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][RECOVER][FINAL_CHECK] final START/flag validation failed | bus=0x%08lX mask=0x%08lX expect=%u SR1=0x%08lX SR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)wait_mask,
                   (unsigned int)expect_set,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2);
        tim2_delay_ms(20U);
        return I2C_RECOVER_FAIL;
    }

    return wait_reg_mask_state(status_reg, wait_mask, expect_set, POST_WAIT_COUNT)
           ? I2C_RECOVER_OK
           : I2C_RECOVER_FAIL;
}

/* Waits for the requested I2C transmit state. */
static int wait_flag_txe_or_btf(I2C_TypeDef *I2Cx, uint32_t flag_mask)
{
    /* Use recovery after both short and extended flag waits fail. */
    if (I2Cx == NULL) {
        return 0;
    }

    if (!wait_reg_mask_state(&I2Cx->SR1, flag_mask, 1U, SHORT_WAIT_COUNT)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][TXE_BTF_SHORT] TXE/BTF flag not set in short window | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                           (unsigned long)I2Cx,
                           (unsigned long)flag_mask,
                           (unsigned long)I2Cx->SR1,
                           (unsigned long)I2Cx->SR2,
                           (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR1, flag_mask, 1U)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][TXE_BTF_LONG] TXE/BTF flag still missing after long wait; recovery will be attempted | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                           (unsigned long)I2Cx,
                           (unsigned long)flag_mask,
                           (unsigned long)I2Cx->SR1,
                           (unsigned long)I2Cx->SR2,
                           (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
    }

    if ((I2Cx->SR1 & flag_mask) == 0U) {
        if (!i2c_recover(I2Cx, &I2Cx->SR2, I2C_SR2_BUSY, 0U, 0U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WAIT][TXE_BTF_RECOVER_FAIL] recovery failed while waiting for TXE/BTF | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                               (unsigned long)I2Cx,
                               (unsigned long)flag_mask,
                               (unsigned long)I2Cx->SR1,
                               (unsigned long)I2Cx->SR2,
                               (unsigned long)I2Cx->CR1);
            tim2_delay_ms(20U);
            return 0;
        }

        return ((I2Cx->SR1 & flag_mask) != 0U) ? 1 : 0;
    }

    return 1;
}

/* Waits for the I2C bus to become idle. */
static int wait_flag_busy_clear(I2C_TypeDef *I2Cx)
{
    /* Attempt bus recovery if BUSY remains set after the waits. */
    if (I2Cx == NULL) {
        return 0;
    }

    if (!wait_reg_mask_state(&I2Cx->SR2, I2C_SR2_BUSY, 0U, SHORT_WAIT_COUNT)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][BUSY_SHORT] BUSY did not clear in short window | bus=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR2, I2C_SR2_BUSY, 0U)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][BUSY_LONG] BUSY still set after long wait; recovery will be attempted | bus=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
    }

    if ((I2Cx->SR2 & I2C_SR2_BUSY) != 0U) {
        if (!i2c_recover(I2Cx, &I2Cx->SR2, I2C_SR2_BUSY, 0U, 0U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WAIT][BUSY_RECOVER_FAIL] BUSY recovery failed | bus=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX CR2=0x%08lX\r\n",
                       (unsigned long)I2Cx,
                       (unsigned long)I2Cx->SR1,
                       (unsigned long)I2Cx->SR2,
                       (unsigned long)I2Cx->CR1,
                       (unsigned long)I2Cx->CR2);
            tim2_delay_ms(20U);
            return 0;
        }
    }

    return ((I2Cx->SR2 & I2C_SR2_BUSY) == 0U) ? 1 : 0;
}

/* Waits for I2C address or start-bit completion. */
static int wait_flag_addr_or_sb(I2C_TypeDef *I2Cx, uint32_t flag_mask)
{
    /* Recover the bus when the requested SR1 flag remains absent. */
    if (I2Cx == NULL) {
        return 0;
    }

    if (!wait_reg_mask_state(&I2Cx->SR1, flag_mask, 1U, SHORT_WAIT_COUNT)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][SR1_SHORT] SR1 flag not set in short window | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)flag_mask,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR1, flag_mask, 1U)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][SR1_LONG] SR1 flag still missing after long wait; recovery will be attempted | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)flag_mask,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
    }

    if ((I2Cx->SR1 & flag_mask) == 0U) {
        if (!i2c_recover(I2Cx, &I2Cx->SR2, I2C_SR2_BUSY, 0U, 0U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WAIT][SR1_RECOVER_FAIL] recovery failed while waiting for SR1 flag | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                       (unsigned long)I2Cx,
                       (unsigned long)flag_mask,
                       (unsigned long)I2Cx->SR1,
                       (unsigned long)I2Cx->SR2,
                       (unsigned long)I2Cx->CR1);
            tim2_delay_ms(20U);
            return 0;
        }

        return ((I2Cx->SR1 & flag_mask) != 0U) ? 1 : 0;
    }

    return 1;
}

/* Maps an I2C DMA stream failure to its sensor. */
static void i2c_dma_mark_fault_owner_from_stream(DMA_Stream_TypeDef *stream)
{
    /* Recognize the RX and TX stream pair for each bus. */
    if (stream == NULL) {
        return;
    }

    if ((stream == DMA1_Stream5) || (stream == DMA1_Stream6)) {
        i2c_dma_mark_fault_owner(I2C1);    
    } else if ((stream == DMA1_Stream1) || (stream == DMA1_Stream4)) {
        i2c_dma_mark_fault_owner(I2C3);    
    }
}

/* Waits for a DMA stream to stop before reconfiguration. */
static int wait_dma_stream_disabled(DMA_Stream_TypeDef *stream,
                                    volatile uint32_t *ifcr,
                                    uint32_t en_mask,
                                    uint32_t clear_all_mask)
{
    /* Force disable and clear flags if the initial wait fails. */
    if ((stream == NULL) || (ifcr == NULL)) {
        return 0;
    }

    if (!wait_reg_mask_state(&stream->CR, en_mask, 0U, SHORT_WAIT_COUNT)) {
        i2c_dma_mark_fault_owner_from_stream(stream);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][DMA_WAIT][EN_SHORT] DMA stream EN bit did not clear in short window | stream=0x%08lX CR=0x%08lX NDTR=%lu\r\n",
                   (unsigned long)stream,
                   (unsigned long)stream->CR,
                   (unsigned long)stream->NDTR);
        tim2_delay_ms(20U);
    }

    if (!wait_reg_mask_state_long(&stream->CR, en_mask, 0U)) {
        i2c_dma_mark_fault_owner_from_stream(stream);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][DMA_WAIT][EN_LONG] DMA stream EN bit still set; forcing disable/flag clear | stream=0x%08lX CR=0x%08lX NDTR=%lu\r\n",
                   (unsigned long)stream,
                   (unsigned long)stream->CR,
                   (unsigned long)stream->NDTR);
        tim2_delay_ms(20U);
    }

    if ((stream->CR & en_mask) != 0U) {
        stream->CR &= ~en_mask;
        *ifcr = clear_all_mask;
        reg_barrier();
        short_delay(1000U);
    }

    if ((stream->CR & en_mask) != 0U) {
        stream->CR = 0U;
        *ifcr = clear_all_mask;
        reg_barrier();
        short_delay(2000U);
    }

    return ((stream->CR & en_mask) == 0U) ? 1 : 0;
}

/* ----------------------------- Public init API ----------------------------- */

/* Configures the selected I2C bus and GPIO routing. */
i2c_select_status_t i2c_init_select(uint8_t i2c_num, uint8_t scl_pin, uint8_t sda_pin)
{
    /* Apply the special PA8/PB4 alternate functions for I2C3. */
    GPIO_TypeDef *scl_port = GPIOB;
    GPIO_TypeDef *sda_port = GPIOB;
    uint32_t scl_af = I2C_AF_DEFAULT;
    uint32_t sda_af = I2C_AF_DEFAULT;
    I2C_TypeDef *I2Cx;

    if ((scl_pin > 15U) || (sda_pin > 15U) || (scl_pin == sda_pin)) {
        return I2C_SELECT_ERR_INVALID_ARG;
    }

    I2Cx = i2c_instance_from_num(i2c_num);
    if (I2Cx == NULL) {
        return I2C_SELECT_ERR_INVALID_ARG;
    }

    if ((i2c_num == 3U) && (scl_pin == 8U) && (sda_pin == 4U)) {
        scl_port = GPIOA;
        sda_port = GPIOB;
        scl_af = 4U;
        sda_af = 9U;
    } else {
        if (i2c_num == 2U) {
            if (sda_pin == 3U) {
                sda_port = GPIOB;
                sda_af = 9U;
            }
            if (scl_pin == 10U) {
                scl_port = GPIOB;
                scl_af = I2C_AF_DEFAULT;
            }
        } else if (i2c_num == 1U) {
            if ((scl_pin == 6U) || (scl_pin == 8U) || (scl_pin == 10U)) {
                scl_port = GPIOB;
                scl_af = I2C_AF_DEFAULT;
            }
            if ((sda_pin == 7U) || (sda_pin == 9U) || (sda_pin == 11U)) {
                sda_port = GPIOB;
                sda_af = I2C_AF_DEFAULT;
            }
        }
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    if ((scl_port == GPIOA) || (sda_port == GPIOA)) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
    reg_barrier();
    short_delay(1000U);

    gpio_set_mode(scl_port, scl_pin, 2U);
    gpio_set_af(scl_port, scl_pin, scl_af);
    gpio_set_otype_od(scl_port, scl_pin);
    gpio_set_pullup(scl_port, scl_pin);
    gpio_set_highspeed(scl_port, scl_pin);

    gpio_set_mode(sda_port, sda_pin, 2U);
    gpio_set_af(sda_port, sda_pin, sda_af);
    gpio_set_otype_od(sda_port, sda_pin);
    gpio_set_pullup(sda_port, sda_pin);
    gpio_set_highspeed(sda_port, sda_pin);

    scl_port->BSRR = (1U << scl_pin);
    sda_port->BSRR = (1U << sda_pin);

    if (i2c_num == 1U) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    } else if (i2c_num == 2U) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    } else {
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
    }
    reg_barrier();
    short_delay(1000U);

    I2Cx->CR1 = I2C_CR1_SWRST;
    short_delay(10U);
    I2Cx->CR1 = 0U;
    reg_barrier();

    I2Cx->CR1 &= ~I2C_CR1_NOSTRETCH;
    I2Cx->CR1 &= ~I2C_CR1_ENGC;

    i2c_restore_config(I2Cx);

    if ((I2Cx->CR1 & I2C_CR1_PE) == 0U) {
        return I2C_SELECT_ERR_PE_NOT_SET;
    }

    return I2C_SELECT_OK;
}

/* ----------------------------- DMA init API ----------------------------- */

/* Initializes a DMA1 stream for an I2C transfer direction. */
dma_status_t dma_stream_init(DMA_Stream_TypeDef *stream,
                             uint32_t channel,
                             dma_direction_t dir,
                             volatile uint32_t *flag_reg,
                             uint32_t flag_mask,
                             IRQn_Type irq)
{
    /* Clear pending stream flags before enabling its interrupt. */
    uint32_t cr = 0U;

    if ((stream == NULL) || (flag_reg == NULL) || (channel > 7U)) {
        i2c_dma_mark_fault_owner_from_stream(stream);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_INIT] invalid DMA init argument | stream=0x%08lX flag_reg=0x%08lX channel=%lu\r\n",
                   (unsigned long)stream,
                   (unsigned long)flag_reg,
                   (unsigned long)channel);
        tim2_delay_ms(20U);
        return DMA_ERR_DISABLE_TIMEOUT;
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    reg_barrier();

    stream->CR &= ~DMA_SxCR_EN;
    if (!wait_dma_stream_disabled(stream, flag_reg, DMA_SxCR_EN, flag_mask)) {
        i2c_dma_mark_fault_owner_from_stream(stream);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_INIT] stream disable timeout before configuration | stream=0x%08lX channel=%lu CR=0x%08lX NDTR=%lu\r\n",
                   (unsigned long)stream,
                   (unsigned long)channel,
                   (unsigned long)stream->CR,
                   (unsigned long)stream->NDTR);
        tim2_delay_ms(20U);
        return DMA_ERR_DISABLE_TIMEOUT;
    }

    stream->CR   = 0U;
    stream->NDTR = 0U;
    stream->PAR  = 0U;
    stream->M0AR = 0U;
    stream->FCR  = 0U;

    cr |= ((channel & 0x7U) << 25U);

    if (dir == DMA_DIR_M2P) {
        cr |= DMA_SxCR_DIR_0;
    }

    cr |= DMA_SxCR_MINC;
    cr |= DMA_SxCR_TCIE;
    cr |= DMA_SxCR_PL_1;

    cr &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE | DMA_SxCR_PINC);

    stream->CR = cr;

    *flag_reg = flag_mask;
    reg_barrier();

    NVIC_ClearPendingIRQ(irq);

#ifdef configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_SetPriority(irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

    NVIC_EnableIRQ(irq);

    return DMA_OK;
}

/* ----------------------------- Internal DMA start helper ----------------------------- */

/* Arms the requested DMA1 stream for I2C data movement. */
static int dma1_i2c_transfer_receive(uint8_t dma_stream_num,
                                     I2C_TypeDef *I2Cx,
                                     dma_direction_t dir,
                                     uint8_t *msg,
                                     uint32_t len)
{
    /* Reject a stream that does not match the bus and direction. */
    dma_stream_hw_t hw;

    if ((I2Cx == NULL) || (msg == NULL) || (len == 0U)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_START] invalid transfer argument | bus=0x%08lX stream=%u dir=%u buf=0x%08lX len=%lu\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)dma_stream_num,
                   (unsigned int)dir,
                   (unsigned long)msg,
                   (unsigned long)len);
        tim2_delay_ms(20U);
        return 0;
    }

    if (I2Cx == I2C1) {
        /* BME680 on I2C1:
         *   RX -> DMA1 Stream5
         *   TX -> DMA1 Stream6
         */
        if ((dir == DMA_DIR_P2M) && (dma_stream_num != 5U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][DMA_MAP] invalid RX stream | expected=DMA1_Stream5 actual=Stream%u\r\n",
                       (unsigned int)dma_stream_num);
            tim2_delay_ms(20U);
            return 0;
        }

        if ((dir == DMA_DIR_M2P) && (dma_stream_num != 6U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][DMA_MAP] invalid TX stream | expected=DMA1_Stream6 actual=Stream%u\r\n",
                       (unsigned int)dma_stream_num);
            tim2_delay_ms(20U);
            return 0;
        }

        I2C1->CR2 |= (I2C_CR2_DMAEN | I2C_CR2_LAST);
    } else if (I2Cx == I2C3) {
        /* MPU9250 on I2C3:
         *   RX -> DMA1 Stream1
         *   TX -> DMA1 Stream4
         */
        if ((dir == DMA_DIR_P2M) && (dma_stream_num != 1U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][DMA_MAP] invalid RX stream | expected=DMA1_Stream1 actual=Stream%u\r\n",
                       (unsigned int)dma_stream_num);
            tim2_delay_ms(20U);
            return 0;
        }

        if ((dir == DMA_DIR_M2P) && (dma_stream_num != 4U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][DMA_MAP] invalid TX stream | expected=DMA1_Stream4 actual=Stream%u\r\n",
                       (unsigned int)dma_stream_num);
            tim2_delay_ms(20U);
            return 0;
        }

        I2C3->CR2 |= (I2C_CR2_DMAEN | I2C_CR2_LAST);
    }

    if (!dma1_get_stream_hw(dma_stream_num, &hw)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_START] unsupported DMA1 stream | stream=%u bus=0x%08lX dir=%u len=%lu\r\n",
                   (unsigned int)dma_stream_num,
                   (unsigned long)I2Cx,
                   (unsigned int)dir,
                   (unsigned long)len);
        tim2_delay_ms(20U);
        return 0;
    }

    hw.stream->CR &= ~DMA_SxCR_EN;
    if (!wait_dma_stream_disabled(hw.stream, hw.ifcr, DMA_SxCR_EN, hw.clear_all_mask)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_START] stream disable failed before reprogramming | stream=DMA1_Stream%u CR=0x%08lX NDTR=%lu dir=%u len=%lu\r\n",
                   (unsigned int)dma_stream_num,
                   (unsigned long)hw.stream->CR,
                   (unsigned long)hw.stream->NDTR,
                   (unsigned int)dir,
                   (unsigned long)len);
        tim2_delay_ms(20U);
        return 0;
    }

    if ((I2Cx == I2C1) && (dma_stream_num == 5U) && (dir == DMA_DIR_P2M)) {
        bme680g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
    } else if ((I2Cx == I2C1) && (dma_stream_num == 6U) && (dir == DMA_DIR_M2P)) {
        bme680g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;
    } else if ((I2Cx == I2C3) && (dma_stream_num == 1U) && (dir == DMA_DIR_P2M)) {
        mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
    } else if ((I2Cx == I2C3) && (dma_stream_num == 4U) && (dir == DMA_DIR_M2P)) {
        mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;
    }

    *hw.ifcr = hw.clear_all_mask;

    hw.stream->PAR  = (uint32_t)&(I2Cx->DR);
    hw.stream->M0AR = (uint32_t)msg;
    hw.stream->NDTR = len;

    reg_barrier();
    hw.stream->CR |= DMA_SxCR_EN;
    reg_barrier();

    if ((hw.stream->CR & DMA_SxCR_EN) == 0U) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_START] stream enable verification failed | stream=DMA1_Stream%u CR=0x%08lX NDTR=%lu PAR=0x%08lX M0AR=0x%08lX\r\n",
                   (unsigned int)dma_stream_num,
                   (unsigned long)hw.stream->CR,
                   (unsigned long)hw.stream->NDTR,
                   (unsigned long)hw.stream->PAR,
                   (unsigned long)hw.stream->M0AR);
        tim2_delay_ms(20U);
        return 0;
    }

    return 1;
}

/* ----------------------------- I2C DMA write ----------------------------- */

/* Starts a DMA-backed I2C register write. */
i2c_dma_status_t i2c_dma_write_reg(I2C_TypeDef *I2Cx,
                                   uint8_t slave_addr_7bit,
                                   uint8_t reg_addr,
                                   const uint8_t *data,
                                   uint16_t data_len)
{
    /* Wait for the address phase before handing data to DMA. */
    uint8_t tx_stream_num;

    if (I2Cx == NULL) {
        return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    }

    if ((I2Cx != I2C1) && (I2Cx != I2C2) && (I2Cx != I2C3)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WRITE] unsupported I2C instance | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr_7bit,
                   (unsigned int)reg_addr,
                   (unsigned int)data_len);
        tim2_delay_ms(20U);
        return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    }

    if ((data_len > 0U) && (data == NULL)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WRITE] NULL data pointer with non-zero payload | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr_7bit,
                   (unsigned int)reg_addr,
                   (unsigned int)data_len);
        tim2_delay_ms(20U);
        return I2C_DMA_ERR_NULL_BUFFER;
    }

    if (!wait_flag_busy_clear(I2Cx)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WRITE][BUSY] bus busy before START | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u SR1=0x%08lX SR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr_7bit,
                   (unsigned int)reg_addr,
                   (unsigned int)data_len,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2);
        tim2_delay_ms(20U);
        return I2C_DMA_ERR_BUSY_TIMEOUT;
    }

    I2Cx->CR1 |= I2C_CR1_START;
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_SB)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WRITE][START] SB timeout after START | bus=0x%08lX slave7=0x%02X reg=0x%02X SR1=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr_7bit,
                   (unsigned int)reg_addr,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;

    I2Cx->DR = (uint8_t)((slave_addr_7bit << 1) | 0U);
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_ADDR)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WRITE][ADDR_W] slave did not ACK address | bus=0x%08lX slave7=0x%02X reg=0x%02X SR1=0x%08lX SR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr_7bit,
                   (unsigned int)reg_addr,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    if (!wait_flag_txe_or_btf(I2Cx, I2C_SR1_TXE)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WRITE][TXE_BEFORE_REG] TXE not set before register byte | bus=0x%08lX slave7=0x%02X reg=0x%02X SR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr_7bit,
                   (unsigned int)reg_addr,
                   (unsigned long)I2Cx->SR1);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

    I2Cx->DR = reg_addr;
    if (!wait_flag_txe_or_btf(I2Cx, I2C_SR1_TXE)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WRITE][TXE_AFTER_REG] register byte not accepted by peripheral | bus=0x%08lX slave7=0x%02X reg=0x%02X SR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr_7bit,
                   (unsigned int)reg_addr,
                   (unsigned long)I2Cx->SR1);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

    if (data_len == 0U) {
        (void)wait_flag_txe_or_btf(I2Cx, I2C_SR1_BTF);
        I2Cx->CR1 |= I2C_CR1_STOP;

        if (I2Cx == I2C1) {
            bme680g_tx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        } else if (I2Cx == I2C3) {
            mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        }

        return I2C_DMA_OK;
    }

    if (data_len == 1U) {
        I2Cx->DR = data[0];
        (void)wait_flag_txe_or_btf(I2Cx, I2C_SR1_BTF);
        I2Cx->CR1 |= I2C_CR1_STOP;

        if (I2Cx == I2C1) {
            bme680g_tx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        } else if (I2Cx == I2C3) {
            mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        }

        return I2C_DMA_OK;
    }

    if (I2Cx == I2C1) {
        tx_stream_num = 6U;
    } else if (I2Cx == I2C2) {
        tx_stream_num = 7U;
    } else {
        tx_stream_num = 4U;
    }

    if (!dma1_i2c_transfer_receive(tx_stream_num, I2Cx, DMA_DIR_M2P, (uint8_t *)data, (uint32_t)data_len)) {
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_DMA_START;
    }

    return I2C_DMA_OK;
}

/* ----------------------------- I2C DMA read ----------------------------- */

/* Starts a DMA-backed I2C register read. */
i2c_dma_status_t i2c_dma_read(I2C_TypeDef *I2Cx,
                              uint8_t slave_addr,
                              uint8_t reg_addr,
                              uint8_t *p_read_buff,
                              uint16_t num_of_bytes)
{
    /* Address the register before switching to the read transaction. */
    uint8_t rx_stream_num;

    if (p_read_buff == NULL) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ] NULL RX buffer | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes);
        tim2_delay_ms(20U);
        return I2C_DMA_ERR_NULL_BUFFER;
    }

    if (num_of_bytes == 0U) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ] zero-length RX request rejected | bus=0x%08lX slave7=0x%02X reg=0x%02X\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr);
        tim2_delay_ms(20U);
        return I2C_DMA_ERR_ZERO_LEN;
    }

    if ((I2Cx != I2C1) && (I2Cx != I2C2) && (I2Cx != I2C3)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ] unsupported I2C instance | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes);
        tim2_delay_ms(20U);
        return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    }

    if (!wait_flag_busy_clear(I2Cx)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][BUSY] bus busy before START | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u SR1=0x%08lX SR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2);
        tim2_delay_ms(20U);
        return I2C_DMA_ERR_BUSY_TIMEOUT;
    }

    I2Cx->CR1 |= I2C_CR1_START;
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_SB)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][START] SB timeout during register-address phase | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u SR1=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;

    I2Cx->DR = (uint8_t)((slave_addr << 1) | 0U);
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_ADDR)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][ADDR_W] slave did not ACK write-address phase | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u SR1=0x%08lX SR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    if (!wait_flag_txe_or_btf(I2Cx, I2C_SR1_TXE)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][TXE_BEFORE_REG] TXE not set before register-address write | bus=0x%08lX slave7=0x%02X reg=0x%02X SR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned long)I2Cx->SR1);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

    if (num_of_bytes >= 2U) {
        I2Cx->CR1 |= I2C_CR1_ACK;
    } else {
        I2Cx->CR1 &= ~I2C_CR1_ACK;
    }

    I2Cx->DR = reg_addr;
    if (!wait_flag_txe_or_btf(I2Cx, I2C_SR1_BTF)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][BTF_AFTER_REG] register-address byte did not complete before repeated START | bus=0x%08lX slave7=0x%02X reg=0x%02X SR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned long)I2Cx->SR1);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

    I2Cx->CR1 |= I2C_CR1_START;
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_SB)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][RESTART] SB timeout during repeated START | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u SR1=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->CR1);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;

    I2Cx->DR = (uint8_t)((slave_addr << 1) | 1U);
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_ADDR)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][ADDR_R] slave did not ACK read-address phase | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u SR1=0x%08lX SR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2);
        tim2_delay_ms(20U);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }

    if (num_of_bytes == 1U) {
        I2Cx->CR1 &= ~I2C_CR1_ACK;
        (void)I2Cx->SR1;
        (void)I2Cx->SR2;
        I2Cx->CR1 |= I2C_CR1_STOP;

        if (!wait_reg_mask_state(&I2Cx->SR1, I2C_SR1_RXNE, 1U, I2C_WAIT_TIMEOUT_COUNT)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][RXNE_1B] RXNE timeout during single-byte read | bus=0x%08lX slave7=0x%02X reg=0x%02X SR1=0x%08lX\r\n",
                       (unsigned long)I2Cx,
                       (unsigned int)slave_addr,
                       (unsigned int)reg_addr,
                       (unsigned long)I2Cx->SR1);
            tim2_delay_ms(20U);
            return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
        }

        p_read_buff[0] = (uint8_t)I2Cx->DR;

        if (I2Cx == I2C1) {
            bme680g_rx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        } else if (I2Cx == I2C3) {
            mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        }

        return I2C_DMA_OK;
    }

    if (num_of_bytes == 2U) {
        I2Cx->CR1 &= ~I2C_CR1_ACK;
        (void)I2Cx->SR1;
        (void)I2Cx->SR2;

        if (!wait_reg_mask_state(&I2Cx->SR1, I2C_SR1_BTF, 1U, I2C_WAIT_TIMEOUT_COUNT)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ][BTF_2B] BTF timeout during two-byte read | bus=0x%08lX slave7=0x%02X reg=0x%02X SR1=0x%08lX\r\n",
                       (unsigned long)I2Cx,
                       (unsigned int)slave_addr,
                       (unsigned int)reg_addr,
                       (unsigned long)I2Cx->SR1);
            tim2_delay_ms(20U);
            I2Cx->CR1 |= I2C_CR1_STOP;
            return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
        }

        I2Cx->CR1 |= I2C_CR1_STOP;
        p_read_buff[0] = (uint8_t)I2Cx->DR;
        p_read_buff[1] = (uint8_t)I2Cx->DR;

        if (I2Cx == I2C1) {
            bme680g_rx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        } else if (I2Cx == I2C3) {
            mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        }

        return I2C_DMA_OK;
    }

    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    if (I2Cx == I2C1) {
        rx_stream_num = 5U;
    } else if (I2Cx == I2C2) {
        rx_stream_num = 2U;
    } else {
        rx_stream_num = 1U;
    }

    if (!dma1_i2c_transfer_receive(rx_stream_num, I2Cx, DMA_DIR_P2M, p_read_buff, (uint32_t)num_of_bytes)) {
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_DMA_START;
    }

    return I2C_DMA_OK;
}

/* ----------------------------- DMA IRQ handlers ----------------------------- */

/* Checks I2C byte-transfer completion inside the interrupt. */
static inline void i2c_irq_wait_btf_short(I2C_TypeDef *I2Cx)
{
    /* Keep the BTF wait bounded to avoid a prolonged ISR. */
    /* bounded wait only (IRQ safe); helps ensure last byte shifted before STOP on TX */
    for (uint32_t i = 0U; i < I2C_IRQ_BTF_SPINS; ++i) {
        if ((I2Cx->SR1 & I2C_SR1_BTF) != 0U) {
            break;
        }
        __asm__ volatile ("nop");
    }
}

/* Services the BME680 I2C1 transmit DMA interrupt. */
void DMA1_Stream6_IRQHandler(void) /* I2C1_TX complete/error -> bme680g_tx_cmplt */
{
    /* Acknowledge Stream6 flags before publishing the transfer result. */
    uint32_t hisr = DMA1->HISR;

    if ((hisr & DMA_HISR_TCIF6) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_irq_wait_btf_short(I2C1);

        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_STOP;
        reg_barrier();

        bme680g_tx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        i2c_dma_give_sem_from_isr(bme680_tx_sem);
        return;
    }

    if ((hisr & DMA_HISR_TEIF6) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTEIF6;

        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_force_stop_and_clear(I2C1);

        system_fault_logging_active = 1U;
        system_fault_owner = SYSTEM_FAULT_OWNER_BME680;

        bme680g_tx_cmplt = I2C_DMA_TRANSFER_ERROR;
        i2c_dma_give_sem_from_isr(bme680_tx_sem);

        return;
    }
}

/* Services the BME680 I2C1 receive DMA interrupt. */
void DMA1_Stream5_IRQHandler(void) /* I2C1_RX complete/error -> bme680g_rx_cmplt */
{
    /* Clear Stream5 completion or error flags before returning. */
    uint32_t hisr = DMA1->HISR;

    if ((hisr & DMA_HISR_TCIF5) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;

        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        heartbeat_signal_bme680();
        reg_barrier();

        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_STOP;
        reg_barrier();

        bme680g_rx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        i2c_dma_give_sem_from_isr(bme680_rx_sem);
        return;
    }

    if ((hisr & DMA_HISR_TEIF5) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTEIF5;

        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_force_stop_and_clear(I2C1);

        system_fault_logging_active = 1U;
        system_fault_owner = SYSTEM_FAULT_OWNER_BME680;

        bme680g_rx_cmplt = I2C_DMA_TRANSFER_ERROR;
        i2c_dma_give_sem_from_isr(bme680_rx_sem);

        return;
    }
}

/* Services the MPU9250 I2C3 receive DMA interrupt. */
void DMA1_Stream1_IRQHandler(void) /* I2C3_RX complete/error -> mpu9250g_rx_cmplt */
{
    /* Publish the RX result from the Stream1 interrupt flags. */
    uint32_t lisr = DMA1->LISR;

    if ((lisr & DMA_LISR_TCIF1) != 0U) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;

        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        heartbeat_signal_mpu9250();
        reg_barrier();

        I2C3->CR1 &= ~I2C_CR1_ACK;
        I2C3->CR1 |= I2C_CR1_STOP;
        reg_barrier();

        mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        i2c_dma_give_sem_from_isr(mpu9250_rx_sem);
        return;
    }

    if ((lisr & DMA_LISR_TEIF1) != 0U) {
        DMA1->LIFCR = DMA_LIFCR_CTEIF1;

        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_force_stop_and_clear(I2C3);

        system_fault_logging_active = 1U;
        system_fault_owner = SYSTEM_FAULT_OWNER_MPU9250;

        mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_ERROR;
        i2c_dma_give_sem_from_isr(mpu9250_rx_sem);

        return;
    }
}

/* Services the MPU9250 I2C3 transmit DMA interrupt. */
void DMA1_Stream4_IRQHandler(void) /* I2C3_TX complete/error -> mpu9250g_tx_cmplt */
{
    /* Handle Stream4 completion and transfer errors separately. */
    uint32_t hisr = DMA1->HISR;

    if ((hisr & DMA_HISR_TCIF4) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4;

        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_irq_wait_btf_short(I2C3);

        I2C3->CR1 &= ~I2C_CR1_ACK;
        I2C3->CR1 |= I2C_CR1_STOP;
        reg_barrier();

        mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        i2c_dma_give_sem_from_isr(mpu9250_tx_sem);
        return;
    }

    if ((hisr & DMA_HISR_TEIF4) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTEIF4;

        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_force_stop_and_clear(I2C3);

        system_fault_logging_active = 1U;
        system_fault_owner = SYSTEM_FAULT_OWNER_MPU9250;

        mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_ERROR;
        i2c_dma_give_sem_from_isr(mpu9250_tx_sem);

        return;
    }
}

/* Acknowledges the retained DMA1 Stream7 interrupt path. */
void DMA1_Stream7_IRQHandler(void) { /* Clear the pending stream flags reported by DMA1. */  }
/* Acknowledges the retained DMA1 Stream2 interrupt path. */
void DMA1_Stream2_IRQHandler(void) { /* Use the low DMA1 flag register for Stream2 events. */  }

