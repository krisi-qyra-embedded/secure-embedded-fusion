/**
 * @file i2c_dma.c
 * @brief I2C DMA, recovery and shared fault handling.
 */

#include "i2c_dma.h"
#include "tim.h"
#include "uart.h"
#include "pwr.h" /* low-power helpers used by caller paths */
/*#include "sig_sensors.h"*/
#include "iwdg.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Shared fault state is kept here for sensor and ESP32 link faults. */
#ifndef SYSTEM_FAULT_OWNER_PM25
#define SYSTEM_FAULT_OWNER_PM25       4U
#endif

#ifndef SYSTEM_FAULT_MASK_PM25
#define SYSTEM_FAULT_MASK_PM25        (1UL << 3)
#endif

#ifndef SYSTEM_FAULT_OWNER_DS18B20
#define SYSTEM_FAULT_OWNER_DS18B20    5U
#endif

#ifndef SYSTEM_FAULT_MASK_DS18B20
#define SYSTEM_FAULT_MASK_DS18B20     (1UL << 4)
#endif

#ifndef SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM
#define SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM  6U
#endif

#ifndef SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM
#define SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM   (1UL << 5)
#endif

/*
 * BME680: I2C1 PB8/PB9, RX Stream5 CH1, TX Stream6 CH1.
 * MPU9250: I2C3 PA8/PB4, RX Stream1 CH1, TX Stream4 CH3.
 */

#define I2C_AF_DEFAULT              (4U)

/*
 * I2C timing uses APB1 = 48 MHz.
 * Standard mode target is about 100 kHz.
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

/* Short bounded wait used inside IRQ. */
#define I2C_IRQ_BTF_SPINS           (128U)

/* BME680 I2C1 DMA transfer state */
volatile i2c_dma_transfer_state_t bme680g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
volatile i2c_dma_transfer_state_t bme680g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;

/* MPU9250 I2C3 DMA transfer state */
volatile i2c_dma_transfer_state_t mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
volatile i2c_dma_transfer_state_t mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;

/* PM2.5 UART DMA state is handled in pm25_uart.c. */

volatile uint8_t system_fault_logging_active = 0U;
volatile uint8_t system_fault_owner = SYSTEM_FAULT_OWNER_NONE;

volatile uint32_t system_fault_active_mask = 0U;
volatile uint32_t system_fault_completed_mask = 0U;
volatile uint32_t system_fault_in_progress_mask = 0U;

/**
 * @brief Delay during fault handling.
 *
 * @param ms Delay in ms.
 */
 void app_fault_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        tim2_delay_ms(ms);
    }
}

/**
 * @brief Mark a fault owner active and in progress.
 *
 * @param owner Fault owner ID.
 */
 void system_fault_mark_in_progress(uint8_t owner)
 {
     uint32_t mask = system_fault_mask_from_owner(owner);

     if (mask == 0U) {
         return;
     }

     taskENTER_CRITICAL();

     system_fault_logging_active = 1U;
     system_fault_active_mask |= mask;
     system_fault_in_progress_mask |= mask;
     system_fault_owner = owner;

     taskEXIT_CRITICAL();
 }

/* Mark I2C fault from task context. */
 static void i2c_dma_mark_fault_owner_task(I2C_TypeDef *I2Cx)
 {
     if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
         return;
     }

     if (I2Cx == I2C1) {
         system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_BME680);
     } else if (I2Cx == I2C3) {
         system_fault_mark_in_progress(SYSTEM_FAULT_OWNER_MPU9250);
     }
 }

/**
 * @brief Convert fault owner ID to its mask.
 *
 * @param owner Fault owner ID.
 * @return Fault mask, or 0 for invalid owner.
 */
 uint32_t system_fault_mask_from_owner(uint8_t owner)
 {
     switch (owner) {
         case SYSTEM_FAULT_OWNER_BME680:
             return SYSTEM_FAULT_MASK_BME680;

         case SYSTEM_FAULT_OWNER_MPU9250:
             return SYSTEM_FAULT_MASK_MPU9250;

         case SYSTEM_FAULT_OWNER_MAX31865:
             return SYSTEM_FAULT_MASK_MAX31865;

         case SYSTEM_FAULT_OWNER_PM25:
             return SYSTEM_FAULT_MASK_PM25;

         case SYSTEM_FAULT_OWNER_DS18B20:
             return SYSTEM_FAULT_MASK_DS18B20;

         case SYSTEM_FAULT_OWNER_ESP32_UART_AES_GCM:
             return SYSTEM_FAULT_MASK_ESP32_UART_AES_GCM;

         default:
             return 0U;
     }
 }

/**
 * @brief Mark a fault owner active.
 *
 * @param owner Fault owner ID.
 */
void system_fault_mark_active(uint8_t owner)
{
    uint32_t mask = system_fault_mask_from_owner(owner);

    if (mask == 0U) {
        return;
    }

    taskENTER_CRITICAL();
    system_fault_logging_active = 1U;
    system_fault_active_mask |= mask;
    system_fault_owner = owner;   /* last fault owner for debug */
    taskEXIT_CRITICAL();
}

/**
 * @brief Mark fault handling completed.
 *
 * @param owner Fault owner ID.
 */
void system_fault_mark_completed(uint8_t owner)
{
    uint32_t mask = system_fault_mask_from_owner(owner);

    if (mask == 0U) {
        return;
    }

    taskENTER_CRITICAL();

    system_fault_completed_mask |= mask;
    system_fault_in_progress_mask &= ~mask;

    taskEXIT_CRITICAL();
}

/**
 * @brief Clear fault state for one owner.
 *
 * @param owner Fault owner ID.
 */
void system_fault_clear_active(uint8_t owner)
{
    uint32_t mask = system_fault_mask_from_owner(owner);

    if (mask == 0U) {
        return;
    }

    taskENTER_CRITICAL();

    system_fault_active_mask &= ~mask;
    system_fault_in_progress_mask &= ~mask;
    system_fault_completed_mask &= ~mask;

    if (system_fault_active_mask == 0U) {
        system_fault_logging_active = 0U;
        system_fault_owner = SYSTEM_FAULT_OWNER_NONE;
    }

    taskEXIT_CRITICAL();
}

/**
 * @brief Mark a fault owner active from IRQ context.
 *
 * @param owner Fault owner ID.
 */
void system_fault_mark_active_from_isr(uint8_t owner)
{
    uint32_t mask = system_fault_mask_from_owner(owner);

    if (mask == 0U) {
        return;
    }

    system_fault_logging_active = 1U;
    system_fault_active_mask |= mask;
    system_fault_owner = owner;
}

/* Mark fault for the selected I2C bus. */
static void i2c_dma_mark_fault_owner(I2C_TypeDef *I2Cx)
{
    i2c_dma_mark_fault_owner_task(I2Cx);
}

/* DMA completion semaphores for BME680 and MPU9250. */
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

/* Private helpers */

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

/* Flush register writes. */
static inline void reg_barrier(void)
{
    __DSB();
    __ISB();
}

/* Short CPU delay. */
static inline void short_delay(volatile uint32_t d)
{
    while (d--) {
        __asm__ volatile ("nop");
    }
}

/**
 * @brief Create I2C DMA completion semaphores.
 *
 * @return 1 on success, otherwise 0.
 */
int i2c_dma_rtos_sync_init(void)
{
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

    /* Clear stale semaphore state. */
    (void)xSemaphoreTake(bme680_rx_sem,  0U);
    (void)xSemaphoreTake(bme680_tx_sem,  0U);
    (void)xSemaphoreTake(mpu9250_rx_sem, 0U);
    (void)xSemaphoreTake(mpu9250_tx_sem, 0U);

    return 1;
}

/* Give DMA semaphore from IRQ. */
static inline void i2c_dma_give_sem_from_isr(SemaphoreHandle_t sem)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (sem != NULL) {
        (void)xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Force STOP and clear I2C status. */
static void i2c_force_stop_and_clear(I2C_TypeDef *I2Cx)
{
    if (I2Cx == NULL) {
        return;
    }

    /* Try STOP first. */
    I2Cx->CR1 |= I2C_CR1_STOP;
    reg_barrier();

    /* Clear SR read sequence. */
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    /* Keep ACK off while idle. */
    I2Cx->CR1 &= ~I2C_CR1_ACK;
    reg_barrier();
}

/* Restore I2C timing and DMA configuration. */
static void i2c_restore_config(I2C_TypeDef *I2Cx)
{
    if (I2Cx == NULL) {
        return;
    }

    /* Disable peripheral before timing update. */
    I2Cx->CR1 &= ~I2C_CR1_PE;
    reg_barrier();

    /* APB1 frequency and DMA bits. */
    I2Cx->CR2 = (uint32_t)PERIPH_CLK_MHZ;
    I2Cx->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;

    /* Standard mode ~100 kHz. */
    I2Cx->CCR   = (uint32_t)I2C_CCR_100KHZ;
    I2Cx->TRISE = (uint32_t)I2C_TRISE_STD;

    /* ACK off while idle. */
    I2Cx->CR1 &= ~I2C_CR1_ACK;

    /* enable */
    I2Cx->CR1 |= I2C_CR1_PE;
    reg_barrier();

    /* Clear status registers. */
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;
}

/* Set GPIO mode. */
static void gpio_set_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode)
{
    uint32_t pos = (uint32_t)pin * 2U;
    port->MODER = (port->MODER & ~(3U << pos)) | ((mode & 3U) << pos);
}

/* Set GPIO alternate function. */
static void gpio_set_af(GPIO_TypeDef *port, uint8_t pin, uint32_t af)
{
    uint32_t idx = (uint32_t)pin >> 3U;
    uint32_t shift = (uint32_t)(pin & 7U) * 4U;
    port->AFR[idx] = (port->AFR[idx] & ~(0xFU << shift)) | ((af & 0xFU) << shift);
}

/* Set GPIO open-drain output. */
static void gpio_set_otype_od(GPIO_TypeDef *port, uint8_t pin)
{
    port->OTYPER |= (1U << pin);
}

/* Enable GPIO pull-up. */
static void gpio_set_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    uint32_t pos = (uint32_t)pin * 2U;
    port->PUPDR = (port->PUPDR & ~(3U << pos)) | (1U << pos);
}

/* Set GPIO high-speed mode. */
static void gpio_set_highspeed(GPIO_TypeDef *port, uint8_t pin)
{
    uint32_t pos = (uint32_t)pin * 2U;
    port->OSPEEDR = (port->OSPEEDR & ~(3U << pos)) | (2U << pos);
}

static I2C_TypeDef *i2c_instance_from_num(uint8_t i2c_num)
{
    switch (i2c_num) {
        case 1U: return I2C1;
        case 2U: return I2C2;
        case 3U: return I2C3;
        default: return NULL;
    }
}

/* Get DMA1 stream registers and IRQ mapping. */
static int dma1_get_stream_hw(uint8_t dma_stream_num, dma_stream_hw_t *out)
{
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

/* Check and clear TIM2 timeout flag. */
static uint8_t tim2__1s_check_flag(void)
{
    if (tim2_1s_flag != 0U) {
        tim2_1s_flag = 0U;
        return 1U;
    }
    return 0U;
}

/* TIM2 timeout IRQ used by long waits. */
void TIM2_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->DIER &= ~TIM_DIER_UIE;
        tim2_1s_flag = 1U;
    }
}

/* Wait for register mask to reach requested state. */
static int wait_reg_mask_state(volatile uint32_t *reg, uint32_t mask, uint8_t want_set, uint32_t max_count)
{
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

/* Long register wait with timer fallback. */
static int wait_reg_mask_state_long(volatile uint32_t *reg, uint32_t mask, uint8_t want_set)
{
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

    /* Timer expired, check current state. */
    {
        uint8_t is_set = (((*reg) & mask) != 0U) ? 1U : 0U;
        return (is_set == want_set) ? 1 : 0;
    }
#else
    return wait_reg_mask_state(reg, mask, want_set, LONG_WAIT_COUNT);
#endif
}

/* Weak BME680 reinit hook. */
__attribute__((weak)) int i2c1_bme680_device_reinit(void)
{
    return 0;
}

/* Weak BME680 self-test hook. */
__attribute__((weak)) int i2c1_bme680_selftest(void)
{
    return 0;
}

/* Weak MPU9250 reinit hook. */
__attribute__((weak)) int i2c3_mpu9250_device_reinit(void)
{
    return 0;
}

/* Weak MPU9250 self-test hook. */
__attribute__((weak)) int i2c3_mpu9250_selftest(void)
{
    return 0;
}

/* Stop and clear BME680 DMA streams. */
static void i2c_bme680_dma_quiesce(void)
{
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

/* Stop and clear MPU9250 DMA streams. */
static void i2c_mpu9250_dma_quiesce(void)
{
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

/* Reset the selected I2C peripheral through APB1. */
static void i2c_toggle_apb_reset(I2C_TypeDef *I2Cx)
{
    if (I2Cx == I2C1) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        app_fault_delay_ms(1);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    } else if (I2Cx == I2C2) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        app_fault_delay_ms(1);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
    } else if (I2Cx == I2C3) {
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
        app_fault_delay_ms(1);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
    }

    reg_barrier();
}

/**
 * @brief Initialize BME680 I2C1 and DMA streams.
 *
 * @return I2C DMA init status.
 */
i2c_dma_init_status_t i2c_dma_bme680_peripherals_init(void)
{
    i2c_select_status_t rc;
    dma_status_t status;

    /* 1) I2C1 on PB8/PB9 */
    rc = i2c_init_select(1U, 8U, 9U);
    if (rc != I2C_SELECT_OK) {
        i2c_dma_mark_fault_owner(I2C1);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][INIT] GPIO/I2C select failed | rc=%d pins=SCL:PB8 SDA:PB9 AF=4\r\n",
                           (int)rc);
        app_fault_delay_ms(20U);
        return I2C_DMA_INIT_ERR_I2C_SELECT;
    }

    /* 2) Enable FPU (CP10/CP11 full access) */
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
        return I2C_DMA_INIT_ERR_DMA_TX_INIT;
    }

    /* 5) Standard BME defaults */
    fnAPP_Init_Standard_Values();

    return I2C_DMA_INIT_OK;
}

/**
 * @brief Initialize MPU9250 I2C3 and DMA streams.
 *
 * @return I2C DMA init status.
 */
i2c_dma_init_status_t i2c_dma_mpu9250_peripherals_init(void)
{
    i2c_select_status_t rc;
    dma_status_t status;

    /* 1) I2C3 on PA8/PB4 */
    rc = i2c_init_select(3U, 8U, 4U);
    if (rc != I2C_SELECT_OK) {
        i2c_dma_mark_fault_owner(I2C3);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][INIT] GPIO/I2C select failed | rc=%d pins=SCL:PA8 SDA:PB4 AF_SCL=4 AF_SDA=9\r\n",
                           (int)rc);
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
        return I2C_DMA_INIT_ERR_DMA_TX_INIT;
    }

    return I2C_DMA_INIT_OK;
}

/**
 * @brief Recover an I2C bus after a transfer fault.
 *
 * @param I2Cx I2C instance.
 * @param status_reg Register used for final state check.
 * @param wait_mask Flag mask to check.
 * @param expect_set Expected flag state.
 * @param allow_restart Allow START after recovery.
 * @return I2C recovery status.
 */
i2c_recover_status_t i2c_recover(I2C_TypeDef *I2Cx,
                                 volatile uint32_t *status_reg,
                                 uint32_t wait_mask,
                                 uint8_t expect_set,
                                 uint8_t allow_restart)
{
    uint8_t bus_released = 0U;

    if ((I2Cx == NULL) || (status_reg == NULL)) {
        return I2C_RECOVER_FAIL;
    }

    /*
     * Quiesce the DMA streams for the matching sensor path before recovery.
     */
    if (I2Cx == I2C1) {
        i2c_bme680_dma_quiesce();
    } else if (I2Cx == I2C3) {
        i2c_mpu9250_dma_quiesce();
    }

    /* 1) Try graceful bus release first */
    i2c_force_stop_and_clear(I2Cx);
    app_fault_delay_ms(2);

    if (wait_reg_mask_state(&I2Cx->SR2, I2C_SR2_BUSY, 0U, BUSY_WAIT_COUNT)) {
        bus_released = 1U;
    } else {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][RECOVER][BUS_RELEASE] STOP/SR clear did not release bus | bus=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        app_fault_delay_ms(20U);
    }

    /* 2) Optional quick restart path if bus is already usable */
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
        app_fault_delay_ms(20U);
    }

    /*
     * 3) Escalate: peripheral-local software reset.
     */
    I2Cx->CR1 |= I2C_CR1_SWRST;
    reg_barrier();
    short_delay(2000U);
    I2Cx->CR1 &= ~I2C_CR1_SWRST;
    reg_barrier();
    short_delay(2000U);

    i2c_restore_config(I2Cx);
    app_fault_delay_ms(2);

    if (I2Cx == I2C1) {
        i2c_bme680_dma_quiesce();
    } else if (I2Cx == I2C3) {
        i2c_mpu9250_dma_quiesce();
    }

    if (wait_reg_mask_state(&I2Cx->SR2, I2C_SR2_BUSY, 0U, POST_WAIT_COUNT)) {
        /* SWRST released the bus. */
    } else {
        /*
         * 4) Final escalation: APB peripheral reset.
         */
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][RECOVER][SWRST_FAIL] software reset did not release bus; escalating to APB peripheral reset | bus=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX CR2=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1,
                   (unsigned long)I2Cx->CR2);
        app_fault_delay_ms(20U);

        i2c_toggle_apb_reset(I2Cx);

        if (I2Cx == I2C1) {
            i2c_bme680_dma_quiesce();
        } else if (I2Cx == I2C3) {
            i2c_mpu9250_dma_quiesce();
        }

        i2c_force_stop_and_clear(I2Cx);
        i2c_restore_config(I2Cx);
        app_fault_delay_ms(2);
    }

    /*
     * 5) Sensor-specific lightweight reinit + selftest hooks.
     */
    if (I2Cx == I2C1) {
        if (i2c1_bme680_device_reinit() != 0) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][RECOVER] device lightweight reinit failed after bus recovery\r\n");
            app_fault_delay_ms(20U);
            return I2C_RECOVER_FAIL;
        }

        if (i2c1_bme680_selftest() != 0) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][RECOVER] post-recovery self-test failed\r\n");
            app_fault_delay_ms(20U);
            return I2C_RECOVER_FAIL;
        }

        I2C_DMA_LOG_PRINTF("[INFO][I2C_DMA][I2C1][BME680][RECOVER] lightweight restore completed | DMA=quiesced I2C=config-restored selftest=OK\r\n");
        app_fault_delay_ms(20U);
    } else if (I2Cx == I2C3) {
        if (i2c3_mpu9250_device_reinit() != 0) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][RECOVER] device lightweight reinit failed after bus recovery\r\n");
            app_fault_delay_ms(20U);
            return I2C_RECOVER_FAIL;
        }

        if (i2c3_mpu9250_selftest() != 0) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][RECOVER] post-recovery self-test failed\r\n");
            app_fault_delay_ms(20U);
            return I2C_RECOVER_FAIL;
        }
    }

    /* 6) Final restart attempt if requested */
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
        app_fault_delay_ms(20U);
        return I2C_RECOVER_FAIL;
    }

    return wait_reg_mask_state(status_reg, wait_mask, expect_set, POST_WAIT_COUNT)
           ? I2C_RECOVER_OK
           : I2C_RECOVER_FAIL;
}

/* Wait for TXE/BTF and recover bus if needed. */
static int wait_flag_txe_or_btf(I2C_TypeDef *I2Cx, uint32_t flag_mask)
{
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
        app_fault_delay_ms(20U);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR1, flag_mask, 1U)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][TXE_BTF_LONG] TXE/BTF flag still missing after long wait; recovery will be attempted | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                           (unsigned long)I2Cx,
                           (unsigned long)flag_mask,
                           (unsigned long)I2Cx->SR1,
                           (unsigned long)I2Cx->SR2,
                           (unsigned long)I2Cx->CR1);
        app_fault_delay_ms(20U);
    }

    if ((I2Cx->SR1 & flag_mask) == 0U) {
        /* Recover bus only. Caller restarts the transaction. */
        if (!i2c_recover(I2Cx, &I2Cx->SR2, I2C_SR2_BUSY, 0U, 0U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WAIT][TXE_BTF_RECOVER_FAIL] recovery failed while waiting for TXE/BTF | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                               (unsigned long)I2Cx,
                               (unsigned long)flag_mask,
                               (unsigned long)I2Cx->SR1,
                               (unsigned long)I2Cx->SR2,
                               (unsigned long)I2Cx->CR1);
            app_fault_delay_ms(20U);
            return 0;
        }

        return ((I2Cx->SR1 & flag_mask) != 0U) ? 1 : 0;
    }

    return 1;
}

/* Wait for BUSY to clear and recover if needed. */
static int wait_flag_busy_clear(I2C_TypeDef *I2Cx)
{
    if (I2Cx == NULL) {
        return 0;
    }

    if (!wait_reg_mask_state(&I2Cx->SR2, I2C_SR2_BUSY, 0U, SHORT_WAIT_COUNT)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][BUSY_SHORT] BUSY did not clear in short window | bus=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        app_fault_delay_ms(20U);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR2, I2C_SR2_BUSY, 0U)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][BUSY_LONG] BUSY still set after long wait; recovery will be attempted | bus=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        app_fault_delay_ms(20U);
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
            app_fault_delay_ms(20U);
            return 0;
        }
    }

    return ((I2Cx->SR2 & I2C_SR2_BUSY) == 0U) ? 1 : 0;
}

/* Wait for ADDR/SB and recover if needed. */
static int wait_flag_addr_or_sb(I2C_TypeDef *I2Cx, uint32_t flag_mask)
{
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
        app_fault_delay_ms(20U);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR1, flag_mask, 1U)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][WAIT][SR1_LONG] SR1 flag still missing after long wait; recovery will be attempted | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                   (unsigned long)I2Cx,
                   (unsigned long)flag_mask,
                   (unsigned long)I2Cx->SR1,
                   (unsigned long)I2Cx->SR2,
                   (unsigned long)I2Cx->CR1);
        app_fault_delay_ms(20U);
    }

    if ((I2Cx->SR1 & flag_mask) == 0U) {
        /* Recover bus only. Caller restarts ADDR/SB flow. */
        if (!i2c_recover(I2Cx, &I2Cx->SR2, I2C_SR2_BUSY, 0U, 0U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WAIT][SR1_RECOVER_FAIL] recovery failed while waiting for SR1 flag | bus=0x%08lX mask=0x%08lX SR1=0x%08lX SR2=0x%08lX CR1=0x%08lX\r\n",
                       (unsigned long)I2Cx,
                       (unsigned long)flag_mask,
                       (unsigned long)I2Cx->SR1,
                       (unsigned long)I2Cx->SR2,
                       (unsigned long)I2Cx->CR1);
            app_fault_delay_ms(20U);
            return 0;
        }

        return ((I2Cx->SR1 & flag_mask) != 0U) ? 1 : 0;
    }

    return 1;
}

/* Map DMA stream back to its I2C fault owner. */
static void i2c_dma_mark_fault_owner_from_stream(DMA_Stream_TypeDef *stream)
{
    if (stream == NULL) {
        return;
    }

    if ((stream == DMA1_Stream5) || (stream == DMA1_Stream6)) {
        i2c_dma_mark_fault_owner(I2C1);   /* BME680 */
    } else if ((stream == DMA1_Stream1) || (stream == DMA1_Stream4)) {
        i2c_dma_mark_fault_owner(I2C3);   /* MPU9250 */
    }
}

/* Wait until DMA stream is disabled. */
static int wait_dma_stream_disabled(DMA_Stream_TypeDef *stream,
                                    volatile uint32_t *ifcr,
                                    uint32_t en_mask,
                                    uint32_t clear_all_mask)
{
    if ((stream == NULL) || (ifcr == NULL)) {
        return 0;
    }

    if (!wait_reg_mask_state(&stream->CR, en_mask, 0U, SHORT_WAIT_COUNT)) {
        i2c_dma_mark_fault_owner_from_stream(stream);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][DMA_WAIT][EN_SHORT] DMA stream EN bit did not clear in short window | stream=0x%08lX CR=0x%08lX NDTR=%lu\r\n",
                   (unsigned long)stream,
                   (unsigned long)stream->CR,
                   (unsigned long)stream->NDTR);
        app_fault_delay_ms(20U);
    }

    if (!wait_reg_mask_state_long(&stream->CR, en_mask, 0U)) {
        i2c_dma_mark_fault_owner_from_stream(stream);

        I2C_DMA_LOG_PRINTF("[WARN][I2C_DMA][DMA_WAIT][EN_LONG] DMA stream EN bit still set; forcing disable/flag clear | stream=0x%08lX CR=0x%08lX NDTR=%lu\r\n",
                   (unsigned long)stream,
                   (unsigned long)stream->CR,
                   (unsigned long)stream->NDTR);
        app_fault_delay_ms(20U);
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

/**
 * @brief Configure GPIO and I2C instance.
 *
 * @param i2c_num I2C peripheral number.
 * @param scl_pin SCL pin number.
 * @param sda_pin SDA pin number.
 * @return I2C select status.
 */
i2c_select_status_t i2c_init_select(uint8_t i2c_num, uint8_t scl_pin, uint8_t sda_pin)
{
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

    /* SWRST reset then restore full config (expert: ensures consistent timing/DMA bits) */
    I2Cx->CR1 = I2C_CR1_SWRST;
    short_delay(10U);
    I2Cx->CR1 = 0U;
    reg_barrier();

    /* Keep stretching enabled, general call disabled */
    I2Cx->CR1 &= ~I2C_CR1_NOSTRETCH;
    I2Cx->CR1 &= ~I2C_CR1_ENGC;

    /* Restore full timing/DMA configuration */
    i2c_restore_config(I2Cx);

    if ((I2Cx->CR1 & I2C_CR1_PE) == 0U) {
        return I2C_SELECT_ERR_PE_NOT_SET;
    }

    return I2C_SELECT_OK;
}

/**
 * @brief Configure one DMA stream for I2C transfer.
 *
 * @param stream DMA stream.
 * @param channel DMA channel.
 * @param dir Transfer direction.
 * @param flag_reg DMA flag clear register.
 * @param flag_mask Flags to clear.
 * @param irq DMA IRQ number.
 * @return DMA status.
 */
dma_status_t dma_stream_init(DMA_Stream_TypeDef *stream,
                             uint32_t channel,
                             dma_direction_t dir,
                             volatile uint32_t *flag_reg,
                             uint32_t flag_mask,
                             IRQn_Type irq)
{
    uint32_t cr = 0U;

    if ((stream == NULL) || (flag_reg == NULL) || (channel > 7U)) {
        i2c_dma_mark_fault_owner_from_stream(stream);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_INIT] invalid DMA init argument | stream=0x%08lX flag_reg=0x%08lX channel=%lu\r\n",
                   (unsigned long)stream,
                   (unsigned long)flag_reg,
                   (unsigned long)channel);
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
    /* Required because DMA IRQ handlers call xSemaphoreGiveFromISR(). */
    NVIC_SetPriority(irq, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

    NVIC_EnableIRQ(irq);

    return DMA_OK;
}

/* Start one DMA transfer for the selected I2C bus. */
static int dma1_i2c_transfer_receive(uint8_t dma_stream_num,
                                     I2C_TypeDef *I2Cx,
                                     dma_direction_t dir,
                                     uint8_t *msg,
                                     uint32_t len)
{
    dma_stream_hw_t hw;

    if ((I2Cx == NULL) || (msg == NULL) || (len == 0U)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_START] invalid transfer argument | bus=0x%08lX stream=%u dir=%u buf=0x%08lX len=%lu\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)dma_stream_num,
                   (unsigned int)dir,
                   (unsigned long)msg,
                   (unsigned long)len);
        app_fault_delay_ms(20U);
        return 0;
    }

    /* Correctness guards for the canonical stream/dir pairs */
    if (I2Cx == I2C1) {
        /* BME680 uses Stream5 RX and Stream6 TX. */
        if ((dir == DMA_DIR_P2M) && (dma_stream_num != 5U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][DMA_MAP] invalid RX stream | expected=DMA1_Stream5 actual=Stream%u\r\n",
                       (unsigned int)dma_stream_num);
            app_fault_delay_ms(20U);
            return 0;
        }

        if ((dir == DMA_DIR_M2P) && (dma_stream_num != 6U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C1][BME680][DMA_MAP] invalid TX stream | expected=DMA1_Stream6 actual=Stream%u\r\n",
                       (unsigned int)dma_stream_num);
            app_fault_delay_ms(20U);
            return 0;
        }

        /* Defensive: ensure DMA mode bits remain enabled */
        I2C1->CR2 |= (I2C_CR2_DMAEN | I2C_CR2_LAST);
    } else if (I2Cx == I2C3) {
        /* MPU9250 uses Stream1 RX and Stream4 TX. */
        if ((dir == DMA_DIR_P2M) && (dma_stream_num != 1U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][DMA_MAP] invalid RX stream | expected=DMA1_Stream1 actual=Stream%u\r\n",
                       (unsigned int)dma_stream_num);
            app_fault_delay_ms(20U);
            return 0;
        }

        if ((dir == DMA_DIR_M2P) && (dma_stream_num != 4U)) {
            i2c_dma_mark_fault_owner(I2Cx);

            I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][I2C3][MPU9250][DMA_MAP] invalid TX stream | expected=DMA1_Stream4 actual=Stream%u\r\n",
                       (unsigned int)dma_stream_num);
            app_fault_delay_ms(20U);
            return 0;
        }

        /* Defensive: ensure DMA mode bits remain enabled */
        I2C3->CR2 |= (I2C_CR2_DMAEN | I2C_CR2_LAST);
    }

    if (!dma1_get_stream_hw(dma_stream_num, &hw)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][DMA_START] unsupported DMA1 stream | stream=%u bus=0x%08lX dir=%u len=%lu\r\n",
                   (unsigned int)dma_stream_num,
                   (unsigned long)I2Cx,
                   (unsigned int)dir,
                   (unsigned long)len);
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
        return 0;
    }

    /* Only touch the matching sensor DMA completion flags */
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
        app_fault_delay_ms(20U);
        return 0;
    }

    return 1;
}

/**
 * @brief Write register data using I2C DMA.
 *
 * @param I2Cx I2C instance.
 * @param slave_addr_7bit 7-bit slave address.
 * @param reg_addr Register address.
 * @param data Data to write.
 * @param data_len Number of data bytes.
 * @return I2C DMA status.
 */
i2c_dma_status_t i2c_dma_write_reg(I2C_TypeDef *I2Cx,
                                   uint8_t slave_addr_7bit,
                                   uint8_t reg_addr,
                                   const uint8_t *data,
                                   uint16_t data_len)
{
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
        app_fault_delay_ms(20U);
        return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    }

    if ((data_len > 0U) && (data == NULL)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][WRITE] NULL data pointer with non-zero payload | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr_7bit,
                   (unsigned int)reg_addr,
                   (unsigned int)data_len);
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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

/**
 * @brief Read register data using I2C DMA.
 *
 * @param I2Cx I2C instance.
 * @param slave_addr 7-bit slave address.
 * @param reg_addr Register address.
 * @param p_read_buff Read buffer.
 * @param num_of_bytes Number of bytes to read.
 * @return I2C DMA status.
 */
i2c_dma_status_t i2c_dma_read(I2C_TypeDef *I2Cx,
                              uint8_t slave_addr,
                              uint8_t reg_addr,
                              uint8_t *p_read_buff,
                              uint16_t num_of_bytes)
{
    uint8_t rx_stream_num;

    if (p_read_buff == NULL) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ] NULL RX buffer | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes);
        app_fault_delay_ms(20U);
        return I2C_DMA_ERR_NULL_BUFFER;
    }

    if (num_of_bytes == 0U) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ] zero-length RX request rejected | bus=0x%08lX slave7=0x%02X reg=0x%02X\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr);
        app_fault_delay_ms(20U);
        return I2C_DMA_ERR_ZERO_LEN;
    }

    if ((I2Cx != I2C1) && (I2Cx != I2C2) && (I2Cx != I2C3)) {
        i2c_dma_mark_fault_owner(I2Cx);

        I2C_DMA_LOG_PRINTF("[ERROR][I2C_DMA][READ] unsupported I2C instance | bus=0x%08lX slave7=0x%02X reg=0x%02X len=%u\r\n",
                   (unsigned long)I2Cx,
                   (unsigned int)slave_addr,
                   (unsigned int)reg_addr,
                   (unsigned int)num_of_bytes);
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
        app_fault_delay_ms(20U);
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
            app_fault_delay_ms(20U);
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
            app_fault_delay_ms(20U);
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

/* Short BTF wait used in TX DMA IRQ. */
static inline void i2c_irq_wait_btf_short(I2C_TypeDef *I2Cx)
{
    /* bounded wait only (IRQ safe); helps ensure last byte shifted before STOP on TX */
    for (uint32_t i = 0U; i < I2C_IRQ_BTF_SPINS; ++i) {
        if ((I2Cx->SR1 & I2C_SR1_BTF) != 0U) {
            break;
        }
        __asm__ volatile ("nop");
    }
}

/* BME680 I2C1 TX DMA IRQ. */
void DMA1_Stream6_IRQHandler(void)
{
    uint32_t hisr = DMA1->HISR;

    if ((hisr & DMA_HISR_TCIF6) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        /* stop DMA stream */
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        /* expert: bounded wait for BTF before STOP */
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

        /* ISR-safe fault marking: no mutex, no blocking, no delay */
        system_fault_mark_active_from_isr(SYSTEM_FAULT_OWNER_BME680);

        bme680g_tx_cmplt = I2C_DMA_TRANSFER_ERROR;
        i2c_dma_give_sem_from_isr(bme680_tx_sem);

        return;
    }
}

/* BME680 I2C1 RX DMA IRQ. */
void DMA1_Stream5_IRQHandler(void)
{
    uint32_t hisr = DMA1->HISR;

    if ((hisr & DMA_HISR_TCIF5) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;

        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        heartbeat_signal_bme680();
        reg_barrier();

        /* For RX: with CR2.LAST set, DMA handles NACK of last transfer.
           STOP here is standard and keeps IRQ short. */
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

        /* ISR-safe fault marking: no mutex, no blocking, no delay */
        system_fault_mark_active_from_isr(SYSTEM_FAULT_OWNER_BME680);

        bme680g_rx_cmplt = I2C_DMA_TRANSFER_ERROR;
        i2c_dma_give_sem_from_isr(bme680_rx_sem);

        return;
    }
}

/* MPU9250 I2C3 RX DMA IRQ. */
void DMA1_Stream1_IRQHandler(void)
{
    uint32_t lisr = DMA1->LISR;

    if ((lisr & DMA_LISR_TCIF1) != 0U) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;

        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        heartbeat_signal_mpu9250();
        reg_barrier();

        /* For RX: with CR2.LAST set, DMA handles NACK of last transfer.
           STOP here is standard and keeps IRQ short. */
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

        /* ISR-safe fault marking: no mutex, no blocking, no delay */
        system_fault_mark_active_from_isr(SYSTEM_FAULT_OWNER_MPU9250);

        mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_ERROR;
        i2c_dma_give_sem_from_isr(mpu9250_rx_sem);

        return;
    }
}

/* MPU9250 I2C3 TX DMA IRQ. */
void DMA1_Stream4_IRQHandler(void)
{
    uint32_t hisr = DMA1->HISR;

    if ((hisr & DMA_HISR_TCIF4) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4;

        /* stop DMA stream */
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        /* bounded wait for BTF before STOP */
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

        /* ISR-safe fault marking: no mutex, no blocking, no delay */
        system_fault_mark_active_from_isr(SYSTEM_FAULT_OWNER_MPU9250);

        mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_ERROR;
        i2c_dma_give_sem_from_isr(mpu9250_tx_sem);

        return;
    }
}

/* Unused DMA1 Stream7 IRQ. */
void DMA1_Stream7_IRQHandler(void) { }
/* Unused DMA1 Stream2 IRQ. */
void DMA1_Stream2_IRQHandler(void) { }
