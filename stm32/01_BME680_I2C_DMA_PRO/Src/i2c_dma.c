/* adc_dma.c | Author: i fiksuari mbas prog | Date: 28 January 2026 */
#include "i2c_dma.h"
#include "tim.h"
#include "uart.h"
#include "pwr.h" 
#include "sig_sensors.h"
#include "iwdg.h"

/* Multi-sensor I2C DMA driver: BME680 uses I2C1 with DMA: I2C1_RX -> DMA1 Stream5, Channel 1 I2C1_TX ->. */


#define I2C_AF_DEFAULT              (4U)

/* IMPORTANT: I2C timing on STM32F411 is derived from the APB1 peripheral clock (PCLK1), not directly from. */
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

/* very short bounded waits inside IRQ only (keeps IRQ lightweight but improves correctness) */
#define I2C_IRQ_BTF_SPINS           (128U)


volatile i2c_dma_transfer_state_t bme680g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
volatile i2c_dma_transfer_state_t bme680g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;


volatile i2c_dma_transfer_state_t mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_IDLE;
volatile i2c_dma_transfer_state_t mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_IDLE;


typedef struct {
    DMA_Stream_TypeDef *stream;
    volatile uint32_t  *ifcr;
    uint32_t            clear_all_mask;
    uint32_t            clear_tc_mask;
    uint32_t            clear_te_mask;
    IRQn_Type           irq;
} dma_stream_hw_t;



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

/* static int i2c_recover(I2C_TypeDef *I2Cx, */





static int dma1_i2c_transfer_receive(uint8_t dma_stream_num,
                                     I2C_TypeDef *I2Cx,
                                     dma_direction_t dir,
                                     uint8_t *msg,
                                     uint32_t len);

static inline void i2c_irq_wait_btf_short(I2C_TypeDef *I2Cx);

__attribute__((weak)) int i2c1_bme680_device_reinit(void);
__attribute__((weak)) int i2c1_bme680_selftest(void);
__attribute__((weak)) int i2c3_mpu9250_device_reinit(void);
__attribute__((weak)) int i2c3_mpu9250_selftest(void);



/* Ensures the register update completes before execution continues. */
static inline void reg_barrier(void)
{
    /* Check the current status before continuing with the operation. */
    __DSB();
    __ISB();
}

/* Provides the short delay required by the driver timing. */
static inline void short_delay(volatile uint32_t d)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    while (d--) {
        __asm__ volatile ("nop");
    }
}

/* Minimal “release bus / clear sticky state” helper */
static void i2c_force_stop_and_clear(I2C_TypeDef *I2Cx)
{
    /* Check the current status before continuing with the operation. */
    if (I2Cx == NULL) {
        return;
    }

    
    I2Cx->CR1 |= I2C_CR1_STOP;
    reg_barrier();

    
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    /* leave ACK off by default (safe idle) */
    I2Cx->CR1 &= ~I2C_CR1_ACK;
    reg_barrier();
}

/* Restore I2C configuration that is critical for correct operation. */
static void i2c_restore_config(I2C_TypeDef *I2Cx)
{
    /* Configure only the resources required by this module. */
    if (I2Cx == NULL) {
        return;
    }

    
    I2Cx->CR1 &= ~I2C_CR1_PE;
    reg_barrier();

    /* CR2: APB1 freq in MHz + DMAEN + LAST */
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

/* Updates set mode and propagates any transfer error. */
static void gpio_set_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode)
{
    /* Stop immediately if the peripheral reports an error. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->MODER = (port->MODER & ~(3U << pos)) | ((mode & 3U) << pos);
}

/* Updates set af and propagates any transfer error. */
static void gpio_set_af(GPIO_TypeDef *port, uint8_t pin, uint32_t af)
{
    /* Stop immediately if the peripheral reports an error. */
    uint32_t idx = (uint32_t)pin >> 3U;
    uint32_t shift = (uint32_t)(pin & 7U) * 4U;
    port->AFR[idx] = (port->AFR[idx] & ~(0xFU << shift)) | ((af & 0xFU) << shift);
}

/* Updates set otype od and propagates any transfer error. */
static void gpio_set_otype_od(GPIO_TypeDef *port, uint8_t pin)
{
    /* Stop immediately if the peripheral reports an error. */
    port->OTYPER |= (1U << pin);
}

/* Updates set pullup and propagates any transfer error. */
static void gpio_set_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    /* Stop immediately if the peripheral reports an error. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->PUPDR = (port->PUPDR & ~(3U << pos)) | (1U << pos);
}

/* Updates set highspeed and propagates any transfer error. */
static void gpio_set_highspeed(GPIO_TypeDef *port, uint8_t pin)
{
    /* Stop immediately if the peripheral reports an error. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->OSPEEDR = (port->OSPEEDR & ~(3U << pos)) | (2U << pos);
}

/* Handles the instance from num operation and returns the resulting status. */
static I2C_TypeDef *i2c_instance_from_num(uint8_t i2c_num)
{
    /* Check the current status before continuing with the operation. */
    switch (i2c_num) {
        case 1U: return I2C1;
        case 2U: return I2C2;
        case 3U: return I2C3;
        default: return NULL;
    }
}

/* DMA stream mapping */

static int dma1_get_stream_hw(uint8_t dma_stream_num, dma_stream_hw_t *out)
{
    /* Check the current status before continuing with the operation. */
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



/* Validates tim2  1s check flag before continuing with the operation. */
static uint8_t tim2__1s_check_flag(void)
{
    /* Reject invalid state before continuing with the next step. */
    if (tim2_1s_flag != 0U) {
        tim2_1s_flag = 0U;
        return 1U;
    }
    return 0U;
}

/* Handles the interrupt events associated with TIM2 IRQHandler. */
void TIM2_IRQHandler(void)
{
    /* Clear handled flags and update the transfer state. */
    if ((TIM2->SR & TIM_SR_UIF) != 0U) {
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->DIER &= ~TIM_DIER_UIE;
        tim2_1s_flag = 1U;
    }
}



/* Waits for wait reg mask state with a bounded timeout. */
static int wait_reg_mask_state(volatile uint32_t *reg, uint32_t mask, uint8_t want_set, uint32_t max_count)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
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

/* Waits for wait reg mask state long with a bounded timeout. */
static int wait_reg_mask_state_long(volatile uint32_t *reg, uint32_t mask, uint8_t want_set)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
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

/* Configures i2c1 bme680 device reinit for normal operation. */
__attribute__((weak)) int i2c1_bme680_device_reinit(void)
{
    /* Configure only the resources required by this module. */
    return 0;
}

/* Validates i2c1 bme680 selftest before continuing with the operation. */
__attribute__((weak)) int i2c1_bme680_selftest(void)
{
    /* Check the current status before continuing with the operation. */
    return 0;
}

/* Configures i2c3 mpu9250 device reinit for normal operation. */
__attribute__((weak)) int i2c3_mpu9250_device_reinit(void)
{
    /* Configure only the resources required by this module. */
    return 0;
}

/* Validates i2c3 mpu9250 selftest before continuing with the operation. */
__attribute__((weak)) int i2c3_mpu9250_selftest(void)
{
    /* Check the current status before continuing with the operation. */
    return 0;
}


/* Handles the bme680 dma quiesce operation and returns the resulting status. */
static void i2c_bme680_dma_quiesce(void)
{
    /* Check the current status before continuing with the operation. */
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

/* Handles the mpu9250 dma quiesce operation and returns the resulting status. */
static void i2c_mpu9250_dma_quiesce(void)
{
    /* Check the current status before continuing with the operation. */
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

/* Resets toggle apb reset to a known state. */
static void i2c_toggle_apb_reset(I2C_TypeDef *I2Cx)
{
    /* Attempt the least disruptive recovery step before escalating. */
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


/* Configures dma bme680 peripherals init for normal operation. */
i2c_dma_init_status_t i2c_dma_bme680_peripherals_init(void)
{
    /* Configure only the resources required by this module. */
    i2c_select_status_t rc;
    dma_status_t status;

    
    rc = i2c_init_select(1U, 8U, 9U);
    if (rc != I2C_SELECT_OK) {
        log_printf("[ERROR] i2c_dma_bme680_peripherals_init: i2c_init_select failed rc=%d\r\n", (int)rc);
        tim2_delay_ms(2);
        return I2C_DMA_INIT_ERR_I2C_SELECT;
    }

    
    SCB->CPACR |= ((3UL << (10U * 2U)) | (3UL << (11U * 2U)));
    __DSB();
    __ISB();

    
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
        log_printf("[ERROR] i2c_dma_bme680_peripherals_init: DMA1_Stream5 init failed rc=%d\r\n", (int)status);
        tim2_delay_ms(2);
        return I2C_DMA_INIT_ERR_DMA_RX_INIT;
    }

    
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
        log_printf("[ERROR] i2c_dma_bme680_peripherals_init: DMA1_Stream6 init failed rc=%d\r\n", (int)status);
        tim2_delay_ms(2);
        return I2C_DMA_INIT_ERR_DMA_TX_INIT;
    }

    
    fnAPP_Init_Standard_Values();

    return I2C_DMA_INIT_OK;
}


/* Configures dma mpu9250 peripherals init for normal operation. */
i2c_dma_init_status_t i2c_dma_mpu9250_peripherals_init(void)
{
    /* Configure only the resources required by this module. */
    i2c_select_status_t rc;
    dma_status_t status;

    
    rc = i2c_init_select(3U, 8U, 4U);
    if (rc != I2C_SELECT_OK) {
        log_printf("[ERROR] i2c_dma_mpu9250_peripherals_init: i2c_init_select failed rc=%d\r\n", (int)rc);
        tim2_delay_ms(2);
        return I2C_DMA_INIT_ERR_I2C_SELECT;
    }

    
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
        log_printf("[ERROR] i2c_dma_mpu9250_peripherals_init: DMA1_Stream1 init failed rc=%d\r\n", (int)status);
        tim2_delay_ms(2);
        return I2C_DMA_INIT_ERR_DMA_RX_INIT;
    }

    
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
        log_printf("[ERROR] i2c_dma_mpu9250_peripherals_init: DMA1_Stream4 init failed rc=%d\r\n", (int)status);
        tim2_delay_ms(2);
        return I2C_DMA_INIT_ERR_DMA_TX_INIT;
    }

    return I2C_DMA_INIT_OK;
}


/* I2C recovery/waits */

i2c_recover_status_t i2c_recover(I2C_TypeDef *I2Cx,
                                 volatile uint32_t *status_reg,
                                 uint32_t wait_mask,
                                 uint8_t expect_set,
                                 uint8_t allow_restart)
{
    /* Attempt the least disruptive recovery step before escalating. */
    uint8_t bus_released = 0U;

    if ((I2Cx == NULL) || (status_reg == NULL)) {
        return I2C_RECOVER_FAIL;
    }

    /* log_printf("[DEBUG] I2C recover: mask=0x%08lX expect=%u\r\n", (unsigned long)wait_mask. */

    /* Quiesce the DMA streams for the matching sensor path before recovery. */
    if (I2Cx == I2C1) {
        i2c_bme680_dma_quiesce();
    } else if (I2Cx == I2C3) {
        i2c_mpu9250_dma_quiesce();
    }

    
    i2c_force_stop_and_clear(I2Cx);
    tim2_delay_ms(2);

    if (wait_reg_mask_state(&I2Cx->SR2, I2C_SR2_BUSY, 0U, BUSY_WAIT_COUNT)) {
        bus_released = 1U;
        /* log_printf("[DEBUG] i2c_recover: graceful bus release succeeded\r\n"); tim2_delay_ms(2); */
    } else {
        log_printf("[WARN] i2c_recover: graceful bus release failed\r\n");
        tim2_delay_ms(2);
    }

    /* 2) Optional quick restart path if bus is already usable */
    if ((bus_released != 0U) && (allow_restart != 0U)) {
        I2Cx->CR1 |= I2C_CR1_START;
        reg_barrier();

        if (wait_reg_mask_state(status_reg, wait_mask, expect_set, POST_WAIT_COUNT)) {
            return I2C_RECOVER_OK;
        }

        log_printf("[WARN] i2c_recover: restart-after-graceful-release failed\r\n");
        tim2_delay_ms(2);
    }

    /* 3) Escalate: peripheral-local software reset. */
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
        /* log_printf("[DEBUG] i2c_recover: SWRST recovery succeeded\r\n"); tim2_delay_ms(2); */
    } else {
        /* 4) Final escalation: APB peripheral reset. */
        log_printf("[WARN] i2c_recover: SWRST recovery failed -> toggling APB reset\r\n");
        tim2_delay_ms(2);

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

    /* 5) Sensor-specific lightweight reinit + selftest hooks. */
    if (I2Cx == I2C1) {
        if (i2c1_bme680_device_reinit() != 0) {
            log_printf("[ERROR] i2c_recover: i2c1_bme680_device_reinit() failed\r\n");
            tim2_delay_ms(2);
            return I2C_RECOVER_FAIL;
        }

        if (i2c1_bme680_selftest() != 0) {
            log_printf("[ERROR] i2c_recover: i2c1_bme680_selftest() failed\r\n");
            tim2_delay_ms(2);
            return I2C_RECOVER_FAIL;
        }

        log_printf("[DEBUG] i2c_recover: BME680 lightweight restore OK\r\n");
        tim2_delay_ms(2);
    } else if (I2Cx == I2C3) {
        if (i2c3_mpu9250_device_reinit() != 0) {
            log_printf("[ERROR] i2c_recover: i2c3_mpu9250_device_reinit() failed\r\n");
            tim2_delay_ms(2);
            return I2C_RECOVER_FAIL;
        }

        if (i2c3_mpu9250_selftest() != 0) {
            log_printf("[ERROR] i2c_recover: i2c3_mpu9250_selftest() failed\r\n");
            tim2_delay_ms(2);
            return I2C_RECOVER_FAIL;
        }

        /* log_printf("[DEBUG] i2c_recover: MPU9250 lightweight restore OK\r\n"); tim2_delay_ms(2); */
    }

    /* 6) Final restart attempt if requested */
    if (allow_restart != 0U) {
        I2Cx->CR1 |= I2C_CR1_START;
        reg_barrier();

        if (wait_reg_mask_state(status_reg, wait_mask, expect_set, POST_WAIT_COUNT)) {
            return I2C_RECOVER_OK;
        }

        log_printf("[ERROR] i2c_recover: final restart attempt failed\r\n");
        tim2_delay_ms(2);
        return I2C_RECOVER_FAIL;
    }

    return wait_reg_mask_state(status_reg, wait_mask, expect_set, POST_WAIT_COUNT)
           ? I2C_RECOVER_OK
           : I2C_RECOVER_FAIL;
}


/* Waits for wait flag busy clear with a bounded timeout. */
static int wait_flag_busy_clear(I2C_TypeDef *I2Cx)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    if (I2Cx == NULL) {
        return 0;
    }

    if (!wait_reg_mask_state(&I2Cx->SR2, I2C_SR2_BUSY, 0U, SHORT_WAIT_COUNT)) {
        log_printf("[WARN] BUSY short wait timeout\r\n");
        tim2_delay_ms(2);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR2, I2C_SR2_BUSY, 0U)) {
        log_printf("[WARN] BUSY long wait timeout\r\n");
        tim2_delay_ms(2);
    }

    if ((I2Cx->SR2 & I2C_SR2_BUSY) != 0U) {
        if (!i2c_recover(I2Cx, &I2Cx->SR2, I2C_SR2_BUSY, 0U, 0U)) {
            log_printf("[ERROR] BUSY clear recovery failed\r\n");
            tim2_delay_ms(2);
            return 0;
        }
    }

    return ((I2Cx->SR2 & I2C_SR2_BUSY) == 0U) ? 1 : 0;
}

/* Waits for wait flag addr or sb with a bounded timeout. */
static int wait_flag_addr_or_sb(I2C_TypeDef *I2Cx, uint32_t flag_mask)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    if (I2Cx == NULL) {
        return 0;
    }

    if (!wait_reg_mask_state(&I2Cx->SR1, flag_mask, 1U, SHORT_WAIT_COUNT)) {
        log_printf("[WARN] timeout waiting for SR1 flag 0x%08lX\r\n",
                   (unsigned long)flag_mask);
        tim2_delay_ms(2);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR1, flag_mask, 1U)) {
        log_printf("[WARN] SR1 long wait timeout for flag 0x%08lX\r\n",
                   (unsigned long)flag_mask);
        tim2_delay_ms(2);
    }

    if ((I2Cx->SR1 & flag_mask) == 0U) {
        /* IMPORTANT: Recover the bus/peripheral only. */
        if (!i2c_recover(I2Cx, &I2Cx->SR2, I2C_SR2_BUSY, 0U, 0U)) {
            log_printf("[ERROR] SR1 recovery failed for flag 0x%08lX\r\n",
                       (unsigned long)flag_mask);
            tim2_delay_ms(2);
            return 0;
        }

        return ((I2Cx->SR1 & flag_mask) != 0U) ? 1 : 0;
    }

    return 1;
}

/* Waits for wait flag txe or btf with a bounded timeout. */
static int wait_flag_txe_or_btf(I2C_TypeDef *I2Cx, uint32_t flag_mask)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    if (I2Cx == NULL) {
        return 0;
    }

    if (!wait_reg_mask_state(&I2Cx->SR1, flag_mask, 1U, SHORT_WAIT_COUNT)) {
        log_printf("[WARN] TXE/BTF short wait timeout for flag 0x%08lX\r\n",
                   (unsigned long)flag_mask);
        tim2_delay_ms(2);
    }

    if (!wait_reg_mask_state_long(&I2Cx->SR1, flag_mask, 1U)) {
        log_printf("[WARN] TXE/BTF long wait timeout for flag 0x%08lX\r\n",
                   (unsigned long)flag_mask);
        tim2_delay_ms(2);
    }

    if ((I2Cx->SR1 & flag_mask) == 0U) {
        /* Same principle as ADDR/SB: recover the bus/peripheral only, let the caller restart the whole I2C. */
        if (!i2c_recover(I2Cx, &I2Cx->SR2, I2C_SR2_BUSY, 0U, 0U)) {
            log_printf("[ERROR] TXE/BTF recovery failed for flag 0x%08lX\r\n",
                       (unsigned long)flag_mask);
            tim2_delay_ms(2);
            return 0;
        }

        return ((I2Cx->SR1 & flag_mask) != 0U) ? 1 : 0;
    }

    return 1;
}

/* Waits for wait dma stream disabled with a bounded timeout. */
static int wait_dma_stream_disabled(DMA_Stream_TypeDef *stream,
                                    volatile uint32_t *ifcr,
                                    uint32_t en_mask,
                                    uint32_t clear_all_mask)
{
    /* Keep the wait bounded so the task cannot remain blocked indefinitely. */
    if ((stream == NULL) || (ifcr == NULL)) {
        return 0;
    }

    if (!wait_reg_mask_state(&stream->CR, en_mask, 0U, SHORT_WAIT_COUNT)) {
        log_printf("[WARN] DMA EN short wait timeout\r\n");
        tim2_delay_ms(2);
    }

    if (!wait_reg_mask_state_long(&stream->CR, en_mask, 0U)) {
        log_printf("[WARN] DMA EN long wait timeout\r\n");
        tim2_delay_ms(2);
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



/* Configures init select for normal operation. */
i2c_select_status_t i2c_init_select(uint8_t i2c_num, uint8_t scl_pin, uint8_t sda_pin)
{
    /* Configure only the resources required by this module. */
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

    
    I2Cx->CR1 &= ~I2C_CR1_NOSTRETCH;
    I2Cx->CR1 &= ~I2C_CR1_ENGC;

    
    i2c_restore_config(I2Cx);

    if ((I2Cx->CR1 & I2C_CR1_PE) == 0U) {
        return I2C_SELECT_ERR_PE_NOT_SET;
    }

    return I2C_SELECT_OK;
}



/* Configures stream init for normal operation. */
dma_status_t dma_stream_init(DMA_Stream_TypeDef *stream,
                             uint32_t channel,
                             dma_direction_t dir,
                             volatile uint32_t *flag_reg,
                             uint32_t flag_mask,
                             IRQn_Type irq)
{
    /* Configure only the resources required by this module. */
    uint32_t cr = 0U;

    if ((stream == NULL) || (flag_reg == NULL) || (channel > 7U)) {
        log_printf("[ERROR] dma_stream_init: invalid argument\r\n");
        tim2_delay_ms(2);
        return DMA_ERR_DISABLE_TIMEOUT;
    }

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    reg_barrier();

    stream->CR &= ~DMA_SxCR_EN;
    if (!wait_dma_stream_disabled(stream, flag_reg, DMA_SxCR_EN, flag_mask)) {
        log_printf("[ERROR] dma_stream_init: stream disable timeout\r\n");
        tim2_delay_ms(2);
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
    NVIC_EnableIRQ(irq);

    return DMA_OK;
}

/* Internal DMA start helper */

static int dma1_i2c_transfer_receive(uint8_t dma_stream_num,
                                     I2C_TypeDef *I2Cx,
                                     dma_direction_t dir,
                                     uint8_t *msg,
                                     uint32_t len)
{
    /* Check the current status before continuing with the operation. */
    dma_stream_hw_t hw;

    if ((I2Cx == NULL) || (msg == NULL) || (len == 0U)) {
        log_printf("[ERROR] dma1_i2c_transfer_receive: invalid args\r\n");
        tim2_delay_ms(2);
        return 0;
    }

    
    if (I2Cx == I2C1) {
        
        if ((dir == DMA_DIR_P2M) && (dma_stream_num != 5U)) {
            log_printf("[ERROR] dma1_i2c_transfer_receive: I2C1 RX must use Stream5\r\n");
            tim2_delay_ms(2);
            return 0;
        }
        if ((dir == DMA_DIR_M2P) && (dma_stream_num != 6U)) {
            log_printf("[ERROR] dma1_i2c_transfer_receive: I2C1 TX must use Stream6\r\n");
            tim2_delay_ms(2);
            return 0;
        }

        
        I2C1->CR2 |= (I2C_CR2_DMAEN | I2C_CR2_LAST);
    } else if (I2Cx == I2C3) {
        
        if ((dir == DMA_DIR_P2M) && (dma_stream_num != 1U)) {
            log_printf("[ERROR] dma1_i2c_transfer_receive: I2C3 RX must use Stream1\r\n");
            tim2_delay_ms(2);
            return 0;
        }
        if ((dir == DMA_DIR_M2P) && (dma_stream_num != 4U)) {
            log_printf("[ERROR] dma1_i2c_transfer_receive: I2C3 TX must use Stream4\r\n");
            tim2_delay_ms(2);
            return 0;
        }

        
        I2C3->CR2 |= (I2C_CR2_DMAEN | I2C_CR2_LAST);
    }

    if (!dma1_get_stream_hw(dma_stream_num, &hw)) {
        log_printf("[ERROR] dma1_i2c_transfer_receive: unsupported stream %u\r\n", dma_stream_num);
        tim2_delay_ms(2);
        return 0;
    }

    hw.stream->CR &= ~DMA_SxCR_EN;
    if (!wait_dma_stream_disabled(hw.stream, hw.ifcr, DMA_SxCR_EN, hw.clear_all_mask)) {
        log_printf("[ERROR] dma1_i2c_transfer_receive: stream disable failed\r\n");
        tim2_delay_ms(2);
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
        log_printf("[ERROR] dma1_i2c_transfer_receive: stream did not enable\r\n");
        tim2_delay_ms(2);
        return 0;
    }

    return 1;
}


/* Updates dma write reg and propagates any transfer error. */
i2c_dma_status_t i2c_dma_write_reg(I2C_TypeDef *I2Cx,
                                   uint8_t slave_addr_7bit,
                                   uint8_t reg_addr,
                                   const uint8_t *data,
                                   uint16_t data_len)
{
    /* Stop immediately if the peripheral reports an error. */
    uint8_t tx_stream_num;

    if (I2Cx == NULL) {
        return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    }

    if ((I2Cx != I2C1) && (I2Cx != I2C2) && (I2Cx != I2C3)) {
        log_printf("[ERROR] i2c_dma_write_reg: unsupported I2C instance\r\n");
        tim2_delay_ms(2);
        return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    }

    if ((data_len > 0U) && (data == NULL)) {
        log_printf("[ERROR] i2c_dma_write_reg: NULL data with non-zero length\r\n");
        tim2_delay_ms(2);
        return I2C_DMA_ERR_NULL_BUFFER;
    }

    if (!wait_flag_busy_clear(I2Cx)) {
        log_printf("[ERROR] i2c_dma_write_reg: BUSY did not clear\r\n");
        tim2_delay_ms(2);
        return I2C_DMA_ERR_BUSY_TIMEOUT;
    }

    I2Cx->CR1 |= I2C_CR1_START;
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_SB)) {
        log_printf("[ERROR] i2c_dma_write_reg: SB timeout\r\n");
        tim2_delay_ms(2);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;

    I2Cx->DR = (uint8_t)((slave_addr_7bit << 1) | 0U);
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_ADDR)) {
        log_printf("[ERROR] i2c_dma_write_reg: ADDR timeout (write)\r\n");
        tim2_delay_ms(2);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    if (!wait_flag_txe_or_btf(I2Cx, I2C_SR1_TXE)) {
        log_printf("[ERROR] i2c_dma_write_reg: TXE timeout before reg write\r\n");
        tim2_delay_ms(2);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

    I2Cx->DR = reg_addr;
    if (!wait_flag_txe_or_btf(I2Cx, I2C_SR1_TXE)) {
        log_printf("[ERROR] i2c_dma_write_reg: TXE timeout after reg write\r\n");
        tim2_delay_ms(2);
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



/* Reads dma read and returns the resulting status. */
i2c_dma_status_t i2c_dma_read(I2C_TypeDef *I2Cx,
                              uint8_t slave_addr,
                              uint8_t reg_addr,
                              uint8_t *p_read_buff,
                              uint16_t num_of_bytes)
{
    /* Validate the transfer result before using the returned data. */
    uint8_t rx_stream_num;

    if (p_read_buff == NULL) {
        log_printf("[ERROR] i2c_dma_read: NULL buffer\r\n");
        tim2_delay_ms(2);
        return I2C_DMA_ERR_NULL_BUFFER;
    }

    if (num_of_bytes == 0U) {
        log_printf("[ERROR] i2c_dma_read: zero length\r\n");
        tim2_delay_ms(2);
        return I2C_DMA_ERR_ZERO_LEN;
    }

    if ((I2Cx != I2C1) && (I2Cx != I2C2) && (I2Cx != I2C3)) {
        log_printf("[ERROR] i2c_dma_read: unsupported I2C instance\r\n");
        tim2_delay_ms(2);
        return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    }

    if (!wait_flag_busy_clear(I2Cx)) {
        log_printf("[ERROR] i2c_dma_read: BUSY did not clear\r\n");
        tim2_delay_ms(2);
        return I2C_DMA_ERR_BUSY_TIMEOUT;
    }

    I2Cx->CR1 |= I2C_CR1_START;
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_SB)) {
        log_printf("[ERROR] i2c_dma_read: SB timeout (start)\r\n");
        tim2_delay_ms(2);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;

    I2Cx->DR = (uint8_t)((slave_addr << 1) | 0U);
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_ADDR)) {
        log_printf("[ERROR] i2c_dma_read: slave address ACK not received in write phase\r\n");
        tim2_delay_ms(2);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    if (!wait_flag_txe_or_btf(I2Cx, I2C_SR1_TXE)) {
        log_printf("[ERROR] i2c_dma_read: TXE timeout before reg address write\r\n");
        tim2_delay_ms(2);
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
        log_printf("[ERROR] i2c_dma_read: BTF timeout after reg address write\r\n");
        tim2_delay_ms(2);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

    I2Cx->CR1 |= I2C_CR1_START;
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_SB)) {
        log_printf("[ERROR] i2c_dma_read: SB timeout (restart)\r\n");
        tim2_delay_ms(2);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;

    I2Cx->DR = (uint8_t)((slave_addr << 1) | 1U);
    if (!wait_flag_addr_or_sb(I2Cx, I2C_SR1_ADDR)) {
        log_printf("[ERROR] i2c_dma_read: ADDR timeout (read phase)\r\n");
        tim2_delay_ms(2);
        I2Cx->CR1 |= I2C_CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }

    if (num_of_bytes == 1U) {
        I2Cx->CR1 &= ~I2C_CR1_ACK;
        (void)I2Cx->SR1;
        (void)I2Cx->SR2;
        I2Cx->CR1 |= I2C_CR1_STOP;

        if (!wait_reg_mask_state(&I2Cx->SR1, I2C_SR1_RXNE, 1U, I2C_WAIT_TIMEOUT_COUNT)) {
            log_printf("[ERROR] i2c_dma_read: RXNE timeout for single-byte read\r\n");
            tim2_delay_ms(2);
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
            log_printf("[ERROR] i2c_dma_read: BTF timeout for two-byte read\r\n");
            tim2_delay_ms(2);
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




/* Handles the interrupt events associated with irq wait btf short. */
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

/* BME680 I2C1 DMA IRQ handlers. */

void DMA1_Stream6_IRQHandler(void) 
{
    /* Clear handled flags and update the transfer state. */
    uint32_t hisr = DMA1->HISR;

    if ((hisr & DMA_HISR_TCIF6) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

        
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        /* expert: bounded wait for BTF before STOP */
        i2c_irq_wait_btf_short(I2C1);

        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_STOP;
        reg_barrier();

        bme680g_tx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        return;
    }

    if ((hisr & DMA_HISR_TEIF6) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTEIF6;

        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_force_stop_and_clear(I2C1);

        bme680g_tx_cmplt = I2C_DMA_TRANSFER_ERROR;
        log_printf("[ERROR] DMA1_Stream6_IRQHandler: TEIF6\r\n");
        tim2_delay_ms(2);
        return;
    }
}

/* Handles the interrupt events associated with DMA1 Stream5 IRQHandler. */
void DMA1_Stream5_IRQHandler(void) 
{
    /* Clear handled flags and update the transfer state. */
    uint32_t hisr = DMA1->HISR;

    if ((hisr & DMA_HISR_TCIF5) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;

        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        heartbeat_signal_bme680();
        reg_barrier();

        /* For RX: with CR2.LAST set, DMA handles NACK of last transfer. */
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_STOP;
        reg_barrier();

        bme680g_rx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        return;
    }

    if ((hisr & DMA_HISR_TEIF5) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTEIF5;

        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_force_stop_and_clear(I2C1);

        bme680g_rx_cmplt = I2C_DMA_TRANSFER_ERROR;
        log_printf("[ERROR] DMA1_Stream5_IRQHandler: TEIF5\r\n");
        tim2_delay_ms(2);
        return;
    }
}

/* MPU9250 I2C3 DMA IRQ handlers. */

void DMA1_Stream1_IRQHandler(void) 
{
    /* Clear handled flags and update the transfer state. */
    uint32_t lisr = DMA1->LISR;

    if ((lisr & DMA_LISR_TCIF1) != 0U) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;

        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        
        reg_barrier();

        /* For RX: with CR2.LAST set, DMA handles NACK of last transfer. */
        I2C3->CR1 &= ~I2C_CR1_ACK;
        I2C3->CR1 |= I2C_CR1_STOP;
        reg_barrier();

        mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_COMPLETE;
        return;
    }

    if ((lisr & DMA_LISR_TEIF1) != 0U) {
        DMA1->LIFCR = DMA_LIFCR_CTEIF1;

        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_force_stop_and_clear(I2C3);

        mpu9250g_rx_cmplt = I2C_DMA_TRANSFER_ERROR;
        log_printf("[ERROR] DMA1_Stream1_IRQHandler: TEIF1\r\n");
        tim2_delay_ms(2);
        return;
    }
}


/* Handles the interrupt events associated with DMA1 Stream4 IRQHandler. */
void DMA1_Stream4_IRQHandler(void) 
{
    /* Clear handled flags and update the transfer state. */
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
        return;
    }

    if ((hisr & DMA_HISR_TEIF4) != 0U) {
        DMA1->HIFCR = DMA_HIFCR_CTEIF4;

        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        reg_barrier();

        i2c_force_stop_and_clear(I2C3);

        mpu9250g_tx_cmplt = I2C_DMA_TRANSFER_ERROR;
        log_printf("[ERROR] DMA1_Stream4_IRQHandler: TEIF4\r\n");
        tim2_delay_ms(2);
        return;
    }
}



/* Handles the interrupt events associated with DMA1 Stream7 IRQHandler. */
void DMA1_Stream7_IRQHandler(void) {
    /* Clear handled flags and update the transfer state. */ }
/* Handles the interrupt events associated with DMA1 Stream2 IRQHandler. */
void DMA1_Stream2_IRQHandler(void) {
    /* Clear handled flags and update the transfer state. */ }



