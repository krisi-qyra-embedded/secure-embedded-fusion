/* i2c_dma.c | Author: i fiksuari pas prog | Date: 17 January 2026 */
#include "i2c_dma.h"
#include"tim.h"
#include "uart.h"
#include "mpu9250.h"

 





#define GPIOBEN (1U<<1)
#define I2C1EN (1U<<21)
#define CR1_SWRST (1U<<15)
#define CR1_NOSTRETCH (1U<<7)
#define CR1_ENGC (1U<<6)
#define CR2_DMAEN (1U<<11)
#define CR2_LAST (1U<<12)
#define CR1_PE (1U<<0)
#define DMA1EN (1U<<21)
#define DMA_SCR_MINC (1U<<10)
#define DMA_SCR_TCIE (1U<<4)
#define DMA_SCR_EN (1U<<0)
#define DMA_LIFCR_CTEIF2 (1U << 9)
#define LIFCR_CTCIF2 (1U << 21) // Stream2 (I2C2_RX)
#define LIFCR_CTCIF1 (1U << 11)  
#define HIFCR_CTCIF4 (1U << 5)
#define HIFCR_CTCIF5 (1U << 11) // Stream5 (I2C1_RX)
#define HIFCR_CTCIF6 (1U << 21) // Stream6 (I2C1_TX)
#define HIFCR_CTCIF7 (1U << 27) // Stream7 (I2C2_TX)

#define WAIT_SHORT_MAX 10000U
#define PERIPH_CLK 16
#define SHORT_WAIT_COUNT 10000U  
#define LONG_WAIT_COUNT 100000U  
#define BUSY_WAIT_COUNT 100000U /* wait for BUSY clear after STOP */
#define POST_WAIT_COUNT 2000U  

#define USE_TIMER_FALLBACK

 
#ifndef DMA_SCR_PSIZE_8BIT
#define DMA_SCR_PSIZE_8BIT (0U << 11)  
#endif
#ifndef DMA_SCR_MSIZE_8BIT
#define DMA_SCR_MSIZE_8BIT (0U << 13)  
#endif
#ifndef DMA_SCR_PINC_DISABLE
#define DMA_SCR_PINC_DISABLE (0U << 9)  
#endif
#ifndef DMA_SCR_PL_HIGH
#define DMA_SCR_PL_HIGH (2U << 16)  
#endif

#define SR2_BUSY (1U<<1)
#define I2C_100KHZ 80 // 0B 0101 0000
#define SD_MODE_MAX_RISE_TIME 17
#define I2C_WAIT_TIMEOUT_COUNT 100000u

#define CR1_START (1U<<8)
#define SR1_SB (1U<<0)
#define SR1_ADDR (1U<<1)
#define SR1_TXE (1U<<7)
#define CR1_ACK (1U<<10)
#define SR1_BTF (1U<<2)
#define CR1_STOP (1U<<9)

 
#define RCC_AHB1ENR_GPIOB_EN (1U << 1)

/* Default alternate function number for I2C on common STM32 parts (AF4) */
#define I2C_AF_DEFAULT (4U)

 
#define RCC_APB1ENR_I2C1_EN (1U << 21) /* I2C1 clock enable bit in APB1ENR */
#define RCC_APB1ENR_I2C2_EN (1U << 22) /* I2C2 clock enable bit in APB1ENR */

/* APB1 clock in MHz used for I2C timing calculations (set to your board value) */
#define APB1_CLK_MHZ_DEFAULT (16U)

/* Example timing values for APB1 = 16 MHz -> ~100 kHz I2C (standard mode) These are example constants used in */
#define I2C_CCR_100KHZ (80U)
#define I2C_TRISE_STD (17U)

volatile uint8_t g_rx_cmplt;
volatile uint8_t g_tx_cmplt;

typedef enum {
    I2C_DMA_OK = 0,
    I2C_DMA_ERR_NULL_BUFFER,
    I2C_DMA_ERR_UNSUPPORTED_INSTANCE,
    I2C_DMA_ERR_ZERO_LEN,
    I2C_DMA_ERR_BUSY_TIMEOUT,
    I2C_DMA_ERR_SB_TIMEOUT,
    I2C_DMA_ERR_ADDR_TIMEOUT,
    I2C_DMA_ERR_TXE_BTF_TIMEOUT,
    I2C_DMA_ERR_DMA_START
} i2c_dma_status_t;

typedef enum {
    DMA1_STREAM5_OK = 0,            // Initialization successful
    DMA1_STREAM5_ERR_BUSY,          // Stream still enabled / busy
    DMA1_STREAM5_ERR_DISABLE_TIMEOUT, // Stream did not disable in expected time
    DMA1_STREAM5_ERR_CR_CONFIG,     // CR register config failed (if any check added later)
    DMA1_STREAM5_ERR_HIFCR_FLAG     // HIFCR clear/flag issue
} dma1_stream5_status_t;

typedef enum {
    DMA_DIR_P2M = 0, // Peripheral-to-Memory (RX)
    DMA_DIR_M2P = 1  // Memory-to-Peripheral (TX)
} dma_direction_t;

typedef enum {
    DMA_OK = 0,
    DMA_ERR_DISABLE_TIMEOUT
} dma_status_t;

typedef enum {
    DMA_DIR_MEM_TO_PERIPH = 0,
    DMA_DIR_PERIPH_TO_MEM = 1
} dma_dir_t;

typedef enum {
    I2C_SELECT_OK = 0,
    I2C_SELECT_ERR_INVALID_ARG = -1,
    I2C_SELECT_ERR_PE_NOT_SET = -2
} i2c_select_status_t;

static void i2c1_error_handler(const char *reason);

i2c_dma_status_t i2c_dma_read(I2C_TypeDef *I2Cx, uint8_t slave_addr, uint8_t reg_addr, uint8_t *p_read_buff, uint16_t num_of_bytes);
i2c_dma_status_t i2c_dma_write_reg(I2C_TypeDef *I2Cx,
                                   uint8_t slave_addr_7bit,
                                   uint8_t reg_addr,
                                   const uint8_t *data,
                                   uint16_t data_len);
dma_status_t dma_stream_init(DMA_Stream_TypeDef *stream, uint32_t channel, dma_direction_t dir, volatile uint32_t *flag_reg, uint32_t flag_mask, IRQn_Type irq);

 
/* Handles the reg barrier operation. */
static inline void reg_barrier(void) {
    /* Check the operation status before continuing. */ __DSB(); __ISB(); }

 
/* Provides the delay required by short delay. */
static inline void short_delay(volatile uint32_t d) {
    /* Keep the delay within the configured timing interval. */ while (d--) { __asm__ volatile ("nop"); } }

/* GPIO helpers (operate on GPIOB only here) */
/* Sets the gpio set mode configuration value. */
static inline void gpio_set_mode(GPIO_TypeDef *port, uint8_t pin, uint32_t mode) {
    /* Apply the new value only after validating the input. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->MODER = (port->MODER & ~(3U << pos)) | ((mode & 3U) << pos);
}
/* Sets the gpio set af configuration value. */
static inline void gpio_set_af(GPIO_TypeDef *port, uint8_t pin, uint32_t af) {
    /* Apply the new value only after validating the input. */
    uint32_t idx = (uint32_t)pin >> 3U;
    uint32_t shift = (uint32_t)(pin & 7U) * 4U;
    port->AFR[idx] = (port->AFR[idx] & ~(0xFU << shift)) | ((af & 0xFU) << shift);
}
/* Sets the gpio set otype od configuration value. */
static inline void gpio_set_otype_od(GPIO_TypeDef *port, uint8_t pin) {
    /* Apply the new value only after validating the input. */
    port->OTYPER |= (1U << pin);
}
/* Sets the gpio set pullup configuration value. */
static inline void gpio_set_pullup(GPIO_TypeDef *port, uint8_t pin) {
    /* Apply the new value only after validating the input. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->PUPDR = (port->PUPDR & ~(3U << pos)) | (1U << pos);  
}
/* Sets the gpio set highspeed configuration value. */
static inline void gpio_set_highspeed(GPIO_TypeDef *port, uint8_t pin) {
    /* Apply the new value only after validating the input. */
    uint32_t pos = (uint32_t)pin * 2U;
    port->OSPEEDR = (port->OSPEEDR & ~(3U << pos)) | (2U << pos);  
}

/* Initializes i2c init select for the requested operation. */
int i2c_init_select(uint8_t i2c_num, uint8_t scl_pin, uint8_t sda_pin) {
    /* Validate required parameters before applying the setup. */
     
    if ((scl_pin > 15U) || (sda_pin > 15U) || (scl_pin == sda_pin)) {
        return I2C_SELECT_ERR_INVALID_ARG;
    }
    if ((i2c_num != 1U) && (i2c_num != 2U) && (i2c_num != 3U)) {
        return I2C_SELECT_ERR_INVALID_ARG;
    }

     
    GPIO_TypeDef *scl_port = GPIOB;
    GPIO_TypeDef *sda_port = GPIOB;
    uint32_t scl_af = I2C_AF_DEFAULT;
    uint32_t sda_af = I2C_AF_DEFAULT;

     
    if ((i2c_num == 3U) && (scl_pin == 8U) && (sda_pin == 4U)) {
        scl_port = GPIOA;  
        sda_port = GPIOB;  
        scl_af = 4U;  
        sda_af = 9U;  
    } else {
         




        if (i2c_num == 2U) {
             
            if (sda_pin == 3U) {
                sda_af = 9U;
                sda_port = GPIOB;
            }
             
            if (scl_pin == 10U) { scl_af = I2C_AF_DEFAULT; scl_port = GPIOB; }
        } else if (i2c_num == 1U) {
             


            if ((scl_pin == 8U) || (scl_pin == 6U) || (scl_pin == 10U)) { scl_af = I2C_AF_DEFAULT; scl_port = GPIOB; }
            if ((sda_pin == 9U) || (sda_pin == 7U) || (sda_pin == 11U)) { sda_af = I2C_AF_DEFAULT; sda_port = GPIOB; }
        }
    }

     
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOB_EN;
    if (scl_port == GPIOA || sda_port == GPIOA) { RCC->AHB1ENR |= (1U << 0);   }
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

    /* Enable peripheral clock for selected I2C instance */
    if (i2c_num == 1U) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1_EN;
    } else if (i2c_num == 2U) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2_EN;
    } else {  
        RCC->APB1ENR |= (1U << 23);  
    }

    I2C_TypeDef *I2Cx = (i2c_num == 1U) ? I2C1 : (i2c_num == 2U) ? I2C2 : I2C3;

     
    I2Cx->CR1 = CR1_SWRST;
    short_delay(10U);
    I2Cx->CR1 = 0;  
    reg_barrier();

    /* Enable clock stretching, disable general call */
    I2Cx->CR1 &= ~CR1_NOSTRETCH;
    I2Cx->CR1 &= ~CR1_ENGC;

    /* Set APB1 frequency in MHz and enable DMA in CR2 */
    I2Cx->CR2 = PERIPH_CLK;
    I2Cx->CR2 |= CR2_DMAEN | CR2_LAST;

     
    I2Cx->CCR = I2C_100KHZ;
    I2Cx->TRISE = SD_MODE_MAX_RISE_TIME;

     
    I2Cx->CR1 |= CR1_PE;

    return I2C_SELECT_OK;
}

/* Configures a DMA stream for the requested peripheral transfer. */
dma_status_t dma_stream_init(DMA_Stream_TypeDef *stream, uint32_t channel, dma_direction_t dir, volatile uint32_t *flag_reg, uint32_t flag_mask, IRQn_Type irq) {
    /* Validate stream settings before enabling the DMA channel. */
    /* Enable clock access to DMA1 (for now only DMA1 supported) */
    RCC->AHB1ENR |= DMA1EN;

     
    stream->CR = 0U;

    /* Wait until stream disabled (your helper handles fallback/recovery) */
    if (!wait_dma_stream_disabled(&stream->CR, flag_reg, (uint32_t)DMA_SCR_EN, flag_mask)) {
        log_printf("[ERROR] dma_stream_init: stream failed to disable\r\n");
        return DMA_ERR_DISABLE_TIMEOUT;
    }

     
    uint32_t cr = 0U;

     
    cr |= (channel << 25);

     
    if (dir == DMA_DIR_M2P) {
        cr |= (1U << 6); // DIR[0] = 1
        cr &= ~(1U << 7); // DIR[1] = 0
    } else {
        cr &= ~((1U << 6) | (1U << 7)); // DIR = 00 (P2M)
    }

     
    cr |= DMA_SCR_MINC;

     
    cr &= ~(1U << 9);

     
    cr &= ~((3U << 11) | (3U << 13));
    cr |= DMA_SCR_PSIZE_8BIT | DMA_SCR_MSIZE_8BIT;

     
    cr &= ~(3U << 16);
    cr |= DMA_SCR_PL_HIGH;

     
    cr |= DMA_SCR_TCIE;

     
    stream->CR = cr;

     
    *flag_reg = flag_mask;

     
    NVIC_ClearPendingIRQ(irq);
    NVIC_EnableIRQ(irq);

    return DMA_OK;
}

/* Generic DMA1 I2C receive/transfer function (now supports streams 1,2,4,5,6,7) */
/* Handles the dma1 i2c transfer receive operation. */
void dma1_i2c_transfer_receive(uint8_t dma_stream, I2C_TypeDef *I2Cx, dma_dir_t dir, uint8_t *msg, uint32_t len) {
    /* Check the operation status before continuing. */
    if (msg == NULL || len == 0U) { log_printf("[WARN] dma1_i2c_transfer_receive: null/zero args\r\n"); return; }

    /* Determine DMA stream registers & flags */
    DMA_Stream_TypeDef *stream_reg = NULL;
    volatile uint32_t *flag_reg = NULL;
    uint32_t ctcif_flag = 0;

    switch (dma_stream) {
        case 1:
            stream_reg = DMA1_Stream1;
            flag_reg = &DMA1->LIFCR;  
            ctcif_flag = LIFCR_CTCIF1;
            break;
        case 2:
            stream_reg = DMA1_Stream2;
            flag_reg = &DMA1->LIFCR;  
            ctcif_flag = LIFCR_CTCIF2;
            break;
        case 4:
            stream_reg = DMA1_Stream4;
            flag_reg = &DMA1->HIFCR;  
            ctcif_flag = HIFCR_CTCIF4;
            break;
        case 5:
            stream_reg = DMA1_Stream5;
            flag_reg = &DMA1->HIFCR;  
            ctcif_flag = HIFCR_CTCIF5;
            break;
        case 6:
            stream_reg = DMA1_Stream6;
            flag_reg = &DMA1->HIFCR;  
            ctcif_flag = HIFCR_CTCIF6;
            break;
        case 7:
            stream_reg = DMA1_Stream7;
            flag_reg = &DMA1->HIFCR;  
            ctcif_flag = HIFCR_CTCIF7;
            break;
        default:
            log_printf("[ERROR] dma1_i2c_transfer_receive: unsupported stream %u\r\n", dma_stream);
            return;
    }

    /* Disable the stream (request stop) */
    stream_reg->CR &= ~DMA_SCR_EN;
    if (!wait_dma_stream_disabled(&stream_reg->CR, flag_reg, (uint32_t)DMA_SCR_EN, ctcif_flag)) {
         
        stream_reg->CR = 0U;
        *flag_reg = ctcif_flag;
        for (volatile uint32_t i = 0; i < 2000U; i++) { __asm__ volatile ("nop"); }
    }

     
    if (dir == DMA_DIR_PERIPH_TO_MEM) { g_rx_cmplt = 0; } else { g_tx_cmplt = 0; }

    /* Clear DMA stream flags (write clear-mask to IFCR) */
    *flag_reg = ctcif_flag;

     
    stream_reg->PAR = (uint32_t)(&(I2Cx->DR));
    stream_reg->M0AR = (uint32_t)msg;
    stream_reg->NDTR = len;

     
    __DSB(); __ISB();

    /* Enable the DMA stream (start transfer) */
    stream_reg->CR |= DMA_SCR_EN;
}

/* Generic I2C DMA write: pass I2C1 or I2C2 as the first parameter
   This version follows the precise sequence:
     START -> SLAVE_ADDR+W -> ACK -> REG_ADDR -> ACK -> DATA -> ACK -> STOP

   Behavior:
   - Writes first byte (register) directly to DR.
   - If only 1 byte total: blocking write, generate STOP and mark g_tx_cmplt = 1.
   - If 2 bytes total: blocking write of data byte, STOP, g_tx_cmplt = 1.
   - If >2 bytes total: hand remaining bytes (data only) to DMA (non-blocking).
     Caller should wait on g_tx_cmplt; DMA IRQ will generate STOP and set g_tx_cmplt.
*/
/* Writes i2c dma write reg to the selected interface. */
i2c_dma_status_t i2c_dma_write_reg(I2C_TypeDef *I2Cx,
                                   uint8_t slave_addr_7bit,
                                   uint8_t reg_addr,
                                   const uint8_t *data,
                                   uint16_t data_len)
{
    /* Check status before reporting a completed transfer. */
    if (I2Cx == NULL) return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    if (data_len > 0 && data == NULL) {
        log_printf("[ERROR] i2c_dma_write_reg: NULL data with non-zero length\r\n");
        return I2C_DMA_ERR_NULL_BUFFER;
    }
    if ((I2Cx != I2C1) && (I2Cx != I2C2) && (I2Cx != I2C3)) {
        log_printf("[ERROR] i2c_dma_write_reg: unsupported I2C instance\r\n");
        return I2C_DMA_ERR_UNSUPPORTED_INSTANCE;
    }

     
    if (!wait_flag_BUSY_CLEAR(&I2Cx->SR2, &I2Cx->CR1, SR2_BUSY, CR1_SWRST, CR1_STOP)) {
        log_printf("[ERROR] i2c_dma_write_reg: BUSY bit did not clear\r\n");
        return I2C_DMA_ERR_BUSY_TIMEOUT;
    }

     
    I2Cx->CR1 |= CR1_START;
    if (!wait_flag_ADDR_SB(&I2Cx->SR1, &I2Cx->CR1, SR1_SB, CR1_SWRST, CR1_START)) {
        log_printf("[ERROR] i2c_dma_write_reg: SB not set (start)\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;  

     
    I2Cx->DR = (uint8_t)((slave_addr_7bit << 1) | 0u);
    if (!wait_flag_ADDR_SB(&I2Cx->SR1, &I2Cx->CR1, SR1_ADDR, CR1_SWRST, CR1_START)) {
        log_printf("[ERROR] i2c_dma_write_reg: ADDR not set (write)\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;  

    /* Ensure TXE ready (treat timeout as error) */
    if (!wait_flag_BTF_TXE(&I2Cx->SR1, &I2Cx->CR1, SR1_TXE, CR1_SWRST, CR1_STOP, CR1_START)) {
        log_printf("[ERROR] i2c_dma_write_reg: TXE/BTF not set before reg write\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

     
    I2Cx->DR = reg_addr;
    if (!wait_flag_BTF_TXE(&I2Cx->SR1, &I2Cx->CR1, SR1_TXE, CR1_SWRST, CR1_STOP, CR1_START)) {
        log_printf("[ERROR] i2c_dma_write_reg: TXE not set after reg write\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

     

    if (data_len == 0U) {
        /* Only register written -> STOP and complete */
        if (!wait_flag_BTF_TXE(&I2Cx->SR1, &I2Cx->CR1, SR1_BTF, CR1_SWRST, CR1_STOP, CR1_START)) {
             
        }
        I2Cx->CR1 |= CR1_STOP;
        g_tx_cmplt = 1;
        return I2C_DMA_OK;
    }

     
    if (data_len == 1U) {
        if (!wait_flag_BTF_TXE(&I2Cx->SR1, &I2Cx->CR1, SR1_TXE, CR1_SWRST, CR1_STOP, CR1_START)) {
            log_printf("[WARN] i2c_dma_write_reg: TXE not set before single data byte\n");
        }
        I2Cx->DR = data[0];
        if (!wait_flag_BTF_TXE(&I2Cx->SR1, &I2Cx->CR1, SR1_BTF, CR1_SWRST, CR1_STOP, CR1_START)) {
            log_printf("[WARN] i2c_dma_write_reg: BTF not set after single data byte\n");
        }
        I2Cx->CR1 |= CR1_STOP;
        g_tx_cmplt = 1;
        return I2C_DMA_OK;
    }

    /* data_len >= 2 -> hand off all data bytes to DMA (non-blocking) */
    {
        uint8_t tx_stream = 0;
        if (I2Cx == I2C1) tx_stream = 6; /* DMA1 Stream6 -> I2C1_TX */
        else if (I2Cx == I2C2) tx_stream = 7; /* DMA1 Stream7 -> I2C2_TX */
        else tx_stream = 4;  

        /* Clear completion marker and start DMA to write 'data_len' bytes from 'data' (not including reg) */
        g_tx_cmplt = 0;
        dma1_i2c_transfer_receive(tx_stream, I2Cx, DMA_DIR_MEM_TO_PERIPH, (uint8_t *)data, (uint32_t)data_len);

         
        (void)I2Cx->SR1;
        (void)I2Cx->SR2;

        return I2C_DMA_OK;
    }
}


/* Handles the poll flag cleared with timeout operation. */
static int poll_flag_cleared_with_timeout(volatile uint32_t *reg, uint32_t mask) {
    /* Check the operation status before continuing. */
    uint32_t cnt = I2C_WAIT_TIMEOUT_COUNT;
    while (((*reg) & mask) != 0U) {  
        if (--cnt == 0U) return -1;  
    }
    return 0;  
}

/* simple, safe error handler: attempt to bring bus/peripheral to a known state */
/* Handles the i2c1 error handler operation. */
static void i2c1_error_handler(const char *reason) {
    /* Check the operation status before continuing. */
    (void)reason;  

    /* Try to generate STOP to free the bus */
    I2C1->CR1 |= CR1_STOP;

    /* Optionally: disable ACK, disable DMA stream(s), clear interrupts, reset I2C if available */
    /* Example (uncomment and adapt to your project if you have these helpers):
       DMA1_Stream5->CR &= ~DMA_SxCR_EN;
       I2C1->CR1 &= ~CR1_ACK;
    */

    /* Optional hard reset of I2C peripheral if your platform allows and it's safe:
       I2C1->CR1 |= CR1_SWRST;
       I2C1->CR1 &= ~CR1_SWRST;
    */

    /* Return to caller (function is void). Up to caller to retry or report error (e.g. via LED, log) */
}

/* Generic I2C DMA read: pass I2C1 or I2C2 as the first parameter Implements: START, SLAVE_ADDR+W, ACK */
/* Reads i2c dma read from the device or configured buffer. */
i2c_dma_status_t i2c_dma_read(I2C_TypeDef *I2Cx,
                             uint8_t slave_addr,
                             uint8_t reg_addr,
                             uint8_t *p_read_buff,
                             uint16_t num_of_bytes)
{
    /* Check the transfer result before returning the data. */
    const uint32_t RXNE = (1U << 6);  
    if (p_read_buff == NULL) { log_printf("[ERROR] i2c_dma_read: NULL buffer\r\n"); return I2C_DMA_ERR_NULL_BUFFER; }
    if (num_of_bytes == 0U)    { log_printf("[ERROR] i2c_dma_read: zero length\r\n");  return I2C_DMA_ERR_ZERO_LEN; }
    if ((I2Cx != I2C1) && (I2Cx != I2C2) && (I2Cx != I2C3)) {
        log_printf("[ERROR] i2c_dma_read: unsupported I2C instance\r\n");
        return I2C_DMA_ERR_NULL_BUFFER;
    }

     
    if (!wait_flag_BUSY_CLEAR(&I2Cx->SR2, &I2Cx->CR1, SR2_BUSY, CR1_SWRST, CR1_STOP)) {
        log_printf("[ERROR] i2c_dma_read: BUSY bit did not clear for I2C instance\r\n");
        return I2C_DMA_ERR_BUSY_TIMEOUT;
    }

     
    I2Cx->CR1 |= CR1_START;
    if (!wait_flag_ADDR_SB(&I2Cx->SR1, &I2Cx->CR1, SR1_SB, CR1_SWRST, CR1_START)) {
        log_printf("[ERROR] i2c_dma_read: SB not set (start)\r\n");
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;

     
    I2Cx->DR = (uint8_t)((slave_addr << 1) | 0u);
    if (!wait_flag_ADDR_SB(&I2Cx->SR1, &I2Cx->CR1, SR1_ADDR, CR1_SWRST, CR1_START)) {
        log_printf("[ERROR] i2c_dma_read: ADDR not set (write)\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }
     
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

     
    if (!wait_flag_BTF_TXE(&I2Cx->SR1, &I2Cx->CR1, SR1_TXE, CR1_SWRST, CR1_STOP, CR1_START)) {
        log_printf("[ERROR] i2c_dma_read: TXE/BTF not set before reg write\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

    /* Decide ACK behavior for read phase: if caller wants >=2 bytes then we'll enable ACK for read */
    if (num_of_bytes >= 2U) {
        I2Cx->CR1 |= CR1_ACK;  /* master will ACK received bytes (DMA path or multi-byte) */
    } else {
        I2Cx->CR1 &= ~CR1_ACK; /* single byte: disable ACK before clearing ADDR on read path */
    }

     
    I2Cx->DR = reg_addr;

    /* Wait until register address transferred (BTF) */
    if (!wait_flag_BTF_TXE(&I2Cx->SR1, &I2Cx->CR1, SR1_BTF, CR1_SWRST, CR1_STOP, CR1_START)) {
        log_printf("[ERROR] i2c_dma_read: BTF not set after reg write\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
    }

     
    I2Cx->CR1 |= CR1_START;
    if (!wait_flag_ADDR_SB(&I2Cx->SR1, &I2Cx->CR1, SR1_SB, CR1_SWRST, CR1_START)) {
        log_printf("[ERROR] i2c_dma_read: SB not set (restart)\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_SB_TIMEOUT;
    }
    (void)I2Cx->SR1;

    I2Cx->DR = (uint8_t)((slave_addr << 1) | 1u);
    if (!wait_flag_ADDR_SB(&I2Cx->SR1, &I2Cx->CR1, SR1_ADDR, CR1_SWRST, CR1_START)) {
        log_printf("[ERROR] i2c_dma_read: ADDR not set (read)\r\n");
        I2Cx->CR1 |= CR1_STOP;
        return I2C_DMA_ERR_ADDR_TIMEOUT;
    }

     

     
    if (num_of_bytes == 1U) {
        /* For single byte: ensure ACK is disabled BEFORE clearing ADDR */
        I2Cx->CR1 &= ~CR1_ACK;
         
        (void)I2Cx->SR1;
        (void)I2Cx->SR2;
        /* Generate STOP to end transfer after the single byte is received */
        I2Cx->CR1 |= CR1_STOP;

         
        {
            uint32_t t = 0U;
            while (!((I2Cx->SR1) & RXNE) && (t++ < I2C_WAIT_TIMEOUT_COUNT)) { __asm__ volatile ("nop"); }
            if (!((I2Cx->SR1) & RXNE)) {
                log_printf("[ERROR] i2c_dma_read: timeout waiting RXNE for single byte\r\n");
                return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
            }
        }
         
        *p_read_buff = (uint8_t)I2Cx->DR;

        /* Signal completion same as DMA path so caller waiting on g_rx_cmplt behaves uniformly */
        g_rx_cmplt = 1U;
        return I2C_DMA_OK;
    }

     
    if (num_of_bytes == 2U) {
        /* For two bytes: clear ACK BEFORE clearing ADDR (see RM sequence) */
        I2Cx->CR1 &= ~CR1_ACK;

         
        (void)I2Cx->SR1;
        (void)I2Cx->SR2;

         
        {
            uint32_t t = 0U;
            while (!((I2Cx->SR1) & SR1_BTF) && (t++ < I2C_WAIT_TIMEOUT_COUNT)) { __asm__ volatile ("nop"); }
            if (!((I2Cx->SR1) & SR1_BTF)) {
                log_printf("[ERROR] i2c_dma_read: timeout waiting BTF for two-byte read\r\n");
                 
                I2Cx->CR1 |= CR1_STOP;
                return I2C_DMA_ERR_TXE_BTF_TIMEOUT;
            }
        }
        /* Generate STOP now that both bytes are available */
        I2Cx->CR1 |= CR1_STOP;
         
        p_read_buff[0] = (uint8_t)I2Cx->DR;
        p_read_buff[1] = (uint8_t)I2Cx->DR;

        /* Signal completion for non-DMA two-byte case as well */
        g_rx_cmplt = 1U;
        return I2C_DMA_OK;
    }

    /* CASE C: >2 bytes (use DMA, non-blocking) */
    /* Clear ADDR by reading SR1 and SR2 (required before enabling DMA) */
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    /* Decide rx_stream based on I2C instance */
    uint8_t rx_stream = 0;
    if (I2Cx == I2C1) { rx_stream = 5; /* DMA1 Stream5 -> I2C1_RX */ }
    else if (I2Cx == I2C2) { rx_stream = 2; /* DMA1 Stream2 -> I2C2_RX */ }
    else { rx_stream = 1;   }

    /* Start DMA receive for num_of_bytes. */
    dma1_i2c_transfer_receive(rx_stream, I2Cx, DMA_DIR_PERIPH_TO_MEM, p_read_buff, (uint32_t)num_of_bytes);

     
    (void)I2Cx->SR1;
    (void)I2Cx->SR2;

    return I2C_DMA_OK;
}


/* Waits for completion of wait flag long fallback with a timeout. */
int wait_flag_long_fallback(volatile __IO uint32_t *reg, uint32_t mask) {
    /* Return a timeout rather than waiting indefinitely. */
    int success = 0;

    /* Special BUSY-bit handling: wait while BUSY is SET, succeed when cleared */
    if (((mask == DMA_SCR_EN) && ((reg == &DMA1_Stream6->CR) || (reg == &DMA1_Stream5->CR) || (reg == &DMA1_Stream2->CR) || (reg == &DMA1_Stream7->CR) || (reg == &DMA1_Stream1->CR) || (reg == &DMA1_Stream4->CR))) || ((mask == SR2_BUSY) && ((reg == &I2C1->SR2) || (reg == &I2C2->SR2) || (reg == &I2C3->SR2)))) {
        if (((*reg) & mask)) {
#ifdef USE_TIMER_FALLBACK
            tim2_1s_flag = 0;
            tim2_1s_start();
            while (((*reg) & mask) && !tim2_1s_check_flag()) { __asm__ volatile ("nop"); }
            tim2_1s_stop();
            if (((*reg) & mask)) { log_printf("[DEBUG] timer fallback expired waiting for BUSY=0x%08X (reg=0x%08X)\r\n", (unsigned)mask, (unsigned)(*reg)); }
#else
            uint32_t long_wait = LONG_WAIT_COUNT;
            while (((*reg) & mask) && long_wait--) { __asm__ volatile ("nop"); }
            if (((*reg) & mask)) { log_printf("[DEBUG] long busy-poll expired waiting for BUSY=0x%08X (reg=0x%08X)\r\n", (unsigned)mask, (unsigned)(*reg)); }
#endif
        }
        /* success = 1 when BUSY cleared (bit == 0) */
        success = (((*reg) & mask) == 0U) ? 1 : 0;
    } else {
         
        if (!((*reg) & mask)) {
#ifdef USE_TIMER_FALLBACK
            tim2_1s_flag = 0;
            tim2_1s_start();
            while (!((*reg) & mask) && !tim2_1s_check_flag()) { __asm__ volatile ("nop"); }
            tim2_1s_stop();
            if (!((*reg) & mask)) { log_printf("[DEBUG] timer fallback expired waiting for SR_BIT=0x%08X (reg=0x%08X)\r\n", (unsigned)mask, (unsigned)(*reg)); }
#else
            {
                uint32_t long_wait = LONG_WAIT_COUNT;
                while (!((*reg) & mask) && long_wait--) { __asm__ volatile ("nop"); }
            }
            if (!((*reg) & mask)) { log_printf("[DEBUG] long busy-poll expired waiting for SR_BIT=0x%08X (reg=0x%08X)\r\n", (unsigned)mask, (unsigned)(*reg)); }
#endif
        }
         
        success = (((*reg) & mask) != 0U) ? 1 : 0;
    }

    return success; /* 1 == success, 0 == failure (caller: if (!wait_flag_long_fallback(...)) { recovery } ) */
}

/* I2C Recovery Helper Attempts to recover when SR_BIT is not set (e.g. stuck bus or missed event) */
/* Attempts to restore a stalled I2C peripheral and bus. */
static int i2c_recover(volatile __IO uint32_t *reg_SR, volatile __IO uint32_t *reg_CR, uint32_t SR_BIT, uint32_t CR_SWRST, uint32_t CR_STOP, uint32_t CR_START) {
    /* Stop the failed transfer and restore the bus before retrying. */
    log_printf("[DEBUG] entering recovery for SR_BIT=0x%08X\r\n", (unsigned)SR_BIT);

    /* 1) Request STOP to release the bus */
    *reg_CR |= CR_STOP;
    log_printf("[DEBUG] STOP requested (CR_STOP set). Waiting for BUSY to clear...\r\n");

    /* 2) Wait bounded for BUSY to clear (explicit SR2 check) */
    {
        uint32_t busy_wait = BUSY_WAIT_COUNT;
        while (((I2C1->SR2 & SR2_BUSY) || (I2C2->SR2 & SR2_BUSY) || (I2C3->SR2 & SR2_BUSY)) && busy_wait--) { __asm__ volatile ("nop"); }
        if ((I2C1->SR2 & SR2_BUSY) || (I2C2->SR2 & SR2_BUSY) || (I2C3->SR2 & SR2_BUSY)){
            log_printf("[DEBUG] BUSY did not clear after STOP (SR2=0x%08X)\r\n", (unsigned)I2C1->SR2);
        } else {
            log_printf("[DEBUG] BUSY cleared (SR2=0x%08X)\r\n", (unsigned)I2C1->SR2);
        }
    }

     
    {
        volatile uint32_t I2C1_sr1 = I2C1->SR1;
        volatile uint32_t I2C1_sr2 = I2C1->SR2;
        volatile uint32_t I2C2_sr1 = I2C2->SR1;
        volatile uint32_t I2C2_sr2 = I2C2->SR2;
        volatile uint32_t I2C3_sr1 = I2C3->SR1;
        volatile uint32_t I2C3_sr2 = I2C3->SR2;
        (void)I2C1_sr1; (void)I2C1_sr2; (void)I2C2_sr1; (void)I2C2_sr2; (void)I2C3_sr1; (void)I2C3_sr2;
        if(reg_SR == &I2C1->SR1){ log_printf("[DEBUG] Performed SR1/SR2 reads: SR1=0x%08X SR2=0x%08X\r\n", (unsigned)I2C1->SR1, (unsigned)I2C1->SR2); }
        else if(reg_SR == &I2C2->SR1){ log_printf("[DEBUG] Performed SR1/SR2 reads: SR1=0x%08X SR2=0x%08X\r\n", (unsigned)I2C2->SR1, (unsigned)I2C2->SR2); }
        else{ log_printf("[DEBUG] Performed SR1/SR2 reads: SR1=0x%08X SR2=0x%08X\r\n", (unsigned)I2C3->SR1, (unsigned)I2C3->SR2); }
    }

     
    if (((SR_BIT == SR1_BTF) && ((reg_SR == &I2C1->SR1) || (reg_SR == &I2C2->SR1) || (reg_SR == &I2C3->SR1))) || ((SR_BIT == SR1_TXE) && ((reg_SR == &I2C1->SR1) || (reg_SR == &I2C2->SR1) || (reg_SR == &I2C3->SR1)))) {
        log_printf("[DEBUG] Attempting to generate RESTART (CR_START set)...\r\n");
        *reg_CR |= CR_START;
        {
            uint32_t post_wait = POST_WAIT_COUNT;
            while (!((*reg_SR) & SR_BIT) && post_wait--) { __asm__ volatile ("nop"); }
        }
        if ((*reg_SR) & SR_BIT) {
            log_printf("[DEBUG] SR_BIT=0x%08X set after restart attempt (reg_SR val=0x%08X)\r\n", (unsigned)SR_BIT, (unsigned)(*reg_SR));
            return 1; // success
        } else {
            log_printf("[DEBUG] SR_BIT still NOT set after restart attempt (reg_SR val=0x%08X)\r\n", (unsigned)(*reg_SR));
        }
    }

    /* 5) Last-resort: peripheral software reset (bounded) */
    log_printf("[DEBUG] Performing peripheral software reset (CR_SWRST) as last resort\r\n");
    *reg_CR |= CR_SWRST;
    for (volatile uint32_t i = 0; i < 2000U; ++i) { __asm__ volatile ("nop"); }
    *reg_CR &= ~CR_SWRST;
    log_printf("[DEBUG] software reset toggled\r\n");

     
    for (volatile uint32_t i = 0; i < 2000U; ++i) { __asm__ volatile ("nop"); }
    (void)I2C1->SR1; (void)I2C1->SR2; (void)I2C2->SR1; (void)I2C2->SR2; (void)I2C3->SR1; (void)I2C3->SR2;
    if(reg_SR == &I2C1->SR1){ log_printf("[DEBUG] post-reset SR1=0x%08X SR2=0x%08X\r\n", (unsigned)I2C1->SR1, (unsigned)I2C1->SR2); }
    else if(reg_SR == &I2C2->SR1){ log_printf("[DEBUG] post-reset SR1=0x%08X SR2=0x%08X\r\n", (unsigned)I2C2->SR1, (unsigned)I2C2->SR2); }
    else{ log_printf("[DEBUG] post-reset SR1=0x%08X SR2=0x%08X\r\n", (unsigned)I2C3->SR1, (unsigned)I2C3->SR2); }

    return 0; // failure if we reach here
}

/* Waits for completion of i2c wait short with a timeout. */
static int i2c_wait_short(volatile __IO uint32_t *reg_SR, uint32_t SR_BIT) {
    /* Return a timeout rather than waiting indefinitely. */
    uint32_t short_wait = 0U;

    if ( ((SR_BIT == SR1_BTF) || (SR_BIT == SR1_TXE) || (SR_BIT == SR1_ADDR) || (SR_BIT == SR1_SB)) && ((reg_SR == &I2C1->SR1) || (reg_SR == &I2C2->SR1) || (reg_SR == &I2C3->SR1))) {
        /* Short busy-poll loop for BTF/TXE/ADDR/SB */
        while (!((*reg_SR) & SR_BIT) && (short_wait++ < SHORT_WAIT_COUNT)) { __asm__ volatile ("nop"); }
        if (!((*reg_SR) & SR_BIT)) {
            log_printf("[DEBUG] short busy-poll timeout waiting for SR_BIT=0x%08X (reg_SR=%p val=0x%08X)\r\n", (unsigned)SR_BIT, (void *)reg_SR, (unsigned)(*reg_SR));
            return 0; // timeout
        }
        return 1; // success
    } else {
        /* Short busy-poll loop: wait while BUSY bit is set */
        while ((*reg_SR) & SR_BIT && (short_wait++ < SHORT_WAIT_COUNT)) { __asm__ volatile ("nop"); }
        if ((*reg_SR) & SR_BIT) {
            log_printf("[DEBUG] short busy-poll timeout while waiting for BUSY bit=0x%08X (reg_SR=%p, val=0x%08X)\r\n", (unsigned)SR_BIT, (void *)reg_SR, (unsigned)(*reg_SR));
            return 0; // timeout
        }
        return 1; // success
    }
}

/* Waits for completion of finalize wait flag result with a timeout. */
int finalize_wait_flag_result(volatile __IO uint32_t *reg_SR, uint32_t SR_BIT, int flag_seen) {
    /* Return a timeout rather than waiting indefinitely. */
    /* BTF / TXE: expect the bit to be SET and reg_SR must be &I2C1->SR1 */
    if ( ((SR_BIT == SR1_BTF) && ((reg_SR == &I2C1->SR1) || (reg_SR == &I2C2->SR1) || (reg_SR == &I2C3->SR1))) || ((SR_BIT == SR1_TXE) && ((reg_SR == &I2C1->SR1) || (reg_SR == &I2C2->SR1) || (reg_SR == &I2C3->SR1)))) {
        if (((*reg_SR) & SR_BIT)) {
             
            return 1;
        } else {
            log_printf("[ERROR] wait_flag_BTF_TXE FAILURE: SR_BIT=0x%08X not set after recovery attempts (reg_SR val=0x%08X)\r\n", (unsigned)SR_BIT, (unsigned)(*reg_SR));
            return 0;
        }
    }
    /* ADDR / SB: success if bit set OR flag_seen was set when clearing ADDR, and reg_SR must be &I2C1->SR1 */
    else if ( ((SR_BIT == SR1_ADDR) && ((reg_SR == &I2C1->SR1) || (reg_SR == &I2C2->SR1) || (reg_SR == &I2C3->SR1))) || ((SR_BIT == SR1_SB) && ((reg_SR == &I2C1->SR1) || (reg_SR == &I2C2->SR1) || (reg_SR == &I2C3->SR1)))) {
        if (((*reg_SR) & SR_BIT) || (flag_seen == 1)) {
             
            return 1;
        } else {
            log_printf("[ERROR] wait_flag_ADDR_SB FAILURE: SR_BIT=0x%08X not set after all recovery attempts (reg_SR val=0x%08X)\r\n", (unsigned)SR_BIT, (unsigned)(*reg_SR));
            return 0;
        }
    }
    /* DMA stream disable: check reg_SR for this case (per prior request) */
    else if ((SR_BIT == DMA_SCR_EN) && ((reg_SR == &DMA1_Stream6->CR)|| (reg_SR == &DMA1_Stream5->CR)|| (reg_SR == &DMA1_Stream2->CR)|| (reg_SR == &DMA1_Stream7->CR)|| (reg_SR == &DMA1_Stream1->CR)|| (reg_SR == &DMA1_Stream4->CR))) {
        if (!(((*reg_SR) & SR_BIT))) {
            /* log_printf("[INFO] wait_dma_stream_disabled success: DMA stream disabled (reg_SR=0x%08X)\r\n" */
            return 1;
        } else {
            log_printf("[ERROR] wait_dma_stream_disabled FAILURE: stream still enabled (reg_SR=0x%08X)\r\n", (unsigned)(*reg_SR));
            return 0;
        }
    }
    /* BUSY (SR2) case: success when BUSY cleared and reg_SR must be &I2C1->SR2 */
    else if ((SR_BIT == SR2_BUSY) && ((reg_SR == &I2C1->SR2)||(reg_SR == &I2C2->SR2) || (reg_SR == &I2C3->SR2))) {
        if (!(((*reg_SR) & SR_BIT))) {
            /* log_printf("[INFO] wait_flag_BUSY_CLEAR success: BUSY bit cleared (reg_SR val=0x%08X)\r\n" */
            return 1;
        } else {
            log_printf("[ERROR] wait_flag_BUSY_CLEAR FAILURE: BUSY bit still set after recovery attempts (reg_SR val=0x%08X)\r\n", (unsigned)(*reg_SR));
            return 0;
        }
    }
     
    else {
        log_printf("[WARN] finalize_wait_flag_result: unknown SR_BIT=0x%08X (reg_SR=0x%08X)\r\n", (unsigned)SR_BIT, (unsigned)(*reg_SR));
        return -1;
    }
}

/* Waits for completion of wait flag btf txe with a timeout. */
int wait_flag_BTF_TXE(volatile __IO uint32_t *reg_SR, volatile __IO uint32_t *reg_CR, uint32_t SR_BIT, uint32_t CR_SWRST, uint32_t CR_STOP, uint32_t CR_START) // signature unchanged
{
    /* Return a timeout rather than waiting indefinitely. */
    /* Use the new i2c_wait_short helper for short busy-poll */
    if (!i2c_wait_short(reg_SR, SR_BIT)) {
        log_printf("[WARN] short busy-poll timeout for SR_BIT=0x%08X, proceeding to fallback/recovery\n", (unsigned)SR_BIT);
    }

     
    if (!wait_flag_long_fallback(reg_SR, SR_BIT)) {
        /* fallback expired: decide recovery here */
        log_printf("[WARN] SB not seen\n");
        /* do recovery actions: STOP, clear SRs, maybe SWRST */
    }

     
    if (!((*reg_SR) & SR_BIT)) {
        int ret = i2c_recover(reg_SR, reg_CR, SR_BIT,CR_SWRST,CR_STOP,CR_START);
        if (ret) {
            /* log_printf("[INFO] wait_flag_BTF_TXE recovery SUCCESS\n") */
            return 1;
        } else {
            log_printf("[ERROR] wait_flag_BTF_TXE recovery FAILED\n");
            // continue to final failure handling
        }
    }

     
     
    return finalize_wait_flag_result(reg_SR, SR_BIT, 0);
}

/* Waits for completion of wait flag addr sb with a timeout. */
int wait_flag_ADDR_SB(volatile __IO uint32_t *reg_SR, volatile __IO uint32_t *reg_CR, uint32_t SR_BIT, uint32_t CR_SWRST, uint32_t CR_STOP) {
    /* Return a timeout rather than waiting indefinitely. */
    uint8_t flag_seen = 0;  

    /* Short busy-poll loop using helper */
    if (!i2c_wait_short(reg_SR, SR_BIT)) {
        log_printf("[WARN] short busy-poll timeout for SR_BIT=0x%08X, proceeding to fallback/recovery\n", (unsigned)SR_BIT);
    }

     
    if (!wait_flag_long_fallback(reg_SR, SR_BIT)) {
        /* fallback expired: decide recovery here */
        log_printf("[WARN] SB not seen\n");
        /* do recovery actions: STOP, clear SRs, maybe SWRST */
    }

     
    if (!((*reg_SR) & SR_BIT)) {
        int ret = i2c_recover(reg_SR, reg_CR, SR_BIT,CR_SWRST,CR_STOP,CR_STOP);
        if (ret) {
            /* log_printf("[INFO] wait_flag_ADDR_SB recovery SUCCESS\n") */
            // optionally clear ADDR if needed later
        } else {
            log_printf("[ERROR] wait_flag_ADDR_SB recovery FAILED\n");
        }
    }

     
    if (((*reg_SR) & SR_BIT) && (SR_BIT == SR1_ADDR)) {
        volatile uint32_t I2C1_sr1_val = I2C1->SR1;  
        volatile uint32_t I2C2_sr1_val = I2C2->SR1;
        volatile uint32_t I2C3_sr1_val = I2C3->SR1;
        __asm__ volatile ("" ::: "memory");  
        volatile uint32_t I2C1_sr2_val = I2C1->SR2;  
        volatile uint32_t I2C2_sr2_val = I2C2->SR2;  
        volatile uint32_t I2C3_sr2_val = I2C3->SR2;  
        (void)I2C1_sr1_val; (void)I2C2_sr1_val; (void)I2C3_sr1_val; (void)I2C1_sr2_val; (void)I2C2_sr2_val; (void)I2C3_sr2_val;
        flag_seen = 1;
#ifdef DEBUG_LOG
        log_printf("[DEBUG] ADDR special-case: cleared ADDR by reading SR1=0x%08X SR2=0x%08X\r\n", (unsigned)sr1_val, (unsigned)sr2_val);
#endif
    }

     
    return finalize_wait_flag_result(reg_SR, SR_BIT, flag_seen);
}

/* Waits for completion of wait flag busy clear with a timeout. */
int wait_flag_BUSY_CLEAR(volatile __IO uint32_t *reg_SR, volatile __IO uint32_t *reg_CR, uint32_t SR_BUSY_BIT, uint32_t CR_SWRST, uint32_t CR_STOP) {
    /* Return a timeout rather than waiting indefinitely. */
    /* Short busy-poll loop using helper */
    if (!i2c_wait_short(reg_SR, SR_BUSY_BIT)) {
        log_printf("[WARN] short busy-poll timeout for BUSY bit=0x%08X, proceeding to fallback/recovery\n", (unsigned)SR_BUSY_BIT);
    }

     
    if (!wait_flag_long_fallback(reg_SR, SR_BUSY_BIT)) {
        /* fallback expired: decide recovery here */
        log_printf("[WARN] SB not seen\n");
        /* do recovery actions: STOP, clear SRs, maybe SWRST */
    }

     
    if (((*reg_SR) & SR_BUSY_BIT)) {
        int ret = i2c_recover(reg_SR, reg_CR, SR_BUSY_BIT,CR_SWRST,CR_STOP,CR_STOP);
        if (ret) {
            /* log_printf("[INFO] wait_flag_BUSY_CLEAR recovery SUCCESS\n") */
        } else {
            log_printf("[ERROR] wait_flag_BUSY_CLEAR recovery FAILED\n");
        }
    }

    /* Finalize result (finalizer will treat BUSY case correctly) */
    return finalize_wait_flag_result(reg_SR, SR_BUSY_BIT, 0); /* flag_seen not applicable for BUSY */
}

/* Wait until a DMA stream's EN bit clears (disabled). */
/* Waits for completion of wait dma stream disabled with a timeout. */
int wait_dma_stream_disabled(volatile __IO uint32_t *stream_CR, volatile __IO uint32_t *dma_HIFCR, uint32_t CR_EN_MASK, uint32_t HIFCR_CTCIF_MASK) {
    /* Return a timeout rather than waiting indefinitely. */
    /* Short busy-poll loop using helper */
    if (!i2c_wait_short(stream_CR, CR_EN_MASK)) {
        log_printf("[WARN] short busy-poll timeout for DMA stream enable bit=0x%08X, proceeding to fallback/recovery\r\n", (unsigned)CR_EN_MASK);
    }

     
    if (!wait_flag_long_fallback(stream_CR, CR_EN_MASK)) {
        /* fallback expired: decide recovery here */
        log_printf("[WARN] SB not seen\r\n");
        /* do recovery actions: STOP, clear SRs, maybe SWRST */
    }

    /* Recovery attempt: graceful disable + clear flags */
    if ((*stream_CR) & CR_EN_MASK) {
        log_printf("[DEBUG] Attempting graceful disable + flag clear...\r\n");
        *stream_CR &= ~CR_EN_MASK;  
        *dma_HIFCR |= HIFCR_CTCIF_MASK;  
        for (volatile uint32_t i = 0; i < 1000U; ++i) { __asm__ volatile ("nop"); }
        if ((*stream_CR) & CR_EN_MASK) {
            log_printf("[DEBUG] Graceful disable failed, forcing CR=0.\r\n");
            *stream_CR = 0U;  
            *dma_HIFCR |= HIFCR_CTCIF_MASK;  
            for (volatile uint32_t i = 0; i < 2000U; ++i) { __asm__ volatile ("nop"); }
        }
    }

    /* Final result: delegate -to finalizer (handles DMA case) */
    return finalize_wait_flag_result(stream_CR, CR_EN_MASK, 0);  
}

 
/* Checks the tim2 1s check flag condition. */
uint8_t tim2_1s_check_flag(void) {
    /* Return the status when the validation does not pass. */
    if (tim2_1s_flag) { tim2_1s_flag = 0; // Reset immediately
        return 1;
    }
    return 0;
}

/* existing IRQ: OK as you had it, it stops timer and sets flag */
/* Handles the tim2 irqhandler interrupt. */
void TIM2_IRQHandler(void) {
    /* Acknowledge the handled event and update transfer status. */
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        TIM2->CR1 &= ~TIM_CR1_CEN; // stop timer
        TIM2->DIER &= ~TIM_DIER_UIE; // disable its interrupt
        tim2_1s_flag = 1;
    }
}

/* Handles the dma1 stream6 irqhandler interrupt. */
void DMA1_Stream6_IRQHandler(void) // TX complete
{
    /* Acknowledge the handled event and update transfer status. */
    uint32_t hisr = DMA1->HISR;

     
    if (hisr & DMA_HISR_TCIF6) // Transfer complete
    {
         
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;

         
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;

        /* 3) Ensure register writes complete before proceeding */
        __DSB(); __ISB();

        /* 4) For I2C TX the right action is to generate STOP after the last byte has been shifted out. */
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_STOP;

         
        g_tx_cmplt = 1;
        return;
    }

    /* DMA transfer error path */
    if (hisr & DMA_HISR_TEIF6) // Transfer error
    {
         
        DMA1->HIFCR = DMA_HIFCR_CTEIF6;

        /* Minimal immediate reaction: request stream disable (avoid busy-wait) */
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;

         
        __DSB(); __ISB();

        /* Signal an error to main context (choose an error code/enum you use) */
        g_tx_cmplt = 2; /* 1 == success, 2 == error (example) */

        log_printf("[ERROR] DMA1_Stream6_IRQHandler: TEIF6 (DMA transfer error)\r\n");

        /* Return quickly; full recovery handled from main context */
        return;
    }

    /* Optionally handle other flags (FIFO, etc.) here if needed */
}

/* Handles the dma1 stream7 irqhandler interrupt. */
void DMA1_Stream7_IRQHandler(void) // TX complete
{
    /* Acknowledge the handled event and update transfer status. */
    uint32_t hisr = DMA1->HISR;

     
    if (hisr & DMA_HISR_TCIF7) // Transfer complete
    {
         
        DMA1->HIFCR = DMA_HIFCR_CTCIF7;

         
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;

         
        __DSB(); __ISB();

        /* 4) Optional: I2C STOP generation for TX if needed */
        I2C2->CR1 &= ~I2C_CR1_ACK; // disable ACK -> NACK on last byte
        I2C2->CR1 |= I2C_CR1_STOP;

         
        g_tx_cmplt = 1;
        return;
    }

    /* DMA transfer error path */
    if (hisr & DMA_HISR_TEIF7)
    {
         
        DMA1->HIFCR = DMA_HIFCR_CTEIF7;

        /* Signal error to main loop */
        g_tx_cmplt = 2;
        log_printf("[ERROR] DMA1_Stream7_IRQHandler: TEIF7 (DMA transfer error)\r\n");
        return;
    }
}

/* Handles the dma1 stream2 irqhandler interrupt. */
void DMA1_Stream2_IRQHandler(void) // RX complete
{
    /* Acknowledge the handled event and update transfer status. */
    uint32_t lisr = DMA1->LISR;

     
    if (lisr & DMA_LISR_TCIF2) // Transfer complete
    {
         
        DMA1->LIFCR = LIFCR_CTCIF2;

         
        DMA1_Stream2->CR &= ~DMA_SxCR_EN;

         
        __DSB(); __ISB();

        /* 4) I2C NACK + STOP for last byte(s) */
        I2C2->CR1 &= ~I2C_CR1_ACK; // disable ACK -> NACK on last byte
        I2C2->CR1 |= I2C_CR1_STOP; // generate STOP condition

         
        g_rx_cmplt = 1;
        return;
    }

    /* DMA transfer error path */
    if (lisr & DMA_LISR_TEIF2)
    {
         
        DMA1->LIFCR = DMA_LIFCR_CTEIF2;

        /* Signal error */
        g_rx_cmplt = 2;
        log_printf("[ERROR] DMA1_Stream2_IRQHandler: TEIF2 (DMA transfer error)\r\n");
        return;
    }
}

/* Handles the dma1 stream5 irqhandler interrupt. */
void DMA1_Stream5_IRQHandler(void) // RX complete
{
    /* Acknowledge the handled event and update transfer status. */
    uint32_t hisr = DMA1->HISR;

     
    if (hisr & DMA_HISR_TCIF5) // Transfer complete
    {
         
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;

         
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;

         
        __DSB(); __ISB();

        /* 4) I2C ACK/NACK/STOP: For typical multi-byte DMA I2C reads you must arrange ACK/NACK before the last */
        I2C1->CR1 &= ~I2C_CR1_ACK; // disable ACK -> NACK on last byte
        I2C1->CR1 |= I2C_CR1_STOP; // generate STOP condition

         
        g_rx_cmplt = 1;
         
        return;
    }

    /* DMA transfer error path */
    if (hisr & DMA_HISR_TEIF5)
    {
         
        DMA1->HIFCR = DMA_HIFCR_CTEIF5;

        /* Attempt minimal recovery / signal error to main loop */
        g_rx_cmplt = 2; /* choose a value meaning "error" in your app */
        log_printf("[ERROR] DMA1_Stream5_IRQHandler: TEIF5 (DMA transfer error)\r\n");

         




        return;
    }

     
}

/* Handles the dma1 stream1 irqhandler interrupt. */
void DMA1_Stream1_IRQHandler(void) // RX complete
{
    /* Acknowledge the handled event and update transfer status. */
    uint32_t lisr = DMA1->LISR;

     
    if (lisr & DMA_LISR_TCIF1) // Transfer complete
    {
         
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;

         
        DMA1_Stream1->CR &= ~DMA_SxCR_EN;

         
        __DSB(); __ISB();

         
        I2C3->CR1 &= ~I2C_CR1_ACK; // disable ACK -> NACK on last byte
        I2C3->CR1 |= I2C_CR1_STOP; // generate STOP condition

         
        g_rx_cmplt = 1;
        return;
    }

    /* DMA transfer error path */
    if (lisr & DMA_LISR_TEIF1)
    {
         
        DMA1->LIFCR = DMA_LIFCR_CTEIF1;

        /* Signal error */
        g_rx_cmplt = 2;
        log_printf("[ERROR] DMA1_Stream1_IRQHandler: TEIF1 (DMA transfer error)\r\n");
        return;
    }
}

/* Handles the dma1 stream4 irqhandler interrupt. */
void DMA1_Stream4_IRQHandler(void) // RX/TX complete
{
    /* Acknowledge the handled event and update transfer status. */
    uint32_t hisr = DMA1->HISR; // Stream4 flags are in the high ISR

     
    if (hisr & DMA_HISR_TCIF4) // Transfer complete
    {
         
        DMA1->HIFCR = DMA_HIFCR_CTCIF4;

         
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;

         
        __DSB(); __ISB();

         
        I2C3->CR1 &= ~I2C_CR1_ACK; // disable ACK -> NACK on last byte
        I2C3->CR1 |= I2C_CR1_STOP; // generate STOP condition

         
        g_tx_cmplt = 1;
        return;
    }

    /* DMA transfer error path */
    if (hisr & DMA_HISR_TEIF4)
    {
         
        DMA1->HIFCR = DMA_HIFCR_CTEIF4;

        /* Signal error */
        g_tx_cmplt = 2;
        log_printf("[ERROR] DMA1_Stream4_IRQHandler: TEIF4 (DMA transfer error)\r\n");
        return;
    }

     
    // if (hisr & DMA_HISR_HTIF4) { ... }
}
