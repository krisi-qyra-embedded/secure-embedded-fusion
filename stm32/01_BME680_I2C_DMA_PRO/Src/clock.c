/* adc_dma.c | Author: i fiksuari mbas prog | Date: 28 January 2026 */

#include "clock.h"
#include "stm32f4xx.h"
#include <stdint.h>

/*
 * Some projects do not compile/link system_stm32f4xx.c, which normally defines
 * SystemCoreClock. Provide a weak fallback definition here so the project links.
 * If the CMSIS system file is present, its strong definition will override this.
 */
__attribute__((weak)) uint32_t SystemCoreClock = 16000000UL;

/* Updates clock set flash latency 96mhz and propagates any transfer error. */
static void clock_set_flash_latency_96mhz(void)
{
    /* 96 MHz on STM32F4 typically uses 3 wait states. */
    FLASH->ACR =
          FLASH_ACR_ICEN
        | FLASH_ACR_DCEN
        | FLASH_ACR_PRFTEN
        | FLASH_ACR_LATENCY_3WS;

    /* Wait until latency is actually applied */
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS) {
        /* wait */
    }
}

/* Configures clock 96 mhz init for normal operation. */
void clock_96_mhz_init(void)
{
    /* Configure only the resources required by this module. */
    
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
        /* wait */
    }

    /* 2) Enable power interface clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;

    /* 3) Configure Flash for high-speed operation before switching clock */
    clock_set_flash_latency_96mhz();

    /* 4) Configure bus prescalers AHB = SYSCLK / 1 = 96 MHz APB1 = HCLK / 2 = 48 MHz APB2 = HCLK / 1 = 96 MHz */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |=  RCC_CFGR_HPRE_DIV1
               |  RCC_CFGR_PPRE1_DIV2
               |  RCC_CFGR_PPRE2_DIV1;

    /* 5) Disable PLL before reconfiguring it (safe if previously enabled) */
    if ((RCC->CR & RCC_CR_PLLON) != 0U) {
        RCC->CR &= ~RCC_CR_PLLON;
        while ((RCC->CR & RCC_CR_PLLRDY) != 0U) {
            /* wait */
        }
    }

    
    RCC->PLLCFGR =
          ((uint32_t)PLL_M << RCC_PLLCFGR_PLLM_Pos)
        | ((uint32_t)PLL_N << RCC_PLLCFGR_PLLN_Pos)
        | ((((uint32_t)PLL_P >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos)
        | RCC_PLLCFGR_PLLSRC_HSE
        | ((uint32_t)PLL_Q << RCC_PLLCFGR_PLLQ_Pos);

    
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
        
    }

    
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        
    }

    /* 9) Manual core clock update: no SystemCoreClockUpdate() dependency */
    SystemCoreClock = SYSCLK_96MHZ;

    __DSB();
    __ISB();
}


/* Configures clock 100 mhz init for normal operation. */
void clock_100_mhz_init(void)
{
    /* Configure only the resources required by this module. */
    clock_96_mhz_init();
}
