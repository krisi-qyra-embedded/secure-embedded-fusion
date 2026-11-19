/**
 * @file clock.c
 * @brief STM32F411 96 MHz system clock configuration.
 *
 * Uses the 8 MHz HSE as PLL source.
 *
 * Clock tree:
 * - HSE   = 8 MHz
 * - PLL_M = 8
 * - PLL_N = 192
 * - PLL_P = 2
 * - PLL_Q = 4
 *
 * Result:
 * - VCO_IN  = 1 MHz
 * - VCO_OUT = 192 MHz
 * - SYSCLK  = 96 MHz
 * - HCLK    = 96 MHz
 * - APB1    = 48 MHz
 * - APB2    = 96 MHz
 */

#include "clock.h"
#include "stm32f4xx.h"
#include <stdint.h>

/*
 * Some projects do not compile/link system_stm32f4xx.c, which normally defines
 * SystemCoreClock. Provide a weak fallback definition here so the project links.
 * If the CMSIS system file is present, its strong definition will override this.
 */
__attribute__((weak)) uint32_t SystemCoreClock = 16000000UL;


/**
 * @brief Configure Flash for 96 MHz operation.
 *
 * Sets Flash latency to 3 wait states and enables instruction cache,
 * data cache and prefetch.
 */
static void clock_set_flash_latency_96mhz(void)
{
    /*
     * 96 MHz on STM32F4 typically uses 3 wait states.
     * Also enable I-cache, D-cache, and prefetch.
     */
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


/**
 * @brief Configure the STM32F411 system clock to 96 MHz.
 *
 * Enables the 8 MHz HSE, configures the PLL and sets the bus
 * prescalers for 96 MHz HCLK, 48 MHz APB1 and 96 MHz APB2.
 *
 * @note SystemCoreClock is updated manually after switching to the PLL.
 */
void clock_96_mhz_init(void)
{
    /* 1) Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
        /* wait */
    }

    /* 2) Enable power interface clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;

    /* 3) Configure Flash for high-speed operation before switching clock */
    clock_set_flash_latency_96mhz();

    /*
     * 4) Configure bus prescalers
     *    AHB  = SYSCLK / 1  = 96 MHz
     *    APB1 = HCLK   / 2  = 48 MHz
     *    APB2 = HCLK   / 1  = 96 MHz
     */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |=  RCC_CFGR_HPRE_DIV1
               | RCC_CFGR_PPRE1_DIV2
               | RCC_CFGR_PPRE2_DIV1;

    /* 5) Disable PLL before reconfiguring it (safe if previously enabled) */
    if ((RCC->CR & RCC_CR_PLLON) != 0U) {
        RCC->CR &= ~RCC_CR_PLLON;

        while ((RCC->CR & RCC_CR_PLLRDY) != 0U) {
            /* wait */
        }
    }

    /* 6) Configure PLL for 96 MHz SYSCLK from 8 MHz HSE */
    RCC->PLLCFGR =
          ((uint32_t)PLL_M << RCC_PLLCFGR_PLLM_Pos)
        | ((uint32_t)PLL_N << RCC_PLLCFGR_PLLN_Pos)
        | ((((uint32_t)PLL_P >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos)
        | RCC_PLLCFGR_PLLSRC_HSE
        | ((uint32_t)PLL_Q << RCC_PLLCFGR_PLLQ_Pos);

    /* 7) Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
        /* wait */
    }

    /* 8) Switch SYSCLK to PLL */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        /* wait */
    }

    /* 9) Manual core clock update: no SystemCoreClockUpdate() dependency */
    SystemCoreClock = SYSCLK_96MHZ;

    __DSB();
    __ISB();
}


/**
 * @brief Backward-compatible clock initialization wrapper.
 *
 * Calls clock_96_mhz_init().
 */
void clock_100_mhz_init(void)
{
    clock_96_mhz_init();
}
