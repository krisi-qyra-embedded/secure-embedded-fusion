/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

/*
 * clock.c
 *
 * 96 MHz system clock from 8 MHz HSE for STM32F411 / STM32F4xx.
 *
 * Clock tree:
 *   HSE   = 8 MHz
 *   PLL_M = 8
 *   PLL_N = 192
 *   PLL_P = 2
 *   PLL_Q = 4
 *
 * Result:
 *   VCO_IN  = 8 / 8   = 1 MHz
 *   VCO_OUT = 1 * 192 = 192 MHz
 *   SYSCLK  = 192 / 2 = 96 MHz
 *   HCLK    = 96 MHz
 *   APB1    = 48 MHz   (HCLK / 2)
 *   APB2    = 96 MHz   (HCLK / 1)
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

/* Configures Flash access for the 96 MHz clock. */
static void clock_set_flash_latency_96mhz(void)
{
    /* Wait for the three-wait-state setting to read back. */
    FLASH->ACR =
          FLASH_ACR_ICEN
        | FLASH_ACR_DCEN
        | FLASH_ACR_PRFTEN
        | FLASH_ACR_LATENCY_3WS;

    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS) {
    }
}

/* Switches the system clock to the HSE-derived 96 MHz PLL. */
void clock_96_mhz_init(void)
{
    /* Set Flash latency and APB dividers before selecting the PLL clock. */
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
    }

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;

    clock_set_flash_latency_96mhz();

    /* 4) Configure bus prescalers
     *    AHB  = SYSCLK / 1  = 96 MHz
     *    APB1 = HCLK   / 2  = 48 MHz
     *    APB2 = HCLK   / 1  = 96 MHz
     */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |=  RCC_CFGR_HPRE_DIV1
               |  RCC_CFGR_PPRE1_DIV2
               |  RCC_CFGR_PPRE2_DIV1;

    if ((RCC->CR & RCC_CR_PLLON) != 0U) {
        RCC->CR &= ~RCC_CR_PLLON;
        while ((RCC->CR & RCC_CR_PLLRDY) != 0U) {
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

    SystemCoreClock = SYSCLK_96MHZ;

    __DSB();
    __ISB();
}

/* Calls the retained 96 MHz clock setup entry point. */
void clock_100_mhz_init(void)
{
    /* The legacy function name delegates to the 96 MHz configuration. */
    clock_96_mhz_init();
}
