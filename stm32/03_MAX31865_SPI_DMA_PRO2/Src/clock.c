/* clock.c 96 MHz system clock from 8 MHz HSE for STM32F411 / STM32F4xx. */

#include "clock.h"
#include "stm32f4xx.h"
#include <stdint.h>

/* Some projects do not compile/link system_stm32f4xx.c, which normally defines SystemCoreClock. */
__attribute__((weak)) uint32_t SystemCoreClock = 16000000UL;

/* Accesses the requested setting through clock set flash latency 96mhz. */
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

/* Initializes the resources used by clock 96 mhz init. */
void clock_96_mhz_init(void)
{
    /* Set up the required peripheral configuration. */
     
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
        /* wait */
    }

    /* 2) Enable power interface clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;

    /* 3) Configure Flash for high-speed operation before switching clock */
    clock_set_flash_latency_96mhz();

     
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |=  RCC_CFGR_HPRE_DIV1
               |  RCC_CFGR_PPRE1_DIV2
               |  RCC_CFGR_PPRE2_DIV1;

     
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
        /* wait */
    }

     
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

 
/* Initializes the resources used by clock 100 mhz init. */
void clock_100_mhz_init(void)
{
    /* Set up the required peripheral configuration. */
    clock_96_mhz_init();
}
