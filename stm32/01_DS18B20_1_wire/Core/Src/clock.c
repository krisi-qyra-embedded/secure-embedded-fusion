/* clock.c | Author: i fiksuari pas prog | Date: 22 February 2026 */


#include"clock.h"

//system clock source = src(enble HSE oscillator)
//wait hsready goes low
//sysclk              = 100Mhz
//hclk                - 100Mhz
//ahbprescaler        = 1
//apb1prescaler       = 2
//apb2prescaler       = 1
//HSE                 = 8Mhz



void clock_100_mhz_init(void)
{
	// Enable HSE oscillator
		    RCC->CR |= RCC_CR_HSEON;
		    // Wait till HSE is ready
		    while(!(RCC->CR & RCC_CR_HSERDY)) {}
		    // Set the PLL configuration
		    RCC->PLLCFGR = (PLL_M << RCC_PLLCFGR_PLLM_Pos) |
		                   (PLL_N << RCC_PLLCFGR_PLLN_Pos) |
		                   (((PLL_P >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos) |
		                   (RCC_PLLCFGR_PLLSRC_HSE) |
		                   (PLL_Q << RCC_PLLCFGR_PLLQ_Pos);
		    // Enable the PLL
		    RCC->CR |= RCC_CR_PLLON;
		    // Wait till PLL is ready
		    while(!(RCC->CR & RCC_CR_PLLRDY)) {}
		    // Configure the Flash latency
		    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
		    // Select the PLL as system clock source
		    RCC->CFGR |= (RCC_CFGR_SW_PLL);
		    // Wait till the PLL is used as the system clock source
		    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
		    // Optional: Update the SystemCoreClock variable (defined in system_stm32f4xx.c)
		    SystemCoreClockUpdate();
}

