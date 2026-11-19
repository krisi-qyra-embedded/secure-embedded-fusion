#include "stm32f4xx.h"

/**
 * @brief Enable the floating point unit.
 */
void fpu_enable(void)
{
	/* Enable full access to CP10 and CP11. */
	SCB->CPACR |=(1<<20);
	SCB->CPACR |=(1<<21);
	SCB->CPACR |=(1<<22);
	SCB->CPACR |=(1<<23);
}
