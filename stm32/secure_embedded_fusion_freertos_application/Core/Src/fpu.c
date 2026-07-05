/* Author: i fiksuari pas prog
 * Documentation date: 2026-07-03
 */

#include "stm32f4xx.h"


/* Enables access to the Cortex-M4 floating-point coprocessors. */
void fpu_enable(void)
{
	/* Grant full access to both CP10 and CP11. */
	SCB->CPACR |=(1<<20);
	SCB->CPACR |=(1<<21);
	SCB->CPACR |=(1<<22);
	SCB->CPACR |=(1<<23);

}
