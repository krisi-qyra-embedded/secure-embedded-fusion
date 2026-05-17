/* Author: i fiksuari pas prog | Documentation date: 17 May 2026 */
#include "stm32f4xx.h"



/* Handles the fpu enable operation. */
void fpu_enable(void)
{
	/*Enable floating point unit:  Enable CP10 and CP11 full access*/
	SCB->CPACR |=(1<<20);
	SCB->CPACR |=(1<<21);
	SCB->CPACR |=(1<<22);
	SCB->CPACR |=(1<<23);

}
