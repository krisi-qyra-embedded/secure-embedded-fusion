/* systick.c | Author: i fiksuari pas prog | Date: 17 February 2026 */
#include"stm32f4xx.h"
#include"systick.h"

/*
 * systick.c
 *
 *  Created on: May 10, 2022
 *      Author: i fiksuari mbas prog
 */
#define SYSTICK_LOAD_VAL         16000000                   //our deafoult clock is 16MHZ 16000000 MEANING 16000000 HAPPENS IN A SECOND WHEREAS 16000 cycles happen
#define CTRL_ENABLE                 (1U << 0)              //in a millisecond becouse our goal is to delay for 1 ms
#define CTRL_CLKSRC                 (1U << 2)
#define CTRL_COUNTFLAG          (1U << 16)


/* Implements the systicDelayMs operation. */
void systicDelayMs(int delay)
{
	//Configure systick
	//Reload with numbers of clocks per milliseconds
	SysTick->LOAD = SYSTICK_LOAD_VAL;

	//clear current systick val register
	SysTick->VAL = 0;
	//enable sustick and select internal clock source
	SysTick->CTRL = (CTRL_ENABLE | CTRL_CLKSRC);
for(int i=0;i<delay;i++)
{
	//wait until the countflag is set
	while(!(SysTick->CTRL & CTRL_COUNTFLAG)){}
}
SysTick->CTRL  = 0;
}

