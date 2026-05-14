/* Author: i fiksuari pas prog | Documentation date: 14 May 2026 */
#include "timebase.h"
#include "stm32f4xx.h"

#define CTRL_ENABLE		(1U<<0)
#define CTRL_TICKINT	(1U<<1)
#define CTRL_CLCKSRC	(1U<<2)
#define CTRL_COUNTFLAG	(1U<<16)

#define ONE_SEC_LOAD	 16000000
#define ONE_MSEC_LOAD	 16000

#define MAX_DELAY		 0xFFFFFFFF

#define TICK_FREQ		 1

volatile uint32_t g_curr_tick;
volatile uint32_t g_curr_tick_p;


/*Delay in seconds*/
void delay(uint32_t delay)
{
    /* Keep the wait bounded to avoid a stalled execution path. */
	uint32_t tickstart =  get_tick();
	uint32_t wait =  delay;

	if(wait < MAX_DELAY)
	{
		wait += (uint32_t)TICK_FREQ;
	}

    while((get_tick() - tickstart) < wait){}

}

/* Handles the get tick operation. */
uint32_t get_tick(void)
{
    /* Check the return status before continuing to the next step. */
	__disable_irq();
	g_curr_tick_p = g_curr_tick;
	__enable_irq();

	return g_curr_tick_p;

}
/* Handles the tick increment operation. */
static void tick_increment(void)
{
    /* Check the return status before continuing to the next step. */
	g_curr_tick += TICK_FREQ;
}
/* Configures the module for its next operation. */
void timebase_init(void)
{

	/*Disable global interrupts*/
	__disable_irq();

    /*Load the timer with number of clock cycles per msecond*/
	SysTick->LOAD =  ONE_MSEC_LOAD - 1;

    /*Clear systick current value register*/
	SysTick->VAL = 0;

    /*Select internal clock source*/
	SysTick->CTRL = CTRL_CLCKSRC;

    /*Enable interrupt*/
	SysTick->CTRL |= CTRL_TICKINT;

	/*Enable systick*/
	SysTick->CTRL |=CTRL_ENABLE;

	/*Enable global interrupts*/
	__enable_irq();
}

/* Handles the related interrupt and transfer status. */
void SysTick_Handler(void)
{
    /* Clear the handled interrupt flags and update the transfer state. */
	tick_increment();
}
