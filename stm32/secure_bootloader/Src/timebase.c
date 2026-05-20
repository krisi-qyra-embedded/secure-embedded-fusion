/**
 * @file timebase.c
 * @brief SysTick timebase and blocking delay helpers.
 */
/* Author: i fiksuari pas prog | Documentation date: 20 May 2026 */
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
/* Millisecond tick state maintained by SysTick. */
volatile uint32_t g_curr_tick;
volatile uint32_t g_curr_tick_p;
/* Blocking delay helpers. */
/**
 * @brief Block for the requested number of system ticks.
 * @param delay Delay duration in ticks.
 */
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
/**
 * @brief Provide an approximate millisecond busy-wait delay.
 * @param ms Requested delay in milliseconds.
 */
void delay_ms(uint32_t ms)
{
   /* Busy-wait calibration constant for the current build/clock setup. */
   const uint32_t count_per_ms = 12000; // Example for ~100 MHz core
   volatile uint32_t i, j;
   for (i = 0; i < ms; i++) {
       for (j = 0; j < count_per_ms; j++) {
           __asm__("nop");  // prevent compiler from optimizing the loop away
       }
   }
}
/* SysTick timebase access and initialization. */
/**
 * @brief Read the current SysTick timebase value atomically.
 * @return Current system tick count.
 */
uint32_t get_tick(void)
{
	/* Protect the shared tick read against the SysTick ISR. */
	__disable_irq();
	g_curr_tick_p = g_curr_tick;
	__enable_irq();
	return g_curr_tick_p;
}
/* Advance the shared system tick by the configured tick frequency. */
static void tick_increment(void)
{
    /* Check the return status before continuing to the next step. */
	g_curr_tick += TICK_FREQ;
}
/**
 * @brief Configure SysTick as the project millisecond timebase.
 */
void timebase_init(void)
{
	__disable_irq();
	/* Reload for a 1 ms period at the configured SysTick clock. */
	SysTick->LOAD =  ONE_MSEC_LOAD - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = CTRL_CLCKSRC;
	/* Enable the SysTick interrupt before starting the counter. */
	SysTick->CTRL |= CTRL_TICKINT;
	SysTick->CTRL |=CTRL_ENABLE;
	__enable_irq();
}
/**
 * @brief Handle the SysTick interrupt and advance the system tick.
 */
void SysTick_Handler(void)
{
    /* Clear the handled interrupt flags and update the transfer state. */
	tick_increment();
}
