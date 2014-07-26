/*
===============================================================================
 Name        : delay_module.c
 Author      : Konstantin Werner
 Version     : 0.1
 Copyright   : Copyright (C) Konstantin Werner
 Description : Uses a Timer for Delays
===============================================================================
*/

#include "env.h"
#include "delay_module.h"

#if DELAY_TIMER_NR == 1
	#define DELAY_TIMER_CLK_BIT	10
	#define DELAY_TIMER			LPC_CT32B1
#endif

#if DELAY_TIMER_NR == 0
	#define DELAY_TIMER_CLK_BIT	9
	#define DELAY_TIMER			LPC_CT32B0
#endif

#define MHZ_PRESCALE		(CPU_CLK / 1000000)
#define KHZ_PRESCALE		(CPU_CLK / 1000)

uint8_t delay_initialized = 0;

void delay_init(void)	// MUST be initialized before any function that uses delays
{
	if (!delay_initialized)
	{
		LPC_SYSCON->SYSAHBCLKCTRL |= (1 << DELAY_TIMER_CLK_BIT);
		DELAY_TIMER->PR = MHZ_PRESCALE;
		DELAY_TIMER->TC	= 0;
		DELAY_TIMER->TCR = 1;

		delay_initialized = 1;
	}
}

inline void delay_us(uint32_t delay)
{
	uint32_t start = DELAY_TIMER->TC;

	while((DELAY_TIMER->TC - start) < delay);
}

inline void delay_ms(uint32_t delay)
{
	uint32_t start = DELAY_TIMER->TC;

	while ((DELAY_TIMER->TC - start) < (delay * 1000));
}

uint32_t micros(void)
{
	return DELAY_TIMER->TC;
}
