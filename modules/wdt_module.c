/*
===============================================================================
 Name        : wdt_module.c
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Configure and use the Watchdog Timer
===============================================================================
*/
#include "env.h"
#include "wdt_module.h"

WDT_IRQ_ WDT_IRQ;

void WDT_init(WDT_IRQ_ WDT_IRQ_cb)
{
	uint32_t i;

	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 15); // Enable clock to WDT

	LPC_SYSCON->WDTOSCCTRL = 0x03F; 		// ~8kHz
	LPC_SYSCON->PDRUNCFG  &= ~(1 << 6);
/*
	LPC_SYSCON->WDTCLKSEL = 0x02;			// Select WDTOSC as clock source for WDT
	LPC_SYSCON->WDTCLKUEN = 0x01;			// Update clock
	LPC_SYSCON->WDTCLKUEN = 0x00;			// Toggle update register once
	LPC_SYSCON->WDTCLKUEN = 0x01;

	while (!(LPC_SYSCON->WDTCLKUEN & 0x01));// Wait until updated

	LPC_SYSCON->WDTCLKDIV = 1;				// WDT Clock Divider = 1
*/
#if WDT_ENABLE_IRQ==1
	NVIC_EnableIRQ(WDT_IRQn);
#endif

	LPC_WWDT->TC = WDT_FEED_VALUE;	// once WDEN is set, the WDT will start after feeding

#if WDT_WATCHDOG_RESET==1
	LPC_WWDT->MOD = (1 << 0) | (1 << 1);
#else
	LPC_WWDT->MOD = (1 << 0);
#endif

	LPC_WWDT->FEED = 0xAA;		// Feeding sequence
	LPC_WWDT->FEED = 0x55;

	for (i = 0; i < 0x80000; i++); // Make sure feed sequence executed properly

	LPC_WWDT->WARNINT = WDT_WARN_VALUE;

#if WDT_PROTECT_MODE==1
	LPC_WWDT->MOD = (1 << 0) | (1 << 4);
#endif

#if WDT_WINDOW_MODE==1
	LPC_WWDT->WINDOW = WDT_WINDOW_VALUE;

	while (1)
	{
		wdt_counter = LPC_WWDT->TV;
		while (wdt_counter >= 0x0000027F)
		{
			wdt_counter = LPC_WWDT->TV;
		}

		LPC_WWDT->FEED = 0xAA;		/* Feeding sequence */
		LPC_WWDT->FEED = 0x55;
		/* Make sure feed sequence executed properly */
		for (i = 0; i < 0x80000; i++);
	}
#endif

	WDT_IRQ = WDT_IRQ_cb;
}

inline uint8_t WDT_check(void)
{
	// Check if last reset was by Watchdog
	if (LPC_WWDT->MOD & (1 << 2))
		return 1;
	else
		return 0;
}

inline void WDT_clear(void)
{
	// clear the time-out flag and interrupt flag
	LPC_WWDT->MOD &= ~(1 << 2);
	LPC_WWDT->MOD &= ~(1 << 3);
}

inline void WDT_feed(void)
{
	LPC_WWDT->FEED = 0xAA;		/* Feeding sequence */
	LPC_WWDT->FEED = 0x55;
}

void WDT_IRQHandler(void)
{
	WDT_IRQ();
	WDT_clear();
}
