/*
===============================================================================
 Name        : wdt_module.h
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Configure and use the Watchdog Timer
===============================================================================
*/

#ifndef WDT_MODULE_H_
#define WDT_MODULE_H_

#include "type.h"

#define WDT_ENABLE_WARN_IRQ		0					// Enable Watchdog Interrupt
#define	WDT_WATCHDOG_RESET		0					// Watchdog Event causes Reset
#define WDT_PROTECT_MODE		0					// Watchdog Timeout is protected from changes by software
#define WDT_WINDOW_MODE			0					// Watchdog is windowed. Early feed generates reset.

#define WDT_TIMEOUT_MS			100					// Time in Milliseconds for WatchDog Timeout (Reset with WDT_FEED())
#define WDT_TIMEWARN_MS			10					// Time in Milliseconds for WatchDog Warning Interrupt before Timeout occurs
#define WDT_WINDOW_MS			50					// Time in Milliseconds for WatchDog Window

#define WDT_FEED_VALUE			(8 * WDT_TIMEOUT_MS)	// WatchDogTimer runs at 8kHz
#define WDT_WARN_VALUE			(8 * WDT_TIMEWARN_MS)
#define WDT_WINDOW_VALUE		(8 * WDT_WINDOW_MS)

typedef void (*WDT_IRQ_) (void);	// Function Pointer for WDT IRQ Callback

void WDT_init(WDT_IRQ_ WDT_IRQ_cb);
void WDT_feed(void);
void WDT_clear(void);
uint8_t WDT_check(void);

#endif /* WDT_MODULE_H_ */
