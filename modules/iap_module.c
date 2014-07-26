/*
===============================================================================
 Name        : iap_module.c
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Provide access to the IAP Functions, especially to reinvoking
 	 	 	   the bootloader correctly
===============================================================================
*/

#include <stdarg.h>
#include "env.h"
#include "iap_module.h"

void IAP_InvokeBootloader(void)
{
    // Make sure that you define command[] and  result [] as static if their locally declared or just declared them globally.
    // This is because by the time you call iap_entry(command, result) the stack pointer would have already been modified by you
    // and your code try to pass command[] and result[] to iap_entry() which won't be valid. It will try to pass data from an unexpected
    // location in RAM where the data could be anything.
    // iap_entry will most likely return (when you were expecting it not to) and causing the function that called it to fail
    // or possibly causing the CPU to rise a hard fault due to an invalid return pointer or from the stack corruption.
    static unsigned long command[5] = {57UL, 0UL, 0UL, 0UL, 0UL};
    static unsigned long result[4];

//    LPC_SYSCON->WDTCLKDIV = 0;	// Disable Watchdog Clock

    __disable_irq();

    // Make sure 32-bit Timer 1 is turned on before calling ISP
    LPC_SYSCON->SYSAHBCLKCTRL |= 0x00400;
    // Make sure GPIO clock is turned on before calling ISP
    LPC_SYSCON->SYSAHBCLKCTRL |= 0x00040;
    // Make sure IO configuration clock is turned on before calling ISP
    LPC_SYSCON->SYSAHBCLKCTRL |= 0x10000;

    // make sure AHB clock divider is 1:1
    LPC_SYSCON->SYSAHBCLKDIV = 1;

    // Disable the PLL
    LPC_SYSCON->MAINCLKSEL 	  = 0; 					// Enable the IRC oscillator
    LPC_SYSCON->MAINCLKUEN    = 0x01;               /* Update MCLK Clock Source */
    LPC_SYSCON->MAINCLKUEN    = 0x00;               /* Toggle Update Register   */
    LPC_SYSCON->MAINCLKUEN    = 0x01;
    while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       /* Wait Until Updated       */

    // Reset the stack pointer to point to the top of the stack minus the 32 byte for IAP
    // for LPC11Cxx where top of RAM is 0x1000_2000; MSP = (0x1000_2000 - 0x20) = 0x10001FE0
    __set_MSP(0x10001FE0);
    // The Cortex M0 needs to synchronize the the CPU cache
    __ISB();

    // Invoke the bootloader (iap_entry defined according to datasheet)
    IAP_cmd(command, result);
}


void IAP_cmd_(unsigned long *result, uint8_t length, ...)
{
	unsigned long command[5];

	va_list args;
	uint8_t byte;

	if (length > 5) length = 5;

	va_start(args, length);

	for (byte = 0; byte < length; byte++)
		command[byte] = (uint8_t) va_arg(args, int);
	for (byte = length; byte < 5; byte++)
		command[byte] = 0x00000000UL;

	__disable_irq();
	IAP_cmd(command, result);
	__enable_irq();
}

