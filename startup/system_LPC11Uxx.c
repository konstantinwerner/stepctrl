/******************************************************************************
 * @file     system_LPC11Uxx.c
 * @purpose  CMSIS Cortex-M3 Device Peripheral Access Layer Source File
 *           for the NXP LPC13xx Device Series
 * @version  V1.10
 * @date     24. November 2010
 *
 * @note
 * Copyright (C) 2009-2010 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#include <stdint.h>
#include "LPC11Uxx.h"

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK;/*!< System Clock Frequency (Core Clock)*/


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
  uint32_t wdt_osc = 0;

  /* Determine clock frequency according to clock register values             */
  switch ((LPC_SYSCON->WDTOSCCTRL >> 5) & 0xF) {
    case 0:  wdt_osc =       0; break;
    case 1:  wdt_osc =  500000; break;
    case 2:  wdt_osc =  800000; break;
    case 3:  wdt_osc = 1100000; break;
    case 4:  wdt_osc = 1400000; break;
    case 5:  wdt_osc = 1600000; break;
    case 6:  wdt_osc = 1800000; break;
    case 7:  wdt_osc = 2000000; break;
    case 8:  wdt_osc = 2200000; break;
    case 9:  wdt_osc = 2400000; break;
    case 10: wdt_osc = 2600000; break;
    case 11: wdt_osc = 2700000; break;
    case 12: wdt_osc = 2900000; break;
    case 13: wdt_osc = 3100000; break;
    case 14: wdt_osc = 3200000; break;
    case 15: wdt_osc = 3400000; break;
  }
  wdt_osc /= ((LPC_SYSCON->WDTOSCCTRL & 0x1F) << 1) + 2;
 
  switch (LPC_SYSCON->MAINCLKSEL & 0x3) {
    case 0:                             /* Internal RC oscillator             */
      SystemCoreClock = __IRC_OSC_CLK;
      break;
    case 1:                             /* Input Clock to System PLL          */
      switch (LPC_SYSCON->SYSPLLCLKSEL & 0x3) {
          case 0:                       /* Internal RC oscillator             */
            SystemCoreClock = __IRC_OSC_CLK;
            break;
          case 1:                       /* System oscillator                  */
            SystemCoreClock = __SYS_OSC_CLK;
            break;
          case 2:                       /* Reserved                           */
          case 3:                       /* Reserved                           */
            SystemCoreClock = 0;
            break;
      }
      break;
    case 2:                             /* WDT Oscillator                     */
      SystemCoreClock = wdt_osc;
      break;
    case 3:                             /* System PLL Clock Out               */
      switch (LPC_SYSCON->SYSPLLCLKSEL & 0x3) {
          case 0:                       /* Internal RC oscillator             */
            if (LPC_SYSCON->SYSPLLCTRL & 0x180) {
              SystemCoreClock = __IRC_OSC_CLK;
            } else {
              SystemCoreClock = __IRC_OSC_CLK * ((LPC_SYSCON->SYSPLLCTRL & 0x1F) + 1);
            }
            break;
          case 1:                       /* System oscillator                  */
            if (LPC_SYSCON->SYSPLLCTRL & 0x180) {
              SystemCoreClock = __SYS_OSC_CLK;
            } else {
              SystemCoreClock = __SYS_OSC_CLK * ((LPC_SYSCON->SYSPLLCTRL & 0x1F) + 1);
            }
            break;
          case 2:                       /* Reserved                           */
          case 3:                       /* Reserved                           */
            SystemCoreClock = 0;
            break;
      }
      break;
  }

  SystemCoreClock /= LPC_SYSCON->SYSAHBCLKDIV;  

}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void) {
  volatile uint32_t i;

#if (CLOCK_SETUP)                                 /* Clock Setup              */

#if ((SYSPLLCLKSEL_Val & 0x3) == 1)
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 5);          /* Power-up System Osc      */
  LPC_SYSCON->SYSOSCCTRL    = SYSOSCCTRL_Val;
  for (i = 0; i < 200; i++) __NOP();
#endif

  LPC_SYSCON->SYSPLLCLKSEL  = SYSPLLCLKSEL_Val;   /* Select PLL Input         */
  LPC_SYSCON->SYSPLLCLKUEN  = 0x1;               /* Update Clock Source      */
  LPC_SYSCON->SYSPLLCLKUEN  = 0x0;               /* Toggle Update Register   */
  LPC_SYSCON->SYSPLLCLKUEN  = 0x1;
  while (!(LPC_SYSCON->SYSPLLCLKUEN & 0x1));     /* Wait Until Updated       */
#if ((MAINCLKSEL_Val & 0x3) == 3)                /* Main Clock is PLL Out    */
  LPC_SYSCON->SYSPLLCTRL    = SYSPLLCTRL_Val;
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 7);          /* Power-up SYSPLL          */
  while (!(LPC_SYSCON->SYSPLLSTAT & 0x1));	      /* Wait Until PLL Locked    */
#endif

#if (((MAINCLKSEL_Val & 0x3) == 2) )
  LPC_SYSCON->WDTOSCCTRL    = WDTOSCCTRL_Val;
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 6);          /* Power-up WDT Clock       */
  for (i = 0; i < 200; i++) __NOP();
#endif

  LPC_SYSCON->MAINCLKSEL    = MAINCLKSEL_Val;     /* Select PLL Clock Output  */
  LPC_SYSCON->MAINCLKUEN    = 0x1;               /* Update MCLK Clock Source */
  LPC_SYSCON->MAINCLKUEN    = 0x0;               /* Toggle Update Register   */
  LPC_SYSCON->MAINCLKUEN    = 0x1;
  while (!(LPC_SYSCON->MAINCLKUEN & 0x1));       /* Wait Until Updated       */

  LPC_SYSCON->SYSAHBCLKDIV  = SYSAHBCLKDIV_Val;

#if ((USBCLKDIV_Val & 0x1FF) != 0)                /* USB clock is used        */
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 10);         /* Power-up USB PHY         */

#if ((USBCLKSEL_Val & 0x3) == 0)                /* USB clock is USB PLL out */
  LPC_SYSCON->PDRUNCFG     &= ~(1 << 8);         /* Power-up USB PLL         */
  LPC_SYSCON->USBPLLCLKSEL  = USBPLLCLKSEL_Val;   /* Select PLL Input         */
  LPC_SYSCON->USBPLLCLKUEN  = 0x1;               /* Update Clock Source      */
  LPC_SYSCON->USBPLLCLKUEN  = 0x0;               /* Toggle Update Register   */
  LPC_SYSCON->USBPLLCLKUEN  = 0x1;
  while (!(LPC_SYSCON->USBPLLCLKUEN & 0x1));     /* Wait Until Updated       */
  LPC_SYSCON->USBPLLCTRL    = USBPLLCTRL_Val;
  while (!(LPC_SYSCON->USBPLLSTAT & 0x1));     /* Wait Until PLL Locked    */
  LPC_SYSCON->USBCLKSEL     = 0x0;               /* Select USB PLL           */
#endif

  LPC_SYSCON->USBCLKSEL     = USBCLKSEL_Val;      /* Select USB Clock         */
  LPC_SYSCON->USBCLKDIV     = USBCLKDIV_Val;      /* Set USB clock divider    */

#else                                             /* USB clock is not used    */                        
  LPC_SYSCON->PDRUNCFG     |=  (1 << 10);         /* Power-down USB PHY       */
  LPC_SYSCON->PDRUNCFG     |=  (1 <<  8);         /* Power-down USB PLL       */
#endif

#endif

  /* System clock to the IOCON needs to be enabled or
  most of the I/O related peripherals won't work. */
  LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);

}
