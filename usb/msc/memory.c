/*
===============================================================================
 Name        : main.c
 Author      :
 Version     :
 Copyright   : Copyright (C)
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC11Uxx.h"
#endif

#include <cr_section_macros.h>
#include <NXP/crp.h>

#include "type.h"
#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "mscuser.h"
#include "memory.h"
#include "gpio.h"
#include "config.h"
#ifdef CFG_SDCARD
#include "diskio.h"
#endif

// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

#ifdef CFG_SDCARD
volatile uint32_t mmcTicks = 0;

/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
  // Increment systick counter for Delay();
  msTicks++;

  // Increment SD card timer counter
  mmcTicks++;
  // Run MMC timer every 10 ms (mmc.c)
  if (mmcTicks == 10)
  {
    mmcTicks = 0;
    disk_timerproc();
  }
}

/*----------------------------------------------------------------------------
  No RTC on the LPC11U14 so return 0 for FATFS timestamp
 *----------------------------------------------------------------------------*/
DWORD get_fattime ()
{
  // The LPC11U14 has no RTC so just return 0 for the timestamp
  return 0;
}

/*----------------------------------------------------------------------------
  Tries to initialise the SD card using SSP0.  The code will hang if
  a card is not inserted (STA_NODISK) or if the init failed (STA_NOINIT)
 *----------------------------------------------------------------------------*/
void SDInit(void)
{
	// Initialise SD Card
	DSTATUS stat;
	stat = disk_initialize(0);
	if (stat & STA_NODISK)
	{
		// No SD Card Present ... hold
		while(1);
	}
	if (stat & STA_NOINIT)
	{
		// SD Init Failed ... hold
		while(1);
	}
}
#else
extern uint8_t Memory[MSC_MemorySize];           /* MSC Memory in RAM */
#endif // SDCARD_SUPPORT

/*----------------------------------------------------------------------------
  Program Entry Point
 *----------------------------------------------------------------------------*/
int main (void) {
  /* Setup SysTick Timer for 1 msec interrupts  */
  SysTick_Config(SystemCoreClock / 1000);

#ifdef CFG_SDCARD
  /* Setup SD Card/FATFS */
  SDInit();
#endif

  USB_Init();                               /* USB Initialization */
  USB_Connect(TRUE);                        /* USB Connect */

  // Set LED pin to output
  GPIOSetDir(CFG_LED_PORT, CFG_LED_PIN, 1);

  while (1)
  {
	  GPIOSetBitValue(CFG_LED_PORT, CFG_LED_PIN, 0);
	  Delay(500);
	  GPIOSetBitValue(CFG_LED_PORT, CFG_LED_PIN, 1);
	  Delay(500);
  }

  return 0;
}

