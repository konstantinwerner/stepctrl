/*----------------------------------------------------------------------------
 * Name:    memory.h
 * Purpose: USB Memory Storage Demo Definitions
 * Version: V1.20
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC microcontroller devices only. Nothing else 
 *      gives you the right to use this software.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/
#ifndef __MEMORY_H
#define __MEMORY_H

/* MSC Disk Image Definitions */
//#define MSC_ImageSize   0x00001000
//extern const unsigned char DiskImage[MSC_ImageSize];   /* Disk Image */

volatile uint32_t msTicks;                             /* counts 1ms timeTicks */
volatile uint32_t blockCount;
volatile uint32_t blockSize;

/*------------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *------------------------------------------------------------------------------*/
static inline void Delay (uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

#endif
