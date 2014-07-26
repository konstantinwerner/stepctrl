/*
===============================================================================
 Name        : config.h
 Author      :
 Version     :
 Copyright   : Copyright (C)
 Description : Common config settings
===============================================================================
*/
#ifndef __CONFIG_H
#define __CONFIG_H

//#define CFG_MEMORY
#define CFG_SDCARD
#if !defined(CFG_MEMORY) && !defined(CFG_SDCARD)
#error "ERROR: No configuration defined"
#endif

#define BOARD_EA_REVB
//#define BOARD_NGX
#if !defined(BOARD_EA_REVB) && !defined(BOARD_NGX)
#error "ERROR: No board defined"
#endif


// SD CARD uses SPI with the following pins
// MISO = 0.8
// MOSI = 0.9
// SCK =  1.29

// SPI Select Port/Pin for SD CARD
#ifdef BOARD_EA_REVB
#define CFG_CSPORT (0)
#define CFG_CSPIN  (23)
#endif
#ifdef BOARD_NGX
#define CFG_CSPORT (0)
#define CFG_CSPIN  (2)
#endif

// Card Detect Port/Pin for SD CARD
#define CFG_SDCARD_CDPORT (0)
#define CFG_SDCARD_CDPIN  (12)

// LED Port/Pin (for quick debugging)
#define CFG_LED_PORT (0)
#define CFG_LED_PIN (7)

#endif
