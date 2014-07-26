/*
===============================================================================
 Name        : ssp_module.h
 Author      : Konstantin Werner
 Version     : 0.1a
 Copyright   : Copyright (C) Konstantin Werner
 Description : SSP Communication
===============================================================================
*/

#ifndef __SSP_MODULE_H__
#define __SSP_MODULE_H__

#include "type.h"

#define SSP_MASTER		0
#define SSP_SLAVE		1

#define SSP0_MODE	SSP_MASTER
#define SSP1_MODE	SSP_MASTER

#define LOOPBACK_MODE_0	0		// 1 = loopback, 0 = normal operation
#define LOOPBACK_MODE_1	0		// 1 = loopback, 0 = normal operation

/* If USE_CS is zero, SSEL is GPIO and must be controlled by the user */
/* When LOOPBACK_MODE = 1 set USE_CS to 1. */

#define USE_CS_0		1
#define USE_CS_1		0

// SPI read and write buffer size
#define SSP_BUFSIZE		16
#define FIFOSIZE		8

#define DELAY_COUNT		10
#define MAX_TIMEOUT		0xFF

#define FF_SPI			0x00
#define FF_TI			0x01
#define FF_MICROWIRE	0x02

#define SSP0_DATA_SIZE		8		// Data per Frame in Bits
#define SSP0_FRAME_FORMAT	FF_SPI	// FF_SPI, FF_TI, FF_MICROWIRE
#define SSP0_CPOL			0
#define SSP0_CPHA			0
#define SSP0_SCR			16		// Number of Prescaler Clocks per Bit on the Bus

#define SSP1_DATA_SIZE		8		// Data per Frame in Bits
#define SSP1_FRAME_FORMAT	FF_SPI
#define SSP1_CPOL			0
#define SSP1_CPHA			0
#define SSP1_SCR			16		// Number of Prescaler Clocks per Bit on the Bus

// SSP0 IO Config
#define SCK0_LOC	PIN0_6		// Options : PIN0_6, PIN0_10 (only with _JTAG_DISABLED), PIN1_29

// SSP1 IO Config
#define	SSEL1_LOC	PIN1_23		// Options : PIO1_19, PIO1_23
#define SCK1_LOC	PIN1_20		// Options : PIN1_15, PIN1_20
#define MISO1_LOC	PIN0_22		// Options : PIN1_21, PIN0_22
#define MOSI1_PIN	PIO0_21		// Options : PIO0_21, PIO1_22

#define CS_LOW			0
#define CS_HIGH			1

/* If RX_INTERRUPT is enabled, the SSP RX will be handled in the ISR, SSPReceive() will not be needed. */
void SSP_init(uint8_t port);
void SSP_send(uint8_t port, uint32_t size, uint8_t *data);
void SSP_receive(uint8_t port, uint32_t size, uint8_t *buf);

#if USE_CS_0 == 0
void SSP0_CS(uint8_t cs);
#endif

#if USE_CS_1 == 0
void SSP1_CS(uint8_t cs);
#endif

#endif  /* __SSP_MODULE_H__ */
