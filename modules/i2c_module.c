/*
===============================================================================
 Name        : i2c_module.c
 Author      : Konstantin Werner
 Version     : 0.1
 Copyright   : Copyright (C) Konstantin Werner
 Description : Functions for communicating with I2C Slaves
===============================================================================
*/
#include "env.h"

#include "i2c_module.h"
#include <stdarg.h>

#define I2C_IDLE				0
#define I2C_STARTED				1
#define I2C_RESTARTED			2
#define I2C_REPEATED_START		3
#define DATA_ACK				4
#define DATA_NACK				5
#define I2C_BUSY              	6
#define I2C_NO_DATA           	7
#define I2C_NACK_ON_ADDRESS   	8
#define I2C_NACK_ON_DATA      	9
#define I2C_ARBITRATION_LOST  	10  // 0x0A
#define I2C_TIME_OUT          	11	// 0x0B
#define I2C_OK                	12	// 0x0C

#define I2CONSET_I2EN       (1 << 6)  /* I2C Control Set Register */
#define I2CONSET_AA         (1 << 2)
#define I2CONSET_SI         (1 << 3)
#define I2CONSET_STO        (1 << 4)
#define I2CONSET_STA        (1 << 5)

#define I2CONCLR_AAC        (1 << 2)  /* I2C Control clear Register */
#define I2CONCLR_SIC        (1 << 3)
#define I2CONCLR_STAC       (1 << 5)
#define I2CONCLR_I2ENC      (1 << 6)

#define I2DAT_I2C			0x00000000  /* I2C Data Reg */
#define I2ADR_I2C			0x00000000  /* I2C Slave Address Reg */
#define I2SCLH_SCLH			0x00000180  /* I2C SCL Duty Cycle High Reg */
#define I2SCLL_SCLL			0x00000180  /* I2C SCL Duty Cycle Low Reg */
#define I2SCLH_HS_SCLH		0x00000015  /* Fast Plus I2C SCL Duty Cycle High Reg */
#define I2SCLL_HS_SCLL		0x00000015  /* Fast Plus I2C SCL Duty Cycle Low Reg */

volatile uint32_t I2CCount;
volatile uint8_t  I2CMasterBuffer[BUFFERSIZE];
volatile uint8_t  I2CSlaveBuffer[BUFFERSIZE];
volatile uint32_t I2CMasterState = I2C_IDLE;
volatile uint32_t I2CReadLength, I2CWriteLength;
volatile uint32_t timeout = 0;

volatile uint32_t RdIndex = 0;
volatile uint32_t WrIndex = 0;

uint8_t i2c_initialized = 0;

/* I2C Interface Initialisation */
void I2C_init(void)
{
	if (!i2c_initialized)
	{
		LPC_SYSCON->PRESETCTRL	  |= (1 << 1);
		LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 5);

		LPC_IOCON->PIO0_4 &= ~0x3F;		/* I2C I/O config */
		LPC_IOCON->PIO0_4 |=  0x01;		/* I2C SCL */
		LPC_IOCON->PIO0_5 &= ~0x3F;
		LPC_IOCON->PIO0_5 |=  0x01;		/* I2C SDA */

		/* IOCON may change in the next release, save change for future references. */
		//  LPC_IOCON->PIO0_4 |= (1 << 10);	/* open drain pins */
		//  LPC_IOCON->PIO0_5 |= (1 << 10);	/* open drain pins */

		/* Clear flags */
		LPC_I2C->CONCLR = I2CONCLR_AAC | I2CONCLR_SIC | I2CONCLR_STAC | I2CONCLR_I2ENC;

		/* Reset registers */
		#if FAST_MODE_PLUS
		LPC_IOCON->PIO0_4 |= (2 << 8);
		LPC_IOCON->PIO0_5 |= (2 << 8);
		LPC_I2C->SCLL   = I2SCLL_HS_SCLL;
		LPC_I2C->SCLH   = I2SCLH_HS_SCLH;
		#else
		LPC_I2C->SCLL   = I2SCLL_SCLL;
		LPC_I2C->SCLH   = I2SCLH_SCLH;
		#endif

		NVIC_SetPriority(I2C_IRQn, 0); // Higher than USB Interrupt
		/* Enable the I2C Interrupt */
		NVIC_EnableIRQ(I2C_IRQn);

		LPC_I2C->CONSET = I2CONSET_I2EN;

		i2c_initialized = 1;
	}
}

uint32_t I2C_Engine(void)
{
	RdIndex = 0;
	WrIndex = 0;

	LPC_I2C->CONSET = I2CONSET_STA;	// Set Start flag

	I2CMasterState = I2C_BUSY;

	while (I2CMasterState == I2C_BUSY)
	{
		if (timeout >= MAX_TIMEOUT)
		{
			I2CMasterState = I2C_TIME_OUT;
			break;
		}
		timeout++;
	}

	LPC_I2C->CONCLR = I2CONCLR_STAC;

	return (I2CMasterState);
}

/* This handler deals with master read and master write only */
void I2C_IRQHandler(void)
{
	uint8_t StatValue;

	timeout = 0;

	StatValue = LPC_I2C->STAT;

	switch (StatValue)
	{
		case 0x08:			/* A Start condition is issued. */
			WrIndex = 0;
			LPC_I2C->DAT = I2CMasterBuffer[WrIndex++];
			LPC_I2C->CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
			break;

		case 0x10:			/* A repeated started is issued */
			RdIndex = 0;
			/* Send SLA with R bit set, */
			LPC_I2C->DAT = I2CMasterBuffer[WrIndex++];
			LPC_I2C->CONCLR = (I2CONCLR_SIC | I2CONCLR_STAC);
			break;

		case 0x18:			/* Regardless, it's a ACK */
			if ( I2CWriteLength == 1 )
			{
				LPC_I2C->CONSET = I2CONSET_STO;      /* Set Stop flag */
				I2CMasterState = I2C_NO_DATA;
			}
			else
			{
				LPC_I2C->DAT = I2CMasterBuffer[WrIndex++];
			}
			LPC_I2C->CONCLR = I2CONCLR_SIC;
			break;

		case 0x28:	/* Data byte has been transmitted, regardless ACK or NACK */
			if ( WrIndex < I2CWriteLength )
			{
				LPC_I2C->DAT = I2CMasterBuffer[WrIndex++]; /* this should be the last one */
			}
			else
			{
				if ( I2CReadLength != 0 )
				{
					LPC_I2C->CONSET = I2CONSET_STA;	/* Set Repeated-start flag */
				}
				else
				{
					LPC_I2C->CONSET = I2CONSET_STO;      /* Set Stop flag */
					I2CMasterState = I2C_OK;
				}
			}
			LPC_I2C->CONCLR = I2CONCLR_SIC;
			break;

		case 0x30:
			LPC_I2C->CONSET = I2CONSET_STO;      /* Set Stop flag */
			I2CMasterState = I2C_NACK_ON_DATA;
			LPC_I2C->CONCLR = I2CONCLR_SIC;
			break;

		case 0x40:	/* Master Receive, SLA_R has been sent */
			if ( (RdIndex + 1) < I2CReadLength )
			{
				/* Will go to State 0x50 */
				LPC_I2C->CONSET = I2CONSET_AA;	/* assert ACK after data is received */
			}
			else
			{
				/* Will go to State 0x58 */
				LPC_I2C->CONCLR = I2CONCLR_AAC;	/* assert NACK after data is received */
			}
			LPC_I2C->CONCLR = I2CONCLR_SIC;
			break;

		case 0x50:	/* Data byte has been received, regardless following ACK or NACK */
			I2CSlaveBuffer[RdIndex++] = LPC_I2C->DAT;
			if ( (RdIndex + 1) < I2CReadLength )
			{
				LPC_I2C->CONSET = I2CONSET_AA;	/* assert ACK after data is received */
			}
			else
			{
				LPC_I2C->CONCLR = I2CONCLR_AAC;	/* assert NACK on last byte */
			}
			LPC_I2C->CONCLR = I2CONCLR_SIC;
			break;

		case 0x58:
			I2CSlaveBuffer[RdIndex++] = LPC_I2C->DAT;
			I2CMasterState = I2C_OK;
			LPC_I2C->CONSET = I2CONSET_STO;	/* Set Stop flag */
			LPC_I2C->CONCLR = I2CONCLR_SIC;	/* Clear SI flag */
			break;

		case 0x20:		/* regardless, it's a NACK */
		case 0x48:
			LPC_I2C->CONSET = I2CONSET_STO;      /* Set Stop flag */
			I2CMasterState = I2C_NACK_ON_ADDRESS;
			LPC_I2C->CONCLR = I2CONCLR_SIC;
			break;

		case 0x38:		/* Arbitration lost, in this example, we don't
						deal with multiple master situation */
		default:
			I2CMasterState = I2C_ARBITRATION_LOST;
			LPC_I2C->CONCLR = I2CONCLR_SIC;
			break;
	}
}

/* I2C Data Transmission
 * Transmits contents of data-array to the specified slave */
void I2C_send(uint8_t address, uint8_t length, uint8_t * data)
{
	uint8_t byte;

	if (length > BUFFERSIZE) length = BUFFERSIZE;

	I2CWriteLength = length + 1;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = address & ~RD_BIT;	/* Set R/W Bit to 0 for Writing */

	for (byte = 1; byte < length + 1; byte++)
	{
		I2CMasterBuffer[byte] = data[byte];
	}

	I2C_Engine();
}

/* I2C Data Transmission
 * Transmits data to the specified slave without the need for an array*/
void I2C_send_(uint8_t address, uint8_t length, ...)
{
	va_list args;
	uint8_t byte;

	if (length > BUFFERSIZE) length = BUFFERSIZE;

	I2CWriteLength = length + 1;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = address & ~RD_BIT;	/* Set R/W Bit to 0 for Writing */

	va_start(args, length);

	for (byte = 1; byte < length + 1; byte++)
	{
		I2CMasterBuffer[byte] = (uint8_t) va_arg(args, int);
	}

	I2C_Engine();

	va_end(args);
}

/* I2C Data Reception
 * Transmits a register location to the specified slave address and
 * then reads the specified number of bytes from the slave into the data-array */
void I2C_read(uint8_t address, uint8_t location, uint8_t length, uint8_t * data)
{
	uint8_t byte;

	if (length > BUFFERSIZE) length = BUFFERSIZE;

	for (byte = 0; byte < length; byte++)	// Clear Read Buffer
	{
		I2CSlaveBuffer[byte] = 0x00;
	}

	I2CWriteLength = 2;
	I2CReadLength = length;
	I2CMasterBuffer[0] = address & ~RD_BIT;	/* Slave Address  (Write)*/
	I2CMasterBuffer[1] = location;			/* Register Address */
	I2CMasterBuffer[2] = address | RD_BIT;	/* Slave Address (Read) */
	I2C_Engine();

	for (byte = 0; byte < length; byte++)	// Copy Read Buffer
	{
		data[byte] = I2CSlaveBuffer[byte];
	}
}

/* I2C Data Reception
 * Reads the specified number of bytes from the slave into the data-array */
void I2C_read_(uint8_t address, uint8_t length, uint8_t * data)
{
	uint8_t byte;

	if (length > BUFFERSIZE) length = BUFFERSIZE;

	for (byte = 0; byte < length; byte++)	// Clear Read Buffer
	{
		I2CSlaveBuffer[byte] = 0x00;
	}

	I2CWriteLength = 0;
	I2CReadLength = length;
	I2CMasterBuffer[0] = address | RD_BIT;	/* Slave Address (Read) */
	I2C_Engine();

	for (byte = 0; byte < length; byte++)	// Copy Read Buffer
	{
		data[byte] = I2CSlaveBuffer[byte];
	}
}
