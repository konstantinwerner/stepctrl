/*
===============================================================================
 Name        : mcp2515.c
 Author      : Konstantin
 Version     : 0.1a
 Copyright   : Copyright (C)
 Description : MCP2515 CAN Transceiver
===============================================================================
*/

#include "env.h"

#include "mcp2515_module.h"
#include "ssp_module.h"

/* SPI Commands to the MCP2515 */

#define MCP_RESET		0xC0
#define	MCP_READ		0x03
#define	MCP_READ_RX		0x90
#define	MCP_WRITE		0x02
#define	MCP_WRITE_TX	0x40
#define	MCP_RTS			0x80
#define MCP_READ_STATUS	0xA0
#define	MCP_RX_STATUS	0xB0
#define	MCP_BIT_MODIFY	0x05
#define MCP_RTS_TXB0    0x81
#define MCP_RTS_TXB1    0x82
#define MCP_RTS_TXB2    0x84

/* TRX_init
 *
 * This Function initializes the MCP2515 and configures some basic settings.
 * The Baudrate is set to the value given by the parameter,
 * all Filters are disabled, i.e. all messages are received.
 */
void TRX_init(uint32_t baudrate)
{
	uint8_t i;

	SSP_init(SSP_INTERFACE);

	TRX_reset();									// Perform one Reset to ensure the Chip is in Configuration Mode

	TRX_set_baudrate(baudrate);						// Set CAN Baudrate

	TRX_set_filter_mode(0, RXM_RCV_ALL);			// Set Filter for Buffer 0 to "Receive All" (= Ignore Filters)
	TRX_set_filter_mode(1, RXM_RCV_ALL);			// Set Filter for Buffer 1 to "Receive All" (= Ignore Filters)

	for (i = 0; i < 6; i++)							// Loop through all 6 Filters
	{
		TRX_set_filter_id(i, 0x000, 0);			// Set StandardID to 0x000
#if	SUPPORT_EXTENDED_CANID == 1
		TRX_set_filter_id(i, 0x000, 1);			// Set ExtendedID to 0x000
		TRX_select_filter(0, i, EXIDE_SET);		// Configure each Filter to apply to Extended IDs
#else
		TRX_select_filter(0, i, EXIDE_RESET);		// Configure each Filter to apply only to Standard IDs
#endif
	}

	TRX_set_mask_id(0, 0x000, 1);				// Set Mask for Standard & Ext ID to 0x000
	TRX_set_mask_id(1, 0x000, 1);				// Set Mask for Standard & Ext ID to 0x000

	TRX_bit_modify(BFPCTRL, 0xFF/*(B0BFE | B1BFE | B0BFM | B1BFM)*/, 0x00);	// Disable the RXnBF Pins and set their mode to Digital Output
	TRX_bit_modify(TXRTSCTRL, 0xFF/*(TXB0RTS | TXB1RTS | TXB2RTS)*/, 0x00);	// Set the TXnRTS Pins to Digital Input Mode (= Request-to-Send disabled)

	TRX_disable_interrupts(MERRE | WAKIE | ERRIE | TX2IE | TX1IE | TX0IE);		// Disable all interrupts

	TRX_clear_interrupts(0xFF);													// Clear all Interrupts

	TRX_switch_mode(OPMODE_NORMAL);												// Switch back to normal Operating Mode
}


inline void TRX_read_interrupt(uint8_t * interrupt)
{
	TRX_read_register(CANINTF, 1, interrupt);	// Read the Interrupt Flags register
}

inline void TRX_enable_interrupts(uint8_t interrupts)
{
	TRX_bit_modify(CANINTE, interrupts, 0xFF);	// Set bits in the Interrupt Enable register to '1' according to provided mask
}

inline void TRX_clear_interrupts(uint8_t interrupts)
{
	TRX_bit_modify(CANINTF, interrupts, 0x00);		// Set all bits in the Interrupt Enable register to '0'
	TRX_read_interrupt(&interrupts);				// Found to be necessary in order to set the correct value
}

inline void TRX_disable_interrupts(uint8_t interrupts)
{
	TRX_bit_modify(CANINTE, interrupts, 0x00);		// Set bits in the Interrupt Enable register to '0' according to provided mask
}

inline void TRX_read_mode(uint8_t * mode)
{
	TRX_read_register(CANCTRL, 1, mode);	// Read CAN Control register
	*mode &= REQOP;							// Mask OPMODE bits
}

/* TRX_switch_mode
 * Change the Operating Mode of the MCP2515
 */
void TRX_switch_mode(uint8_t mode)
{
	uint8_t new_mode = 0;
	uint32_t timeout = 0;

	SSP_CS(CS_HIGH);								// Reset CS to ensure the next byte is interpreted as a command

	TRX_bit_modify(CANCTRL, REQOP, mode);			// Set the OPMode Bits in the CAN Control register

//	Sleep(10);										// Wait for Mode Change to be completed

	while (new_mode != mode)						// Check if the Mode has been changed
	{
		if (timeout++ > MCP_TIMEOUT)				// If Timeout has been reached, exit the function with errorcode
		{
		#ifdef _DEBUG
			printf("\r\nModeSwitch Timeout\r\n");
		#endif
			return;
		}

		TRX_read_mode(&new_mode);					// Read the current Mode
//		Sleep(1);
	}
}

inline void TRX_abort_transmission(void)
{
	TRX_bit_modify(CANCTRL, ABAT, ABAT);		// Set the 'Abort Transmission' Bit in the CAN Control register. This aborts the current message Transmission
	TRX_bit_modify(CANCTRL, ABAT, ~ABAT);		// Reset the bit
}

inline void TRX_retry_transmission(uint8_t enable)
{
	if (enable)
		TRX_bit_modify(CANCTRL, OSM, OSM_ENABLED);		// Set the 'One Shot' Bit in the CAN Control Register. This enables Message Retransmission until it has been received.
	else
		TRX_bit_modify(CANCTRL, OSM, ~OSM_ENABLED);		// Unset the 'One Shot' Bit in the CAN Control Register. Messages will only be sent once
}

/* TRX_set_baudrate
 * Configures the Timing and the Baudrate of the MCP2515
 */
void TRX_set_baudrate(uint32_t baudrate)
{
	uint8_t data[3];

	data[0] = SOF_DISABLED | WAKFIL_DISABLED | PHSEG2_LENGTH;			/*CNF3*/
	data[1] = BTLMODE_CNF3 | SMPL_1X | PHSEG1_LENGTH | TPROP_LENGTH;	/*CNF2*/
	data[2] = TSJW_LENGTH | BR_PRESCALER(baudrate);						/*CNF1*/

	TRX_write_register(CNF3, 3, data);
}

void TRX_set_filter_mode(uint8_t buffer_num, uint8_t filter_mode)
{
	if (buffer_num == 0)
		TRX_bit_modify(RXB0CTRL, RXM, filter_mode);
	else
		TRX_bit_modify(RXB1CTRL, RXM, filter_mode);
}

void TRX_select_filter(uint8_t buffer_num, uint8_t filter_num, uint8_t extended)
{
	if ((buffer_num == 0) & (filter_num < 2))
		TRX_bit_modify(RXB0CTRL, FILHIT0, filter_num);
	else
		TRX_bit_modify(RXB1CTRL, FILHIT25, filter_num);

	switch (filter_num)
	{
		case 0: TRX_bit_modify(RXF0SIDL, 1 << EXIDE , extended << EXIDE); break;
		case 1: TRX_bit_modify(RXF1SIDL, 1 << EXIDE , extended << EXIDE); break;
		case 2: TRX_bit_modify(RXF2SIDL, 1 << EXIDE , extended << EXIDE); break;
		case 3: TRX_bit_modify(RXF3SIDL, 1 << EXIDE , extended << EXIDE); break;
		case 4: TRX_bit_modify(RXF4SIDL, 1 << EXIDE , extended << EXIDE); break;
		default:
		case 5: TRX_bit_modify(RXF5SIDL, 1 << EXIDE , extended << EXIDE); break;
	}
}

void TRX_set_filter_id(uint8_t filter_num, uint32_t id, uint8_t extended)
{
	uint32_t offset = filter_num * 4;

	if (filter_num > 2)
		offset = 0x10 + (filter_num - 3) * 4;

	// Standard ID
	TRX_bit_modify(RXF0SIDH + offset, 0xFF, (uint8_t) (id >> 3));
	TRX_bit_modify(RXF0SIDL + offset, 0xE0, (uint8_t) (id << 5));

	// Extended ID
#if SUPPORT_EXTENDED_CANID == 1
	if (extended)
	{
		TRX_bit_modify(RXF0SIDL + offset, 0x03, (uint8_t) (id >> 27));
		TRX_bit_modify(RXF0EID8 + offset, 0xFF, (uint8_t) (id >> 19));
		TRX_bit_modify(RXF0EID0 + offset, 0xFF, (uint8_t) (id >> 11));
	}
#endif
}

void TRX_set_mask_id(uint8_t mask_num, uint32_t id, uint8_t extended)
{
	uint32_t offset = mask_num * 4;

	TRX_bit_modify(RXM0SIDH + offset, 0xFF, (uint8_t) (id >> 3));
	TRX_bit_modify(RXM0SIDL + offset, 0xE0, (uint8_t) (id << 5));
#if SUPPORT_EXTENDED_CANID == 1
	if (extended)
	{
		TRX_bit_modify(RXM0SIDL + offset, 0x03, (uint8_t) (id >> 27));
		TRX_bit_modify(RXM0EID8 + offset, 0xFF, (uint8_t) (id >> 19));
		TRX_bit_modify(RXM0EID0 + offset, 0xFF, (uint8_t) (id >> 11));
	}
#endif
}

void TRX_read_rx_buffer(uint8_t buffer_num, uint32_t *id, uint8_t *ext, uint8_t *rtr, uint8_t *size, uint8_t *data)
{
	uint8_t buffer[5];

	buffer[0] = MCP_READ_RX | (buffer_num << 2);

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 1, buffer);
	SSP_receive(SSP_INTERFACE, 5, buffer);
	SSP_receive(SSP_INTERFACE, 8, data);
	SSP_CS(CS_HIGH);

	*id = (uint32_t) (buffer[0] << 3) + (uint32_t) (buffer[1] >> 5);
#if	SUPPORT_EXTENDED_CANID == 1
	*id += (uint32_t) ((buffer[1] & 0x03) << 27) + (uint32_t) (buffer[2] << 19) + (uint32_t) (buffer[3] << 11);
#else
//	*id = 0x0000;
#endif
	*ext  = (uint8_t) (buffer[1] & 0x08) >> 3;

	if (*ext == 1)
		*rtr  = (uint8_t) (buffer[4] & 0x40) >> 6;
	else
		*rtr  = (uint8_t) (buffer[1] & 0x10) >> 4;

	*size = (uint8_t) (buffer[4] & 0x0F);

	if (*size > 8) *size = 8;
}

void TRX_load_tx_buffer(uint8_t buffer_num, uint32_t id, uint8_t ext, uint8_t rtr, uint8_t size, uint8_t * data)
{
	uint8_t buffer[6];

	buffer[0] = (uint8_t) (MCP_WRITE_TX | (buffer_num * 2));

	buffer[1] = (uint8_t) (id >> 3);
#if	SUPPORT_EXTENDED_CANID == 1
	buffer[2] = (uint8_t) ((id << 5) | ((ext & 0x01) << 3) | ((id >> 27) & 0x03));
	buffer[3] = (uint8_t) ((id >> 19) & 0xFF);
	buffer[4] = (uint8_t) ((id >> 11) & 0xFF);
	buffer[5] = (uint8_t) (((rtr & 0x01) << 6) | (size & 0x0F));
#else
	buffer[2] = (uint8_t) (id << 5);
	buffer[3] = (uint8_t) 0x00;
	buffer[4] = (uint8_t) 0x00;
	buffer[5] = (uint8_t) (((rtr & 0x01) << 6) | (size & 0x0F));
#endif

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 6, buffer);
	SSP_send(SSP_INTERFACE, size, data);
	SSP_CS(CS_HIGH);
}

void TRX_RTS(uint8_t buffer_num)
{
	uint8_t buffer;

	buffer = MCP_RTS | (1 << buffer_num);

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 1, &buffer);
	SSP_CS(CS_HIGH);
}

void TRX_read_rx_status(uint8_t *rx_status)
{
	uint8_t buffer;

	buffer = MCP_RX_STATUS;

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 1, &buffer);
	SSP_receive(SSP_INTERFACE, 1, rx_status);
	SSP_CS(CS_HIGH);
}

void TRX_read_status(uint8_t *status)
{
	uint8_t buffer;

	buffer = MCP_READ_STATUS;

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 1, &buffer);
	SSP_receive(SSP_INTERFACE, 1, status);
	SSP_CS(CS_HIGH);
}
void TRX_reset(void)
{
	uint8_t buffer;

	buffer = MCP_RESET;

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 1, &buffer);

//	Sleep(1);

	SSP_CS(CS_HIGH);

//	Sleep(10);
}

void TRX_write_register(uint8_t address, uint32_t size, uint8_t *data)
{
	uint8_t buffer[2];

	buffer[0] = MCP_WRITE;
	buffer[1] = address;

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 2, buffer);
	SSP_send(SSP_INTERFACE, size, data);
	SSP_CS(CS_HIGH);
}

void TRX_read_register(uint8_t address, uint32_t size, uint8_t *data)
{
	uint8_t buffer[2];

	buffer[0] = MCP_READ;
	buffer[1] = address;

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 2, buffer);
	SSP_receive(SSP_INTERFACE, size, data);
	SSP_CS(CS_HIGH);
}

void TRX_bit_modify(uint8_t address, uint8_t mask, uint8_t data)
{
	uint8_t buffer[4];

	buffer[0] = MCP_BIT_MODIFY;
	buffer[1] = address;
	buffer[2] = mask;
	buffer[3] = data;

	SSP_CS(CS_LOW);
	SSP_send(SSP_INTERFACE, 4, buffer);
	SSP_CS(CS_HIGH);
}
