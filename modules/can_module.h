/*
===============================================================================
 Name        : can_module.h
 Author      : Konstantin
 Version     : 0.1a
 Copyright   : Copyright (C)
 Description : CAN Communication
===============================================================================
*/

#ifndef _CAN_MODULE_
#define _CAN_MODULE_

#include "type.h"

/* Data Structure for a CAN Message */
typedef struct can_message_
{
	uint32_t id;					// Message ID (11-29 bits)
	struct flags {
		uint8_t rtr;				// Remote-Transmit-Request-Frame?
#if SUPPORT_EXTENDED_CANID == 1
		uint8_t ext;				// Extended ID?
#endif
	} flags;

	uint8_t length;					// Length of Payload
	uint8_t data[8];				// Payload
} can_message_;

/* Data Structure for a CAN Filter */
typedef struct
{
#if SUPPORT_EXTENDED_CANID == 1
	uint8_t ext;
#endif
	uint32_t mask;
	uint32_t id;
} can_filter_;

#if SUPPORT_EXTENDED_CANID == 1
void CAN_Message_(can_message_ *msg, uint32_t id, uint8_t rtr, uint8_t ext, uint8_t length, ...);
void CAN_Message(can_message_ *msg, uint32_t id, uint8_t rtr, uint8_t ext, uint8_t length, uint8_t *data);
#else
void CAN_Message_(can_message_ *msg, uint32_t id, uint8_t rtr, uint8_t length, ...);
void CAN_Message(can_message_ *msg, uint32_t id, uint8_t rtr, uint8_t length, uint8_t *data);
#endif

void CAN_rx_isr(void);

void CAN_init(uint32_t baudrate, uint32_t rx_int_nr, uint32_t port, uint32_t pin, void (*rx_callback)(can_message_));
void CAN_send(uint8_t buffer_num, can_message_ msg);														// Configure ID and Data of designated buffer then send RTS
void CAN_recv(uint8_t buffer_num, can_message_ *msg);														// Read ID and Data from designated buffer
void CAN_check_rx(signed char *buffer_num,  uint8_t *filter_num, uint8_t *ext, uint8_t *rtr);	// Check if RX Buffer has a message
void CAN_check_tx(uint8_t *buffer_num);																	// Check if TX Buffer has a message / is free
void CAN_set_filter(uint8_t filter_num, uint8_t mask_num, can_filter_ *flt);						// Configure a Filter with ID and Mask
void CAN_enable_filter(uint8_t buffer_num, uint8_t filter_num, uint8_t extended);				// Enable a Filter for a RX buffer
void CAN_reset(uint32_t baudrate);
void CAN_clear(void);

#endif
