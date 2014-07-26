/*
===============================================================================
 Name        : can_module.c
 Author      : Konstantin
 Version     : 0.1a
 Copyright   : Copyright (C)
 Description : CAN Communication
===============================================================================
*/
#include "env.h"
#include "can_module.h"

#include <stdarg.h>

#include "gpio_module.h"
#include "mcp2515_module.h"

void (*rx_int_callback)(can_message_ msg);

#if SUPPORT_EXTENDED_CANID == 1
void CAN_Message_(can_message_ *msg, uint32_t id, uint8_t rtr, uint8_t ext, uint8_t length, ...)
#else
void CAN_Message_(can_message_ *msg, uint32_t id, uint8_t rtr, uint8_t length, ...)
#endif
{
	uint8_t i;
	va_list args;

	msg->id = id;
	msg->flags.rtr = rtr;
#if SUPPORT_EXTENDED_CANID == 1
	msg->flags.ext = ext;
#endif
	msg->length = length;

	va_start(args, length);

	for (i = 0; i < length; i++)
		msg->data[i] = (uint8_t) va_arg(args, int);

	va_end(args);
}

#if SUPPORT_EXTENDED_CANID == 1
void CAN_Message(can_message_ *msg, uint32_t id, uint8_t rtr, uint8_t ext, uint8_t length, uint8_t *data)
#else
void CAN_Message(can_message_ *msg, uint32_t id, uint8_t rtr, uint8_t length, uint8_t *data)
#endif
{
	uint8_t i;

	msg->id = id;
	msg->flags.rtr = rtr;
#if SUPPORT_EXTENDED_CANID == 1
	msg->flags.ext = ext;
#endif
	msg->length = length;

	for (i = 0; i < length; i++)
		msg->data[i] = data[i];
}

void CAN_rx_isr(void)
{
	can_message_ msg;
	uint8_t filled_buffer;

	TRX_read_interrupt(&filled_buffer);

	if ((filled_buffer & 0x03) < 0x03 && filled_buffer != 0)
#if SUPPORT_EXTENDED_CANID == 1
		TRX_read_rx_buffer(filled_buffer - 1, &(msg.id), &(msg.flags.ext), &(msg.flags.rtr), &(msg.length), msg.data);
#else

#endif
	TRX_clear_interrupts(filled_buffer);

	(*rx_int_callback)(msg);
}

void CAN_init(uint32_t baudrate, uint32_t rx_int_nr, uint32_t port, uint32_t pin, void (*rx_callback)(can_message_))
{
	TRX_init(baudrate);

	if (rx_callback != NULL)
	{
		TRX_enable_interrupts(RX1IE | RX0IE);				// Enable the "Receive Buffer n Full" Interrupt
		GPIO_set_dir(CAN_RX_INT_PORT, CAN_RX_INT_PIN, 0);

		GPIO_config_int(rx_int_nr, port, pin, FALLING_EDGE, &CAN_rx_isr, NULL);
		GPIO_enable_int(rx_int_nr);

		rx_int_callback = rx_callback;
	} else
		rx_int_callback = NULL;
}

void CAN_send(uint8_t buffer_num, can_message_ msg)
{
	uint8_t tx_status;

	CAN_check_tx(&tx_status);

	if ((tx_status & (1 << buffer_num)) != 0)
		return;

#if SUPPORT_EXTENDED_CANID == 1
	TRX_load_tx_buffer(buffer_num, msg.id, msg.flags.ext, msg.flags.rtr, msg.length, msg.data);
#else
	TRX_load_tx_buffer(buffer_num, msg.id, 0, msg.flags.rtr, msg.length, msg.data);
#endif

	TRX_RTS(buffer_num);
}

void CAN_recv(uint8_t buffer_num, can_message_ *msg)
{
	uint8_t i;

	for(i = 0; i < 8; i++)
		msg->data[i] = 0x00;

#if SUPPORT_EXTENDED_CANID == 1
	TRX_read_rx_buffer(buffer_num, &(msg->id), &(msg->flags.ext), &(msg->flags.rtr), &(msg->length), (uint8_t*) &(msg->data));

	if (msg->flags.ext == 0)
		msg->id &= 0x7FF;
#else
	TRX_read_rx_buffer(buffer_num, &(msg->id), &i, &(msg->flags.rtr), &(msg->length), (uint8_t*) &(msg->data));	// i as dummy for ext
#endif

	TRX_clear_interrupts((uint8_t) (1 << buffer_num)); // RX0IF = 0x01 and RX1IF = 0x02
}

void CAN_check_rx(signed char *buffer_num,  uint8_t *filter_num, uint8_t *ext, uint8_t *rtr)
{
	uint8_t rx_status;

	TRX_read_rx_status(&rx_status);

	*buffer_num = ((rx_status & 0xC0) >> 6) - 1;

	if (filter_num != NULL)
		*filter_num = rx_status & 0x07;

	if (ext != NULL)
		*ext = (rx_status & 0x10) >> 4;

	if (rtr != NULL)
		*rtr = (rx_status & 0x08) >> 3;
}

void CAN_check_tx(uint8_t *buffer_num)
{
	uint8_t tx_status;

	TRX_read_status(&tx_status);

	*buffer_num  = (tx_status & 0x04) >> 2;
	*buffer_num += (tx_status & 0x10) >> 3;
	*buffer_num += (tx_status & 0x40) >> 4;
}

void CAN_set_filter(uint8_t filter_num, uint8_t mask_num, can_filter_ *flt)
{
	TRX_switch_mode(OPMODE_CONFIG);

#if SUPPORT_EXTENDED_CANID == 1
	TRX_set_filter_id(filter_num, flt->id, flt->ext);
	TRX_set_mask_id(mask_num, flt->mask, flt->ext);

	switch (filter_num)
	{
		case 0: TRX_bit_modify(RXF0SIDL, 1 << EXIDE , flt->ext << EXIDE); break;
		case 1: TRX_bit_modify(RXF1SIDL, 1 << EXIDE , flt->ext << EXIDE); break;
		case 2: TRX_bit_modify(RXF2SIDL, 1 << EXIDE , flt->ext << EXIDE); break;
		case 3: TRX_bit_modify(RXF3SIDL, 1 << EXIDE , flt->ext << EXIDE); break;
		case 4: TRX_bit_modify(RXF4SIDL, 1 << EXIDE , flt->ext << EXIDE); break;
		default:
		case 5: TRX_bit_modify(RXF5SIDL, 1 << EXIDE , flt->ext << EXIDE); break;
	}

#else
	TRX_set_filter_id(filter_num, flt->id, 0);
	TRX_set_mask_id(mask_num, flt->mask, 0);

	switch (filter_num)
	{
		case 0: TRX_bit_modify(RXF0SIDL, 1 << EXIDE , 0 << EXIDE); break;
		case 1: TRX_bit_modify(RXF1SIDL, 1 << EXIDE , 0 << EXIDE); break;
		case 2: TRX_bit_modify(RXF2SIDL, 1 << EXIDE , 0 << EXIDE); break;
		case 3: TRX_bit_modify(RXF3SIDL, 1 << EXIDE , 0 << EXIDE); break;
		case 4: TRX_bit_modify(RXF4SIDL, 1 << EXIDE , 0 << EXIDE); break;
		default:
		case 5: TRX_bit_modify(RXF5SIDL, 1 << EXIDE , 0 << EXIDE); break;
	}

#endif

	TRX_switch_mode(OPMODE_NORMAL);
}

void CAN_enable_filter(uint8_t buffer_num, uint8_t filter_num, uint8_t mode)
{
	TRX_switch_mode(OPMODE_CONFIG);

	TRX_set_filter_mode(buffer_num, mode);

	switch (mode)
	{
		case CAN_FILTER_ONLY_EXT : TRX_select_filter(buffer_num, filter_num, 1); break;
		case CAN_FILTER_ONLY_STD : TRX_select_filter(buffer_num, filter_num, 0); break;
	}

	TRX_switch_mode(OPMODE_NORMAL);
}

void CAN_reset(uint32_t baudrate)
{
	TRX_reset();
//	TRX_init(baudrate);
}

void CAN_clear(void)
{
	TRX_clear_interrupts(0xFF);
}
