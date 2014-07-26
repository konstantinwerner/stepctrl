/*
 ===============================================================================
 Name        : vendor.c
 Author      : Konstantin Werner
 Version     : 0.1
 Description : USB Vendor Specific Class Implementation
 ===============================================================================
 */

#include "type.h"
#include "usb.h"
#include "usbcfg.h"
#include "vendor.h"
#include "usbcore.h"

#include "A3984_module.h"
#include "can_module.h"

extern USB_SETUP_PACKET SetupPacket;
can_message_ msg;

extern uint32_t A3984_accel;				// [st / s^2]
extern uint32_t A3984_decel;				// [st / s^2]
extern uint32_t A3984_speed;				// [st / s]
extern uint32_t A3984_stepping;

extern volatile uint32_t A3984_current;
extern volatile uint32_t A3984_idle_current;

uint32_t USB_ReqVendorDev(uint32_t is_setup)
{
	return TRUE;
}

uint32_t USB_ReqVendorIF(uint32_t is_setup)
{
	return TRUE;
}
uint32_t USB_ReqVendorEP(uint32_t is_setup)
{
	int32_t tmp;

	if (is_setup)
	{
		// Only Setup Packet has been received
		if (SetupPacket.bmRequestType.BM.Dir == REQUEST_DEVICE_TO_HOST)
		{
		// Prepare Data for Sending
			switch (SetupPacket.bRequest)
			{
			// Motor Commands
			case USB_CMD_MOTOR_POSITION :
				tmp = A3984_get_position();
				break;

			case USB_CMD_MOTOR_REMSTEPS :
				tmp = A3984_get_steps();
				break;

			case USB_CMD_MOTOR_VELOCITY :
				tmp = A3984_speed;
				break;

			case USB_CMD_MOTOR_ACCEL :
				tmp = A3984_accel;
				break;

			case USB_CMD_MOTOR_DECEL :
				tmp = A3984_decel;
				break;

			case USB_CMD_MOTOR_STEPPING :
				tmp = A3984_stepping;
				break;

			case USB_CMD_MOTOR_CURRENT :
				tmp = A3984_current;
				break;

			case USB_CMD_MOTOR_IDLE_CURRENT :
				tmp = A3984_idle_current;
				break;

			default :
				tmp = 0;
				break;
			}

			EP0Buf[0] = (uint8_t) (tmp >> 24);
			EP0Buf[1] = (uint8_t) (tmp >> 16);
			EP0Buf[2] = (uint8_t) (tmp >>  8);
			EP0Buf[3] = (uint8_t) (tmp >>  0);
		}
	} else
	{
		// Control Message has been received, handle Command
		// Request specifies Command

		switch (SetupPacket.bRequest)
		{
		// Motor Commands
		case USB_CMD_MOTOR_POSITION :
			A3984_set_position((int32_t) CHARS2INT(EP0Buf));
			break;

		case USB_CMD_MOTOR_GOTO :
			A3984_goto((int32_t) CHARS2INT(EP0Buf));
			break;

		case USB_CMD_MOTOR_MOVE :
			tmp = (int32_t) CHARS2INT(EP0Buf);

			if (tmp < 0)
				A3984_move(A3984_DIR_CW, -tmp);
			else if (tmp > 0)
				A3984_move(A3984_DIR_CCW, tmp);
			else
				A3984_stop();

			break;

		case USB_CMD_MOTOR_BRAKE :
			A3984_brake();
			break;

		case USB_CMD_MOTOR_STOP :
			A3984_stop();
			break;

		case USB_CMD_MOTOR_VELOCITY :
			A3984_set_speed((uint32_t) CHARS2INT(EP0Buf));
			break;

		case USB_CMD_MOTOR_ACCEL :
			A3984_set_acceleration((uint32_t) CHARS2INT(EP0Buf));
			break;

		case USB_CMD_MOTOR_DECEL :
			A3984_set_deceleration((uint32_t) CHARS2INT(EP0Buf));
			break;

		case USB_CMD_MOTOR_CURRENT :
			A3984_set_current((uint32_t) CHARS2SHORT(EP0Buf));
			break;

		case USB_CMD_MOTOR_IDLE_CURRENT :
			A3984_set_idle_current((uint32_t) CHARS2SHORT(EP0Buf));
			break;

		case USB_CMD_MOTOR_STEPPING :
			A3984_set_stepping(EP0Buf[0]);
			break;

		case USB_CMD_MOTOR_STATE :
			A3984_set_state(EP0Buf[0]);
			break;

		// IO Commands

		case USB_CMD_IO_LED :

			break;

		// CAN Commands

		case USB_CMD_CAN_SEND :
			// Index => CAN ID
			// Value.H => Request for Remote Transmission (0,1)
			// Value.L => Buffer to send from (0,1,2)
			// Length  => CAN Data Size (0..8)
			// EP0Buf  => CAN Data to send

			if (SetupPacket.wLength > 8)
				SetupPacket.wLength = 8;

			CAN_Message(&msg, SetupPacket.wIndex.W, SetupPacket.wValue.WB.H, SetupPacket.wLength, EP0Buf);
			CAN_send(SetupPacket.wValue.WB.L, msg);
			break;
		}
	}

	return TRUE;
}
