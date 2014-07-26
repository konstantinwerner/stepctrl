/*
===============================================================================
 Name        : vendor.h
 Author      : Konstantin Werner
 Version     : 0.1
 Description : USB Vendor Specific Class Implementation
===============================================================================
*/

#ifndef VENDOR_H_
#define VENDOR_H_

#include "type.h"

#define USB_CMD_MOTOR_POSITION		0x00
#define USB_CMD_MOTOR_REMSTEPS		0x01
#define USB_CMD_MOTOR_VELOCITY		0x02
#define USB_CMD_MOTOR_ACCEL			0x03
#define USB_CMD_MOTOR_DECEL			0x04
#define USB_CMD_MOTOR_CURRENT		0x05
#define USB_CMD_MOTOR_IDLE_CURRENT	0x06
#define USB_CMD_MOTOR_STEPPING		0x07
#define USB_CMD_MOTOR_GOTO			0x08
#define USB_CMD_MOTOR_MOVE			0x09
#define USB_CMD_MOTOR_BRAKE			0x0A
#define USB_CMD_MOTOR_STOP			0x0B
#define USB_CMD_MOTOR_STATE			0x0C

#define USB_CMD_CAN_SEND			0x10
#define USB_CMD_CAN_RECV			0x11

#define USB_CMD_IO_LED				0x20



#define CHARS2INT(x)	((x[0] << 24) + (x[1] << 16) + (x[2] << 8) + (x[3] << 0))
#define CHARS2SHORT(x)	((x[0] <<  8) + (x[1] <<  0))

uint32_t USB_ReqVendorDev(uint32_t is_setup);
uint32_t USB_ReqVendorIF(uint32_t is_setup);
uint32_t USB_ReqVendorEP(uint32_t is_setup);


#endif /* VENDOR_H_ */
