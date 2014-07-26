/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usb.h
 * Purpose: USB Definitions
 * Version: V1.20
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC family microcontroller devices only. Nothing 
 *      else gives you the right to use this software.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#ifndef __USB_H__
#define __USB_H__

#if defined(__GNUC__)   // Code Red tools
#define PRE_PACK
#define POST_PACK     __attribute__((packed))

// The following two typedefs are required as GCC does
// not directly support a method of hinting that a variable
// (as opposed to a structure) should be accessed with
// the assumption that it is unaligned, which can be done
// in Keil using, for example, __packed uint32_t *p;
typedef struct
{
	uint32_t value __attribute__(( packed ));
} unaligned_uint32;

typedef struct
{
	uint16_t value __attribute__(( packed ));
} unaligned_uint16;

#else                   // Other toolchain
#define PRE_PACK      __packed
#define POST_PACK
#define PACKVAR		__attribute__((packed))

#endif

typedef PRE_PACK union
{
	uint16_t W;
	PRE_PACK struct
	{
		uint8_t L;
		uint8_t H;
	}POST_PACK WB;
} POST_PACK WORD_BYTE;

/* bmRequestType.Dir */
#define REQUEST_HOST_TO_DEVICE     0
#define REQUEST_DEVICE_TO_HOST     1

/* bmRequestType.Type */
#define REQUEST_STANDARD           0
#define REQUEST_CLASS              1
#define REQUEST_VENDOR             2
#define REQUEST_RESERVED           3

/* bmRequestType.Recipient */
#define REQUEST_TO_DEVICE          0
#define REQUEST_TO_INTERFACE       1
#define REQUEST_TO_ENDPOINT        2
#define REQUEST_TO_OTHER           3

/* bmRequestType Definition */
typedef PRE_PACK union _REQUEST_TYPE
{
	PRE_PACK struct _BM
	{
		uint8_t Recipient : 5;
		uint8_t Type : 2;
		uint8_t Dir : 1;
	}POST_PACK BM;
	uint8_t B;
} POST_PACK REQUEST_TYPE;

/* USB Standard Request Codes */
#define USB_REQUEST_GET_STATUS                 0
#define USB_REQUEST_CLEAR_FEATURE              1
#define USB_REQUEST_SET_FEATURE                3
#define USB_REQUEST_SET_ADDRESS                5
#define USB_REQUEST_GET_DESCRIPTOR             6
#define USB_REQUEST_SET_DESCRIPTOR             7
#define USB_REQUEST_GET_CONFIGURATION          8
#define USB_REQUEST_SET_CONFIGURATION          9
#define USB_REQUEST_GET_INTERFACE              10
#define USB_REQUEST_SET_INTERFACE              11
#define USB_REQUEST_SYNC_FRAME                 12

/* USB GET_STATUS Bit Values */
#define USB_GETSTATUS_SELF_POWERED             0x01
#define USB_GETSTATUS_REMOTE_WAKEUP            0x02
#define USB_GETSTATUS_ENDPOINT_STALL           0x01

/* USB Standard Feature selectors */
#define USB_FEATURE_ENDPOINT_STALL             0
#define USB_FEATURE_REMOTE_WAKEUP              1

/* USB Default Control Pipe Setup Packet */
typedef PRE_PACK struct _USB_SETUP_PACKET
{
	REQUEST_TYPE bmRequestType;
	uint8_t bRequest;
	WORD_BYTE wValue;
	WORD_BYTE wIndex;
	uint16_t wLength;
} POST_PACK USB_SETUP_PACKET;

/* USB Descriptor Types */
#define USB_DEVICE_DESCRIPTOR_TYPE                  1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE           2
#define USB_STRING_DESCRIPTOR_TYPE                  3
#define USB_INTERFACE_DESCRIPTOR_TYPE               4
#define USB_ENDPOINT_DESCRIPTOR_TYPE                5
#define USB_DEVICE_QUALIFIER_DESCRIPTOR_TYPE        6
#define USB_OTHER_SPEED_CONFIG_DESCRIPTOR_TYPE      7
#define USB_INTERFACE_POWER_DESCRIPTOR_TYPE         8
#define USB_OTG_DESCRIPTOR_TYPE                     9
#define USB_DEBUG_DESCRIPTOR_TYPE                  10
#define USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE  11 
/* Wireless USB extension Descriptor Type. */
#define USB_SECURITY_TYPE                          12
#define USB_KEY_TYPE                               13
#define USB_ENCRIPTION_TYPE                        14
#define USB_BOS_TYPE                               15
#define USB_DEVICE_CAPABILITY_TYPE                 16
#define USB_WIRELESS_ENDPOINT_COMPANION_TYPE       17

/* USB Device Classes */
#define USB_DEVICE_CLASS_RESERVED              0x00
#define USB_DEVICE_CLASS_AUDIO                 0x01
#define USB_DEVICE_CLASS_COMMUNICATIONS        0x02
#define USB_DEVICE_CLASS_HUMAN_INTERFACE       0x03
#define USB_DEVICE_CLASS_MONITOR               0x04
#define USB_DEVICE_CLASS_PHYSICAL_INTERFACE    0x05
#define USB_DEVICE_CLASS_POWER                 0x06
#define USB_DEVICE_CLASS_PRINTER               0x07
#define USB_DEVICE_CLASS_STORAGE               0x08
#define USB_DEVICE_CLASS_HUB                   0x09
#define USB_DEVICE_CLASS_MISCELLANEOUS         0xEF
#define USB_DEVICE_CLASS_VENDOR_SPECIFIC       0xFF

/* bmAttributes in Configuration Descriptor */
#define USB_CONFIG_POWERED_MASK                0x40
#define USB_CONFIG_BUS_POWERED                 0x80
#define USB_CONFIG_SELF_POWERED                0xC0
#define USB_CONFIG_REMOTE_WAKEUP               0x20

/* bMaxPower in Configuration Descriptor */
#define USB_CONFIG_POWER_MA(mA)                ((mA)/2)

/* bEndpointAddress in Endpoint Descriptor */
#define USB_ENDPOINT_DIRECTION_MASK            0x80
#define USB_ENDPOINT_OUT(addr)                 ((addr) | 0x00)
#define USB_ENDPOINT_IN(addr)                  ((addr) | 0x80)

/* bmAttributes in Endpoint Descriptor */
#define USB_ENDPOINT_TYPE_MASK                 0x03
#define USB_ENDPOINT_TYPE_CONTROL              0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS          0x01
#define USB_ENDPOINT_TYPE_BULK                 0x02
#define USB_ENDPOINT_TYPE_INTERRUPT            0x03
#define USB_ENDPOINT_SYNC_MASK                 0x0C
#define USB_ENDPOINT_SYNC_NO_SYNCHRONIZATION   0x00
#define USB_ENDPOINT_SYNC_ASYNCHRONOUS         0x04
#define USB_ENDPOINT_SYNC_ADAPTIVE             0x08
#define USB_ENDPOINT_SYNC_SYNCHRONOUS          0x0C
#define USB_ENDPOINT_USAGE_MASK                0x30
#define USB_ENDPOINT_USAGE_DATA                0x00
#define USB_ENDPOINT_USAGE_FEEDBACK            0x10
#define USB_ENDPOINT_USAGE_IMPLICIT_FEEDBACK   0x20
#define USB_ENDPOINT_USAGE_RESERVED            0x30

/* USB Standard Device Descriptor */
typedef PRE_PACK struct _USB_DEVICE_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t iManufacturer;
	uint8_t iProduct;
	uint8_t iSerialNumber;
	uint8_t bNumConfigurations;
} POST_PACK USB_DEVICE_DESCRIPTOR;

/* USB 2.0 Device Qualifier Descriptor */
typedef PRE_PACK struct _USB_DEVICE_QUALIFIER_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdUSB;
	uint8_t bDeviceClass;
	uint8_t bDeviceSubClass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0;
	uint8_t bNumConfigurations;
	uint8_t bReserved;
} POST_PACK USB_DEVICE_QUALIFIER_DESCRIPTOR;

/* USB Standard Configuration Descriptor */
typedef PRE_PACK struct _USB_CONFIGURATION_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration;
	uint8_t bmAttributes;
	uint8_t bMaxPower;
} POST_PACK USB_CONFIGURATION_DESCRIPTOR;

/* USB Standard Interface Descriptor */
typedef PRE_PACK struct _USB_INTERFACE_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubClass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
} POST_PACK USB_INTERFACE_DESCRIPTOR;

/* USB Standard Endpoint Descriptor */
typedef PRE_PACK struct _USB_ENDPOINT_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
} POST_PACK USB_ENDPOINT_DESCRIPTOR;

/* USB String Descriptor */
typedef PRE_PACK struct _USB_STRING_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bString/*[]*/;
} POST_PACK USB_STRING_DESCRIPTOR;

/* USB Common Descriptor */
typedef PRE_PACK struct _USB_COMMON_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
} POST_PACK USB_COMMON_DESCRIPTOR;

/* USB BOS Descriptor. */
typedef PRE_PACK struct _USB_BOS_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumDeviceCaps;
} POST_PACK USB_BOS_DESCRIPTOR;

/* USB Super Speed Device Capability Descriptor. */
typedef PRE_PACK struct _USB_SS_DEVICE_CAPABLITY_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDeviceCapabilityType;
	uint8_t bAttributes;
	uint16_t wSpeedsSupported;
	uint8_t bFunctionalitySupport;
	uint8_t bU1DevExitLat;
	uint16_t bU2DevExitLat;
} POST_PACK USB_SS_DEVICE_CAPABILITY_DESCRIPTOR;

/* USB Device Capability Descriptor, USB 2.0 extension for LPM. */
typedef PRE_PACK struct _USB_DEVICE_CAPABLITY_DESCRIPTOR
{
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDeviceCapabilityType;
	uint32_t bmAttributes;
} POST_PACK USB_DEVICE_CAPABILITY_DESCRIPTOR;

#endif  /* __USB_H__ */
