/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbdesc.c
 * Purpose: USB Descriptors
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
 *----------------------------------------------------------------------------
 * History:
 *          V1.20 Changed string descriptor handling
 *          V1.00 Initial Version
 *----------------------------------------------------------------------------*/
#include "type.h"

#include "usb.h"
#include "usbcfg.h"
#include "usbdesc.h"

#if (USB_MSC)
#include "msc.h"

/* USB Standard Device Descriptor */
const uint8_t USB_DeviceDescriptor[] = {
  USB_DEVICE_DESC_SIZE,              /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,        /* bDescriptorType */
#if LPM_SUPPORT
  WBVAL(0x0201), /* 2.01 */          /* bcdUSB */
#else
  WBVAL(0x0200), /* 2.00 */          /* bcdUSB */
#endif
  0x00,                              /* bDeviceClass */
  0x00,                              /* bDeviceSubClass */
  0x00,                              /* bDeviceProtocol */
  USB_MAX_PACKET0,                   /* bMaxPacketSize0 */
  WBVAL(0x1FC9),                     /* idVendor */


  WBVAL(0x1234),                     /* idProduct */
  WBVAL(0x0100), /* 1.00 */          /* bcdDevice */
  0x01,                              /* iManufacturer */
  0x02,                              /* iProduct */
  0x03,                              /* iSerialNumber */
  0x01                               /* bNumConfigurations: one possible configuration*/
};

/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor) */
const uint8_t USB_ConfigDescriptor[] = {
/* Configuration 1 */
  USB_CONFIGUARTION_DESC_SIZE,       /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType */
  WBVAL(                             /* wTotalLength */
    USB_CONFIGUARTION_DESC_SIZE +
    USB_INTERFACE_DESC_SIZE     +
    USB_ENDPOINT_DESC_SIZE		+
    USB_ENDPOINT_DESC_SIZE
  ),
  0x01,                              /* bNumInterfaces */
  0x01,                              /* bConfigurationValue */
  0x00,                              /* iConfiguration */
#if REMOTE_WAKEUP_ENABLE			 /* bmAttributes */
  USB_CONFIG_BUS_POWERED|USB_CONFIG_REMOTE_WAKEUP, 
#else
  USB_CONFIG_BUS_POWERED,
#endif
  USB_CONFIG_POWER_MA(100),          /* bMaxPower */
  /* Interface 0, Alternate Setting 0, MSC Class */
  USB_INTERFACE_DESC_SIZE,           /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
  0x00,                              /* bInterfaceNumber */
  0x00,                              /* bAlternateSetting */
  0x02,                              /* bNumEndpoints */
  USB_DEVICE_CLASS_STORAGE,          /* bInterfaceClass */
  MSC_SUBCLASS_SCSI,                 /* bInterfaceSubClass */
  MSC_PROTOCOL_BULK_ONLY,            /* bInterfaceProtocol */
  0x04,                              /* iInterface */

  /* Endpoint, EP2 Bulk IN */
  USB_ENDPOINT_DESC_SIZE,            /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
  USB_ENDPOINT_IN(2),                /* bEndpointAddress */
  USB_ENDPOINT_TYPE_BULK,            /* bmAttributes */
  WBVAL(USB_MAX_NON_ISO_SIZE),       /* wMaxPacketSize */
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /* Endpoint, EP2 Bulk OUT */
  USB_ENDPOINT_DESC_SIZE,            /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
  USB_ENDPOINT_OUT(2),               /* bEndpointAddress */
  USB_ENDPOINT_TYPE_BULK,            /* bmAttributes */
  WBVAL(USB_MAX_NON_ISO_SIZE),       /* wMaxPacketSize */
  0x00,                              /* bInterval: ignore for Bulk transfer */
  /* Terminator */
  0                                  /* bLength */
};

#endif

#if (USB_HID)
#include "hid.h"
#include "hiduser.h"

/* HID Report Descriptor */
const uint8_t HID_ReportDescriptor[] = {
  HID_UsagePageVendor(0x00),
  HID_Usage(0x01),
  HID_Collection(HID_Application),
  HID_UsagePage(HID_USAGE_PAGE_BUTTON),
  HID_UsageMin(1),
  HID_UsageMax(3),
  HID_LogicalMin(0),
  HID_LogicalMax(1),
  HID_ReportCount(3),
  HID_ReportSize(1),
  HID_Input(HID_Data | HID_Variable | HID_Absolute),
  HID_ReportCount(1),
  HID_ReportSize(5),
  HID_Input(HID_Constant),
  HID_UsagePage(HID_USAGE_PAGE_LED),
  HID_Usage(HID_USAGE_LED_GENERIC_INDICATOR),
  HID_LogicalMin(0),
  HID_LogicalMax(1),
  HID_ReportCount(8),
  HID_ReportSize(1),
  HID_Output(HID_Data | HID_Variable | HID_Absolute),
  HID_EndCollection,
};

const uint16_t HID_ReportDescSize = sizeof(HID_ReportDescriptor);


/* USB Standard Device Descriptor */
const uint8_t USB_DeviceDescriptor[] = {
  USB_DEVICE_DESC_SIZE,              /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,        /* bDescriptorType */
#if LPM_SUPPORT
  WBVAL(0x0201), /* 2.01 */          /* bcdUSB */
#else
  WBVAL(0x0200), /* 2.00 */          /* bcdUSB */
#endif
  0x00,                              /* bDeviceClass */
  0x00,                              /* bDeviceSubClass */
  0x00,                              /* bDeviceProtocol */
  USB_MAX_PACKET0,                   /* bMaxPacketSize0 */
  WBVAL(0x16C0),                     /* idVendor */
  WBVAL(0x05DF),                     /* idProduct */
  WBVAL(0x0100), /* 1.00 */          /* bcdDevice */
  0x01,                              /* iManufacturer */
  0x02,                              /* iProduct */
  0x03,                              /* iSerialNumber */
  0x01                               /* bNumConfigurations: one possible configuration*/
};

/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor) */
const uint8_t USB_ConfigDescriptor[] = {
/* Configuration 1 */
  USB_CONFIGUARTION_DESC_SIZE,       /* bLength */
  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType */
  WBVAL(                             /* wTotalLength */
    USB_CONFIGUARTION_DESC_SIZE +
    USB_INTERFACE_DESC_SIZE     +
    HID_DESC_SIZE               +
    USB_ENDPOINT_DESC_SIZE		+
    USB_ENDPOINT_DESC_SIZE
  ),
  0x01,                              /* bNumInterfaces */
  0x01,                              /* bConfigurationValue */
  0x00,                              /* iConfiguration */
#if REMOTE_WAKEUP_ENABLE			 /* bmAttributes */
  USB_CONFIG_BUS_POWERED|USB_CONFIG_REMOTE_WAKEUP,
#else
  USB_CONFIG_BUS_POWERED,
#endif
  USB_CONFIG_POWER_MA(100),          /* bMaxPower */
/* Interface 0, Alternate Setting 0, HID Class */
  USB_INTERFACE_DESC_SIZE,           /* bLength */
  USB_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
  0x00,                              /* bInterfaceNumber */
  0x00,                              /* bAlternateSetting */
  0x02,                              /* bNumEndpoints */
  USB_DEVICE_CLASS_HUMAN_INTERFACE,  /* bInterfaceClass */
  HID_SUBCLASS_NONE,                 /* bInterfaceSubClass */
  HID_PROTOCOL_NONE,                 /* bInterfaceProtocol */
  0x00,                              /* iInterface */
/* HID Class Descriptor */
/* HID_DESC_OFFSET = 0x0012 */
  HID_DESC_SIZE,                     /* bLength */
  HID_HID_DESCRIPTOR_TYPE,           /* bDescriptorType */
  WBVAL(0x0100), /* 1.00 */          /* bcdHID */
  0x00,                              /* bCountryCode */
  0x01,                              /* bNumDescriptors */
  HID_REPORT_DESCRIPTOR_TYPE,        /* bDescriptorType */
  WBVAL(HID_REPORT_DESC_SIZE),       /* wDescriptorLength */
/* Endpoint, HID Interrupt In */
  USB_ENDPOINT_DESC_SIZE,            /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
  USB_ENDPOINT_IN(HID_EP_IN), 		 /* bEndpointAddress */
  USB_ENDPOINT_TYPE_INTERRUPT,       /* bmAttributes */
  WBVAL(0x0004),                     /* wMaxPacketSize */
  0x20,          /* 32ms */          /* bInterval */
/* Endpoint, HID Interrupt Out */
  USB_ENDPOINT_DESC_SIZE,            /* bLength */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
  USB_ENDPOINT_OUT(HID_EP_OUT),      /* bEndpointAddress */
  USB_ENDPOINT_TYPE_INTERRUPT,       /* bmAttributes */
  WBVAL(0x0004),                     /* wMaxPacketSize */
  0x20,          /* 32ms */          /* bInterval */
/* Terminator */
  0                                  /* bLength */
};
#endif

#if (USB_VENDOR)
/* USB Standard Device Descriptor */
const uint8_t USB_DeviceDescriptor[] = {
  USB_DEVICE_DESC_SIZE,              /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,        /* bDescriptorType */
#if LPM_SUPPORT
  WBVAL(0x0201), /* 2.01 */          /* bcdUSB */
#else
  WBVAL(0x0200), /* 2.00 */          /* bcdUSB */
#endif
  USB_DEVICE_CLASS_VENDOR_SPECIFIC,	 /* bDeviceClass */
  0x00,                              /* bDeviceSubClass */
  0x00,                              /* bDeviceProtocol */
  USB_MAX_PACKET0,                   /* bMaxPacketSize0 */
  WBVAL(0x4B57), // KW               /* idVendor */
  WBVAL(0x4D43), // MC               /* idProduct */
  WBVAL(0x0100), /* 1.00 */          /* bcdDevice */
  0x01,                              /* iManufacturer */
  0x02,                              /* iProduct */
  0x03,                              /* iSerialNumber */
  0x01                               /* bNumConfigurations */
};

/* USB Configuration Descriptor */
/* All Descriptors (Configuration, Interface, Endpoint, Class, Vendor) */
const uint8_t USB_ConfigDescriptor[] = {
		/* Configuration 1 */
		  USB_CONFIGUARTION_DESC_SIZE,       /* bLength */
		  USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType */
		  WBVAL(                             /* wTotalLength */
		    USB_CONFIGUARTION_DESC_SIZE +
		    USB_INTERFACE_DESC_SIZE     +
		    USB_ENDPOINT_DESC_SIZE		+
		    USB_ENDPOINT_DESC_SIZE
		  ),
		  0x01,                              /* bNumInterfaces */
		  0x01,                              /* bConfigurationValue */
		  0x04,                              /* iConfiguration (String Descriptor Index)*/
		#if REMOTE_WAKEUP_ENABLE			 /* bmAttributes */
		  USB_CONFIG_BUS_POWERED|USB_CONFIG_REMOTE_WAKEUP,
		#else
		  USB_CONFIG_BUS_POWERED,
		#endif
		  USB_CONFIG_POWER_MA(100),          /* bMaxPower */

		/* Interface 0, Alternate Setting 0, HID Class */
		  USB_INTERFACE_DESC_SIZE,           /* bLength */
		  USB_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
		  0x00,                              /* bInterfaceNumber */
		  0x00,                              /* bAlternateSetting */
		  0x02,                              /* bNumEndpoints */
		  USB_DEVICE_CLASS_VENDOR_SPECIFIC,  /* bInterfaceClass */
		  0xC3,				                 /* bInterfaceSubClass */
		  0xFF, 			                 /* bInterfaceProtocol */
		  0x04,                              /* iInterface  (String Descriptor Index)*/
		/* Vendor Class Descriptor */
		/* Endpoint, Interrupt Out */
		  USB_ENDPOINT_DESC_SIZE,            /* bLength */
		  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
		  USB_ENDPOINT_OUT(1), 				 /* bEndpointAddress */
		  USB_ENDPOINT_TYPE_INTERRUPT,       /* bmAttributes */
		  WBVAL(0x0010),                     /* wMaxPacketSize */
		  0x20,          /* 32ms */          /* bInterval */
		/* Endpoint, Interrupt In */
		  USB_ENDPOINT_DESC_SIZE,            /* bLength */
		  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
		  USB_ENDPOINT_IN(2), 			     /* bEndpointAddress */
		  USB_ENDPOINT_TYPE_INTERRUPT,     	 /* bmAttributes */
		  WBVAL(0x0010),                     /* wMaxPacketSize */
		  0x20,          /* 32ms */          /* bInterval */
		/* Terminator */
		  0                                  /* bLength */
		};
#endif

/* USB String Descriptor (optional) */
const uint8_t USB_StringDescriptor[] = {
/* Index 0x00: LANGID Codes */
  0x04,                              /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  WBVAL(0x0409), /* US English */    /* wLANGID */
/* Index 0x01: Manufacturer */
  DESCSTRLEN(11),                    /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'K',0,
  'O',0,
  'N',0,
  'S',0,
  'T',0,
  'A',0,
  'N',0,
  'T',0,
  '.',0,
  'I',0,
  'N',0,
/* Index 0x02: Product */
  DESCSTRLEN(27),                    /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'U',0,
  'S',0,
  'B',0,
  ' ',0,
  'S',0,
  't',0,
  'e',0,
  'p',0,
  'p',0,
  'e',0,
  'r',0,
  'm',0,
  'o',0,
  't',0,
  'o',0,
  'r',0,
  ' ',0,
  'C',0,
  'o',0,
  'n',0,
  't',0,
  'r',0,
  'o',0,
  'l',0,
  'l',0,
  'e',0,
  'r',0,
/* Index 0x03: Serial Number */
  DESCSTRLEN(12),                    /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  '0',0,                             /* allowed characters are       */
  '0',0,                             /*   0x0030 - 0x0039 ('0'..'9') */
  '0',0,                             /*   0x0041 - 0x0046 ('A'..'F') */
  '1',0,                             /*   length >= 26               */
  'A',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
/* Index 0x04: Configuration 0, Interface 0, Alternate Setting 0 */
   DESCSTRLEN(5),                     /* bLength */
   USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  'M',0,
  'o',0,
  't',0,
  'o',0,
  'r',0,
};

#if LPM_SUPPORT
/* In order to test Link Power Management(LPM), bcdUSB needs to be 0x0201(USB 2.0
Extension) instead of 0x0200. Below BOS descriptor is for LPM test only. However, 
current USBC2.0 CV test requires both Super Speed Device Capability Descriptor
and Device Capability descriptor to be present and wSpeedSupport should be 0x000F 
instead of 0x0003 indicating it's a Super Speed Device or USBCV test will fail. 
For USB2.0 extention and LPM support, the device is not necessary a Super Speed 
Device. To be discussed with USB-IF. */

/* USB BOS Descriptor */
const uint8_t USB_BOSDescriptor[] = {
  USB_BOS_DESC_SIZE,                 /* bLength */
  USB_BOS_TYPE,                      /* bDescriptorType, BOS */
  WBVAL(                             /* wTotalLength */
    USB_BOS_DESC_SIZE +
    USB_SS_DEVICE_CAPABILITY_SIZE +
    USB_DEVICE_CAPABILITY_SIZE
  ),
  0x02,                               /* bNumDeviceCaps, One Device Capability desc. */
  /* Device Capabilities, USB 2.0 Extension Descriptor */
  USB_SS_DEVICE_CAPABILITY_SIZE,     /* bLength */
  USB_DEVICE_CAPABILITY_TYPE,        /* bDescriptorType, DEVICE CAPABILITY */
  0x03,                              /* bDeviceCapabilityType, USB Super Speed */
  0x00,                              /* bmAttributes */
  WBVAL(0x000F),                     /* wSpeedsSupported, All speeds are supported. */
  0x02,                              /* bFunctionalitySupport, Full Speed */
  0x00,                              /* bU1DevExitLat */
  WBVAL(0x0000),                     /* bU2DevExitLat */
  /* Device Capabilities, USB 2.0 Extension Descriptor */
  USB_DEVICE_CAPABILITY_SIZE,        /* bLength */
  USB_DEVICE_CAPABILITY_TYPE,        /* bDescriptorType, DEVICE CAPABILITY */
  0x02,                              /* bDeviceCapabilityType, USB2.0 EXTENSION */
  (0x1<<1),                          /* bmAttributes, bit1 to 1, LPM supported. */
  0x00,
  0x00,
  0x00
};
#endif

