/*
===============================================================================
 Name        : iap_module.h
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Provide access to the IAP Functions, especially to reinvoking
 	 	 	   the bootloader correctly
===============================================================================
*/

#ifndef IAP_MODULE_H_
#define IAP_MODULE_H_

#include "type.h"

#define IAP_LOCATION 0x1FFF1FF1

#define IAP_CMD_PREPARE_SECTOR		50
#define IAP_CMD_COPY_RAM_TO_FLASH	51
#define IAP_CMD_ERASE_SECTOR		52
#define IAP_CMD_BLANK_CHECK			53
#define IAP_CMD_READ_PART_ID		54
#define IAP_CMD_READ_BOOT_CODE		55
#define IAP_CMD_COMPARE				56
#define IAP_CMD_REINVOKE_ISP		57
#define IAP_CMD_READ_UID			58
#define IAP_CMD_ERASE_PAGE			59

#define IAP_STATUS_CMD_SUCCESS			0
#define IAP_STATUS_INVALID_COMMAND		1
#define IAP_STATUS_SRC_ADDR_ERROR		2
#define IAP_STATUS_DST_ADDR_ERROR		3
#define IAP_STATUS_SRC_ADDR_NOT_MAPPED	4
#define IAP_STATUS_DST_ADDR_NOT_MAPPED	5
#define IAP_STATUS_COUNT_ERROR			6
#define IAP_STATUS_INVALID_SECTOR		7
#define IAP_STATUS_SECTOR_NOT_BLANK		8
#define IAP_STATUS_SECTOR_NOT_PREPARED	9
#define IAP_STATUS_COMPARE_ERROR		10
#define IAP_STATUS_BUSY					11

typedef void (*IAP)(unsigned long[], unsigned long[]);

static const IAP IAP_cmd = (IAP) IAP_LOCATION;

void IAP_cmd_(unsigned long *result, uint8_t length, ...);
void IAP_InvokeBootloader(void);

#endif
