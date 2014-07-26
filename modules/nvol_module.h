/*
===============================================================================
 Name        : nvol_module.h
 Author      : Konstantin Werner (adapted from Embedded Systems Academy, Inc.)
 Version     : 0.1
 Copyright   : Copyright (C) Konstantin Werner
 Description : Non-Volatile Storage of Variables in Flashmemory
===============================================================================
*/

#ifndef __NVOL_MODULE_
#define __NVOL_MODULE_

#include "type.h"

int32_t NVOL_init(void);
int32_t NVOL_set_variable(uint16_t Id, uint8_t *Value, uint16_t Size);
int32_t NVOL_get_variable(uint16_t Id, uint8_t *Value, uint16_t Size);

#endif
