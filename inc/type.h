/*
===============================================================================
 Name        : type.h
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Type Definitions
===============================================================================
*/
#ifndef __TYPE_H__
#define __TYPE_H__

#if defined   (  __GNUC__  )
#include "stdint.h"
#else

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
//typedef   signed       __int64 int64_t;

typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
//typedef unsigned       __int64 uint64_t;

#endif

#ifndef NULL
#define NULL    ((void *)0)
#endif

typedef enum {FALSE = 0, TRUE = !FALSE} Bool;

#endif  /* __TYPE_H__ */
