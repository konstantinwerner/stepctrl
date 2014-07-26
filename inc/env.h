/*
===============================================================================
 Name        : env.h
 Author      : Konstantin Werner
 Version     : 0.1
 Copyright   : Copyright (C) Konstantin Werner
 Description : Prepare Environment: Include things needed by all modules
===============================================================================
*/

#ifndef ENV_H_
#define ENV_H_

#ifdef __USE_CMSIS
#include <LPC11Uxx.h>
#endif

#define CPU_CLK __SYSTEM_CLOCK


#endif /* ENV_H_ */
