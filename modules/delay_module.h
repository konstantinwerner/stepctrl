/*
===============================================================================
 Name        : delay_module.h
 Author      : Konstantin Werner
 Version     : 0.1
 Copyright   : Copyright (C) Konstantin Werner
 Description : Uses a Timer for Delays
===============================================================================
*/

#ifndef DELAY_MODULE_H_
#define DELAY_MODULE_H_

#define DELAY_TIMER_NR		0

void delay_init(void);	// MUST be initialized before any function that uses delays
void delay_us(uint32_t delay);
void delay_ms(uint32_t delay);

uint32_t micros(void);

static void delay_us_short(uint32_t) __attribute__((always_inline, unused));
static inline void delay_us_short(uint32_t delay)
{
  asm volatile
  (
    "L_%=_delay:" "\n\t"
    "sub %0, #1" "\n\t"
    "bne L_%=_delay" "\n"
    : "+r" (delay) :
  );
}

#endif /* DELAY_MODULE_H_ */
