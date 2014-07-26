/*
===============================================================================
 Name        : gpio_module.h
 Author      : Konstantin Werner
 Version     : 0.1
 Description : GPIO and Interrupt Configuration
===============================================================================
*/

#ifndef INPUT_MODULE_H_
#define INPUT_MODULE_H_

#include "type.h"

#define FALLING_EDGE	1
#define RISING_EDGE		2

#define HIGH_LEVEL		4
#define LOW_LEVEL		8

#define INPUT			0
#define OUTPUT			1

void GPIO_init(void);

void FLEX_INT0_IRQHandler(void);
void FLEX_INT1_IRQHandler(void);
void FLEX_INT2_IRQHandler(void);
void FLEX_INT3_IRQHandler(void);
void FLEX_INT4_IRQHandler(void);
void FLEX_INT5_IRQHandler(void);
void FLEX_INT6_IRQHandler(void);
void FLEX_INT7_IRQHandler(void);

void GPIO_config_int(uint32_t int_nr, uint32_t port, uint32_t pin, uint32_t type, void (*falling)(void), void (*rising)(void));
void GPIO_enable_int(uint32_t int_nr);
void GPIO_disable_int(uint32_t int_nr);

void GPIO_clock_out(uint32_t onoff);

void GPIO_set_dir(uint32_t port, uint32_t pin, uint32_t dir);
void GPIO_set_value(uint32_t port, uint32_t pin, uint32_t bitVal);
uint32_t GPIO_get_value(uint32_t port, uint32_t pin);


#endif /* INPUT_MODULE_H_ */
