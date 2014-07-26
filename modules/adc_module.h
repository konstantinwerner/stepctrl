/*
===============================================================================
 Name        : adc_module.h
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Interfaces the integrated ADC
===============================================================================
*/

#ifndef ADC_MODULE_H_
#define ADC_MODULE_H_

#include "type.h"

#define ADC_RESOLUTION_10BITS	0x00
#define ADC_RESOLUTION_09BITS	0x01
#define ADC_RESOLUTION_08BITS	0x02
#define ADC_RESOLUTION_07BITS	0x03
#define ADC_RESOLUTION_06BITS	0x04
#define ADC_RESOLUTION_05BITS	0x05
#define ADC_RESOLUTION_04BITS	0x06
#define ADC_RESOLUTION_03BITS	0x07

#define ADC_CLK			4500000		/* set to 4.5Mhz */

#define ADC_CH1		1
#define ADC_CH2		2
#define ADC_CH3		4
#define ADC_CH4		8
#define ADC_CH5		16
#define ADC_CH6		32
#define ADC_CH7		64
#define ADC_CH8		128

void ADC_init(uint32_t ADC_Clk, uint8_t channels);//, uint8_t resolution);

uint32_t ADC_read(uint8_t channel);

#endif /* ADC_MODULE_H_ */
