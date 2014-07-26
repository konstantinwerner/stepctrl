/*
===============================================================================
 Name        : dac_pot_module.h
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Set and read DAC Voltage and Digital Potentiometer Resistance
 	 	 	   (MCP47X6 DAC & MCP4541 Digital Potentiometer)
===============================================================================
*/

#ifndef DAC_POT_MODULE_H_
#define DAC_POT_MODULE_H_

#define VOLATILE		0x01
#define NONVOLATILE		0x00

#define SUPPLY_VOLTAGE	3.30f

#define ADC_STEPS		1024
#define ADC_VOLTAGE(C)	(float) (SUPPLY_VOLTAGE / ADC_STEPS * C)

//MCP4716 is 10bit / MCP4706 is 8bit
#define DAC_STEPS		1024

#define DAC_STEP(V)			(unsigned short) (DAC_STEPS / SUPPLY_VOLTAGE * V)
#define DAC_VOLTAGE(S)		(float) (SUPPLY_VOLTAGE / DAC_STEPS * S)

#define DAC_ADDRESS		0xC0
#define DAC_GET_V_CMD	0x00
#define DAC_SET_V_CMD	0x40
#define DAC_WRT_V_CMD	0x60

//MCP4541
#define MAX_RESISTANCE	10000
#define POT_STEPS		128

#define POT_STEP(R)			(unsigned char) (POT_STEPS - (((float) POT_STEPS / (float) MAX_RESISTANCE) * R))
#define POT_RESISTANCE(S)	(float) (MAX_RESISTANCE / POT_STEPS * (POT_STEPS - S))

#define POT_ADDRESS		0x5C
#define POT_SET_R_CMD	0x00
#define POT_GET_R_CMD	0x0C

#define POT_INC_R_CMD	0x04
#define POT_DEC_R_CMD	0x08

#define POT_WRT_R_CMD	0x20
#define POT_RED_R_CMD	0x2C

void DAC_set_voltage(unsigned char voltage, unsigned char mode);
unsigned char DAC_get_voltage(unsigned char mode);

float DAC_step2voltage(unsigned char step);
unsigned char DAC_voltage2step(float voltage);

void POT_set_position(unsigned char position, unsigned char mode);
void POT_inc_position(void);
void POT_dec_position(void);
unsigned char POT_get_position(unsigned char mode);


#endif /* DAC_POT_MODULE_H_ */
