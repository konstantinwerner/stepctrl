/*
===============================================================================
 Name        : dac_pot_module.c
 Author      : Konstantin Werner
 Version     : 0.1
 Copyright   : Copyright (C) Konstantin Werner
 Description : Set and read DAC Voltage and Digital Potentiometer Resistance
 	 	 	   (MCP47X6 DAC & MCP4541 Digital Potentiometer)
===============================================================================
*/
#include "env.h"

#include "dac_pot_module.h"
#include "i2c_module.h"

inline void DAC_set_voltage(uint8_t voltage, uint8_t mode)
{
#if (DAC_STEPS == 1024)
	uint8_t v_hi, v_lo;

	v_hi =  voltage >> 2;
	v_lo = (voltage << 6) & 0xC0;

	if (mode == NONVOLATILE)
		I2C_send_(DAC_ADDRESS, 3, DAC_WRT_V_CMD, v_hi, v_lo);
	else
		I2C_send_(DAC_ADDRESS, 3, DAC_SET_V_CMD, v_hi, v_lo);

#elif (DAC_STEPS == 256)
	if (mode == NONVOLATILE)
		I2C_send_(DAC_ADDRESS, 3, DAC_WRT_V_CMD, voltage, 0x00);
	else
		I2C_send_(DAC_ADDRESS, 3, DAC_SET_V_CMD, voltage, 0x00);
#endif
}

inline uint8_t DAC_get_voltage(uint8_t mode)
{
#if (DAC_STEPS == 1024)
	uint8_t v[6];

	I2C_read(DAC_ADDRESS, DAC_GET_V_CMD, 6, v);

	if (mode == NONVOLATILE)
		return (unsigned short) (((v[4] << 8) + v[5]) >> 6);
	else
		return (unsigned short) (((v[1] << 8) + v[2]) >> 6);

#elif (DAC_STEPS == 256)
	uint8_t v[4];

	I2C_read(DAC_ADDRESS, DAC_GET_V_CMD, 4, v);

	if (mode == NONVOLATILE)
		return (unsigned short) (v[3]);
	else
		return (unsigned short) (v[1]);
#endif
}

inline float DAC_step2voltage(uint8_t step)
{
	return (float)(step * SUPPLY_VOLTAGE / DAC_STEPS);
}

inline uint8_t DAC_voltage2step(float voltage)
{
	return (uint8_t) (voltage / SUPPLY_VOLTAGE * DAC_STEPS);
}

inline void POT_set_position(uint8_t position, uint8_t mode)
{
	if (mode == NONVOLATILE)
		I2C_send_(POT_ADDRESS, 2, POT_WRT_R_CMD, position);
	else
		I2C_send_(POT_ADDRESS, 2, POT_SET_R_CMD, position);
}

inline void POT_inc_position(void)
{
	I2C_send_(POT_ADDRESS, 1, POT_INC_R_CMD);
}

inline void POT_dec_position(void)
{
	I2C_send_(POT_ADDRESS, 1, POT_DEC_R_CMD);
}

inline uint8_t POT_get_position(uint8_t mode)
{
	uint8_t p[2];

	if (mode == NONVOLATILE)
		I2C_read(POT_ADDRESS, POT_RED_R_CMD, 2, p);
	else
		I2C_read(POT_ADDRESS, POT_GET_R_CMD, 2, p);

	return p[1];
}

