/*
 * i2c_module.h
 *
 * Contains all settings necessary for communication via the CAN bus
 *
 */

#ifndef I2C_MODULE_H_
#define I2C_MODULE_H_

#include "type.h"

#define FAST_MODE_PLUS	0

#define BUFFERSIZE		64
#define MAX_TIMEOUT		0x00FFFFFF

#define I2CMASTER		0x01
#define I2CSLAVE		0x02

#define READ_WRITE		0x01
#define RD_BIT			0x01

void I2C_init(void);

uint32_t I2C_Engine(void);
void I2C_IRQHandler(void);

void I2C_send(uint8_t address, uint8_t length, uint8_t * data);
void I2C_send_(uint8_t address, uint8_t length, ...);

void I2C_read(uint8_t address, uint8_t location, uint8_t length, uint8_t * data);
void I2C_read_(uint8_t address, uint8_t length, uint8_t * data);


#endif /* I2C_MODULE_H_ */
