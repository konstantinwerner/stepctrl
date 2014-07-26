/*
===============================================================================
 Name        : A3984_module.h
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Interface for the A3984 Stepper Motor Driver
===============================================================================
*/

#ifndef A3984_MODULE_H_
#define A3984_MODULE_H_

#include "type.h"

// Pin Definitions

#define A3984_MS1_PORT		0
#define A3984_MS1_PIN		17

#define A3984_MS2_PORT		1
#define A3984_MS2_PIN		15

#define A3984_SLEEP_PORT	0
#define A3984_SLEEP_PIN		16

#define A3984_ENABLE_PORT	0
#define A3984_ENABLE_PIN	18

#define A3984_RESET_PORT	0
#define A3984_RESET_PIN		23

#define A3984_DIR_PORT		1
#define A3984_DIR_PIN		14

#define A3984_STEP_PORT		0
#define A3984_STEP_PIN		15

#define A3984_TIMER_NR		1	// 32Bit Timer used for Stepping

#define A3984_DIR_CW		0	// Negative
#define A3984_DIR_CCW		1	// Positive

#define A3984_CURRENT_ISR		1	// Change Current in Main Loop Handler if used from Interrupt level

#define A3984_STEP_FULL			0
#define A3984_STEP_HALF			1
#define A3984_STEP_QUARTER		2
#define A3984_STEP_SIXTEENTH	3

#define A3984_STATE_SLEEP		0
#define A3984_STATE_DISABLE		1
#define A3984_STATE_WAKEUP		3
#define A3984_STATE_ACTIVE		4
#define A3984_STATE_RESET		5

void A3984_init(void);

void A3984_set_stepping(uint8_t stepping);
void A3984_set_speed(uint32_t speed);
void A3984_set_acceleration(uint32_t accel);
void A3984_set_deceleration(uint32_t decel);
void A3984_set_direction(uint8_t direction);
void A3984_set_state(uint8_t state);
void A3984_set_current(uint32_t current_mA);
void A3984_set_idle_current(uint32_t current_mA);

#if A3984_CURRENT_ISR==1
void A3984_set_current_handler();
#endif

uint32_t A3984_get_steps();

int32_t A3984_get_position();
void A3984_set_position(int32_t position);

void A3984_move(uint8_t direction, uint32_t steps);
void A3984_goto(int32_t position);
void A3984_stop(void);
void A3984_brake(void);

#endif /* A3984_H_ */
