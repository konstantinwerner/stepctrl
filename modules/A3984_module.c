/*
===============================================================================
 Name        : A3984_module.c
 Author      : Konstantin Werner
 Version     : 0.1
 Description : Interface for the A3984 Stepper Motor Driver
===============================================================================
*/

#include "env.h"

#include "A3984_module.h"
#include "i2c_module.h"
#include "dac_pot_module.h"
#include "delay_module.h"
#include "gpio_module.h"

#if A3984_TIMER_NR == 1
	#define A3984_TIMER_CLK_BIT	10
	#define A3984_TIMER			LPC_CT32B1
	#define A3984_TIMER_IRQ		TIMER_32_1_IRQn
	#define A3984_TIMER_ISR		TIMER32_1_IRQHandler
#endif

#if A3984_TIMER_NR == 0
	#define A3984_TIMER_CLK_BIT	9
	#define A3984_TIMER			LPC_CT32B0
	#define A3984_TIMER_IRQ		TIMER_32_0_IRQn
	#define A3984_TIMER_ISR		TIMER32_0_IRQHandler
#endif

#define A3984_SET_MS1(x)	GPIO_set_value(A3984_MS1_PORT, A3984_MS1_PIN, x)
#define A3984_SET_MS2(x)	GPIO_set_value(A3984_MS2_PORT, A3984_MS2_PIN, x)
#define A3984_SET_SLEEP(x)	GPIO_set_value(A3984_SLEEP_PORT, A3984_SLEEP_PIN, x)
#define A3984_SET_ENABLE(x)	GPIO_set_value(A3984_ENABLE_PORT, A3984_ENABLE_PIN, x)
#define A3984_SET_RESET(x)	GPIO_set_value(A3984_RESET_PORT, A3984_RESET_PIN, x)
#define A3984_SET_DIR(x)	GPIO_set_value(A3984_DIR_PORT, A3984_DIR_PIN, x)

#define A3984_STATE_STOP		0
#define A3984_STATE_ACCEL		1
#define A3984_STATE_RUN			2
#define A3984_STATE_DECEL		3

#define R_s						0.2f
#define V_ref(I_max)			(I_max / 1000.0f * 8 * R_s)

#define MHZ_PRESCALE			(CPU_CLK / 1000000)

static uint32_t sqrt(uint32_t x);

volatile int32_t A3984_position = 0;	// [st] Absolute Position

volatile uint8_t A3984_direction = 0;
volatile uint8_t A3984_pos_limit = 0;
volatile uint8_t A3984_neg_limit = 0;
volatile uint8_t A3984_reached_pos = 1;	// 1 = Position reached, 0 = Motion was aborted (Limit Switch or manual Stop)

volatile uint32_t A3984_steps = 0;		// [st]
uint32_t A3984_accel = 0;				// [st / s^2]
uint32_t A3984_decel = 0;				// [st / s^2]
uint32_t A3984_speed = 0;				// [st / s]

volatile uint32_t A3984_accel_count = 0;
volatile uint32_t A3984_min_delay   = 0;			// [µs]
volatile uint32_t A3984_step_delay  = 0;			// [µs]
volatile uint32_t A3984_decel_start = 0;

volatile uint8_t A3984_state  = A3984_STATE_STOP;
uint32_t A3984_stepping = A3984_STEP_FULL;

#if A3984_CURRENT_ISR == 1
volatile uint8_t A3984_change_current = 0;
volatile uint32_t A3984_current = 0;
volatile uint32_t A3984_idle_current = 0;
#else
uint32_t A3984_current = 0;
uint32_t A3984_idle_current = 0;
#endif


void A3984_init(void)
{
	I2C_init();
	delay_init();

	GPIO_set_dir(A3984_MS1_PORT, A3984_MS1_PIN, OUTPUT);
	GPIO_set_dir(A3984_MS2_PORT, A3984_MS2_PIN, OUTPUT);
	GPIO_set_dir(A3984_SLEEP_PORT, A3984_SLEEP_PIN, OUTPUT);
	GPIO_set_dir(A3984_ENABLE_PORT, A3984_ENABLE_PIN, OUTPUT);
	GPIO_set_dir(A3984_RESET_PORT, A3984_RESET_PIN, OUTPUT);
	GPIO_set_dir(A3984_DIR_PORT, A3984_DIR_PIN, OUTPUT);
	GPIO_set_dir(A3984_STEP_PORT, A3984_STEP_PIN, OUTPUT);

	LPC_IOCON->SWDIO_PIO0_15 |= 0x01;	// Select IO Function on Pin 0_15

	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << A3984_TIMER_CLK_BIT);
	A3984_TIMER->PR  = MHZ_PRESCALE; 			/* set prescaler to get 1 M counts/sec */
	A3984_TIMER->MR2 = 500; 					/* Set up 0.5 mS interval */
	A3984_TIMER->MCR = (1 << 7) | (1 << 6);		/* Reset and Interrupt on Match2 */

	NVIC_SetPriority(A3984_TIMER_IRQ, 1);		// Lower Priority than I2C interrupt
	NVIC_EnableIRQ(A3984_TIMER_IRQ);
}

void A3984_TIMER_ISR(void)
{
	static uint8_t toggle = 0;
	static uint32_t next_step_delay;
	static uint32_t last_accel_delay;
	static uint32_t rest = 0;

	A3984_TIMER->IR = (1 << 2);	// Clear interrupt flag

	// Make a step
	toggle ^= 1;

	if (A3984_state != A3984_STATE_STOP)
		GPIO_set_value(A3984_STEP_PORT, A3984_STEP_PIN, toggle);

	if (toggle)	// Toggled from low to high (made a Step)
	{
		A3984_TIMER->MR2 = A3984_step_delay >> 1;

		switch (A3984_state)
		{
		case A3984_STATE_STOP :	A3984_steps = 0;
								A3984_TIMER->TCR = 0;	// Stop Timer
								A3984_TIMER->MR2 = 500;
#if A3984_CURRENT_ISR == 1
								A3984_change_current = A3984_idle_current;
#else
								DAC_set_voltage(DAC_STEP(V_ref(A3984_idle_current)), VOLATILE);
#endif
								break;

		case A3984_STATE_ACCEL: A3984_steps--;
								A3984_accel_count++;
								if (A3984_direction) A3984_position++; else A3984_position--;
								next_step_delay = A3984_step_delay - ((2 * A3984_step_delay + rest) / (4 * A3984_accel_count + 1));
								rest = (2 * A3984_step_delay + rest) % (4 * A3984_accel_count + 1);

								if (A3984_steps <= A3984_decel_start)
								{
									A3984_accel_count = A3984_decel_start;
									A3984_state = A3984_STATE_DECEL;
								}
								else if (next_step_delay <= A3984_min_delay)
								{
									last_accel_delay = next_step_delay;
									next_step_delay = A3984_min_delay;
									rest = 0;
									A3984_state = A3984_STATE_RUN;
								}
								break;
		case A3984_STATE_RUN :	A3984_steps--;
								if (A3984_direction) A3984_position++; else A3984_position--;
								next_step_delay = A3984_min_delay;
								if (A3984_steps  <= A3984_decel_start)
								{
									A3984_accel_count = A3984_decel_start;
									next_step_delay = last_accel_delay;
									A3984_state = A3984_STATE_DECEL;
								}
								break;
		case A3984_STATE_DECEL:	A3984_steps--;
								A3984_accel_count--;
								if (A3984_direction) A3984_position++; else A3984_position--;
								next_step_delay = A3984_step_delay + ((2 * A3984_step_delay + rest) / (4 * A3984_accel_count + 1));
								rest = (2 * A3984_step_delay + rest) % (4 * A3984_accel_count + 1);

								if (A3984_accel_count == 0)
								{
									A3984_reached_pos = 1;
									A3984_state = A3984_STATE_STOP;
								}
								break;
		}
		A3984_step_delay = next_step_delay;
	}
}

void A3984_goto(int32_t position)
{
	int32_t distance = position - A3984_position;

	if (distance < 0)
		A3984_move(A3984_DIR_CW, -distance);
	else if (distance > 0)
		A3984_move(A3984_DIR_CCW, distance);
	else
		A3984_stop();
}

void A3984_move(uint8_t direction, uint32_t steps)
{
	uint32_t accel_end = 0;
	uint32_t accel_limit = 0;

	if (A3984_state == A3984_STATE_STOP && steps)
	{
		if (direction && A3984_pos_limit)
			return;

		if (!direction && A3984_neg_limit)
			return;

#if A3984_CURRENT_ISR == 1
		A3984_change_current = A3984_current;
#else
		DAC_set_voltage(DAC_STEP(V_ref(A3984_current)), VOLATILE);
#endif

		A3984_direction = direction;
		A3984_SET_DIR(direction);
		A3984_steps = steps;

		// Ramp Calculation

		A3984_step_delay = 6760 * (sqrt(200000000 / A3984_accel) / 100);
		A3984_min_delay  = 1000000 / A3984_speed;

		accel_end = A3984_speed * A3984_speed / (2 * A3984_accel);
		if (accel_end == 0) accel_end = 1;

		accel_limit = A3984_steps * A3984_decel / (A3984_accel + A3984_decel);
		if (accel_limit == 0) accel_limit = 1;

		if (accel_end >= accel_limit)
			A3984_decel_start = A3984_steps - accel_limit;
		else
			A3984_decel_start = accel_end * A3984_accel / A3984_decel;

		if (A3984_decel_start == 0) A3984_decel_start = 1;

		// Initialize Ramping State Machine

		A3984_accel_count = 0;
		A3984_state = A3984_STATE_ACCEL;
		A3984_reached_pos = 0;

		A3984_TIMER->TCR = 2;	// Reset Timer
		A3984_TIMER->TCR = 1;	// Start Timer
	}
}

inline void A3984_stop(void)
{
	A3984_state = A3984_STATE_STOP;
}

inline void A3984_brake(void)
{
	A3984_steps  = A3984_decel_start;
}

inline int32_t A3984_get_position()
{
	return A3984_position;
}

inline void A3984_set_position(int32_t position)
{
	A3984_position = position;
	A3984_reached_pos = 1;
}

inline uint32_t A3984_get_steps()
{
	return A3984_steps;
}

inline void A3984_set_speed(uint32_t speed)
{
	if (A3984_state == A3984_STATE_STOP)
		A3984_speed = speed;
}

inline void A3984_set_acceleration(uint32_t accel)
{
	if (A3984_state == A3984_STATE_STOP)
		A3984_accel = accel;
}

inline void A3984_set_deceleration(uint32_t decel)
{
	if (A3984_state == A3984_STATE_STOP)
		A3984_decel = decel;
}

inline void A3984_set_stepping(uint8_t stepping)
{
	if (A3984_state == A3984_STATE_STOP)
	{
		A3984_stepping = stepping;
		switch (stepping)
		{
			default :
			case A3984_STEP_FULL	  : A3984_SET_MS2(0); A3984_SET_MS1(0); break;
			case A3984_STEP_HALF      : A3984_SET_MS2(0); A3984_SET_MS1(1); break;
			case A3984_STEP_QUARTER   : A3984_SET_MS2(1); A3984_SET_MS1(0); break;
			case A3984_STEP_SIXTEENTH : A3984_SET_MS2(1); A3984_SET_MS1(1); break;
		}
	}
}

inline void A3984_set_direction(uint8_t direction)
{
	if (A3984_state == A3984_STATE_STOP)
		A3984_SET_DIR(direction);
}

inline void A3984_set_state(uint8_t state)
{
	switch (state)
	{
		case A3984_STATE_SLEEP	 : A3984_SET_SLEEP(0);  break;
		case A3984_STATE_DISABLE : A3984_SET_ENABLE(1); break;
		case A3984_STATE_WAKEUP	 : A3984_SET_SLEEP(1);  delay_ms(10); break;
		default:
		case A3984_STATE_ACTIVE  : A3984_SET_RESET(1); A3984_SET_ENABLE(0); break;
		case A3984_STATE_RESET   : A3984_SET_RESET(0);  break;
	}
}

inline void A3984_set_current(uint32_t current_mA)
{
	A3984_current = current_mA;
}

inline void A3984_set_idle_current(uint32_t current_mA)
{
	A3984_idle_current = current_mA;

#if A3984_CURRENT_ISR == 1
	A3984_change_current = A3984_idle_current;
#else
	if (A3984_state == A3984_STATE_STOP)
		DAC_set_voltage(DAC_STEP(V_ref(A3984_idle_current)), NONVOLATILE);
#endif
}

#if A3984_CURRENT_ISR == 1
inline void A3984_set_current_handler()
{
	if (A3984_change_current)
	{
		DAC_set_voltage(DAC_STEP(V_ref(A3984_change_current)), NONVOLATILE);
		A3984_change_current = 0;
	}
}
#endif


static uint32_t sqrt(uint32_t x)
{
	register uint32_t xr; 	// result register
	register uint32_t q2;	// scan-bit register
	register uint8_t f;		// flag (one bit)

	xr = 0;               		// clear result
	q2 = 0x40000000L;        	// highest possible result bit
	do
	{
		if ((xr + q2) <= x)
		{
			x -= xr + q2;
			f = 1;				// set flag
		}
		else
		{
			f = 0;				// clear flag
		}
		xr >>= 1;
		if (f)
		{
			xr += q2;			// test flag
		}
	} while (q2 >>= 2);			// shift twice

	if (xr < x)
	{
		return xr + 1;			// add for rounding
	}
	else
	{
		return xr;
	}
}
