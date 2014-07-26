/*
===============================================================================
 Name        : gpio_module.c
 Author      : Konstantin Werner
 Version     : 0.1
 Description : GPIO and Interrupt Configuration
===============================================================================
*/

#include "env.h"
#include "gpio_module.h"

typedef void (*isr_callback)(void);

isr_callback isr_rising[8] = {NULL};
isr_callback isr_falling[8] = {NULL};

uint32_t isr_type[8] = {0};

void GPIO_init(void)
{
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6);	// Enable AHB clock to the GPIO domain.
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 19);	// Enable AHB clock to the FlexInt domain
}

void GPIO_config_int(uint32_t int_nr, uint32_t port, uint32_t pin, uint32_t type, void (*falling)(void), void (*rising)(void))
{
	if (int_nr >= 0 && int_nr < 8)
	{
		LPC_SYSCON->PINTSEL[int_nr]	= port * 24 + pin;	// Set FlexInt Channel to Pin

		isr_type[int_nr] = type;

		if (type & RISING_EDGE || type & FALLING_EDGE)
		{
			LPC_GPIO_PIN_INT->ISEL &= ~(1 << int_nr);		// Select Edge Trigger on Channel

			if (type & RISING_EDGE && rising != NULL)
			{
				LPC_GPIO_PIN_INT->SIENR |=  (1 << int_nr);	// Enable Rising Edge Interrupt
				isr_rising[int_nr] = rising;
			}
			else
			{
				LPC_GPIO_PIN_INT->CIENR |=  (1 << int_nr);	// Disable Rising Edge Interrupt
				isr_rising[int_nr] = NULL;
			}

			if (type & FALLING_EDGE && falling != NULL)
			{
				LPC_GPIO_PIN_INT->SIENF |=  (1 << int_nr);	// Enable Rising Edge Interrupt
				isr_falling[int_nr] = falling;
			}
			else
			{
				LPC_GPIO_PIN_INT->CIENF |=  (1 << int_nr);	// Disable Rising Edge Interrupt
				isr_falling[int_nr] = NULL;
			}
		} else if (type & HIGH_LEVEL || type & LOW_LEVEL)
		{
			LPC_GPIO_PIN_INT->ISEL	|= (1 << int_nr);		// Select Level Trigger on Channel
			LPC_GPIO_PIN_INT->SIENR	|= (1 << int_nr);		// Enable Level Interrupt

			if (type & HIGH_LEVEL)
				LPC_GPIO_PIN_INT->SIENF |=  (1 << int_nr);	// Select High Level
			else if (type & LOW_LEVEL)
				LPC_GPIO_PIN_INT->CIENF |=  (1 << int_nr);	// Select Low Level
		}
	}
}

inline void GPIO_enable_int(uint32_t int_nr)
{
	NVIC_EnableIRQ(int_nr);
}

inline void GPIO_disable_int(uint32_t int_nr)
{
	NVIC_DisableIRQ(int_nr);
}

inline void GPIO_clear_irq(uint32_t int_nr, uint32_t type)
{
	 if (type & RISING_EDGE )
		 LPC_GPIO_PIN_INT->RISE |= (1 << int_nr);

	 if (type & FALLING_EDGE)
		 LPC_GPIO_PIN_INT->FALL |= (1 << int_nr);

	 if (type & HIGH_LEVEL || type & LOW_LEVEL)
		 LPC_GPIO_PIN_INT->IST |= (1 << int_nr);
}

void FLEX_INT0_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 0) && isr_rising[0] != NULL)		// Check if Rising Edge has occurred
		(*isr_rising[0])();
	else if (LPC_GPIO_PIN_INT->FALL & (1 << 0) && isr_falling[0] != NULL)	// Check if Falling Edge has occurred
		(*isr_falling[0])();
}

void FLEX_INT1_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 0) && isr_rising[1] != NULL)		// Check if Rising Edge has occurred
		(*isr_rising[1])();
	else if (LPC_GPIO_PIN_INT->FALL & (1 << 0) && isr_falling[1] != NULL)	// Check if Falling Edge has occurred
		(*isr_falling[1])();
}

void FLEX_INT2_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 0) && isr_rising[2] != NULL)		// Check if Rising Edge has occurred
		(*isr_rising[2])();
	else if (LPC_GPIO_PIN_INT->FALL & (1 << 0) && isr_falling[2] != NULL)	// Check if Falling Edge has occurred
		(*isr_falling[2])();
}

void FLEX_INT3_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 0) && isr_rising[3] != NULL)		// Check if Rising Edge has occurred
		(*isr_rising[3])();
	else if (LPC_GPIO_PIN_INT->FALL & (1 << 0) && isr_falling[3] != NULL)	// Check if Falling Edge has occurred
		(*isr_falling[3])();
}

void FLEX_INT4_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 0) && isr_rising[4] != NULL)		// Check if Rising Edge has occurred
		(*isr_rising[4])();
	else if (LPC_GPIO_PIN_INT->FALL & (1 << 0) && isr_falling[4] != NULL)	// Check if Falling Edge has occurred
		(*isr_falling[4])();
}

void FLEX_INT5_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 0) && isr_rising[5] != NULL)		// Check if Rising Edge has occurred
		(*isr_rising[5])();
	else if (LPC_GPIO_PIN_INT->FALL & (1 << 0) && isr_falling[5] != NULL)	// Check if Falling Edge has occurred
		(*isr_falling[5])();
}

void FLEX_INT6_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 0) && isr_rising[6] != NULL)		// Check if Rising Edge has occurred
		(*isr_rising[6])();
	else if (LPC_GPIO_PIN_INT->FALL & (1 << 0) && isr_falling[6] != NULL)	// Check if Falling Edge has occurred
		(*isr_falling[6])();
}

void FLEX_INT7_IRQHandler(void)
{
	if (LPC_GPIO_PIN_INT->RISE & (1 << 0) && isr_rising[7] != NULL)		// Check if Rising Edge has occurred
		(*isr_rising[7])();
	else if (LPC_GPIO_PIN_INT->FALL & (1 << 0) && isr_falling[7] != NULL)	// Check if Falling Edge has occurred
		(*isr_falling[7])();
}

void GPIO_clock_out(uint32_t onoff)
{
	if (onoff)
	{
		GPIO_set_dir(0, 1, 1);
		LPC_SYSCON->CLKOUTDIV = 0x01;
		LPC_SYSCON->CLKOUTSEL = 0x03;
		LPC_SYSCON->CLKOUTUEN = 0x01;
		LPC_SYSCON->CLKOUTUEN = 0x00;
		LPC_SYSCON->CLKOUTUEN = 0x01;
		LPC_IOCON->PIO0_1 |= 0x01;
	} else
	{
		LPC_IOCON->PIO0_1 &= ~0x01;
	}
}

inline void GPIO_set_dir(unsigned int port, unsigned int pin, unsigned int dir)
{
	if (dir)
		LPC_GPIO->DIR[port] |=  (1 << pin);
	else
		LPC_GPIO->DIR[port] &= ~(1 << pin);
}

inline void GPIO_set_value(unsigned int port, unsigned int pin, unsigned int val)
{
	if (val)
		LPC_GPIO->SET[port] = 1 << pin;
	else
		LPC_GPIO->CLR[port] = 1 << pin;
}

inline unsigned int GPIO_get_value(unsigned int port, unsigned int pin)
{
	unsigned int val = 0;

	if (pin < 0x20)
	{
		if (LPC_GPIO->PIN[port] & (1 << pin))
			val = 1;
	}
	else if (pin == 0xFF)
	{
		val = LPC_GPIO->PIN[port];
	}

  return val;
}
