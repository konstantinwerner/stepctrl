/*
===============================================================================
 Name        : ssp_module.c
 Author      : Konstantin Werner
 Version     : 0.1a
 Copyright   : Copyright (C) Konstantin Werner
 Description : SSP Communication
===============================================================================
*/

#include "env.h"

#include "ssp_module.h"
#include "gpio_module.h"

#define PIN0_6 	6
#define PIN0_10	10
#define PIN1_29	29

#if (SCK0_LOC == PIN0_6)
	#define SCK0_PIN	PIO0_6
	#define SCK0_FUNC	0x2
#elif (SCK0_LOC == PIN0_10 && __JTAG_DISABLED)
	#define SCK0_PIN	SWCLK_PIO0_10
	#define SCK0_FUNC	0x2
#else
	#define SCK0_PIN	PIO1_29
	#define	SCK0_FUNC	0x1
#endif

#define	PIN1_15	15
#define PIN1_20	20
#define PIN1_21	21
#define PIN0_22 22
#define PIN1_19	19
#define PIN1_23	23

#if SSEL1_LOC == PIN1_19
	#define SSEL1_PIN	PIO1_19
#else
	#define SSEL1_PIN	PIO1_23
#endif

#if SCK1_LOC == PIN1_15
	#define SCK1_PIN	PIO1_15
	#define SCK1_FUNC	0x3
#else
	#define SCK1_PIN	PIO1_20
	#define	SCK1_FUNC	0x2
#endif

#if MISO1_LOC == PIN1_21
	#define MISO1_PIN	PIO1_21
	#define MISO1_FUNC	0x2
#else
	#define MISO1_PIN	PIO0_22
	#define	MISO1_FUNC	0x3
#endif

/* SSP Status register */
#define SSPSR_TFE       (0x1<<0)
#define SSPSR_TNF       (0x1<<1)
#define SSPSR_RNE       (0x1<<2)
#define SSPSR_RFF       (0x1<<3)
#define SSPSR_BSY       (0x1<<4)

/* SSP CR0 register */
#define SSPCR0_DSS      (0x1<<0)
#define SSPCR0_FRF      (0x1<<4)
#define SSPCR0_SPO      (0x1<<6)
#define SSPCR0_SPH      (0x1<<7)
#define SSPCR0_SCR      (0x1<<8)

/* SSP CR1 register */
#define SSPCR1_LBM      (0x1<<0)
#define SSPCR1_SSE      (0x1<<1)
#define SSPCR1_MS       (0x1<<2)
#define SSPCR1_SOD      (0x1<<3)

/* SSP Interrupt Mask Set/Clear register */
#define SSPIMSC_RORIM   (0x1<<0)
#define SSPIMSC_RTIM    (0x1<<1)
#define SSPIMSC_RXIM    (0x1<<2)
#define SSPIMSC_TXIM    (0x1<<3)

/* SSP0 Interrupt Status register */
#define SSPRIS_RORRIS   (0x1<<0)
#define SSPRIS_RTRIS    (0x1<<1)
#define SSPRIS_RXRIS    (0x1<<2)
#define SSPRIS_TXRIS    (0x1<<3)

/* SSP0 Masked Interrupt register */
#define SSPMIS_RORMIS   (0x1<<0)
#define SSPMIS_RTMIS    (0x1<<1)
#define SSPMIS_RXMIS    (0x1<<2)
#define SSPMIS_TXMIS    (0x1<<3)

/* SSP0 Interrupt clear register */
#define SSPICR_RORIC    (0x1<<0)
#define SSPICR_RTIC     (0x1<<1)

#define SSP0_CR0		(SSP0_DATA_SIZE - 1) | (SSP0_FRAME_FORMAT << 4) | (SSP0_CPOL << 6) | (SSP0_CPHA << 7) | ((SSP0_SCR-1) << 8)
#define SSP1_CR0		(SSP1_DATA_SIZE - 1) | (SSP1_FRAME_FORMAT << 4) | (SSP1_CPOL << 6) | (SSP1_CPHA << 7) | ((SSP1_SCR-1) << 8)


void SSP_init(uint8_t port)
{
	uint8_t i, Dummy=Dummy;

	if (port == 0)
	{
		LPC_SYSCON->PRESETCTRL		|= (1 << 0);
		LPC_SYSCON->SYSAHBCLKCTRL	|= (1 << 11);
		LPC_SYSCON->SSP0CLKDIV		 =  0x02;		/* Divided by 2 */
		LPC_IOCON->PIO0_8			&= ~0x07;		/* SSP I/O config */
		LPC_IOCON->PIO0_8       	|=  0x01;		/* SSP MISO */
		LPC_IOCON->PIO0_21       	&= ~0x07;
		LPC_IOCON->PIO0_21       	|=  0x01;		/* SSP MOSI */
		LPC_IOCON->SCK0_PIN 		&= ~0x07;
		LPC_IOCON->SCK0_PIN 		|= SCK0_FUNC;	/* SSP CLK */

#if USE_CS_0
		LPC_IOCON->PIO0_2 &= ~0x07;
		LPC_IOCON->PIO0_2 |= 0x01;					/* SSP SSEL */
#else
		LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6);		/* Enable AHB clock to the GPIO domain. */
		LPC_IOCON->PIO0_2 &= ~0x07;					/* SSP SSEL is a GPIO pin */
		GPIO_set_dir(PORT0, 2, 1);					/* Pin is set to GPIO output and high */
		GPIO_set_value(PORT0, 2, 1);
#endif

		LPC_SSP0->CR0  = SSP0_CR0;	/* Set DSS data length, Frame format, CPOL, CPHA, and SCR*/
		LPC_SSP0->CPSR = 0x2;		/* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02 */

		for (i = 0; i < FIFOSIZE; i++)
			Dummy = LPC_SSP0->DR;		/* clear the RxFIFO */

		NVIC_EnableIRQ(SSP0_IRQn);	/* Enable the SSP Interrupt */

		#if LOOPBACK_MODE_1
		LPC_SSP0->CR1 = SSPCR1_LBM | SSPCR1_SSE;
		#else
		#if SSP0_MODE == SSP_SLAVE
		if ( LPC_SSP0->CR1 & SSPCR1_SSE )
			LPC_SSP0->CR1 &= ~SSPCR1_SSE;	/* The slave bit can't be set until SSE bit is zero. */

		LPC_SSP0->CR1 = SSPCR1_MS;			/* Enable slave bit first */
		LPC_SSP0->CR1 |= SSPCR1_SSE;		/* Enable SSP */
		#elif SSP0_MODE == SSP_MASTER
		LPC_SSP0->CR1 = SSPCR1_SSE;
		#endif
		#endif
		LPC_SSP0->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM; /* Set SSPINMS registers to enable interrupts, enable all error related interrupts */
	}
	else
	{
		LPC_SYSCON->PRESETCTRL 		|= (1 << 2);
		LPC_SYSCON->SYSAHBCLKCTRL	|= (1 << 18);
		LPC_SYSCON->SSP1CLKDIV 		 =  0x02;		/* Divided by 2 */
		LPC_IOCON->MISO1_PIN 		&= ~0x07;		/* SSP I/O config */
		LPC_IOCON->MISO1_PIN 		|=  MISO1_FUNC;	/* SSP MISO */
		LPC_IOCON->PIO0_21 			&= ~0x07;
		LPC_IOCON->PIO0_21 			|=  0x02;		/* SSP MOSI */
		LPC_IOCON->SCK1_PIN 		&= ~0x07;
		LPC_IOCON->SCK1_PIN 		|=  SCK1_FUNC;	/* SSP CLK */

#if USE_CS_1
		LPC_IOCON->SSEL1_PIN &= ~0x07;
		LPC_IOCON->SSEL1_PIN |=  0x02;			/* SSP SSEL */
#else
		LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6);	/* Enable AHB clock to the GPIO domain. */
		LPC_IOCON->SSEL1_PIN &= ~0x07;			/* SSP SSEL is a GPIO pin */
		GPIO_set_dir(1, SSEL1_LOC, 1);			/* Pin is set to GPIO output and high */
		GPIO_set_value(1, SSEL1_LOC, 1);
#endif

		LPC_SSP1->CR0  = SSP1_CR0;	/* Set DSS data length, Frame format, CPOL, CPHA, and SCR*/
		LPC_SSP1->CPSR = 0x2;		/* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02 */
	
		for (i = 0; i < FIFOSIZE; i++)
			Dummy = LPC_SSP1->DR;		/* clear the RxFIFO */
	
		NVIC_EnableIRQ(SSP1_IRQn); 		/* Enable the SSP Interrupt */

		/* Device select as master, SSP Enabled */
		#if LOOPBACK_MODE_1
		LPC_SSP1->CR1 = SSPCR1_LBM | SSPCR1_SSE;
		#else
		#if SSP1_MODE == SSP_SLAVE
		if ( LPC_SSP1->CR1 & SSPCR1_SSE )
			LPC_SSP1->CR1 &= ~SSPCR1_SSE;   /* The slave bit can't be set until SSE bit is zero. */

		LPC_SSP1->CR1 = SSPCR1_MS;		/* Enable slave bit first */
		LPC_SSP1->CR1 |= SSPCR1_SSE;	/* Enable SSP */
		#elif SSP1_MODE == SSP_MASTER
		LPC_SSP1->CR1 = SSPCR1_SSE;
		#endif
		#endif
		LPC_SSP1->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM;	/* Set SSPINMS registers to enable interrupts, enable all error related interrupts */
	}
}

#if USE_CS_0 == 0
inline void SSP0_CS(uint8_t cs)
{
	GPIO_set_value(PORT0, 2, cs);
}
#endif

#if USE_CS_1 == 0
inline void SSP1_CS(uint8_t cs)
{
	GPIO_set_value(1, SSEL1_LOC, cs);
}
#endif

void SSP_send(uint8_t port, uint32_t size, uint8_t *data)
{
	uint32_t i;
	uint8_t Dummy = Dummy;

	if (port == 0)
	{
		for (i = 0; i < size; i++)
		{
			while ((LPC_SSP0->SR & (SSPSR_TNF | SSPSR_BSY)) != SSPSR_TNF);	/* Move on only if NOT busy and TX FIFO not full. */
			LPC_SSP0->DR = *data;
			data++;
#if !LOOPBACK_MODE_0
			while ((LPC_SSP0->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE);
			/* Whenever a byte is written, MISO FIFO counter increments, Clear FIFO
			on MISO. Otherwise, when SSP0Receive() is called, previous data byte
			is left in the FIFO. */
			Dummy = LPC_SSP0->DR;
#else
			/* Wait until the Busy bit is cleared. */
			while (LPC_SSP0->SR & SSPSR_BSY);
#endif
		}
	}
	else
	{
		for (i = 0; i < size; i++)
		{
			while ((LPC_SSP1->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF);	/* Move on only if NOT busy and TX FIFO not full. */
			LPC_SSP1->DR = *data;
			data++;
#if !LOOPBACK_MODE_1
			while ((LPC_SSP1->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE);
			/* Whenever a byte is written, MISO FIFO counter increments, Clear FIFO
			on MISO. Otherwise, when SSP0Receive() is called, previous data byte
			is left in the FIFO. */
			Dummy = LPC_SSP1->DR;
#else
			while (LPC_SSP1->SR & SSPSR_BSY);	/* Wait until the Busy bit is cleared. */
#endif
		}
	}
}

void SSP_receive(uint8_t port, uint32_t size, uint8_t *data)
{
	uint32_t i;

	  for (i = 0; i < size; i++)
	  {
			/* As long as Receive FIFO is not empty, I can always receive. */
			/* If it's a loopback test, clock is shared for both TX and RX,
			no need to write dummy byte to get clock to get the data */
			/* if it's a peer-to-peer communication, SSPDR needs to be written
			before a read can take place. */
			if (port == 0)
			{
#if !LOOPBACK_MODE_0
#if SSP_SLAVE_0
				while (!(LPC_SSP0->SR & SSPSR_RNE));
#else
				LPC_SSP0->DR = 0xFF;
				while ((LPC_SSP0->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE);	/* Wait until the Busy bit is cleared */
#endif
#else
				while (!(LPC_SSP0->SR & SSPSR_RNE));
#endif
				*data = LPC_SSP0->DR;
				data++;
			}
			else
			{
#if !LOOPBACK_MODE_1
#if SSP_SLAVE_1
				while (!(LPC_SSP1->SR & SSPSR_RNE));
#else
				LPC_SSP1->DR = 0xFF;
				while ((LPC_SSP1->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE);	/* Wait until the Busy bit is cleared */
#endif
#else
			  	while (!(LPC_SSP1->SR & SSPSR_RNE));
#endif
				*data = LPC_SSP1->DR;
				data++;
			}
	  }
}
