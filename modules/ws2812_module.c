/*
===============================================================================
 Name        : ws2812.h
 Author      : Konstantin Werner
 Version     : 0.1a
 Copyright   : Copyright (C) Konstantin Werner
 Description : WS2812 RGB LED
===============================================================================
*/

#include "env.h"

#include "ws2812_module.h"
#include "gpio_module.h"
#include "delay_module.h"

/* W2812b Timing
 * T0H = 0.40µs +-0.15µs
 * T0L = 0.85µs +-0.15µs
 * T1H = 0.80µs +-0.15µs
 * T1L = 0.45µs +-0.15µs
 */

uint8_t pixels[WS2812_NUMLEDS * 3] = {0};

void WS2812_init(void)
{
	GPIO_init();
	delay_init();

	LPC_IOCON->PIO0_9 = (LPC_IOCON->PIO0_9 & ~(0b11 << 3)) | (0b01 << 3);
	delay_ms(2);
	LPC_IOCON->PIO0_9 = (LPC_IOCON->PIO0_9 & ~(0b11)) | 0b01;

	LPC_SYSCON->PRESETCTRL    |= (1 << 0);	// clear SPI reset, SPI held in reset by default (3.5.2)
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 11);	// enable clock to SPI0 block (3.5.14)
	LPC_SYSCON->SSP0CLKDIV = 1;				// enable SPI clk by writing non-zero clk divisor (3.5.15)
	LPC_SSP0->CPSR = 0x2;					// spi clock prescaler (14.6.5)
	LPC_SSP0->CR0 = (15-1) | (0b00 << 4) | (0b10 << 6) | (1 << 8);	// 15 bit transfer, SPI frame format, CPOL=0 + CPHA=1, SCR=1 (14.6.1)
    LPC_SSP0->CR1 |= (1 << 1); // enable SPI (14.6.2)
//	GPIO_set_dir(WS2812_PORT, WS2812_PIN, OUTPUT);
}

void WS2812_show(void)
{
	static uint32_t endtime = 0;

	uint8_t *data = pixels;
	int32_t len = WS2812_NUMLEDS * 3;

	uint32_t bitmask, bits;

	while ((micros() - endtime) < 50);

	__disable_irq();

	// 800 KHz bitstream
	do
	{
		bitmask = 1 << 7;
		do
		{
			if (*data & bitmask)
				bits = 0b111111111100000;	//10 * 83.333 ~800ns
			else
				bits = 0b111110000000000;

			while (!(LPC_SSP0->SR & (1 << 1))); // wait until !TNF (Transmit Not Full) (14.6.4)

			LPC_SSP0->DR = bits; 				// push bits into Transmit FIFO
			bitmask >>= 1;

		} while(bitmask);

		data++;
	} while(--len);

	while (!(LPC_SSP0->SR & (1<<4))); // wait until ! BSY (Busy) (14.6.4)

	__enable_irq();

	endtime = micros();
}

void WS2812_setpixel_num(const uint32_t num, const uint8_t r, const uint8_t g, const uint8_t b)
{
	if (num < WS2812_NUMLEDS)
	{
		uint8_t *p = &pixels[num * 3];
#if (WS2812_TYPE == GRB)
	    *p++ = g;
	    *p++ = r;
#else
    	*p++ = r;
    	*p++ = g;
#endif
    	*p = b;
	  }
}

void WS2812_setpixel(const uint32_t x, const uint32_t y, const uint8_t r, const uint8_t g, const uint8_t b)
{
	uint32_t num;

	if (y % 2 == 1)
		num = (y + 1) * WS2812_COLUMNS - x - 1;
	else
		num =  y	  * WS2812_COLUMNS + x;

	if (num < WS2812_NUMLEDS)
	{
		uint8_t *p = &pixels[num * 3];
#if (WS2812_TYPE == GRB)
	    *p++ = g;
	    *p++ = r;
#else
    	*p++ = r;
    	*p++ = g;
#endif
    	*p = b;
	  }
}

void WS2812_clear(void)
{
	uint32_t l;
	uint8_t *p = pixels;

	for (l = 0; l < WS2812_NUMLEDS * 3; l++)
	{
		*p++ = 0;
	}
}

void WS2812_fill(const uint8_t r, const uint8_t g, const uint8_t b)
{
	uint32_t l;
	uint8_t *p = pixels;

	for (l = 0; l < WS2812_NUMLEDS; l++)
	{
#if (WS2812_TYPE == GRB)
		*p++ = g;
		*p++ = r;
#else
		*p++ = r;
		*p++ = g;
#endif
		*p++ = b;
	}
}
