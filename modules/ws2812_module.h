/*
===============================================================================
 Name        : ws2812.h
 Author      : Konstantin Werner
 Version     : 0.1a
 Copyright   : Copyright (C) Konstantin Werner
 Description : WS2812 RGB LED
===============================================================================
*/

#ifndef WS2812_MODULE_H_
#define WS2812_MODULE_H_

#define WS2812_COLUMNS	10
#define WS2812_ROWS		10

#define WS2812_NUMLEDS	(WS2812_COLUMNS * WS2812_ROWS)

#define WS2812_TYPE		GRB		// RGB or GRB

#define WS2812CMD_SHOW	0
#define WS2812CMD_CLEAR	1
#define WS2812CMD_FILL	2
#define WS2812CMD_PIXEL	3

extern uint8_t pixels[WS2812_NUMLEDS * 3];

void WS2812_init(void);
void WS2812_show(void);
void WS2812_setpixel_num(const uint32_t num, const uint8_t r, const uint8_t g, const uint8_t b);
void WS2812_setpixel(const uint32_t x, const uint32_t y, const uint8_t r, const uint8_t g, const uint8_t b);
void WS2812_clear(void);
void WS2812_fill(const uint8_t r, const uint8_t g, const uint8_t b);

#endif /* W2812_MODULE_H_ */
