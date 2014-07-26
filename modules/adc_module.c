/*
===============================================================================
 Name        : adc_module.c
 Author      : Konstantin Werner
 Version     : 0.1
  Description : Interfaces the integrated ADC
===============================================================================
*/
#include "env.h"
#include "adc_module.h"

#define ADC_NUM 7
#define ADC_OFFSET		0x10
#define ADC_INDEX		4

#define ADC_DONE		0x80000000
#define ADC_OVERRUN		0x40000000
#define ADC_ADINT		0x00010000

void ADC_init(uint32_t ADC_Clk, uint8_t channels)//, uint8_t resolution)
{
	LPC_SYSCON->PDRUNCFG      &= ~(1 << 4);	// Disable Power down bit to the ADC block.

    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 13);	// Enable AHB clock to the ADC.

    if (channels & (1 << 0))
    	LPC_IOCON->TDI_PIO0_11 = 0x02;	// Select AD0 pin function

    if (channels & (1 << 1))
    	LPC_IOCON->PIO1_0 = 0x02;		// Select AD1 pin function

    if (channels & (1 << 2))
    	LPC_IOCON->PIO1_1 = 0x02;		// Select AD2 pin function

    if (channels & (1 << 3))
    	LPC_IOCON->PIO1_2 = 0x02;		// Select AD3 pin function

    if (channels & (1 << 4))
    	LPC_IOCON->PIO1_3 = 0x02; 		// Select AD4 pin function

    if (channels & (1 << 5))
    	LPC_IOCON->PIO1_4    = 0x01;	// Select AD5 pin function

    if (channels & (1 << 6))
    	LPC_IOCON->PIO1_10   = 0x01;	// Select AD6 pin function

    if (channels & (1 << 7))
    	LPC_IOCON->PIO1_11   = 0x01;	// Select AD7 pin function

	LPC_ADC->CR  = (((CPU_CLK/LPC_SYSCON->SYSAHBCLKDIV) / ADC_Clk - 1) << 8);	// Set ADC Clock
//	LPC_ADC->CR |= (resolution << 17);	// Only in Burst Mode
}

uint32_t ADC_read(uint8_t channel)
{
	uint32_t adc_data;

	if (channel > ADC_NUM)
		channel = 0;

	/* Select channel, start A/D convert */
	LPC_ADC->CR &= 0xFFFFFF00;
	LPC_ADC->CR |= (1 << 24) | (1 << channel);

	while (1)
	{
		adc_data = *(volatile unsigned long *)(LPC_ADC_BASE + ADC_OFFSET + ADC_INDEX * channel);
		if (adc_data & ADC_DONE) break;
	}

	LPC_ADC->CR &= 0xF8FFFFFF;	// stop ADC
	if (adc_data & ADC_OVERRUN) return (0);

	return( (adc_data >> 6) & 0x3FF);
}

