/*
 * GENERATED FILE - DO NOT EDIT
 * (C) Code Red Technologies Ltd, 2008-2013
 * Generated linker script file for LPC11U24/401
 * Created from generic_c.ld (vLPCXpresso v4.3 (0 [Build 1023] [2012-09-19] ))
 * By LPCXpresso v4.3.0 [Build 1023] [2012-09-19]  on Mon Aug 19 03:39:20 CEST 2013
 */


INCLUDE "stepctrl_Release_lib.ld"
INCLUDE "stepctrl_Release_mem.ld"

ENTRY(ResetISR)

SECTIONS
{

	/* MAIN TEXT SECTION */	
	.text : ALIGN(4)
	{
		FILL(0xff)
		KEEP(*(.isr_vector))
		
		/* Global Section Table */
		. = ALIGN(4) ;
		__section_table_start = .;
		__data_section_table = .;
		LONG(LOADADDR(.data));
		LONG(    ADDR(.data)) ;
		LONG(  SIZEOF(.data));
		LONG(LOADADDR(.data_RAM2));
		LONG(    ADDR(.data_RAM2)) ;
		LONG(  SIZEOF(.data_RAM2));
		__data_section_table_end = .;
		__bss_section_table = .;
		LONG(    ADDR(.bss));
		LONG(  SIZEOF(.bss));
		LONG(    ADDR(.bss_RAM2));
		LONG(  SIZEOF(.bss_RAM2));
		__bss_section_table_end = .;
		__section_table_end = . ;
		/* End of Global Section Table */
		

		*(.after_vectors*)
		
		*(.text*)
		*(.rodata .rodata.*)
		. = ALIGN(4);
		
	} > MFlash32

	/*
	 * for exception handling/unwind - some Newlib functions (in common
	 * with C++ and STDC++) use this. 
	 * Use KEEP so not discarded with --gc-sections
	 */
	.ARM.extab : ALIGN(4)
	{
		KEEP(*(.ARM.extab* .gnu.linkonce.armextab.*))
	} > MFlash32
	__exidx_start = .;
	
	.ARM.exidx : ALIGN(4)
	{
		KEEP(*(.ARM.exidx* .gnu.linkonce.armexidx.*))
	} > MFlash32
	__exidx_end = .;
	
	_etext = .;
		
	
	.data_RAM2 : ALIGN(4)
	{
	   FILL(0xff)
		*(.data.$RAM2*)
		*(.data.$RamUsb2*)
	   . = ALIGN(4) ;
	} > RamUsb2 AT>MFlash32
	
	/* MAIN DATA SECTION */

	.uninit_RESERVED : ALIGN(4)
	{
		KEEP(*(.bss.$RESERVED*))
		. = ALIGN(4) ;
		_end_uninit_RESERVED = .;
	} > RamLoc8

	.data : ALIGN(4)
	{
		FILL(0xff)
		_data = .;
		*(vtable)
		*(.data*)
		. = ALIGN(4) ;
		_edata = .;
	} > RamLoc8 AT>MFlash32

	
	.bss_RAM2 : ALIGN(4)
	{
		*(.bss.$RAM2*)
		*(.bss.$RamUsb2*)
	   . = ALIGN(4) ;
	} > RamUsb2

	/* MAIN BSS SECTION */
	.bss : ALIGN(4)
	{
		_bss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4) ;
		_ebss = .;
		PROVIDE(end = .);
	} > RamLoc8
	
	PROVIDE(_pvHeapStart = .);
	PROVIDE(_vStackTop = __top_RamLoc8 - 0);
}
