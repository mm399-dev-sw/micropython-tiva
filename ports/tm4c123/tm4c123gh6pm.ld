/******************************************************************************
 *
 * Default Linker script for the Texas Instruments TM4C123GH6PM
 *
 * This is derived from revision 15071 of the TivaWare Library.
 * Source: https://e2e.ti.com/support/microcontrollers/other/f/908/p/479812/2164975
 * Modified with the CMSIS linker script
 *
 *****************************************************************************/

MEMORY
{
    /* FLASH (RX) : ORIGIN = 0x00000000, LENGTH = 256K */
	FLASH (RX) : ORIGIN = 0x00000000, LENGTH = 232K
	FLASH_FS (WX) : ORIGIN = 0x0003A000, LENGTH = 24K
	/* SRAM  (WX) : ORIGIN = 0x20000000, LENGTH = 32K */
    SRAM  (WX) : ORIGIN = 0x20000000, LENGTH = 24K
	CACHE (WX) : ORIGIN = 0x20006000, LENGTH = 8K
}

HEAPSIZE = 0x2000;
STACKSIZE = 0x0800;

_Min_Heap_Size = 0x2000;
_Min_Stack_Size = 0x0800;

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_BSS", SRAM);
REGION_ALIAS("REGION_DATA", SRAM);
REGION_ALIAS("REGION_STACK", SRAM);
REGION_ALIAS("REGION_HEAP", SRAM);
REGION_ALIAS("REGION_ARM_EXIDX", FLASH);
REGION_ALIAS("REGION_ARM_EXTAB", FLASH);

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and SRAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   _heap_start
 *   __heap_start__
 *   _heap_end
 *   __heap_end__
 *   __HeapLimit
 *   _stack
 *   __stack
 *   __StackLimit
 *   __StackTop
 *   __stack
 */
ENTRY(Reset_Handler)

SECTIONS
{
	.text :
	{
		KEEP(*(.vectors))
		*(.text*)

		KEEP(*(.init))
		KEEP(*(.fini))

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
 		*crtbegin.o(.dtors)
 		*crtbegin?.o(.dtors)
 		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
 		*(SORT(.dtors.*))
 		*(.dtors)

		*(.rodata*)

		KEEP(*(.eh_frame*))
	} > FLASH

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > FLASH

	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > FLASH
	__exidx_end = .;

	/* To copy multiple ROM to SRAM sections,
	 * uncomment .copy.table section and,
	 * define __STARTUP_COPY_MULTIPLE in startup_ARMCMx.S */
	/*
	.copy.table :
	{
		. = ALIGN(4);
		__copy_table_start__ = .;
		LONG (__etext)
		LONG (__data_start__)
		LONG (__data_end__ - __data_start__)
		LONG (__etext2)
		LONG (__data2_start__)
		LONG (__data2_end__ - __data2_start__)
		__copy_table_end__ = .;
	} > FLASH
	*/

	/* To clear multiple BSS sections,
	 * uncomment .zero.table section and,
	 * define __STARTUP_CLEAR_BSS_MULTIPLE in startup_ARMCMx.S */
	/*
	.zero.table :
	{
		. = ALIGN(4);
		__zero_table_start__ = .;
		LONG (__bss_start__)
		LONG (__bss_end__ - __bss_start__)
		LONG (__bss2_start__)
		LONG (__bss2_end__ - __bss2_start__)
		__zero_table_end__ = .;
	} > FLASH
	*/

	/* Location counter can end up 2byte aligned with narrow Thumb code but
	   __etext is assumed by startup code to be the LMA of a section in SRAM
	   which must be 4byte aligned */
	__etext = ALIGN (4);
	_etext = __etext;
	_sidata = _etext;
	/* TODO */

	.data : AT (__etext)
	{
		__data_start__ = .;
		_sdata = __data_start__;
		*(vtable)
		*(.data*)

		. = ALIGN(4);
		/* preinit data */
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP(*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);

		. = ALIGN(4);
		/* init data */
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);


		. = ALIGN(4);
		/* finit data */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		PROVIDE_HIDDEN (__fini_array_end = .);

		KEEP(*(.jcr*))
		. = ALIGN(4);
		/* All data end */
		__data_end__ = .;
		_edata = __data_end__;

	} > SRAM

	.bss :
	{
		. = ALIGN(4);
		__bss_start__ = .;
		_sbss = __bss_start__;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
		_ebss = __bss_end__;
	} > SRAM

	.heap (COPY):
	{
		__end__ = .;
		__heap_start__ = __end__;
		_heap_start = __heap_start__;
		PROVIDE(end = .);
		*(.heap*)
		__HeapLimit = .;
		__heap_end__ = __HeapLimit;
		_heap_end = __heap_end__;
	} > SRAM

	/* .stack_dummy section doesn't contain any symbols. It is only
	 * used for linker to calculate size of stack sections, and assign
	 * values to stack symbols later */
	.stack_dummy (COPY):
	{
		*(.stack*)
	} > SRAM

	/* Set stack top to end of SRAM, and stack limit move down by
	 * size of stack_dummy section */
	__StackTop = ORIGIN(SRAM) + LENGTH(SRAM);
	_estack = __StackTop;
	__StackLimit = __StackTop - SIZEOF(.stack_dummy);
	PROVIDE(__stack = __StackTop);

	/* Check if data + heap + stack exceeds SRAM limit */
	ASSERT(__StackLimit >= __HeapLimit, "region SRAM overflowed with stack")
}


/* SRAM extents for the garbage collector */
_ram_start = ORIGIN(SRAM);
_ram_end = ORIGIN(SRAM) + LENGTH(SRAM);

_flash_fs_start = ORIGIN(FLASH_FS);
_flash_fs_end = ORIGIN(FLASH_FS) + LENGTH(FLASH_FS);

_cache_start = ORIGIN(CACHE);
_cache_end = ORIGIN(CACHE) + LENGTH(CACHE);
