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

ENTRY(Reset_Handler)

SECTIONS {

    PROVIDE (_intvecs_base_address = 0x0);

    .intvecs (_intvecs_base_address) : AT (_intvecs_base_address) {
        KEEP (*(.vectors))
    } > REGION_TEXT

    PROVIDE (_vtable_base_address = 0x20000000);

    .vtable (_vtable_base_address) : AT (_vtable_base_address) {
        KEEP (*(.vtable))
    } > REGION_DATA

    .text : {
        CREATE_OBJECT_SYMBOLS
        *(.text)
        *(.text.*)
        . = ALIGN(0x4);
        KEEP (*(.ctors))
        . = ALIGN(0x4);
        KEEP (*(.dtors))
        . = ALIGN(0x4);
        __init_array_start = .;
        KEEP (*(.init_array*))
        __init_array_end = .;
        *(.init)
        *(.fini*)
    } > REGION_TEXT

    PROVIDE (__etext = .);
    PROVIDE (_etext = .);
    PROVIDE (etext = .);
	PROVIDE (_sidata = .);

    .rodata : {
        *(.rodata)
        *(.rodata*)
    } > REGION_TEXT

    .data : ALIGN (4) {
        __data_load__ = LOADADDR (.data);
        __data_start__ = .;
		_sdata = __data_start__;
        *(.data)
        *(.data*)
        . = ALIGN (4);
        __data_end__ = .;
		_edata = __data_end__;
    } > REGION_DATA AT> REGION_TEXT

    .ARM.exidx : {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > REGION_ARM_EXIDX

    .ARM.extab : {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > REGION_ARM_EXTAB

    .bss : {
        __bss_start__ = .;
		_sbss = __bss_start__;
        *(.shbss)
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN (4);
        __bss_end__ = .;
		_ebss = __bss_end__;
    } > REGION_BSS

    .heap : {
        __heap_start__ = .;
        end = __heap_start__;
        _heap_start = __heap_start__;
        _end = end;
        __end = end;
        . = . + HEAPSIZE;
        KEEP(*(.heap))
        __heap_end__ = .;
        __HeapLimit = __heap_end__;
        _heap_end = __heap_end__;
    } > REGION_HEAP

    .stack : ALIGN(0x8) {
        _stack = .;
        __stack = .;
        . = . + STACKSIZE;
        __StackTop = . ;
        _estack = __StackTop;
        KEEP(*(.stack))
    } > REGION_STACK

	ASSERT(_Min_Heap_Size <= (_heap_end - _heap_start), "Could not allocate enough space for heap")
	ASSERT(_Min_Stack_Size <= (_estack - _stack), "Could not allocate enough space for stack")
}


/* SRAM extents for the garbage collector */
_ram_start = ORIGIN(SRAM);
_ram_end = ORIGIN(SRAM) + LENGTH(SRAM);

_flash_fs_start = ORIGIN(FLASH_FS);
_flash_fs_end = ORIGIN(FLASH_FS) + LENGTH(FLASH_FS);

_cache_start = ORIGIN(CACHE);
_cache_end = ORIGIN(CACHE) + LENGTH(CACHE);
