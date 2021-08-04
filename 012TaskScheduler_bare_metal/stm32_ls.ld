ENTRY(Reset_Handler)

MEMORY
{
	FLASH(rx):ORIGIN = 0x08000000,LENGTH = 1024K
	SRAM(rwx):ORIGIN = 0x20000000,LENGTH = 128K
}

SECTIONS
{
	.text :
	{
		*(.isr_vector)
		*(.text)
		*(.text.*) 		/* Merge standard C library sections */ 
		*(.init)
		*(.fini)
		*(.rodata)
		*(.rodata.*)
		. = ALIGN(4);	/* Word-align */
		_etext = .;		/* Location counter tracks VMA */
	}> FLASH

	_la_data = LOADADDR(.data);
	.data :
	{
		_sdata = .;
		*(.data)
		*(.data.*)		/* Merge standard C library sections */ 
		. = ALIGN(4);
		_edata = .;
	}> SRAM AT> FLASH

	.bss :
	{
		_sbss = .;
		__bss_start__ = _sbss;
		*(.bss)
		*(.bss.*)		/* Merge standard C library sections */ 
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		__bss_end__ = _ebss;
		. = ALIGN(4);
		end = .;
		__end__ = .; /* For rdimon */
	}> SRAM

}
