MEMORY
{
	vector (rx)  : ORIGIN = 0x08000000, LENGTH = 16K
	config (rw)  : ORIGIN = 0x08004000, LENGTH = 16K
	rom (rx)     : ORIGIN = 0x08008000, LENGTH = 96K
	ram (rwx)    : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Enforce emmition of the vector table. */
EXTERN (vector_table)
ENTRY(reset_handler)

/* Define sections. */
SECTIONS
{
		/DISCARD/ : {
			*(.text*dmul*)
			*(.text*dadd*)
		}
		
        . = ORIGIN(vector);
		.vector : {
                KEEP(*(.vectors))     /* Vector table */
        } >vector
		
        . = ORIGIN(rom);
		.text : {
                . = ALIGN(4);
                *(.text*)       /* Program code */
                . = ALIGN(4);
                *(.rodata*)     /* Read-only data */
                _etext = .;
        } >rom

		/* C++ Static constructors/destructors, also used for __attribute__
		 * ((constructor)) and the likes */
		.preinit_array : {
			. = ALIGN(4);
			__preinit_array_start = .;
			KEEP (*(.preinit_array))
			__preinit_array_end = .;
		} >rom
		.init_array : {
			. = ALIGN(4);
			__init_array_start = .;
			KEEP (*(SORT(.init_array.*)))
			KEEP (*(.init_array))
			__init_array_end = .;
		} >rom
		.fini_array : {
			. = ALIGN(4);
			__fini_array_start = .;
			KEEP (*(.fini_array))
			KEEP (*(SORT(.fini_array.*)))
			__fini_array_end = .;
		} >rom
		
		/*
		 * Another section used by C++ stuff, appears when using newlib with
		 * 64bit (long long) printf support
		 */
		.ARM.extab : {
			*(.ARM.extab*)
		} >rom
		.ARM.exidx : {
			__exidx_start = .;
			*(.ARM.exidx*)
			__exidx_end = .;
		} >rom

        . = ORIGIN(ram);

        .data : {
                _data = .;
                *(.data*)       /* Read-write initialized data */
                . = ALIGN(4);
                _edata = .;
        } >ram AT > rom
        PROVIDE(_data_loadaddr = LOADADDR(.data));

        .bss : {
    			. = ALIGN(4);
    			_bss = .;
                *(.bss*)        /* Read-write zero initialized data */
                *(COMMON)
                . = ALIGN(4);
                _ebss = .;
        } >ram
		
        . = ALIGN(4);
        end = .;
}

PROVIDE(_dataAtFlash = LOADADDR(.data));
PROVIDE(_flashConfigROMBegin = ORIGIN(config));
PROVIDE(_flashConfigROMEnd = ORIGIN(config) + LENGTH(config));

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
PROVIDE(_ramBegin = ORIGIN(ram));
