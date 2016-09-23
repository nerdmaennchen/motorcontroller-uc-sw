MEMORY
{
	rom (rx)     : ORIGIN = 0x08000000, LENGTH = 512K
	ram (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
	garbage (rw) : ORIGIN = 0x00000000, LENGTH = 1K
	config (rw)  : ORIGIN = 0x080e0000, LENGTH = 128K
}

/* Enforce emmition of the vector table. */
EXTERN (vector_table)
ENTRY(reset_handler)

/* Define sections. */
SECTIONS
{
		/DISCARD/ : {}
		
        . = ORIGIN(rom);
		.text : {
                KEEP(*(.vectors))     /* Vector table */
                *(.text*)       /* Program code */
                . = ALIGN(4);
                *(.rodata*)     /* Read-only data */
                
                _initModules_Begin = .;
                KEEP(*(.moduleclasses_init_vector*))
                _initModules_End = .;
                
	            _logStringsBegin = .;
	            KEEP(*(.logStrings*))
	            _logStringsEnd = .;
                
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
                
                _application_configs_begin = .;
                KEEP(*(.application_configs*))
                _application_configs_end = .;
                
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

        .configuration (NOLOAD) : {
                . = ALIGN(4);
                _flashConfigRAMBegin = .;
                KEEP(*(.applicationConfigRAM*))   /* Read-write application configuration */
                . = ALIGN(4);
                _flashConfigRAMEnd = .;
        } >ram AT > config
		
        . = ALIGN(4);
        end = .;
        
        .garbage : AT(ORIGIN(garbage)) 
        {
        	*(.*NEVER*)
        } >garbage
}

PROVIDE(_dataAtFlash = LOADADDR(.data));
PROVIDE(_flashConfigROMBegin = ORIGIN(config));
PROVIDE(_flashConfigROMEnd = ORIGIN(config) + LENGTH(config));

PROVIDE(_stack = ORIGIN(ram) + LENGTH(ram));
PROVIDE(_ramBegin = ORIGIN(ram));
