#include <interfaces/memory.h>


/* Symbols exported by the linker script(s): */
extern unsigned int _data_loadaddr, _data, _edata,_bss,  _ebss, _ramBegin;
typedef void (*funcp_t) (void);
extern funcp_t __preinit_array_start, __preinit_array_end;
extern funcp_t __init_array_start, __init_array_end;


void memory_init(void)
{

	volatile unsigned *src, *dest;
	for (dest = &_bss; dest < &_ebss; ++dest)
	{
		*dest = 0U;
	}
	for (src = &_data_loadaddr, dest = &_data; dest < &_edata; src++, dest++)
	{
		*dest = *src;
	}

	/* Constructors. */
	funcp_t *fp;
	for (fp = &__preinit_array_start; fp < &__preinit_array_end; fp++) {
		(*fp)();
	}
	for (fp = &__init_array_start; fp < &__init_array_end; fp++) {
		(*fp)();
	}
}

