/*
 * float.c
 *
 *  Created on: Nov 21, 2012
 *      Author: lutz
 */


#include <flawless/module/Module.h>
#include <flawless/stdtypes.h>

#define SCB_CPACR (*(volatile uint32_t *)(0xE000ED88))

namespace
{

struct SCBMod : public flawless::Module<0>
{
	void init() override {
		/* enable floating point calculation on coprocessor */
		SCB_CPACR |= (3 << 22) | (3 << 20);
	}
} scbModule;
}
