/*
 * random.c
 *
 *  Created on: Jun 13, 2013
 *      Author: lutz
 */

#include <flawless/module/Module.h>
#include <flawless/stdtypes.h>


#include <libopencm3/stm32/f4/rnd.h>
#include <libopencm3/stm32/f4/rcc.h>


uint32_t getRandomNumber()
{
	return RNG_DR;
}

namespace
{

struct InitHelper : public flawless::Module
{
	InitHelper(unsigned int level) : flawless::Module(level) {}
	void init(unsigned int) override {
		RCC_AHB2ENR |= RCC_AHB2ENR_RNGEN;
		RNG_CR |= RNG_CR_RNGEN;
	}
} initHelper(2);

}
