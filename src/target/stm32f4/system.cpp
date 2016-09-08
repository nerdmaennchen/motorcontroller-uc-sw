

#include <flawless/stdtypes.h>
#include <flawless/platform/system.h>

#include <flawless/logging/logging.h>
#include <flawless/module/Module.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/wwdg.h>

namespace {
volatile uint16_t g_lockCounter = 0U;
}

void system_mutex_lock()
{
	__asm("CPSID I"); /* Disable Interrupts */
	++g_lockCounter;
}


void system_mutex_unlock()
{
	if ( g_lockCounter > 0U)
	{
		--g_lockCounter;
	} else
	{
		LOG_INFO_0("resumeAllInterrupts too often called g_lockCounter == 0U");
	}
	if (0 == g_lockCounter)
	{
		__asm("CPSIE I"); /* enable Interrupts */
	}
}



void system_reset(void)
{
	RCC_APB1ENR |= RCC_APB1ENR_WWDGEN;

	WWDG_CR = WWDG_CR_WDGA;
}


namespace
{
struct InitHelper : public flawless::Module
{
	InitHelper(unsigned int level) : flawless::Module(level) {}
	void init(unsigned int) override {
		g_lockCounter = 0U;
	}
} initHelper(0);
}
