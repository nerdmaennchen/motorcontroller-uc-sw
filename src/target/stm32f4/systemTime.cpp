/*
 * systemTime.c
 *
 *  Created on: Apr 6, 2013
 *      Author: lutz
 */

#include <flawless/module/Module.h>

#include <target/stm32f4/clock.h>
#include "interfaces/systemTime.h"

#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/nvic.h>

#define SYSTEM_TIME_TIMER TIM2

#define SYSTEM_TIME_RESOLUTION 1000000ULL


static uint32_t g_systemTimeHighPart = 0;


extern "C"
{
void tim2_isr()
{
	g_systemTimeHighPart += 1;
	TIM_SR(SYSTEM_TIME_TIMER) = 0;
}
}

systemTime_t SystemTime::getSystemTimeUS() const
{
	systemTime_t ret = TIM_CNT(SYSTEM_TIME_TIMER);
	ret += (((systemTime_t)g_systemTimeHighPart) << 32);
	return ret;
}


systemTime_t SystemTime::getCurrentTime() const {
	return getSystemTimeUS();
}


void SystemTime::sleep(systemTime_t duration) const {
	const systemTime_t startTime = getSystemTimeUS();
	while ((startTime + duration) > getSystemTimeUS());
}

namespace
{
struct InitHelper : public flawless::Module
{
	InitHelper(unsigned int level) : flawless::Module(level) {}

	void init(unsigned int) override
	{
		/* Enable TIM clock. */
		RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

		/* Enable TIM interrupt. */
		nvic_enable_irq(NVIC_TIM2_IRQ);
		nvic_set_priority(NVIC_TIM2_IRQ, 4);

		g_systemTimeHighPart = 0U;
		TIM_PSC(SYSTEM_TIME_TIMER) = (CLOCK_APB1_TIMER_CLK / SYSTEM_TIME_RESOLUTION - 1);
		TIM_CNT(SYSTEM_TIME_TIMER) = 0U;
		TIM_ARR(SYSTEM_TIME_TIMER) = 0xffffffff;
		TIM_EGR(SYSTEM_TIME_TIMER) = TIM_EGR_UG;

		TIM_SR(SYSTEM_TIME_TIMER) = 0;
		TIM_DIER(SYSTEM_TIME_TIMER) = TIM_DIER_UIE;

		/* enable timer */
		TIM_CR1(SYSTEM_TIME_TIMER) |= TIM_CR1_CEN;
	}
} initHelper(1);
}
