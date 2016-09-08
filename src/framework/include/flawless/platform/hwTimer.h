#pragma once

#include <inttypes.h>
#include <flawless/util/Singleton.h>



/*
 * functions implemented on hw_timer
 */
namespace flawless
{
namespace platform
{

class HWTimer final: public flawless::util::Singleton<HWTimer> {
public:
	using hw_timerTicks = uint32_t;
	using timer_TimeInterval_us = uint32_t;
	hw_timerTicks getTicksForInterval_us(timer_TimeInterval_us interval);
	hw_timerTicks getTicksElapsed();
	void setupTimer(hw_timerTicks);
};

}
}
