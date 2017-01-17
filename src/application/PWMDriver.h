#pragma once
#include <flawless/stdtypes.h>
#include <flawless/util/Array.h>
#include <flawless/util/Singleton.h>
#include <libopencm3/stm32/f4/timer.h>

#include <target/stm32f4/clock.h>

namespace pwmdriver
{

#define PWM_TIMER TIM1
constexpr float pwm_timer_base_freq = float(CLOCK_APB2_TIMER_CLK);
constexpr float pwm_target_freq     = float(30.e3f); // a frequency well outside the audible range

// off time is before the PWM and after to there is always a PWM_OFF_TIME*2 off time between two consecutive PWM pulses
constexpr uint32_t PwmPreOffTimer       = 8; // has to be something more than zero to enable a timeframe to fetch hall data via DMA
constexpr uint32_t PwmAmplitude         = 64;

constexpr uint32_t PwmCentralDutyMoment = PwmPreOffTimer + PwmAmplitude / 2;
constexpr uint32_t PwmMinCyclePeriod    = PwmPreOffTimer + PwmAmplitude;

constexpr uint32_t StepsCount   = 600;
constexpr uint32_t TicksPerStep = 4;
constexpr uint32_t TotalTickCnt = StepsCount * TicksPerStep;
constexpr size_t   CommutationPatternBufferSize = 4 * StepsCount;


using CommutationPattern = Array<Array<uint16_t, TicksPerStep>, CommutationPatternBufferSize>;

struct DriverInterface {
	virtual void unclaim() = 0;
};

class Driver : public flawless::util::Singleton<pwmdriver::Driver> {
	public:
		CommutationPattern& getPattern();

		// make the PWM output the values from stepStart to stepStart+stepCnt automatically with
		void runSteps(uint32_t stepStart, uint32_t stepCnt, uint32_t delayUS, bool cycle=true);
		void set_delay(uint32_t delayUS);

		// get the index of the last performed step
		uint32_t getCurStep() const;
		// returns the current speed at which the dma updates the pwm timer
		uint32_t getCurTickSpeed() const;

		void setEnabled(bool enabled);

		void claim(DriverInterface* interface);
};

}
