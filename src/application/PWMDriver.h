#pragma once
#include <flawless/stdtypes.h>
#include <flawless/util/Array.h>
#include <flawless/util/Singleton.h>

namespace pwmdriver
{

#define PWM_TIMER TIM1

// off time is before the PWM and after to there is always a PWM_OFF_TIME*2 off time between two consecutive PWM pulses
constexpr uint32_t PwmPreOffTimer       = 16; // has to be something more than zero to enable a timeframe to fetch hall data via DMA
constexpr uint32_t PwmAmplitude         = 512;
constexpr uint32_t PwmPostOffTimer      = 16;

constexpr uint32_t PwmCentralDutyMoment = PwmPreOffTimer + PwmAmplitude / 2;
constexpr uint32_t PwmMinCyclePeriod    = PwmPreOffTimer + PwmAmplitude + PwmPostOffTimer;

constexpr uint32_t StepsCount   = 600;
constexpr uint32_t TicksPerStep = 4;
constexpr uint32_t TotalTickCnt = StepsCount * TicksPerStep;


using CommutationPattern = Array<uint16_t, TicksPerStep>;

struct DriverInterface {
	virtual void unclaim() = 0;
};

class Driver : public flawless::util::Singleton<pwmdriver::Driver> {
	public:
		CommutationPattern* getPattern();
		// get the amount of steps in the pattern;
		uint32_t getPatternSize();

		// make the PWM output the values from stepStart to stepStart+stepCnt automatically with
		void runSteps(uint32_t stepStart, uint32_t stepCnt, uint32_t mHZ, bool cycle=true);
		void set_mHZ(uint32_t mHZ);

		// get the index of the last performed step
		uint32_t getCurStep();

		void setEnabled(bool enabled);

		void claim(DriverInterface* interface);
};

}
