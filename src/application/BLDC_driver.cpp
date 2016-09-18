#include "PWMDriver.h"
#include "PWMLookupTableGenerator.h"
#include "HallFeedback.h"

#include <flawless/core/Message.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/module/Module.h>

#include <algorithm>


namespace {


struct BLDC_driver :
		public flawless::Module,
		public flawless::Callback<bool&, bool>,
		public pwmdriver::DriverInterface,
		public flawless::Listener<HallFeedback, 0> {
	flawless::ApplicationConfig<bool> enableBLDC{"bldc_enable", this, true};
	flawless::ApplicationConfig<Array<uint16_t, 8>> mCWHallIndexes{"bldc_cwConfigs"};
	flawless::ApplicationConfig<Array<uint16_t, 8>> mCCWHallIndexes{"bldc_ccwConfigs"};
	flawless::ApplicationConfig<int> mTargetSpeed{"bldc_targetSpeed", 0};
	flawless::ApplicationConfig<int> mAdvance    {"bldc_advance", 150};
	flawless::Message<HallFeedback> mLastHallFeedback;

	uint32_t StepsCount;
	pwmdriver::Driver* driver;
	BLDC_driver(unsigned int level) : flawless::Module(level) {}

	void callback(bool& enable, bool setter) {
		if (setter) {
			if (enable) {
				PWMLookupTableGenerator lookupGenerator;
				driver->claim(this);

				// setup the pwm configs
				StepsCount = driver->getPatternSize() / 4;
				lookupGenerator.generateLookupTable(3, StepsCount, driver->getPattern() + StepsCount * 0);
				lookupGenerator.generateLookupTable(3, StepsCount, driver->getPattern() + StepsCount * 1);
				lookupGenerator.generateLookupTableReversed(3, StepsCount, driver->getPattern() + StepsCount * 2);
				lookupGenerator.generateLookupTableReversed(3, StepsCount, driver->getPattern() + StepsCount * 3);;
			}
		}
	}

	void callback(flawless::Message<HallFeedback> const& hallFeedback) {
		mLastHallFeedback = hallFeedback;
		if (enableBLDC) {
			uint8_t hallStates = (*mLastHallFeedback).currentHallValues;
			uint64_t speed = 1000000000LL / ((*mLastHallFeedback).lastTickDelay * StepsCount / 12);
			speed = std::max(1ULL, speed);
			speed = std::min(speed, uint64_t(std::abs(mTargetSpeed)));
			if (mTargetSpeed > 0) {
				uint32_t start = mCWHallIndexes.get()[hallStates];
				uint32_t target = start + mAdvance;
				driver->runSteps(target, StepsCount / 12, speed, false);
			} else if (mTargetSpeed < 0) {
				int32_t start = StepsCount*2 + mCCWHallIndexes.get()[hallStates];
				int32_t target = start + mAdvance;
				driver->runSteps(target, StepsCount / 12, speed, false);
			}
		}
	}

	int myModulo(int a, int b) {
		return (b + a % b) % b;
	}

	void init(unsigned int) {
		driver = &(flawless::util::Singleton<pwmdriver::Driver>::get());
		callback(enableBLDC.get(), true);


		// setup the lookup tables
		uint8_t step0 = 2; // at phase 0 we read a hall feedback of 2
		for (int i=0; i < 6; ++i) {
			mCWHallIndexes.get()[step0]  = myModulo(-50 + 100 * i, StepsCount);
			step0 = hallUtils::getNextStep(step0, true);
		}
		for (int i=0; i < 6; ++i) {
			// this pattern looks pretty mixed up but it represents the order in which we cycle through the CCWConfigs
			mCCWHallIndexes.get()[step0] = myModulo(50 + 100 * i, StepsCount);
			step0 = hallUtils::getNextStep(step0, false);
		}
	}
} driver(10);

}
