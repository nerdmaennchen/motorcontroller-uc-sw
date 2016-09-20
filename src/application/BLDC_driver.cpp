#include "PWMDriver.h"
#include "PWMLookupTableGenerator.h"
#include "HallFeedback.h"

#include <flawless/core/Message.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/module/Module.h>

#include <interfaces/systemTime.h>

#include <cmath>
#include <algorithm>


namespace {


struct BLDC_driver final :
		public flawless::Module,
		public flawless::Callback<bool&, bool>,
		public flawless::Callback<float&, bool>,
		public flawless::Callback<int&, bool>,
		public pwmdriver::DriverInterface,
		public flawless::Listener<HallFeedback, 0> {
	flawless::ApplicationConfig<bool> mEnableBLDC                   {"bldc_enable", "B", this, true};
	flawless::ApplicationConfig<Array<uint16_t, 8>> mCWHallIndexes  {"bldc_cwConfigs", "8H"};
	flawless::ApplicationConfig<Array<uint16_t, 8>> mCCWHallIndexes {"bldc_ccwConfigs", "8H"};
	flawless::ApplicationConfig<int> mTarget_frequency_mHz          {"bldc_target_frequency", "i", this, 0};
	flawless::ApplicationConfig<int> mMaxAdvance                       {"bldc_advance", "i", 150};
	flawless::ApplicationConfig<float> mShapeFactor                 {"bldc_shape_factor", "f", this, 1.25f};

	flawless::ApplicationConfig<float> mControllerP                 {"bldc_controll_p", "f", .1f};

	flawless::Message<HallFeedback> mLastHallFeedback;

	int32_t mLastKnownPhase          {0};
	int32_t mTargetTickFrequency_mHz {0};

	uint32_t StepsCount {0};
	pwmdriver::Driver* driver {nullptr};
	BLDC_driver(unsigned int level) : flawless::Module(level) {}

	SystemTime& time = SystemTime::get();
	systemTime_t mExpectedTickTime;

	void unclaim() override {
		mEnableBLDC = false;
	}

	void callback(bool& enable, bool setter) {
		if (setter) {
			if (enable) {
				driver->claim(this);
				callback(mTarget_frequency_mHz, true);

				// setup the pwm configs
				PWMLookupTableGeneratorShaped lookupGenerator;
				lookupGenerator.generateLookupTable(3, StepsCount, mShapeFactor, driver->getPattern() + StepsCount * 0);
				lookupGenerator.generateLookupTable(3, StepsCount, mShapeFactor, driver->getPattern() + StepsCount * 1);
				lookupGenerator.generateLookupTableReversed(3, StepsCount, mShapeFactor, driver->getPattern() + StepsCount * 2);
				lookupGenerator.generateLookupTableReversed(3, StepsCount, mShapeFactor, driver->getPattern() + StepsCount * 3);;
			}
		}
	}

	void callback(float&, bool setter) {
		callback(mEnableBLDC, setter);
	}

	void callback(int& target_frequency_mHz, bool setter) {
		if (setter) {
			mTargetTickFrequency_mHz = 6 * target_frequency_mHz;
		}
	}

	void callback(flawless::Message<HallFeedback> const& hallFeedback) {
		flawless::LockGuard lock;
		mLastHallFeedback = hallFeedback;

		if (mLastHallFeedback->movingDirection == MovingDirection::CW) {
			mLastKnownPhase = mCWHallIndexes.get()[mLastHallFeedback->currentHallValues];
		} else {
			mLastKnownPhase = mCCWHallIndexes.get()[mLastHallFeedback->currentHallValues];
		}
		if (mEnableBLDC) {
			int64_t avgTickFreq =mLastHallFeedback->averageTickFreq_mHz;
			float speedError = float(mTargetTickFrequency_mHz - avgTickFreq) / 1000.f;
			float controll = speedError * mControllerP;
			float powerOutput = std::max(1.f, std::abs(controll));
			driver->setPower(powerOutput);
			int targetIndex = std::round(mLastKnownPhase + mMaxAdvance * controll);
			if (avgTickFreq > 0) { // run clockwise
				driver->runSteps(targetIndex, StepsCount / 6, avgTickFreq * StepsCount / 6,false);
			} else { // run counter clockwise
				driver->runSteps(3 * StepsCount - targetIndex, StepsCount / 6, avgTickFreq * StepsCount / 6,false);
			}
		}
	}

	int myModulo(int a, int b) {
		return (b + a % b) % b;
	}

	void init(unsigned int) {
		driver = &(flawless::util::Singleton<pwmdriver::Driver>::get());
		StepsCount = driver->getPatternSize() / 4;
		callback(mEnableBLDC, true);


		// setup the lookup tables
		uint8_t step0 = 2; // at phase 0 we read a hall feedback of 2
		for (int i=0; i < 6; ++i) {
			mCWHallIndexes.get()[step0]  = myModulo(-(StepsCount/12) + (StepsCount/6) * i, StepsCount);
			mCCWHallIndexes.get()[step0] = myModulo( (StepsCount/12) + (StepsCount/6) * i, StepsCount);
			step0 = hallUtils::getNextStep(step0, true);
		}
	}
} driver(10);

}
