#include "PWMDriver.h"
#include "PWMLookupTableGenerator.h"
#include "HallFeedback.h"
#include "CurrentController.h"

#include "controller/PIDController.h"

#include <flawless/core/Message.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/module/Module.h>

#include <interfaces/systemTime.h>

#include <cmath>
#include <algorithm>


namespace {
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


struct BLDC_driver final :
		public flawless::Module,
		public flawless::Callback<bool&, bool>,
		public flawless::Callback<float&, bool>,
		public flawless::Callback<int&, bool>,
		public pwmdriver::DriverInterface,
		public flawless::Listener<hall::Feedback, 0> {
	flawless::ApplicationConfig<bool> mEnableBLDC                   {"bldc.enable", "B", this, false};
	flawless::ApplicationConfig<Array<uint16_t, 8>> mHallIndexes    {"bldc.hall_mappings", "8H"};
	flawless::ApplicationConfig<int> mTarget_frequency_mHz          {"bldc.target_frequency", "i", this, 0};
	flawless::ApplicationConfig<int> mMaxAdvance                    {"bldc.advance", "i", -150};
	flawless::ApplicationConfig<float> mShapeFactor                 {"bldc.shape_factor", "f", this, 0.f};
	flawless::ApplicationConfig<int> mLastKnownPhase                {"bldc.cur_phase", "i", 0};

	flawless::ApplicationConfig<PIDControllerParams> mControllerParams {"bldc_controller", "6f", {0.f,0.f,0.f,
			2.5e-4f, // default P
			2.e-2f,  // default I
			2.6e-4f,
	}};

	PIDController mController{&(mControllerParams.get())};

	float mTargetTickFrequency {0};
	uint32_t mTargetStepFrequency {0};

	uint32_t StepsCount {0};
	pwmdriver::Driver* driver {nullptr};
	CurrentController& currentController {CurrentController::get()};
	BLDC_driver(unsigned int level) : flawless::Module(level) {}

	SystemTime& time = SystemTime::get();
	systemTime_t mLastUpdateTime;

	void unclaim() override {
		mEnableBLDC = false;
	}

	void callback(bool& enable, bool setter) {
		if (setter) {
			if (enable) {
				driver->claim(this);
				callback(mTarget_frequency_mHz, true);
				pwmdriver::CommutationPattern *commPattern = driver->getPattern();

				// setup the pwm configs
				PWMLookupTableGeneratorShaped lookupGenerator;
				lookupGenerator.generateLookupTable(3, StepsCount, mShapeFactor, commPattern + StepsCount * 0);
				lookupGenerator.generateLookupTable(3, StepsCount, mShapeFactor, commPattern + StepsCount * 1);
				lookupGenerator.generateLookupTableReversed(3, StepsCount, mShapeFactor, commPattern + StepsCount * 2);
				lookupGenerator.generateLookupTableReversed(3, StepsCount, mShapeFactor, commPattern + StepsCount * 3);

				// the last element contains information about when to sample the hall sensors
				for (uint32_t i = 0; i < pwmdriver::StepsCount; ++i) {
					commPattern[i*pwmdriver::TicksPerStep][pwmdriver::TicksPerStep-1] = 0;
				}

				mControllerParams->errorP = 0;
				mControllerParams->errorI = 0;
			} else {
				driver->claim(nullptr);
			}
		}
	}

	void callback(float&, bool setter) {
		callback(mEnableBLDC, setter);
	}

	void callback(int& target_frequency_mHz, bool setter) {
		if (setter) {
			if (sgn(mTargetTickFrequency) != sgn(target_frequency_mHz)) {
				mControllerParams->errorP = 0;
				mControllerParams->errorI = 0;
			}
			mTargetTickFrequency = float(target_frequency_mHz * 6.f / 1e3f);
			mTargetStepFrequency = std::abs(target_frequency_mHz * int(StepsCount));
		}
	}

	void callback(flawless::Message<hall::Feedback> const& hallFeedback) {
		flawless::LockGuard lock;

		mLastKnownPhase = mHallIndexes.get()[hallFeedback->currentHallValues];
		if (mEnableBLDC) {
			const float avgTickFreq = hallFeedback->tickFreq_Hz;
			systemTime_t now = time.getSystemTimeUS();
			float dT = (now - mLastUpdateTime) / 1e6f;
			float e  = (mTargetTickFrequency - avgTickFreq);

			mController.setError(e, dT);
			float controll = mController.getControll();
			controll = std::min(1.f, std::max(-1.f, controll));
			mLastUpdateTime = now;

			float powerOutput = std::abs(controll);
			currentController.setPower(powerOutput);
			int targetIndex = mLastKnownPhase;
			if (controll > 0.f) {
				targetIndex = mLastKnownPhase + mMaxAdvance;
			} else {
				targetIndex = mLastKnownPhase - mMaxAdvance;
			}
			targetIndex = myModulo(targetIndex, StepsCount);

			uint32_t stepFrequency = uint32_t(std::abs(avgTickFreq)) * 1000 * StepsCount;
			if (avgTickFreq > 0.f) { // run clockwise
				driver->runSteps(targetIndex, StepsCount / 6, stepFrequency, false);
			} else { // run counter clockwise
				driver->runSteps(3 * StepsCount - targetIndex, StepsCount / 6, stepFrequency,false);
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
		uint8_t step0 = 1; // at phase 0 we read a hall feedback of 1
		for (int i=0; i < 6; ++i) {
			int idx = (StepsCount/12) + (StepsCount/6) * i;
			mHallIndexes.get()[step0]  = myModulo(idx, StepsCount);
			step0 = hall::getNextStep(step0, true);
		}
	}
} driver(10);

}
