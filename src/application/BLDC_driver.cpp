#include "CurrentSense.h"
#include "PWMDriver.h"
#include "HallFeedback.h"

#include "controller/PIDController.h"

#include <flawless/core/Message.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/module/Module.h>

#include <interfaces/systemTime.h>

#include "SiLi/SiLi.h"

#include <cmath>


namespace {
template <typename T> constexpr int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

constexpr float pi = 3.14159265359f;

struct BLDC_driver final :
		public flawless::Module,
		public pwmdriver::DriverInterface,
		public flawless::Listener<hall::Tick>,
		public flawless::Listener<hall::Timeout>,
		public flawless::Listener<CurrentMeasurement>
{
	static constexpr int StepsCount = pwmdriver::StepsCount;
	static constexpr int StepsPerPhase = StepsCount / 6;

	bool mEnabled {false};
	PIDController mController {
	    	0.f
			,0.f,
			0.f,
			1.e-2f, // default P
			2.e-3f,  // default I
			2.6e-4f, // default D
			0.f, 0.f, 0.f
	};


	float mOutput2Advance                {1e-2f};
	int   mMaxAdvance                    {190};
	float mMinControlOutput              {0.f};
	float mTargetTickFrequency           {0.f};
	uint32_t mTargetStepFrequency        {0};

	float mAdvanceScale {1.f};
	float mTargetPhase {0.f};
	float mControl {0.f};
	float mMinControl {.05f};

	pwmdriver::Driver* driver {nullptr};
	BLDC_driver(unsigned int level) : flawless::Module(level) {}

	SystemTime& time = SystemTime::get();
	systemTime_t mLastUpdateTime;

	void unclaim() override {
		mEnabled = false;
	}

	void buildLookupTable() {
		// setup the pwm configs
		pwmdriver::CommutationPattern &commPattern = driver->getPattern();

		for (auto &element : commPattern) {
			for (auto & sElement : element) {
				sElement = 0;
			}
		}

		// build the U-signal
		for (int i = 0; i < 2*StepsPerPhase; ++i) {
			const float s = std::sin(float(i) * (2 * pi / float(StepsCount)));
			commPattern[i][0] = uint16_t(std::round(s * pwmdriver::PwmAmplitude) + pwmdriver::PwmPreOffTimer);
		}
		// mirror that
		for (int i = 0; i < 2*StepsPerPhase; ++i) {
			commPattern[4*StepsPerPhase-i-1][0] = commPattern[i][0];
		}
		// build the V-Signal
		for (int i = 0; i < 4*StepsPerPhase; ++i) {
			commPattern[2*StepsPerPhase+i][1] = commPattern[i][0];
		}
		// build the W-Signal
		for (int i = 0; i < 4*StepsPerPhase; ++i) {
			commPattern[(4*StepsPerPhase + i) % StepsCount][2] = commPattern[i][0];
		}

		// copy everything from the first whole commutation pattern buffer to the second
		for (int i = 0; i < StepsCount; ++i) {
			for (int j = 0; j < 3; ++j) {
				commPattern[StepsCount+i][j] = commPattern[i][j];
			}
		}

		// mirror the left half of the buffer to the right half to get the CCW-Pattern
		for (int i = 0; i < 2*StepsCount; ++i) {
			for (int j = 0; j < 3; ++j) {
				commPattern[pwmdriver::CommutationPatternBufferSize-i-1][j] = commPattern[i][j];
			}
		}

		// the last element contains information about when to sample the hall sensors
		for (uint32_t i = 0; i < pwmdriver::CommutationPatternBufferSize; ++i) {
			commPattern[i][pwmdriver::TicksPerStep-1] = 0;
		}
	}

	void enable(bool enable) {
		mEnabled = enable;
		if (enable) {
			driver->claim(this);
			mController.errorP = 0;
			mController.errorI = 0;
			TIM_BDTR(PWM_TIMER) |= TIM_BDTR_MOE;
		} else {
			TIM_BDTR(PWM_TIMER) &= ~TIM_BDTR_MOE;
			driver->claim(nullptr);
		}
	}

	void setTargetFrequency(float target_frequency) {
		if (sgn(mTargetTickFrequency) != sgn(target_frequency)) {
			mController.errorI = 0;
		}
		mTargetTickFrequency = float(target_frequency * 6.f);
		mTargetStepFrequency = std::abs(int32_t(target_frequency * StepsCount));
	}


	void callback(flawless::Message<CurrentMeasurement> const& current) {

	}

	void callback(flawless::Message<hall::Timeout> const& state) {

	}

	void callback(flawless::Message<hall::Tick> const& state) {
		if (mEnabled) {
//			float const& speed = state->estSpeed;
//			float const& pos   = state->lastKnownPos;
//
//			systemTime_t now = time.getSystemTimeUS();
//			float dT = (now - mLastUpdateTime) * 1e-6f;
//			dT = std::max(1e-6, dT);
//			float e  = mTargetTickFrequency - speed;
//			mLastUpdateTime = now;
//
//			mControl = mController.update(e, dT);
//
////			mAdvanceScale = 1.f - 2.f / (1.f + std::exp(mOutput2Advance * (mTargetStepFrequency)));
//			mAdvanceScale = sgn(mControl);
//			int targetIndex = int(pos * (StepsPerPhase));
//			targetIndex += int(mMaxAdvance * mAdvanceScale);
//			targetIndex = myModulo(targetIndex, StepsCount);
//			mTargetPhase = float(targetIndex) / (StepsPerPhase);
////			uint32_t stepFrequency = uint32_t(std::abs(state->estSpeed) * 1000 * StepsCount);
//			if (speed >= 0.f) { // run clockwise
//				driver->runSteps(targetIndex, 1, mTargetStepFrequency, false);
//			} else { // run counter clockwise
//				driver->runSteps(3 * StepsCount - targetIndex, 1, mTargetStepFrequency,false);
//			}
		}
	}

	void init(unsigned int) {
		driver = &(flawless::util::Singleton<pwmdriver::Driver>::get());
		buildLookupTable();
		enable(mEnabled);
	}


	int myModulo(int a, int b) {
		return (b + a % b) % b;
	}
} driver(10);

// configuration Helpers

struct : public flawless::Callback<bool&, bool> {
flawless::ApplicationConfigMapping<bool> mEnableBLDC              {"bldc.enable", "B", this, driver.mEnabled};
	void callback(bool& enable, bool setter) {
		if (setter) {
			driver.enable(enable);
		}
	}
} enableHelper;

struct : public flawless::Callback<float&, bool> {
	flawless::ApplicationConfig<float> mTarget_frequency          {"bldc.target_frequency", "f", this, 0};
	void callback(float& target, bool setter) {
		if (setter) {
			driver.setTargetFrequency(target);
		}
	}
} targetFrequencyHelper;

flawless::ApplicationConfigMapping<float> cfgFreq2AdvanceB2                 {"bldc.outp_2_advance",       "f", driver.mOutput2Advance};
flawless::ApplicationConfigMapping<int>   cfgMaxAdvance                     {"bldc.advance",              "i", driver.mMaxAdvance};
flawless::ApplicationConfigMapping<float> cfgMaxAdvanceScale                {"bldc.advance_scale",        "f", driver.mAdvanceScale};
flawless::ApplicationConfigMapping<float> cfgControl                        {"bldc.control",              "f", driver.mControl};
flawless::ApplicationConfigMapping<float> cfgTargetPhase                    {"bldc.target_phase",         "f", driver.mTargetPhase};
flawless::ApplicationConfigMapping<float> cfgMinControlOutput               {"bldc.min_controll_output",  "f", driver.mMinControlOutput};

flawless::ApplicationConfigMapping<float> cfgControllerEP                   {"bldc.controller.ep",         "f", driver.mController.errorP};
flawless::ApplicationConfigMapping<float> cfgControllerEI                   {"bldc.controller.ei",         "f", driver.mController.errorI};
flawless::ApplicationConfigMapping<float> cfgControllerED                   {"bldc.controller.ed",         "f", driver.mController.errorD};

flawless::ApplicationConfigMapping<float> cfgControllerOP                   {"bldc.controller.op",         "f", driver.mController.outputP};
flawless::ApplicationConfigMapping<float> cfgControllerOI                   {"bldc.controller.oi",         "f", driver.mController.outputI};
flawless::ApplicationConfigMapping<float> cfgControllerOD                   {"bldc.controller.od",         "f", driver.mController.outputD};

flawless::ApplicationConfigMapping<float> cfgControllerP                    {"bldc.controller.p",          "f", driver.mController.controllP};
flawless::ApplicationConfigMapping<float> cfgControllerI                    {"bldc.controller.i",          "f", driver.mController.controllI};
flawless::ApplicationConfigMapping<float> cfgControllerD                    {"bldc.controller.d",          "f", driver.mController.controllD};
}
