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
	PIDController mController
	{
	    	0.f,
			0.f,
			0.f,
			5e-5f,  // default P
			1.e-2f, // default I
			0.f,    // default D
			0.f, 0.f, 0.f
	};
//	{
//	    	0.f,
//			0.f,
//			0.f,
//			1.e-2f, // default P
//			2.e-3f,  // default I
//			2.6e-4f, // default D
//			0.f, 0.f, 0.f
//	};


	float    mOutput2Advance                {1e-2f};
	int      mMaxAdvance                    {190};
	float    mMinControlOutput              {0.f};
	float    mTargetTickFrequency           {0};
	bool     mTargetDirCW                   {true};

	float mAdvanceScale {1.f};
	float mTargetPhase {0.f};
	float mControl {0.f};
	float mTickSpeed {0.f};
	float mSpeed {0.f};
	float mSpeedInnovation {.7f};
	float mMinControl {.05f};
	float mCurrentError {0.f};

	float mMaxCurrent                    {.88f}; // maximum current the motor can handle in A
	float mShutoffMaxCurrent             {1.5f}; // maximum current the motor can handle in A

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
				sElement = pwmdriver::PwmPreOffTimer;
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
			commPattern[2*StepsPerPhase+i][2] = commPattern[i][0];
		}
		// build the W-Signal
		for (int i = 0; i < 4*StepsPerPhase; ++i) {
			commPattern[(4*StepsPerPhase + i) % StepsCount][1] = commPattern[i][0];
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
			int sampleMoment = pwmdriver::PwmCentralDutyMoment;
			for (int j = 0; j < 3; ++j) {
				if (std::abs(sampleMoment - int(commPattern[i][j])) < 10) {
					sampleMoment = commPattern[i][j] - 10;
				}
			}
			commPattern[i][pwmdriver::TicksPerStep-1] = sampleMoment;
		}
	}

	void enable(bool enable) {
		mEnabled = enable;
		if (enable) {
			driver->claim(this);
			mController.errorP = 0;
			mController.errorI = 0;
			TIM_BDTR(PWM_TIMER) |= TIM_BDTR_MOE;
			mLastUpdateTime = time.getSystemTimeUS();
		} else {
			TIM_BDTR(PWM_TIMER) &= ~TIM_BDTR_MOE;
			driver->claim(nullptr);
		}
	}

	void setTargetFrequency(float target_frequency) {
		mTargetDirCW = target_frequency >= 0;
		if (sgn(mTargetTickFrequency) != sgn(target_frequency)) {
			mController.errorI = 0;
		}
		mTargetTickFrequency = target_frequency * 6;
	}


	void callback(flawless::Message<CurrentMeasurement> const& current) {
		mCurrentError = std::abs(current->current) - mMaxCurrent;
		mCurrentError = std::max(0.f, mCurrentError);
		if (current->current > mShutoffMaxCurrent) {
			TIM_BDTR(PWM_TIMER) &= ~TIM_BDTR_MOE;
		} else {
			if (mEnabled) {
				setDutcyCycle(std::max(0.f, std::abs(mControl) - mCurrentError));
				TIM_BDTR(PWM_TIMER) |= TIM_BDTR_MOE;
			}
		}
	}

	void callback(flawless::Message<hall::Timeout> const& state) {
		if (mEnabled) {
			systemTime_t now = time.getSystemTimeUS();
			float dT = (now - mLastUpdateTime) * 1.e-6f;
			dT = std::max(1e-6, dT);
			mTickSpeed = (1-mSpeedInnovation) * mTickSpeed + mSpeedInnovation * ((state->movingCW)?1.e6f:-1.e6f) / (state->delaySinceLastTickUS);
			mSpeed = mTickSpeed / 6.f;
			float e  = mTargetTickFrequency - mTickSpeed;
			mLastUpdateTime = now;
			mControl = mController.update(e, dT);

			setDutcyCycle(std::max(0.f, std::abs(mControl) - mCurrentError));

			int stepStart = state->currentPos * StepsPerPhase / 2 + mMaxAdvance * sgn(mControl);
			stepStart = (StepsCount + stepStart) % StepsCount;
			if (mControl < 0) {
				stepStart = 3 * StepsCount - stepStart;
			}
			driver->runSteps(stepStart, StepsPerPhase, 0, false);
		}
	}

	void callback(flawless::Message<hall::Tick> const& state) {
		if (mEnabled) {
			// run the controller
			systemTime_t now = time.getSystemTimeUS();
			float dT = (now - mLastUpdateTime) * 1.e-6f;
			dT = std::max(1e-6f, dT);
			mTickSpeed = (1-mSpeedInnovation) * mTickSpeed + mSpeedInnovation * ((state->movingCW)?1.e6f:-1.e6f) / (state->prevTickDelayUS);
			mSpeed = mTickSpeed / 6.f;
			float e  = mTargetTickFrequency - mTickSpeed;
			mLastUpdateTime = now;
			mControl = mController.update(e, dT);

			setDutcyCycle(std::max(0.f, std::abs(mControl) - mCurrentError));

			int stepStart = state->currentPos * StepsPerPhase / 2 + mMaxAdvance * sgn(mControl);
			stepStart = (StepsCount + stepStart) % StepsCount;
			if (not state->movingCW) {
				stepStart = 3 * StepsCount - stepStart;
			}
			driver->runSteps(stepStart, StepsPerPhase, state->prevTickDelayUS / StepsPerPhase, false);
		}
	}

	void setDutcyCycle(float scale) {
		flawless::LockGuard lock;
		uint32_t arr = int(pwmdriver::PwmPreOffTimer + pwmdriver::PwmAmplitude / scale);
		arr = std::max(arr, pwmdriver::PwmMinCyclePeriod);
		arr = std::min(arr, uint32_t(pwmdriver::pwm_timer_base_freq / pwmdriver::pwm_target_freq));
		uint32_t psc = int(pwmdriver::pwm_timer_base_freq / (pwmdriver::pwm_target_freq * (arr)));
		if (psc > 0) {
			psc -= 1;
		}
		TIM_ARR(PWM_TIMER) = arr;
		TIM_PSC(PWM_TIMER) = psc;
	}

	void init(unsigned int) {
		driver = &(flawless::util::Singleton<pwmdriver::Driver>::get());
		buildLookupTable();
		enable(mEnabled);
		setTargetFrequency(0.f);
		setDutcyCycle(2.5e-1f);
		TIM_BDTR(PWM_TIMER) |= TIM_BDTR_MOE;
	}

	int myModulo(int a, int b) {
		return (b + a % b) % b;
	}
} driver(10);

// configuration Helpers

struct : public flawless::Callback<bool&, bool> {
flawless::ApplicationConfigMapping<bool> mEnableBLDC              {"motor.enable", "B", this, driver.mEnabled};
	void callback(bool& enable, bool setter) {
		if (setter) {
			driver.enable(enable);
		}
	}
} enableHelper;

struct : public flawless::Callback<float&, bool> {
	flawless::ApplicationConfig<float> mTarget_frequency          {"motor.target_frequency", "f", this, 0};
	void callback(float& target, bool setter) {
		if (setter) {
			driver.setTargetFrequency(target);
		}
	}
} targetFrequencyHelper;

flawless::ApplicationConfigMapping<float> cfgMaxMotorCurrent                {"motor.max_motor_current",    "f", driver.mMaxCurrent};

//flawless::ApplicationConfigMapping<float> cfgFreq2AdvanceB2                 {"motor.outp_2_advance",       "f", driver.mOutput2Advance};
flawless::ApplicationConfigMapping<int>   cfgMaxAdvance                     {"motor.advance",              "i", driver.mMaxAdvance};
flawless::ApplicationConfigMapping<float> cfgMaxAdvanceScale                {"motor.advance_scale",        "f", driver.mAdvanceScale};
flawless::ApplicationConfigMapping<float> cfgControl                        {"motor.control",              "f", driver.mControl};
flawless::ApplicationConfigMapping<float> cfgSpeed                          {"motor.speed",                "f", driver.mSpeed};
flawless::ApplicationConfigMapping<float> cfgSpeedInnovation                {"motor.speed_innovation",     "f", driver.mSpeedInnovation};
flawless::ApplicationConfigMapping<float> cfgTargetPhase                    {"motor.target_phase",         "f", driver.mTargetPhase};
flawless::ApplicationConfigMapping<float> cfgMinControlOutput               {"motor.min_controll_output",  "f", driver.mMinControlOutput};

flawless::ApplicationConfigMapping<float> cfgControllerEP                   {"motor.controller.ep",         "f", driver.mController.errorP};
flawless::ApplicationConfigMapping<float> cfgControllerEI                   {"motor.controller.ei",         "f", driver.mController.errorI};
flawless::ApplicationConfigMapping<float> cfgControllerED                   {"motor.controller.ed",         "f", driver.mController.errorD};

flawless::ApplicationConfigMapping<float> cfgControllerOP                   {"motor.controller.op",         "f", driver.mController.outputP};
flawless::ApplicationConfigMapping<float> cfgControllerOI                   {"motor.controller.oi",         "f", driver.mController.outputI};
flawless::ApplicationConfigMapping<float> cfgControllerOD                   {"motor.controller.od",         "f", driver.mController.outputD};

flawless::ApplicationConfigMapping<float> cfgControllerP                    {"motor.controller.p",          "f", driver.mController.controllP};
flawless::ApplicationConfigMapping<float> cfgControllerI                    {"motor.controller.i",          "f", driver.mController.controllI};
flawless::ApplicationConfigMapping<float> cfgControllerD                    {"motor.controller.d",          "f", driver.mController.controllD};
}
