#include "CurrentSense.h"
#include "PWMDriver.h"
#include "HallFeedback.h"

#include "controller/PIDController.h"
#include "controller/DecayController.h"

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
		public flawless::Module<10>,
		public pwmdriver::DriverInterface,
		public flawless::Listener<hall::Tick>,
		public flawless::Listener<hall::Timeout>,
		public flawless::Listener<CurrentMeasurement>
{
	static constexpr int StepsCount = pwmdriver::StepsCount;
	static constexpr int StepsPerPhase = StepsCount / 6;

	bool mEnabled {false};
	PIDController mSpeedController
	{
	    	0.f,
			0.f,
			0.f,
			1e-6f,  // default P
			5.e-3f, // default I
			0.f,    // default D
			0.f, 0.f, 0.f
	};

	DecayController mCurrentController
	{
	    	0.f,
			0.f,
			0.f,  // default P
			1.e-3f, // default I
			1.e-2f, // decay
			0.f, 0.f
	};


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

	float mPWMFrequency                  {pwmdriver::pwm_target_freq};

	pwmdriver::Driver* driver {nullptr};

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
			mSpeedController.errorP = 0;
			mSpeedController.errorI = 0;
			TIM_BDTR(PWM_TIMER) |= TIM_BDTR_MOE;
			mLastUpdateTime = time.getSystemTimeUS();
		} else {
			TIM_BDTR(PWM_TIMER) &= ~TIM_BDTR_MOE;
			driver->claim(nullptr);
			mTargetTickFrequency = 0.f;
		}
	}

	void setTargetFrequency(float target_frequency) {
		mTargetDirCW = target_frequency >= 0;
		if (sgn(mTargetTickFrequency) != sgn(target_frequency)) {
			mSpeedController.errorI = 0;
		}
		mTargetTickFrequency = target_frequency * 6;
	}


	systemTime_t mLastCurrentMeasurement;
	void callback(flawless::Message<CurrentMeasurement> const& current) {
		systemTime_t now = time.getSystemTimeUS();
		mCurrentError = mCurrentController.update(std::max(std::abs(current->current) - mMaxCurrent, 0.f), now - mLastCurrentMeasurement);
		mLastCurrentMeasurement = now;
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
		systemTime_t now = time.getSystemTimeUS();
		float dT = (now - mLastUpdateTime) * 1.e-6f;
		dT = std::max(1e-6, dT);
		mTickSpeed = (1-mSpeedInnovation) * mTickSpeed + mSpeedInnovation * ((state->movingCW)?1.e6f:-1.e6f) / (state->delaySinceLastTickUS);
		if (not state->moving) {
			mTickSpeed = 0;
		}
		mSpeed = mTickSpeed / 6.f;
		mLastUpdateTime = now;
		if (mEnabled) {
			float e  = mTargetTickFrequency - mTickSpeed;
			mControl = mSpeedController.update(e, dT);

			setDutcyCycle(std::max(0.f, std::abs(mControl) - mCurrentError));

			int stepStart = state->currentPos * StepsPerPhase / 2 + mMaxAdvance * sgn(mControl);
			stepStart = (StepsCount + stepStart) % StepsCount;
			if (mControl < 0) {
				stepStart = 3 * StepsCount - stepStart;
			}
			driver->runSteps(stepStart, 1, 0, false);
		}
	}

	void callback(flawless::Message<hall::Tick> const& state) {
		// run the controller
		systemTime_t now = time.getSystemTimeUS();
		float dT = (now - mLastUpdateTime) * 1.e-6f;
		dT = std::max(1e-6f, dT);
		int delay = state->prevTickDelayUS;
		mTickSpeed = (1-mSpeedInnovation) * mTickSpeed + mSpeedInnovation * ((state->movingCW)?1.e6f:-1.e6f) / delay;
		if (not state->moving) {
			delay = 0;
			mTickSpeed = 0;
		}

		mSpeed = mTickSpeed / 6.f;
		mLastUpdateTime = now;
		if (mEnabled) {
			float e  = mTargetTickFrequency - mTickSpeed;
			mControl = mSpeedController.update(e, dT);

			setDutcyCycle(std::max(0.f, std::abs(mControl) - mCurrentError));

			int stepStart = state->currentPos * StepsPerPhase / 2 + mMaxAdvance * sgn(mControl);
			stepStart = (StepsCount + stepStart) % StepsCount;
			if (not state->movingCW) {
				stepStart = 3 * StepsCount - stepStart;
			}
			driver->runSteps(stepStart, StepsPerPhase, delay / StepsPerPhase, false);
		}
	}

	void setDutcyCycle(float scale) {
		flawless::LockGuard lock;
		uint32_t arr = int(pwmdriver::PwmPreOffTimer + pwmdriver::PwmAmplitude / scale);
		if (arr > pwmdriver::PwmMinCyclePeriod) {
			TIM_BDTR(PWM_TIMER) &= TIM_BDTR_MOE; // this is the same as arr = infinity since the pwm is effectively turned off
		} else {
			TIM_BDTR(PWM_TIMER) |= TIM_BDTR_MOE;
		}
		arr = std::min(arr, uint32_t(pwmdriver::pwm_timer_base_freq / mPWMFrequency));
		uint32_t psc = int(pwmdriver::pwm_timer_base_freq / (mPWMFrequency * (arr)));
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
		setDutcyCycle(1.f);
		TIM_BDTR(PWM_TIMER) |= TIM_BDTR_MOE;
	}

	int myModulo(int a, int b) {
		return (b + a % b) % b;
	}
} driver;

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

flawless::ApplicationConfigMapping<float> cfgControllerEP                   {"motor.controller.ep",         "f", driver.mSpeedController.errorP};
flawless::ApplicationConfigMapping<float> cfgControllerEI                   {"motor.controller.ei",         "f", driver.mSpeedController.errorI};
flawless::ApplicationConfigMapping<float> cfgControllerED                   {"motor.controller.ed",         "f", driver.mSpeedController.errorD};

flawless::ApplicationConfigMapping<float> cfgControllerOP                   {"motor.controller.op",         "f", driver.mSpeedController.outputP};
flawless::ApplicationConfigMapping<float> cfgControllerOI                   {"motor.controller.oi",         "f", driver.mSpeedController.outputI};
flawless::ApplicationConfigMapping<float> cfgControllerOD                   {"motor.controller.od",         "f", driver.mSpeedController.outputD};

flawless::ApplicationConfigMapping<float> cfgControllerP                    {"motor.controller.p",          "f", driver.mSpeedController.controllP};
flawless::ApplicationConfigMapping<float> cfgControllerI                    {"motor.controller.i",          "f", driver.mSpeedController.controllI};
flawless::ApplicationConfigMapping<float> cfgControllerD                    {"motor.controller.d",          "f", driver.mSpeedController.controllD};

flawless::ApplicationConfigMapping<float> cfgCurrentControllerEP            {"motor.currentcontroller.ep",  "f", driver.mCurrentController.errorP};
flawless::ApplicationConfigMapping<float> cfgCurrentControllerEI            {"motor.currentcontroller.ei",  "f", driver.mCurrentController.errorI};

flawless::ApplicationConfigMapping<float> cfgCurrentControllerOP            {"motor.currentcontroller.op",  "f", driver.mCurrentController.outputP};
flawless::ApplicationConfigMapping<float> cfgCurrentControllerOI            {"motor.currentcontroller.oi",  "f", driver.mCurrentController.outputI};

flawless::ApplicationConfigMapping<float> cfgCurrentControllerP             {"motor.currentcontroller.p",   "f", driver.mCurrentController.controllP};
flawless::ApplicationConfigMapping<float> cfgCurrentControllerI             {"motor.currentcontroller.i",   "f", driver.mCurrentController.controllI};
flawless::ApplicationConfigMapping<float> cfgCurrentControllerDecay         {"motor.currentcontroller.dec", "f", driver.mCurrentController.decay};

flawless::ApplicationConfigMapping<float> cfgPWMFrequency                   {"pwm.frequency",               "f", driver.mPWMFrequency};

}
