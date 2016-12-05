#include "PWMDriver.h"
#include "PWMLookupTableGenerator.h"
#include "CurrentController.h"

#include "MotorModel.h"

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
		public flawless::Listener<MotorState, 0>
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

	float mShapeFactor {0.f};
	float mAdvanceScale {1.f};
	float mTargetPhase {0.f};
	float mControl {0.f};
	float mMinControl {.05f};

	pwmdriver::Driver* driver {nullptr};
	CurrentController& currentController {CurrentController::get()};
	BLDC_driver(unsigned int level) : flawless::Module(level) {}

	SystemTime& time = SystemTime::get();
	systemTime_t mLastUpdateTime;

	void unclaim() override {
		mEnabled = false;
	}

	void buildLookupTable() {
		if (mEnabled) { // this implies we have claimed the memory to fill our commutation pattern
			pwmdriver::CommutationPattern &commPattern = driver->getPattern();
			// setup the pwm configs
			PWMLookupTableGeneratorShaped lookupGenerator;
			lookupGenerator.generateLookupTable(3, StepsCount, mShapeFactor, &(commPattern[StepsCount * 0]));
			lookupGenerator.generateLookupTable(3, StepsCount, mShapeFactor, &(commPattern[StepsCount * 1]));
			lookupGenerator.generateLookupTableReversed(3, StepsCount, mShapeFactor, &(commPattern[StepsCount * 2]));
			lookupGenerator.generateLookupTableReversed(3, StepsCount, mShapeFactor, &(commPattern[StepsCount * 3]));

			// the last element contains information about when to sample the hall sensors
			for (uint32_t i = 0; i < pwmdriver::CommutationPatternBufferSize; ++i) {
				commPattern[i][pwmdriver::TicksPerStep-1] = 0; //pwmdriver::PwmPreOffTimer / 2;
			}
		}
	}

	void enable(bool enable) {
		mEnabled = enable;
		if (enable) {
			driver->claim(this);
			buildLookupTable();
			mController.errorP = 0;
			mController.errorI = 0;
		} else {
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


	void callback(flawless::Message<MotorState> const& state) {
		if (mEnabled) {
			float const& speed = state->estSpeed;
			float const& pos   = state->lastKnownPos;

			systemTime_t now = time.getSystemTimeUS();
			float dT = (now - mLastUpdateTime) * 1e-6f;
			dT = std::max(1e-6, dT);
			float e  = mTargetTickFrequency - speed;
			mLastUpdateTime = now;

			mControl = mController.update(e, dT);

			currentController.setPower(std::max(mMinControl, std::abs(mControl)));

//			mAdvanceScale = 1.f - 2.f / (1.f + std::exp(mOutput2Advance * (mTargetStepFrequency)));
			mAdvanceScale = sgn(mControl);
			int targetIndex = int(pos * (StepsPerPhase));
			targetIndex += int(mMaxAdvance * mAdvanceScale);
			targetIndex = myModulo(targetIndex, StepsCount);
			mTargetPhase = float(targetIndex) / (StepsPerPhase);
//			uint32_t stepFrequency = uint32_t(std::abs(state->estSpeed) * 1000 * StepsCount);
			if (speed >= 0.f) { // run clockwise
				driver->runSteps(targetIndex, 1, mTargetStepFrequency, false);
			} else { // run counter clockwise
				driver->runSteps(3 * StepsCount - targetIndex, 1, mTargetStepFrequency,false);
			}
		}
	}

	void init(unsigned int) {
		driver = &(flawless::util::Singleton<pwmdriver::Driver>::get());
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
struct : public flawless::Callback<float&, bool> {
	flawless::ApplicationConfigMapping<float> mShapeFactor        {"bldc.shape_factor", "f", this, driver.mShapeFactor};
	void callback(float&, bool setter) {
		if (setter) {
			driver.buildLookupTable();
		}
	}
} shapeFactorHelper;


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
