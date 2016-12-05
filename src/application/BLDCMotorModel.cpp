#include "MotorModel.h"

#include "PWMDriver.h"
#include "HallFeedback.h"
#include "CurrentController.h"

#include <flawless/core/Message.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/module/Module.h>

#include "SiLi/SiLi.h"

#include <libopencm3/stm32/f4/timer.h>


#include <cmath>
#include <algorithm>

namespace
{

flawless::MessageBufferMemory<MotorState, 5> motorStateMessageBufferMemory;

template <typename T> constexpr int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

struct BLDCMotorModel :
		public flawless::Module,
		public flawless::Listener<hall::Tick, 0>,
		public flawless::Listener<hall::Timeout, 0>,
		public flawless::Listener<CurrentMeasurement, 0>
{
	static constexpr int TotalTickCnt = pwmdriver::TotalTickCnt;
	static constexpr int StepsCount = pwmdriver::StepsCount;

	Array<float, 8> mHallIndexes       {};

	float mMotorTorqueConstant           {16.e-3f};  // Nm / A
	float mMotorInertia                  {2.12e-7f}; // (kg * m^2)
	float mMotorDrag                     {100.f};

	float mLastKnownPhase                {0};
	float mNextPhaseUpperLimit           {0};
	float mNextPhaseLowerLimit           {0};

	float mLastCurrentMeasurement        {0.f};

	SiLi::Matrix<2, 1> mState            {{0.f, 0.f}};                // x vector
	SiLi::Matrix<2, 2> mCertainty        {{1.f, 0.f, 0.f, 1.f}};      // P matrix

	SiLi::Matrix<2, 2> mPredictQ         {{1.e1f, 0.f, 0.f, 5.e2f}};  // Q matrix
	SiLi::Matrix<2, 2> mCorrectR         {{0.f, 0.f, 0.f, 5.e0f}};    // R matrix
	static constexpr float mCorrectRegularisation         {1e-6f};

	static constexpr float mCovClipping {1.e4f};

	pwmdriver::Driver* driver {nullptr};

	void predict(float dT) {
		float speedDiff = (mState(1, 0) * mMotorDrag) * dT;
		if (mState(1, 0) >= 0) {
			speedDiff = -std::min(mState(1, 0), speedDiff);
		} else {
			speedDiff = -std::max(mState(1, 0), speedDiff);
		}

		const SiLi::Matrix<2, 2> F({
			1.f, dT,
			0.f, 1.f
		});
		const SiLi::Matrix<2, 1> BComSpeed({
			.5f * dT,
			1.f
		});
		mState = F * mState + BComSpeed * speedDiff;

		mState(0, 0) = myModulo(mState(0, 0), 6.f);

		mCertainty = F * mCertainty * F.t() + mPredictQ * dT;
		for (int row(0); row < 2; ++row) {
			for (int col(0); col < 2; ++col) {
				if (mCertainty(row, col) > mCovClipping) {
					mCertainty(row, col) = mCovClipping;
				}
			}
		}
	}

	void correct(float measuredPosition, float measuredSpeed) {
		// the H matrix is I here

		// measurement
		SiLi::Matrix<2, 1> z ({measuredPosition, measuredSpeed});
		// innovation
		SiLi::Matrix<2, 1> y = z - mState;
//		z(0, 0) = myModulo(z(0,0), 6.f);
		// residual covariance
		SiLi::Matrix<2, 2> S = mCertainty + mCorrectR;
		// kalman gain
		SiLi::Matrix<2, 2> K = mCertainty * (S + SiLi::Matrix<2, 2>({mCorrectRegularisation, 0.f, 0.f, mCorrectRegularisation})).inv();

		mState = mState + K * y;
		mState(0, 0) = measuredPosition;
//		mState = z; // it is always this value
		mCertainty = mCertainty - K * S * K.t();
		for (int row(0); row < 2; ++row) {
			for (int col(0); col < 2; ++col) {
				if (mCertainty(row, col) > mCovClipping) {
					mCertainty(row, col) = mCovClipping;
				}
			}
		}
	}

	void callback(flawless::Message<CurrentMeasurement> const& currentMeasurement) {
		mLastCurrentMeasurement = currentMeasurement->current;
	}

	void publishState() {
		auto msg = flawless::getFreeMessage<MotorState>();
		if (msg) {
			msg->lastKnownPos = mLastKnownPhase;
			msg->lastKnownSpeed = mLastFreq;
			msg->estPhase = mState(0, 0);
			msg->estSpeed = mState(1, 0);
			msg.post();
		}
	}

	float mLastPredictDelay {0.f};
	void callback(flawless::Message<hall::Timeout> const& hallTimeout) {
		predict(hallTimeout->delaySinceLastTick - mLastPredictDelay);
		mLastPredictDelay = hallTimeout->delaySinceLastTick;
		publishState();
	}

	float mLastFreq = 0;
	void callback(flawless::Message<hall::Tick> const& hallTick) {
		mNextPhaseUpperLimit = mNextPhaseLowerLimit = mLastKnownPhase = mHallIndexes[hallTick->currentHallValues];

		int direction = 1;
		if (sgn(hallTick->tickFreq_Hz) != sgn(mLastFreq)) {
			direction = 0;
		} else {
			if (hallTick->tickFreq_Hz >= 0) {
				mNextPhaseUpperLimit = mLastKnownPhase + 1.f;
			} else {
				mNextPhaseLowerLimit = mLastKnownPhase - 1.f;
			}
		}

		mLastFreq = hallTick->tickFreq_Hz;

		mLastKnownPhase = myModulo(mLastKnownPhase, 6.f);
		predict(hallTick->prevTickDelay - mLastPredictDelay);
		mLastPredictDelay = 0;
		correct(mLastKnownPhase, hallTick->tickFreq_Hz * direction);
		publishState();
	}

	int myModulo(int a, int b) {
		return (b + a % b) % b;
	}
	float myModulo(float a, float b) {
		if (a < 0) {
			while (a < 0) {
				a += b;
			}
			return a;
		} else {
			while (a > 0) {
				a -= b;
			}
			return a;
		}
	}

	void init(unsigned int) {
		driver = &(flawless::util::Singleton<pwmdriver::Driver>::get());
		// setup the lookup tables
		int step = 4; // at phase 0 we read a hall feedback of 4
		for (int i=0; i < 6; ++i) {
			float idx = i;
			mHallIndexes[step]  = myModulo(idx, 6.f);
			step = hall::getNextStep(step, true);
		}
	}

	BLDCMotorModel(unsigned int level) : flawless::Module(level) {}
} motorModel(9);


flawless::ApplicationConfigMapping<float> cfgMotorDrag                      {"motor.drag",                 "f", motorModel.mMotorDrag};
flawless::ApplicationConfigMapping<float> cfgMotorInertia                   {"motor.inertia",              "f", motorModel.mMotorInertia};
flawless::ApplicationConfigMapping<float> cfgMotorTorqueConstant            {"motor.torqueconstant",       "f", motorModel.mMotorTorqueConstant};

flawless::ApplicationConfigMapping<Array<float, 8>> cfgHallIndexes          {"motor.hall_mappings",       "8f", motorModel.mHallIndexes};

flawless::ApplicationConfigMapping<float> cfgLastKnownPhase                 {"motor.known_phase",          "f", motorModel.mLastKnownPhase};
flawless::ApplicationConfigMapping<float> cfgEstimatedPhase                 {"motor.est_phase",            "f", motorModel.mState(0, 0)};
flawless::ApplicationConfigMapping<float> cfgEstimatedFreq                  {"motor.est_frequency",        "f", motorModel.mState(1, 0)};

}
