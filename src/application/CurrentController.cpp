
#include "CurrentController.h"
#include <libopencm3/stm32/f4/timer.h>

#include <algorithm>

#include <flawless/module/Module.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/core/Listener.h>
#include <flawless/util/Array.h>
#include <flawless/applicationConfig/ApplicationConfig.h>

#include <application/MotorCurrent.h>
#include <application/PWMDriver.h>


struct : public flawless::Listener<VoltageMeasure, 0>
{
	flawless::ApplicationConfig<Voltage> mMotorCurrentMean{"current_ctl.current", "f"};
	flawless::ApplicationConfig<Voltage> mMaxCurrent{"current_ctl.max_current", "f", 0.5f};
	flawless::ApplicationConfig<Voltage> mCurrentP{"current_ctl.controll_p", "f", .01f};
	flawless::ApplicationConfig<Voltage> mCurrentError{"current_ctl.error", "f"};
	flawless::ApplicationConfig<Voltage> currentOutputScale{"current_ctl.output_scale", "f", 1.f};

	flawless::ApplicationConfig<Voltage> v_gain{"current_ctl.v_gain", "f", 10.f};
	flawless::ApplicationConfig<Voltage> v_ref{"current_ctl.v_ref", "f", 3.3f / 2.f};

	flawless::ApplicationConfig<int> max_arr{"current_ctl.max_arr", "I", 0xffff/12};

	void callback(flawless::Message<VoltageMeasure> const& motorCurrent) override {
		mMotorCurrentMean = (v_ref - motorCurrent->motorCurrent) * v_gain;
		const float maxCurrent = mMaxCurrent*currentOutputScale;
		mCurrentError = mMotorCurrentMean - maxCurrent;
		int targetAmplitude = int(float(TIM_ARR(PWM_TIMER)) * (1.f + (mCurrentError / maxCurrent) * mCurrentP));
		targetAmplitude = std::max(int(pwmdriver::PwmMinCyclePeriod), std::min(max_arr.get(), targetAmplitude));
		TIM_ARR(PWM_TIMER) = targetAmplitude;
	}
} currentController;

void CurrentController::setPower(float power)
{
	currentController.currentOutputScale = std::min(1.f, std::max(0.f, power));
}

float CurrentController::getPower() const
{
	return currentController.currentOutputScale;
}
