
#include <application/CurrentSense.h>
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

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>

#include <flawless/util/Singleton.h>

namespace {
flawless::MessageBufferMemory<CurrentMeasurement, 5> currentMeasurement;

#define DRV_DC_CAL_PIN  GPIO10
#define DRV_DC_CAL_PORT GPIOB

static constexpr float regularisationEpsilon = 1e-5;

struct : public flawless::Listener<VoltageMeasure, 0>
{
	flawless::ApplicationConfig<Voltage> mMotorCurrentMean{"motor.current", "f"};

	flawless::ApplicationConfig<Voltage> v_gain{"motor.current.v_gain", "f", 10.f};
	flawless::ApplicationConfig<Voltage> v_ref{"motor.current.v_ref", "f", 3.3f / 2.f};

	int measurementCouter {0};
	bool calibrating      {false};
	bool wasCalibrating   {false};
	void callback(flawless::Message<VoltageMeasure> const& motorCurrent) override {
		mMotorCurrentMean = (v_ref - motorCurrent->motorCurrent) * v_gain;
		if (motorCurrent->motorCurrent < .5f) { // in this case the driver is not powered
			mMotorCurrentMean = 0.f;
		} else {
			if (not calibrating) {
				if (measurementCouter > 1000) {
					gpio_set(DRV_DC_CAL_PORT, DRV_DC_CAL_PIN);
					calibrating = true;
					measurementCouter = 0;
					return;
				} else {
					++measurementCouter;
				}
			} else {
				calibrating = false;
				wasCalibrating = true;
				v_ref = motorCurrent->motorCurrent;
				gpio_clear(DRV_DC_CAL_PORT, DRV_DC_CAL_PIN);
				return;
			}
		}
		if (wasCalibrating) {
			wasCalibrating = false;
			return; // thow this measurement away
		}

		auto msg = flawless::getFreeMessage<CurrentMeasurement>();
		if (msg) {
			msg->current = mMotorCurrentMean;
			msg.post();
		}
	}
} currentController;


struct InitHelper : public flawless::Module<50> {
	void init() override {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
		gpio_mode_setup(DRV_DC_CAL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DRV_DC_CAL_PIN);
		gpio_clear(DRV_DC_CAL_PORT, DRV_DC_CAL_PIN);
	}
} initHelper;
}



