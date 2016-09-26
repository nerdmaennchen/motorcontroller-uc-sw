#include "PWMDriver.h"

#include <flawless/stdtypes.h>
#include <flawless/module/Module.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/core/Listener.h>
#include <flawless/util/Array.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/timer/swTimer.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/nvic.h>

#include <target/stm32f4/clock.h>

#include <interfaces/ISRTime.h>

#include "MotorCurrent.h"

#include <algorithm>

#define PWM_PORT  GPIOB
#define PWM_PIN_0 GPIO6
#define PWM_PIN_1 GPIO7
#define PWM_PIN_2 GPIO8
#define PWM_PIN_3 GPIO9

#define PWN_ENABLE_PIN1 GPIO4
#define PWN_ENABLE_PIN2 GPIO5

#define PWM_PINS (PWM_PIN_0 | PWM_PIN_1 | PWM_PIN_2 | PWM_PIN_3)
#define PWM_ENABLE_PINS (PWN_ENABLE_PIN1 | PWN_ENABLE_PIN2)

#define PWM_TIMER TIM4

#define CCR_DMA DMA1
#define CCR_DMA_STREAM 2
#define CCR_DMA_CHANNEL 5

#define DMA_TRIGGER_TIMER TIM3


using namespace pwmdriver;

using RemapType = Array<uint8_t, 4>;
constexpr size_t commutationPatternBufferSize = 4 * StepsCount;
using CommmutationPatternContainer = Array<CommutationPattern, commutationPatternBufferSize>;

namespace {

flawless::ApplicationConfig<CommmutationPatternContainer> gCommutationPattern {"commutation_pattern", "9600H"};
uint32_t gLastSetSNDTR;
DriverInterface* gCurrentInterface;

struct : public flawless::Listener<MotorCurrent, 0>
{
	flawless::ApplicationConfig<MotorCurrent> mMotorCurrentMean{"motor_current", "f"};
	flawless::ApplicationConfig<MotorCurrent> mMaxCurrent{"max_motor_current", "f", 0.5f};
	flawless::ApplicationConfig<MotorCurrent> mCurrentP{"motor_current_controll_p", "f", 1.f};
	flawless::ApplicationConfig<MotorCurrent> mCurrentError{"motor_current_error", "f"};
	float currentOutputScale {1.f};
	void callback(flawless::Message<MotorCurrent> const& motorCurrent) override {
		mMotorCurrentMean = motorCurrent;
		const float maxCurrent = mMaxCurrent*currentOutputScale;
		mCurrentError = mMotorCurrentMean - maxCurrent;
		uint32_t targetAmplitude = TIM_ARR(PWM_TIMER) * uint32_t(1.f + (mMotorCurrentMean-maxCurrent) / maxCurrent * mCurrentP);
		targetAmplitude = std::max(PwmAmplitude+PwmOffTimer, std::min(uint32_t(0xffff), targetAmplitude));
		TIM_ARR(PWM_TIMER)   = targetAmplitude;
	}
} currentController;

}


CommutationPattern* Driver::getPattern() {
	return gCommutationPattern->data();
}

uint32_t Driver::getPatternSize() {
	return gCommutationPattern->size();
}

// make the PWM output the values from stepStart to stepStart+stepCnt automatically with
void Driver::runSteps(uint32_t stepStart, uint32_t stepCnt, uint32_t mHZ, bool cycle) {
	do {
		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_EN;
	} while (0 != (DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) & DMA_CR_EN));

	TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
	TIM_SR(DMA_TRIGGER_TIMER) = 0;
	DMA_LIFCR(CCR_DMA) = 0x3d << 16;

	DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) &(gCommutationPattern.get()[stepStart]);
	gLastSetSNDTR = DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) = stepCnt * TicksPerStep;
	if (cycle) {
		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_CIRC;
	} else {
		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_CIRC;
	}
	DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_EN;
	set_mHZ(mHZ);
}

void Driver::set_mHZ(uint32_t mHZ) {
	if (mHZ) {
		const uint64_t delayTicks = (CLOCK_APB1_TIMER_CLK * 1000) / mHZ;
		uint16_t psc = (delayTicks >> 16) & 0xffff;
		uint16_t arr = delayTicks / (psc + 1);
		TIM_PSC(DMA_TRIGGER_TIMER) = psc;
		TIM_ARR(DMA_TRIGGER_TIMER) = arr;
		TIM_CNT(DMA_TRIGGER_TIMER) = 0;
		TIM_EGR(DMA_TRIGGER_TIMER)  = TIM_EGR_UG;
		TIM_CR1(DMA_TRIGGER_TIMER) |= TIM_CR1_CEN;
	} else {
		TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
		TIM_EGR(DMA_TRIGGER_TIMER)  = TIM_EGR_UG;
	}
}

// get the index of the last performed step
uint32_t Driver::getCurStep() {
	Array<uint16_t, TicksPerStep> const* curPtr = (Array<uint16_t, TicksPerStep> const*)DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM);
	uint32_t stepsDone = (gLastSetSNDTR - DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM)) / TicksPerStep - ((DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) & DMA_CR_CIRC)?0:1);
	uint32_t ptrOffset = curPtr - gCommutationPattern.get().begin();
	return ptrOffset + stepsDone;
}

void Driver::setEnabled(bool enabled) {
	if (enabled) {
		gpio_set(PWM_PORT, PWM_ENABLE_PINS);
		TIM_CR1(PWM_TIMER) |= TIM_CR1_CEN;
		TIM_SMCR(PWM_TIMER) = TIM_SMCR_MSM;
		TIM_CR2(PWM_TIMER)  = TIM_CR2_MMS_UPDATE;
		TIM_EGR(PWM_TIMER) = TIM_EGR_UG;
	} else {
		gpio_clear(PWM_PORT, PWM_ENABLE_PINS);
		TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
		TIM_SMCR(PWM_TIMER) = 0;
		TIM_CR2(PWM_TIMER) = 0;
		TIM_CR1(PWM_TIMER)  &= ~TIM_CR1_CEN;
//		TIM_CCR1(PWM_TIMER) = 0;
//		TIM_CCR2(PWM_TIMER) = 0;
//		TIM_CCR3(PWM_TIMER) = 0;
//		TIM_CCR4(PWM_TIMER) = 0;
//		TIM_EGR(PWM_TIMER) = TIM_EGR_UG;
	}
}

void Driver::claim(DriverInterface* interface) {
	gCurrentInterface = interface;
}


void Driver::setPower(float power) {
	power = std::min(1.f, std::max(0.f, power));
	currentController.currentOutputScale = power;
}
float Driver::getPower() {
	return currentController.currentOutputScale;
}

namespace {
struct __attribute__((packed)) ManualStepsParams {
	uint32_t startStep;
	uint32_t stepCnt;
	uint32_t mHZ;
	bool cyclic;
};
struct : public flawless::Callback<ManualStepsParams&, bool> {
	void callback(ManualStepsParams& step, bool set) override {
		if (set) {
			pwmdriver::Driver::get().runSteps(step.startStep, step.stepCnt, step.mHZ, step.cyclic);
		}
	}
	flawless::ApplicationConfig<ManualStepsParams> mStep {"manual_steps", "IIIb", this};
} manualStepsHelper;

struct : public flawless::Callback<Array<uint16_t, 4>&, bool> {
	void callback(Array<uint16_t, 4>& manPWMs, bool set) override {
		if (set) {
			TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
			TIM_CR1(PWM_TIMER) |= TIM_CR1_CEN;
			TIM_CCR1(PWM_TIMER) = manPWMs[0];
			TIM_CCR2(PWM_TIMER) = manPWMs[1];
			TIM_CCR3(PWM_TIMER) = manPWMs[2];
			TIM_CCR4(PWM_TIMER) = manPWMs[3];
		}
	}
	flawless::ApplicationConfig<Array<uint16_t, 4>> mManualPWM {"manual_pwm", "4H", this};
} manualPWMHelper;

struct : public flawless::Callback<bool&, bool> {
	void callback(bool& enable, bool set) override {
		if (set) {
			pwmdriver::Driver::get().setEnabled(enable);
		}
	}
	flawless::ApplicationConfig<bool> mEnable {"enable_motor", "B", this};
} enableHelper;

struct InitHelper : public flawless::Module {
	InitHelper(unsigned int level) : flawless::Module(level) {}

	void initPins() {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
		gpio_mode_setup(PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_PINS);
		gpio_mode_setup(PWM_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWM_ENABLE_PINS);

		gpio_set_output_options(PWM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PWM_PINS);
		gpio_set_output_options(PWM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PWM_ENABLE_PINS);

		gpio_clear(PWM_PORT, PWM_ENABLE_PINS);
		gpio_set_af(PWM_PORT, GPIO_AF2, PWM_PINS);
	}

	void initPWMTimer() {
		RCC_APB1ENR |= RCC_APB1ENR_TIM4EN;

		TIM_CR1(PWM_TIMER) = 0;
		TIM_CR2(PWM_TIMER) = 0;
		TIM_SMCR(PWM_TIMER) = 0;

		TIM_CR1(PWM_TIMER)   = TIM_CR1_ARPE;// | TIM_CR1_CMS_CENTER_3;

		TIM_CCMR1(PWM_TIMER) = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
		TIM_CCMR2(PWM_TIMER) = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
		TIM_CCER(PWM_TIMER) |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

		TIM_PSC(PWM_TIMER)   = 0;
		TIM_ARR(PWM_TIMER)   = PwmAmplitude + PwmOffTimer;
		TIM_CNT(PWM_TIMER)   = 0;

		TIM_CCR1(PWM_TIMER) = 0;
		TIM_CCR2(PWM_TIMER) = 0;
		TIM_CCR3(PWM_TIMER) = 0;
		TIM_CCR4(PWM_TIMER) = 0;

		TIM_DCR(PWM_TIMER)  = (3 << 8) | 0xd;

		TIM_SR(PWM_TIMER)   = 0;
		TIM_EGR(PWM_TIMER)  = TIM_EGR_UG;
		TIM_SR(PWM_TIMER)   = 0;
	}

	void initDMA()
	{
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;

		TIM_CR1(DMA_TRIGGER_TIMER)  = TIM_CR1_ARPE | (TIM_CR1(DMA_TRIGGER_TIMER) & ~0x03ff);
		TIM_ARR(DMA_TRIGGER_TIMER)  = 0;

		TIM_CCMR1(DMA_TRIGGER_TIMER) = TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
		TIM_CCER(DMA_TRIGGER_TIMER) |= TIM_CCER_CC2E;

		TIM_EGR(DMA_TRIGGER_TIMER)  = TIM_EGR_UG;
		TIM_DIER(DMA_TRIGGER_TIMER) = TIM_DIER_UDE;

		do {
			DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_EN;
		} while (0 != (DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) & DMA_CR_EN));

		DMA_SFCR(CCR_DMA, CCR_DMA_STREAM) = DMA_FCR_DMDIS | DMA_FCR_FTH_50;
		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) =
							CCR_DMA_CHANNEL << DMA_CR_CHSEL_LSB |
							DMA_CR_MBURST_INCR4 | DMA_CR_PBURST_INCR4 |
							DMA_CR_MSIZE_HALFWORD | DMA_CR_PSIZE_HALFWORD |
							DMA_CR_CIRC | DMA_CR_DIR | DMA_CR_MINC;
		DMA_SPAR(CCR_DMA, CCR_DMA_STREAM)  = (uint32_t) &TIM_DMAR(PWM_TIMER);
		DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) gCommutationPattern->data();
		gLastSetSNDTR = DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) = 1;
	}

	void init(unsigned int) override {
		initPWMTimer();
		initPins();
		initDMA();
	}
} initHelper(5);
}

