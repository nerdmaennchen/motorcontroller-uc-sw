#include "HallFeedback.h"

#include <flawless/module/Module.h>
#include <flawless/util/Array.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/platform/system.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/applicationConfig/ApplicationConfig.h>

#include <target/stm32f4/clock.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/nvic.h>

constexpr uint32_t HALL_PORT = GPIOA;
constexpr uint32_t HALL_U    = GPIO10;
constexpr uint32_t HALL_V    = GPIO9;
constexpr uint32_t HALL_W    = GPIO8;
constexpr uint32_t HALL_PINS = (HALL_U | HALL_V | HALL_W);
constexpr uint32_t HALL_TIMER = TIM1;



namespace
{
flawless::MessageBufferMemory<HallFeedback, 5> hallFeedbackMessageBuffer;

}

namespace hallUtils {
uint8_t getNextStep(uint8_t curStep, bool cw) {
	uint8_t inv = ~curStep;
	if (cw) {
		inv = ((inv >> 1) & 3) | ((inv << 2) & 4);
		return inv;
	} else {
		inv = ((inv << 1) & 6) | ((inv >> 2) & 1);
		return inv;
	}
}
}

struct HallManager {
	void notifyHallTick(uint64_t delay) {
		static flawless::MessageBufferManager<HallFeedback>& bufferManager = flawless::MessageBufferManager<HallFeedback>::get();

		uint8_t expectedCWState = hallUtils::getNextStep(mHallVWUStates, true);
		mHallVWUStates = (gpio_port_read(HALL_PORT) >> 8) & 0x7;
		mMovingCW = (expectedCWState == mHallVWUStates);

		mHallDelays[mMeasureHallIndex] = mLastHallDelay = delay;
		++mMeasureHallIndex;
		if (mMeasureHallIndex >= mHallDelays.size()) {
			mMeasureHallIndex = 0;
		}
		mHallDelay = 0;
		for (auto const& d : mHallDelays) {
			mHallDelay += d;
		}
		mLastHallStates = mHallVWUStates;

		flawless::Message<HallFeedback> msg = bufferManager.getFreeMessage();
		if (msg) {
			(*msg).currentHallValues = mHallVWUStates;
			(*msg).lastTickDelay     = delay;
			(*msg).movingClockwise   = mMovingCW;
			msg.invokeDirectly<0>();
		}
	}

	bool mMovingCW;
	size_t mMeasureHallIndex {0};
	Array<uint64_t, 6> mHallDelays;
	uint64_t mLastHallDelay {0};
	int mLastSetMeasurementHalls {0};
	uint8_t mLastHallStates {0};
	Array<uint16_t, 8> mTickCounts;
	flawless::ApplicationConfig<uint8_t> mHallVWUStates{"hallstates"};
	flawless::ApplicationConfig<uint64_t> mHallDelay{"HallDelay"};
} hallManager;

namespace {
struct InitHelper : public flawless::Module {
	InitHelper(unsigned int level) : flawless::Module(level) {}

	void init(unsigned int) override {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;

		TIM_CR2(HALL_TIMER)   = TIM_CR2_TI1S;
		TIM_PSC(HALL_TIMER)   = CLOCK_APB2_TIMER_CLK / 1000000; // useconds precision
		TIM_ARR(HALL_TIMER)   = 0xffff;
		TIM_CCMR1(HALL_TIMER) = TIM_CCMR1_CC1S_IN_TI1;
		TIM_CCER(HALL_TIMER)  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;
		TIM_SMCR(HALL_TIMER)  = TIM_SMCR_TS_IT1F_ED | TIM_SMCR_SMS_RM;

		TIM_DIER(HALL_TIMER)  = TIM_DIER_CC1IE | TIM_DIER_UIE;

		TIM_CNT(HALL_TIMER)   = 0;
		TIM_CR1(HALL_TIMER)   = TIM_CR1_URS | TIM_CR1_CEN;

		nvic_enable_irq(NVIC_TIM1_CC_IRQ);
		nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);

		gpio_mode_setup(HALL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HALL_PINS);
		gpio_set_af(HALL_PORT, GPIO_AF1, HALL_PINS);

		hallManager.notifyHallTick(0);
	}
} initHelper(9);
}

extern "C" {
static volatile uint64_t delayBetweenTicks;
void tim1_up_tim10_isr() {
	if (TIM_SR(HALL_TIMER) & TIM_SR_UIF) {
		flawless::LockGuard lock;
		TIM_SR(HALL_TIMER) &= ~TIM_SR_UIF;
		delayBetweenTicks += TIM_ARR(HALL_TIMER);
		hallManager.notifyHallTick(delayBetweenTicks);
	}
}

void tim1_cc_isr()
{
	TIM_EGR(HALL_TIMER) = TIM_EGR_UG;
	flawless::LockGuard lock;
	delayBetweenTicks += TIM_CCR1(HALL_TIMER);

	hallManager.notifyHallTick(delayBetweenTicks);

	delayBetweenTicks = 0;
	TIM_SR(HALL_TIMER) = 0;
}
}
