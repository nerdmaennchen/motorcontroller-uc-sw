#include "HallFeedback.h"
#include <interfaces/ISRTime.h>

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
	flawless::MessageBufferManager<HallFeedback>& bufferManager {flawless::MessageBufferManager<HallFeedback>::get()};
	void notifyHallTick(uint64_t delay_us) {

		mHallStates = (gpio_port_read(HALL_PORT) >> 8) & 0x7;
		if (mHallStates == mLastHallState) {
			mMovingDirection = MovingDirection::None;
		} else {
			uint8_t expectedCWState = hallUtils::getNextStep(mLastHallState, true);
			if (expectedCWState == mHallStates) {
				mMovingDirection = MovingDirection::CW;
				mLastMovingDirectionCW = true;
				++mOdometry;
			} else {
				mMovingDirection = MovingDirection::CCW;
				mLastMovingDirectionCW = false;
				--mOdometry;
			}
		}
		int64_t delay_signed = delay_us * (mLastMovingDirectionCW?1:-1);
		mLastHallFrequency_mHz = 1000000000 / delay_signed;
		mHallDelays[mMeasureHallIndex] = mLastHallDelay_us = delay_signed;
		if (mMovingDirection != MovingDirection::None) {
			++mMeasureHallIndex;
			if (mMeasureHallIndex >= mHallDelays.size()) {
				mMeasureHallIndex = 0;
			}
		}
		mAveragedHallDelay_us = 0;
		for (auto const& d : mHallDelays) {
			mAveragedHallDelay_us += d;
		}
		mAveragedHallFrequency_mHz = 1000000000 / (mAveragedHallDelay_us / mHallDelays.size());
		mLastHallState = mHallStates;

		flawless::Message<HallFeedback> msg = bufferManager.getFreeMessage();
		if (msg) {
			msg->currentHallValues    = mHallStates;
			msg->lastTickDelay_us     = mLastHallDelay_us;
			msg->averagedTickDelay_us = mAveragedHallDelay_us / 6;
			msg->movingDirection      = mMovingDirection;
			msg->lastTickFreq_mHz     = mLastHallFrequency_mHz;
			msg->averageTickFreq_mHz  = mAveragedHallFrequency_mHz;
			msg.invokeDirectly<0>();
		}
	}
	bool mLastMovingDirectionCW {true};
	MovingDirection mMovingDirection {MovingDirection::None};
	size_t mMeasureHallIndex {0};
	Array<int64_t, 6> mHallDelays;
	int64_t mLastHallDelay_us {0};
	int64_t mLastHallFrequency_mHz{0};
	uint8_t mLastHallState {0};
	flawless::ApplicationConfig<uint8_t> mHallStates{"hall_states", "B"};
	flawless::ApplicationConfig<int64_t> mAveragedHallDelay_us{"hall_delay", "q"};
	flawless::ApplicationConfig<int64_t> mAveragedHallFrequency_mHz{"hall_frequency", "q"};
	flawless::ApplicationConfig<int64_t> mOdometry{"hall_odometry", "q"};
} hallManager;

namespace {
struct InitHelper : public flawless::Module {
	InitHelper(unsigned int level) : flawless::Module(level) {}

	void init(unsigned int) override {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;

		TIM_CR2(HALL_TIMER)   = TIM_CR2_TI1S;
		TIM_PSC(HALL_TIMER)   = CLOCK_APB2_TIMER_CLK / 1000000 - 1; // useconds precision
		TIM_ARR(HALL_TIMER)   = 1000;
		TIM_CCMR1(HALL_TIMER) = TIM_CCMR1_CC1S_IN_TI1;
		TIM_CCER(HALL_TIMER)  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;
		TIM_SMCR(HALL_TIMER)  = TIM_SMCR_TS_IT1F_ED | TIM_SMCR_SMS_RM;

		TIM_DIER(HALL_TIMER)  = TIM_DIER_CC1IE | TIM_DIER_UIE;

		TIM_CNT(HALL_TIMER)   = 0;
		TIM_CR1(HALL_TIMER)   = TIM_CR1_URS | TIM_CR1_CEN;

		gpio_mode_setup(HALL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HALL_PINS);
		gpio_set_af(HALL_PORT, GPIO_AF1, HALL_PINS);

		SystemTime::get().sleep(1000); // 1ms delay to settle the pullups

		hallManager.mLastHallState = (gpio_port_read(HALL_PORT) >> 8) & 0x7;
		hallManager.notifyHallTick(0);

		nvic_enable_irq(NVIC_TIM1_CC_IRQ);
		nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);


	}
} initHelper(50);
}

extern "C" {
static volatile uint64_t delayBetweenTicks;
void tim1_up_tim10_isr() {
	ISRTime isrTimer;
	if (TIM_SR(HALL_TIMER) & TIM_SR_UIF) {
		flawless::LockGuard lock;
		TIM_SR(HALL_TIMER) &= ~TIM_SR_UIF;
		delayBetweenTicks += TIM_ARR(HALL_TIMER);
		hallManager.notifyHallTick(delayBetweenTicks);
	}
}

void tim1_cc_isr()
{
	ISRTime isrTimer;
	TIM_EGR(HALL_TIMER) = TIM_EGR_UG;
	flawless::LockGuard lock;
	delayBetweenTicks += TIM_CCR1(HALL_TIMER);

	hallManager.notifyHallTick(delayBetweenTicks);

	delayBetweenTicks = 0;
	TIM_SR(HALL_TIMER) = 0;
}
}
