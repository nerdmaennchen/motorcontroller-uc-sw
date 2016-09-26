#include "HallFeedback.h"
#include <interfaces/ISRTime.h>

#include <flawless/module/Module.h>
#include <flawless/util/Array.h>
#include <flawless/util/FiFo.h>
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

#include <algorithm>
#include <cmath>

constexpr uint32_t HALL_PORT = GPIOA;
constexpr uint32_t HALL_U    = GPIO10;
constexpr uint32_t HALL_V    = GPIO9;
constexpr uint32_t HALL_W    = GPIO8;
constexpr uint32_t HALL_PINS = (HALL_U | HALL_V | HALL_W);
constexpr uint32_t HALL_TIMER = TIM1;



namespace
{
flawless::MessageBufferMemory<hall::Feedback, 5> hallFeedbackMessageBuffer;

Array<uint8_t, 8> cwNext  {0, 3, 6, 2, 5, 1, 4, 0};
Array<uint8_t, 8> ccwNext {0, 5, 3, 1, 6, 4, 2, 0};

int expectedARR = 0;
}

namespace hall {

uint8_t getNextStep(uint8_t curStep, bool cw) {
	if (cw) {
		return cwNext[curStep];
	} else {
		return ccwNext[curStep];
	}
}

uint8_t readHallState() {
	uint8_t hallU = 0 != (GPIO_IDR(HALL_PORT) & HALL_U);
	uint8_t hallV = 0 != (GPIO_IDR(HALL_PORT) & HALL_V);
	uint8_t hallW = 0 != (GPIO_IDR(HALL_PORT) & HALL_W);
	return (hallU << 2) | (hallV << 1) | (hallW << 0);
}
}

enum class MovingDirection {
	None,
	CW,
	CCW
};

struct HallManager {
	flawless::MessageBufferManager<hall::Feedback>& bufferManager {flawless::MessageBufferManager<hall::Feedback>::get()};

	// assuming the position of the rotor folows the formula p(t) = a + b*x + c*x^2 where a = 0
	float velocityModelB {0.f};
	float velocityModelC {0.f};

	void publishMessage() {
		auto msg = bufferManager.getFreeMessage();
		if (msg) {
			msg->currentHallValues    = mHallStates;
			msg->lastTickDelay_us     = mDelaySinceLastTick;
			msg->tickFreq_Hz          = mEstimatedFrequency;
			msg.invokeDirectly<0>();
		}
	}

	void notifyTimeout(uint64_t delay_us) {
		mDelaySinceLastTick += delay_us;
		if (mDelaySinceLastTick > mPrevTickDelay) {
			mEstimatedFrequency = (mStepDir / float(mDelaySinceLastTick* 1e-6)); // assuming we have moved a bit but slow down
			mPrevTickDelay = mDelaySinceLastTick;
		}
		unsigned int timeout = mDelaySinceLastTick + mDelaySinceLastTick / 4;
		timeout = std::min(0xffffU, timeout);
		expectedARR = TIM_ARR(HALL_TIMER) = timeout;
		publishMessage();
	}

	void notifyHallTick(uint64_t delay_us) {
		mHallStates = hall::readHallState();
		mDelaySinceLastTick += delay_us;

		bool validTransition = false;
		uint8_t expectedCWState = hall::getNextStep(mLastHallState, true);
		uint8_t expectedCCWState = hall::getNextStep(mLastHallState, false);
		if (mHallStates != mLastHallState) {
			if (expectedCWState == mHallStates) {
				++mOdometry;
				validTransition = true;
				if (mStepDir == .5f) {
					mStepDir = 1.f;
				} else {
					mStepDir = .5f;
				}
			} else if (expectedCCWState == mHallStates) {
				--mOdometry;
				validTransition = true;
				if (mStepDir == -.5f) {
					mStepDir = -1.f;
				} else {
					mStepDir = -.5f;
				}
			} else {
				mStepDir = 0.f;
			}
		}

		if (validTransition and mDelaySinceLastTick > 100) {
			float t = float(mDelaySinceLastTick) * 1e-6f;
			mPrevTickDelay = mDelaySinceLastTick;
			float tsq = t * t;

			velocityModelC = (mStepDir - velocityModelB * t) / tsq;
			velocityModelB = mStepDir / t;
			mEstimatedFrequency = velocityModelB;

			unsigned int timeout = mDelaySinceLastTick + mDelaySinceLastTick / 4;
			timeout = std::min(0xffffU, timeout);
			expectedARR = TIM_ARR(HALL_TIMER) = timeout;
		} else {
		}
		publishMessage();
		mDelaySinceLastTick = 0;
		mLastHallState = mHallStates;
	}
	uint64_t mDelaySinceLastTick {0};
	uint8_t mLastHallState {0};
	float mStepDir = 0.f;
	flawless::ApplicationConfig<uint64_t> mPrevTickDelay {"hall_delay", "Q", 0ULL};
	flawless::ApplicationConfig<float> mEstimatedFrequency{"hall_frequency", "f"};
	flawless::ApplicationConfig<uint8_t> mHallStates{"hall_states", "B"};
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
		expectedARR = TIM_ARR(HALL_TIMER)   = 10000;
		TIM_CCMR1(HALL_TIMER) = TIM_CCMR1_CC1S_IN_TI1;
		TIM_CCER(HALL_TIMER)  = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;
		TIM_SMCR(HALL_TIMER)  = TIM_SMCR_TS_IT1F_ED | TIM_SMCR_SMS_RM;


		TIM_CNT(HALL_TIMER)   = 0;
		TIM_CR1(HALL_TIMER)   = TIM_CR1_URS | TIM_CR1_CEN;

		gpio_mode_setup(HALL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HALL_PINS);
		gpio_set_af(HALL_PORT, GPIO_AF1, HALL_PINS);

		SystemTime::get().sleep(1000); // 1ms delay to settle the pullups

		hallManager.mLastHallState = hallManager.mHallStates = hall::readHallState();
		hallManager.publishMessage();

		TIM_SR(HALL_TIMER) = 0;
		nvic_enable_irq(NVIC_TIM1_CC_IRQ);
		nvic_enable_irq(NVIC_TIM1_UP_TIM10_IRQ);
		TIM_DIER(HALL_TIMER)  = TIM_DIER_CC1IE | TIM_DIER_UIE;
	}
} initHelper(50);
}

flawless::ApplicationConfig<int> magicCnt{"magic_cnt", "i"};

extern "C" {
void tim1_up_tim10_isr() {
	ISRTime isrTimer;
	flawless::LockGuard lock;
	int sr = TIM_SR(HALL_TIMER);
	if ((sr & TIM_SR_UIF) && !(sr & TIM_SR_CC1IF)) {
		magicCnt = TIM_ARR(HALL_TIMER);
		TIM_SR(HALL_TIMER) &= ~TIM_SR_UIF;
		hallManager.notifyTimeout(TIM_ARR(HALL_TIMER));
	}
}

void tim1_cc_isr()
{
	ISRTime isrTimer;
	flawless::LockGuard lock;
	TIM_SR(HALL_TIMER) = 0;
	hallManager.notifyHallTick(TIM_CCR1(HALL_TIMER));
}
}

