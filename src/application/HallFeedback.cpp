#include "HallFeedback.h"
#include "PWMDriver.h"

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
#include <libopencm3/stm32/f4/dma.h>

#include <algorithm>
#include <cmath>

constexpr uint32_t HALL_PORT = GPIOA;
constexpr uint32_t HALL_U    = GPIO0;
constexpr uint32_t HALL_V    = GPIO1;
constexpr uint32_t HALL_W    = GPIO2;
constexpr uint32_t HALL_PINS = (HALL_U | HALL_V | HALL_W);
#define HALL_TIMER TIM2

#define HALL_DMA DMA2
#define HALL_DMA_STREAM  DMA_STREAM_5
#define HALL_DMA_CHANNEL 6


namespace
{
flawless::MessageBufferMemory<hall::Feedback, 5> hallFeedbackMessageBuffer;

Array<int, 8> ccwNext  {0, 3, 6, 2, 5, 1, 4, 0};
Array<int, 8> cwNext {0, 5, 3, 1, 6, 4, 2, 0};
}

namespace hall {

int getNextStep(int curStep, bool cw) {
	if (cw) {
		return cwNext[curStep];
	} else {
		return ccwNext[curStep];
	}
}

inline int readHallState() {
	return (GPIO_IDR(HALL_PORT) & HALL_PINS);
}
}

enum class MovingDirection {
	None,
	CW,
	CCW
};

constexpr uint32_t DebounceSize = 4;
using DebounceBuffer = Array<int, DebounceSize>;

namespace {
struct HallManager : public flawless::Module {
	HallManager(unsigned int level) : flawless::Module(level) {}


	void publishMessage() {
		auto msg = flawless::getFreeMessage<hall::Feedback>();
		if (msg) {
			msg->currentHallValues    = mHallStates;
			msg->lastTickDelay_us     = mDelaySinceLastTick;
			msg->tickFreq_Hz          = mEstimatedFrequency;
			msg.invokeDirectly<0>();
		}
	}

	void notifyHallTick(int newHallStates) {
		bool validTransition = false;
		const int expectedCWState  = cwNext[mHallStates];
		const int expectedCCWState = ccwNext[mHallStates];

		flawless::LockGuard lock;
		float t = float(mDelaySinceLastTick) * (1.f / float(CLOCK_APB1_TIMER_CLK));
		if (expectedCWState == newHallStates) {
			++mOdometry;
			validTransition = true;
		} else if (expectedCCWState == newHallStates) {
			--mOdometry;
			t = -t;
			validTransition = true;
		} else {
			++mInvalidTransitions;
		}

		mHallStates = newHallStates;
		if (validTransition and mDelaySinceLastTick != 0) {
			mPrevTickDelay = mDelaySinceLastTick;
			mEstimatedFrequency = (1.f-mFrequencyInnovation) * mEstimatedFrequency + mFrequencyInnovation * (1. / t);
			mDelaySinceLastTick = 0;
			publishMessage();
		}
	}

	uint64_t mDelaySinceLastTick {0};

	flawless::ApplicationConfig<uint64_t> mPrevTickDelay {"hall.delay", "Q", 0ULL};
	flawless::ApplicationConfig<float> mEstimatedFrequency{"hall.frequency", "f"};
	flawless::ApplicationConfig<float> mFrequencyInnovation{"hall.frequency_innovation", "f", 1.f};
	flawless::ApplicationConfig<int>   mHallStates{"hall.states", "I"};
	flawless::ApplicationConfig<int64_t> mOdometry{"hall.odometry", "q"};
	flawless::ApplicationConfig<int64_t> mInvalidTransitions{"hall.invalid_transitions", "q"};


	flawless::ApplicationConfig<DebounceBuffer> mRawMeasureBuffer1{"analog.buffer1", "16I"};
	flawless::ApplicationConfig<DebounceBuffer> mRawMeasureBuffer2{"analog.buffer2", "16I"};
	void notifyDMADone() {
		DebounceBuffer const* buffer = &(mRawMeasureBuffer2.get());
		if (DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) & DMA_CR_CT) {
			buffer = &(mRawMeasureBuffer1.get());
		}
		int state = (*buffer)[0] & 0x7;
		if (state == mHallStates) {
			return;
		}
		for (uint32_t i = 1; i < buffer->size(); ++i) {
			if (state != ((*buffer)[i] & 0x7)) {
				return;
			}
		}
		notifyHallTick(state);
	}

	void initDMA() {
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;
		while (0 != (DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) & DMA_CR_EN)) {
			DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) &= ~DMA_CR_EN;
		}
		DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) =
				(HALL_DMA_CHANNEL << DMA_CR_CHSEL_LSB) |
				DMA_CR_DBM |
				DMA_CR_PSIZE_WORD |
				DMA_CR_MSIZE_WORD;
		if (mRawMeasureBuffer1.get().size() > 1) {
			DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) |= DMA_CR_MINC;
		}

		nvic_enable_irq(NVIC_DMA2_STREAM5_IRQ);
		DMA_SPAR(HALL_DMA, HALL_DMA_STREAM)  = (uint32_t) (&(GPIO_IDR(HALL_PORT)));
		DMA_SNDTR(HALL_DMA, HALL_DMA_STREAM) = mRawMeasureBuffer1.get().size();
		DMA_SM0AR(HALL_DMA, HALL_DMA_STREAM) = (uint32_t) (mRawMeasureBuffer1.get().data());
		DMA_SM1AR(HALL_DMA, HALL_DMA_STREAM) = (uint32_t) (mRawMeasureBuffer2.get().data());
		DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) |=  DMA_CR_TCIE;
		DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) |= DMA_CR_EN;

		TIM_DIER(PWM_TIMER) |= TIM_DIER_UDE;
	}

	void initTimer() {
		RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

		TIM_CR2(HALL_TIMER)   = TIM_CR2_TI1S;
		TIM_SMCR(HALL_TIMER)  = TIM_SMCR_MSM | TIM_SMCR_TS_IT1F_ED | TIM_SMCR_SMS_RM;
		TIM_CCMR1(HALL_TIMER) = TIM_CCMR1_CC1S_IN_TI1;
		TIM_CCER(HALL_TIMER) |= TIM_CCER_CC1E;


		TIM_PSC(HALL_TIMER)   = 0;
		TIM_ARR(HALL_TIMER)   = (1<<16)-1;

		TIM_CNT(HALL_TIMER)   = 0;
		TIM_CR1(HALL_TIMER)   = TIM_CR1_URS | TIM_CR1_CEN;

		TIM_SR(HALL_TIMER) = 0;
		nvic_enable_irq(NVIC_TIM2_IRQ);
		nvic_set_priority(NVIC_TIM2_IRQ, 1);
		TIM_DIER(HALL_TIMER)  = TIM_DIER_CC1IE | TIM_DIER_UIE;
	}

	void init(unsigned int) override {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		gpio_mode_setup(HALL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, HALL_PINS);
		gpio_set_af(HALL_PORT, GPIO_AF1, HALL_PINS);

		SystemTime::get().sleep(1000); // 1ms delay to settle the pullups
		mHallStates = hall::readHallState();
		publishMessage();

		initDMA();
		initTimer();

	}
} hallManager(50);
}

extern "C" {
void tim2_isr()
{
	flawless::LockGuard lock;
	ISRTime isrTimer;
	int sr = TIM_SR(HALL_TIMER);
	TIM_SR(HALL_TIMER) = 0;
	if (sr & TIM_SR_UIF) {
		hallManager.mDelaySinceLastTick += TIM_ARR(HALL_TIMER);
	}
	if (sr & TIM_SR_CC1IF) {
		hallManager.mDelaySinceLastTick += TIM_CCR1(HALL_TIMER);
		TIM_CCR1(HALL_TIMER) = 0;
	}
}

void dma2_stream5_isr()
{
	ISRTime isrTimer;
	hallManager.notifyDMADone();
	DMA_HIFCR(HALL_DMA) = 0x3d<<6;
}
}

