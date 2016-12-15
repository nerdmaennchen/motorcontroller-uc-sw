#include "HallFeedback.h"
#include "PWMDriver.h"

#include <interfaces/ISRTime.h>
#include <interfaces/systemTime.h>

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
flawless::MessageBufferMemory<hall::Tick, 5> hallTickMessageBuffer;
flawless::MessageBufferMemory<hall::Timeout, 5> hallTimeoutMessageBuffer;

Array<int, 8> const ccwNext  {0, 3, 6, 2, 5, 1, 4, 0};
Array<int, 8> const cwNext {0, 5, 3, 1, 6, 4, 2, 0};

Array<int, 8> hall2Pos; // translate a hall value to a position (0..11)

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

constexpr uint32_t DebounceSize = 8;
using DebounceBuffer = Array<int, DebounceSize>;

constexpr uint32_t TickTimerBaseFrequency = CLOCK_APB1_CLK;
constexpr uint32_t TickTimerFrequency = 1000000;

namespace {
struct HallManager : public flawless::Module {
	HallManager(unsigned int level) : flawless::Module(level) {}

	systemTime_t mTimeOfLastTick {0};
	SystemTime const& time  {SystemTime::get()};
	void testForTransition() {
//		DebounceBuffer const* buffer = &mRawMeasureBuffer2;
//		if (DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) & DMA_CR_CT) {
//			buffer = &mRawMeasureBuffer1;
//		}
//		int state = (*buffer)[DebounceSize-1] & 0x7;
		int state = hall::readHallState();
		mCurrentPos = hall2Pos[state];
		systemTime_t now = time.getSystemTimeUS();
		mDelaySinceLastTick = now - mTimeOfLastTick;
		if (state == mHallStates) {
			// no transition happened
			if (mDelaySinceLastTick > mMaxDelayTillNextTick) {
				// timeout
				// setup new "waiting" interval
				mMaxDelayTillNextTick = std::min((mDelaySinceLastTick * 3) / 2, mMaxTickTimeout);
				TIM_ARR(HALL_TIMER) = std::max(1000ULL, mMaxDelayTillNextTick);
				auto msg = flawless::getFreeMessage<hall::Timeout>();
				if (msg) {
					msg->currentHallValues    = mHallStates;
					msg->delaySinceLastTickUS = mDelaySinceLastTick * (1000000 / TickTimerFrequency);
					msg->currentPos           = mCurrentPos;
					msg->movingCW             = mMovingCW;
					msg->moving               = mMoving;
					msg.post();
				}
			}
		} else {
			bool validTransition = false;
			const int expectedCWState  = cwNext[mHallStates];
			const int expectedCCWState = ccwNext[mHallStates];

			bool newMoveCW = false;
			if (expectedCWState == state) {
				++mOdometry;
				validTransition = true;
				newMoveCW = true;
				mCurrentPos -= 1;
			} else if (expectedCCWState == state) {
				--mOdometry;
				validTransition = true;
				mCurrentPos += 1;
			} else {
				++mInvalidTransitions;
			}
			mCurrentPos = (mCurrentPos+12) % 12;
			mHallStates = state;
			mTimeOfLastTick = now;
			if (validTransition && mMovingCW == newMoveCW) {
				mMoving = true;
				mPrevTickDelay = mDelaySinceLastTick;
//				mDelaySinceLastTick = 0;

				auto msg = flawless::getFreeMessage<hall::Tick>();
				if (msg) {
					msg->currentHallValues    = mHallStates;
					msg->prevTickDelayUS      = mPrevTickDelay;
					msg->currentPos           = mCurrentPos;
					msg->movingCW             = newMoveCW;
					msg->moving               = mMoving;
					msg.post();
				}
			} else {
				mMoving = false;
			}
			mMovingCW = newMoveCW;
		}
	}

	uint64_t mDelaySinceLastTick    {0ULL};
	uint64_t mMaxDelayTillNextTick  {0ULL};
	uint64_t mMaxTickTimeout        {(TickTimerFrequency/1000) * 100}; // 0.1s
	uint64_t mPrevTickDelay         {0ULL};
	int      mHallStates            ;
	int      mCurrentPos            ;
	int64_t  mOdometry              ;
	int64_t  mInvalidTransitions    ;
	uint64_t mTicksCaptured         {0ULL};
	bool     mMovingCW              {true};
	bool     mMoving                {false};


	DebounceBuffer mRawMeasureBuffer1;
	DebounceBuffer mRawMeasureBuffer2;

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
		if (mRawMeasureBuffer1.size() > 1) {
			DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) |= DMA_CR_MINC;
		}

//		nvic_enable_irq(NVIC_DMA2_STREAM5_IRQ);
		DMA_SPAR(HALL_DMA, HALL_DMA_STREAM)  = (uint32_t) (&(GPIO_IDR(HALL_PORT)));
		DMA_SNDTR(HALL_DMA, HALL_DMA_STREAM) = mRawMeasureBuffer1.size();
		DMA_SM0AR(HALL_DMA, HALL_DMA_STREAM) = (uint32_t) (mRawMeasureBuffer1.data());
		DMA_SM1AR(HALL_DMA, HALL_DMA_STREAM) = (uint32_t) (mRawMeasureBuffer2.data());
		DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) |=  DMA_CR_TCIE;
		DMA_SCCR(HALL_DMA, HALL_DMA_STREAM) |= DMA_CR_EN;

		TIM_DIER(PWM_TIMER) |= TIM_DIER_UDE;
	}

	void initTimer() {
		RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

		TIM_CR2(HALL_TIMER)   = TIM_CR2_TI1S;
		TIM_SMCR(HALL_TIMER)  = TIM_SMCR_TS_IT1F_ED | TIM_SMCR_SMS_RM;
		TIM_CCMR1(HALL_TIMER) = TIM_CCMR1_CC1S_IN_TRC;
		TIM_CCER(HALL_TIMER) |= TIM_CCER_CC1E;


		TIM_PSC(HALL_TIMER)   = TickTimerBaseFrequency / TickTimerFrequency - 1;
		TIM_ARR(HALL_TIMER)   = std::max(0xffffULL, mMaxTickTimeout);

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


		int step = 1; // at phase 1 we read a hall feedback of 1
		for (int i=0; i < 6; ++i) {
			hall2Pos[step] = (12 + (2*i)) % 12;
			step = hall::getNextStep(step, true);
		}

		SystemTime::get().sleep(1000); // 1ms delay to settle the pullups
		mHallStates = hall::readHallState();
		mCurrentPos = hall2Pos[mHallStates];
		for (auto &debounceState : mRawMeasureBuffer1) { debounceState = mCurrentPos; }
		for (auto &debounceState : mRawMeasureBuffer2) { debounceState = mCurrentPos; }
		mMaxDelayTillNextTick = mMaxTickTimeout;
		mDelaySinceLastTick = mMaxDelayTillNextTick + 1;
		testForTransition();

		initDMA();
		initTimer();
	}
} hallManager(50);
}

extern "C" {
static unsigned int ticks = 0; // a workaround for when the CCR fires like mad (happens for some reasons)
void tim2_isr()
{
	ISRTime isrTimer;
	int sr = TIM_SR(HALL_TIMER);
	TIM_SR(HALL_TIMER) = 0;
	bool perfomTest = false;
	if (sr & TIM_SR_UIF) {
//		hallManager.mDelaySinceLastTick += TIM_ARR(HALL_TIMER);
		perfomTest = true;
	}
	if (sr & TIM_SR_CC1IF) {
		ticks += TIM_CCR1(HALL_TIMER);
//		hallManager.mDelaySinceLastTick += TIM_CCR1(HALL_TIMER);
		++(hallManager.mTicksCaptured);
		TIM_CCR1(HALL_TIMER) = 0;
		perfomTest = true;
	}
	if (perfomTest) {
		hallManager.testForTransition();
	}
}

void dma2_stream5_isr()
{
	ISRTime isrTimer;
	ticks = 0;
	hallManager.testForTransition();
	DMA_HIFCR(HALL_DMA) = 0x3d<<6;
}
}

namespace
{

flawless::ApplicationConfigMapping<uint64_t> mTicksCaptured       {"hall.ticks_captured",      "Q", hallManager.mTicksCaptured}; // only for debug purposes

flawless::ApplicationConfigMapping<uint64_t> mDelaySinceLastTick  {"hall.delay",               "Q", hallManager.mDelaySinceLastTick};
flawless::ApplicationConfigMapping<bool>     mMoving              {"hall.moving",              "B", hallManager.mMoving};
flawless::ApplicationConfigMapping<uint64_t> mPrevTickDelay       {"hall.delay_prev",          "Q", hallManager.mPrevTickDelay};
flawless::ApplicationConfigMapping<int>      mHallStates          {"hall.states",              "I", hallManager.mHallStates};
flawless::ApplicationConfigMapping<int>      mHallPhase           {"hall.phase",               "I", hallManager.mCurrentPos};
flawless::ApplicationConfigMapping<int64_t>  mOdometry            {"hall.odometry",            "q", hallManager.mOdometry};
flawless::ApplicationConfigMapping<int64_t>  mInvalidTransitions  {"hall.invalid_transitions", "q", hallManager.mInvalidTransitions};
}
