#include "MotorDriver.h"

#include <flawless/stdtypes.h>
#include <flawless/module/Module.h>

#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>

#include <flawless/core/Listener.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/nvic.h>

#include <target/stm32f4/clock.h>

#include <flawless/util/Array.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/timer/swTimer.h>

#include "MotorCurrent.h"

#include <math.h>
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

#define M_PI 3.14159265359f

using namespace motordriver;

static int myModulo(int i, int m) {
	return (i%m + m) % m;
}

struct MotorDriver : public flawless::Module, flawless::Listener<MotorCurrentMeasure, 3>, flawless::Listener<MotorCurrent, 3>, public flawless::TimerCallback
{
	using CommutatinPattern = Array<Array<uint16_t, TicksPerStep>, StepsCount * 2>;
	using RemapType = Array<uint8_t, 4>;

	MotorDriver(unsigned int level) : flawless::Module(level), flawless::TimerCallback(500000, true) {}
	virtual ~MotorDriver() {}

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
		TIM_ARR(PWM_TIMER)   = PWM_AMPLITUDE + PWM_OFF_TIME;
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

		while (0 != (DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) & DMA_CR_EN)) {
			DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_EN;
		}

		DMA_SFCR(CCR_DMA, CCR_DMA_STREAM) = DMA_FCR_DMDIS | DMA_FCR_FTH_50;
		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) =
							CCR_DMA_CHANNEL << DMA_CR_CHSEL_LSB |
							DMA_CR_MBURST_INCR4 | DMA_CR_PBURST_INCR4 |
							DMA_CR_MSIZE_HALFWORD | DMA_CR_PSIZE_HALFWORD |
							DMA_CR_CIRC | DMA_CR_DIR | DMA_CR_MINC;
		DMA_SPAR(CCR_DMA, CCR_DMA_STREAM)  = (uint32_t) &TIM_DMAR(PWM_TIMER);
		DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) g_cwConfigs->data();
		mLastSetSNDTR = DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) = StepsCount * TicksPerStep;
		runTickIntervall(0, 0, 0, true);
	}

	flawless::ApplicationConfig<MotorCurrent> mMotorCurrentMean{"motorCurrentMean"};
	flawless::ApplicationConfig<MotorCurrent> mMaxCurrent{"maxCurrent", 0.2f};
	flawless::ApplicationConfig<MotorCurrent> mCurrentP{"currentP", 0.1f};
	flawless::ApplicationConfig<MotorCurrent> mCurrentError{"currentError"};
	void callback(flawless::Message<MotorCurrent> const& motorCurrent) override {
		mMotorCurrentMean = *motorCurrent;
		mCurrentError = mMotorCurrentMean - mMaxCurrent;
		uint32_t targetAmplitude = TIM_ARR(PWM_TIMER) * (1.f + (mMotorCurrentMean-mMaxCurrent) * mCurrentP);
		targetAmplitude = std::max(PWM_AMPLITUDE+PWM_OFF_TIME, std::min(uint32_t(0xffff), targetAmplitude));
		TIM_ARR(PWM_TIMER)   = targetAmplitude;
	}

	void callback(flawless::Message<MotorCurrentMeasure> const& motorCurrent) override {
		(void) motorCurrent;
		for (uint32_t i = 0; i < (*motorCurrent).vals.size(); ++i) {
			mMotorCurrent->vals[i] = (*motorCurrent).vals[i];
		}
	}

	void callback() {

	}

	flawless::ApplicationConfig<MotorCurrentMeasure> mMotorCurrent{"motorCurrent"};

	void setupPWMConfigs(uint8_t stepsPerPhase) {
		mStepsPerPhase = stepsPerPhase;
		double phaseOffset = 2.f * M_PI / float(stepsPerPhase);
		const float scaleFactor = M_PI * 2.f / float(StepsCount);
		for (std::size_t sIdx = 0; sIdx < g_ccwConfigs->size(); ++sIdx) {
			float phase0 = (float)sIdx * scaleFactor;
			for (size_t i = 0; i < TicksPerStep; ++i) {
				float phase = phase0 + phaseOffset * i;
				float s = std::sin(phase);
				s = s * .5f + .5f;
				const float intermediate = s * PWM_AMPLITUDE + PWM_OFF_TIME / 2;
				const uint16_t sVal = uint32_t(roundf(intermediate));
				g_cwConfigs.get()[sIdx][mRemaps[i]] = sVal;
				g_ccwConfigs.get()[g_ccwConfigs.get().size() - 1 - sIdx][mRemaps[i]] = sVal;
			}
		}
	}

	void runTickIntervall(int32_t stepStart, uint32_t stepCnt, int mHZ, bool cycle=true) {
		while (0 != (DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) & DMA_CR_EN)) {
			DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_EN;
		}

		TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
		stepStart = myModulo(stepStart, StepsCount);
		if (mHZ != 0) {
			const long unsigned int delayTicks = (CLOCK_APB1_TIMER_CLK * mStepsPerPhase * 1000) / (abs(mHZ) * StepsCount);
			uint16_t psc = (delayTicks >> 16) & 0xffff;
			uint16_t arr = delayTicks / (psc + 1);
			TIM_PSC(DMA_TRIGGER_TIMER) = psc;
			TIM_ARR(DMA_TRIGGER_TIMER) = arr;
			TIM_CNT(DMA_TRIGGER_TIMER) = 0;

			DMA_LIFCR(CCR_DMA) = 0x3d << 16;

			if (mHZ > 0) {
				DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) &(g_cwConfigs.get()[stepStart]);
			} else {
				DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) &(g_ccwConfigs.get()[StepsCount - stepStart]);
			}

			mLastSetSNDTR = DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) = stepCnt * TicksPerStep;
			if (cycle) {
				DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_CIRC;
			} else {
				DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_CIRC;
			}
			DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_EN;

			TIM_EGR(DMA_TRIGGER_TIMER) = TIM_EGR_UG;
			TIM_CR1(DMA_TRIGGER_TIMER) |= TIM_CR1_CEN;
		} else {
			Array<uint16_t, 4> const* arrVals = &(g_cwConfigs.get()[stepStart]);
			DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) &(g_cwConfigs.get()[stepStart]);
			mLastSetSNDTR = DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) = 0;

			TIM_CCR1(PWM_TIMER) = (*arrVals)[0];
			TIM_CCR2(PWM_TIMER) = (*arrVals)[1];
			TIM_CCR3(PWM_TIMER) = (*arrVals)[2];
			TIM_CCR4(PWM_TIMER) = (*arrVals)[3];
		}
	}

	uint32_t getCurStep() {
		Array<uint16_t, TicksPerStep> const* curPtr = (Array<uint16_t, TicksPerStep> const*)DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM);
		uint32_t stepsDone = (mLastSetSNDTR - DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM)) / TicksPerStep;
		if (g_ccwConfigs.get().begin() <= curPtr and curPtr < g_ccwConfigs.get().end()) {
			uint32_t ptrOffset = curPtr - g_ccwConfigs.get().begin();
			return (2 * StepsCount - ptrOffset + stepsDone) % StepsCount;
		} else {
			uint32_t ptrOffset = curPtr - g_cwConfigs.get().begin();
			return (2 * StepsCount + ptrOffset + stepsDone) % StepsCount;
		}
	}

	void advanceToTick(uint32_t tickCnt) {
		tickCnt = tickCnt % (TotalTickCnt);
		TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
		TIM_CR1(PWM_TIMER) |= TIM_CR1_UDIS;
		while ((DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) % (TotalTickCnt)) != tickCnt) {
			TIM_EGR(DMA_TRIGGER_TIMER) = TIM_EGR_UG;
		}
		TIM_CR1(PWM_TIMER) &= ~TIM_CR1_UDIS;
	}

	void manualUpdateCCRs() {
		uint32_t idx = StepsCount - DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) / TicksPerStep;
		Array<uint16_t, 4> const* arrVals = ((Array<uint16_t, 4> const*)DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM)) + idx;
		TIM_CCR1(PWM_TIMER) = (*arrVals)[0];
		TIM_CCR2(PWM_TIMER) = (*arrVals)[1];
		TIM_CCR3(PWM_TIMER) = (*arrVals)[2];
		TIM_CCR4(PWM_TIMER) = (*arrVals)[3];
	}

	void setEnabled(bool enabled) {
		if (enabled) {
			gpio_set(PWM_PORT, PWM_ENABLE_PINS);
			manualUpdateCCRs();
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


			TIM_CCR1(PWM_TIMER) = 0;
			TIM_CCR2(PWM_TIMER) = 0;
			TIM_CCR3(PWM_TIMER) = 0;
			TIM_CCR4(PWM_TIMER) = 0;
			TIM_EGR(PWM_TIMER) = TIM_EGR_UG;
		}
	}

	void setSpeed(int mHZ)
	{
		int sign = mHZ >= 0;
		const long unsigned int delayTicks = (CLOCK_APB1_TIMER_CLK * mStepsPerPhase * 1000) / (abs(mHZ) * StepsCount);

		TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
		const uint32_t oldPSC = TIM_PSC(DMA_TRIGGER_TIMER);
		const uint32_t oldARR = TIM_ARR(DMA_TRIGGER_TIMER);
		const uint32_t oldDelayTicks = oldARR * (oldPSC + 1);
		const uint32_t ticksPassed   = TIM_CNT(DMA_TRIGGER_TIMER) * (oldPSC + 1);
		int needUG = (!oldDelayTicks) || (delayTicks < (oldDelayTicks - ticksPassed) / 2);

		uint16_t psc = (delayTicks >> 16) & 0xffff;
		uint16_t arr = delayTicks / (psc + 1);
		TIM_PSC(DMA_TRIGGER_TIMER) = psc;
		TIM_ARR(DMA_TRIGGER_TIMER) = arr;

		if (sign) {
			if (DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) == (uint32_t) (&g_ccwConfigs.get()[0])) {
				// we need to change the direction of movement
				while (0 != (DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) & DMA_CR_EN)) {
					DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_EN;
				}
				DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) (&g_cwConfigs.get()[0]);
				uint32_t ticks = DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM);
				DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_EN;
				advanceToTick(TotalTickCnt - ticks);
				needUG = 1;
			}
		} else {
			if (DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) == (uint32_t) (&g_cwConfigs.get()[0])) {
				// we need to change the direction of movement
				while (0 != (DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) & DMA_CR_EN)) {
					DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_EN;
				}
				DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) (&g_ccwConfigs.get()[0]);
				uint32_t ticks = DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM);
				DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_EN;
				advanceToTick(TotalTickCnt - ticks);
				needUG = 1;
			}
		}

		TIM_DIER(DMA_TRIGGER_TIMER) = 0;
		if (needUG) {
			TIM_EGR(DMA_TRIGGER_TIMER) = TIM_EGR_UG;
		}
		TIM_DIER(DMA_TRIGGER_TIMER) = TIM_DIER_UDE;
		if (mHZ) {
			TIM_CR1(DMA_TRIGGER_TIMER) |= TIM_CR1_CEN;
		} else {
			TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
			manualUpdateCCRs();
		}
		mCurSpeed = mHZ;
	}

	void setBufferConfig(uint8_t stepsPerPhase, RemapType const& remaps) {
		setEnabled(false);
		mRemaps = remaps;
		setupPWMConfigs(stepsPerPhase);
	}

	void init(unsigned int) override {
		initPWMTimer();
		initPins();
		initDMA();

		speedCallback.mDriver        = this;
		bufferConfigCallback.mDriver = this;
		enableCallback.mDriver       = this;
		phaseCallback.mDriver        = this;
		runIntervalCallback.mDriver  = this;
		bufferConfigCallback.callback();
		setEnabled(false);
	}

	RemapType mRemaps {0, 1, 2, 3};

	flawless::ApplicationConfig<CommutatinPattern> g_cwConfigs  {"cwConfigs"};
	flawless::ApplicationConfig<CommutatinPattern> g_ccwConfigs {"ccwConfigs"};

	int mCurSpeed {0};
	uint32_t mLastSetSNDTR {0};
	uint8_t mStepsPerPhase {0};

	struct : flawless::Callback<void> {
		MotorDriver* mDriver;
		void callback() override {
			mDriver->setSpeed(speedConfig);
		}
		flawless::ApplicationConfig<int> speedConfig{"speed", this, 0};
	} speedCallback;

	struct : flawless::Callback<void> {
		MotorDriver* mDriver;
		struct __attribute__((packed)) RunTickParams {
			uint32_t start;
			uint32_t count;
			uint32_t speed;
			bool cyclic;
		};
		void callback() override {
			mDriver->runTickIntervall(tickConfig->start, tickConfig->count, tickConfig->speed, tickConfig->cyclic);
		}
		flawless::ApplicationConfig<RunTickParams> tickConfig{"runTicks", this};
	} runIntervalCallback;

	struct : flawless::Callback<void> {
		MotorDriver* mDriver;
		void callback() override {
			mDriver->setBufferConfig(bufferConfig->stepsPerPhase, bufferConfig->remaps);
		}
		struct BufferConfig {
			uint8_t stepsPerPhase;
			RemapType remaps;
		};
		flawless::ApplicationConfig<BufferConfig> bufferConfig{"remaps", this, {3, 0, 1, 2, 3}};
	} bufferConfigCallback;

	struct : flawless::Callback<void> {
		MotorDriver* mDriver;
		void callback() override {
			mDriver->setEnabled(enableConfig);
		}
		flawless::ApplicationConfig<bool> enableConfig{"enable", this, false};
	} enableCallback;
	
	struct : flawless::Callback<void> {
		MotorDriver* mDriver;
		void callback() override {
			mDriver->runTickIntervall(phase, 1, 0, false);
		}
		flawless::ApplicationConfig<uint16_t> phase{"phase", this, false};
	} phaseCallback;
};

static MotorDriver motorDriver(9);

constexpr uint32_t HALL_PORT = GPIOA;
constexpr uint32_t HALL_U    = GPIO10;
constexpr uint32_t HALL_V    = GPIO9;
constexpr uint32_t HALL_W    = GPIO8;
constexpr uint32_t HALL_PINS = (HALL_U | HALL_V | HALL_W);
constexpr uint32_t HALL_TIMER = TIM1;



struct HallModule : public flawless::Module, public flawless::Callback<void>
{
	HallModule(unsigned int level) : flawless::Module(level) {}

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

		// setup the lookup tables
		uint8_t step0 = 2; // at phase 0 we read a hall feedback of 2
		for (int i=0; i < 6; ++i) {
			mCWHallIndexes.get()[step0]  = myModulo(-50 + 100 * i, StepsCount);
			step0 = getNextStep(step0, true);
		}
		for (int i=0; i < 6; ++i) {
			mCCWHallIndexes.get()[step0] = myModulo(50 + 100 * i, StepsCount);
			step0 = getNextStep(step0, false);
		}
	}

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

	void stepUpdate() {
		uint64_t speed = 2000000 * 3000 / mLastHallDelay;
		speed = std::min(speed, uint64_t(std::abs(mRunWithTargetSpeed)));
		if (mRunWithTargetSpeed > 0) {
			uint32_t start = mCWHallIndexes.get()[mHallVWUStates];
			uint32_t target = start + mAdvance;
			motorDriver.runTickIntervall(target, StepsCount / 12, speed, false);
		} else if (mRunWithTargetSpeed < 0) {
			int32_t start = mCWHallIndexes.get()[mHallVWUStates];
			int32_t target = start - mAdvance;
			motorDriver.runTickIntervall(target, StepsCount / 12, -speed, false);
		}
	}

	void notifyHallTick(uint64_t delay) {
		mHallVWUStates = (gpio_port_read(HALL_PORT) >> 8) & 0x7;
		mHallDelays[mMeasureHallIndex] = mLastHallDelay = delay;
		++mMeasureHallIndex;
		if (mMeasureHallIndex >= mHallDelays.size()) {
			mMeasureHallIndex = 0;
		}
		mHallDelay = 0;
		for (auto const& d : mHallDelays) {
			mHallDelay += d;
		}

		if (mRunWithTargetSpeed != 0) {
			stepUpdate();
		}
		if (mLastHallStates != mHallVWUStates and mLastLastHallStates != mHallVWUStates) {
			 if (not mRunWithTargetSpeed and mSetMeasureHalls and
				mLastHallStates and mLastLastHallStates) {
				uint32_t curStep = motorDriver.getCurStep();
				if (mSetMeasureHalls > 0) {
					mCWHallIndexes.get()[mHallVWUStates] += curStep;
				} else {
					mCCWHallIndexes.get()[mHallVWUStates] += curStep;
				}
				++mTickCounts[mHallVWUStates];
			}
		}
		mLastLastHallStates = mLastHallStates;
		mLastHallStates = mHallVWUStates;
	}

	void callback() override {
		if (mRunWithTargetSpeed != 0) {
			mHallVWUStates = (gpio_port_read(HALL_PORT) >> 8) & 0x7;
			stepUpdate();
		} else {
			if (mSetMeasureHalls) {
				for (auto & val : mTickCounts) {
					val = 0;
				}
				if (mSetMeasureHalls > 0) {
					for (auto & val : mCWHallIndexes.get()) {
						val = 0;
					}
				} else {
					for (auto & val : mCCWHallIndexes.get()) {
						val = 0;
					}
				}
				motorDriver.setEnabled(true);
				motorDriver.runTickIntervall(0, StepsCount, mSetMeasureHalls, true);
			} else {
				motorDriver.setEnabled(false);
				if (mLastSetMeasurementHalls > 0) {
					for (size_t i(0); i < mTickCounts.size(); ++i) {
						mCWHallIndexes.get()[i] = mCWHallIndexes.get()[i] / mTickCounts[i];
					}
				} else if (mLastSetMeasurementHalls < 0) {
					for (size_t i(0); i < mTickCounts.size(); ++i) {
						mCCWHallIndexes.get()[i] = mCCWHallIndexes.get()[i] / mTickCounts[i];
					}
				}
			}
			mLastSetMeasurementHalls = mSetMeasureHalls;
		}
	}

	size_t mMeasureHallIndex {0};
	Array<uint64_t, 6> mHallDelays;
	uint64_t mLastHallDelay {0};
	int mLastSetMeasurementHalls {0};
	uint8_t mLastHallStates {0};
	uint8_t mLastLastHallStates {0};
	Array<uint16_t, 8> mTickCounts;
	flawless::ApplicationConfig<Array<uint16_t, 8>> mCWHallIndexes{"CWHallIndexes"};
	flawless::ApplicationConfig<Array<uint16_t, 8>> mCCWHallIndexes{"CCWHallIndexes"};
	flawless::ApplicationConfig<uint8_t> mHallVWUStates{"hallstates"};
	flawless::ApplicationConfig<uint64_t> mHallDelay{"HallDelay"};

	flawless::ApplicationConfig<int> mSetMeasureHalls{"measureHallIndexes", this};
	flawless::ApplicationConfig<int> mRunWithTargetSpeed{"targetSpeed", this};
	flawless::ApplicationConfig<int> mAdvance{"advance", {100}};
};

static HallModule  hallModule(9);

extern "C" {
static volatile uint64_t delayBetweenTicks;
void tim1_up_tim10_isr() {
	if (TIM_SR(HALL_TIMER) & TIM_SR_UIF) {
		flawless::LockGuard lock;
		TIM_SR(HALL_TIMER) &= ~TIM_SR_UIF;
		delayBetweenTicks += TIM_ARR(HALL_TIMER);
		hallModule.notifyHallTick(delayBetweenTicks);
	}
}

void tim1_cc_isr()
{
	TIM_EGR(HALL_TIMER) = TIM_EGR_UG;
	flawless::LockGuard lock;
	delayBetweenTicks += TIM_CCR1(HALL_TIMER);

	hallModule.notifyHallTick(delayBetweenTicks);

	delayBetweenTicks = 0;
	TIM_SR(HALL_TIMER) = 0;
}

}
