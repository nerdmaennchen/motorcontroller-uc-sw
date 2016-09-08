/*------------------------includes---------------------------------*/
#include <flawless/stdtypes.h>
#include <flawless/module/Module.h>

#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>

#include <flawless/core/Listener.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/dma.h>

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

#define PWM_FREQUENCY_HZ 20000


#define CCR_DMA DMA1
#define CCR_DMA_STREAM 2
#define CCR_DMA_CHANNEL 5

#define DMA_TRIGGER_TIMER TIM3

#define PWM_TIMER TIM4

constexpr uint32_t PWM_AMPLITUDE = 1024;
constexpr uint32_t MIN_PWM_VAL   = PWM_AMPLITUDE;

#define PWM_BUFFER_STEP_SIZE 512

#define M_PI 3.14159265359f

class MotorDriver : public flawless::Module, flawless::Listener<MotorCurrentMeasure, 3>, flawless::Listener<MaxMotorCurrent, 3>
{
	using CommutatinPattern = Array<Array<uint16_t, 4>, PWM_BUFFER_STEP_SIZE>;
	using RemapType = Array<uint8_t, 4>;
public:
	MotorDriver(unsigned int level) : flawless::Module(level) {}
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

		TIM_CR1(PWM_TIMER)   = TIM_CR1_ARPE | TIM_CR1_CMS_CENTER_3;

		TIM_CCMR1(PWM_TIMER) = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
		TIM_CCMR2(PWM_TIMER) = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC4M_PWM1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
		TIM_CCER(PWM_TIMER) |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

		TIM_PSC(PWM_TIMER)   = (uint16_t) (CLOCK_APB1_TIMER_CLK / (PWM_FREQUENCY_HZ * MIN_PWM_VAL * 2)) - 1;
		TIM_ARR(PWM_TIMER)   = 0;//(MIN_PWM_VAL);
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

	class DummyTimer : public flawless::TimerCallback {
	public:
		DummyTimer(flawless::timerInterval_t interval, bool repeating) : flawless::TimerCallback(interval, repeating) {}
		void callback() override {
		}
	} mOnTimer{500000, true};

	void initDMA()
	{
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;

		TIM_CR1(DMA_TRIGGER_TIMER)  = TIM_CR1_ARPE | (TIM_CR1(DMA_TRIGGER_TIMER) & ~0x03ff);
		TIM_DIER(DMA_TRIGGER_TIMER) &= ~TIM_DIER_UDE;
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
		DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) (&(g_cwConfigs.get())[0]);

		DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) = g_cwConfigs.get().size() * g_cwConfigs.get()[0].size();

		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_EN;
	}

	flawless::ApplicationConfig<MaxMotorCurrent> mMotorCurrentMean{"motorCurrentMean"};
	flawless::ApplicationConfig<MaxMotorCurrent> mMaxCurrent{"maxCurrent", 0.1f};
	flawless::ApplicationConfig<MaxMotorCurrent> mCurrentP{"currentP", 0.9f};
	void callback(flawless::Message<MaxMotorCurrent> const& motorCurrent) override {
		mMotorCurrentMean = *motorCurrent * float(PWM_AMPLITUDE) / float(TIM_ARR(PWM_TIMER));
		float error = (1.f + (mMotorCurrentMean - mMaxCurrent) * mCurrentP);
		uint32_t targetAmplitude = TIM_ARR(PWM_TIMER) * error;
		targetAmplitude = std::max(MIN_PWM_VAL, std::min(uint32_t(0xffff), targetAmplitude));

//		uint16_t psc = (uint16_t) (CLOCK_APB1_TIMER_CLK / (PWM_FREQUENCY_HZ * targetAmplitude)) ;
//		if (psc != 0) {
//			psc -= 1;
//		}
		TIM_PSC(PWM_TIMER)   = 0;//psc;
		TIM_ARR(PWM_TIMER)   = targetAmplitude;
	}

	void callback(flawless::Message<MotorCurrentMeasure> const& motorCurrent) override {
		(void) motorCurrent;
		for (uint32_t i = 0; i < (*motorCurrent).vals.size(); ++i) {
			mMotorCurrent->vals[i] = (*motorCurrent).vals[i];
		}
	}

	flawless::ApplicationConfig<MotorCurrentMeasure> mMotorCurrent{"motorCurrent"};

	void setupPWMConfigs(uint8_t stepsPerPhase) {
		mStepsPerPhase = stepsPerPhase;
		double phaseOffset = 2. * M_PI / float(stepsPerPhase);
		const std::size_t cnt = g_cwConfigs.get().size();
		for (std::size_t sIdx = 0; sIdx < cnt; ++sIdx) {
			float phase0 = (float)sIdx / (float)(cnt) * M_PI * 2;
			for (size_t i = 0; i < g_cwConfigs.get()[0].size(); ++i) {
				float phase = phase0 + phaseOffset * i;
				const float s = std::sin(phase) * .5;
				const float intermediate = s * PWM_AMPLITUDE + PWM_AMPLITUDE / 2;
				const uint16_t sVal = roundf(intermediate);
				g_cwConfigs.get()[sIdx][mRemaps[i]] = sVal;
				g_ccwConfigs.get()[cnt - 1 - sIdx][mRemaps[i]] = sVal;
			}
		}
	}

	void doSomeTicks(uint32_t tickCnt) {
		TIM_DIER(DMA_TRIGGER_TIMER) = TIM_DIER_UDE;
		const int transferCount = g_cwConfigs.get().size() * g_cwConfigs.get()[0].size();
		uint32_t targetCnt = (DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) + tickCnt) % transferCount;
		TIM_CR1(PWM_TIMER) |= TIM_CR1_UDIS;
		while ((DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) % transferCount) != targetCnt) {
			TIM_EGR(DMA_TRIGGER_TIMER) = TIM_EGR_UG;
			for (volatile int i(0); i < 10; ++i);
		}
		TIM_CR1(PWM_TIMER) &= ~TIM_CR1_UDIS;
	}

	void setEnabled(bool enabled) {
		if (enabled) {
			gpio_set(PWM_PORT, PWM_ENABLE_PINS);
			TIM_CR1(PWM_TIMER) |= TIM_CR1_CEN;
			TIM_SMCR(PWM_TIMER) = TIM_SMCR_MSM;
			TIM_CR2(PWM_TIMER)  = TIM_CR2_MMS_UPDATE;
		} else {
			gpio_clear(PWM_PORT, PWM_ENABLE_PINS);
			setSpeed(0);
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
		const long unsigned int delayTicks = (CLOCK_APB1_TIMER_CLK * mStepsPerPhase * 1000) / (abs(mHZ) * g_cwConfigs.get().size());

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
				doSomeTicks(g_cwConfigs.get().size() * g_cwConfigs.get()[0].size() - ticks);
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
				doSomeTicks(g_cwConfigs.get().size() * g_cwConfigs.get()[0].size() - ticks);
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
			// write the current word manually into the CC registers
			uint32_t idx = g_cwConfigs.get().size() - DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) / g_cwConfigs.get()[0].size();
			Array<uint16_t, 4> const* arrVals = ((Array<uint16_t, 4> const*)DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM)) + idx;
			TIM_CCR1(PWM_TIMER) = (*arrVals)[0];
			TIM_CCR2(PWM_TIMER) = (*arrVals)[1];
			TIM_CCR3(PWM_TIMER) = (*arrVals)[2];
			TIM_CCR4(PWM_TIMER) = (*arrVals)[3];
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

		setEnabled(false);
		speedCallback.mDriver = this;
		bufferConfigCallback.mDriver = this;
		enableCallback.mDriver = this;

		bufferConfigCallback.callback();
	}

private:

	RemapType mRemaps {0, 1, 2, 3};

	flawless::ApplicationConfig<CommutatinPattern> g_cwConfigs  {"cwConfigs"};
	flawless::ApplicationConfig<CommutatinPattern> g_ccwConfigs {"ccwConfigs"};

	int mCurSpeed {0};
	uint8_t mStepsPerPhase {0};

	class : flawless::Callback<void> {
	public:
		MotorDriver* mDriver;
		void callback() override {
			mDriver->setSpeed(speedConfig);
		}
	private:
		flawless::ApplicationConfig<int> speedConfig{"speed", this, 0};
	} speedCallback;

	class : flawless::Callback<void> {
	public:
		MotorDriver* mDriver;
		void callback() override {
			mDriver->setBufferConfig(bufferConfig->stepsPerPhase, bufferConfig->remaps);
		}
	private:
		struct BufferConfig {
			uint8_t stepsPerPhase;
			RemapType remaps;
		};
		flawless::ApplicationConfig<BufferConfig> bufferConfig{"remaps", this, {4, 0, 1, 2, 3}};
	} bufferConfigCallback;

	class : flawless::Callback<void> {
	public:
		MotorDriver* mDriver;
		void callback() override {
			mDriver->setEnabled(enableConfig);
		}
	private:
		flawless::ApplicationConfig<bool> enableConfig{"enable", this, false};
	} enableCallback;
};

namespace {
static MotorDriver motorDriver(9);
}
