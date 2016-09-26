
#include "MotorCurrent.h"
#include "PWMDriver.h"
#include "interfaces/ISRTime.h"

#include <flawless/module/Module.h>

#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/applicationConfig/ApplicationConfig.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/nvic.h>

#include <flawless/timer/swTimer.h>
#include <flawless/util/Array.h>

#include "target/stm32f4/clock.h"

#include "interfaces/systemTime.h"

#include <algorithm>
#include <cmath>


#define SENSE_PORT GPIOC
#define SENSE_PIN_A GPIO0
#define SENSE_PIN_B GPIO1
#define SENSE_PINS (SENSE_PIN_A | SENSE_PIN_B)

#define SENSE_ADC ADC1
#define SENSE_DMA DMA2
#define SENSE_DMA_STREAM DMA_STREAM_4
#define SENSE_DMA_CHANNEL 0

#define AUX_TIMER TIM8

#define ADC_RESOLUTION ((1 << 12) - 1)

flawless::MessageBufferMemory<MotorCurrentMeasure, 5> currentMeasurements;
flawless::MessageBufferMemory<MotorCurrent, 5> currentMeanMeasurements;

constexpr float SHUNT_CONDUCTIVITY = 20;
#define MIN_ADC_DELAY_US 100000

class MotorCurrentMeasurer final
	: public flawless::Module
	, public flawless::Callback<uint16_t&, bool>
{
	using RawMeasurementType_t = MotorCurrentMeasure;
public:
	MotorCurrentMeasurer(unsigned int level) : flawless::Module(level) {}

	virtual ~MotorCurrentMeasurer() {}

//	void onDMADone() {
//		auto detailledMsg = flawless::MessageBufferManager<MotorCurrentMeasure>::get().getFreeMessage();
//		auto maxMsg = flawless::MessageBufferManager<MotorCurrent>::get().getFreeMessage();
//		if (detailledMsg) {
//			float max = 0;
//			Array<uint16_t, SMOOTHING_CNT> const* buffer = &(mRawMeasureBuffer2);
//			if (DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) & DMA_CR_CT) {
//				buffer = &(mRawMeasureBuffer1);
//			}
//			uint32_t cnt = 0;
//			for (size_t i(0); i < buffer->size(); ++i) {
//				float val = (float((*buffer)[i]) * 3.3f * SHUNT_CONDUCTIVITY / float(ADC_RESOLUTION));
//				if ((*buffer)[i] > 0) {
//					max += val;
//					++cnt;
//				}
//
//				(*detailledMsg).vals[i] = val;
//			}
//			if (cnt > 0) {
//				maxMsg = max / cnt;
//			} else {
//				maxMsg = 0.f;
//			}
//			maxMsg.invokeDirectly<0>();
//			detailledMsg.post<0>();
//		}
//	}

	void onDMADone() {
		auto meanMsg = flawless::MessageBufferManager<MotorCurrent>::get().getFreeMessage();
		if (meanMsg) {
			uint32_t mean = 0;
			Array<uint16_t, SMOOTHING_CNT> const* buffer = &(mRawMeasureBuffer2);
			if (DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) & DMA_CR_CT) {
				buffer = &(mRawMeasureBuffer1);
			}
			for (auto const& val : *buffer) {
				mean += val;
			}
			meanMsg = float(mean) * 3.3f * SHUNT_CONDUCTIVITY / float(ADC_RESOLUTION) / float(buffer->size());
			meanMsg.invokeDirectly<0>();
		}
	}

	void initGPIOs() {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
		RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
		gpio_mode_setup(SENSE_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, SENSE_PINS);
	}

	void initADC() {
		RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
		ADC_CR2(SENSE_ADC) = ADC_CR2_ADON;

		ADC_CR1(SENSE_ADC) = ADC_CR1_SCAN;
		ADC_CR2(SENSE_ADC) |= ADC_CR2_EXTEN_FALLING_EDGE | ADC_CR2_EXTSEL_TIM8_TRGO;
		ADC_CR2(SENSE_ADC) |= ADC_CR2_DDS | ADC_CR2_DMA;

		ADC_SQR1(SENSE_ADC) = (2 << ADC_SQR1_L_LSB);
		ADC_SQR1(SENSE_ADC) = 0;
		ADC_SQR2(SENSE_ADC) = 0;
		ADC_SQR3(SENSE_ADC) = 0;
		ADC_SQR3(SENSE_ADC) |= (10 << ADC_SQR3_SQ1_LSB);
		ADC_SQR3(SENSE_ADC) |= (11 << ADC_SQR3_SQ2_LSB);

//		ADC_JSQR(SENSE_ADC) = ADC_JSQR_JL_1CHANNELS | (14 << ADC_JSQR_JSQ1_LSB);

		ADC_SMPR1(SENSE_ADC) =
				(ADC_SMPR_SMP_3CYC << ADC_SMPR1_SMP10_LSB)
				| (ADC_SMPR_SMP_3CYC << ADC_SMPR1_SMP11_LSB);
		ADC_SMPR2(SENSE_ADC) = 0;
	}

	void initDMA() {
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;

		while (0 != (DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) & DMA_CR_EN)) {
			DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) &= ~DMA_CR_EN;
		}

		/*enable dma interrupt*/
		nvic_enable_irq(NVIC_DMA2_STREAM4_IRQ);

		DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) =
				(SENSE_DMA_CHANNEL << DMA_CR_CHSEL_LSB) |
				DMA_CR_PL_VERYHIGH |
				DMA_CR_DBM |
				DMA_CR_PSIZE_HALFWORD |
				DMA_CR_MSIZE_HALFWORD;

		if (mRawMeasureBuffer1.size() != 1) {
			DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) |= DMA_CR_MINC;
		}


		DMA_SPAR(SENSE_DMA, SENSE_DMA_STREAM)  = (uint32_t) (&(ADC_DR(SENSE_ADC)));
		DMA_SNDTR(SENSE_DMA, SENSE_DMA_STREAM) = mRawMeasureBuffer1.size();
		DMA_SM0AR(SENSE_DMA, SENSE_DMA_STREAM) = (uint32_t) (mRawMeasureBuffer1.data());
		DMA_SM1AR(SENSE_DMA, SENSE_DMA_STREAM) = (uint32_t) (mRawMeasureBuffer2.data());
		DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) |=  DMA_CR_TCIE;
		DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) |= DMA_CR_EN;
	}

	void initAuxTimer() {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
		gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO7);
		gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO7);
		gpio_set_af(GPIOC, GPIO_AF3, GPIO7);
		RCC_APB2ENR |= RCC_APB2ENR_TIM8EN;

		TIM_CR1(AUX_TIMER) = 0;
		TIM_CR2(AUX_TIMER) = 0;

		TIM_PSC(AUX_TIMER) = 1;
		TIM_ARR(AUX_TIMER) = 0xffff;
		TIM_CNT(AUX_TIMER) = 0;
		TIM_SMCR(AUX_TIMER) = TIM_SMCR_TS_ITR2 | TIM_SMCR_SMS_RM;

		TIM_CCMR1(AUX_TIMER) = TIM_CCMR1_OC2M_PWM1;
		TIM_CCER(AUX_TIMER) = TIM_CCER_CC2E;

		TIM_BDTR(AUX_TIMER) |= TIM_BDTR_MOE;
		TIM_CCR2(AUX_TIMER)  = enableConfig;

		TIM_CR2(AUX_TIMER) |= TIM_CR2_MMS_COMPARE_OC2REF;

		TIM_EGR(AUX_TIMER) = TIM_EGR_UG;
		TIM_CR1(AUX_TIMER) |= TIM_CR1_CEN;
		TIM_SR(AUX_TIMER) = 0;
	}

	void callback(uint16_t&, bool) override {
		TIM_BDTR(AUX_TIMER) |= TIM_BDTR_MOE;
		TIM_CCR2(AUX_TIMER)  = enableConfig;
	}
	flawless::ApplicationConfig<uint16_t> enableConfig{"motor_current_adc_delay", "H", this, (pwmdriver::PwmOffTimer+pwmdriver::PwmAmplitude)/2};


	void init(unsigned int) override {
		initGPIOs();
		initDMA();
		initAuxTimer();
		initADC();
	}

	class DummyTimer : public flawless::TimerCallback {
	public:
		DummyTimer(flawless::timerInterval_t interval, bool repeating) : flawless::TimerCallback(interval, repeating) {}
		void callback() override {
		}
	} mOnTimer{500000, true};

private:
	Array<uint16_t, SMOOTHING_CNT> mRawMeasureBuffer1;
	Array<uint16_t, SMOOTHING_CNT> mRawMeasureBuffer2;
} ;
namespace
{
MotorCurrentMeasurer motorCurrent(8);
}


extern "C" {
static systemTime_t lastISRTime;
static systemTime_t lastISRDelay;
void dma2_stream4_isr(void) {
	ISRTime isrTimer;
	lastISRDelay = SystemTime::get().getSystemTimeUS() - lastISRTime;
	lastISRTime = SystemTime::get().getSystemTimeUS();

	(void)TIM_CNT(AUX_TIMER);
	motorCurrent.onDMADone();
	DMA_HIFCR(SENSE_DMA) = 0x3d;
}
}
