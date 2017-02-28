
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


#define DBG_TIMER_PORT GPIOB
#define DBG_TIMER_PIN  GPIO9

#define SENSE_PORT_1  GPIOB
#define SENSE_PIN_C   GPIO0

#define SENSE_PORT_2  GPIOA
#define SENSE_PIN_V_1 GPIO7
#define SENSE_PIN_V_2 GPIO6
#define SENSE_PIN_V_3 GPIO5

#define SENSE_PINS_1 (SENSE_PIN_C)
#define SENSE_PINS_2 (SENSE_PIN_V_1 | SENSE_PIN_V_2 | SENSE_PIN_V_3)

#define SENSE_ADC ADC1
#define SENSE_DMA DMA2
#define SENSE_DMA_STREAM DMA_STREAM_0
#define SENSE_DMA_CHANNEL 0

#define SENSE_TRIGGER_TIMER TIM4

constexpr int numChannels = 5;
constexpr int ADC_Resolution = (1 << 12) - 1;

flawless::MessageBufferMemory<VoltageMeasure, 5> voltageMeanMeasurement;

constexpr int Averaging = 16;
constexpr float Measurement2Voltage = 3.3f / float(ADC_Resolution);
constexpr float ScaleFactor = Measurement2Voltage / float(Averaging);

class AnalogMeasurer final
	: public flawless::Module<5>
	, public flawless::Callback<uint16_t&, bool>
{
	using RawMeasurement  = Array<uint16_t, numChannels>;
	using RawMeasurements = Array<RawMeasurement, Averaging>;
	flawless::ApplicationConfig<systemTime_t> mISRDelay{"analog.isr_delay", "1Q"};
	flawless::ApplicationConfig<VoltageMeasure> mLastVoltageMeasure{"analog.measurement", "5f"};

	flawless::ApplicationConfig<RawMeasurements> mRawMeasureBuffer1{"analog.buffer1", "10H"};
	flawless::ApplicationConfig<RawMeasurements> mRawMeasureBuffer2{"analog.buffer2", "10H"};
public:
	systemTime_t mLastPublishTime;
	void onDMADone() {
		auto meanMsg = flawless::getFreeMessage<VoltageMeasure>();
		if (meanMsg) {
			systemTime_t now = SystemTime::get().getCurrentTime();
			RawMeasurements const* buffer = &(mRawMeasureBuffer2.get());
			if (DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) & DMA_CR_CT) {
				buffer = &(mRawMeasureBuffer1.get());
			}

//			 uncomment to use the real average
			Array<int, numChannels> sums = {0,0,0,0};
			for (auto const& val : *buffer) {
				for (int i(0); i < numChannels; ++i) {
					sums[i] += val[i];
				}
			}
			meanMsg->motorCurrent    = float(sums[0]) * ScaleFactor;
			meanMsg->voltage_phase_U = float(sums[1]) * ScaleFactor;
			meanMsg->voltage_phase_V = float(sums[2]) * ScaleFactor;
			meanMsg->voltage_phase_W = float(sums[3]) * ScaleFactor;
			meanMsg->voltage_VPP     = float(sums[4]) * ScaleFactor;

			// just use the last measurement
//			meanMsg->motorCurrent    = float((*buffer)[Averaging-1][0]) * Measurement2Voltage;
//			meanMsg->voltage_phase_U = float((*buffer)[Averaging-1][1]) * Measurement2Voltage;
//			meanMsg->voltage_phase_V = float((*buffer)[Averaging-1][2]) * Measurement2Voltage;
//			meanMsg->voltage_phase_W = float((*buffer)[Averaging-1][3]) * Measurement2Voltage;
//			meanMsg->voltage_VPP     = float((*buffer)[Averaging-1][4]) * Measurement2Voltage;
			mLastVoltageMeasure = meanMsg;
			mISRDelay = now - mLastPublishTime;
			mLastPublishTime = now;
			meanMsg.invokeDirectly();
		}
	}

	void initGPIOs() {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
		RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
		gpio_mode_setup(SENSE_PORT_1, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, SENSE_PINS_1);
		gpio_mode_setup(SENSE_PORT_2, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, SENSE_PINS_2);
	}

	void initADC() {
		RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;
		ADC_CR2(SENSE_ADC) = ADC_CR2_ADON;

		ADC_CR1(SENSE_ADC) = ADC_CR1_SCAN;
		ADC_CR2(SENSE_ADC) |= ADC_CR2_EXTEN_BOTH_EDGES | ADC_CR2_EXTSEL_TIM4_CC4;
		ADC_CR2(SENSE_ADC) |= ADC_CR2_DDS | ADC_CR2_DMA;

		ADC_SQR1(SENSE_ADC) = ((numChannels-1) << ADC_SQR1_L_LSB);
		ADC_SQR2(SENSE_ADC) = 0;
		ADC_SQR3(SENSE_ADC) =
				  (8 << ADC_SQR3_SQ1_LSB)
				| (7 << ADC_SQR3_SQ2_LSB)
				| (6 << ADC_SQR3_SQ3_LSB)
				| (5 << ADC_SQR3_SQ4_LSB)
				| (4 << ADC_SQR3_SQ5_LSB)
				;

		ADC_SMPR1(SENSE_ADC) = 0;
		ADC_SMPR2(SENSE_ADC) = 0;
	}

	void initDMA() {
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;

		while (0 != (DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) & DMA_CR_EN)) {
			DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) &= ~DMA_CR_EN;
		}

		/*enable dma interrupt*/
		nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);

		DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) =
				(SENSE_DMA_CHANNEL << DMA_CR_CHSEL_LSB) |
				DMA_CR_DBM |
				DMA_CR_PSIZE_HALFWORD |
				DMA_CR_MSIZE_HALFWORD;

		if (mRawMeasureBuffer1.get().size() != 1) {
			DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) |= DMA_CR_MINC;
		}


		DMA_SPAR(SENSE_DMA, SENSE_DMA_STREAM)  = (uint32_t) (&(ADC_DR(SENSE_ADC)));
		DMA_SNDTR(SENSE_DMA, SENSE_DMA_STREAM) = mRawMeasureBuffer1.get().size() * mRawMeasureBuffer1.get()[0].size();
		DMA_SM0AR(SENSE_DMA, SENSE_DMA_STREAM) = (uint32_t) (mRawMeasureBuffer1.get().data());
		DMA_SM1AR(SENSE_DMA, SENSE_DMA_STREAM) = (uint32_t) (mRawMeasureBuffer2.get().data());
		DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) |=  DMA_CR_TCIE;
		DMA_SCCR(SENSE_DMA, SENSE_DMA_STREAM) |= DMA_CR_EN;
	}

	void initAuxTimer() {
		RCC_APB1ENR |= RCC_APB1ENR_TIM4EN;

		TIM_CR1(SENSE_TRIGGER_TIMER) = 0;
		TIM_CR2(SENSE_TRIGGER_TIMER) = 0;

		TIM_PSC(SENSE_TRIGGER_TIMER) = 0;
		TIM_ARR(SENSE_TRIGGER_TIMER) = 0xffff;
		TIM_CNT(SENSE_TRIGGER_TIMER) = 0;
		TIM_SMCR(SENSE_TRIGGER_TIMER) = TIM_SMCR_TS_ITR0 | TIM_SMCR_SMS_RM;

		TIM_CCMR2(SENSE_TRIGGER_TIMER)= TIM_CCMR2_OC4M_TOGGLE;
		TIM_CCER(SENSE_TRIGGER_TIMER) = TIM_CCER_CC4E;

		TIM_CCR4(SENSE_TRIGGER_TIMER)  = enableConfig;

		TIM_EGR(SENSE_TRIGGER_TIMER) = TIM_EGR_UG;
		TIM_CR1(SENSE_TRIGGER_TIMER) |= TIM_CR1_CEN;
		TIM_SR(SENSE_TRIGGER_TIMER) = 0;
	}

	void callback(uint16_t&, bool) override {
		TIM_CCR4(SENSE_TRIGGER_TIMER)  = enableConfig;
	}
	flawless::ApplicationConfig<uint16_t> enableConfig{"analog.adc_delay", "H", this, 0};


	void init() override {
		initGPIOs();
		initDMA();
		initAuxTimer();
		initADC();

//		gpio_mode_setup(DBG_TIMER_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, DBG_TIMER_PIN);
//		gpio_set_af(DBG_TIMER_PORT, GPIO_AF2, DBG_TIMER_PIN);
	}
} analogMeasurement;


extern "C" {
void dma2_stream0_isr(void) {
	ISRTime isrTimer;
	analogMeasurement.onDMADone();
	DMA_LIFCR(SENSE_DMA) = 0x3d;
}
}
