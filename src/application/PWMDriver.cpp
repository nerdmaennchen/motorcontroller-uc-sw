#include "PWMDriver.h"

#include <flawless/stdtypes.h>
#include <flawless/module/Module.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/core/Listener.h>
#include <flawless/util/Array.h>
#include <flawless/applicationConfig/ApplicationConfig.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/spi.h>

#include <target/stm32f4/clock.h>
#include <interfaces/ISRTime.h>
#include <interfaces/gpio_interrupts.h>

#include <algorithm>

#define PWM_PORTA GPIOA
#define PWM_PORTB GPIOB
#define PWM_PIN_0_P GPIO8
#define PWM_PIN_1_P GPIO9
#define PWM_PIN_2_P GPIO10
#define PWM_PIN_0_N GPIO13
#define PWM_PIN_1_N GPIO14
#define PWM_PIN_2_N GPIO1

#define PWM_ENABLE_PORT GPIOB
#define PWM_ENABLE_PIN  GPIO12

#define PWM_PINS_A (PWM_PIN_0_P | PWM_PIN_1_P | PWM_PIN_2_P)
#define PWM_PINS_B (PWM_PIN_0_N | PWM_PIN_1_N | PWM_PIN_2_N)

#define CCR_DMA DMA2
#define CCR_DMA_STREAM DMA_STREAM_4
#define CCR_DMA_CHANNEL 6

#define DMA_TRIGGER_TIMER TIM3

#define DRV_NOCTW_PIN  GPIO13
#define DRV_NFAULT_PIN GPIO14
#define DRV_FAULT_PORT GPIOC

#define DRV_SPI_PORT     GPIOB
#define DRV_SPI_SCK_PIN  GPIO3
#define DRV_SPI_MISO_PIN GPIO4
#define DRV_SPI_MOSI_PIN GPIO5
#define DRV_SPI_PINS (DRV_SPI_SCK_PIN | DRV_SPI_MISO_PIN | DRV_SPI_MOSI_PIN)

#define DRV_SPI_NSS_PORT GPIOC
#define DRV_SPI_NSS_PIN  GPIO15

#define DRV_SPI SPI1


using namespace pwmdriver;

using RemapType = Array<uint8_t, 4>;
constexpr size_t commutationPatternBufferSize = 4 * StepsCount;
using CommmutationPatternContainer = Array<CommutationPattern, commutationPatternBufferSize>;

namespace {
flawless::ApplicationConfig<CommmutationPatternContainer> gCommutationPattern {"commutation_pattern", "9600H"};
uint32_t gLastSetSNDTR;
DriverInterface* gCurrentInterface;
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

	DMA_HIFCR(CCR_DMA) = 0x3d << 0;

	DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) &(gCommutationPattern.get()[stepStart]);
	DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) = gLastSetSNDTR = stepCnt * TicksPerStep;
	if (cycle) {
		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_CIRC;
	} else {
		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_CIRC;
	}
	TIM_SR(PWM_TIMER);
	DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) |= DMA_CR_EN;
	set_mHZ(mHZ);
}

void Driver::set_mHZ(uint32_t mHZ) {
	if (mHZ) {
		const uint64_t delayTicks = (CLOCK_APB1_TIMER_CLK * 1000) / mHZ;
		uint16_t psc = (delayTicks >> 16) & 0xffff;
		uint16_t arr = delayTicks / (psc + 1);
		TIM_PSC(DMA_TRIGGER_TIMER)  = psc;
		TIM_ARR(DMA_TRIGGER_TIMER)  = arr;
		TIM_CNT(DMA_TRIGGER_TIMER)  = 0;
		TIM_CR1(DMA_TRIGGER_TIMER) |= TIM_CR1_CEN;
	} else {
		TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
		TIM_SR(DMA_TRIGGER_TIMER) = 0;
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
		TIM_CR1(PWM_TIMER)  |= TIM_CR1_CEN;
		TIM_BDTR(PWM_TIMER) |= TIM_BDTR_MOE;
		TIM_EGR(PWM_TIMER) = TIM_EGR_COMG | TIM_EGR_UG;
	} else {
		TIM_BDTR(PWM_TIMER) &= ~TIM_BDTR_MOE;
//		TIM_CR1(PWM_TIMER)  &= ~TIM_CR1_CEN;
		TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;

		TIM_CCR1(PWM_TIMER) = 0;
		TIM_CCR2(PWM_TIMER) = 0;
		TIM_CCR3(PWM_TIMER) = 0;
		TIM_CCR4(PWM_TIMER) = 0;
		TIM_EGR(PWM_TIMER) = TIM_EGR_COMG | TIM_EGR_UG;
	}
}

void Driver::claim(DriverInterface* interface) {
	gCurrentInterface = interface;
}


namespace {
struct : public flawless::Callback<uint32_t&, bool> {
	void callback(uint32_t& step, bool set) override {
		if (not set) {
			step = pwmdriver::Driver::get().getCurStep();
		}
	}
	flawless::ApplicationConfig<uint32_t> gCurStepConfig{"commutation_pattern.cur_step", "I", this};
} curStepHelper;

struct : public flawless::Callback<Array<uint16_t, 4>&, bool> {
	void callback(Array<uint16_t, 4>& output, bool set) override {
		if (not set) {
			output[0] = TIM_CCR1(PWM_TIMER);
			output[1] = TIM_CCR2(PWM_TIMER);
			output[2] = TIM_CCR3(PWM_TIMER);
			output[3] = TIM_CCR4(PWM_TIMER);
		}
	}
	flawless::ApplicationConfig<Array<uint16_t, 4>> gCurStepConfig{"pwm.current_output", "4H", this};
} curOutputHelper;

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
	flawless::ApplicationConfig<ManualStepsParams> mStep {"commutation_pattern.manual_steps", "IIIb", this};
} manualStepsHelper;

struct : public flawless::Callback<Array<uint16_t, TicksPerStep>&, bool> {
	void callback(Array<uint16_t, TicksPerStep>& manPWMs, bool set) override {
		if (set) {
			TIM_CR1(DMA_TRIGGER_TIMER) &= ~TIM_CR1_CEN;
			TIM_CR1(PWM_TIMER) |= TIM_CR1_CEN;
			TIM_CCR1(PWM_TIMER) = manPWMs[0];
			TIM_CCR2(PWM_TIMER) = manPWMs[1];
			TIM_CCR3(PWM_TIMER) = manPWMs[2];
			TIM_CCR4(PWM_TIMER) = manPWMs[3];
		}
	}
	flawless::ApplicationConfig<Array<uint16_t, TicksPerStep>> mManualPWM {"pwm.manual", "4H", this};
} manualPWMHelper;

struct : public flawless::Callback<bool&, bool> {
	void callback(bool& enable, bool set) override {
		if (set) {
			pwmdriver::Driver::get().setEnabled(enable);
		}
	}
	flawless::ApplicationConfig<bool> mEnable {"pwm.enable", "B", this};
} enableHelper;

struct : public flawless::Callback<bool&, bool> {
	void callback(bool& enable, bool set) override {
		if (set) {
			if (enable) {
				gpio_set(PWM_ENABLE_PORT, PWM_ENABLE_PIN);
			} else {
				gpio_clear(PWM_ENABLE_PORT, PWM_ENABLE_PIN);
			}
		}
	}
	flawless::ApplicationConfig<bool> mEnable {"pwm.drv.enable", "B", this};
} drvEnableHelper;


struct : public flawless::Callback<Array<uint16_t, 2>&, bool> {
	void callback(Array<uint16_t, 2>& rxtx, bool set) override {
		while (SPI_SR(DRV_SPI) & SPI_SR_RXNE) {
			(void)SPI_DR(DRV_SPI);
		}
		while (not (SPI_SR(DRV_SPI) & SPI_SR_TXE));
		gpio_clear(DRV_SPI_NSS_PORT, DRV_SPI_NSS_PIN);
		if (set) {
			uint16_t oWord = ((rxtx[0]&0xf) << 11);
			oWord |= rxtx[1] & ((1<<10)-1);
			// send first byte
			SPI_DR(DRV_SPI) = oWord;
			while (SPI_SR(DRV_SPI) & SPI_SR_BSY);
			while (not (SPI_SR(DRV_SPI) & SPI_SR_RXNE));
			(void)SPI_DR(DRV_SPI);
		} else {
			uint16_t oWord = 1 << 15 | ((rxtx[0]&0xf) << 11);
			SPI_DR(DRV_SPI) = oWord;
			while (SPI_SR(DRV_SPI) & SPI_SR_BSY);
			while (not (SPI_SR(DRV_SPI) & SPI_SR_RXNE));
			gpio_set(DRV_SPI_NSS_PORT, DRV_SPI_NSS_PIN);
			rxtx[1] = SPI_DR(DRV_SPI);

			SystemTime::get().sleep(1);

			gpio_clear(DRV_SPI_NSS_PORT, DRV_SPI_NSS_PIN);
			while (not (SPI_SR(DRV_SPI) & SPI_SR_TXE));
			SPI_DR(DRV_SPI) = oWord;
			while (SPI_SR(DRV_SPI) & SPI_SR_BSY);
			while (not (SPI_SR(DRV_SPI) & SPI_SR_RXNE));
			rxtx[1] = SPI_DR(DRV_SPI);
		}
		gpio_set(DRV_SPI_NSS_PORT, DRV_SPI_NSS_PIN);
	}
	flawless::ApplicationConfig<Array<uint16_t, 2>> mSpiData {"pwm.drv.read", "2H", this};
} spiHelper;

struct InitHelper : public flawless::Module, public flawless::Callback<uint8_t &, bool>{
	InitHelper(unsigned int level) : flawless::Module(level) {}

	void initPins() {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		gpio_mode_setup(PWM_PORTA, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_PINS_A);
		gpio_mode_setup(PWM_PORTB, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_PINS_B);
		gpio_mode_setup(PWM_ENABLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWM_ENABLE_PIN);
		gpio_mode_setup(DRV_FAULT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, DRV_NFAULT_PIN | DRV_NOCTW_PIN);

//		gpio_registerFor_interrupt(nullptr, DRV_FAULT_PORT, DRV_NFAULT_PIN, GPIO_TRIGGER_LEVEL_FALLING, nullptr);
//		gpio_registerFor_interrupt(nullptr, DRV_FAULT_PORT, DRV_NOCTW_PIN, GPIO_TRIGGER_LEVEL_FALLING, nullptr);

		gpio_set_output_options(PWM_PORTA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PWM_PINS_A);
		gpio_set_output_options(PWM_PORTB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PWM_PINS_B);
		gpio_set_output_options(PWM_ENABLE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, PWM_ENABLE_PIN);

		gpio_set(PWM_ENABLE_PORT, PWM_ENABLE_PIN); // this can be always on
		gpio_set_af(PWM_PORTA, GPIO_AF1, PWM_PINS_A);
		gpio_set_af(PWM_PORTB, GPIO_AF1, PWM_PINS_B);
	}


	flawless::ApplicationConfig<uint8_t> pwm_deadTime{"pwm.dead_time", "B", this, 11};
	void callback(uint8_t &dtVal, bool setter) {
		if (setter) {
			TIM_BDTR(PWM_TIMER) = (TIM_BDTR(PWM_TIMER) & ~0x7f) | (dtVal & 0x7f);
		}
	}

	void initPWMTimer() {
		RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;

		TIM_CR1(PWM_TIMER) = TIM_CR1_CEN;
//		TIM_CR1(PWM_TIMER) = 0;
		TIM_SMCR(PWM_TIMER) |= TIM_SMCR_TS_ITR2; // set timer 1 as slave to timer 3 (this should generate com events on updates of tim3)

		TIM_CR2(PWM_TIMER)   = TIM_CR2_CCUS | TIM_CR2_CCPC | TIM_CR2_MMS_UPDATE;

		TIM_CCMR1(PWM_TIMER) = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
		TIM_CCMR2(PWM_TIMER) = TIM_CCMR2_OC3M_PWM1 | TIM_CCMR2_OC3PE;
		TIM_CCER(PWM_TIMER)  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E
				 | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;

		TIM_BDTR(PWM_TIMER)  = (pwm_deadTime & 0x7f);

		TIM_PSC(PWM_TIMER)   = 0;
		TIM_ARR(PWM_TIMER)   = pwmdriver::PwmMinCyclePeriod;
		TIM_CNT(PWM_TIMER)   = 0;

		TIM_CCR1(PWM_TIMER) = 0;
		TIM_CCR2(PWM_TIMER) = 0;
		TIM_CCR3(PWM_TIMER) = 0;
		TIM_CCR4(PWM_TIMER) = 0;


		TIM_SR(PWM_TIMER)   = 0;
		TIM_EGR(PWM_TIMER)  = TIM_EGR_UG;
		TIM_SR(PWM_TIMER)   = 0;
	}

	void initDMA()
	{
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;
		RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;

		TIM_CR1(DMA_TRIGGER_TIMER)  = 0;
		TIM_CR2(DMA_TRIGGER_TIMER)  = TIM_CR2_MMS_UPDATE;
		TIM_ARR(DMA_TRIGGER_TIMER)  = 0;

		TIM_EGR(DMA_TRIGGER_TIMER)  = TIM_EGR_UG;

		TIM_DIER(PWM_TIMER)  = TIM_DIER_TDE;
		TIM_DCR(PWM_TIMER)   = (3 << 8) | 0xd;
		TIM_SMCR(PWM_TIMER) |= TIM_SMCR_SMS_TM;

		do {
			DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) &= ~DMA_CR_EN;
		} while (0 != (DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) & DMA_CR_EN));

		DMA_SFCR(CCR_DMA, CCR_DMA_STREAM) = DMA_FCR_DMDIS | DMA_FCR_FTH_50;
		DMA_SCCR(CCR_DMA, CCR_DMA_STREAM) =
							(CCR_DMA_CHANNEL << DMA_CR_CHSEL_LSB) |
							DMA_CR_MBURST_INCR4 | DMA_CR_PBURST_INCR4 |
							DMA_CR_MSIZE_HALFWORD | DMA_CR_PSIZE_HALFWORD
							| DMA_CR_DIR
							| DMA_CR_MINC
							;
		DMA_SPAR(CCR_DMA, CCR_DMA_STREAM)  = (uint32_t) &(TIM_DMAR(PWM_TIMER));
		DMA_SM0AR(CCR_DMA, CCR_DMA_STREAM) = (uint32_t) gCommutationPattern->data();
		gLastSetSNDTR = DMA_SNDTR(CCR_DMA, CCR_DMA_STREAM) = 1;
	}

	void initSPI()
	{
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
		RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;

		gpio_mode_setup(DRV_SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, DRV_SPI_PINS);
		gpio_set_output_options(DRV_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, DRV_SPI_MOSI_PIN | DRV_SPI_SCK_PIN);
		gpio_set_af(DRV_SPI_PORT, GPIO_AF5, DRV_SPI_PINS);

		gpio_mode_setup(DRV_SPI_NSS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DRV_SPI_NSS_PIN);
		gpio_set_output_options(DRV_SPI_NSS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, DRV_SPI_NSS_PIN);

		gpio_set(DRV_SPI_NSS_PORT, DRV_SPI_NSS_PIN);

		SPI_CR2(DRV_SPI) = 0;
		SPI_CR1(DRV_SPI) = SPI_CR1_DFF;
		SPI_CR1(DRV_SPI) |=
				SPI_CR1_SSM
				| SPI_CR1_SSI
				| SPI_CR1_MSBFIRST
				| SPI_CR1_BAUDRATE_FPCLK_DIV_32
				| SPI_CR1_MSTR
				| SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE
				| SPI_CR1_CPHA;
		SPI_CR1(DRV_SPI) |= SPI_CR1_SPE;
	}

	void init(unsigned int) override {
		initPins();
		initPWMTimer();
		initDMA();
		initSPI();
	}
} initHelper(5);
}

