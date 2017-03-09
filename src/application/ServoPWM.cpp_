
#include <flawless/stdtypes.h>
#include <flawless/module/Module.h>
#include <flawless/applicationConfig/ApplicationConfig.h>
#include <flawless/util/Array.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>

#include <target/stm32f4/clock.h>

namespace
{

constexpr uint32_t SERVO_PWM_PORT  = GPIOB;
constexpr uint32_t SERVO_PWM_PIN_1 = GPIO8;
constexpr uint32_t SERVO_PWM_PIN_2 = GPIO9;
constexpr uint32_t SERVO_PWM_PINS  = (SERVO_PWM_PIN_1 | SERVO_PWM_PIN_2);

constexpr uint32_t SERVO_PWM_TIMER_1 = TIM10;
constexpr uint32_t SERVO_PWM_TIMER_2 = TIM11;

constexpr uint32_t SERVO_FREQUENCY_HZ = 50;

struct InitHelper : public flawless::Module<5> {
	void initPins() {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPBEN;
		gpio_mode_setup(SERVO_PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SERVO_PWM_PINS);
		gpio_set_output_options(SERVO_PWM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, SERVO_PWM_PINS);
		gpio_set_af(SERVO_PWM_PORT, GPIO_AF3, SERVO_PWM_PINS);
	}

	void initPWMTimer() {
		RCC_APB2ENR |= RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN;

		TIM_CR1(SERVO_PWM_TIMER_1)   = TIM_CR1_ARPE;
		TIM_CCMR1(SERVO_PWM_TIMER_1) = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
		TIM_CCER(SERVO_PWM_TIMER_1) |= TIM_CCER_CC1E;

		TIM_PSC(SERVO_PWM_TIMER_1)   = (CLOCK_APB2_TIMER_CLK / 1000000)-1;
		TIM_ARR(SERVO_PWM_TIMER_1)   = 20000; // this generates a base frequency of 50Hz with 1us PWM accuracy

		TIM_CCR1(SERVO_PWM_TIMER_1) = 0;
		TIM_CCR2(SERVO_PWM_TIMER_1) = 0;

		TIM_SR(SERVO_PWM_TIMER_1)   = 0;
		TIM_EGR(SERVO_PWM_TIMER_1)  = TIM_EGR_UG;
		TIM_SR(SERVO_PWM_TIMER_1)   = 0;

		TIM_CR1(SERVO_PWM_TIMER_2)   = TIM_CR1_ARPE;
		TIM_CCMR1(SERVO_PWM_TIMER_2) = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
		TIM_CCER(SERVO_PWM_TIMER_2) |= TIM_CCER_CC1E;

		TIM_PSC(SERVO_PWM_TIMER_2)   = (CLOCK_APB2_TIMER_CLK / 1000000)-1;
		TIM_ARR(SERVO_PWM_TIMER_2)   = 20000; // this generates a base frequency of 50Hz with 1us PWM accuracy

		TIM_CCR1(SERVO_PWM_TIMER_2) = 0;
		TIM_CCR2(SERVO_PWM_TIMER_2) = 0;

		TIM_SR(SERVO_PWM_TIMER_2)   = 0;
		TIM_EGR(SERVO_PWM_TIMER_2)  = TIM_EGR_UG;
		TIM_SR(SERVO_PWM_TIMER_2)   = 0;
		TIM_CR1(SERVO_PWM_TIMER_1) |= TIM_CR1_CEN;
		TIM_CR1(SERVO_PWM_TIMER_2) |= TIM_CR1_CEN;
	}

	void init() override {
		initPWMTimer();
		initPins();
	}
} initHelper;

using ServoPWMConfig = Array<uint16_t, 2>;
struct : public flawless::Callback<ServoPWMConfig &, bool> {
	void callback(ServoPWMConfig & config, bool setter) {
		if (setter) {
			TIM_CCR1(SERVO_PWM_TIMER_1) = config[0];
			TIM_CCR1(SERVO_PWM_TIMER_2) = config[1];
		}
	}
	flawless::ApplicationConfig<Array<uint16_t, 2>> mPWMVAls{"servo.pwm_values", "2H", this};
} pwmConfigHelper;

}
