/*------------------------includes---------------------------------*/
#include <flawless/stdtypes.h>

#include <flawless/module/Module.h>

#include <flawless/core/Message.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>

#include <flawless/core/Listener.h>
#include <flawless/timer/swTimer.h>

#include <flawless/applicationConfig/ApplicationConfig.h>

#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>

#define LED_PORT GPIOC
#define LED_PIN GPIO13

namespace
{
flawless::MessageBufferMemory<int, 5> intMsgBuf;
flawless::MessageBufferMemory<int, 1> intMsgBuf2;

class TestModule : public flawless::Module
{
	class : public flawless::Listener<int, 1> {
		void callback(flawless::Message<int> const&) override {
			gpio_set(LED_PORT, LED_PIN);
		}
	} mListener1;
	class : public flawless::Listener<int, 2> {
		void callback(flawless::Message<int> const&) override {
			gpio_clear(LED_PORT, LED_PIN);
		}
	} mListener2;

	class : public flawless::TimerCallback {
		class : public flawless::TimerCallback {
			void callback() override {
				auto msg = flawless::MessageBufferManager<int>::get().getFreeMessage();
				if (msg) {
					msg.post<2>();
				}
			}
		} mOffTimer;

		void callback() override {
			auto msg = flawless::MessageBufferManager<int>::get().getFreeMessage();
			if (msg) {
				msg.post<1>();
				mOffTimer.start(500000, false);
			}
		}
	} mOnTimer;

	flawless::Message<int> mLastMsg;

public:
	TestModule(unsigned int level) : flawless::Module(level) {}
	virtual ~TestModule() {};

	void init(unsigned int) override {
		mOnTimer.start(1000000, true);

		RCC_AHB1ENR |= RCC_AHB1ENR_IOPCEN;
		gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
		gpio_set_output_options(LED_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, LED_PIN);
	}
};

TestModule testModule(9);

}


