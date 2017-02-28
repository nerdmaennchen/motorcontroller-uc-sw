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

#define LED_PORT GPIOA
#define LED_PIN GPIO15

namespace
{
struct BlinkInstruction {
	flawless::timerInterval_t delay;
	bool ledState;
};

Array<BlinkInstruction, 4> heartBeatSequence {{
	{600000, true},
	{70000, false},
	{100000, true},
	{50000, false}
}};

flawless::MessageBufferMemory<int, 5> intMsgBuf2;

class TestModule : public flawless::Module<9>, public flawless::Listener<int, 1>, public flawless::TimerCallback
{
	flawless::Message<int> mLastMsg;

	void callback() override {
		auto msg = flawless::getFreeMessage<int>();
		if (msg) {
			msg = mLastMsg + 1;
			if (msg == int(heartBeatSequence.size())) {
				msg = 0;
			}
			msg.post<1>();
		} else {
			msg = 0;
		}
	}

	void callback(flawless::Message<int> const& msg) override {
		mLastMsg = msg;

		if (heartBeatSequence[int(mLastMsg)].ledState) {
			gpio_set(LED_PORT, LED_PIN);
		} else {
			gpio_clear(LED_PORT, LED_PIN);
		}
		this->start(heartBeatSequence[int(mLastMsg)].delay, false);
	}

public:

	void init() override {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
		gpio_set_output_options(LED_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, LED_PIN);
		gpio_set(LED_PORT, LED_PIN);

		auto msg = flawless::getFreeMessage<int>();
		if (msg) {
			msg = 0;
			msg.post<1>();
		}
	}
};

TestModule testModule;

}


