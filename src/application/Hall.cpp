#include <flawless/stdtypes.h>

#include <flawless/core/Message.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/module/Module.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/nvic.h>


namespace
{
flawless::MessageBufferMemory<int, 5> intMsgBuf;
flawless::MessageBufferMemory<int, 1> intMsgBuf2;

constexpr uint32_t HALL_PORT = GPIOA;
constexpr uint32_t HALL_U    = GPIO10;
constexpr uint32_t HALL_V    = GPIO9;
constexpr uint32_t HALL_W    = GPIO8;
constexpr uint32_t HALL_PINS = (HALL_U | HALL_V | HALL_W);

struct HallModule : public flawless::Module
{
	HallModule(unsigned int level) : flawless::Module(level) {}

	void init(unsigned int) override {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;

		gpio_mode_setup(HALL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, HALL_PINS);
//		gpio_set_af(HALL_PORT, GPIO_AF1, HALL_PINS);
	}

} hallModule(7);

}
