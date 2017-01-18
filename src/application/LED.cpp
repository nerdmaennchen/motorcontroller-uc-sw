#include <flawless/stdtypes.h>

#include <flawless/core/Message.h>
#include <flawless/core/MessageBufferMemory.h>
#include <flawless/core/MessageBufferManager.h>
#include <flawless/module/Module.h>
#include <flawless/applicationConfig/ApplicationConfig.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/spi.h>
#include <libopencm3/stm32/f4/dma.h>


/**
 * some helper to build strings from constexpressions
 */

namespace
{

using rgb_t = union {
	struct {
		uint8_t b, r, g;
	} vals;
	int rgb;
};

constexpr uint32_t bytesToSubstituteCnt(uint32_t bytes) {
	return bytes * 4;
}


constexpr uint32_t LED_PORT = GPIOB;
constexpr uint32_t LED_PIN  = GPIO15;

constexpr uint32_t LED_SPI         = SPI2;
constexpr uint32_t LED_DMA         = DMA1;
constexpr uint32_t LED_DMA_STREAM  = DMA_STREAM_4;
constexpr uint32_t LED_DMA_CHANNEL = 0;

constexpr uint32_t PREAMBLE_BYTES = 20;

constexpr uint32_t NUM_LEDS_FRONT = 10;
constexpr uint32_t NUM_LEDS_BACK  = 11;
constexpr uint32_t NUM_LEDS = NUM_LEDS_FRONT + NUM_LEDS_BACK;

constexpr uint8_t LED_ONE_SUBSTITUTE  = 0xe;
constexpr uint8_t LED_ZERO_SUBSTITUTE = 0x8;

struct LEDModule : public flawless::Module
{
	LEDModule(unsigned int level) : flawless::Module(level) {}

	void init(unsigned int) override {
		mOutBufferFront = &(mOutputBuffer[PREAMBLE_BYTES]);
		mOutBufferBack  = mOutBufferFront + bytesToSubstituteCnt(NUM_LEDS_FRONT*3);
		for (uint32_t i(PREAMBLE_BYTES); i < mOutputBuffer.size(); ++i) {
			mOutputBuffer[i] = LED_ZERO_SUBSTITUTE | LED_ZERO_SUBSTITUTE << 4;
		}
		setupSPI();
		setupDMA();
	}

	void setupDMA(void)
	{
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;

		/* disable dma channel */
		while (0 != (DMA_SCCR(LED_DMA, LED_DMA_STREAM) & DMA_CR_EN)) {
			DMA_SCCR(LED_DMA, LED_DMA_STREAM)  = 0;
		}
		DMA_HIFCR(LED_DMA) = (DMA_HISR(LED_DMA) & (0x3d << 0));

		DMA_SM0AR(LED_DMA, LED_DMA_STREAM) = (uint32_t)mOutputBuffer.data();
		DMA_SNDTR(LED_DMA, LED_DMA_STREAM) = mOutputBuffer.size();
		DMA_SPAR(LED_DMA, LED_DMA_STREAM)  = (uint32_t)&SPI_DR(LED_SPI);

		/* set priority*/
		DMA_SCCR(LED_DMA, LED_DMA_STREAM)  =
				(LED_DMA_CHANNEL << DMA_CR_CHSEL_LSB)
				| DMA_CR_PL_HIGH
				| DMA_CR_MSIZE_BYTE
				| DMA_CR_PSIZE_BYTE
				| DMA_CR_MINC
				| DMA_CR_DIR
				| DMA_CR_CIRC;

		DMA_SCCR(LED_DMA, LED_DMA_STREAM) |= DMA_CR_EN;
	}

	void setupSPI() {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		RCC_APB1ENR |= RCC_APB1ENR_SPI2EN;

		gpio_set_output_options(LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, LED_PIN);
		gpio_mode_setup(LED_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LED_PIN);
		gpio_set_af(LED_PORT, GPIO_AF5, LED_PIN);


		SPI_CR2(LED_SPI)  = SPI_CR2_TXDMAEN;
		SPI_CR1(LED_SPI)  =
				SPI_CR1_BAUDRATE_FPCLK_DIV_16 // 42MHz / 32 ergibt 2.625 MHz
				| SPI_CR1_SSM | SPI_CR1_SSI
				| SPI_CR1_MSTR
				| SPI_CR1_MSBFIRST
				| SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE
				| SPI_CR1_CPHA_CLK_TRANSITION_1;

		SPI_CR1(LED_SPI) |= SPI_CR1_SPE;
	}

	Array<uint8_t, PREAMBLE_BYTES + bytesToSubstituteCnt(NUM_LEDS * 3)> mOutputBuffer;
	uint8_t *mOutBufferFront {nullptr};
	uint8_t *mOutBufferBack  {nullptr};
} ledModule(9);

namespace {
using LedValuesFront = Array<rgb_t, NUM_LEDS_FRONT>;
struct : public flawless::Callback<LedValuesFront&, bool> {
	LedValuesFront rgbValues;
	void callback(LedValuesFront&, bool setter) override {
		if (setter) {
			uint8_t *outBufPtr = ledModule.mOutBufferFront;
			for (rgb_t const& rgb : rgbValues) {
				for (int j = 23; j >= 0; j -= 2) {
					int subVal = 0;
					if (rgb.rgb & (1 << (j))) {
						subVal = LED_ONE_SUBSTITUTE << 4;
					} else {
						subVal = LED_ZERO_SUBSTITUTE << 4;
					}
					if (rgb.rgb & (1 << (j - 1))) {
						subVal |= LED_ONE_SUBSTITUTE;
					} else {
						subVal |= LED_ZERO_SUBSTITUTE;
					}
					*outBufPtr = subVal;
					++outBufPtr;
				}
			}
		}
	}
} ledFrontHelper;
flawless::ApplicationConfigMapping<LedValuesFront> cfgLEDFrontMapping {"led.values.front", "10I", &ledFrontHelper, ledFrontHelper.rgbValues};
}

namespace {
using LedValuesBack = Array<rgb_t, NUM_LEDS_BACK>;
struct : public flawless::Callback<LedValuesBack&, bool> {
	LedValuesBack rgbValues;
	void callback(LedValuesBack&, bool setter) override {
		if (setter) {
			uint8_t *outBufPtr = ledModule.mOutBufferBack;
			for (rgb_t const& rgb : rgbValues) {
				for (int j = 23; j >= 0; j -= 2) {
					int subVal = 0;
					if (rgb.rgb & (1 << (j))) {
						subVal = LED_ONE_SUBSTITUTE << 4;
					} else {
						subVal = LED_ZERO_SUBSTITUTE << 4;
					}
					if (rgb.rgb & (1 << (j - 1))) {
						subVal |= LED_ONE_SUBSTITUTE;
					} else {
						subVal |= LED_ZERO_SUBSTITUTE;
					}
					*outBufPtr = subVal;
					++outBufPtr;
				}
			}
		}
	}
} ledBackHelper;
flawless::ApplicationConfigMapping<LedValuesBack> cfgLEDBackMapping {"led.values.back", "11I", &ledBackHelper, ledBackHelper.rgbValues};
}

}
