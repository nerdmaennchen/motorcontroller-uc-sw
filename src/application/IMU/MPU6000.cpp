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
#include <libopencm3/stm32/f4/spi.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/nvic.h>

constexpr uint32_t SPI_PORT = GPIOA;
constexpr uint32_t SPI_NSS  = GPIO4;

constexpr uint32_t SPI_SCK  = GPIO5;
constexpr uint32_t SPI_MISO = GPIO6;
constexpr uint32_t SPI_MOSI = GPIO7;

constexpr uint32_t SPI_PINS = (SPI_SCK | SPI_MISO | SPI_MOSI);


#define MPU6000_SPI        SPI1

#define SPI_TX_DMA         DMA2
#define SPI_TX_DMA_STREAM  DMA_STREAM_3
#define SPI_TX_DMA_CHANNEL 3
#define SPI_RX_DMA         DMA2
#define SPI_RX_DMA_STREAM  DMA_STREAM_2
#define SPI_RX_DMA_CHANNEL 3

namespace
{

struct MPU6000Data {
	Array<int16_t, 3> accData;
	int16_t temperatureData;
	Array<int16_t, 3> gyrData;
};

static uint8_t dummyZero {0};

struct MPU6000 : public flawless::Module, public flawless::TimerCallback
{
	flawless::ApplicationConfig<MPU6000Data> mMPUData{"imu_raw_data", "7h"};

	MPU6000(unsigned int level) : flawless::Module(level) {}
	virtual ~MPU6000() {};

	Array<uint8_t, 4 + sizeof(MPU6000Data)> mRxBuf;
	void *curRXTargetBuffer {nullptr};
	int curRXBufferLen {0};
	bool mReadingIMUValues {false};

	void readRegisters(int startRegister, int numRegisters, void* targetBuffer, bool blocking = true) {
		while ((DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM) & DMA_CR_EN));
		while ((DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM) & DMA_CR_EN));
		gpio_set(SPI_PORT, SPI_NSS);
		DMA_LIFCR(SPI_RX_DMA) = (DMA_LISR(SPI_RX_DMA) & (0x3d << 16));
		DMA_LIFCR(SPI_TX_DMA) = (DMA_LISR(SPI_TX_DMA) & (0x3d << 22));

		{
			flawless::LockGuard lock;
			curRXTargetBuffer = targetBuffer;
			curRXBufferLen    = numRegisters;

			dummyZero = 0;
			DMA_SM0AR(SPI_TX_DMA, SPI_TX_DMA_STREAM) = (uint32_t)(&dummyZero);
			DMA_SM0AR(SPI_RX_DMA, SPI_RX_DMA_STREAM) = (uint32_t)&(mRxBuf[3]);

			DMA_SNDTR(SPI_TX_DMA, SPI_TX_DMA_STREAM) = numRegisters;
			DMA_SNDTR(SPI_RX_DMA, SPI_RX_DMA_STREAM) = numRegisters+1;

			while (SPI_SR(MPU6000_SPI) & SPI_SR_RXNE) {
				(void) SPI_DR(MPU6000_SPI);
			}

			DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM)  =
					(SPI_TX_DMA_CHANNEL << DMA_CR_CHSEL_LSB)
					| DMA_CR_MSIZE_BYTE
					| DMA_CR_PSIZE_BYTE
					| DMA_CR_DIR;
			DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM)  =
					(SPI_RX_DMA_CHANNEL << DMA_CR_CHSEL_LSB)
					| DMA_CR_MSIZE_BYTE
					| DMA_CR_PSIZE_BYTE
					| DMA_CR_MINC
					| DMA_CR_TCIE;

			gpio_clear(SPI_PORT, SPI_NSS);
			// transmit the adress by hand and then start the DMAs
			SPI_DR(MPU6000_SPI) =  startRegister | (1 << 7);

			DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM) |= DMA_CR_EN;
			DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM) |= DMA_CR_EN;
		}
		while (blocking and (DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM) & DMA_CR_EN));
	}

	void writeRegisters(int startRegister, int numRegisters, void const* data, bool blocking = true) {
		while ((DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM) & DMA_CR_EN));
		while ((DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM) & DMA_CR_EN));
		gpio_set(SPI_PORT, SPI_NSS);
		DMA_LIFCR(SPI_RX_DMA) = (DMA_LISR(SPI_RX_DMA) & (0x3d << 16));
		DMA_LIFCR(SPI_TX_DMA) = (DMA_LISR(SPI_TX_DMA) & (0x3d << 22));
		while(curRXTargetBuffer) {}

		{
			flawless::LockGuard lock;

			DMA_SM0AR(SPI_TX_DMA, SPI_TX_DMA_STREAM) = (uint32_t)(data);
			DMA_SM0AR(SPI_RX_DMA, SPI_RX_DMA_STREAM) = (uint32_t)(&dummyZero);

			DMA_SNDTR(SPI_TX_DMA, SPI_TX_DMA_STREAM) = numRegisters;
			DMA_SNDTR(SPI_RX_DMA, SPI_RX_DMA_STREAM) = numRegisters+1;

			DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM)  =
					(SPI_TX_DMA_CHANNEL << DMA_CR_CHSEL_LSB)
					| DMA_CR_MSIZE_BYTE
					| DMA_CR_PSIZE_BYTE
					| DMA_CR_MINC
					| DMA_CR_DIR;
			DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM)  =
					(SPI_RX_DMA_CHANNEL << DMA_CR_CHSEL_LSB)
					| DMA_CR_MSIZE_BYTE
					| DMA_CR_PSIZE_BYTE
					| DMA_CR_DIR;

			gpio_clear(SPI_PORT, SPI_NSS);
			// transmit the adress by hand and then start the DMAs
			SPI_DR(MPU6000_SPI) = startRegister & ~(1 << 7);

			DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM) |= DMA_CR_EN;
			DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM) |= DMA_CR_EN;
		}
		while (blocking and (DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM) & DMA_CR_EN));
	}

	void setupDMA(void)
	{
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;

		/* disable dma channel */
		while (0 != (DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM) & DMA_CR_EN)) {
			DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM)  = 0;
		}
		while (0 != (DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM) & DMA_CR_EN)) {
			DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM)  = 0;
		}

		DMA_LIFCR(SPI_RX_DMA) = (DMA_LISR(SPI_RX_DMA) & (0x3d << 16));
		DMA_LIFCR(SPI_TX_DMA) = (DMA_LISR(SPI_TX_DMA) & (0x3d << 22));

		DMA_SPAR(SPI_TX_DMA, SPI_TX_DMA_STREAM)  = (uint32_t)&SPI_DR(MPU6000_SPI);
		DMA_SPAR(SPI_RX_DMA, SPI_RX_DMA_STREAM)  = (uint32_t)&SPI_DR(MPU6000_SPI);

		DMA_SM0AR(SPI_RX_DMA, SPI_RX_DMA_STREAM) = (uint32_t)&(mRxBuf[3]);
		DMA_SM0AR(SPI_TX_DMA, SPI_TX_DMA_STREAM) = (uint32_t)(&dummyZero);

		DMA_SCCR(SPI_TX_DMA, SPI_TX_DMA_STREAM)  =
				(SPI_TX_DMA_CHANNEL << DMA_CR_CHSEL_LSB)
				| DMA_CR_MSIZE_BYTE
				| DMA_CR_PSIZE_BYTE
				| DMA_CR_MINC
				| DMA_CR_DIR;
		DMA_SCCR(SPI_RX_DMA, SPI_RX_DMA_STREAM)  =
				(SPI_RX_DMA_CHANNEL << DMA_CR_CHSEL_LSB)
				| DMA_CR_MSIZE_BYTE
				| DMA_CR_PSIZE_BYTE
				| DMA_CR_MINC
				| DMA_CR_TCIE;
	}

	void setupSPI() {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;

		gpio_mode_setup(SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_NSS);
		gpio_set_output_options(SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, SPI_NSS);
		gpio_set(SPI_PORT, SPI_NSS);

		gpio_mode_setup(SPI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SPI_PINS);
		gpio_set_output_options(SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, SPI_PINS);
		gpio_set_af(SPI_PORT, GPIO_AF5, SPI_PINS);


		SPI_CR2(MPU6000_SPI)  = SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
		SPI_CR1(MPU6000_SPI)  =
				SPI_CR1_BAUDRATE_FPCLK_DIV_128
				| SPI_CR1_SSM | SPI_CR1_SSI
				| SPI_CR1_MSTR
				| SPI_CR1_MSBFIRST
				| SPI_CR1_CPHA_CLK_TRANSITION_1
				| SPI_CR1_SPE;
	}

	void callback() { // timer callback
//		readRegisters(117, 1);
//		readRegisters(103, 9, mRxBuf.data());
		mReadingIMUValues = true;
		readRegisters(59, 15, &(mMPUData.get()), true);
	}

	void onTransactionComplete() {
		if (curRXTargetBuffer) {
			memcpy(curRXTargetBuffer, &(mRxBuf[4]), curRXBufferLen);

			if (mReadingIMUValues) {
				for (auto & v : mMPUData->accData) {
					v = ((reinterpret_cast<uint16_t&>(v) >> 8) & 0x00ff) | ((reinterpret_cast<uint16_t&>(v) << 8) & 0xff00);
				}
				mMPUData->temperatureData = ((reinterpret_cast<uint16_t&>(mMPUData->temperatureData) >> 8) & 0x00ff) | ((reinterpret_cast<uint16_t&>(mMPUData->temperatureData) << 8) & 0xff00);
				for (auto & v : mMPUData->gyrData) {
					v = ((reinterpret_cast<uint16_t&>(v) >> 8) & 0x00ff) | ((reinterpret_cast<uint16_t&>(v) << 8) & 0xff00);
				}
			}
		}
		curRXTargetBuffer = nullptr;
	}

	void init(unsigned int) override {
		setupDMA();
		setupSPI();

		nvic_enable_irq(NVIC_DMA2_STREAM2_IRQ);

		const uint8_t configSetupData[] = {
				0,
				0,
				(2 << 3), // gyro resolution to +- 1000°/s
				(0 << 3), // acc resolution to +-2g
		};
		const uint8_t pwrMgmtSetupData[] = {
				0, 0, 0, 0, 0
		};
		writeRegisters(25, sizeof(configSetupData), configSetupData);
		writeRegisters(103, sizeof(pwrMgmtSetupData), pwrMgmtSetupData);

		this->start(1000, true);
	}
} mpu6000(9);



}

extern "C"
{

void dma2_stream2_isr()
{
	DMA_LIFCR(SPI_RX_DMA) = (DMA_LISR(SPI_RX_DMA) & (0x3d << 16));
	gpio_set(SPI_PORT, SPI_NSS);
	mpu6000.onTransactionComplete();
}

}

