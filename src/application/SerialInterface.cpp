#include <flawless/core/Message.h>
#include <flawless/applicationConfig/PersistentApplicationConfig.h>
#include <flawless/module/Module.h>
#include <flawless/util/Array.h>
#include <flawless/util/FiFo.h>

#include <flawless/timer/swTimer.h>

#include <interfaces/ISRTime.h>
#include "interfaces/usb.h"
#include "interfaces/SerialAdressableConfig.h"
#include <target/stm32f4/clock.h>

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/usart.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/dma.h>

#define COM_PORT GPIOA
#define COM_PIN  GPIO2

#define COM_UART  USART2
#define COM_SPEED 115200ULL

#define COM_DMA DMA1
#define COM_DMA_RX_STREAM  DMA_STREAM_5
#define COM_DMA_RX_CHANNEL 4
#define COM_DMA_TX_STREAM  DMA_STREAM_6
#define COM_DMA_TX_CHANNEL 4

namespace
{

Array<char, 512> g_rxBuffer;
Array<char, 512> g_txBuffer;
flawless::PersistentConfiguration<uint8_t> gID {"serial.communication.id", "B", 1};

enum class InstructionType : uint8_t {
	QUERRY           = 0, // Request a description of a publishable
	READ             = 1, // pubish all publishables to which there is a subscription
	SUBSCRIBE        = 2, // subscribe to a publishable
	SET              = 3, // set the value of (one or more) publishables
	QUERRY_RESPONSE  = 4, // a response to a query
	READ_RESPONSE    = 5, // the read response
};

enum class BusState {
	IDLE,
	SENDING,
	RECIEVING,
};

struct PacketHeader {
	uint8_t         targetID;
	InstructionType instruction;
} __attribute__((packed));

struct QuerryReply {
	uint8_t index;
	uint8_t dataSize;
};
struct QueryResponsePacket {
	PacketHeader header;
	QuerryReply queryReply;
};

struct SerialInterfaceHandler {
	virtual void onTXPacketDone() = 0;
	virtual void onRXPacketDone(void const* packet, int recievedCnt) = 0;
};

struct SerialInterfaceCommon :
		public flawless::Module<9> {
	BusState mBusState {BusState::IDLE};
	SerialInterfaceHandler *mCurrentHandler {nullptr};

	void init() override {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		RCC_AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		RCC_APB1ENR |= RCC_APB1ENR_USART2EN;

		// init pins
		gpio_mode_setup(COM_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, COM_PIN);
		gpio_set_output_options(COM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, COM_PIN);
		gpio_set_af(COM_PORT, GPIO_AF7, COM_PIN);

		// init uart
		USART_BRR(COM_UART) = CLOCK_APB1_CLK / COM_SPEED;
		USART_CR1(COM_UART) |= USART_CR1_RE | USART_CR1_TE;
		USART_CR2(COM_UART) |= USART_CR2_LBDIE;
		USART_CR3(COM_UART) |= USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_HDSEL;

		nvic_enable_irq(NVIC_USART2_IRQ);

		while (DMA_SCCR(COM_DMA, COM_DMA_TX_STREAM) & DMA_CR_EN) {
			DMA_SCCR(COM_DMA, COM_DMA_TX_STREAM) &= ~DMA_CR_EN;
		}
		while (DMA_SCCR(COM_DMA, COM_DMA_RX_STREAM) & DMA_CR_EN) {
			DMA_SCCR(COM_DMA, COM_DMA_RX_STREAM) &= ~DMA_CR_EN;
		}

		/********* TX DMA **********/
		/* set channel */
		DMA_SCCR(COM_DMA, COM_DMA_TX_STREAM)  = COM_DMA_TX_CHANNEL << DMA_CR_CHSEL_LSB | DMA_CR_DIR | DMA_CR_MINC | DMA_CR_TCIE;
		/* write to usart_dr */
		DMA_SPAR(COM_DMA, COM_DMA_TX_STREAM)  = (uint32_t) &USART_DR(COM_UART);
		nvic_enable_irq(NVIC_DMA1_STREAM6_IRQ);

		/********* RX DMA **********/
		DMA_SCCR(COM_DMA, COM_DMA_RX_STREAM)  = COM_DMA_RX_CHANNEL << DMA_CR_CHSEL_LSB | DMA_CR_MINC | DMA_CR_CIRC;
		/* read from usart_dr */
		DMA_SPAR(COM_DMA, COM_DMA_RX_STREAM)  = (uint32_t) &USART_DR(COM_UART);
		DMA_SM0AR(COM_DMA, COM_DMA_RX_STREAM) = (uint32_t) g_rxBuffer.data();
		DMA_SNDTR(COM_DMA, COM_DMA_RX_STREAM) = g_rxBuffer.size();

		DMA_SCCR(COM_DMA, COM_DMA_RX_STREAM) |= DMA_CR_EN;

		USART_CR1(COM_UART) |= USART_CR1_UE;
	}

	void setRXBuffer(void *buffer, int length) {
		while (DMA_SCCR(COM_DMA, COM_DMA_RX_STREAM) & DMA_CR_EN) {
			DMA_SCCR(COM_DMA, COM_DMA_RX_STREAM) &= ~DMA_CR_EN;
		}

		DMA_HIFCR(COM_DMA) = 0x3d<<6;

		DMA_SM0AR(COM_DMA, COM_DMA_RX_STREAM) = (uint32_t) buffer;
		DMA_SNDTR(COM_DMA, COM_DMA_RX_STREAM) = length;
		DMA_SCCR(COM_DMA, COM_DMA_RX_STREAM) |= DMA_CR_EN;
	}

	bool send(void const* buffer, int len) {
		if (mBusState != BusState::IDLE) {
			return false;
		}
		mBusState = BusState::SENDING;
		while (DMA_SCCR(COM_DMA, COM_DMA_TX_STREAM) & DMA_CR_EN) {
			DMA_SCCR(COM_DMA, COM_DMA_TX_STREAM) &= ~DMA_CR_EN;
		}
		DMA_HIFCR(COM_DMA) = 0x3d<<16;

		DMA_SM0AR(COM_DMA, COM_DMA_TX_STREAM) = (uint32_t) buffer;
		DMA_SNDTR(COM_DMA, COM_DMA_TX_STREAM) = len;
		DMA_SCCR(COM_DMA, COM_DMA_TX_STREAM) |= DMA_CR_EN;
		return true;
	}

	bool reply(void const* buffer, int len) {
		return send(buffer, len);
	}

	void onSendDone() {
		// send a break charachter to mark a packet end
		USART_CR1(COM_UART) |= USART_CR1_SBK;
	}

	void resetState() {
		setRXBuffer(g_rxBuffer.data(), g_rxBuffer.size());
		mBusState = BusState::IDLE;
	}

	void onBreakReceived() {
		if (mBusState == BusState::SENDING) {
			mBusState = BusState::IDLE;
			mCurrentHandler->onTXPacketDone();
		} else { // if we were not sending then we have receved something
			int recievedCnt = g_rxBuffer.size() - DMA_SNDTR(COM_DMA, COM_DMA_RX_STREAM) - 1; // ignore the BREAK-mark
			mCurrentHandler->onRXPacketDone(g_rxBuffer.data(), recievedCnt);
		}
	}

} interfaceCommonStuff;

struct SlaveSerialInterface :
		public flawless::Module<9>,
		public SerialInterfaceHandler
{
	SerialAdressableConfig *mFirstSubscribed {nullptr};

	void init() override {
		interfaceCommonStuff.mCurrentHandler = this;
	}

	void handleQuerry(char const* data, int payloadLen) {
		if (payloadLen < 2) {
			return;
		}
		// data contains a string that is to be queried
		SerialAdressableConfig *handle = flawless::util::LinkedList<SerialAdressableConfig>::get().mFirst;
		uint8_t index = 0;
		QueryResponsePacket *replyHeader = reinterpret_cast<QueryResponsePacket*>(g_txBuffer.data());
		while (handle) {
			if (int(strlen(handle->mConfig.getName())) == payloadLen and
				0 == strncmp(data, handle->mConfig.getName(), payloadLen)) {
				// found the handle
				replyHeader->header.targetID = 0;
				replyHeader->header.instruction = InstructionType::QUERRY_RESPONSE;
				replyHeader->queryReply.dataSize = handle->mConfig.getSize();
				replyHeader->queryReply.index = index;
				interfaceCommonStuff.reply(g_txBuffer.data(), sizeof(*replyHeader));
				return;
			}
			++index;
			handle = handle->getNext();
		}
		replyHeader->header.targetID = 0;
		replyHeader->header.instruction = InstructionType::QUERRY_RESPONSE;
		replyHeader->queryReply.dataSize = 0;
		replyHeader->queryReply.index = -1;
		interfaceCommonStuff.reply(g_txBuffer.data(), sizeof(*replyHeader));
	}

	void handleRead(char const*, int payloadLen) {
		if (payloadLen != 0) {
			return; // if we are misconfigured we don't send on the bus
		}
		SerialAdressableConfig *subscribedHandle = mFirstSubscribed;
		PacketHeader* header = reinterpret_cast<PacketHeader*>(g_txBuffer.data());
		header->instruction  = InstructionType::READ_RESPONSE;
		header->targetID     = 0; // back to master
		int bytesFilled = sizeof(*header);
		char *txBuf = &(g_txBuffer.data()[bytesFilled]);
		uint8_t index = 0;
		while (subscribedHandle) {
			int len = subscribedHandle->mConfig.getSize();
			memcpy(txBuf, subscribedHandle->mConfig.getValue(), len);
			txBuf += len; bytesFilled += len;
			index += 1;
			subscribedHandle = subscribedHandle->mNextSubscribed;
		}
		interfaceCommonStuff.reply(g_txBuffer.data(), bytesFilled);
	}

	void handleSingleRead(char const* data, int payloadLen) {
		if (payloadLen != 1) {
			return; // if we are misconfigured we don't send on the bus
		}
		SerialAdressableConfig *handle = flawless::util::LinkedList<SerialAdressableConfig>::get().mFirst;
		uint8_t index = 0;
		uint8_t targetIdx = data[0];
		PacketHeader *replyHeader = reinterpret_cast<PacketHeader*>(g_txBuffer.data());
		while (handle) {
			if (index == targetIdx) {
				// found the handle
				replyHeader->targetID = 0;
				replyHeader->instruction = InstructionType::READ_RESPONSE;
				memcpy(g_txBuffer.data()+sizeof(*replyHeader), handle->mConfig.getValue(), handle->mConfig.getSize());
				interfaceCommonStuff.reply(g_txBuffer.data(), sizeof(*replyHeader) + handle->mConfig.getSize());
				return;
			}
			++index;
			handle = handle->getNext();
		}
	}

	void handleSubscibe(char const* data, int payloadLen) {
		if (1 != payloadLen) {
			return;
		}
		SerialAdressableConfig *handle = flawless::util::LinkedList<SerialAdressableConfig>::get().mFirst;
		uint8_t index = 0;
		uint8_t targetIndex = static_cast<uint8_t>(data[0]);
		while (handle) {
			if (index == targetIndex) {
				SerialAdressableConfig **subscribedHandle = &mFirstSubscribed;
				while (*subscribedHandle) {
					if (*subscribedHandle == handle) { // dont double subscribe
						return;
					}
					subscribedHandle = &((*subscribedHandle)->mNextSubscribed);
				}
				*subscribedHandle = handle;
				return;
			}
			++index;
			handle = handle->getNext();
		}
	}

	void handleSet(char const* data, int payloadLen) {
		SerialAdressableConfig *handle = flawless::util::LinkedList<SerialAdressableConfig>::get().mFirst;
		uint8_t index = 0;
		uint8_t targetIndex = static_cast<uint8_t>(data[0]);
		payloadLen -= 1; // the actual payload is a bis smaller
		data += 1; // the actual payload starts here
		while (handle) {
			if (targetIndex == index) {
				if (payloadLen == handle->mConfig.getSize()) {
					handle->mConfig.setValue(data);
				}
				break;
			}
			++index;
			handle = handle->getNext();
		}
	}

	void onTXPacketDone() override {
		interfaceCommonStuff.setRXBuffer(g_rxBuffer.data(), g_rxBuffer.size());
	}

	void onRXPacketDone(void const* packet, int recievedCnt) override {
		PacketHeader const* header = reinterpret_cast<PacketHeader const*>(packet);
		interfaceCommonStuff.setRXBuffer(g_rxBuffer.data(), g_rxBuffer.size());
		if (recievedCnt > 1 and header->targetID == gID and gID != 0) {
			char const* payload = ((char const*)(packet)) + sizeof(*header);
			int payloadLen = recievedCnt - 2;
			switch (header->instruction) {
				case InstructionType::QUERRY:
					handleQuerry(payload, payloadLen);
					break;
				case InstructionType::READ:
					if (payloadLen == 0) {
						handleRead(payload, payloadLen);
					} else if (payloadLen == 1) {
						handleSingleRead(payload, payloadLen);
					}
					break;
				case InstructionType::SUBSCRIBE:
					handleSubscibe(payload, payloadLen);
					break;
				case InstructionType::SET:
					handleSet(payload, payloadLen);
					break;
				default:
					break;
			}
		}
	}
} slaveSerialInterface;

struct MasterSerialInterface :
		public flawless::Listener<UsbState>,
		public SerialInterfaceHandler,
		public flawless::TimerCallback
{
	using ResponseCallback = flawless::Callback<void const*, int>;
	struct Transaction {
		PacketHeader header;
		Array<char, 64> payload;
		int payloadLen;
		void *responseBuffer {nullptr};
		int responseLength {0};
		ResponseCallback *rxCallback {nullptr};
	} __attribute__((packed));

	flawless::FIFO<Transaction, 10, flawless::LockGuard> transactionFifo;

	struct SubscriptionInfo {
		uint8_t targetID;
		int length;
	};
	Array<SubscriptionInfo, 64> subscriptionInfos;
	int numSubscriptions {0};

	// only active when usb is connected
	void callback(flawless::Message<UsbState> const& state) {
		if (state->mHostConnected) {
			interfaceCommonStuff.mCurrentHandler = this;
		} else {
			interfaceCommonStuff.mCurrentHandler = &slaveSerialInterface;
		}
	}

	void callback() override { // on rx timeout
		interfaceCommonStuff.resetState();
		transactionFifo.pop(1);
		triggerTransaction();
	}

	void onTXPacketDone() override {
		if (transactionFifo.count() == 0) {
			interfaceCommonStuff.setRXBuffer(g_rxBuffer.data(), g_rxBuffer.size());
		} else {
			Transaction const& transaction = transactionFifo[0];
			if (transaction.responseBuffer == nullptr || transaction.responseLength == 0) {
				interfaceCommonStuff.setRXBuffer(g_rxBuffer.data(), g_rxBuffer.size());
			} else {
				interfaceCommonStuff.setRXBuffer(transaction.responseBuffer, transaction.responseLength);
			}
			// those two instructions dont produce a reply
			if (transaction.header.instruction == InstructionType::SET ||
				transaction.header.instruction == InstructionType::SUBSCRIBE) {
				transactionFifo.pop(1);
				triggerTransaction();
			} else {
				this->start(5000, false);
			}
		}
	}
	void onRXPacketDone(void const* packet, int receivedLen) override {
		interfaceCommonStuff.setRXBuffer(g_rxBuffer.data(), g_rxBuffer.size());;
		if (0 == transactionFifo.count()) {
			return; // in this case we have received something without asking for it
		}
		PacketHeader const* rxHeader = reinterpret_cast<PacketHeader const*>(packet);
		PacketHeader const* txHeader = &(transactionFifo[0].header);
		switch (rxHeader->instruction) {
			case InstructionType::QUERRY_RESPONSE:
				if (txHeader->instruction == InstructionType::QUERRY) {
					if (transactionFifo[0].rxCallback) {
						transactionFifo[0].rxCallback->callback(packet, receivedLen);
					}
				}
				break;
			case InstructionType::READ_RESPONSE:
				if (txHeader->instruction == InstructionType::READ) {
					if (transactionFifo[0].rxCallback) {
						transactionFifo[0].rxCallback->callback(packet, receivedLen);
					}
				}
				break;
			default:
				break;
		}
		transactionFifo.pop(1);
	}
	void triggerTransaction() {
		if (interfaceCommonStuff.mBusState == BusState::IDLE and transactionFifo.count()) {
			interfaceCommonStuff.send(&(transactionFifo[0]), transactionFifo[0].payloadLen + sizeof(transactionFifo[0].header));
		}
	}

	void performSet(uint8_t targetID, uint8_t idx, char* payload, int payloadLen) {
		Transaction transaction;
		transaction.header.targetID    = targetID;
		transaction.header.instruction = InstructionType::SET;
		transaction.payloadLen = payloadLen + 1;
		transaction.payload[0] = idx;
		memcpy(transaction.payload.data()+1, payload, payloadLen);
		transactionFifo.put(transaction);
		triggerTransaction();
	}

	void performQuerry(uint8_t targetID, char* descriptor, void* responseBuffer = nullptr, int responseBufferSize = 0, ResponseCallback* cb = nullptr) {
		Transaction transaction;
		int payloadLen = strlen(descriptor);
		transaction.header.targetID    = targetID;
		transaction.header.instruction = InstructionType::QUERRY;
		transaction.payloadLen = payloadLen;
		transaction.rxCallback = cb;
		memcpy(transaction.payload.data(), descriptor, strlen(descriptor)+1);

		if (responseBuffer and responseBufferSize) {
			transaction.responseBuffer = responseBuffer;
			transaction.responseLength = responseBufferSize;
		}

		transactionFifo.put(transaction);
		triggerTransaction();
	}

	void performSubscribe(uint8_t targetID, uint8_t targetIdx, uint8_t size) {
		SubscriptionInfo *info = &(subscriptionInfos[0]);
		while (info->targetID != 0 and info->targetID != targetID) {
			++info;
		}
		if (info->targetID == 0) {
			++numSubscriptions;
			info->targetID = targetID;
		}
		info->length += size;
		Transaction transaction;
		transaction.payloadLen = 1;
		transaction.header.instruction = InstructionType::SUBSCRIBE;
		transaction.header.targetID = targetID;
		transaction.payload[0] = targetIdx;
		transactionFifo.put(transaction);
		triggerTransaction();
	}

	void performRead(uint8_t targetID, void* responseBuffer = nullptr, int responseBufferSize = 0, ResponseCallback* cb = nullptr) {
		Transaction transaction;
		transaction.header.targetID    = targetID;
		transaction.header.instruction = InstructionType::READ;
		transaction.payloadLen = 0;
		transaction.rxCallback = cb;

		if (responseBuffer and responseBufferSize) {
			transaction.responseBuffer = responseBuffer;
			transaction.responseLength = responseBufferSize;
		}

		transactionFifo.put(transaction);
		triggerTransaction();
	}

	void performSingleRead(uint8_t targetID, uint8_t targetIdx, void* responseBuffer = nullptr, int responseBufferSize = 0, ResponseCallback* cb = nullptr) {
		Transaction transaction;
		transaction.header.targetID    = targetID;
		transaction.header.instruction = InstructionType::READ;
		transaction.payloadLen = 1;
		transaction.rxCallback = cb;
		transaction.payload[0] = targetIdx;

		if (responseBuffer and responseBufferSize) {
			transaction.responseBuffer = responseBuffer;
			transaction.responseLength = responseBufferSize;
		}

		transactionFifo.put(transaction);
		triggerTransaction();
	}

	int getSubscriptionsBufferSize() const {
		int sum = 0;
		for (int i(0); i < numSubscriptions; ++i) {
			sum += subscriptionInfos[i].length + sizeof(PacketHeader);
		}
		return sum;
	}

	void performFetchSubscriptions(void* responseBuffer, int responseBufferSize, ResponseCallback* cb = nullptr) {
		if (responseBufferSize < getSubscriptionsBufferSize()) {
			return;
		}
		char* targetBuffer = reinterpret_cast<char*>(responseBuffer);
		for (int i(0); i < numSubscriptions; ++i) {
			Transaction transaction;
			transaction.header.targetID    = subscriptionInfos[i].targetID;
			transaction.header.instruction = InstructionType::READ;
			transaction.payloadLen = 0;

			transaction.responseBuffer = targetBuffer;
			transaction.responseLength = subscriptionInfos[i].length + sizeof(PacketHeader);
			targetBuffer += subscriptionInfos[i].length + sizeof(PacketHeader);

			if (i == numSubscriptions-1 or transactionFifo.countFree() == 0) {
				transaction.rxCallback = cb;
			}
			transactionFifo.put(transaction);
		}
		triggerTransaction();
	}

} masterSerialInterface;


struct QuerryInfo {
	uint8_t targetID;
	Array<char, 60> descriptor;
} __attribute__((packed));
struct : public flawless::Callback<QuerryInfo&, bool> {
	void callback(QuerryInfo& info, bool set) override {
		if (set) {
			mQuerryOBuf->queryReply.dataSize = 0xff;
			mQuerryOBuf->queryReply.index    = 0xff;
			masterSerialInterface.performQuerry(info.targetID, info.descriptor.data(), &(mQuerryOBuf.get()), sizeof(mQuerryOBuf.get()));
		}
	}
	flawless::ApplicationConfig<QuerryInfo>  mSetBuf {"serial.interface.querry", "B60s", this};
	flawless::ApplicationConfig<QueryResponsePacket> mQuerryOBuf {"serial.interface.querry.reply", "4B", {{0xff, InstructionType::QUERRY}, {0xff, 0xff}}};
} querryHelper;

struct SetInfo {
	uint8_t targetID;
	uint8_t targetIdx;
	uint8_t payloadLen;
	Array<char, 59> descriptor;
} __attribute__((packed));
struct : public flawless::Callback<SetInfo&, bool> {
	void callback(SetInfo& info, bool set) override {
		if (set) {
			masterSerialInterface.performSet(info.targetID, info.targetIdx, info.descriptor.data(), info.payloadLen);
		}
	}
	flawless::ApplicationConfig<SetInfo>  mSetBuf {"serial.interface.set", "BBB59s", this};
} setHelper;

struct SubscribeInfo {
	uint8_t targetID;
	uint8_t targetIdx;
	uint8_t size;
} __attribute__((packed));
struct : public flawless::Callback<SubscribeInfo&, bool> {
	void callback(SubscribeInfo& info, bool set) override {
		if (set) {
			masterSerialInterface.performSubscribe(info.targetID, info.targetIdx, info.size);
		}
	}
	flawless::ApplicationConfig<SubscribeInfo>  mSetBuf {"serial.interface.subscribe", "BBB", this};
} subscribeHelper;


struct SingleReadInfo {
	uint8_t targetID;
	uint8_t targetIdx;
} __attribute__((packed));
struct : public flawless::Callback<SingleReadInfo&, bool> {
	void callback(SingleReadInfo& info, bool set) override {
		if (set) {
			masterSerialInterface.performSingleRead(info.targetID, info.targetIdx, mResponseBuf->data(), mResponseBuf->size());
		}
	}

	flawless::ApplicationConfig<SingleReadInfo>   mSetBuf {"serial.interface.read.single", "BB", this};
	flawless::ApplicationConfig<Array<char, 64>> mResponseBuf {"serial.interface.read.single.response", "64B"};
} singleReadHelper;

struct : public flawless::Callback<void> {
	void callback() override {
		masterSerialInterface.performFetchSubscriptions(mSubscriptionBuffer->data(), mSubscriptionBuffer->size());
	}
	flawless::ApplicationConfig<void> mFetchSubscriptions {"serial.interface.subscriptions.fetch", this};
	flawless::ApplicationConfig<Array<char, 512>> mSubscriptionBuffer {"serial.interface.subscriptions.fetched", "512B"};
} singleFetchSubscriptionsHelper;


}



extern "C" {

void usart2_isr() {
	int sr = USART_SR(COM_UART);
	if (sr & USART_SR_LBD) {
		interfaceCommonStuff.onBreakReceived();
	}
	USART_SR(COM_UART) = 0;
}

void dma1_stream6_isr() // tx stream
{
	ISRTime isrTimer;
	DMA_HIFCR(COM_DMA) = 0x3d<<16;
	interfaceCommonStuff.onSendDone();
}

}
