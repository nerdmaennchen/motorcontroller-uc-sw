#include <flawless/module/Module.h>
#include "interfaces/usb.h"
#include <flawless/protocol/ProtocolPacketDispatcher.h>
#include <flawless/protocol/PhyInterface.h>
#include <flawless/util/FiFo.h>

#include <string.h>

#include <interfaces/systemTime.h>

namespace
{
#define COM_USB_IN_ENDPOINT_OUT_NO (0x81)
#define COM_USB_OUT_ENDPOINT_NO (0x01)

#define MAX_PACKET_SIZE 64
#define USB_FIFO_SIZE (MAX_PACKET_SIZE * 4)

const usb_endpoint_descriptor out_usb_ep_descriptor =
{
	/* .bLength = */ USB_DT_ENDPOINT_SIZE,
	/* .bDescriptorType = */ USB_DT_ENDPOINT,
	/* .bEndpointAddress = */ COM_USB_OUT_ENDPOINT_NO,
	/* .bmAttributes = */ USB_ENDPOINT_ATTR_BULK,
	/* .wMaxPacketSize = */ MAX_PACKET_SIZE,
	/* .bInterval = */ 1,
	/* .extra = */ nullptr,
	/* .extralen = */ 0,
};

const usb_endpoint_descriptor in_usb_ep_descriptor =
{
	/* .bLength = */ USB_DT_ENDPOINT_SIZE,
	/* .bDescriptorType = */ USB_DT_ENDPOINT,
	/* .bEndpointAddress = */ COM_USB_IN_ENDPOINT_OUT_NO,
	/* .bmAttributes = */ USB_ENDPOINT_ATTR_BULK,
	/* .wMaxPacketSize = */ MAX_PACKET_SIZE,
	/* .bInterval = */ 1,
	/* .extra = */ nullptr,
	/* .extralen = */ 0,
};

struct USBComModule : public flawless::Module, public flawless::PhyInterface, public usb_ep_callback, public usb_ep_set_config_callback {
	USBManager& manager = USBManager::get();

	flawless::FIFO<uint8_t, USB_FIFO_SIZE*2, flawless::LockGuard> mTxFifo;

	SystemTime &time = SystemTime::get();

	USBComModule(unsigned int level) : flawless::Module(level), flawless::PhyInterface(0) {}
	~USBComModule() {}

	void init(unsigned int) {
		manager.usb_register_endpoint(&in_usb_ep_descriptor, USB_FIFO_SIZE, this , this);
		manager.usb_register_endpoint(&out_usb_ep_descriptor, USB_FIFO_SIZE, this, nullptr);
	}

	void callback(usbd_device *usbd_dev, uint16_t) override {
		UNUSED(usbd_dev);
		mTxFifo.clear();
		mSending = false;
	}

	void callback(usbd_device *usbd_dev, uint8_t ep) override {
		if (ep == COM_USB_OUT_ENDPOINT_NO) {
			flawless::Packet packet = flawless::MessageBufferManager<flawless::Packet_>::get().getFreeMessage();
			if (packet) {
				packet->len = usbd_ep_read_packet(usbd_dev, ep, packet->buffer.data(), MAX_PACKET_SIZE);
				packet->iface = this;
				packet.post<MSG_ID_INCOMMING_PACKET>();
			} else {
				flawless::MessageBufferManager<flawless::Packet_>::get().getFreeMessage();
				// discard this message
				uint8_t dummyBuffer[MAX_PACKET_SIZE];
				usbd_ep_read_packet(usbd_dev, ep, dummyBuffer, MAX_PACKET_SIZE);
			}
		} else {
			if (mTxFifo.count()) {
				mSending = true;
				Array<uint8_t, USB_FIFO_SIZE> buf;
				size_t i = 0;
				while (i < buf.size() and i < mTxFifo.count()) {
					buf[i] = mTxFifo[i];
					++i;
				}
				uint16_t len = usbd_ep_write_packet(usbd_dev, ep, buf.data(), i, MAX_PACKET_SIZE);
				if (len) {
					mTxFifo.pop(len);
					packetTimeout = time.getSystemTimeUS() + 1000000;
				}
			} else {
				mSending = false;
			}
		}
	}

	void sendPacket(uint8_t epNum, void const* msg, uint16_t len) override {
		startPacket(epNum, len);
		sendPacket(msg, len);
	}

	void startPacket(uint8_t epNum, uint16_t totalLen) override {
		size_t stored  = mTxFifo.put(epNum);
		stored += mTxFifo.put((totalLen >> 0) & 0xff);
		stored += mTxFifo.put((totalLen >> 8) & 0xff);
		mCurPacketLen = totalLen;
		packetTimeout = time.getSystemTimeUS() + 1000000;
	}

	void sendPacket(void const* msg, uint16_t len)
	{
		uint8_t const* msgCast = (uint8_t const*)msg;
		systemTime_t now;
		do {
			size_t stored = mTxFifo.put(msgCast, len);
			msgCast += stored;
			len -= stored;
			mCurPacketLen -= stored;
			flawless::LockGuard lock;
			if ((not mSending) and ((mTxFifo.countFree() == 0) or (mCurPacketLen == 0))) {
				callback(manager.getUSBDevice(), uint8_t(COM_USB_IN_ENDPOINT_OUT_NO));
			}
			now = time.getSystemTimeUS();
		} while (len and now < packetTimeout);
		if (len) {
			flawless::LockGuard lock;
			mTxFifo.clear();
			mSending = false;
		}
	}

	volatile systemTime_t packetTimeout;
	volatile uint16_t mCurPacketLen {0};
	volatile bool mSending {false};
};

USBComModule USBtestModule(3);
}
