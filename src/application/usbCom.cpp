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

class USBComModule : public flawless::Module, public flawless::PhyInterface, public usb_ep_callback {
	USBManager& manager = USBManager::get();

	flawless::FIFO<uint8_t, 256, flawless::LockGuard> mTxFifo;

	class : public usb_ep_set_config_callback {
		void callback(usbd_device *usbd_dev, uint16_t) override {
			UNUSED(usbd_dev);
//			usbd_ep_write_packet(usbd_dev, COM_USB_OUT_ENDPOINT_NO, NULL, 0, MAX_PACKET_SIZE);
		}
	} usbSetConfigCallback;

public:
	USBComModule(unsigned int level) : flawless::Module(level), flawless::PhyInterface(0) {}
	~USBComModule() {}

	void init(unsigned int) {
		manager.usb_register_endpoint(&in_usb_ep_descriptor, MAX_PACKET_SIZE, this , nullptr);
		manager.usb_register_endpoint(&out_usb_ep_descriptor, MAX_PACKET_SIZE, this, &usbSetConfigCallback);
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
				Array<uint8_t, MAX_PACKET_SIZE> buf;
				size_t i = 0;
				while (i < buf.size() and i < mTxFifo.count()) {
					buf[i] = mTxFifo[i];
					++i;
				}
				uint16_t len = usbd_ep_write_packet(usbd_dev, ep, buf.data(), i, MAX_PACKET_SIZE);
				mTxFifo.pop(len);
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
		mTxFifo.put(epNum);
		mTxFifo.put((totalLen >> 0) & 0xff);
		mTxFifo.put((totalLen >> 8) & 0xff);
		mCurPacketLen = totalLen;
	}

	void sendPacket(void const* msg, uint16_t len)
	{
		uint8_t const* msgCast = (uint8_t const*)msg;
		while (len) {
			size_t stored = mTxFifo.put(msgCast, len);
			msgCast += stored;
			len -= stored;
			mCurPacketLen -= stored;
			flawless::LockGuard lock;
			if ((not mSending) and ((mTxFifo.countFree() == 0) or (mCurPacketLen == 0))) {
				callback(manager.getUSBDevice(), COM_USB_IN_ENDPOINT_OUT_NO);
			}
		}
	}

private:
	uint16_t mCurPacketLen {0};
	bool mSending {false};
};

USBComModule USBtestModule(3);
}
