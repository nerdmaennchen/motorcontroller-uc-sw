#pragma once


#include <flawless/util/Singleton.h>
#include <flawless/util/Callback.h>
#include <libopencm3/usb/usbd.h>

using usb_ep_callback            = flawless::Callback<usbd_device *, uint8_t>;
using usb_ep_set_config_callback = flawless::Callback<usbd_device *, uint16_t>;

/*
 * a message that is published when a host connects or disconnects
 */
struct UsbState {
	bool mHostConnected;
};

class USBManager : public flawless::util::Singleton<USBManager>
{
public:
	int usb_register_endpoint(const struct usb_endpoint_descriptor* descriptor, uint16_t fifoSize, usb_ep_callback* callback, usb_ep_set_config_callback* setConfCallback);

	usbd_device *getUSBDevice();
};

