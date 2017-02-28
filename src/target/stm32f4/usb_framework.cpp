/*
 * usb.c
 *
 *  Created on: 26.02.2015
 *      Author: lutz
 */

#include "interfaces/usb.h"
#include <interfaces/ISRTime.h>

#include <flawless/timer/swTimer.h>

#include <stdlib.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <flawless/util/Array.h>

//#include <libopencm3/cm3/scb.h>

#include <string.h>


#include "interfaces/usb.h"

namespace
{

static const char *usb_strings[] = {
	"FU Berlin",
	"BLDC-Controller",
	"0.9"
};

usbd_device *g_usbd_dev;

Array<usb_endpoint_descriptor, 8> g_usb_endpoints;
Array<uint16_t, 8> g_usb_endpointFifoSizes;
Array<usb_ep_callback*, 8> g_usb_ep_out_callbacks;
Array<usb_ep_callback*, 8> g_usb_ep_in_callbacks;
Array<usb_ep_set_config_callback*, 8> g_usb_ep_set_config_callbacks;

int g_num_usb_endpoints = 0;
/* Buffer to be used for control requests. */
Array<uint8_t, 128> usbd_control_buffer;
}


extern "C" {
usb_device_descriptor dev {
	/* .bLength = */ USB_DT_DEVICE_SIZE,
	/* .bDescriptorType = */ USB_DT_DEVICE,
	/* .bcdUSB = */ 0x0200,
	/* .bDeviceClass = */ 0x00,
	/* .bDeviceSubClass = */ 0,
	/* .bDeviceProtocol = */ 0,
	/* .bMaxPacketSize0 = */ 64,
	/* .idVendor = */ 0xCAFE,
	/* .idProduct = */ 0xCAFE,
	/* .bcdDevice = */ 0x0200,
	/* .iManufacturer = */ 1,
	/* .iProduct = */ 2,
	/* .iSerialNumber = */ 3,
	/* .bNumConfigurations = */ 1,
};

usb_interface_descriptor iface = {
	/* .bLength = */ USB_DT_INTERFACE_SIZE,
	/* .bDescriptorType = */ USB_DT_INTERFACE,
	/* .bInterfaceNumber = */ 0,
	/* .bAlternateSetting = */ 0,
	/* .bNumEndpoints = */ 0,
	/* .bInterfaceClass = */ 0xFF,
	/* .bInterfaceSubClass = */ 0,
	/* .bInterfaceProtocol = */ 0,
	/* .iInterface = */ 0,
	/* .endpoint = */ g_usb_endpoints.data(),
	nullptr,
	0
};

Array<usb_config_descriptor::usb_interface, 1> ifaces = {{
	/* .num_altsetting = */ 1,
	/* .iface_assoc = */ nullptr,
	/* .altsetting = */ &iface,
}};

usb_config_descriptor config = {
	/* .bLength = */ USB_DT_CONFIGURATION_SIZE,
	/* .bDescriptorType = */ USB_DT_CONFIGURATION,
	/* .wTotalLength = */ 0,
	/* .bNumInterfaces = */ 1,
	/* .bConfigurationValue = */ 1,
	/* .iConfiguration = */ 0,
	/* .bmAttributes = */ 0x80,
	/* .bMaxPower = */ 250,

	/* .interface = */ &ifaces[0],
};


static void usb_set_config_callback(usbd_device *usbd_dev, uint16_t wValue);

}

int USBManager::usb_register_endpoint(const struct usb_endpoint_descriptor* descriptor, uint16_t fifoSize, usb_ep_callback* callback, usb_ep_set_config_callback* setConfCallback)
{
	const int usbEpDescSize = g_usb_endpoints.size();
	if (g_num_usb_endpoints < usbEpDescSize)
	{
		memcpy(&(g_usb_endpoints[g_num_usb_endpoints]), descriptor, usbEpDescSize);
		if (descriptor->bEndpointAddress & 0x80) {
			g_usb_ep_in_callbacks[descriptor->bEndpointAddress & ~0x80] = callback;
		} else {
			g_usb_ep_out_callbacks[descriptor->bEndpointAddress] = callback;
		}
		g_usb_endpointFifoSizes[g_num_usb_endpoints] = fifoSize;
		g_usb_ep_set_config_callbacks[g_num_usb_endpoints] = setConfCallback;
		++g_num_usb_endpoints;
		++(iface.bNumEndpoints);
	}
	return 1;
}

usbd_device *USBManager::getUSBDevice()
{
	return g_usbd_dev;
}

namespace
{
struct InitHelper : public flawless::Module<1000>
{
	void init() override {
		RCC_AHB1ENR |= RCC_AHB1ENR_IOPAEN;
		RCC_AHB2ENR |= RCC_AHB2ENR_OTGFSEN;

		gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
				GPIO11 | GPIO12);
		gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

		// setup usb stuff
		iface.bNumEndpoints = g_num_usb_endpoints;

		g_usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
				usb_strings, 3,
				usbd_control_buffer.data(), 64);

		usbd_register_set_config_callback(g_usbd_dev, &usb_set_config_callback);
		nvic_enable_irq(NVIC_OTG_FS_IRQ);
	}
} initHelper;

}

extern "C" {

static void usbEPCallbackWrapper(usbd_device *usbd_dev, uint8_t ep) {
	if (ep & 0x80) {
		ep = ep & ~0x80;
		if (g_usb_ep_in_callbacks[ep]) {
			g_usb_ep_in_callbacks[ep]->callback(usbd_dev, ep|0x80);
		}
	} else {
		if (g_usb_ep_out_callbacks[ep]) {
			g_usb_ep_out_callbacks[ep]->callback(usbd_dev, ep);
		}
	}
}

static void usb_set_config_callback(usbd_device *usbd_dev, uint16_t wValue)
{
	for (int i = 0; i < g_num_usb_endpoints; ++i) {
		const struct usb_endpoint_descriptor* ep = &(g_usb_endpoints[i]);
		usbd_ep_setup(usbd_dev, ep->bEndpointAddress, ep->bmAttributes, ep->wMaxPacketSize, g_usb_endpointFifoSizes[i], &usbEPCallbackWrapper);
		if (g_usb_ep_set_config_callbacks[i]) {
			g_usb_ep_set_config_callbacks[i]->callback(usbd_dev, wValue);
		}
	}
}

void otg_fs_isr()
{
	ISRTime isrTimer;
	usbd_poll(g_usbd_dev);
}
}
