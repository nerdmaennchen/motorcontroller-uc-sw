from usbtest import *
import struct as st
import time


if __name__ == "__main__":
	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()

	flush(dev)
	configs = fetchConfig(dev)
	while True:
		curDelay = st.unpack('@Q', getConfig(dev, configs, "HallDelay"))
		print(1000000. / (curDelay[0] * 6))
#		print(curDelay[0])
		time.sleep(0.01)
