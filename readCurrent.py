from usbtest import *
import struct as st

def viel(dev, configs):
	formatStr = "{:1.3f} "*2
	while True:
		current = getConfig(dev, configs, "motorCurrent")
		currentInts = st.unpack('@2f', current)
		print(formatStr.format(*currentInts))
		
def wenig(dev, configs):
	formatStr = "{:1.3f} "
	while True:
		current = getConfig(dev, configs, "motorCurrentMean")
		currentInts = st.unpack('@f', current)
		print(formatStr.format(*currentInts))
	

if __name__ == "__main__":
	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()

	flush(dev)
	configs = fetchConfig(dev)
	wenig(dev, configs)
