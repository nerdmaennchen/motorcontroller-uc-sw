from usbtest import *
import struct as st
import time
import random

if __name__ == "__main__":
	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()
	random.seed()
	
#	flush(dev)
	configs = fetchConfig(dev)
	numLEDs = int(configs["ledBuffer"][1] / 4)
	ledArr = [0] * numLEDs
	idx = 0
	fStr = str(numLEDs) + "i"
	print(fStr)
	while True:
		ledArr[idx] = 0
		idx = idx + 1
		if idx >= numLEDs:
			idx = 0
		ledArr[idx] = random.randint(0, 0xffffff) #0xff0000
		data = st.pack(fStr, *tuple(ledArr))
#		print(ledArr, tuple(ledArr), data)
		setConfig(dev, configs, "ledBuffer", data, False)
#		exit(0)
		time.sleep(.01)
	print(numLEDs)
