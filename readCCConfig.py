from usbtest import *
import struct as st
import time


if __name__ == "__main__":
	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()

	configs = fetchConfig(dev)
	
	steps = int(configs['cwConfigs'][1] / 8)
	formatStr = str(steps*4) + "H"
	cwConfigs = st.unpack(formatStr, getConfig(dev, configs, "cwConfigs"))
	ccwConfigs = st.unpack(formatStr, getConfig(dev, configs, "ccwConfigs"))
	formatStr = "{:05d}" + ",{:05d}"*8
	for i in range(0, steps):
		idx = i * 4
		print(formatStr.format(i, 
			cwConfigs[idx+0],cwConfigs[idx+1],cwConfigs[idx+2],cwConfigs[idx+3], 
			ccwConfigs[idx+0],ccwConfigs[idx+1],ccwConfigs[idx+2],ccwConfigs[idx+3]))
