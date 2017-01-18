from usbctl import *
import struct as st
import time
import random

def flush(vals):
	setConfig(dev, configs, "led.values.front", tuple(vals), False)
	setConfig(dev, configs, "led.values.back",  tuple(vals + [0]), False)

def fade():
	while True:
		for brightness in range(0, 255):
			vals = [brightness << 16 | brightness << 8 | 0] * numLEDs
			flush(vals)
			time.sleep(0.00001)
		for brightness in range(255, 0, -1):
			vals = [brightness << 16 | brightness << 8 | 0] * numLEDs
			flush(vals)
			time.sleep(0.00001)
			
def knightRider():
	numLEDsFront = int(configs["led.values.front"][1] / 4)
	numLEDsBack  = int(configs["led.values.back"][1] / 4)
	while True:
		for i in list(range(0, numLEDsFront, 1)) + list(range(numLEDsFront-1, -1, -1)):
			valsFront = [0] * numLEDsFront
			valsFront[i] = 255 << 8
			setConfig(dev, configs, "led.values.front", tuple(valsFront), False)
			valsBack = [0] * numLEDsBack
			valsBack[i] = 255 << 8
			setConfig(dev, configs, "led.values.back",  tuple(valsBack), False)
			time.sleep(0.025)
			
def randomStuff():
	random.seed()
	ledArr = [0] * numLEDs
	idx = 0
	while True:
		idx = idx + 1
		if idx >= numLEDs:
			idx = 0
		ledArr[idx] = random.randint(0, 0xffffff) #0xff0000
		setConfig(dev, configs, "led.values.front", tuple(ledArr), False)
		time.sleep(.01)
	
def lauflicht():
	numLEDs = int(configs["led.values.front"][1] / 4)
	ledArr = [0] * numLEDs
	idx = 0
	while True:
		ledArr[idx] = 0
		idx = idx + 1
		if idx >= numLEDs:
			idx = 0
		ledArr[idx] = 0xff0000
		setConfig(dev, configs, "led.values.front", tuple(ledArr), False)
		time.sleep(.01)


if __name__ == "__main__":
	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()
	configs = fetchConfig(dev)
	numLEDs = int(configs["led.values.front"][1] / 4)

	knightRider()
	fade()
	randomStuff()

