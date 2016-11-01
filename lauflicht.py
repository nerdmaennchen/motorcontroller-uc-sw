from usbtest import *
import struct as st
import time
import random

def flush(vals):
	setConfig(dev, configs, "led_values", tuple(vals), False)

def fade():
	while True:
		for brightness in range(0, 255):
			vals = [brightness << 16 | brightness << 8 | brightness] * numLEDs
			flush(vals)
			time.sleep(0.005)
		for brightness in range(255, 0, -1):
			vals = [brightness << 16 | brightness << 8 | brightness] * numLEDs
			flush(vals)
			time.sleep(0.005)

def randomStuff():
	random.seed()
	ledArr = [0] * numLEDs
	idx = 0
	while True:
		idx = idx + 1
		if idx >= numLEDs:
			idx = 0
		ledArr[idx] = random.randint(0, 0xffffff) #0xff0000
		setConfig(dev, configs, "led_values", tuple(ledArr), False)
		time.sleep(.01)
	
def lauflicht():
	numLEDs = int(configs["led_values"][1] / 4)
	ledArr = [0] * numLEDs
	idx = 0
	while True:
		ledArr[idx] = 0
		idx = idx + 1
		if idx >= numLEDs:
			idx = 0
		ledArr[idx] = 0xff0000
		setConfig(dev, configs, "led_values", tuple(ledArr), False)
		time.sleep(.01)


if __name__ == "__main__":
	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()
	configs = fetchConfig(dev)
	numLEDs = int(configs["led_values"][1] / 4)
	
	fade()

