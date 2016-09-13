from usbtest import *
import struct as st
import time

additionalRounds = 10
myDelay = 0.001
def cw(dev, configs):
	lastHallState, = getConfig(dev, configs, "hallstates")
	lastIdx = 0
	print("{0:03d} {1:b} {2:b} {3:b}".format(lastIdx%steps, (lastHallState >> 2)&1, (lastHallState >> 1)&1, (lastHallState >> 0)&1))
	for i in range(0, steps):
		setConfig(dev, configs, "phase", st.pack('=H', i), False) # advancePhase
		time.sleep(myDelay)
	for i in range(0, steps*additionalRounds):
		setConfig(dev, configs, "phase", st.pack('=H', i), False) # advancePhase
		time.sleep(myDelay)
		hstates, = getConfig(dev, configs, "hallstates")
		if lastHallState != hstates:
			print("{0:03d},{1:b},{2:b},{3:b}".format(i%steps, (lastHallState >> 2)&1, (lastHallState >> 1)&1, (lastHallState >> 0)&1))
			lastHallState = hstates
			lastIdx = i
			print("{0:03d},{1:b},{2:b},{3:b}".format(i%steps, (lastHallState >> 2)&1, (lastHallState >> 1)&1, (lastHallState >> 0)&1))
	
def ccw(dev, configs):
	lastHallState, = getConfig(dev, configs, "hallstates")
	lastIdx = 0
	for i in range(steps, 0, -1):
		setConfig(dev, configs, "phase", st.pack('=H', i), False) # advancePhase
		time.sleep(myDelay)
	for i in range(steps*additionalRounds, 0, -1):
		setConfig(dev, configs, "phase", st.pack('=H', i), False) # advancePhase
		time.sleep(myDelay)
		hstates, = getConfig(dev, configs, "hallstates")
		if lastHallState != hstates:
			print("{0:03d},{1:b},{2:b},{3:b}".format(i%steps, (lastHallState >> 2)&1, (lastHallState >> 1)&1, (lastHallState >> 0)&1))
			lastHallState = hstates
			lastIdx = i
			print("{0:03d},{1:b},{2:b},{3:b}".format(i%steps, (lastHallState >> 2)&1, (lastHallState >> 1)&1, (lastHallState >> 0)&1))

if __name__ == "__main__":
	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()

	configs = fetchConfig(dev)
	
	steps = int(configs['cwConfigs'][1] / 8)
	
	# enable motor
	setConfig(dev, configs, "enable", st.pack('=B', 1), False)
	ccw(dev,configs)
	setConfig(dev, configs, "enable", st.pack('=B', 0), False) # disable motor
		
