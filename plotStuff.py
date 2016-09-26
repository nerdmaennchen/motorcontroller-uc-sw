from usbtest import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sys import stdin
import select

def data_gen():
	while True:
		vals = ()
		for target in targets:
			vals = vals + getConfig(dev, configs, target)
		yield vals
	
numdata = 100
xdata = list(range(0, numdata))
def run(data):
	# update the data
	datas = run.datas
	for i in range(0, len(data)):
		datas[i].append(data[i])
	
	if len(datas[0]) > numdata:
		for i in range(0, len(datas)):
			datas[i] = datas[i][-numdata:]
	
	ymin, ymax = ax.get_ylim()
	mi = min(data)
	ma = max(data)
	if mi < ymin:
		ymin = mi
		ax.set_ylim(ymin, ymax)
		ax.figure.canvas.draw()
	if ma > ymax:
		ymax = ma
		ax.set_ylim(ymin, ymax)
		ax.figure.canvas.draw()

	for i in range(0, len(datas)):
		lines[i].set_data(xdata[0:len(datas[i])], datas[i])
		
	return lines
run.datas = []

if __name__ == "__main__":
	if len(sys.argv) < 2:
		raise ValueError('Must define at least one target to monitor')

	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()
	configs = fetchConfig(dev)
	targets = sys.argv[1:]
	
	lines = []
	fig, ax = plt.subplots()
	ax.set_xlim(0, numdata)
	
	for target in targets:
		prototype = getConfig(dev, configs, target);
		for i in range(0, len(prototype)):
			run.datas.append([])
			line, = ax.plot([], [], lw=2, label=target + ' ' + str(i))
			lines.append(line)

	plt.legend(handles=lines)
	
	ax.set_ylim(-2, 2)
	ax.set_xlim(0, numdata)
	ax.grid()

	ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=0,
		repeat=True)
	plt.show()
	
