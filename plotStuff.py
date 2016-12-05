from usbctl import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sys import stdin
import select

def cutData(l, selector):
	return tuple([l[idx] for idx in selector])

def data_gen():
	while True:
		vals = ()
		for i in range(len(targets)):
			target = targets[i]
			l = getConfig(dev, configs, target);
			l = cutData(l, selectors[i])
			vals = vals + l
		yield vals

numdata = 50
xdata = list(range(0, numdata))
def run(data):
	# update the data
	datas = run.datas
	for i in range(0, len(data)):
		datas[i].append(data[i])
	
	if len(datas[0]) > numdata:
		for i in range(0, len(datas)):
			datas[i] = datas[i][-numdata:]
	
	mi = float('inf')
	ma = float('-inf')
	for d in datas:
		mi = min(mi, min(d)) - .1
		ma = max(ma, max(d)) + .1
	yMin, yMax = ax.get_ylim()
	if (yMin, yMax) != (mi, ma):
		ax.set_ylim(mi, ma)
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
	selectors = []
	fig, ax = plt.subplots()
	ax.set_xlim(0, numdata)
	
	for i in range(len(targets)):
		target = targets[i];
		selectorStr = ""
		if "[" in target and "]" in target:
			selectorStr = target[target.index("["):target.index("]")+1]
			target = target[0:target.index("[")]
		targets[i] = target
		l = getConfig(dev, configs, target);
		if selectorStr == "":
			selectorStr = "range(" + str(len(l)) + ")"
		selectors.append(eval(selectorStr))
		l = cutData(l, selectors[i])
		for i in range(0, len(l)):
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
	
