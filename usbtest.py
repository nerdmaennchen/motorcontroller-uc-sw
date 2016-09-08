import usb.core
import usb.util
from array import array
import struct as st
import collections
import sys

def fetchConfig(dev):
	dev.write(0x01, st.pack('=B', 1))
	msg = dev.read(0x81, 64)
#	print("reading", msg)
	headerHelper = st.Struct('=BH')
	paramHelper = st.Struct('=H')
	(ep, l) = headerHelper.unpack(msg[0:3])
	msg = msg[3:]
#	print(ep, l)
	ret = collections.OrderedDict()
	idx = 0
	while l > 0:
		if len(msg) == 0 or not 0 in msg[2:]:
			msg = msg + dev.read(0x81, 64)
#			print("reading", msg)
		else:
			while len(msg) != 0 and 0 in msg[2:]:
				(valLen,) = paramHelper.unpack(msg[:2])
				msg = msg[2:]
				strLen = msg.index(0)+1
				valName = msg[:strLen-1].tostring().decode("utf-8")
				ret[valName] = (idx, valLen)
				idx = idx + 1
				l = l - 2 - strLen
				msg = msg[strLen:]
	return ret

def argsToMsg(args):
	if len(args) == 0:
		return bytearray()
	else:
		return st.pack('=i', args[0]) + argsToMsg(args[1:])
		

def boolify(s):
    if s == 'True':
        return True
    if s == 'False':
        return False
    raise ValueError("")
def autoconvert(s):
    for fn in (boolify, int, float):
        try:
            return fn(s)
        except:
            pass
    return s
    
def setConfig(dev, configs, target, values):
	if target in configs:
		size = configs[target][1]
		if size == len(values):
			dev.write(0x01, st.pack('=BB', 1, configs[target][0]) + values)
			print("set " + target + " to " + str(values))
		else:
			print("wrong size!\n expected len: " + str(size) + " got: " + str(len(values)))
	else:
		print(target + " not in " + str(configs))

def getConfig(dev, configs, target):
	if target in configs:
#		print("sending: " + str(st.pack('=BB', 1, configs[target][0]))) 
		dev.write(0x01, st.pack('=BB', 1, configs[target][0]))
		size = configs[target][1] + 3
#		print("expecting " + str(size) + " bytes")
		msg = array('B')
		while len(msg) < size:
			msg += dev.read(0x81, 64, 10000)
#			print(msg)
		return msg[3:]
	else:
		print(target + " not in " + str(configs))
		
def flush(dev):
	try:
		while True:
			msg = dev.read(0x81, 64)
			print("flushed" + str(msg))
			if len(msg) == 0:
				break;
	except:
		pass
		
def flush_(dev):
	while True:
		try:
			msg = dev.read(0x81, 64)
			print("flushed" + str(msg))
			if len(msg) == 0:
				break;
		except:
			pass

if __name__ == "__main__":
	dev = usb.core.find(idVendor=0xcafe, idProduct=0xcafe)
	if dev is None:
		raise ValueError('Device not found')

	dev.set_configuration()
	
	if len(sys.argv) == 2 and sys.argv[1] == "flush":
		flush(dev)
		exit(0)
		
	configs = fetchConfig(dev)
	print(configs)

#format: [progname] set target [formatstr (params)*]
	if len(sys.argv) >= 3 and sys.argv[1] == "set":
		target = sys.argv[2]
		args = []
		msg = bytearray()
		if len(sys.argv) >= 5:
			for s in sys.argv[4:]:
				args.append(autoconvert(s))
			print(args)
			msg = st.pack(sys.argv[3], *args)
		setConfig(dev, configs, target, msg)
		
	if len(sys.argv) >= 3 and sys.argv[1] == "get":
		target = sys.argv[2]
		c = getConfig(dev, configs, target)
		if len(sys.argv) == 4:
			print(st.unpack(sys.argv[3], c))
		else:
			print(c)
	
#	flush(dev)
	exit(0)
		
	
