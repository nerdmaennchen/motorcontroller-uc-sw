import usb.core
import usb.util
from array import array
import struct as st
import collections
import sys

def fetchConfig(dev):
	assert(dev.write(0x01, st.pack('=B', 1)) == 1)
	msg = dev.read(0x81, 64, 1000)
#	print("reading", msg)
	headerHelper = st.Struct('=BH')
	paramHelper = st.Struct('=H')
	(ep, l) = headerHelper.unpack(msg[0:3])
	msg = msg[3:]
#	print(ep, l)
	ret = collections.OrderedDict()
	idx = 0
	while len(msg) < l:
		msg = msg + dev.read(0x81, 64, 1000)
	while len(msg) > 0 and 0 in msg[2:]:
		(valLen,) = paramHelper.unpack(msg[:2])
		msg = msg[2:]
		nameLen = msg.index(0)+1
		nameStr = msg[:nameLen-1].tostring().decode("utf-8")
		msg = msg[nameLen:]
		formatStrLen = msg.index(0)+1
		formatStr = msg[:formatStrLen-1].tostring().decode("utf-8")
		msg = msg[formatStrLen:]
		
		ret[nameStr] = (idx, valLen, formatStr)
		idx = idx + 1
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
def customInt(s):
	if len(s) > 2:
		if s[0:2] == "0x":
			return int(s[2:], 16)
		elif s[0:2] == "0b":
			return int(s[2:], 2)
	raise ValueError("")
def autoconvert(s):
    for fn in (boolify, customInt, int, float):
        try:
            return fn(s)
        except:
            pass
    return s
    
def setConfig(dev, configs, target, values, verbose=True):
	if target in configs:
		size = configs[target][1]
		t = configs[target]
		packedValues = st.pack(t[2], *values)
		if size == len(packedValues):
			dev.write(0x01, st.pack('=BB', 1, t[0]) + packedValues)
			if verbose:
				print("set " + target + " to " + str(values))
		else:
			print("wrong size!\n expected: " + str(t[2]) + " got: " + str(values))
	else:
		print(target + " not in " + str(configs))

def getConfig(dev, configs, target):
	if target in configs:
		t = configs[target]
		dev.write(0x01, st.pack('=BB', 1, t[0]))
		size = t[1] + 3
		msg = array('B')
		while len(msg) < size:
			msg += dev.read(0x81, 64, 10)
		return st.unpack(t[2], msg[3:])
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

	dev.set_configuration(1)
	
	if len(sys.argv) == 2 and sys.argv[1] == "flush":
		flush(dev)
		exit(0)
	
	configs = fetchConfig(dev)
	if len(sys.argv) == 1:
		print(configs)
		exit(0)
		
#format: [progname] set target [formatstr (params)*]
	if len(sys.argv) >= 2 and sys.argv[1] == "set":
		target = sys.argv[2]
		args = []
		msg = ()
		if len(sys.argv) >= 3:
			for s in sys.argv[3:]:
				args.append(autoconvert(s))
			msg = tuple(args)
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
		
	
