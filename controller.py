import serial
import struct
import time

class Sparki:

	START = struct.pack('!B',0)
	SPOT2 = struct.pack('!B',1)
	SPOT3 = struct.pack('!B',2)
	SPOT4 = struct.pack('!B',3)
	SPOT1 = struct.pack('!B',4)

	portName = None
	serialPort = None

	def __init__(self, comPort):
		# Error check if comPort is a string
		self.portName = comPort
		self.serialPort = serial.Serial()

	"""
	Returns a boolean as to connection status
	"""
	def connect(self):
		print "Trying to Connect"
		self.serialPort.port = self.portName
		self.serialPort.baudrate = 9600
		self.serialPort.parity = 'N'
		self.serialPort.writeTimeout = 0
		# Might want to set other settings as well, to be safe
		self.serialPort.open()
		# Can throw ValueErrors on failure
		if (self.serialPort.isOpen()):
			print "Connected"
			return True
		else:
			return False
	
	def disconnect(self):
		print "Disconnecting..."
		self.serialPort.close()
		if (self.serialPort.isClosed()):
			print "Disconnected"
	
	def start(self):
		# Should be open port
		self.serialPort.write(self.START)
	
	def moveSpot2(self):
		# Should be open port
		self.serialPort.write(self.SPOT2)
	
	def moveSpot3(self):
		# Should be open port
		self.serialPort.write(self.SPOT3)
	
	def moveSpot4(self):
		# Should be open port
		self.serialPort.write(self.SPOT4)
	
	def moveSpot1(self):
		# Should be open port
		self.serialPort.write(self.SPOT1)
	
	"""
	Returns an int of the ping value or -1
	"""
	def ping(self):
		# Should be open port
		distance = -1
		self.serialPort.write(self.REQ_PING)
		distance = int(self.readString())
		return distance
	
	"""
	Reads return strings from Sparki that EOL with '*'
	Returns the string
	"""
	def readString(self):
		# Should be open port
		# Validate that conversion to string works
		output = ""
		last = ""
		while (True):
			last = str(self.serialPort.read(size=1))
			if (last == "*"):
				break
			output = output + last
		return output
	
	def delay(self, time):
		# Should validate time is int in milliseconds
		time.sleep(time/1000)
		
sparki2 = Sparki("COM9")
sparki2.connect()
#print(sparki2.ping())
sparki2.moveSpot3()
sparki2.moveSpot1()
#start can be missed due to latency, must be repeated
sparki2.start()
sparki2.start()
sparki2.start()
