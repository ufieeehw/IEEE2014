import serial
import struct
import time

class msgTypes(object):
	ACKValid = chr(0)
	ACKInvalid = chr(1)
	EchoRequest = chr(2)
	EchoReply = chr(3)
	PIDsetSpeed = chr(4)
	PIDgetOdometry = chr(5)
	PIDsetMultiplier = chr(6)
	PIDgetMultiplier = chr(7)
	PIDgetSpeed = chr(8)
	TCSGetRawData = chr(9)


class xmegaTester(object):

	def __init__(self):
		self.ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
		print self.ser.name
		self.msgType = msgTypes()

	def sendMsg(self, msgType, msgLen, msg):
		messageToSend = '^^^' + chr(msgLen + 1) + msgType + msg + '\0'
		self.ser.write(messageToSend)

	def readMsg(self):
		self.ser.read(3)
		msg_len = ord(self.ser.read())
		msg_type = self.ser.read()
		msg = ''
		for i in range(0, msg_len - 1):
			msg += self.ser.read()
		self.sendAck()
		return msg

	def sendAck(self):
		self.sendMsg(self.msgType.ACKValid, 0, '')
	
	def echo(self):
		temp = raw_input("Enter text to be echoed: ")
		print "sent: \t\t", temp.encode("hex"), ' ', temp
		self.sendMsg(self.msgType.EchoRequest, len(temp), temp)
		ret_msg = self.readMsg()
		print "recieved: \t",  ret_msg.encode("hex"), ' ', ret_msg
		return
	
	def setWheelSpeeds(self):
		wheelStruct = ''
		wheel1 = int(float( raw_input("Wheel1: ")) * 1000.0)
		wheel2 = int(float( raw_input("Wheel2: ")) * 1000.0)
		wheel3 = int(float( raw_input("wheel3: ")) * 1000.0)
		wheel4 = int(float( raw_input("wheel4: ")) * 1000.0)

		print wheel1, '\t', wheel2, '\t', wheel3, '\t', wheel4

		print hex(wheel1), '\t', hex(wheel2), '\t', hex(wheel3), '\t', hex(wheel4)
		wheelStruct = struct.pack('<llll', wheel1,wheel2,wheel3,wheel4)
		
		self.sendMsg(self.msgType.PIDsetSpeed, len(wheelStruct), wheelStruct)
		return

	def getWheelSpeeds(self, loop):
		while True:
			self.sendMsg(self.msgType.PIDgetSpeed, 0, '')
			temp = self.readMsg()
			wheel1, wheel2, wheel3, wheel4 = struct.unpack("<llll", temp)
			
			wheel1 /= 1000.0
			wheel2 /= 1000.0
			wheel3 /= 1000.0
			wheel4 /= 1000.0
			print wheel1, '\t', wheel2, '\t', wheel3, '\t', wheel4
			time.sleep(.2)
			if loop is not True: break
		return

	def getOdometry(self, loop):
  		while True:
			self.sendMsg(self.msgType.PIDgetOdometry, 0, '')
			vals = self.readMsg()
			wheel1, wheel2, wheel3, wheel4 = struct.unpack("<llll", vals)
			print wheel1, '\t', wheel2, '\t', wheel3, '\t', wheel4
			if loop is not True:
				break
		return

	def helpCommands(self):
		print "Here are a list of commands thus far to run: \n"
		print "a: send ack message"
		print "ws: send a new set of wheel speeds"
		print "gwss: get a single reading of the wheel speeds"
		print "gwsl: begin reading wheel speeds in loop"
		print "ods: get single odometry reading"
		print "odl: begin reading odometry in loop"
		print "e: A command which will simply echo out the text that was entered."
		print "q: exit the program"
		return


xm = xmegaTester()

while True:

	command = raw_input("Type a command\n")
	
	if command == "e":
		xm.echo()
	elif command == "h":
		xm.helpCommands()
	elif command == 'q':
		break
	elif command == 'ack':
		xm.sendAck()
	elif command == 'ws':
		xm.setWheelSpeeds()
	elif command == 'odl':
		xm.getOdometry(True)
	elif command == 'ods':
		xm.getOdometry(False)
	elif command == 'gwss':
		xm.getWheelSpeeds(False)
	elif command == 'gwsl':
		xm.getWheelSpeeds(True)
	else:
		print "That's not a legal command.\n"


