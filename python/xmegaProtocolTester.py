import serial
import struct
ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
print ser.name

def echoText():
	text = raw_input("Enter Text to be echoed: ")
	text = '^^^' + chr( len(text + chr(2)) )+ chr(2) + text + '\0'
	hexStr = text.encode("hex")

	print hexStr
	ser.write(text)

	textback = ser.read(100)
	hexStrBack = textback.encode("hex")
	print hexStrBack
	print textback
#	print ser.read(20)
#	ser.read(3)
#	length = int(ser.read())
#	messageType = int(ser.read())
#	message = ser.read(length - 1)
	
#	print message
	
	returnText = '^^^' + chr(1) + chr(0)
	ser.write(returnText)

	return

def sendAck():
	returnText = '^^^' + chr(1) + chr(0)
	ser.write(returnText)
	return

def setWheelSpeeds():
	wheelStruct = ''
	wheel1 = int(raw_input("Wheel1: "))
	wheel2 = int(raw_input("Wheel2: "))
	wheel3 = int(raw_input("wheel3: "))
	wheel4 = int(raw_input("wheel4: "))

	wheelStruct = struct.pack('bbbb', wheel1,wheel2,wheel3,wheel4)

	test = '^^^' + chr(5) + chr(4) + wheelStruct + '\0'
	hexStr = test.encode("hex")

	print hexStr
	print test

	ser.write(test)

	return

def getOdometry():
  while True:
	message = '^^^' + chr(1)+ chr(5)+ '\0'
	#print message.encode("hex")

	ser.write(message)
#	raw_input("waiting")
	ser.read(5)
	vals = ser.read(16)
#	print vals.encode("hex")
	wheel1, wheel2, wheel3, wheel4 = struct.unpack(">llll", vals)
	print wheel1, ' ', wheel2, ' ', wheel3, ' ', wheel4

	sendAck()

	#return

def helpCommands():
	print "Here are a list of commands thus far to run: \n"
	print "echoText: A command which will simply echo out the text that was entered.\n"
	print "quit: exit the program"
	return

while True:

	command = raw_input("Type a command\n")
	
	if command == "echo":
		echoText()
	elif command == "help":
		helpCommands()
	elif command == 'quit':
		break
	elif command == 'ack':
		sendAck()
	elif command == 'ws':
		setWheelSpeeds()
	elif command == 'od':
		getOdometry()
	else:
		print "That's not a legal command.\n"


