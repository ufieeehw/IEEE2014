#!/usr/bin/python

import roslib
roslib.load_manifest('xmega_connector')

import threading
import serial
import rospy
import math


from std_msgs.msg import Header
from xmega_connector.msg import XMEGAPacket
from xmega_connector.srv import * #Echo, EchoRequest, EchoResponse

rospy.init_node('xmega_connector')

class XMEGAConnector(object):
	
	def __init__(self, port):
		self._serial = serial.Serial(port, 19200)
	
	def read_packet(self):
		packet = XMEGAPacket()
		data = ''
		start_char_count = 0
		message_len = 0
		while True:
			if start_char_count < 3:
				if self._serial.read(1) is '^':
					start_char_count += 1
					continue
			else:
				message_len = ord(self._serial.read(1))
				print "length read: ", hex(message_len)
				data = self._serial.read(message_len)

				packet.header.stamp = rospy.Time.now()
				packet.msg_type = data[0]
				print "type read: ", packet.msg_type
				packet.msg_length = message_len
				packet.msg_body = data[1:]
				print "body read: ", packet.msg_body
				return packet

	def send_packet(self, packet_to_send):
		length = packet_to_send.msg_length
		type = packet_to_send.msg_type
		message = packet_to_send.msg_body

		self._serial.write("^^^")
		self._serial.write(chr(length))
		print "length sent: ", hex(length)
		self._serial.write(chr(type))
		print "type sent: ", hex(type)
		self._serial.write(message)
		self._serial.write('\0') #need to send an additional character to get XMEGA to finish reading 

connector_object = XMEGAConnector(rospy.get_param('~port'))

xmega_lock = threading.Lock()


def echo_service(echo_request):
	xmega_lock.acquire(True)  # wait until lock can be acquired before proceeding
	print "about to echo: ", echo_request.send

	packet = XMEGAPacket()
	packet.msg_body = echo_request.send
	packet.msg_type = 0x02  # 0x02 echo request, 0x03 echo reply
	packet.msg_length = len(packet.msg_body) + 1  

	connector_object.send_packet(packet)
	print "sent echo request packet"
	response_packet = connector_object.read_packet()
	print "received echo response packet"
	
	ack_packet = XMEGAPacket()
	ack_packet.msg_type = 0x00
	ack_packet.msg_length = 0x01
	connector_object.send_packet(ack_packet)
	print "sent ack packet"

	service_response = EchoResponse()
	service_response.recv = response_packet.msg_body
	print "received: ", service_response.recv
	xmega_lock.release()
	return service_response

def set_wheel_speed_service(ws_req):
	xmega_lock.acquire(True)
	packet = XMEGAPacket()
	packet.msg_type = 0x04
	packet.msg_body = struct.pack('bbbb', ws_req.wheel1, ws_req.wheel2, 
		ws_req.wheel3, ws_req.wheel4)
	packet.msg_length = len(packet.msg_body) + 1

	connector_object.send_packet(packet)

	xmega_lock.release()
	return SetWheelSpeedsResponse()

rospy.Service('~echo', Echo, echo_service)
rospy.Service('~set_wheel_speeds', SetWheelSpeeds, set_wheel_speed_service)

while not rospy.is_shutdown():

	rospy.spin()



