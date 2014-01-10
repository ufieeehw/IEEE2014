#!/usr/bin/python

import roslib
roslib.load_manifest('xv11_driver')

import threading
import serial
import rospy
import math


from std_msgs.msg import Header
from xv11_driver.msg import *

rospy.init_node('xv11_driver')

class xv11Driver(object):
	
	def __init__(self, port):
		self._serial = serial.(port, 115200)
	
	def read(self):
		data = ''
		while True:
			data += s.read(1)
			if data[0] != '\xfa':
				data = ''
				continue
  			if len(data) == 22:
				start, index, speed, data0, data1, data2, data3, checksum = struct.unpack('<BBH4s4s4s4sH', data)
				
				for i, subdata in enumerate([data0, data1, data2, data3]):
					angle = index * 4 + i
					distance, strength = struct.unpack('<HH', subdata)
					invalid_data = bool(distance & 0x8000); distance &= ~0x8000
					strength_warning = bool(distance & 0x4000); distance &= ~0x4000
					if invalid_data: continue
				
				data = ''

				angle_min = (index * 4)*(math.pi/180.0)
				angle_max = (index * 4 + 3)*(math.pi/180.0)
				angle_increment = 4.0*(math.pi/180.0)
				time_increment = ((60.0/360.0)*(64.0/speed))
				scan_time = # this is where we left off.
				range_min = 0.06
				range_max = 5.0

