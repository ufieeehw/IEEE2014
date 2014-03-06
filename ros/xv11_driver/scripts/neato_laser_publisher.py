#!/usr/bin/python
from __future__ import division

import roslib
roslib.load_manifest('xv11_driver')

import threading
import serial
import rospy
import math
import struct

from std_msgs.msg import Header
from xv11_driver.msg import LaserMeasurements
from sensor_msgs.msg import LaserScan

rospy.init_node('xv11_driver')

# Create an xv11Driver object for some added structure
class xv11Driver(object):
	# This gets called when the xv11Driver object is instantiated
	def __init__(self, port):
		self._serial = serial.Serial(port, 115200)
	
	def read_packet(self):
		data = ''
		while True:
			data += self._serial.read(1)
			if data[0] != '\xfa':
				data = ''
				continue
  			if len(data) == 22:
				start, index, speed, data0, data1, data2, data3, checksum = struct.unpack('<BBH4s4s4s4sH', data)
				index -= 0xA0
				index = (index + 45-2) % 90
				msg = LaserMeasurements()
				msg.ranges = []
				msg.intensities = []
				
				for i, subdata in enumerate([data0, data1, data2, data3]):
					angle = index * 4 + i
					distance, strength = struct.unpack('<HH', subdata)
					invalid_data = bool(distance & 0x8000); distance &= ~0x8000
					strength_warning = bool(distance & 0x4000); distance &= ~0x4000
					if invalid_data: distance = -1
					msg.ranges.append(distance/1000)
					msg.intensities.append(strength)

				data = ''
				msg.header.stamp = rospy.Time.now()
				msg.packet_index = index
				msg.angle_min = (index * 4)*(math.pi/180.0)
				msg.angle_max = (index * 4 + 3)*(math.pi/180.0)
				msg.angle_increment = 4.0*(math.pi/180.0)
				#msg.time_increment = ((60.0/360.0)*(64.0/speed))
                                msg.time_increment = 1e-6
				msg.range_min = 0.06
				msg.range_max = 5.0
				return msg
	
class LaserScanGenerator(object):
	def __init__(self):
		self.msg = None
	
	def init_msg(self):
		self.msg = LaserScan()
		self.msg.header.frame_id = frame_id
		self.msg.header.stamp = rospy.Time.now()
		self.msg.angle_min = math.radians(0)
		self.msg.angle_max = math.radians(359)
		self.msg.angle_increment = math.radians(1)
		self.msg.time_increment = 0
		self.msg.scan_time = 0
		self.msg.range_min = 0.06
		self.msg.range_max = 5.0
		self.msg.ranges = []
		self.msg.intensities = []

	def generate_packet(self, measurement):
		if measurement.packet_index == 0:
			self.init_msg()
		if self.msg is None: return None
		self.msg.scan_time += measurement.time_increment * 4
		self.msg.ranges.extend(measurement.ranges)
		self.msg.intensities.extend(measurement.intensities)

		if measurement.packet_index == 89:
			self.msg.time_increment = (self.msg.scan_time)/90.0
			return self.msg
		
		else: return None

frame_id = rospy.get_param('~frame_id')

laser_object = xv11Driver(rospy.get_param('~port'))
laser_scan_generator = LaserScanGenerator()

#lmp: LaserMeasurements publisher
#lsp: LaserScan publisher
lmp = rospy.Publisher('laser_measurements', LaserMeasurements)
lsp = rospy.Publisher('laser_scan', LaserScan)

while not rospy.is_shutdown():
	lmp_msg = laser_object.read_packet()
	lsp_msg = laser_scan_generator.generate_packet(lmp_msg)
	
	try:
		lmp.publish(lmp_msg)
		if lsp_msg is not None:
			lsp.publish(lsp_msg)
	except:
		import traceback
		traceback.print_exc()

