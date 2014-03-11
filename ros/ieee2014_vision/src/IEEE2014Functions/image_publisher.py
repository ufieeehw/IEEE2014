#!/usr/bin/env python

import rospy
import time
import numpy as np
import os, string
import cv2, cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
#To use this, run it, and then on the machine you'd like to view the images do
#rosrun image_view image_view image:=/Camera


class image_sender:
	def __init__(self, name="Camera"):

		#cv_image = bridge.imgmsg_to_cv(image_message, desired_encoding="passthrough")
		self.im_pub = rospy.Publisher(name, Image)
		self.bridge = CvBridge()	

	
	def send_message(self, cv_image):
		try:
			image_message = self.bridge.cv_to_imgmsg(cv.fromarray(cv_image),"bgr8")#,desired_encoding="passthrough")
			self.im_pub.publish(image_message)
		except CvBridgeError, e:
			print e

class image_reciever(object):
	def __init__(self, name="Camera"):
		self.im_sub = rospy.Subscriber(name, Image, convert)
		self.bridge = CvBridge()
		self.image = None
	def convert(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv(data,"bgr8")
			self.image = cv_image
		except CvBridgeError, e:
			print e
				
	#@property
	#def image(self):
	#	return cv_image = self.bridge.imgmsg_to_cv(data,"bgr8")
	
def main():
	imsend = image_sender()
	rospy.init_node('camera')
	
	cam = cv2.VideoCapture(0)	

	targetResolution = (640,360)
	#targetResolution = (960,720)
	cam.set(3,targetResolution[0]) #Width
	cam.set(4,targetResolution[1]) #Height
	

	while not rospy.is_shutdown():	
		try:
			ret, frame = cam.read()
			if(ret):
				#rospy.loginfo(frame.shape)
					
				imsend.send_message(frame)
			else:

				rospy.loginfo("FAIL Camera Read")
			#rospy.sleep(0.5)
			#rospy.spin()
		except KeyboardInterrupt:
			print "Shutting Down"
if __name__ == '__main__':
	main()

