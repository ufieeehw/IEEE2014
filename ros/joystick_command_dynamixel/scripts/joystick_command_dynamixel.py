#!/usr/bin/python
from __future__ import division

import roslib
roslib.load_manifest('joystick_command_dynamixel')

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from std_msgs.msg import Float64

rospy.init_node('joystick_command_dynamixel')


def process_joy_data_callback(data):
	global tilt_angle
	global pan_angle

	tilt_axis = data.axes[5]
	if tilt_axis == 1.0:
		tilt_angle -= one_degree #pushing up on the d-pad gives a +1.0
	elif tilt_axis == -1.0:
		tilt_angle += one_degree #and down gives -1.0

	if tilt_angle > lowest_tilt_angle:
		tilt_angle = lowest_tilt_angle

	if tilt_angle < highest_tilt_angle:
		tilt_angle = highest_tilt_angle

	pan_axis = data.axes[4]
	if pan_axis == 1.0:
		pan_angle += one_degree #pushing left on the d-pad gives a +1.0
	elif pan_axis == -1.0:
		pan_angle -= one_degree #and right gives -1.0

	if pan_angle > max_left_pan_angle:
		pan_angle = max_left_pan_angle

	if pan_angle < max_right_pan_angle:
		pan_angle = max_right_pan_angle

	if data.buttons[0] == 1:
		pan_angle = 0
		tilt_angle = 0

	tilt_msg = Float64()
	tilt_msg.data = tilt_angle

	tilt_pub.publish(tilt_msg)

	pan_msg = Float64()
	pan_msg.data = pan_angle

	pan_pub.publish(pan_msg)


#These values are hardcoded for now.
lowest_tilt_angle = 0.255
highest_tilt_angle = -0.560

max_left_pan_angle = 3.088
max_right_pan_angle = -3.19

tilt_angle = 0
pan_angle = 0 #by default, we'll be centered

one_degree = 0.0174533 # 1 degree in radians

tilt_pub = rospy.Publisher('/tilt_controller/command', Float64)
pan_pub = rospy.Publisher('/pan_controller/command', Float64)
rospy.Subscriber('/joy', Joy, process_joy_data_callback)

rospy.spin()	

