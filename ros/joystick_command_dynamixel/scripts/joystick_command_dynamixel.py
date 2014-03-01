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
	tilt_axis = data.axes[1]
	if tilt_axis > 0:
		tilt_angle = abs(tilt_axis) * highest_tilt_angle #pushing forward on the stick gives a positive value
	else:
		tilt_angle = abs(tilt_axis) * lowest_tilt_angle

	pan_axis = data.axes[0]
	if pan_axis > 0:
		pan_angle = abs(pan_axis) * max_left_pan_angle # pushing forward on the stick gives a positive value
	else:
		pan_angle = abs(pan_axis) * max_right_pan_angle 

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

tilt_pub = rospy.Publisher('/tilt_controller/command', Float64)
pan_pub = rospy.Publisher('/pan_controller/command', Float64)
rospy.Subscriber('/joy', Joy, process_joy_data_callback)

rospy.spin()	

