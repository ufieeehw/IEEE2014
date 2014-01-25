#!/usr/bin/python
from __future__ import division

import roslib
roslib.load_manifest('joystick_command')

import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from std_msgs.msg import String

rospy.init_node('joystick_command')


def process_joy_data_callback(data):
	linear_x = data.axes[1] * max_linear_velocity
	linear_y = data.axes[0] * max_linear_velocity
	angular_z = data.axes[3] * max_angular_velocity
	seq = data.header.seq

	twist_stamped_msg = TwistStamped()
	twist_stamped_msg.twist = Twist()
	twist_stamped_msg.twist.linear = Vector3()
	twist_stamped_msg.twist.angular = Vector3()

	twist_stamped_msg.twist.linear.x = linear_x
	twist_stamped_msg.twist.linear.y = linear_y
	twist_stamped_msg.twist.linear.z = 0
	
	twist_stamped_msg.twist.angular.x = 0
	twist_stamped_msg.twist.angular.y = 0
	twist_stamped_msg.twist.angular.z = angular_z

	twist_stamped_msg.header.seq = seq
	twist_stamped_msg.header.frame_id = '/base_link'
	twist_stamped_msg.header.stamp = data.header.stamp
	pub.publish(twist_stamped_msg)

#These values are hardcoded for now.
max_linear_velocity = 0.5 #rospy.get_param('max_linear_velocity')
max_angular_velocity = 5 #rospy.get_param('max_angular_velocity')

pub = rospy.Publisher('/twist', TwistStamped)
rospy.Subscriber('/joy', Joy, process_joy_data_callback)

rospy.spin()	

