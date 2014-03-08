#!/usr/bin/env python

import roslib
roslib.load_manifest('ieee2014_tf_broadcaster')

import rospy

import tf

from geometry_msgs.msg import PoseStamped, Transform
from dynamixel_msgs.msg import JointState

last_pan_position = 0.0
last_tilt_position = 0.0
tf_broad = tf.TransformBroadcaster()

def handle_lidar_pose(msg):
    translation = (msg.pose.position.x, msg.pose.position.y, 0)
    rotation = (msg.pose.orientation.x,msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    time = rospy.Time.now()

    tf_broad.sendTransform(translation, rotation, time, "/robot", "/course")

def read_pan_position(msg):
    global last_pan_position
    last_pan_position = msg.current_pos

    translation = (0, 0, 0.15334)
    # center of rotation of tilt servo is 2.4744 inches directly above the surface of the top plate
    # which is then 3 9/16ths of an inch on top of the ground, (2.4744 + 3 + 9/16) inches = 0.15334 meters
    rotation = tf.transformations.quaternion_from_euler(0, -last_tilt_position, last_pan_position) #rpy
    time = rospy.Time.now()
    tf_broad.sendTransform(translation, rotation, time, "/tilt_bracket", "/robot")

def read_tilt_position(msg):
    global last_tilt_position
    last_tilt_position = msg.current_pos

    translation = (0, 0, 0.15334)
    # center of rotation of tilt servo is 2.4744 inches directly above the surface of the top plate
    # which is then 3 9/16ths of an inch on top of the ground, (2.4744 + 3 + 9/16) inches = 0.15334 meters
    rotation = tf.transformations.quaternion_from_euler(0, -last_tilt_position, last_pan_position) #rpy
    time = rospy.Time.now()
    tf_broad.sendTransform(translation, rotation, time, "/tilt_bracket", "/robot")

if __name__ == '__main__':
    rospy.init_node('ieee2014_tf_broadcaster')

    sim = rospy.get_param('~sim', "N")
    if(sim == 'Y'):
        prefix = 'sim_'
    else:
        prefix = ''

    rospy.Subscriber('/{0}pose'.format(prefix), PoseStamped, handle_lidar_pose)
    rospy.Subscriber('/{0}pan_controller/state'.format(prefix), JointState, read_pan_position)
    rospy.Subscriber('/{0}tilt_controller/state'.format(prefix), JointState, read_tilt_position)

    rospy.spin()
