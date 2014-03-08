#!/usr/bin/env python

import roslib
roslib.load_manifest('ieee2014_tf_target_solver')

import rospy
import numpy
import math
import tf

from geometry_msgs.msg import PoseStamped
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

def spherical_from_cartesian(x, y, z):
    r = math.sqrt(x**2 + y**2 + z**2)               #r
    elev = math.atan2(z, math.sqrt(x**2 + y**2))    #theta
    az = math.atan2(y, x)                           #phi

    return (r, elev, az)

if __name__ == '__main__':
    rospy.init_node('ieee2014_tf_target_solver')

    tf_listener = tf.TransformListener()

    sim = rospy.get_param('~sim', "N")
    if(sim == 'Y'):
        prefix = 'sim_'
    else:
        prefix = ''

    pan_pub = rospy.Publisher('/{0}pan_controller/command'.format(prefix), Float64)
    tilt_pub = rospy.Publisher('/{0}tilt_controller/command'.format(prefix), Float64)
    enabled = True

    hz = 10.0
    dt = 1.0/hz

    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        try:
            (translation, rotation) = tf_listener.lookupTransform('/gun_forward', '/target', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        (r, elev, az) = spherical_from_cartesian(*translation)

        goal_gun_pan = az
        goal_gun_tilt = elev

        if enabled is True:
            if goal_gun_pan < -(numpy.pi / 3):
                goal_gun_pan += 2 * numpy.pi
            pan_msg = Float64()
            pan_msg.data = goal_gun_pan
            pan_pub.publish(pan_msg)

            tilt_msg = Float64()
            tilt_msg.data = goal_gun_tilt
            tilt_pub.publish(tilt_msg)
        rate.sleep()
