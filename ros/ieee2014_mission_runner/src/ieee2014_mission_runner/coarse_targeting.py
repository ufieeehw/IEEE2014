import math

import numpy

import rospy
from std_msgs.msg import Float64
import tf

import fine_targeting

def spherical_from_cartesian(x, y, z):
    r = math.sqrt(x**2 + y**2 + z**2)               #r
    elev = math.atan2(z, math.sqrt(x**2 + y**2))    #theta
    az = math.atan2(y, x)                           #phi

    return r, elev, az

def think(xy_pos):
    pos = numpy.array([xy_pos[0], xy_pos[1], fine_targeting.camera_height])
    
    r, elev, az = spherical_from_cartesian(*fine_targeting.target_center - pos)
    
    return dict(pan=az, tilt=elev)
