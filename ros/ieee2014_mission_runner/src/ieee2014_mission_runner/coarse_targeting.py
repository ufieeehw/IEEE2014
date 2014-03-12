import math

import numpy

import rospy
from std_msgs.msg import Float64
import tf

def spherical_from_cartesian(x, y, z):
    r = math.sqrt(x**2 + y**2 + z**2)               #r
    elev = math.atan2(z, math.sqrt(x**2 + y**2))    #theta
    az = math.atan2(y, x)                           #phi

    return r, elev, az

def think(xy_pos):
    pos = numpy.array([xy_pos[0], xy_pos[1], 0.15334 + 0.08067])
    target_pos = numpy.array([-1.21285, 0.0, 0.6731])
    
    r, elev, az = spherical_from_cartesian(*target_pos - pos)
    
    return dict(pan=az, tilt=elev)
