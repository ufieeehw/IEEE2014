#!/usr/bin/env python
from __future__ import print_function
import blockSpotter as blockSpotter
import rospy
import numpy as np
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D, PoseStamped
from blockpub.msg import BlockPositions
## Sub sim_pose | pose ; camera pan/tilt ;
## Pub blockPositions ;
#TODO:
# - Enable blockSpotter to detect and use an attached camera
# - Enable blockSpotter to display an image if we're debugging

##CHECK:
# - I think I got the coordinate transform equation wrong - Replace with tf.transformations.*
# - Assumed positive right radians for pan

def transformCoordinates(pt, offset, angle):
	#pt = object position
	#offset = robot position
	#angle = robot angle
	x = (pt[0]*np.cos(angle)) - (pt[1]*np.sin(angle)) + offset[0]
	y = (pt[0]*np.sin(angle)) - (pt[1]*np.cos(angle)) + offset[1]
	return(x,y)
class blockHandler:
	def __init__(self):
		rospy.init_node('blockPublisher')
	
		self.pub = rospy.Publisher('BlockPositions', BlockPositions)
		self.poseSub = rospy.Subscriber('sim_pose', PoseStamped, self.newData)
		
		#self.tilt_sub = rospy.Subscriber('/tilt_controller/command', Float64)
		##Too complex to consider tilt - will simply assume that it is zeroed
		self.pan_sub = rospy.Subscriber('/pan_controller/command', Float64, self.setPan)
		
		##Immutable parameters
		self.course_length = (97 - 3/4 * 2) * 0.0254 #Courtesy of Lord Voight
		self.course_width = (49 - 3/4 * 2) * 0.0254
		self.cameraFwd = 5 * 0.0245
		self.cameraLeft = 0
		
		#Variable initialized parameters
		self.pan = 0
		
		##Variable non-initialized parameters
		#Robot Position ; Robot Orientation
		
		##Non constants stored as test constants
		#None!
		
	def setPan(self,data):
		self.pan = data
		#It seems like this is in radians
		return
	def newData(self, data):
		##Might be faster to handle position adjustment inside blockSpotter fcn

		relPositions,imxy = blockSpotter.spotBlocks()
		#RelPos <-> (Forward Displacement, Lateral Displacement Left Positive)
		if relPositions == None:

			rospy.loginfo("||| blockPublisher: No Blocks Detected") #-dbg
			rospy.sleep(0.1) #-dbg
			return

		robotPos = (data.pose.position.x,data.pose.position.y)
		#robotOrientation = data.pose.orientation.z

		quaternion = (
			data.pose.orientation.x,
			data.pose.orientation.y,
			data.pose.orientation.z,
			data.pose.orientation.w)

		rpy = tf.transformations.euler_from_quaternion(quaternion) #Roll, Pitch, Yaw: Radians
		yaw = rpy[2] + -self.pan #CHECK: Assuming positive right for pan 


		blockPos = np.add(robotPos,relPositions)

		msg = BlockPositions()
		msg.blocks = []

		
		
		for pos in relPositions:
			absolutePos = transformCoordinates(pos, robotPos, yaw)
			#absPos.append(absolutePos)
			absPos = Pose2D()
			absPos.x = absolutePos[0]
			absPos.y = absolutePos[1]
			absPos.theta = 0
			msg.blocks.append(absPos)
			

		self.pub.publish(msg)
		#rospy.sleep(0.2) #Is this necessary? Will it reduce stress on odroid?


		
		
if __name__=='__main__':
	try:
		blockHandleObj = blockHandler()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
