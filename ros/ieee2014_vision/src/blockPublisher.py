#!/usr/bin/env python
import blockSpotter  #This does all of the block locating
import numpy as np
import string
import rospy
import cv2
import tf
import os
import sys
import traceback
from geometry_msgs.msg import Pose2D, PoseStamped
from dynamixel_msgs.msg import JointState
from ieee2014_vision.msg import BlockPositions
from std_msgs.msg import Float64
try:
	from IEEE2014Functions import image_publisher as impub
except:
	traceback.print_exc(file=sys.stdout)
	print "Image Publisher Import Failed"

## Sub :: sim_pose | pose ; camera pan/tilt ;
## Pub :: blockPositions ;
#TODO:
# - Enable blockSpotter to detect and use an attached camera
# - Enable blockSpotter to display an image if we're debugging

##CHECK:
# - I think I got the coordinate transform equation wrong - Replace with tf.transformations.*
# - Assumed positive right radians for pan
def getNodeList():
	nodeList = os.popen('rosnode list').read()
	nodes = string.split(string.replace(nodeList,'\n', ' '))
	return nodes

def transformCoordinates(pt, offset, angle):
	#pt = object position
	#offset = robot position
	#angle = robot angle
	x = (pt[0]*np.cos(angle)) - (pt[1]*np.sin(angle)) + offset[0]
	y = (pt[0]*np.sin(angle)) - (pt[1]*np.cos(angle)) + offset[1]
	return(x,y)

class blockHandler:
	def __init__(self):
		rospy.init_node('block_publisher')
		
		##Immutable parameters
		self.course_length = (97 - 3/4 * 2) * 0.0254 #Courtesy of Lord Voight
		self.course_width = (49 - 3/4 * 2) * 0.0254
		self.cameraFwd = 5.25 * 0.0254 #Cam v2 (not precise)
		#self.cameraFwd = 5.25 I think this is the most accurate?
		self.cameraLeft = 0
		self.start_pos = (-self.course_length/2 + (6*0.0254), -self.course_width/2 + ((5+5.25+0.75)*0.0254))	
		#Variable initialized parameters
		self.pan = 0
		
		##Variable non-initialized parameters
		#::Robot Position ; Robot Orientation
		
		##Non constants stored as test constants
		#None!
		
		##Begin publisher and subscribers
		sim = rospy.get_param('/block_publisher/sim', "N")
		self.publish_images = rospy.get_param('/block_publisher/debug', "N")
		self.pub = rospy.Publisher('block_positions', BlockPositions)
		
		#Assume tilt is zerod when using blockSpotter.
		#figure out how to read servo status correctly
		self.pan_sub = rospy.Subscriber('/pan_controller/state', JointState, self.setPan)
		
		if(sim == 'Y'):
			if not '/ieee2014_simulator' in getNodeList():
				os.system('rosrun ieee2014_simulator ieee2014_simulator &')
			rospy.loginfo("\nNOTE: block_publisher Using simulation pose data")
			self.poseSub = rospy.Subscriber('sim_pose', PoseStamped, self.newData)
		
		elif(sim == 'N'):
			rospy.loginfo("\nNOTE: block_publisher NOT simulating")
			self.poseSub = rospy.Subscriber('pose', PoseStamped, self.newData)
		if(self.publish_images == 'Y'):
			self.image_pub = impub.image_sender()
			
	def __enter__(self):
		self.cam = cv2.VideoCapture(-1)
		
		targetResolution = (640,360)
		
		self.cam.set(3,targetResolution[0]) #Width
		self.cam.set(4,targetResolution[1]) #Height
		return self
	
	def __exit__(self, *args):
		self.cam.release()
	def setPan(self,data):
		self.pan = data.current_pos
	
		#It seems like this is in radians : CHECK
		return
	def newData(self, data):
		##Might be faster to handle position adjustment inside blockSpotter fcn
		image = None
		try:
			ret, image = self.cam.read()
		except:
			rospy.loginfo("Image Read from camera failed\nUsing simulated data")
			
		if(ret):
			relPositions,imxy = blockSpotter.spotBlocks(image)
			if self.publish_images == 'Y':
				for k in range(len(imxy)):
					xy = imxy[k]
					cv2.circle(image, xy,5,(50,220,50),thickness=-1)
					pos = relPositions[k]
					cv2.putText(image, str(pos[0])+ ', ' +str(pos[1]), xy, cv2.FONT_HERSHEY_PLAIN, 0.8, (0,0,255), thickness=1)
				
				self.image_pub.send_message(image)
		else:
			relPositions, imxy = blockSpotter.spotBlocks()
		#RelPos <-> (Forward Displacement, Lateral Displacement Left Positive)
		if relPositions == None:

			rospy.loginfo("||| block_publisher: No Blocks Detected") #-dbg
			rospy.sleep(0.1) #-dbg
			return

		#robotPos = (data.pose.position.x,data.pose.position.y)
		robotPos = self.start_pos
		quaternion = (
			data.pose.orientation.x,
			data.pose.orientation.y,
			data.pose.orientation.z,
			data.pose.orientation.w)

		rpy = tf.transformations.euler_from_quaternion(quaternion) #Roll, Pitch, Yaw: Radians
		yaw = rpy[2] + self.pan #CHECK: Assuming positive left for camera PAN
		rospy.loginfo(yaw)	
		#cameraPos = (robotPos[0] + (self.cameraFwd*np.cos(yaw)), robotPos[1] + (self.cameraFwd*np.sin(yaw)))
	
		#blockPos = np.add(robotPos,relPositions)

		msg = BlockPositions()
		msg.blocks = []

		#rospy.loginfo(transformCoordinates((0,0),(1,1),0))


		for pos in relPositions:
		
			phi = np.arctan(pos[1]/(pos[0]+self.cameraFwd))
			L = np.linalg.norm(((pos[0]+self.cameraFwd),pos[1]))
			#assert L == np.sqrt(np.square(pos[0]) + np.square(pos[1])), "Jacob, you was wrong about norms"
			rospy.loginfo(phi)
			#xOffset = L*np.cos(phi + yaw)
			#yOffset = L*np.sin(phi + yaw)
			
			xOffset = L*np.cos(phi+yaw)
			yOffset = L*np.sin(phi+yaw)
			rospy.loginfo((phi,L,xOffset,yOffset))
			#absolutePos = (cameraPos[0] + xOffset, cameraPos[1] + yOffset)
			absolutePos = (robotPos[0] + xOffset, robotPos[1] + yOffset)


			#absolutePos = (cameraPos[0] + pos[0], cameraPos[1] + pos[1])
			
			
			
			#absolutePos = transformCoordinates(pos, cameraPos, yaw + self.pan)
			#absolutePos = transformCoordinates(cameraPos, pos, yaw + self.pan)
			if ((np.abs(absolutePos[0]) < self.course_length) and (np.abs(absolutePos[1]) < self.course_width)):
				
				absPos = Pose2D()
				absPos.x = absolutePos[0]
				absPos.y = absolutePos[1]
				absPos.theta = 0
				msg.blocks.append(absPos)
			

		self.pub.publish(msg)
		#rospy.sleep(0.2) #Is this necessary? Will it reduce stress on odroid?
		
		
if __name__=='__main__':
	try:
		with blockHandler() as blockHandleObj:
			rospy.spin()
	except rospy.ROSInterruptException:
		#rospy.loginfo('failed')
		pass
