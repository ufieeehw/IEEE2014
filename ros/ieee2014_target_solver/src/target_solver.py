#!/usr/bin/env python
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D, PoseStamped
import rospy
import numpy as np
import tf
import string
import os
from collections import deque #This comes in handy later(
def getNodeList():
	nodeList = os.popen('rosnode list').read()
	nodes = string.split(string.replace(nodeList,'\n', ' '))
	return nodes

##Expose a service that enables and disables targeting
# - Should commit to a single value for position
# - Respond to current position of servos to limit jitter
def angle_between(v1, v2, origin):
	
	v1_u = unit_vector(v1-origin)
	v2_u = unit_vector(v2-origin)
	
	dot = np.dot(v1_u, v2_u)
	angle = None
	if(dot >=0 and dot <= 1):
		angle = np.arccos(dot)

	if angle == None:
		if (v1_u == v2_u).all():
			return 0.0
		else:
			return np.pi
	return angle

class targetSolver:
	def __init__(self):
		rospy.init_node('target_solver')
		self.course_length = (97 - 3/4 * 2) * 0.0254 #Courtesy of Lord Voight
		self.course_width = (49 - 3/4 * 2) * 0.0254
		self.target_pos = (-self.course_length/2, 0)
		self.target_height = 26.5 * 0.0254 #meters
		
		self.pan_prev = 0
		self.tilt_prev = 0
		
		#self.gun_height = 8.937 * 0.0254#This is the height of the center of the gun at zero
		self.gun_height = 5.5625 * 0.0254
		
		#self.height_to_gymbal = 3.25 #Gymbal to gun's firing axis
		#self.gymbal_height = 5.6875 * 0.0254
		#The gun height is actually a function of tilt
		
		#self.tilt_servo_offset = 0 #meters
		#self.gun_length = 0.08 #meters
		
		sim = rospy.get_param('/target_solver/sim', "N")
		
		#CHECK: Doesn't look like there is a control subscriber in place yet
		
		#Float64 as per convention established in joystick_command_dynamixel"
		self.pan_pub = rospy.Publisher('/pan_controller/command',Float64) 
		self.tilt_pub = rospy.Publisher('/tilt_controller/command',Float64)
		
		#For smoothing incoming lidar data:
		self.rolling_que = deque()
		
		
		if(sim == "Y"):
			if not '/ieee2014_simulator' in getNodeList():
				pass
				os.system('rosrun ieee2014_simulator ieee2014_simulator &')
			rospy.loginfo("\nNOTE: Target_Solver: Using simulation pose data")
			if not '/state_visualizer' in getNodeList():
				pass
				os.system('rosrun ieee2014_vision state_visualizer _sim:="Y" &')
			rospy.loginfo("\nNOTE: Target_Solver: Using state_visualizer")	
				
			
			
			self.poseSub = rospy.Subscriber('sim_pose', PoseStamped, self.newData)
		elif(sim =="N"):
			
			self.poseSub = rospy.Subscriber('pose', PoseStamped, self.newData)
			
			
	def newData(self,data):
	
		#if len(rolling_que) < 10:
		#	rolling_que.append(data)
		#	return
		#rolling_que.popleft()
		#Lidar Data looks like it is already smoothed in Forrest's code
		
		quaternion = (
			data.pose.orientation.x,
			data.pose.orientation.y,
			data.pose.orientation.z,
			data.pose.orientation.w)
		rpy = tf.transformations.euler_from_quaternion(quaternion)
		yaw = rpy[2]

		robotPos = (data.pose.position.x,data.pose.position.y)
		distance = np.linalg.norm(np.subtract(robotPos, self.target_pos))
		#rospy.loginfo(distance)
		
		tilt_msg = Float64()
		pan_msg = Float64()
		
		pan_to_target = np.pi - np.arccos((robotPos[0] + (self.course_length/2))/distance)
		#pan_to_target = np.pi - np.arctan(
		if robotPos[1] > 0:
			#pan_to_target = np.arccos((robotPos[0] + (self.course_length/2))/distance)
			pan_to_target = np.pi + np.arctan(robotPos[1]/(robotPos[0] + (self.course_length/2)))
			
			
		
		tilt_to_target = np.arctan((self.target_height - self.gun_height)/distance)
		
		if((np.abs(tilt_to_target - self.tilt_prev) > 0.1) and (np.abs(pan_to_target - self.pan_prev) > 0.1)):
			tilt_msg.data = tilt_to_target
			pan_msg.data = pan_to_target - yaw
		
			self.pan_pub.publish(pan_msg)
			self.tilt_pub.publish(tilt_msg)
			
		rospy.sleep(0.1)
try:
	targeter = targetSolver()

	rospy.spin()
except rospy.ROSInterruptException:
	rospy.loginfo('failed')
	pass

			
