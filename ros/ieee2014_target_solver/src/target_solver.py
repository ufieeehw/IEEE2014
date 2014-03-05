from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D, PoseStamped
import rospy
import numpy as np
import tf
import string
from collections import deque #This comes in handy later(
def getNodeList():
	nodeList = os.popen('rosnode list').read()
	nodes = string.split(string.replace(nodeList,'\n', ' '))
	return nodes


class targetSolver:
	def __init__(self):
		rospy.init_node('target_solver')
		self.course_length = (97 - 3/4 * 2) * 0.0254 #Courtesy of Lord Voight
		self.course_width = (49 - 3/4 * 2) * 0.0254
		self.target_pos = (-course_length/2, 0)
		self.target_height = 26.5 * 0.0254 #meters
		
		self.gun_height = 8.937 * 0.0254#This is the height of the center of the gun at zero
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
				os.system('rosrun ieee2014_simulator ieee2014_simulator &')

			rospy.loginfo("\nNOTE: Target_Solver: Using simulation pose data")
			
			self.poseSub = rospy.Subscriber('sim_pose', PoseStamped, self.newData)
		elif(sim =="N"):
			
			self.poseSub = rospy.Subscriber('pose', PoseStamped, self.newData)
			
			
	def newData(data):
	
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
		rpy = tf.transformation.euler_from_quaternion(quaternion)
		yaw = rpy[2]
		
		robotPos = (data.pose.position.x,data.pose.position.y)
		np.subtract(co
		distance = np.linalg.norm(np.subtract(robotPos, self.target_pos))
		
		
		pan_to_target = np.arccos(robotPos[1]/distance) + np.pi/2 - yaw
		tilt_to_target = np.arctan((self.target_height - self.gun_height)/distance)
		self.pan_pub(pan_to_target)
		self.tilt_pub(tilt_to_target)
			
			
			
			
