#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import sys
from tf.transformations import euler_from_quaternion
 
class send_cmdvel():

	def __init__(self):
		self.pub = rospy.Publisher('/nav_vel', Twist, queue_size=1)
		self.command =Twist()
		self.target = rospy.get_param("~translate_target", 200) #cm
		self.diff = 0
		self.filter_pose = 0
		self.kp = 0.001
		self.setPose = True

		self.twist_robot =Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		self.twist_robot.angular.z = 0

	def selectState(self):
		if self.target >= 0:
			self.state = "move_forward"
		elif self.target < 0:
			self.state = "move_backward"

	def rotateRobot(self, state, robot_pose, robot_yaw):

		if state == "move_forward":
			if self.setPose == True:
				self.first_pose = robot_pose
				self.target = self.target * abs(math.cos(robot_yaw))
				self.setPose = False

			self.filter_pose = abs(robot_pose - self.first_pose)

			self.diff = abs(self.target - self.filter_pose)
			velocity = self.kp * self.diff
			if velocity >= 0.25:
				velocity = 0.25
			if velocity <= 0.15:
				velocity = 0.15
			self.twist_robot.linear.x = velocity
			self.pub.publish(self.twist_robot)
			if abs(self.diff) < 1:
				state = "stop"

			print("\n")
			print("Robot state={}".format(state))
			print("Robot velocity={}".format(velocity))
			print("target={} current={}".format(self.target, self.filter_pose))
		
		if state == "stop":
			self.twist_robot.linear.x = 0
			self.pub.publish(self.twist_robot)
			rospy.signal_shutdown('Quit')
		  
		   
class odom_subscriber(object):

	def __init__(self):
		self.pose_centimeter = 0.0
		self.angle_rad = 0.0

	def get_position(self, msg):
		position_q = msg.pose.pose.position
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.angle_rad = yaw #rad
		self.pose_centimeter = position_q.x * 100 #cm

	def listener(self):
		rospy.Subscriber ('/val2/base_controller/odom', Odometry, self.get_position) #For Simulation


if __name__=="__main__":
	rospy.init_node('sim_translateBy_odom_node', disable_signals=True)

	odomSub_node = odom_subscriber()
	odomSub_node.listener()

	sendCmdvel_node = send_cmdvel()
	sendCmdvel_node.selectState()
	state = sendCmdvel_node.state
	
	first_yaw = True
	yaw = 0

	r = rospy.Rate(10)
	rospy.sleep(1)
	try:
		while(not rospy.is_shutdown()):
			if first_yaw == True:
				yaw = odomSub_node.angle_rad
				first_yaw = False
			sendCmdvel_node.rotateRobot(state, odomSub_node.pose_centimeter, yaw)
			r.sleep()
	except Exception as e:
		print(e)