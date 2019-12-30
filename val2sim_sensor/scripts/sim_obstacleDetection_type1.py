#!/usr/bin/env python

# Wait until obstacle give a way

# Important necessary package
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
import os
import dynamic_reconfigure.client

# Class for detect the obstacle 
class obstacle_detection():

	# Initial state
	def __init__(self):
		# Define node name
		rospy.init_node('sim_obstacleDetection_type1_node', disable_signals=True)
		# Publisher configuration
		self.cmdvel_publisher = rospy.Publisher('break_vel', Twist, queue_size=10)
		self.emer_publisher = rospy.Publisher('silent', String, queue_size=10)
		# Time variation variable
		self.first_time = True
		self.start_time = 0.0
		self.interval_time = 0.0
		self.waiting_time = 10.0
		self.emer_round = 0
		# Call function
		self.listener()

	# Call call back function when subscribe to 'scan' topic in LaserScan type 
	def listener(self):
		rospy.Subscriber('scan', LaserScan, self.callback, queue_size=1000)
		rospy.spin()

	# Computing the data from lidar and select the robot state
	def callback(self, msg):
		obstacle_check = False
		for i in range(590,690):
			if msg.ranges[i] != float("inf") and msg.ranges[i] > 0.75:
				obstacle_check = False
			elif msg.ranges[i] != float("inf") and msg.ranges[i] < 0.75:
				obstacle_check = True
				break
		self.check_mode(obstacle_check)

	# Check robot state
	def check_mode(self, obstacle):
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		
		if obstacle == True and self.interval_time < self.waiting_time:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			if self.first_time == True:
				self.start_time = time.time()
				if self.emer_round < 2:
					self.emer_publisher.publish("True")
					self.emer_round = self.emer_round + 1
				self.first_time = False
			self.interval_time = time.time() - self.start_time

		elif obstacle == True and self.emer_round >=2:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)

		elif obstacle == True and self.interval_time >= self.waiting_time:
			print("Stop")
			self.twist_robot.linear.x = 0
			self.twist_robot.angular.z = 0
			self.cmdvel_publisher.publish(self.twist_robot)
			
			self.first_time = True
			self.interval_time = 0

		elif obstacle == False:
			print("Continue moving")
			self.first_time = True
			self.interval_time = 0
			self.emer_round = 0
			pass

	#Convert angle from radians to degree format
	def rad2deg(self, radians):
		pi = math.pi
		degrees = (180 * radians) / pi
		return degrees


if __name__ == '__main__':
	process = obstacle_detection()
