#!/usr/bin/env python

# Import necessary package 
import rospy
from geometry_msgs.msg import Twist

# Class for filtering joy stick velocity
class joyvelFiltered():

	# Initial state
	def __init__(self):
		# Define publisher
		self.joyvel_filtered_publisher = rospy.Publisher("joy_vel_filtered", Twist, queue_size = 10)
		# Define latest velocity which received from joy stick
		self.target_linear_velocity = 0
		self.target_angular_velocity = 0
		# Defind linear and angular acceleration 
		self.linear_acc = 0.1
		self.angular_acc = 0.1

	# Function for ramp up and ramp down the velocity
	def filter(self, joy_vel):
		# if value_change[0] == True:
		# 	self.target_linear_velocity = round(joy_vel.linear.x, 2)
		# 	value_change[0] = False
		# if value_change[1] == True:
		# 	self.target_angular_velocity = round(joy_vel.angular.z, 2)
		# 	value_change[1] = False		
		self.joyvel_filtered_publisher.publish(joy_vel)
		
# Class for receiving velocity which came from joy stick
class joyvel_subscriber(object):

	# Initial state
	def __init__(self):
		self.joy_velocity = Twist()

	# Storage velocity from joy stick
	def callback(self, data):
		self.joy_velocity = data
		
	# Call call back function when subscribe to 'joy_vel' topic in Twist type
	def joyvel_listener(self):
		rospy.Subscriber('joy_vel', Twist, self.callback)


if __name__=="__main__":
	# Define node name
	rospy.init_node('sim_joyvel_filtered_node')

	# Define necessary variable which calling each class
	joyvelSub_node = joyvel_subscriber()
	joyvelSub_node.joyvel_listener()
	joyvelFiltered_node = joyvelFiltered()
	
	# first_linear_vel = True
	# first_angular_vel = True
	# change_vel = [True, True] #linear,angular

	# Loop frequency
	rate = rospy.Rate(50)
	while(not rospy.is_shutdown()):
		joy_vel = joyvelSub_node.joy_velocity

		# if first_linear_vel == True:
		# 	old_linear_vel = joy_vel.linear.x
		# 	first_linear_vel = False
		# if old_linear_vel != joy_vel.linear.x:
		# 	change_vel[0] = True
		# 	first_linear_vel = True
		
		# if first_angular_vel == True:
		# 	old_angular_vel = joy_vel.angular.z
		# 	first_angular_vel = False
		# if old_angular_vel != joy_vel.angular.z:
		# 	change_vel[1] = True
		# 	first_angular_vel = True
		
		joyvelFiltered_node.filter(joy_vel)
		# change_vel[0] = False
		# change_vel[1] = False

		rate.sleep()
