#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class joyvelFiltered():

	def __init__(self):
		self.joyvel_filtered_publisher = rospy.Publisher("joy_vel_filtered", Twist, queue_size = 10)
		self.target_linear_velocity = 0
		self.target_angular_velocity = 0
		self.linear_acc = 0.1
		self.angular_acc = 0.1

	def filter(self, joy_vel, value_change):
		if value_change[0] == True:
			self.target_linear_velocity = round(joy_vel.linear.x, 2)
			value_change[0] = False
		if value_change[1] == True:
			self.target_angular_velocity = round(joy_vel.angular.z, 2)
			value_change[1] = False

		print(self.target_linear_velocity, self.target_angular_velocity)
		
		self.joyvel_filtered_publisher.publish(joy_vel)
		

class joyvel_subscriber(object):
	def __init__(self):
		self.joy_velocity = Twist()

	def callback(self, data):
		self.joy_velocity = data
		
	def joyvel_listener(self):
		rospy.Subscriber('joy_vel', Twist, self.callback)

if __name__=="__main__":
	rospy.init_node('sim_joyvel_filtered_node')
	joyvelSub_node = joyvel_subscriber()
	joyvelSub_node.joyvel_listener()

	joyvelFiltered_node = joyvelFiltered()
	
	first_linear_vel = True
	first_angular_vel = True
	change_vel = [True, True] #linear,angular

	rate = rospy.Rate(50)
	
	while(not rospy.is_shutdown()):
		joy_vel = joyvelSub_node.joy_velocity

		if first_linear_vel == True:
			old_linear_vel = joy_vel.linear.x
			first_linear_vel = False
		if old_linear_vel != joy_vel.linear.x:
			change_vel[0] = True
			first_linear_vel = True
		
		if first_angular_vel == True:
			old_angular_vel = joy_vel.angular.z
			first_angular_vel = False
		if old_angular_vel != joy_vel.angular.z:
			change_vel[1] = True
			first_angular_vel = True
		
		joyvelFiltered_node.filter(joy_vel, change_vel)
		change_vel[0] = False
		change_vel[1] = False

		rate.sleep()