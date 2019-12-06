#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class keyvelFiltered():

	def __init__(self):
		self.keyvel_filtered_publisher = rospy.Publisher("key_vel_filtered", Twist, queue_size = 10)
		self.target_linear_velocity = 0
		self.target_angular_velocity = 0
		self.diff_linear = 0
		self.diff_angular = 0
		self.linear_acc = 0.1
		self.angular_acc = 0.1

	def filter(self, old_linear_vel, key_vel, value_change):
		if value_change[0] == True:
			self.target_linear_velocity = round(key_vel.linear.x, 2)
			value_change[0] = False
		if value_change[1] == True:
			self.target_angular_velocity = round(key_vel.angular.z, 2)
			value_change[1] = False
		# if self.target_linear_velocity > 0:
		# 	if round(old_linear_vel, 2) < self.target_linear_velocity:
		# 		key_vel.linear.x = key_vel.linear.x + 0.01
		# elif self.target_linear_velocity < 0:
		# 	if round(old_linear_vel, 2) > self.target_linear_velocity:
		# 		key_vel.linear.x = key_vel.linear.x - 0.01
		# elif self.target_linear_velocity == 0:
		# 	key_vel.linear.x = 0

		print(self.target_linear_velocity, self.target_angular_velocity)
		
		self.keyvel_filtered_publisher.publish(key_vel)
		

class keyvel_subscriber(object):
	def __init__(self):
		self.key_velocity = Twist()

	def callback(self, data):
		self.key_velocity = data
		
	def keyvel_listener(self):
		rospy.Subscriber('key_vel', Twist, self.callback)

if __name__=="__main__":
	rospy.init_node('sim_keyvel_filtered_node')
	keyvelSub_node = keyvel_subscriber()
	keyvelSub_node.keyvel_listener()

	keyvelFiltered_node = keyvelFiltered()
	
	old_linear_vel = 0
	old_angular_vel = 0
	first_linear_vel = True
	first_angular_vel = True
	change_vel = [True, True] #linear,angular

	rate = rospy.Rate(10)
	
	while(not rospy.is_shutdown()):
		key_vel = keyvelSub_node.key_velocity

		if first_linear_vel == True:
			old_linear_vel = key_vel.linear.x
			first_linear_vel = False
		if old_linear_vel != key_vel.linear.x:
			change_vel[0] = True
			first_linear_vel = True
		
		if first_angular_vel == True:
			old_angular_vel = key_vel.angular.z
			first_angular_vel = False
		if old_angular_vel != key_vel.angular.z:
			change_vel[1] = True
			first_angular_vel = True
		
		keyvelFiltered_node.filter(old_linear_vel, key_vel, change_vel)
		change_vel[0] = False
		change_vel[1] = False

		rate.sleep()