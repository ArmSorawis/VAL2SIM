#!/usr/bin/env python

# Import necessary package 
import rospy
from geometry_msgs.msg import Twist


class teleop_publisher():

	def __init__(self):
		self.pub_robot_velocity = rospy.Publisher('pause_button_vel', Twist, queue_size = 10)  
		rate = rospy.Rate(10)
		self.twist_robot = Twist()
		self.twist_robot.linear.x = 0
		self.twist_robot.linear.y = 0
		self.twist_robot.linear.z = 0
		self.twist_robot.angular.x = 0
		self.twist_robot.angular.y = 0
		self.twist_robot.angular.z = 0
		self.publish_velocity(rate)
	
	# Publish velocity
	def publish_velocity(self, rate):
		while(not rospy.is_shutdown()):
			self.pub_robot_velocity.publish(self.twist_robot)
			rate.sleep()


if __name__ == '__main__':
	rospy.init_node('val2sim_fsm_ces_pause_node', anonymous=False)
	try:
		teleop_publisher()
	except Exception as e:
		print(e)