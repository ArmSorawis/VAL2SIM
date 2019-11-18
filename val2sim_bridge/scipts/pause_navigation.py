#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class pause_navigation():
	def __init__(self):
		self.pausedvel_publisher = rospy.Publisher('paused_vel', Twist, queue_size=10)
		self.twist = Twist()
		self.twist.linear.x = 0
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0

	def publishVel(self, cond):
		if cond == True:
			self.pausedvel_publisher.publish(self.twist)
		elif cond == False:
			pass
		elif cond == None:
			pass

if __name__ == "__main__":
	rospy.init_node('pause_navigation_node', anonymous=True)
	pauseNav_node = pause_navigation()
	rate = rospy.Rate(20)
	while True:
		break_cond = rospy.get_param("break_cond", None)
		pauseNav_node.publishVel(break_cond)
		rate.sleep()