#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import dynamic_reconfigure.client


class bridge_website():

	def __init__(self):
		self.soundPath = "/home/csorawit/val2sim_ws/src/val2sim_sound/sound/"
		self.sound_publisher = rospy.Publisher('touch_sound', String, queue_size=10)
		self.twist = Twist()
		self.twist.linear.x = 0
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0
		self.interest_param_global = '/move_base/global_costmap/obstacle_layer/enabled'
		self.interest_param_local = '/move_base/local_costmap/obstacle_layer/enabled'

	def open_obstacleDetection_node(self):
		nodes = os.popen("rosnode list").read().splitlines()
		interest_node = '/movebase_client_py'
		if interest_node in nodes:
			obstacle_node = roslaunch.core.Node(package='val2sim_sensor', 
												node_type='sim_obstacleDetection_type2.py', 
												name='sim_obstacleDetection_type2_node')
			obstacle_launch = roslaunch.scriptapi.ROSLaunch()
			obstacle_launch.start()
			obstacle_process = obstacle_launch.launch(obstacle_node)
		elif interest_node not in nodes:
			pass

	def close_obstacleDetection_node(self):
		nodes = os.popen("rosnode list").read().splitlines()
		interest_node = '/sim_obstacleDetection_type2_node'
		if interest_node in nodes:
			os.system("rosnode kill {}".format(interest_node))
		elif interest_node not in nodes:
			pass

	def close_obstacleAvoidance(self):
		client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
		params_global = { 'enabled' : 'False' }
		client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
		params_local = { 'enabled' : 'False' }
		config_global = client_global.update_configuration(params_global)
		config_local = client_local.update_configuration(params_local)

	def toggled(self, website_data, change_command):
		# rospy.loginfo("I heard user toggle {}".format(website_data))
		if website_data == "engage":
			if change_command == True:
				rospy.set_param("break_cond", True)
				self.sound_publisher.publish("engage_process")
				self.close_obstacleDetection_node()
				# rospy.loginfo("I'd be happy to do something for you")
				enable_global = rospy.get_param(self.interest_param_global, None)
				enable_local = rospy.get_param(self.interest_param_local, None)
				if enable_global == True and enable_local == True:
					self.close_obstacleAvoidance()
				change_command = False

		elif website_data == "finish":
			if change_command == True:
				rospy.set_param("break_cond", True)
				self.sound_publisher.publish("finish_process")	
				self.open_obstacleDetection_node()
				rospy.sleep(3)
				rospy.set_param("break_cond", False)
				# rospy.loginfo("Thanks, I'm at your service")
				change_command = False

		else:
			if change_command == True:
				rospy.set_param("break_cond", False)
				# rospy.loginfo("Please push only the button on the screen")
				self.sound_publisher.publish("wrong_process")
	
	
class touchcmd_subscriber(object):
    def __init__(self):
        self.user_command = None

    def callback(self, command):
		self.user_command = command.data
        
    def touchScreen_listener(self):
		rospy.Subscriber("user_pressed", String, self.callback)

if __name__ == "__main__":
	rospy.init_node('sim_bridge_subscriber_type2_node', anonymous=True)
	touchcmdSub_node = touchcmd_subscriber()
	touchcmdSub_node.touchScreen_listener()

	bridgeSub_node = bridge_website()
	first_cmd = True
	change_command = True
	rate = rospy.Rate(20)

	while(not rospy.is_shutdown()):
		user_cmd = touchcmdSub_node.user_command
		if first_cmd == True:
			old_cmd = user_cmd
			first_cmd = False
		if old_cmd != user_cmd:
			change_command = True
			first_cmd = True
		if user_cmd != None:
			bridgeSub_node.toggled(user_cmd, change_command)
			change_command = False
		rate.sleep()
