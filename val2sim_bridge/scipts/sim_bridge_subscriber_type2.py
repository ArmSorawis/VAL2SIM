#!/usr/bin/env python

# Import necessary package 
import rospy
import roslaunch
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import os
import dynamic_reconfigure.client
from os.path import expanduser

# Initialize home directory
home = expanduser("~")

# Class for operating the robot to giving desired sound and stop moving
class bridge_website():

	# Initial state
	def __init__(self):
		self.soundPath = "{}/val2sim_ws/src/val2sim_sound/sound/".format(home)
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

	# open the obstacle detection system (the robot stop moving when detected the obstacle)
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

	# close the obstacle detection system (after 3 sec the robot continue moving in the new path)
	def close_obstacleDetection_node(self):
		nodes = os.popen("rosnode list").read().splitlines()
		interest_node = '/sim_obstacleDetection_type2_node'
		if interest_node in nodes:
			os.system("rosnode kill {}".format(interest_node))
		elif interest_node not in nodes:
			pass

	# close the obstacle avoidance system (robot stop create a new path for avoiding the obstacle)
	def close_obstacleAvoidance(self):
		client_global = dynamic_reconfigure.client.Client("/move_base/global_costmap/obstacle_layer")
		params_global = { 'enabled' : 'False' }
		client_local = dynamic_reconfigure.client.Client("/move_base/local_costmap/obstacle_layer")
		params_local = { 'enabled' : 'False' }
		config_global = client_global.update_configuration(params_global)
		config_local = client_local.update_configuration(params_local)

	# function for publishing the sound command and decide to open or close obstacle detection system when receive command from local website
	def toggled(self, website_data, change_command):
		if website_data == "engage":
			if change_command == True:
				rospy.set_param("break_cond", True)
				self.sound_publisher.publish("engage_process")
				self.close_obstacleDetection_node()
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
				change_command = False

		else:
			if change_command == True:
				rospy.set_param("break_cond", False)
				self.sound_publisher.publish("wrong_process")
	
	
# Class for subscibe command from the local website
class touchcmd_subscriber(object):
    def __init__(self):
        self.user_command = None

    def callback(self, command):
		self.user_command = command.data
        
    def touchScreen_listener(self):
		rospy.Subscriber("user_pressed", String, self.callback)

if __name__ == "__main__":
	# Initial node name
	rospy.init_node('sim_bridge_subscriber_type2_node', anonymous=True)
	
	# Initial each class statement
	touchcmdSub_node = touchcmd_subscriber()
	touchcmdSub_node.touchScreen_listener()
	bridgeSub_node = bridge_website()
	
	# Variable for checking the old command and new command are they same or not
	first_cmd = True
	change_command = True
	rate = rospy.Rate(20)

	# Loop for sending the command to operation class
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
