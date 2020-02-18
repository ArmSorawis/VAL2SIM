#!/usr/bin/env python

# Important Library

from std_msgs.msg import String
import rospy
import roslaunch
import os

def ces_pause():
	pause_node = roslaunch.core.Node(package='val2sim_fsm', 
									node_type='val2sim_fsm_ces_pause.py', 
									name='val2sim_fsm_ces_pause_node',
									output='screen')
	pause_launch = roslaunch.scriptapi.ROSLaunch()
	pause_launch.start()
	pause_process = pause_launch.launch(pause_node)
	while pause_process.is_alive():
		if pause_process.is_alive() == False:
			break

class gui_pause_subscriber(object):

	def __init__(self):
		self.gui_command = None

	def callback(self,data):
		self.gui_command = data.data

	def listener(self):
		rospy.Subscriber('gui_cmd', String, self.callback)


if __name__=="__main__":
	rospy.init_node('ces_pause_subscriber_node')
	guiSub_node = gui_pause_subscriber()
	guiSub_node.listener()
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = guiSub_node.gui_command
			if command == "pause":
				ces_pause()
			r.sleep()
	except Exception as e:
		print(e)