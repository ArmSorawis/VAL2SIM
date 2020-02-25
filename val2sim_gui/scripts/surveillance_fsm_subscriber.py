#!/usr/bin/env python

# Important Library

from std_msgs.msg import String
import rospy
import roslaunch
import os

def surveillance_start():		
	fsm_node = roslaunch.core.Node(	package='val2sim_fsm', 
									node_type='val2sim_fsm_surveillance.py', 
									name='val2sim_fsm_surveillance_node',
									output='screen')
	fsm_launch = roslaunch.scriptapi.ROSLaunch()
	fsm_launch.start()
	fsm_process = fsm_launch.launch(fsm_node)
	while fsm_process.is_alive():
		if fsm_process.is_alive() == False:
			break

class gui_fsm_subscriber(object):

	def __init__(self):
		self.gui_command = None

	def callback(self,data):
		self.gui_command = data.data

	def listener(self):
		rospy.Subscriber('gui_cmd', String, self.callback)


if __name__=="__main__":
	rospy.init_node('surveillance_fsm_subscriber_node')
	guiSub_node = gui_fsm_subscriber()
	guiSub_node.listener()
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = guiSub_node.gui_command
			if command == "start":
				surveillance_start()
			r.sleep()
	except Exception as e:
		print(e)