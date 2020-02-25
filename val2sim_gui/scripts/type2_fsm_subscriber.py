#!/usr/bin/env python

# Important Library

from std_msgs.msg import String
import rospy
import roslaunch
import os

def type2_start(goal_list):
	# Robot moving in term of obstacle detection state machine process
	fsm_node = roslaunch.core.Node(	package='val2sim_fsm', 
									node_type='val2sim_fsm_type2.py', 
									name='val2sim_fsm_type2_node',
									output='screen')
	fsm_node.args = "_goal1:={} _goal2:={} _goal3:={} _goal4:={} _goal5:={}".format(goal_list[0], 
																					goal_list[1],
																					goal_list[2],
																					goal_list[3],
																					goal_list[4])
	fsm_launch = roslaunch.scriptapi.ROSLaunch()
	fsm_launch.start()
	fsm_process = fsm_launch.launch(fsm_node)
	while fsm_process.is_alive():
			if fsm_process.is_alive() == False:
					break

class gui_fsm_subscriber(object):

	def __init__(self):
		self.gui_command = None
		self.gui_goal1 = None
		self.gui_goal2 = None
		self.gui_goal3 = None
		self.gui_goal4 = None
		self.gui_goal5 = None

	def callback_command(self,data):
		self.gui_command = data.data

	def callback_goal1(self,data):
		self.gui_goal1 = data.data

	def callback_goal2(self,data):
		self.gui_goal2 = data.data

	def callback_goal3(self,data):
		self.gui_goal3 = data.data

	def callback_goal4(self,data):
		self.gui_goal4 = data.data

	def callback_goal5(self,data):
		self.gui_goal5 = data.data

	def listener(self):
		rospy.Subscriber('gui_cmd', String, self.callback_command)
		rospy.Subscriber('gui_goal1', String, self.callback_goal1)
		rospy.Subscriber('gui_goal2', String, self.callback_goal2)
		rospy.Subscriber('gui_goal3', String, self.callback_goal3)
		rospy.Subscriber('gui_goal4', String, self.callback_goal4)
		rospy.Subscriber('gui_goal5', String, self.callback_goal5)


if __name__=="__main__":
	rospy.init_node('type2_fsm_subscriber_node')
	guiSub_node = gui_fsm_subscriber()
	guiSub_node.listener()
	r = rospy.Rate(10)
	try:
		while(not rospy.is_shutdown()):
			command = guiSub_node.gui_command
			goal1 = guiSub_node.gui_goal1
			goal2 = guiSub_node.gui_goal2
			goal3 = guiSub_node.gui_goal3
			goal4 = guiSub_node.gui_goal4
			goal5 = guiSub_node.gui_goal5
			if command == "start":
				goals = [goal1, goal2, goal3, goal4, goal5]
				if goals.count(None) < 5:
					type2_start(goals)
			r.sleep()
	except Exception as e:
		print(e)
