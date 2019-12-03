#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import String


class operate_command():

	def __init__(self):
		self.rotate_target = 0 

	def toggled(self, joy_data):
		if joy_data == "rotate_90":
			self.rotate_target = 90
			self.rotateRobot()
		elif joy_data == "rotate_180":
			self.rotate_target = 180
			self.rotateRobot()
		elif joy_data == "rotate_-90":
			self.rotate_target = -90
			self.rotateRobot()
		elif joy_data == "rotate_360":
			self.rotate_target = 360
			self.rotateRobot()
		elif joy_data == "fsm":
			self.robotFSM()
	
	def rotateRobot(self):
		rotate_node = roslaunch.core.Node(  package='val2sim_sensor', 
											node_type='sim_rotateBy_odom.py', 
											name='sim_rotateBy_odom_node',
											output="screen")
		rotate_node.args = "_rotate_target:=%d" %(self.rotate_target)
		rotate_launch = roslaunch.scriptapi.ROSLaunch()
		rotate_launch.start()
		rotate_process = rotate_launch.launch(rotate_node)
		while rotate_process.is_alive():
			if rotate_process.is_alive() == False:
				break
		rotate_process.stop()
	
	def robotFSM(self):
		fsm_node = roslaunch.core.Node( package='val2sim_fsm', 
										node_type='val2sim_fsm_ces.py', 
										name='val2sim_fsm_ces_node',
										output='screen')
		fsm_launch = roslaunch.scriptapi.ROSLaunch()
		fsm_launch.start()
		fsm_process = fsm_launch.launch(fsm_node)
		while fsm_process.is_alive():
			if fsm_process.is_alive() == False:
				break


class joycmd_subscriber(object):
	def __init__(self):
		self.joy_command = None

	def callback(self, command):
		self.joy_command = command.data
		
	def joycmd_listener(self):
		rospy.Subscriber("joy_cmd", String, self.callback)


if __name__=="__main__":
	rospy.init_node('joy_cmd_subscriber_node')
	joycmdSub_node = joycmd_subscriber()
	joycmdSub_node.joycmd_listener()
	
	operateCommand_node = operate_command()
	
	rate = rospy.Rate(20)

	while(not rospy.is_shutdown()):
		joy_cmd = joycmdSub_node.joy_command
		if joy_cmd != None:
			operateCommand_node.toggled(joy_cmd)
		rate.sleep()