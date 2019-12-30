#!/usr/bin/env python

# Import necessary package 
import rospy
import roslaunch
from sensor_msgs.msg import Joy
from std_msgs.msg import String

# Class for operating the robot to 
class operate_command():

	# Initial state
	def __init__(self):
		# Define initial target in term of rotation angle
		self.rotate_target = 0

	# Selection state depend on input from joy buttons
	def toggled(self, joy_data):
		if joy_data.count(1) == 1:
			if joy_data.index(1) == 0:
				self.rotate_target = 90
				self.rotateRobot()
			elif joy_data.index(1) == 1:
				self.rotate_target = 180
				self.rotateRobot()
			elif joy_data.index(1) == 2:
				self.rotate_target = -90
				self.rotateRobot()
			elif joy_data.index(1) == 3:
				self.rotate_target = 360
				self.rotateRobot()
			elif joy_data.index(1) == 9:
				self.robotFSM()
		elif joy_data.count(1) > 1:
			rospy.loginfo("Please click atmost 1 button.")

	# Function for rotate robot to desired angle
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
	
	# Function for making robot move in finite state machine process
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

# Class for receiving command which came from joy buttons
class joycmd_subscriber(object):

	# Initial state
	def __init__(self):
		self.joy_command = None

	# Storage joy data which received from joy stick
	def callback(self, data):
		self.joy_command = data.buttons
		
	# Call call back function when subscribe to 'joy' topic in Joy type
	def joycmd_listener(self):
		rospy.Subscriber('joy', Joy, self.callback)
		

if __name__=="__main__":
	# Define node name
	rospy.init_node('sim_joycmd_subscriber_node')

	# Define necessary variable which calling each class
	joycmdSub_node = joycmd_subscriber()
	joycmdSub_node.joycmd_listener()
	operateCommand_node = operate_command()
	
	# Loop frequency
	rate = rospy.Rate(20)
	while(not rospy.is_shutdown()):
		# Storage joy date
		joy_cmd = joycmdSub_node.joy_command
		if joy_cmd != None:
			# Select state
			operateCommand_node.toggled(joy_cmd)
		rate.sleep()
		