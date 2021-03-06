#!/usr/bin/env python

# Import necessary package 
import rospy
from std_msgs.msg import String
import sys, select, termios, tty

# Introduction
msg = """Reading input from the keyboard and Publishing bridge command to Subsciber!
---------------------------------------------------
E/e : Engage the robot 
F/f : Finish using the robot
CTRL-C to quit
"""

# Class for testing the robot movement when someone touched the screen
class bridge_publisher():

	# Initial state
	def __init__(self):
		# Print introduction
		print(msg)
		# Define node name
		rospy.init_node('sim_bridge_publisher_node', anonymous=True)
		self.bridge_command = rospy.Publisher("user_pressed", String, queue_size = 10)  
		self.options = {'E' : None,
						'e' : None,
						'F' : None,
						'f' : None}
		# Loop frequency
		rate = rospy.Rate(10)
		# Main loop
		self.main_loop(rate)

	# Get keyboard key
	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key

	# Select operation 
	def input_keyboard(self,key):
		if key in self.options.keys():
			if key == 'E' or key == 'e':
				print("User pushed 'engage' button")
				self.bridge_command.publish("engage")
			elif key == 'F' or key == 'f':
				print("User pushed 'finish' button")
				self.bridge_command.publish("finish")
		if key not in self.options.keys() and key != '\x03':
			print("Please push only 'Engage','Finish' button")
			self.bridge_command.publish(key)
		else:
			if (key == '\x03'):
				sys.exit(1)

	# Main loop
	def main_loop(self, rate):
		while not rospy.is_shutdown():
			key = self.getKey()
			self.input_keyboard(key)
			rate.sleep()


if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	try:
		bridge_publisher()
	except Exception as e:
		print(e)
	finally:
		bridge_command = rospy.Publisher("user_pressed", String, queue_size = 10)    
		bridge_command.publish("X")
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
