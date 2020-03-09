#!/usr/bin/env python

# Import necessary package 
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from os.path import expanduser

# Initialize home directory
home = expanduser("~")

# Class for alert the alarm sound when obstacle detected by lidar
class play_silent:

	# Initial state
	def __init__(self):
		rospy.init_node('val2sim_silentplay_surveillance_node')
		self.silenthandle = SoundClient()
		self.silent_to_play = '{}/val2sim_ws/src/val2sim_sound/sound/obstacle.wav'.format(home)
		self.silent_cycle_time = 7.0
		self.listener()

	# Call play function when subscribe to 'silent' topic in string type
	def listener(self):
		rospy.Subscriber('silent', String, self.play)
		rospy.spin()
	
	# Alert the alarm sound to the user for give way to robot
	def play(self, data):
		rospy.sleep(1)
		self.silent_volume = rospy.get_param("/val2sim_gui_surveillance_node/silent_volume", 1.0)
		self.silenthandle.playWave(self.silent_to_play, self.silent_volume)
		rospy.sleep(self.silent_cycle_time)

if __name__ == '__main__':
	# Call play_siren class
	process = play_silent()
