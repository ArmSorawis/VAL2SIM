#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

from std_msgs.msg import String
from os.path import expanduser

home = expanduser("~")


class play_silent:
	def __init__(self):
		rospy.init_node('val2sim_silentplay_node')
		self.silenthandle = SoundClient()
		self.silent_volume = 1.0
		self.silent_to_play = '{}/val2sim_ws/src/val2sim_sound/sound/obstacle.wav'.format(home)
		self.silent_cycle_time = 7.0
		self.listener()

	def listener(self):
		rospy.Subscriber('silent', String, self.play)
		rospy.spin()
	
	def play(self, data):
		rospy.sleep(1)
		self.silenthandle.playWave(self.silent_to_play, self.silent_volume)
		rospy.sleep(self.silent_cycle_time)

if __name__ == '__main__':
	process = play_silent()
