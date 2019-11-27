#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from os.path import expanduser

home = expanduser("~")


class play_sound:
	def __init__(self):
		rospy.init_node('val2sim_soundplay_node',disable_signals=True)
		self.soundhandle = SoundClient()
		self.sound_volume = rospy.get_param("~sound_volume", 1.0)
		self.sound_to_play = rospy.get_param("~sound_to_play", "{}/val2sim_ws/src/val2sim_sound/sound/base_station.wav".format(home))
		self.sound_cycle_time = rospy.get_param("~sound_cycle_time", 5.0)
		self.play()
	
	def play(self):
		if self.sound_to_play == None or self.sound_cycle_time == 0: 
			rospy.signal_shutdown("Quit")
		else:
			rospy.sleep(1)
			self.soundhandle.playWave(self.sound_to_play, self.sound_volume)
			rospy.sleep(self.sound_cycle_time)

if __name__ == '__main__':
	process = play_sound()
