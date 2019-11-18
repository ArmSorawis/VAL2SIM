#!/usr/bin/env python

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

from std_msgs.msg import String

class play_touch:
	def __init__(self):
		rospy.init_node('val2sim_touchplay_node')
		self.touchhandle = SoundClient()
		self.touch_volume = 1.0
		self.listener()

	def listener(self):
		rospy.Subscriber('touch_sound', String, self.play)
		rospy.spin()
	
	def play(self, user_input):
		print(user_input.data)
		if user_input.data == "engage_process":
			self.touchhandle.playWave("/home/csorawit/val2sim_ws/src/val2sim_sound/sound/engage_pressed.wav", self.touch_volume)
			rospy.sleep(5.0)
		elif user_input.data == "finish_process":
			self.touchhandle.playWave("/home/csorawit/val2sim_ws/src/val2sim_sound/sound/finish_pressed.wav", self.touch_volume)
			rospy.sleep(2.0)
		elif user_input.data == "wrong_process":
			self.touchhandle.playWave("/home/csorawit/val2sim_ws/src/val2sim_sound/sound/wrong_clicked.wav", self.touch_volume)
			rospy.sleep(4.0)

if __name__ == '__main__':
	process = play_touch()
