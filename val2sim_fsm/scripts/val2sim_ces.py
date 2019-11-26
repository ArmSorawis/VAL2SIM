#!/usr/bin/env python

import rospy
import smach
import smach_ros
import roslaunch
from sim_read_actionFile import sim_read_action
from sim_switch_obstacleDetection_type1 import sim_open_obstacleDetection_node, sim_close_obstacleDetection_node


class systemAvailability(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['system_checked'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state system availability')
		rospy.loginfo('Robot ready to use')
		sim_read_action("initial_station", None, "play_sound")

		sound_node = roslaunch.core.Node(package='val2sim_sound', 
										 node_type='val2sim_soundplay.py', 
										 name='val2sim_soundplay_node')
		sound_node.args = "_sound_to_play:=%s _sound_cycle_time:=%d" %(sim_read_action.sound, 
																	   sim_read_action.sound_cycleTime)
		sound_launch = roslaunch.scriptapi.ROSLaunch()
		sound_launch.start()
		sound_process = sound_launch.launch(sound_node)
		while sound_process.is_alive():
			if sound_process.is_alive() == False:
				break
		sound_process.stop()

		# rospy.loginfo('Robot rotate around for localize the real position')
		# localize_node = roslaunch.core.Node(package='val2sim_sensor', 
		# 									node_type='sim_rotateBy_odom.py', 
		# 									name='sim_rotateBy_odom_node',
		# 									output="screen")
		# localize_node.args = "_rotate_target:=%d" %(360)
		# localize_launch = roslaunch.scriptapi.ROSLaunch()
		# localize_launch.start()
		# localize_process = localize_launch.launch(localize_node)
		# while localize_process.is_alive():
		# 	if localize_process.is_alive() == False:
		# 		break
		# localize_process.stop()

		userdata.goalList_output = userdata.goalList_input

		return 'system_checked'


class move_forward(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['move_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state move forward')
		rospy.loginfo('Robot have been moving to {}'.format(userdata.goalList_input[0]))
		
		sim_open_obstacleDetection_node()

		move_node = roslaunch.core.Node(package='val2sim_sensor', 
											node_type='sim_translateBy_odom.py', 
											name='sim_translateBy_odom_node',
											output="screen")
		move_node.args = "_translate_target:=%d" %(200)
		move_launch = roslaunch.scriptapi.ROSLaunch()
		move_launch.start()
		move_process = move_launch.launch(move_node)
		while move_process.is_alive():
			if move_process.is_alive() == False:
				break
		move_process.stop()

		sim_close_obstacleDetection_node()

		del userdata.goalList_input[0]
		userdata.goalList_output = userdata.goalList_input

		return 'move_success'


class reach2goal(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['reach_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])
							
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state reach to goal')
		rospy.loginfo('Robot reach to {}'.format(userdata.goalList_input[0]))

		sim_read_action("station1", None, "play_sound")

		sound_node = roslaunch.core.Node(package='val2sim_sound', 
										 node_type='val2sim_soundplay.py', 
										 name='val2sim_soundplay_node')
		sound_node.args = "_sound_to_play:=%s _sound_cycle_time:=%d" %(sim_read_action.sound, 
																	   sim_read_action.sound_cycleTime)
		sound_launch = roslaunch.scriptapi.ROSLaunch()
		sound_launch.start()
		sound_process = sound_launch.launch(sound_node)
		while sound_process.is_alive():
			if sound_process.is_alive() == False:
				break
		sound_process.stop()

		userdata.goalList_output = userdata.goalList_input
		
		return 'reach_success'


class turn_around(smach.State):
	def __init__(self):
		smach.State.__init__(self,
							outcomes=['turn_success','all_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])
		self.rotate_angle = 0

	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state turn around')
		if userdata.goalList_input[0] == "station1":
			userdata.goalList_input[0] = "base_station"
			self.rotate_angle = 180
		elif userdata.goalList_input[0] == "base_station":
			userdata.goalList_input[0] = "charging_station"
			self.rotate_angle = -180

		rospy.loginfo("Robot turn around to {}".format(userdata.goalList_input[0]))
		
		turn_node = roslaunch.core.Node(package='val2sim_sensor', 
										node_type='sim_rotateBy_odom.py', 
										name='sim_rotateBy_odom_node',
										output="screen")
		turn_node.args = "_rotate_target:=%d" %(self.rotate_angle)
		turn_launch = roslaunch.scriptapi.ROSLaunch()
		turn_launch.start()
		turn_process = turn_launch.launch(turn_node)
		while turn_process.is_alive():
			if turn_process.is_alive() == False:
				break
		turn_process.stop()

		del userdata.goalList_input[0]
		userdata.goalList_output = userdata.goalList_input

		if len(userdata.goalList_input) > 0:
			return 'turn_success'
		elif len(userdata.goalList_input) == 0:
			return 'all_success'


class wait4nextround(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['finished_process'])


	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state wait for user')
		return 'finished_process'
		

def main():
	sm_nav = smach.StateMachine(outcomes=['shutdown'])
	sm_nav.userdata.goal_list = ["station1", "base_station"]
	with sm_nav:
		smach.StateMachine.add('SYSTEM_AVAILABILITY', systemAvailability(),
								transitions={'system_checked':'MOVE_FORWARD'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('MOVE_FORWARD', move_forward(),
								transitions={'move_success':'SM_ACTION'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('WAIT4NEXTROUND', wait4nextround(),
								transitions={'finished_process':'shutdown'})

		sm_act = smach.StateMachine(outcomes=['turn_finished', 'all_finished'])
		sm_act.userdata.goal_list = ["station1", "base_station"]
		with sm_act:
			smach.StateMachine.add('REACH2GOAL', reach2goal(), 
									transitions={'reach_success':'TURN_AROUND'},
									remapping={ 'goalList_input':'goal_list',
												'goalList_output':'goal_list'})

			smach.StateMachine.add('TURN_AROUND', turn_around(),
									transitions={'turn_success':'turn_finished', 'all_success':'all_finished'},
									remapping={ 'goalList_input':'goal_list',
												'goalList_output':'goal_list'})

		smach.StateMachine.add('SM_ACTION', sm_act,
							   transitions={'turn_finished':'MOVE_FORWARD','all_finished':'WAIT4NEXTROUND'})


	sis = smach_ros.IntrospectionServer('server_name', sm_nav, 'SM_NAV/SM_ACT')
	sis.start()
	outcome = sm_nav.execute()
	# rospy.spin()
	sis.stop()


if __name__ == '__main__':
	rospy.init_node('val2sim_ces_node', anonymous=False)
	main()