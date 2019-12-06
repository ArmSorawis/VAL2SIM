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

		rospy.loginfo('Robot rotate around for localize the real position')
		localize_node = roslaunch.core.Node(package='val2sim_sensor', 
											node_type='sim_rotateBy_odom.py', 
											name='sim_rotateBy_odom_node',
											output="screen")
		localize_node.args = "_rotate_target:=%d" %(360)
		localize_launch = roslaunch.scriptapi.ROSLaunch()
		localize_launch.start()
		localize_process = localize_launch.launch(localize_node)
		while localize_process.is_alive():
			if localize_process.is_alive() == False:
				break
		localize_process.stop()

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
		rospy.loginfo('Robot ready to use')

		userdata.goalList_output = userdata.goalList_input

		return 'system_checked'


class turn2goal(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['turn_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state turn to goal')
		
		rotate_degree = rospy.get_param("/val2sim_gui_ces_node/rotate_degree", 0)
		rospy.loginfo('Robot have been turning to {} by {} degree'.format(userdata.goalList_input[0], rotate_degree))

		turn2goal_node = roslaunch.core.Node(package='val2sim_sensor', 
											node_type='sim_rotateBy_odom.py', 
											name='sim_rotateBy_odom_node',
											output="screen")
		turn2goal_node.args = "_rotate_target:=%d" %(rotate_degree)
		turn2goal_launch = roslaunch.scriptapi.ROSLaunch()
		turn2goal_launch.start()
		turn2goal_process = turn2goal_launch.launch(turn2goal_node)
		while turn2goal_process.is_alive():
			if turn2goal_process.is_alive() == False:
				break
		turn2goal_process.stop()		
		rospy.loginfo('Robot turn to goal success !!!!!!!!!!!!!')

		userdata.goalList_output = userdata.goalList_input

		return 'turn_success'

		
class move_forward(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['move_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])
		self.translate_distance = 0

	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state move forward')
		translate_meter = rospy.get_param("/val2sim_gui_ces_node/translate_meter", 300)
		rospy.loginfo('Robot have been moving to {} by {} meter'.format(userdata.goalList_input[0], translate_meter))

		sim_open_obstacleDetection_node()

		move_node = roslaunch.core.Node(package='val2sim_sensor', 
										node_type='sim_translateBy_odom.py', 
										name='sim_translateBy_odom_node',
										output="screen")
		move_node.args = "_translate_target:=%d" %(abs(translate_meter))
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

		sim_read_action(userdata.goalList_input[0], None, "play_sound")

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
		rospy.loginfo('Robot turn around {}'.format(userdata.goalList_input[0]))

		if userdata.goalList_input[0] == "station1":
			self.rotate_angle = 180
		elif userdata.goalList_input[0] == "base_station":
			self.rotate_angle = -180
		
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


class charging_alignment(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['align_success'])


	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state align to charge station')
		
		rotate_degree = rospy.get_param("/val2sim_gui_ces_node/rotate_degree", 0)
		rospy.loginfo('Robot have been turning to charging station by {} degree'.format(-rotate_degree))

		charge_node = roslaunch.core.Node(package='val2sim_sensor', 
											node_type='sim_rotateBy_odom.py', 
											name='sim_rotateBy_odom_node',
											output="screen")
		charge_node.args = "_rotate_target:=%d" %(-rotate_degree)
		charge_launch = roslaunch.scriptapi.ROSLaunch()
		charge_launch.start()
		charge_process = charge_launch.launch(charge_node)
		while charge_process.is_alive():
			if charge_process.is_alive() == False:
				break
		charge_process.stop()		
		rospy.loginfo('Robot align to chargie station success !!!!!!!!!!!!!')

		return 'align_success'
		

def main():
	sm_nav = smach.StateMachine(outcomes=['shutdown'])
	sm_nav.userdata.goal_list = ["station1", "base_station"]
	with sm_nav:
		smach.StateMachine.add('SYSTEM_AVAILABILITY', systemAvailability(),
								transitions={'system_checked':'TURN2GOAL'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('TURN2GOAL', turn2goal(),
								transitions={'turn_success':'MOVE_FORWARD'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('MOVE_FORWARD', move_forward(),
								transitions={'move_success':'SM_ACTION'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('CHARGING_ALIGNMENT', charging_alignment(),
								transitions={'align_success':'shutdown'})

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
							   transitions={'turn_finished':'MOVE_FORWARD','all_finished':'CHARGING_ALIGNMENT'})


	sis = smach_ros.IntrospectionServer('server_name', sm_nav, 'SM_NAV/SM_ACT')
	sis.start()
	outcome = sm_nav.execute()
	# rospy.spin()
	sis.stop()


if __name__ == '__main__':
	rospy.init_node('val2sim_fsm_ces_node', anonymous=False)
	main()