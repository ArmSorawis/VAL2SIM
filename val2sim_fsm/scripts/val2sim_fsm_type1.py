#!/usr/bin/env python

# Import necessary package 
import rospy
import smach
import smach_ros
import roslaunch
from sim_read_goalFile import sim_read_goal
from sim_read_actionFile import sim_read_action
from clear_costMap import clear_costmaps
from sim_switch_obstacleDetection_type1 import sim_open_obstacleDetection_node, sim_close_obstacleDetection_node


# Storage goal data from /val2_navigation/text/goal_solustar.txt
goal_data = sim_read_goal() 

# Class for checking robot system
class systemAvailability(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['system_checked'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state system availability')
		
		# Robot rotate around itself
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
		rospy.loginfo('Robot ready to use')

		# Robot say "i'm ready to use"
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
		
		# Output equal to input
		userdata.goalList_output = userdata.goalList_input
		return 'system_checked'

# Class for heading the robot to the next goal
class turn2goal(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['turn_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])
		
	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state turn to goal')
		
		# Robot heading to the next goal 
		sim_read_action(None, userdata.goalList_input[0], "turn2nextgoal")
		rospy.loginfo('Robot have been turning to {} by {} degree'.format(userdata.goalList_input[0],sim_read_action.rotate2nextStation))
		turn2goal_node = roslaunch.core.Node(package='val2sim_sensor', 
											node_type='sim_rotateBy_odom.py', 
											name='sim_rotateBy_odom_node',
											output="screen")
		turn2goal_node.args = "_rotate_target:=%d" %(sim_read_action.rotate2nextStation)
		turn2goal_launch = roslaunch.scriptapi.ROSLaunch()
		turn2goal_launch.start()
		turn2goal_process = turn2goal_launch.launch(turn2goal_node)
		while turn2goal_process.is_alive():
			if turn2goal_process.is_alive() == False:
				break
		turn2goal_process.stop()

		# Output equal to input
		userdata.goalList_output = userdata.goalList_input
		return 'turn_success'
		
# Class for moving the robot to the next goal
class move2goal(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['move_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state move to goal')
		rospy.loginfo('Robot have been moving to {}'.format(userdata.goalList_input[0]))

		# Robot moving to the next goal and stop when detected the obstacle
		robot_goal = goal_data[userdata.goalList_input[0]]
		sim_open_obstacleDetection_node()

		nav_node = roslaunch.core.Node(package='val2sim_navigation', 
									   node_type='send_goal.py', 
									   name='movebase_client_py')
		nav_node.args = """ _position_x:={}
							_position_y:={}
							_position_z:={}
							_orientation_x:={} 
							_orientation_y:={} 
							_orientation_z:={}  
							_orientation_w:={} """\
							.format(robot_goal[0],
									robot_goal[1],
									robot_goal[2],
									robot_goal[3],
									robot_goal[4],
									robot_goal[5],
									robot_goal[6])
		nav_launch = roslaunch.scriptapi.ROSLaunch()
		nav_launch.start()
		nav_process = nav_launch.launch(nav_node)
		while nav_process.is_alive():
			if nav_process.is_alive == False:
				break
		nav_process.stop()
		sim_close_obstacleDetection_node()


		# Clear cost map
		clear_costmaps()

		# Output equal to input
		userdata.goalList_output = userdata.goalList_input
		return 'move_success'

# Class for verify that robot reached to goal or not
class reach2goal(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['reach_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state reach to goal')
		rospy.loginfo('Robot reach to {}'.format(userdata.goalList_input[0]))

		# Robot say "station 1", "station 2", "base station" or "i need to charge my battery"
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

		# Output equal to input
		userdata.goalList_output = userdata.goalList_input
		return 'reach_success'

# Class for aligning the robot to each station
class goalAlignment(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self,
							outcomes=['align_success','all_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state goal alignment')
		if userdata.goalList_input[0] == "base_station" and len(userdata.goalList_input) == 1:
			userdata.goalList_input[0] = "charging_station"
		sim_read_action(userdata.goalList_input[0], None, "turn2currentgoal")

		# Robot start aligning to current goal
		rospy.loginfo("Robot start align with {} by {} degree".format(userdata.goalList_input[0],sim_read_action.rotate2currentStation))
		goalAlignment_node = roslaunch.core.Node(package='val2sim_sensor', 
													 node_type='sim_rotateBy_odom.py', 
													 name='sim_rotateBy_odom_node',
													 output="screen")
		goalAlignment_node.args = "_rotate_target:=%d" %sim_read_action.rotate2currentStation
		goalAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		goalAlignment_launch.start()
		goalAlignment_process = goalAlignment_launch.launch(goalAlignment_node)
		while goalAlignment_process.is_alive():
			if goalAlignment_process.is_alive() == False:
				break
		goalAlignment_process.stop()

		# Delete reached goal
		del userdata.goalList_input[0]
		
		# Output equal to input
		userdata.goalList_output = userdata.goalList_input
		if len(userdata.goalList_input) > 0:
			return 'align_success'
		elif len(userdata.goalList_input) == 0:
			return 'all_success'

# Class for waiting user to push the button again
class wait4nextround(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['finished_process'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state wait for user')
		return 'finished_process'
		
# Main function
def main():
	# Get list of goal from gui
	waypoint = []
	goal_1 = rospy.get_param("~goal1", "station1")
	goal_2 = rospy.get_param("~goal2", "station2")
	goal_3 = rospy.get_param("~goal3", "base_station")
	goal_4 = rospy.get_param("~goal4", "None")
	goal_5 = rospy.get_param("~goal5", "None")
	goal_list = [goal_1, goal_2, goal_3, goal_4, goal_5]
	print(goal_list)
	for index in range(len(goal_list)):
		if goal_list[index] != "None":
			waypoint.append(goal_list[index])
	
	sm_nav = smach.StateMachine(outcomes=['shutdown'])
	sm_nav.userdata.goal_list = waypoint
	with sm_nav:
		smach.StateMachine.add('SYSTEM_AVAILABILITY', systemAvailability(),
								transitions={'system_checked':'TURN2GOAL'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('TURN2GOAL', turn2goal(),
								transitions={'turn_success':'MOVE2GOAL'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('MOVE2GOAL', move2goal(),
								transitions={'move_success':'SM_ACTION'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('WAIT4NEXTROUND', wait4nextround(),
								transitions={'finished_process':'shutdown'})


		sm_act = smach.StateMachine(outcomes=['align_finished', 'all_finished'])
		sm_act.userdata.goal_list = waypoint
		with sm_act:
			smach.StateMachine.add('REACH2GOAL', reach2goal(), 
									transitions={'reach_success':'GOAL_ALIGNMENT'},
									remapping={ 'goalList_input':'goal_list',
												'goalList_output':'goal_list'})

			smach.StateMachine.add('GOAL_ALIGNMENT', goalAlignment(),
									transitions={'align_success':'align_finished', 'all_success':'all_finished'},
									remapping={ 'goalList_input':'goal_list',
												'goalList_output':'goal_list'}) 

		smach.StateMachine.add('SM_ACTION', sm_act,
							   transitions={'align_finished':'TURN2GOAL','all_finished':'WAIT4NEXTROUND'})


	sis = smach_ros.IntrospectionServer('server_name', sm_nav, 'SM_NAV/SM_ACT')
	sis.start()
	outcome = sm_nav.execute()
	# rospy.spin()
	sis.stop()


if __name__ == '__main__':
	# Define node name
	rospy.init_node('val2sim_fsm_type1_node', anonymous=False)
	main()
	