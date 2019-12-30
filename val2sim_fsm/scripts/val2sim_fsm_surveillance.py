#!/usr/bin/env python

# Import necessary package 
import rospy
import smach
import smach_ros
import roslaunch
from sim_read_goalFile_surveillance import sim_read_goal_surveillance
from sim_read_actionFile_surveillance import sim_read_action_surveillance
from clear_costMap import clear_costmaps
from sim_switch_obstacleAvoidance_surveillance import sim_open_obstacleAvoidance, sim_close_obstacleAvoidance
import random

# Storage goal data from /val2_navigation/text/goal_surveillance.txt
goal_data = sim_read_goal_surveillance() 

# Class for checking robot system
class systemAvailability(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['system_checked'],
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

		# Random sequence of goal in this bracket 
		self.goalList = ["station1", "station2", "station3", "station4"]
		random.shuffle(self.goalList)
		self.goalList.append("base_station")

		# Output equal to input
		userdata.goalList_output = self.goalList
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
		sim_read_action_surveillance(userdata.goalList_input[-1], userdata.goalList_input[0], "turn2nextgoal")
		rospy.loginfo('Robot have been turning to {} by {} degree'.format(userdata.goalList_input[0],
																		  sim_read_action_surveillance.rotate2nextStation))
		turn2goal_node = roslaunch.core.Node(package='val2sim_sensor', 
											node_type='sim_rotateBy_odom.py', 
											name='sim_rotateBy_odom_node',
											output="screen")
		turn2goal_node.args = "_rotate_target:=%d" %(sim_read_action_surveillance.rotate2nextStation)
		turn2goal_launch = roslaunch.scriptapi.ROSLaunch()
		turn2goal_launch.start()
		turn2goal_process = turn2goal_launch.launch(turn2goal_node)
		while turn2goal_process.is_alive():
			if turn2goal_process.is_alive() == False:
				break
		turn2goal_process.stop()		
		rospy.loginfo('Robot turn to goal success !!!!!!!!!!!!!')

		# Output equal to input
		userdata.goalList_output = userdata.goalList_input
		return 'turn_success'

# Class for moving the robot to the next goal
class move2goal(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['move2goal_success','all_success'],
							input_keys=['goalList_input'],
							output_keys=['goalList_output'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		self.new_goalList = []
		rospy.loginfo('Executing state move to goal')
		rospy.loginfo('Robot have been moving to {}'.format(userdata.goalList_input[0]))

		# Robot navigate to each goal and avoiding the obstacle
		sim_open_obstacleAvoidance()
		robot_goal = goal_data[userdata.goalList_input[0]]
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
		sim_close_obstacleAvoidance()
		rospy.loginfo('Robot move to goal success !!!!!!!!!!!!!')

		# Renew the goal
		self.new_goalList = [userdata.goalList_input[1], userdata.goalList_input[2], 
							 userdata.goalList_input[3], userdata.goalList_input[4], 
							 userdata.goalList_input[0]]
		userdata.goalList_output = self.new_goalList

		# Clear cost map
		clear_costmaps()

		if self.new_goalList[0] != "base_station":
			return 'move2goal_success'
		elif self.new_goalList[0] == "base_station":
			return 'all_success'

# Class for heading the robot to base station
class turn2base(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['turn2base_success'],
							input_keys=['goalList_input'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state turn to base')
		
		# Robot heading to base station
		sim_read_action_surveillance(userdata.goalList_input[-1], "base_station", "turn2nextgoal")
		rospy.loginfo('Robot have been turning to base station by {} degree'.format(sim_read_action_surveillance.rotate2nextStation))
		turn2goal_node = roslaunch.core.Node(package='val2sim_sensor', 
											node_type='sim_rotateBy_odom.py', 
											name='sim_rotateBy_odom_node',
											output="screen")
		turn2goal_node.args = "_rotate_target:=%d" %(sim_read_action_surveillance.rotate2nextStation)
		turn2goal_launch = roslaunch.scriptapi.ROSLaunch()
		turn2goal_launch.start()
		turn2goal_process = turn2goal_launch.launch(turn2goal_node)
		while turn2goal_process.is_alive():
			if turn2goal_process.is_alive() == False:
				break
		turn2goal_process.stop()
		rospy.loginfo('Robot turn to goal success !!!!!!!!!!!!!')

		return 'turn2base_success'

# Class for moving the robot to base station
class move2base(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self, 
							outcomes=['move2base_success'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state move to base')
		rospy.loginfo('Robot have been moving to base_station')

		# Robot navigating to base station and avoiding the obstacle
		sim_open_obstacleAvoidance()
		robot_goal = goal_data["base_station"]
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
		sim_close_obstacleAvoidance()

		# Clear cost map
		clear_costmaps()

		return 'move2base_success'

# Class for aligning the robot to charging station
class chargeAlignment(smach.State):

	# Initial state
	def __init__(self):
		smach.State.__init__(self,
							outcomes=['round_finished'])

	# Execution function
	def execute(self, userdata):
		print('\n')
		rospy.loginfo('Executing state charge alignment')

		# Robot align to charging station
		sim_read_action_surveillance("charging_station", None, "turn2currentgoal")
		rospy.loginfo("Robot start align with charging station by {} degree".format(sim_read_action_surveillance.rotate2currentStation))	
		chargeAlignment_node = roslaunch.core.Node(package='val2sim_sensor', 
													 node_type='sim_rotateBy_odom.py', 
													 name='sim_rotateBy_odom_node')
		chargeAlignment_node.args = "_rotate_target:=%d" %sim_read_action_surveillance.rotate2currentStation
		chargeAlignment_launch = roslaunch.scriptapi.ROSLaunch()
		chargeAlignment_launch.start()
		chargeAlignment_process = chargeAlignment_launch.launch(chargeAlignment_node)
		while chargeAlignment_process.is_alive():
			if chargeAlignment_process.is_alive() == False:
				break
		chargeAlignment_process.stop()

		return "round_finished"

# Finite state machine main loop
def main():
	sm_nav = smach.StateMachine(outcomes=['shutdown'])
	with sm_nav:
		smach.StateMachine.add('SYSTEM_AVAILABILITY', systemAvailability(),
								transitions={'system_checked':'TURN2GOAL'},
								remapping={'goalList_output':'goal_list'})

		smach.StateMachine.add('TURN2GOAL', turn2goal(),
								transitions={'turn_success':'MOVE2GOAL'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('MOVE2GOAL', move2goal(),
								transitions={'move2goal_success':'TURN2GOAL',  'all_success':'TURN2BASE'},
								remapping={ 'goalList_input':'goal_list',
											'goalList_output':'goal_list'})

		smach.StateMachine.add('TURN2BASE', turn2base(),
								transitions={'turn2base_success':'MOVE2BASE'},
								remapping={ 'goalList_input':'goal_list'})

		smach.StateMachine.add('MOVE2BASE', move2base(),
								transitions={'move2base_success':'CHARGE_ALIGNMENT'})

		smach.StateMachine.add('CHARGE_ALIGNMENT', chargeAlignment(),
								transitions={'round_finished':'SYSTEM_AVAILABILITY'})


	sis = smach_ros.IntrospectionServer('server_name', sm_nav, 'SM_NAV')
	sis.start()
	outcome = sm_nav.execute()
	# rospy.spin()
	sis.stop()


if __name__ == '__main__':
	# Define node name
	rospy.init_node('val2sim_fsm_surveillance_node', anonymous=False)
	main()
	