#!/usr/bin/env python

# Import necessary package
import roslaunch
import os

# Open obstacle detection system (robot stop when detected the obstacle infront of it then try to create a new path to avoiding the obstacle)
def sim_open_obstacleDetection_node():
	global obstacle_process
	obstacle_node = roslaunch.core.Node(package='val2sim_sensor', 
										node_type='sim_obstacleDetection_type2.py', 
										name='sim_obstacleDetection_type2_node')
	obstacle_launch = roslaunch.scriptapi.ROSLaunch()
	obstacle_launch.start()
	obstacle_process = obstacle_launch.launch(obstacle_node)

# Close obstacle detection system (robot continue moving in the new path)
def sim_close_obstacleDetection_node():
	nodes = os.popen("rosnode list").read().splitlines()
	interest_node = '/sim_obstacleDetection_type2_node'
	if interest_node in nodes:
		os.system("rosnode kill {}".format(interest_node))
