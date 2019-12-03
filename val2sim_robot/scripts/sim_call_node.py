#!/usr/bin/env python

import rospy
import roslaunch

rotate_node = roslaunch.core.Node(  package='val2sim_sensor', 
                                            node_type='sim_rotateBy_odom.py', 
                                            name='sim_rotateBy_odom_node',
                                            output="screen")
rotate_node.args = "_rotate_target:=%d" %(360)
rotate_launch = roslaunch.scriptapi.ROSLaunch()
rotate_launch.start()
rotate_process = rotate_launch.launch(rotate_node)
while rotate_process.is_alive():
	if rotate_process.is_alive() == False:
		break
rotate_process.stop()