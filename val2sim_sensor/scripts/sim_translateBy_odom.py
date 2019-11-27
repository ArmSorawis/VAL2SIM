#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import sys
 
class send_cmdvel():

    def __init__(self):
        self.pub = rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.command =Twist()
        self.target = rospy.get_param("~translate_target", 100) #cm
        self.diff = 0
        self.filter_pose = 0
        self.kp = 0.1
        self.setPose = True

        self.twist_robot =Twist()
        self.twist_robot.linear.x = 0
        self.twist_robot.linear.y = 0
        self.twist_robot.linear.z = 0
        self.twist_robot.angular.x = 0
        self.twist_robot.angular.y = 0
        self.twist_robot.angular.z = 0

    def selectState(self):
        if self.target > 0:
            self.state = "move_forward"
        elif self.target < 0:
            self.state = "move_backward"
        elif self.target == 0:
            self.state = "stop"

    def rotateRobot(self, state, robot_poseX):

        if state == "move_forward":
            if self.setPose == True:
			    self.first_pose = robot_poseX
			    self.setPose = False

            self.filter_pose = robot_poseX - self.first_pose

    
            self.diff = self.target - self.filter_pose
            velocity = self.kp * self.diff
            if velocity >= 0.3:
                velocity = 0.3
            self.twist_robot.linear.x = velocity
            self.pub.publish(self.twist_robot)
            if abs(self.diff) < 1:
                state = "stop"

            print("\n")
            print("Robot state={}".format(state))
            print("Robot velocity={}".format(velocity))
            print("target={} current={}".format(self.target, self.filter_pose))

        if state == "move_backward":
            if self.setPose == True:
			    self.first_pose = robot_poseX
			    self.setPose = False

            self.filter_pose = robot_poseX - self.first_pose

    
            self.diff = self.target - self.filter_pose
            velocity = self.kp * (self.diff)
            if velocity <= -0.3:
                velocity = -0.3
            self.twist_robot.linear.x = velocity
            self.pub.publish(self.twist_robot)
            if abs(self.diff) < 1:
                state = "stop"

            print("\n")
            print("Robot state={}".format(state))
            print("Robot velocity={}".format(velocity))
            print("target={} current={}".format(self.target, -self.filter_pose))
        
        if state == "stop":
            self.twist_robot.linear.x = 0
            self.pub.publish(self.twist_robot)
            rospy.signal_shutdown('Quit')
          
           
class odom_subscriber(object):

    def __init__(self):
        self.pose_x_centimeter = 0.0

    def get_position(self, msg):
        position_q = msg.pose.pose.position
        pose_x = position_q.x
        self.pose_x_centimeter = pose_x * 100

    def listener(self):
        rospy.Subscriber ('/val2/base_controller/odom', Odometry, self.get_position) #For Simulation


if __name__=="__main__":
    rospy.init_node('sim_translateBy_odom_node', disable_signals=True)

    odomSub_node = odom_subscriber()
    odomSub_node.listener()

    sendCmdvel_node = send_cmdvel()
    sendCmdvel_node.selectState()
    state = sendCmdvel_node.state
    
    r = rospy.Rate(10)
    rospy.sleep(1)
    try:
        while(not rospy.is_shutdown()):
            sendCmdvel_node.rotateRobot(state, odomSub_node.pose_x_centimeter)
            r.sleep()
    except Exception as e:
		print(e)
