#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import sys
 
class send_cmdvel():

    def __init__(self):
        self.pub = rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.command =Twist()
        self.target = rospy.get_param("~rotate_target", 180)
        self.diff = 0
        self.filter_degree = 0
        self.kp = 0.1
        self.robot_velocity = 0
        self.setDegree = True
        self.offset = 17

        self.rotateAround_start = True
        self.round = 1

        self.twist_robot =Twist()
        self.twist_robot.linear.x = 0
        self.twist_robot.linear.y = 0
        self.twist_robot.linear.z = 0
        self.twist_robot.angular.x = 0
        self.twist_robot.angular.y = 0
        self.twist_robot.angular.z = 0

    def selectState(self):
        if self.target > 0 and self.target <= 180:
            self.state = "rotate_left"
        elif self.target < 0 and self.target >= -180:
            self.state = "rotate_right"
        elif self.target > 180 and self.target < 360:
            self.target = self.target - 360
            self.state = "rotate_right"
        elif self.target < -180 and self.target > -360:
            self.target = self.target + 360
            self.state = "rotate_left"
        elif self.target == 360:
            self.state = "rotate_around_left"
        elif self.target == -360:
            self.state = "rotate_around_right"
        else:
            self.state = "stop"

    def rotateRobot(self, state, degree):
        if state == "rotate_left":
            if self.setDegree == True:
			    self.first_degree = degree
			    self.setDegree = False
            if degree > 0 or degree == 0 or degree == 180:
                self.filter_degree = degree - self.first_degree
            elif degree < 0:
                if self.first_degree > 0:
                    filter_degree = -degree + self.first_degree - 360
                    self.filter_degree = -filter_degree
                elif self.first_degree < 0:
                    filter_degree = -degree + self.first_degree
                    self.filter_degree = -filter_degree
    
            self.diff = self.target - self.filter_degree
            velocity = self.kp * (self.diff)
            if velocity >= 0.3:
                velocity = 0.3
            self.twist_robot.angular.z = velocity
            self.pub.publish(self.twist_robot)
            if abs(self.diff) < 1:
                state = "stop"

            print("\n")
            print("Robot state={}".format(state))
            print("Robot velocity={}".format(velocity))
            print("target={} current={}".format(self.target, self.filter_degree))

        elif state == "rotate_right":
            if self.setDegree == True:
			    self.first_degree = degree
			    self.setDegree = False
            if degree > 0 or degree == 0 or degree == 180:
                if self.first_degree < 0:
                    filter_degree = degree - self.first_degree - 360
                    self.filter_degree = -filter_degree
                elif self.first_degree > 0:
                    filter_degree = degree - self.first_degree
                    self.filter_degree = -filter_degree
            elif degree < 0:
                self.filter_degree = -degree + self.first_degree
    
            self.diff = self.target + self.filter_degree
            velocity = self.kp * (self.diff)
            if velocity <= -0.3:
                velocity = -0.3
            self.twist_robot.angular.z = velocity
            self.pub.publish(self.twist_robot)
            if abs(self.diff) < 1:
                state = "stop"
            
            print("\n")
            print("Robot state={}".format(state))
            print("Robot velocity={}".format(velocity))
            print("target={} current={}".format(self.target, -self.filter_degree))

        elif state == "rotate_around_left":
            if self.setDegree == True:
			    self.first_degree = degree
			    self.setDegree = False
            if degree > 0 or degree == 0 or degree == 180:
                if self.first_degree >= 0:
                    if self.round == 1:
                        self.filter_degree = degree - self.first_degree
                    elif self.round == 2:
                        self.filter_degree = degree - self.first_degree + 360
                elif self.first_degree < 0:
                    self.filter_degree = degree - self.first_degree
                    self.round = 2
                    
            elif degree < 0:
                if self.first_degree >= 0:
                    filter_degree = -degree + self.first_degree - 360
                    self.filter_degree = -filter_degree
                    self.round = 2
                elif self.first_degree < 0:
                    if self.round == 1:
                        filter_degree = -degree + self.first_degree
                        self.filter_degree = -filter_degree
                    elif self.round ==2:
                        filter_degree = -degree + self.first_degree
                        self.filter_degree = -filter_degree + 360

            self.diff = self.target - self.filter_degree
            velocity = self.kp * (self.diff)
            if velocity >= 0.3:
                velocity = 0.3
            self.twist_robot.angular.z = velocity
            self.pub.publish(self.twist_robot)
            if abs(self.diff) < 1:
                state = "stop"

            print("\n")
            print("Robot state={}".format(state))
            print("Robot velocity={}".format(velocity))
            print("target={} current={}".format(360, self.filter_degree))
            
        elif state == "rotate_around_right":
            if self.setDegree == True:
			    self.first_degree = degree
			    self.setDegree = False
            if degree > 0 or degree == 0 or degree == 180:
                if self.first_degree < 0:
                    filter_degree = degree - self.first_degree - 360
                    self.filter_degree = -filter_degree
                    self.round = 2
                elif self.first_degree >= 0:
                    if self.round == 1:
                        filter_degree = degree - self.first_degree
                        self.filter_degree = -filter_degree
                    elif self.round ==2:
                        filter_degree = degree - self.first_degree
                        self.filter_degree = -filter_degree + 360
            elif degree < 0:
                if self.first_degree >= 0:
                    self.filter_degree = -degree + self.first_degree
                    self.round = 2
                elif self.first_degree < 0:
                    if self.round == 1:
                        self.filter_degree = -degree + self.first_degree
                    elif self.round == 2:
                        self.filter_degree = -degree + self.first_degree + 360
            
            self.diff = self.target + self.filter_degree
            velocity = self.kp * (self.diff)
            if velocity <= -0.3:
                velocity = -0.3
            self.twist_robot.angular.z = velocity
            self.pub.publish(self.twist_robot)
            if abs(self.diff) < 1:
                state = "stop"

            print("\n")
            print("Robot state={}".format(state))
            print("Robot velocity={}".format(velocity))
            print("target={} current={}".format(-360, -self.filter_degree))
        
        if state == "stop":
            self.twist_robot.angular.z = 0
            self.pub.publish(self.twist_robot)
            rospy.signal_shutdown('Quit')
            
class odom_subscriber(object):

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.degree = 0.0

    def get_rotation(self, msg):
        global roll, pitch, yaw, degree
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        self.degree = self.rad2deg(yaw)

    def rad2deg(self, radians):
        pi = math.pi
        degrees = (180 * radians) / pi
        return degrees

    def listener(self):
        rospy.Subscriber ('/val2/base_controller/odom', Odometry, self.get_rotation) #For Simulation


if __name__=="__main__":
    rospy.init_node('sim_rotateBy_odom_node', disable_signals=True)

    odomSub_node = odom_subscriber()
    odomSub_node.listener()

    sendCmdvel_node = send_cmdvel()
    sendCmdvel_node.selectState()
    state = sendCmdvel_node.state
    
    r = rospy.Rate(10)
    rospy.sleep(1)
    try:
        while(not rospy.is_shutdown()):
            odom_degree = odomSub_node.degree
            # print("Real Degree={}".format(odom_degree))
            sendCmdvel_node.rotateRobot(state, odom_degree)
            r.sleep()
    except Exception as e:
		print(e)
