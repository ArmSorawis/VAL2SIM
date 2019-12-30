#!/usr/bin/env python

# Import necessary package
import rospy
import tf
from tf.transformations import euler_from_quaternion
import math

# Convert radian to degree format 
def rad2deg(radians):
    pi = math.pi
    degrees = (180 * radians) / pi
    return degrees

# Compute heading degree
def compute_turningDegree(pose_x, pose_y): 
    turning_yaw = math.atan2(pose_y, pose_x)
    turning_degree = rad2deg(turning_yaw)
    return turning_degree

# Main function
def find_nextAngle(target_frame):
    # rospy.init_node('findDiff_angle_node')
    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('base_footprint', target_frame , rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        break
    turning_degree = compute_turningDegree(trans[0], trans[1])
    
    return turning_degree

# turning_degree = find_diffAngle("charging_station")
# print(turning_degree)