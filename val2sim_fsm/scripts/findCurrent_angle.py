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

# Compute turning angle to reduce rotation error
def compute_turningDegree(degree1, degree2): 
    turning_degree = degree2 - degree1
    return turning_degree

# Convert angle from quaternion to euler
def quar2euler(orientation_matrix):
    orientation_x = orientation_matrix[0]
    orientation_y = orientation_matrix[1]
    orientation_z = orientation_matrix[2]
    orientation_w = orientation_matrix[3]
    orientation_list = [orientation_x, orientation_y, orientation_z, orientation_w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return yaw

# Main function
def find_currentAngle(target_frame):
    # rospy.init_node('findDiff_angle_node')
    listener = tf.TransformListener()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans1,rot1) = listener.lookupTransform('map', "base_footprint" , rospy.Time(0))
            (trans2,rot2) = listener.lookupTransform('map', target_frame , rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        break
    yaw1 = quar2euler(rot1)
    yaw2 = quar2euler(rot2)
    # print("Yaw from map relative to base footprint:{}".format(yaw1))
    # print("Yaw from map relative to station2:{}".format(yaw2))
    degree1 = rad2deg(yaw1)
    degree2 = rad2deg(yaw2)
    # print("Degree from map relative to base footprint:{}".format(degree1))
    # print("Degree from map relative to station2:{}".format(degree2))
    turning_degree = compute_turningDegree(degree1, degree2) 
    
    return turning_degree

# turning_degree = find_diffAngle("charging_station")
# print(turning_degree)