#!/usr/bin/env python

# Important Library

from geometry_msgs.msg import Twist
import rospy


class filter_keyvel(object):

    def __init__(self):
        self.data = None
        self.keyvel_filtered_publisher = rospy.Publisher('keyvel_filtered', Twist, queue_size = 10)
        self.keyvel_listener()

    def callback(self,data):
        self.data = data.data
        self.keyvel_filtered_publisher.publish(self.data)

    def keyvel_listener(self):
        rospy.Subscriber('key_vel', Twist, self.callback)
        rospy.spin()


if __name__=="__main__":
    rospy.init_node('val2sim_filter_keyvel_node')
    ultrasonic()