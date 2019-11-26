#!/usr/bin/env python

# Important Library

from geometry_msgs.msg import Twist
import rospy


class filter_navvel(object):

    def __init__(self):
        self.data = None
        self.navvel_filtered_publisher = rospy.Publisher('navvel_filtered', Twist, queue_size = 10)
        self.navvel_listener()

    def callback(self,data):
        self.data = data.data
        self.navvel_filtered_publisher.publish(self.data)

    def navvel_listener(self):
        rospy.Subscriber('nav_vel', Twist, self.callback)
        rospy.spin()


if __name__=="__main__":
    rospy.init_node('val2sim_filter_navvel_node')
    ultrasonic()