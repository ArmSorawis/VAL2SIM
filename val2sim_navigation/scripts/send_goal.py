#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import os


class move2goal:
    def __init__(self):
        rospy.init_node('movebase_client_py', disable_signals=True)
        
        pose_x = rospy.get_param("~position_x", 0.0)
        pose_y = rospy.get_param("~position_y", 0.0)
        pose_z = rospy.get_param("~position_z", 0.0)
        orien_x = rospy.get_param("~orientation_x", 0.0)
        orien_y = rospy.get_param("~orientation_y", 0.0)
        orien_z = rospy.get_param("~orientation_z", 0.0)
        orien_w = rospy.get_param("~orientation_w", 0.0)

        self.goal = [pose_x, pose_y, pose_z, orien_x, orien_y, orien_z, orien_w]

        self.movebase_client()

    def movebase_client(self):

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.goal[0]
        goal.target_pose.pose.position.y = self.goal[1]
        goal.target_pose.pose.position.z = self.goal[2]
        
        goal.target_pose.pose.orientation.x = self.goal[3]
        goal.target_pose.pose.orientation.y = self.goal[4]
        goal.target_pose.pose.orientation.z = self.goal[5]
        goal.target_pose.pose.orientation.w = self.goal[6]
        
        client.send_goal(goal)

        wait = client.wait_for_result()
        rospy.loginfo(wait)

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()   


if __name__ == '__main__':
    try:
        result = move2goal()
        if result:
            rospy.loginfo("Goal execution done!")
            rospy.signal_shutdown("Navigate Finished!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
        rospy.signal_shutdown("Navigate Finished!")
