#!/usr/bin/env python

import roslaunch
import rospy
from os.path import expanduser

home = expanduser("~")

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["{}/val2sim_ws/src/val2sim_description/launch/val2sim_description.launch".format(home)])
launch.start()
rospy.loginfo("started")

try:
  launch.spin()
finally:
  launch.shutdown()