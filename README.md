# VAL2SIM

1. Put in .bashrc file
-  export GAZEBO_MODEL_PATH=$HOME/val2sim_ws/src/val2sim_gazebo/models:$GAZEBO_MODEL_PATH$
-  export GAZEBO_RESOURCE_PATH=$HOME/val2sim_ws/src/val2sim_gazebo/worlds:$GAZEBO_RESOURCE_PATH$
-  source ~/val2sim_ws/devel/setup.bash

2. Install necessary package 
git clone **link**
- twist_mux (github)
- joystick-driver (github)

sudo apt-get install ros-melodic-***
- gmapping (apt install)
- map-server (apt install)
- amcl (apt install)
- move-base (apt install)
- sound-play (apt install)
- rosbridge-server (apt install)
- xacro (apt install)
- joint-state-publisher (apt install)
- smach (apt install)
- ddynamic-reconfigure (apt install)
- rgbd-launch (apt install)

When get an error
- sudo apt-get install libusb-dev
- sudo apt-get install libspnav-dev
- sudo apt-get install libbluetooth-dev
- sudo apt install libcwiid-dev


go to /val2sim_ws/src/val2sim_teleop/twist_mux/config
# gedit twist_mux_topics.yaml
replace this message in to specific file
topics:
-
  name    : navigation
  topic   : nav_vel
  timeout : 0.5
  priority: 10
-
  name    : joystick
  topic   : joy_vel_filtered
  timeout : 0.5
  priority: 100
-
  name    : keyboard
  topic   : key_vel
  timeout : 0.5
  priority: 100
-
  name    : obstacle_detection
  topic   : break_vel
  timeout : 0.5
  priority: 75
-
  name    : depth_detection
  topic   : camera_vel
  timeout : 0.5
  priority: 75
-
  name    : touch_screen
  topic   : paused_vel
  timeout : 0.5
  priority: 80
-
  name    : pause_button
  topic   : pause_button_vel
  timeout : 0.5
  priority: 80

go to /val2sim_ws/src/val2sim_teleop/twist_mux/launch
# gedit twist_mux.launch
commend out joystick_relay node


# cd val2sim_ws/
# catkin_make
# catkin_make install
