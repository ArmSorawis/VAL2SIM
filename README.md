# VAL2SIM

1. nano ~/.bashrc
Put in .bashrc file
-  export GAZEBO_MODEL_PATH=$HOME/val2sim_ws/src/val2sim_gazebo/models:$GAZEBO_MODEL_PATH$
-  export GAZEBO_RESOURCE_PATH=$HOME/val2sim_ws/src/val2sim_gazebo/worlds:$GAZEBO_RESOURCE_PATH$
-  source ~/val2sim_ws/devel/setup.bash

-------------------------------------------------------------

2. Install necessary package 

git clone https://github.com/ros-teleop/twist_mux.git
git clone https://github.com/ros-drivers/joystick_drivers.git

sudo apt-get install ros-melodic-gmapping
sudo apt-get install ros-melodic-map-server 
sudo apt-get install ros-melodic-amcl
sudo apt-get install ros-melodic-move-base
sudo apt-get install ros-melodic-sound-play
sudo apt-get install ros-melodic-rosbridge-server
sudo apt-get install ros-melodic-xacro
sudo apt-get install ros-melodic-joint-state-publisher
sudo apt-get install ros-melodic-smach
sudo apt-get install ros-melodic-ddynamic-reconfigure
sudo apt-get install ros-melodic-rgbd-launch

sudo apt-get install libusb-dev
sudo apt-get install libspnav-dev
sudo apt-get install libbluetooth-dev
sudo apt install libcwiid-dev

-------------------------------------------------------------

cd /val2sim_ws/src/val2sim_teleop/twist_mux/config
gedit twist_mux_topics.yaml

replace below message in to specific file
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

cd /val2sim_ws/src/val2sim_teleop/twist_mux/launch
gedit twist_mux.launch
commend out joystick_relay node


cd val2sim_ws/
catkin_make
catkin_make install
