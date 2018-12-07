#!/bin/bash

service ssh start

source /opt/ros/kinetic/setup.bash
alias goros='source devel/setup.sh'
export ROS_HOSTNAME=localhost
export TURTLEBOT_3D_SENSOR=astra
export TURTLEBOT_3D_SENSOR2=sr300
export TURTLEBOT_BATTERY=None
export TURTLEBOT_STACKS=interbotix
export TURTLEBOT_ARM=pincher
source /root/turtlebot2i/devel/setup.bash

Xvfb -shmem -screen 0 1280x1024x24 &
DISPLAY=:0  roslaunch turtlebot2i_gazebo turtlebot_world.launch gui:=false
