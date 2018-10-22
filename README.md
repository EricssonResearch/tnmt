# COLLABORATIVE FENCELESS ROBOTICS

Implemented Reinforcement Machine Learning technique (Q-Learning) on TurtleBot 3-Burger
to avoid bumping into static obstacles and path finding.

## Instructions to set up the workspace

This installation guide assumes that ROS Kinetic and Gazebo is already setup. If not we refer to ros.md for instructions on how to set up the system (only tested on Ubuntu 16)

1.  `$ source /opt/ros/kinetic/setup.bash`

2. Create a directory to prep the workspace

`$ mkdir -p ~/projectcs/src`

3. Go to projectcs folder to catkin make

`$ cd ..`

`$ catkin_make`

4. Go to src and and clone dependencies

`$ cd src`

Clone this repository and source (this implements q-learning to navigate the robot and avoid obstacles):

`$ git clone https://github.com/EricssonResearch/tnmt.git`

And then clone and install dependencies:

`$ git clone https://bitbucket.org/theconstructcore/openai_ros.git`

`$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`

`$ pip install pathlib`

`$ sudo apt install ros-kinetic-pid`

`$ sudo apt install ros-kinetic-controller-manager-msgs`

`$ cd ..`

`$ catkin_make`

`$ source devel/setup.bash`

5. Change laser scan topic

Open file openai_ros/src/openai_ros/robot_envs.turtlebot2_env.py

Change all instances of `/kobuki/laser/scan` to `/scan`

## Instructions to run Q-Learning training
1. Run the environment for turtlebot3 burger

`$ export TURTLEBOT3_MODEL=burger`

`$ source ~/.bashrc`

`$ roslaunch turtlebot3_gazebo turtlebot3_world.launch`

2. And in another terminal run:

`$ roslaunch tmnt_qlearning start_training_maze.launch`
