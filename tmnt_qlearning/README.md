# README

# COLLABORATIVE FENCELESS ROBOTICS

Implemented Reinforcement Machine Learning technique (Q-Learning) on TurtleBot 3-Burger
to avoid bumping into static obstacles.

## Instructions to set up the workspace

This installation guide assumes that ROS Kinetic and Gazebo is already setup

1.  `$ source /opt/ros/kinetic/setup.bash`

2. Create a directory to prep the workspace

`$ mkdir -p ~/projectcs/src`

3. Go to projectcs folder to catkin make

`$ catkin_make`

4. Go to src and and clone dependencies

`$ cd src`

`$ git clone https://bitbucket.org/theconstructcore/openai_ros.git`

`$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git`

`$ git clone https://github.com/EricssonResearch/tnmt.git`

`$ pip install pathlib`

`$ cd ..`

`$ catkin_make`

`$ source devel/setup.bash`

5. Change laser scan topic

Open file openai_ros/src/openai_ros/robot_envs.turtlebot2_env.py

Change all instances of `/kobuki/laser/scan'` to `/scan`

## Instructions to run Q-Learning training
1. Run the environment

`$ roslaunch turtlebot3_gazebo turtlebot3_world.launch`

2. And in another terminal run:

`$ roslaunch tmnt_qlearning start_training_maze.launch`
