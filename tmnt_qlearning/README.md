# README

# COLLABORATIVE FENCELESS ROBOTICS

Implemented Reinforcement Machine Learning technique (Q-Learning) on TurtleBot 3-Burger
to avoid bumping into static obstacles.

## Instructions to SetUp the workspace
1.  $ source /opt/ros/kinetic/setup.bash

2. Create a directory to prep the workspace
mkdir -p ~/projectcs/src

3. Traverse to projectcs folder to catkin make
catkin_make

4. Go to src and and clone dependencies
cd src
git clone https://bitbucket.org/theconstructcore/openai_ros.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/EricssonResearch/tnmt.git
cd ..
catkin_make
source devel/setup.bash

## Instructions to run Q-Learning training
1. Run the environment
roslaunch turtlebot3_gazebo turtlebot3_world.launch

2. And in another terminal run:
roslaunch tmnt_qlearning start_training_maze.launch
