# ROS AND GAZEBO INSTALLATION GUIDE

The following instructions helps to configure ROS KINETIC and GAZEBO 7 for launching your turtlebot simulation on Ubuntu 16

This installation guide assumes that Python2 is already installed

## Instructions to Install ROS KINETIC

1. Setup your computer to accept software from packages.ros.org

`$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

2. Set up your keys

`$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`

3. Install ROS Desktop-Full variant - ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception

`$ sudo apt-get update`

`$ sudo apt-get install ros-kinetic-desktop-full`

4. To find the available ROS packages

`$ apt-cache search ros-kinetic`

5. Initialize ROS Dependency

`$ sudo rosdep init`

`$ rosdep update`

6. ROS Environmental Variables

`$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc`

`$ source ~/.bashrc`

7. Install Dependencies for building packages

`$ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential`


## Instructions to Install Gazebo 7

1. Setup your computer to accept software from packages.osrfoundation.org

`$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'`


2. Set up your keys

`$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -`

3. Install Gazebo 7

`$ sudo apt-get update`

`$ sudo apt-get install gazebo7`

4. Check your Installation

`$ gazebo`
