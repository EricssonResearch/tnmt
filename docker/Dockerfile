# DockerFile stems from tutorial -> https://github.com/Interbotix/turtlebot2i/wiki/Full-Build-Instructions

FROM ros:kinetic-ros-core

ENV TERM=xterm

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release \
    udev \
    sudo \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN apt-get update && apt-get install -q -y \
    build-essential libgtk-3-dev ros-kinetic-desktop gazebo7=7.14.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup environment
EXPOSE 11345

#RUN apt-get update

# ROS Kinetic Dependencies & Install

#RUN apt-get install build-essential libgtk-3-dev ros-kinetic-desktop -yq \ 
    #&& rm -rf /var/lib/apt/lists/*

RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list

RUN rosdep init

# not needed as everything runs as root here
#RUN sudo rosdep fix-permissions

RUN rosdep update

#RUN apt-cache search ros-kinetic

# RealSense ROS Package install

RUN wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src

RUN bash ./enable_kernel_sources.sh

#WORKDIR /root

#RUN git clone https://github.com/IntelRealSense/librealsense.git

#WORKDIR /root/librealsense

#RUN mkdir build

#WORKDIR /root/librealsense/build

#RUN cmake ../

#RUN make &&  make install

#WORKDIR /root

#RUN cp config/99-realsense-libusb.rules /etc/udev/rules.d/

#RUN udevadm control --reload-rules && udevadm trigger

#RUN ./scripts/patch-realsense-ubuntu-xenial.sh

# TurtleBot 2i ROS Package install

RUN apt-get install libglfw3-dev ros-kinetic-librealsense \
    ros-kinetic-realsense-camera python-pip python-dev \
    vim xvfb ros-kinetic-turtlebot* \
    libudev-dev ros-kinetic-find-object-2d ros-kinetic-rtabmap-ros \ 
    ros-kinetic-moveit ros-kinetic-octomap-ros \ 
    ros-kinetic-manipulation-msgs ros-kinetic-controller-manager \
    python-wxgtk3.0 -yq  \ 
    && rm -rf /var/lib/apt/lists/*

#RUN apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control -yq \
#    && rm -rf /var/lib/apt/lists/*

# Setup TurtleBot2i source code and Catkin Environemnt

WORKDIR /root

RUN mkdir -p /root/turtlebot2i/src

WORKDIR /root/turtlebot2i/src

RUN git clone https://github.com/Interbotix/turtlebot2i.git .

RUN git clone https://github.com/Interbotix/arbotix_ros.git -b turtlebot2i

RUN git clone https://github.com/Interbotix/phantomx_pincher_arm.git

RUN git clone https://github.com/Interbotix/ros_astra_camera -b filterlibrary

RUN git clone https://github.com/Interbotix/ros_astra_launch

WORKDIR /root/turtlebot2i

COPY .bashrc /root/.bashrc

# Deep Q-Learning work

COPY turtlebot2i_deep_qlearning/gazebo_stuff/tmnt.world /root/turtlebot2i/src/turtlebot2i_gazebo/worlds
COPY turtlebot2i_deep_qlearning/gazebo_stuff/models /root/.gazebo/models
COPY turtlebot2i_deep_qlearning/gazebo_stuff/turtlebot_world.launch /root/turtlebot2i/src/turtlebot2i_gazebo/launch

COPY turtlebot2i_deep_qlearning/turtlebot2i_deep_qlearning/ /root/turtlebot2i/src/turtlebot2i_deep_qlearning/

RUN /bin/bash -c ". /opt/ros/kinetic/setup.bash; catkin_make"

RUN pip install -r /root/turtlebot2i/src/turtlebot2i_deep_qlearning/dqn/requirements.txt

COPY startup.sh /root/

WORKDIR /root

ENTRYPOINT ["/bin/bash"]
