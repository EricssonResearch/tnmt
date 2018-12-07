
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot2i_deep_qlearning)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)

catkin_install_python(PROGRAMS


  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch save_model
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


