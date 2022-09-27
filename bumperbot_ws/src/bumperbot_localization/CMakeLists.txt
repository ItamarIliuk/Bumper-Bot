cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    nodes/simulate_motion_noise_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)