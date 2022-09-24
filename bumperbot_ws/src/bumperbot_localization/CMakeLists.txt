cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)