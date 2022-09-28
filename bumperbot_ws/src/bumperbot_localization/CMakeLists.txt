cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_localization)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    nav_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
    nodes/simulate_motion_noise_node.py
    nodes/kalman_filter_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(simulate_motion_noise src/simulate_motion_noise.cpp)
add_dependencies(simulate_motion_noise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulate_motion_noise ${catkin_LIBRARIES})

add_executable(simulate_motion_noise_node nodes/simulate_motion_noise_node.cpp)
add_dependencies(simulate_motion_noise_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulate_motion_noise_node ${catkin_LIBRARIES} simulate_motion_noise)